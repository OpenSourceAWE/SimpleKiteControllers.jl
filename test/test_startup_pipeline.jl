# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The startup of `examples/simple_opt_reelout.jl` end to end, in the order the script calls it:
`setup_run`, the startup solve, its corrected retries, the installed path, the gates and the state
the loop starts from (`src/run_setup.jl`, `src/startup_path.jl`). The plant is a stand-in that
`init_model` returns, with the two fields the setup reads; the optimizer is the fake server of
`fake_awetrim_server.jl`. Nothing of the user's is read or written: the menu state is a temporary
copy, the optimizer's caches are off, and rejected curves go to a temporary folder.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: GUI_STATE_FILE_OVERRIDE, FAILED_TRAJECTORY_DIR, merge_into!,
    solve_startup_path!, log_startup_reply, log_lobe_lift, adopt_startup_path!, finish_startup!,
    capture_startup_geometry!, startup_feasibility, init_phase5_and_controller!, init_loop_state!,
    KiteUtils
include(joinpath(@__DIR__, "fake_awetrim_server.jl"))

const PROJECT = "system_reelout_maasvlakte.yaml"
const UP_LOOPS = FC_Settings(fc_settings(project_file(PROJECT))).up_loops
# Two 100-point figures of eight [deg]: TIGHT misses a 1.3 gate at 150 m (margin 1.24), WIDE clears it.
function eight_path(a, b, c)
    r = range(0, 2pi; length = 101)[1:100]
    return prepare_path(a .* sin.(r), c .+ b .* sin.(2 .* r); resample = 100, up_loops = UP_LOOPS)
end
const TIGHT = eight_path(25.0, 6.0, 26.0)
const WIDE = eight_path(32.0, 8.0, 27.0)

# The plant as `setup_run` reads it: its step and its settled tether length (Float32, as V3Kite's).
stand_in_model(project, project_set, fcs, wpc, sim_time; turbulence, set_overrides) =
    (; dt = 1 / project_set.sample_freq, sys_state = (; l_tether = Float32[project_set.l_tether]))

"""
    fly_startup(; tos, wind, path_for, fail_warm) -> (; setup, st, err, paths, log, saved)

The script's startup against a fake server whose replies carry `path_for(power)`, the server's
power counting 1000 W per converged step; `fail_warm` answers every warm `/step` (the retries) with
a 422. `err` is what the startup threw, if anything; `saved` the curves it saved as rejected.
"""
function fly_startup(; tos = Dict{Symbol, Any}(), wind = "default", path_for = p -> TIGHT,
                     fail_warm = false)
    reply(l, p) = Dict("length" => l, "state" => "converged", "step_index" => 1,
        "trajectory" => Dict("azimuth" => path_for(p)[1], "elevation" => path_for(p)[2]),
        "depower" => Dict("mode" => "optimize", "value" => 1.42),
        "metrics" => Dict("energy_J" => 1e5, "total_time_s" => 100.0, "avg_power_W" => 18000.0,
                          "turn_radius_min_m" => 14.0))
    table(p) = Dict("table" => Dict("azimuth" => deg2rad.(path_for(p)[1]),
                                    "elevation" => deg2rad.(path_for(p)[2]),
                                    "distance_radial" => [150.0, 180.0]),
        "spline" => Dict("downloops" => !UP_LOOPS), "metrics" => Dict("avg_power_W" => 18000.0),
        "optimized_parameters" => Dict("input_depower" => 1.42))
    fs = fake_server(; reply, table, fail_warm)
    gui = joinpath(mktempdir(), "gui.yaml")
    write(gui, replace(read(joinpath(skc_data_path(), "gui.yaml.default"), String),
                       "system_fig8_200m.yaml" => PROJECT,
                       "wind_speed: \"default\"" => "wind_speed: \"$wind\""))
    traj_dir = mktempdir()
    data_path, traj_dir0 = KiteUtils.get_data_path(), FAILED_TRAJECTORY_DIR[]
    setup = st = err = nothing
    try
        GUI_STATE_FILE_OVERRIDE[] = gui
        FAILED_TRAJECTORY_DIR[] = traj_dir
        KiteUtils.set_data_path(skc_data_path())
        overrides = merge(Dict{Symbol, Any}(:base_url => fs.url, :autostart_server => false,
                                            :opt_success_cache => false, :opt_failure_cache => false),
                          tos)
        inputs = merge(run_input_defaults(), (; output_path = mktempdir(), tos_overrides = overrides))
        setup = setup_run(inputs; init_model = stand_in_model)
        st = RunState(; l_set = setup.l_set, opt_r_scale = setup.opt_r_scale,
                      opt_r_min = setup.opt_r_min, depower_flown_opt = setup.fcs.depower_setpoint)
        merge_into!(setup, solve_startup_path!(setup, st))
        log_startup_reply(setup.fcs, st.opt_result, setup.opt_r_min)
        log_lobe_lift(setup.fcs, st.opt_result)
        st.c1_startup = setup.c1_at_depower(setup.fcs.depower_setpoint)
        adopt_startup_path!(setup, st)
        finish_startup!(setup, st)
        capture_startup_geometry!(setup, st)
        merge_into!(setup, startup_feasibility(setup, st))
        init_phase5_and_controller!(setup, st)
        init_loop_state!(setup, st)
    catch exc
        err = exc
    finally
        GUI_STATE_FILE_OVERRIDE[] = nothing
        FAILED_TRAJECTORY_DIR[] = traj_dir0
        KiteUtils.set_data_path(data_path)
        close(fs.server)
        reload_turn_rate_table!()        # setup_run loads the project's table
    end
    return (; setup, st, err, paths = paths(fs), log = fs.log, saved = readdir(traj_dir))
end

@testset verbose = true "startup_pipeline" begin
    @testset "default_wind" begin
        r = fly_startup()
        @test isnothing(r.err)
        (; setup, st) = r
        @test setup isa RunSetup
        # No sim_time and no wind override: the project's own sim_time (it used to be `nothing`).
        @test setup.effective_sim_time == setup.project_set.sim_time
        @test r.paths == ["/health", "/init", "/step", "/trajectory"]
        init = r.log[2][2]
        @test init["length"] == 150.0 && init["min_turn_radius"] ≈ setup.opt_r_min
        @test !isnothing(init["pattern_limits"])
        # The reply as installed, and nothing retried: the path clears the default gate.
        @test st.opt_result.metrics.avg_power_W == 18000.0 && st.opt_power_pred == 18000.0
        @test st.opt_paths_raw[1][1] ≈ TIGHT[1] && st.opt_downloops == !UP_LOOPS
        @test isnothing(st.incumbent_score) && isempty(r.saved)
        @test setup.feas.feas_start.margin >= setup.tos.min_feasibility_margin
        @test st.depower_flown_opt ≈ SimpleKiteControllers.awetrim_depower_to_v3kite(1.42)
        # The state the loop starts from.
        @test st.n_path == length(setup.fec.az_path) == length(st.raw_az)
        @test length(st.p5_history) == 1 && st.cc isa CourseController
        @test st.ccs.el_center == st.el_c_path
    end

    @testset "wind_override" begin
        r = fly_startup(; wind = "8.25")
        @test isnothing(r.err)
        @test r.setup.project_set.v_wind == 8.25 && r.setup.inflow.wind_speed ≈ 8.25
        @test r.setup.effective_sim_time > 0
    end

    @testset "retry_takes_over" begin
        # The startup path misses a 1.3 gate; the server's next answer is the wide path.
        r = @test_logs (:info, r"Startup path clears the gates") match_mode = :any fly_startup(;
            tos = Dict{Symbol, Any}(:min_feasibility_margin => 1.3),
            path_for = p -> p <= 1000 ? TIGHT : WIDE)
        @test isnothing(r.err)
        (; setup, st) = r
        @test r.paths == ["/health", "/init", "/step", "/trajectory", "/step", "/trajectory"]
        # The retry is a warm step with its own turn-radius request, sized off the reply.
        @test isnothing(r.log[5][2]["trajectory"]) && !isnothing(r.log[5][2]["min_turn_radius"])
        @test r.log[5][2]["min_turn_radius"] != r.log[2][2]["min_turn_radius"]
        @test st.incumbent_score.ok && st.incumbent_score.margin >= 1.3
        @test st.opt_paths_raw == [st.inc_raw] && st.opt_paths_raw[1][1] ≈ WIDE[1]
        @test setup.feas.feas_start.margin >= 1.3
        # The startup path missed the gate, so its incumbent is recorded.
        @test length(r.saved) == 1 && startswith(only(r.saved), "startup_incumbent_")
    end

    @testset "retries_exhausted" begin
        # Every retry is answered with a 422: the run stops at the startup gate, as the live
        # `ladder` case of examples/regression_baseline.jl does.
        r = fly_startup(; tos = Dict{Symbol, Any}(:min_feasibility_margin => 1.3), fail_warm = true)
        @test r.err isa ErrorException && occursin("below min_feasibility_margin = 1.30", r.err.msg)
        @test count(==("/step"), r.paths) >= 2
        @test r.st.incumbent_score.margin < 1.3 && r.st.opt_paths_raw[1][1] ≈ TIGHT[1]
        @test any(startswith("startup_incumbent_"), r.saved)
    end
end
nothing
