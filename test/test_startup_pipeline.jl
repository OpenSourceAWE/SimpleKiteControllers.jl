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
import SimpleKiteControllers: GUI_STATE_FILE_OVERRIDE, FAILED_TRAJECTORY_DIR, setup_run, merge_into!,
    solve_startup_path!, log_startup_reply, log_lobe_lift, adopt_startup_path!, finish_startup!,
    capture_startup_geometry!, startup_feasibility, init_phase5_and_controller!, init_loop_state!,
    startup_params, startup_solve, write_run_done, step_commands!, record_step!, check_overspeed,
    OptChain, KiteUtils
@isdefined(fake_server) || include(joinpath(@__DIR__, "fake_awetrim_server.jl"))

# Not `PROJECT`: the example scripts assign that as a plain global in the same session, and a
# `const` of that name in `Main` makes their next `include` fail.
const PIPELINE_PROJECT = "system_reelout_maasvlakte.yaml"
const UP_LOOPS = FC_Settings(fc_settings(project_file(PIPELINE_PROJECT))).pattern.up_loops
# Two 100-point figures of eight [deg]: TIGHT misses a 1.3 gate at 150 m (margin 1.24), WIDE clears it.
function eight_path(a, b, c)
    r = range(0, 2pi; length = 101)[1:100]
    return prepare_path(a .* sin.(r), c .+ b .* sin.(2 .* r); resample = 100, up_loops = UP_LOOPS)
end
const TIGHT = eight_path(25.0, 6.0, 26.0)
const WIDE = eight_path(32.0, 8.0, 27.0)
const NARROW_EIGHT = eight_path(20.0, 5.0, 26.0)   # tighter than TIGHT: a retry that is no better
const LOW_EIGHT = eight_path(32.0, 8.0, 12.0)      # wide, but below the elevation floor

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
                       "system_fig8_200m.yaml" => PIPELINE_PROJECT,
                       "wind_speed: \"default\"" => "wind_speed: \"$wind\""))
    traj_dir = mktempdir()
    data_path, traj_dir0 = KiteUtils.get_data_path(), FAILED_TRAJECTORY_DIR[]
    setup = st = err = nothing
    try
        GUI_STATE_FILE_OVERRIDE[] = gui
        FAILED_TRAJECTORY_DIR[] = traj_dir
        KiteUtils.set_data_path(skc_data_path())
        overrides = merge(Dict{Symbol, Any}(:base_url => fs.url,
                                            :opt_success_cache => false, :opt_failure_cache => false),
                          tos)
        inputs = merge(run_input_defaults(), (; output_path = mktempdir(), tos_overrides = overrides))
        setup = setup_run(inputs; init_model = stand_in_model)
        st = RunState(; l_set = setup.l_set, opt_r_scale = setup.opt_r_scale,
                      opt_r_min = setup.opt_r_min, depower_flown_opt = setup.fcs.course.depower_setpoint)
        merge_into!(setup, solve_startup_path!(setup, st))
        log_startup_reply(setup.fcs, st.opt_result, setup.opt_r_min)
        log_lobe_lift(setup.fcs, st.opt_result)
        st.c1_startup = setup.c1_at_depower(setup.fcs.course.depower_setpoint)
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

    tos_gate = Dict{Symbol, Any}(:min_feasibility_margin => 1.3)
    @testset "retry_no_better" begin
        # Every retry converges to a tighter path: each is saved as rejected and the incumbent kept.
        r = @test_logs (:info, r"keeping the incumbent") match_mode = :any fly_startup(;
            tos = tos_gate, path_for = p -> p <= 1000 ? TIGHT : NARROW_EIGHT)
        @test r.err isa ErrorException
        n_retries = Int(r.setup.tos.startup_retries_max)
        @test count(==("/step"), r.paths) == 1 + n_retries
        @test count(startswith("startup_retry"), r.saved) == n_retries
        @test r.st.opt_paths_raw[1][1] ≈ TIGHT[1] && r.st.incumbent_score.margin < 1.3
        @test r.setup.fec.az_path ≈ prepare_path(TIGHT...; resample = 99, up_loops = UP_LOOPS)[1]
    end

    @testset "retry_below_floor" begin
        # A retry that widens past the gate but sinks below the elevation floor stops the retries.
        r = @test_logs (:warn, r"dropped below a floor") match_mode = :any fly_startup(;
            tos = tos_gate, path_for = p -> p <= 1000 ? TIGHT : LOW_EIGHT)
        @test r.err isa ErrorException
        @test count(==("/step"), r.paths) == 2
        @test r.st.opt_paths_raw[1][1] ≈ TIGHT[1]
        @test minimum(r.setup.fec.el_path) >= r.setup.el_floor   # the incumbent is flown
    end

    r = fly_startup()
    (; setup, st) = r

    @testset "seeding_solve" begin
        # With opt_warm_start_awe_trim above the winch's use_awe_trim, a seeding /step at the
        # unreduced winch goes before the first-lap one, which starts from its trajectory.
        fs = fake_server()
        try
            tos = deepcopy(setup.tos)
            tos.opt_warm_start_awe_trim = 1.0
            su = (; opt_chain = OptChain(fs.url; successes = false, failures = false), tos,
                  winch = (; use_awe_trim = 0.5), rcs = setup.rcs, l_set = setup.l_set,
                  winch_first_lap = setup.winch_first_lap)
            result, seed = @test_logs (:info, r"Seeding solve") match_mode = :any startup_solve(
                su, startup_params(setup, setup.el_center_seed))
            @test paths(fs) == ["/init", "/step", "/trajectory", "/step", "/trajectory"]
            warm, first_lap = fs.log[2][2], fs.log[4][2]
            @test warm["winch_params"]["use_awe_trim"] == 1.0
            @test warm["winch_params"]["f_max"] == setup.rcs.f_high
            @test first_lap["winch_params"]["f_max"] == setup.winch_first_lap.f_max
            @test first_lap["trajectory"]["azimuth"] == seed.azimuth
            @test result.metrics.avg_power_W == 2000.0
        finally
            close(fs.server)
        end
    end

    @testset "turn_rate_laws" begin
        # Off the table the laws read NaN; the path side's warns once per run.
        @test (@test_logs (:warn, r"No turn-rate coefficients") setup.c1_at_depower(5.0)) |> isnan
        @test (@test_logs setup.c1_at_depower(5.0)) |> isnan
        @test isnan(setup.c1_ctrl_at(5.0))
        @test setup.c1_ctrl_at(setup.fcs.course.depower_setpoint) == setup.c1_setpoint > 0
        @test !setup.power_gate_off(1000.0)
        @test setup.pattern_depower(st.opt_result) isa Float64
    end

    @testset "first_steps" begin
        # The loop's first steps on a parked kite: the blocks run, and record_step! fills the log slots.
        ss = KiteUtils.SysState(7)
        ss.l_tether[1] = setup.l_set
        ss.elevation, ss.azimuth, ss.v_app = deg2rad(60), 0.0, 20.0
        ss.vel_kite .= (0.0, 0.0, 5.0)
        ss.v_wind_gnd .= (8.0, 0.0, 0.0)
        ss.winch_force[1], ss.v_reelout[1] = 3000.0, 0.5
        dt = setup.s.dt
        phases = Int[]
        cmds = nothing
        for k in 1:200
            t = k * dt
            cmds = step_commands!(st, setup, (; ss, dt, force = 3000.0, v_reel = 0.0), t)
            record_step!(st, setup, (; ss, dt, aoa = 0.1, wind_factor_200 = 1.3), t, cmds)
            push!(phases, cmds.phase)
        end
        @test first(phases) == 0 && all(<(3), phases) && issorted(phases)
        @test cmds.v_set == 0.0 && abs(cmds.rel_steering) <= setup.fcs.course.max_steering
        @test ss.sys_state == phases[end] && ss.var_10 == st.l_set && ss.var_11 == 0.0
        @test ss.var_09 ≈ rad2deg(0.1) && ss.v_wind_200m ≈ [10.4, 0.0, 0.0]
        @test st.e_mech ≈ 200 * 3000.0 * 0.5 * dt / 3600 && ss.e_mech ≈ st.e_mech
        @test length(st.geom_t) == 200 && length(st.ff_log) == 200
        @test !check_overspeed(setup, (; ss))
        ss.v_app = setup.fcs.run.v_app_abort + 1
        @test (@test_logs (:error, r"Overspeed") check_overspeed(setup, (; ss)))
    end

    @testset "write_run_done" begin
        su = (; run_done_file = joinpath(mktempdir(), "last_run_done.txt"), output_path = "/out",
              log_name = "run_opt")
        write_run_done(su, (; archive_dir = "/arch", fig8m = nothing, opt_power_meas = nothing), "ok")
        txt = read(su.run_done_file, String)
        @test occursin("status: ok", txt) && occursin("archive: /arch", txt)
        @test occursin("log: /out/run_opt.yaml", txt)
        @test occursin("criteria: n/a", txt) && occursin("power: n/a", txt) && !occursin("error:", txt)
        fig8m = (; criteria = 7, criteria_failed = String[])
        write_run_done(su, (; archive_dir = "", fig8m, opt_power_meas = 12345.6), "ok")
        txt = read(su.run_done_file, String)
        @test occursin("criteria: all 7 passed", txt) && occursin("power: 12346 W measured", txt)
        fig8m = (; criteria = 7, criteria_failed = ["min elevation", "laps"])
        write_run_done(su, (; archive_dir = "", fig8m, opt_power_meas = nothing), "FAILED";
                       err = ErrorException("boom\nsecond line"))
        txt = read(su.run_done_file, String)
        @test occursin("status: FAILED", txt) && occursin("error: boom\n", txt)
        @test !occursin("second line", txt) && occursin("criteria: FAILED: min elevation, laps", txt)
    end
end
nothing
