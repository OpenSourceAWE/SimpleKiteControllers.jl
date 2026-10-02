# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the startup of a reel-out run that need no optimizer (`src/startup_path.jl`): the
gates on an installed path, the lobe lift rationed to the curvature gate, the startup geometry, the
feasibility laws the loop reads, and the state the loop starts from. Each takes a hand-made
`RunState` and `setup` on a synthetic figure of eight. `retry_startup!` is covered by the live
`ladder` case of `examples/regression_baseline.jl`, its decisions by `test_startup_retry.jl`.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: score_installed, install_optimized_path!, capture_startup_geometry!,
    startup_feasibility, init_phase5_and_controller!, init_loop_state!, TrajOptSettings, c1_at,
    phase5_margin, log_lobe_lift

@testset verbose = true "startup_path" begin
    fcs = FC_Settings()
    up_loops = fcs.pattern.up_loops
    n = 120
    s = range(0, 2pi; length = n + 1)[1:n]
    eight(a, b, c) = prepare_path(a .* sin.(s), c .+ b .* sin.(2 .* s); resample = n, up_loops)
    new_fec(path) = (fec = FigureEightController(fcs; dt = 0.02);
                     set_path!(fec, path...; up_loops);
                     fec)
    gentle = eight(20.0, 5.0, 25.0)   # clears the default gates at L = 200 m (margin 1.18)
    tight = eight(8.0, 2.0, 25.0)     # does not (0.47)
    l_tether = 200.0
    c1 = 0.28

    @testset "score_installed" begin
        fec = new_fec(gentle)
        st = RunState(; c1_startup = c1)
        setup(; margin = 0.82, min_height = 50.0, el_floor = 13.0) =
            (; fec, l_tether, fcs = FC_Settings(; max_steering = fcs.course.max_steering),
             tos = (; min_feasibility_margin = margin, min_height), el_floor)
        m = check_pattern_feasible(fec, l_tether, fcs.course.max_steering; c1, prn = false).margin
        r = score_installed(setup(), st)
        @test r.margin == m && r.el_ok && r.clr_ok && r.ok
        @test r.height ≈ path_min_height(fec, l_tether)
        @test !score_installed(setup(; margin = m + 0.01), st).ok         # the turn margin
        r = score_installed(setup(; el_floor = 21.0), st)                 # the elevation floor
        @test !r.el_ok && !r.ok
        r = score_installed(setup(; min_height = 100.0), st)              # the clearance
        @test !r.clr_ok && !r.ok
        r = score_installed(setup(; min_height = 0.0), st)                # clearance off
        @test isnan(r.height) && r.clr_ok
    end

    @testset "install_optimized_path" begin
        az, el = figure_eight_path(20.0, 10.0, 0.0, 25.0, 0.0, 101)
        az, el = collect(az), collect(el)
        reply = (; trajectory = (; azimuth = az, elevation = el))
        wing_lift(a, e) = lobe_lift(a, e; lift = 2.0)
        lift = wing_lift(az, el)
        function install(; margin, c1 = c1)
            fec = new_fec(gentle)
            setup = (; tos = (; resample_points = 361, min_feasibility_margin = margin),
                     fcs = FC_Settings(; max_steering = fcs.course.max_steering, el_offset_wing = 2.0), fec,
                     l_tether, wing_lift, c1_at_depower = dp -> c1, pattern_depower = r -> 0.27)
            st = RunState()
            raw = install_optimized_path!(setup, st, reply)
            return st, fec, raw
        end
        # The path as installed with a share `fw` of the lift, resampled to the reply's own 100 points.
        function installed(fw)
            fec = new_fec(gentle)
            set_path!(fec, az, el .+ fw .* lift; resample = 100)
            return fec
        end
        margin_at(fw) = check_pattern_feasible(installed(fw), l_tether, fcs.course.max_steering;
                                               c1, prn = false).margin
        st, fec, raw = install(; margin = 0.0)       # gate off: the whole lift
        @test raw == (az, el) && st.startup_wing_frac == 1.0 && st.c1_startup == c1
        @test fec.el_path ≈ installed(1.0).el_path
        # A gate the whole lift misses: the first rung that clears it is installed.
        rungs = (1.0, 0.75, 0.5, 0.25, 0.0)
        margins = map(margin_at, rungs)
        gate = (margins[1] + maximum(margins)) / 2
        @test margins[1] < gate                     # the test needs a lift that costs margin
        fw = rungs[something(findfirst(>=(gate), margins), length(rungs))]
        st, fec, _ = @test_logs (:info, r"Lobe lift held back") install(; margin = gate)
        @test st.startup_wing_frac == fw < 1
        @test fec.el_path ≈ installed(fw).el_path
        st, _, _ = install(; margin = gate, c1 = NaN)  # no turn-rate law: not rationed
        @test st.startup_wing_frac == 1.0
    end

    @testset "capture_startup_geometry" begin
        fec = new_fec(gentle)
        setup = (; fec, l_tether, fcs = FC_Settings(; up_loops))
        st = RunState(; opt_power_pred = 18000.0, opt_downloops = !up_loops)
        capture_startup_geometry!(setup, st)
        @test st.n_path_initial == length(fec.az_path)
        @test st.az_amp_path ≈ 20.0 atol = 0.1
        @test st.el_c_path ≈ 25.0 atol = 0.1
        @test st.el_height_path ≈ 10.0 atol = 0.1
        @test st.az_c_path ≈ 0.0 atol = 1e-9
        @test st.path_min_h_start ≈ path_min_height(fec, l_tether)
        @test st.pred_timeline == [(t = 0.0, power = 18000.0)]
        # A reply that flies against up_loops is refused, not flown reversed.
        st = RunState(; opt_power_pred = 18000.0, opt_downloops = up_loops)
        @test_throws ErrorException capture_startup_geometry!(setup, st)
    end

    @testset "startup_feasibility" begin
        tos = TrajOptSettings()
        c1_at_depower(dp) = turn_rate_coeffs(fcs.run.body_damping, dp).c1
        setup(path) = (; fec = new_fec(path), fcs, tos, l_tether, c1_at_depower,
                       pattern_depower = r -> fcs.course.depower_setpoint)
        st = RunState(; depower_flown_opt = 0.27)
        r = startup_feasibility(setup(gentle), st)
        @test r.feas.feas_start.margin >= tos.min_feasibility_margin
        # The laws the loop reads: the table's c1 at a depower in phases 3-4, the phase-5 law in 5.
        @test r.c1_at_phase(4, 0.27) == c1_at(r.feas, 4, c1_at_depower(0.27))
        @test r.c1_at_phase(5, 0.27) == c1_at(r.feas, 5, NaN)
        dp_st = tos.fly_opt_depower ? st.depower_flown_opt : fcs.course.depower_setpoint
        @test r.c1_at_phase(4, st) == r.c1_at_phase(4, dp_st)
        @test r.phase5_margin_at(gentle...) ==
              phase5_margin(r.feas, gentle[1], gentle[2], fcs.reelout.reelout_l_max, fcs.course.max_steering)
        @test r.margin5 isa Phase5MarginState
        @test_throws ErrorException startup_feasibility(setup(tight), st)   # the gate refuses it
    end

    @testset "init_phase5_and_controller" begin
        fec = new_fec(gentle)
        setup = (; fec, fcs, s = (; dt = 0.01), phase5_margin_at = (az, el) -> 1.7)
        st = RunState(; opt_paths_raw = [gentle], el_c_path = 24.0, p5_fallback_done = true)
        init_phase5_and_controller!(setup, st)
        @test length(st.p5_history) == 1
        h = st.p5_history[1]
        @test h.t == 0.0 && h.margin == 1.7 && h.el_applied == 0.0
        @test h.az == fec.az_path && h.az !== fec.az_path   # a copy, not the live path
        @test h.raw == gentle
        @test !st.p5_fallback_done && isnan(st.p5_q_az_prev) && isnothing(st.p5_fallback)
        @test st.ccs.el_center == 24.0 && st.cc isa CourseController   # the dive aims at the centre
    end

    @testset "init_loop_state" begin
        raw = (collect(gentle[1]), collect(gentle[2]))
        tos = (; resample_points = 361)
        fec = new_fec(gentle)
        set_path!(fec, raw...; resample = n - 1)           # as install_optimized_path! does
        setup = (; fcs = FC_Settings(; depower_setpoint = 0.274, up_loops), fec, tos)
        st = RunState(; opt_paths_raw = [raw], depower_flown_opt = 0.27)
        fec.last_idx = 7
        init_loop_state!(setup, st)
        @test st.rel_depower_prev == 0.274 && st.fig8_idx_prev == 7
        @test st.n_path == length(fec.az_path) == length(st.raw_az) == st.chk_points
        @test st.depower_flown == 0.27 && st.depower_blend_from == 0.27
        # The scored reference must have as many points as the path flown.
        set_path!(fec, raw...; resample = 50)
        @test_throws ErrorException init_loop_state!(setup, RunState(; opt_paths_raw = [raw]))
    end

    @testset "log_lobe_lift" begin
        reply = (; trajectory = (; azimuth = [-30.0, 0.0, 30.0]))
        lift(mode; lift = 2.0) = FC_Settings(; el_offset_wing = lift, el_offset_wing_mode = mode,
                                  el_offset_wing_az = 0.5, el_offset_wing_blend = 0.25)
        @test_logs (:info, r"beyond \|azimuth\| = 0\.5°") log_lobe_lift(lift("azimuth"), reply)
        # As fractions of the pattern's own amplitude, here ±30°: 15° and 7.5°.
        @test_logs (:info, r"0\.50 of the pattern's own amplitude.*15\.0° and 7\.5° on the startup path's ±30\.0°") log_lobe_lift(
            lift("azimuth_frac"), reply)
        @test_logs log_lobe_lift(lift("azimuth_frac"; lift = 0.0), reply)   # no lift: nothing said
    end
end
nothing
