# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the steering blocks of one step of the reel-out loop (`src/reelout_loop.jl`):
`steering_command!` (the hand-over to phase 3, the feed-forward log, the gain scale and the inputs
it hands `calc_steering`) and `xtrack_input!` (the cross-track test input). The kite's state is a
NamedTuple with the fields the blocks read; the course controller and the path are the real ones.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: steering_command!, xtrack_input!, loop_gain_scale, _bearing
using LinearAlgebra: norm

@testset verbose = true "steering_blocks" begin
    fcs = FC_Settings()
    n = 120
    s = range(0, 2pi; length = n + 1)[1:n]
    path = prepare_path(20 .* sin.(s), 25 .+ 5 .* sin.(2 .* s); resample = n, up_loops = fcs.up_loops)
    new_fec() = (fec = FigureEightController(fcs; dt = 0.02);
                 set_path!(fec, path...; up_loops = fcs.up_loops);
                 fec)
    new_cc() = CourseController(CourseControllerSettings(fcs; dt = 0.02))
    # The kite at (azimuth, elevation) [deg], flying east at 25 m/s.
    kite(az = 5.0, el = 26.0; heading = 0.3, course = 0.4) = (; heading, course, azimuth = deg2rad(az),
                                 elevation = deg2rad(el), v_app = 27.0, l_tether = [200.0],
                                 vel_kite = [0.0, 25.0, 0.0], v_reelout = [0.0])
    plant(ss = kite()) = (; ss, dt = 0.02, force = 5000.0)
    c1_setpoint = 0.28

    function steer_setup(; c1_now = c1_setpoint)
        (; fcs, tos = (; fly_opt_depower = false), c1_setpoint, c1_depower_max = Inf,
         c1_ctrl_at = dp -> c1_now, dt0 = 0.02, fec = new_fec())
    end

    @testset "hand_over_to_phase_3" begin
        setup = steer_setup()
        st = RunState(; cc = new_cc(), rel_depower_prev = fcs.depower_setpoint)
        set_phase!(st.cc, 2)
        st.cc.hold_start = 10.0
        hold = st.cc.ccs.hold_time
        cmd = steering_command!(st, setup, plant(), 10.0 + hold / 2, 0.2, 1.0)
        @test cmd.phase == 2 && isnan(st.transition_start)
        cmd = steering_command!(st, setup, plant(), 10.0 + hold + 0.01, 0.2, 1.0)
        @test cmd.phase == 3 && st.transition_start == 10.0 + hold + 0.01
        # What it returns is the controller's own state after the step.
        @test cmd.chi_cmd == st.cc.chi_cmd && cmd.w_lim == st.cc.w_lim && cmd.err == st.cc.err
        # The feed-forward is logged every step, and is off before phase 4.
        @test length(st.ff_log) == length(st.ff_chi_log) == 2
        @test all(iszero, st.ff_log) && cmd.u_ff == 0.0
    end

    @testset "gain_scale_and_inputs" begin
        # c1 at the flown depower half the setpoint's: the loop gain is scaled up twice.
        setup = steer_setup(; c1_now = c1_setpoint / 2)
        st = RunState(; cc = new_cc(), rel_depower_prev = 0.3)
        set_phase!(st.cc, 3)
        ref = deepcopy(st.cc)
        # Heading and course 0.4 rad once `course_offset` is applied: the error is the 0.01 rad commanded.
        ss = kite(; heading = 0.4, course = 0.4 - st.cc.ccs.course_offset)
        gain = loop_gain_scale(c1_setpoint, 0.3, Inf, setup.c1_ctrl_at)
        @test gain == 2.0
        chi_set = 0.41                              # a small course error: the PID does not saturate
        cmd = steering_command!(st, setup, plant(ss), 40.0, chi_set, 1.5)
        u_ref, dp_ref, phase_ref = calc_steering(ref, chi_set, ss.heading, ss.course; t = 40.0,
            elevation = ss.elevation, v_kite = norm(ss.vel_kite), v_app = ss.v_app, dmin = 1.5,
            tangent = path_tangent(setup.fec), gain_scale = gain, u_ff = 0.0, chi_ff = 0.0)
        @test cmd.rel_steering == u_ref && cmd.rel_depower == dp_ref && cmd.phase == phase_ref
        # And the scale matters: at the setpoint's own c1 the same step steers differently.
        st1 = RunState(; cc = new_cc(), rel_depower_prev = 0.3)
        set_phase!(st1.cc, 3)
        cmd1 = steering_command!(st1, steer_setup(), plant(ss), 40.0, chi_set, 1.5)
        @test abs(cmd.rel_steering) < fcs.max_steering && abs(cmd1.rel_steering) < fcs.max_steering
        @test cmd1.rel_steering != cmd.rel_steering
    end

    @testset "xtrack_input" begin
        fec = new_fec()
        calc_attractor(fec, path[1][30], path[2][30])
        chi, az_a, el_a = 0.1, path[1][40], path[2][40]
        # No test input, or before its phase: the guidance passes unchanged and nothing is logged.
        for xtrack_offset in (nothing, τ -> 1.0)
            setup = (; xtrack_offset, xtrack_phase = 4, fec)
            st = RunState(; cc = new_cc())
            set_phase!(st.cc, 3)
            @test xtrack_input!(st, setup, plant(), 20.0, chi, az_a, el_a) == (chi, az_a, el_a)
            @test isempty(st.xt_t) && isnan(st.xt_start)
        end
        # From its phase on: the attractor moved δ along the path normal, the course re-aimed at it.
        setup = (; xtrack_offset = τ -> τ >= 1 ? 1.0 : 0.0, xtrack_phase = 4, fec)
        st = RunState(; cc = new_cc(), rel_depower_prev = 0.3)
        set_phase!(st.cc, 4)
        ss = kite(path[1][30], path[2][30])
        @test xtrack_input!(st, setup, plant(ss), 50.0, chi, az_a, el_a) == (chi, az_a, el_a)
        @test st.xt_start == 50.0 && st.xt_delta == [0.0]          # δ = 0 for τ < 1: logged only
        chi2, az2, el2 = xtrack_input!(st, setup, plant(ss), 51.0, chi, az_a, el_a)
        na, ne = path_normal(fec, attractor_index(fec))
        @test az2 ≈ az_a + na / cosd(el_a) && el2 ≈ el_a + ne
        @test chi2 ≈ _bearing(ss.azimuth, ss.elevation, deg2rad(az2), deg2rad(el2))
        # Every step logged: time, offset, the error to the UNSHIFTED path, Q, phase, L, speeds.
        @test st.xt_t == [50.0, 51.0] && st.xt_delta == [0.0, 1.0]
        @test st.xt_d[end] ≈ signed_cross_track(fec, path[1][30], path[2][30])
        @test st.xt_q[end] == fec.last_idx && st.xt_phase == [4, 4] && st.xt_L == [200.0, 200.0]
        @test st.xt_va[end] == 27.0 && st.xt_dp[end] == 0.3 && st.xt_vk[end] == 25.0
    end
end
nothing
