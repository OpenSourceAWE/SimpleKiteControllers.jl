# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the figure-of-eight inner loop (course controller). Pure
arithmetic, no kite model, so this runs in well under a second.
"""

using Test
using SimpleKiteControllers
import KiteUtils: wrap2pi

@testset verbose = true "course_controller" begin

    @testset "bumpless_park" begin
        ccs = CourseControllerSettings(; dt = 0.02, heading_d = 0.1, park_time = 0.04)
        cc = CourseController(ccs)
        chi_set = deg2rad(10.0)
        for t in (0.0, 0.02)
            rel, _, phase = calc_steering(cc, chi_set, 0.0, 0.0;
                t, elevation = deg2rad(60.0), v_kite = 8.0, v_app = 20.0,
                dmin = 0.0, tangent = 0.0)
            @test phase == 0
            @test rel == 0.0
        end
        rel1, _, phase1 = calc_steering(cc, chi_set, 0.0, 0.0;
            t = 0.04, elevation = deg2rad(60.0), v_kite = 8.0, v_app = 20.0,
            dmin = 0.0, tangent = 0.0)
        @test phase1 == 1

        # A fresh controller's very first call, at the same phase-1 input, should
        # match exactly: stepping the PID with zero error during park leaves no
        # residual state to jump from.
        cc2 = CourseController(CourseControllerSettings(; dt = 0.02, heading_d = 0.1,
                                                         park_time = -1.0))
        rel2, _, phase2 = calc_steering(cc2, chi_set, 0.0, 0.0;
            t = 0.04, elevation = deg2rad(60.0), v_kite = 8.0, v_app = 20.0,
            dmin = 0.0, tangent = 0.0)
        @test phase2 == 1
        @test rel1 ≈ rel2
    end

    @testset "gain_schedule" begin
        ccs = CourseControllerSettings(; dt = 0.02, heading_p = 0.2, v_app_ref = 20.0,
                                       v_app_min = 5.0, entry_gain = 0.5)
        cc = CourseController(ccs)
        set_phase!(cc, 3)
        calc_steering(cc, 0.0, 0.0, 0.0; t = 100.0, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = 20.0, dmin = 100.0, tangent = 0.0)
        @test cc.pid.K ≈ 0.2
        calc_steering(cc, 0.0, 0.0, 0.0; t = 100.02, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = 10.0, dmin = 100.0, tangent = 0.0)
        @test cc.pid.K ≈ 0.4   # v_app halved -> K doubled
        calc_steering(cc, 0.0, 0.0, 0.0; t = 100.04, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = 1.0, dmin = 100.0, tangent = 0.0)
        @test cc.pid.K ≈ 0.2 * 20.0 / 5.0   # v_app_min clamps v_app

        # phase < 3 applies entry_gain (default park_time = 2.0, t = -1 stays phase 0)
        cc2 = CourseController(ccs)
        _, _, phase = calc_steering(cc2, 0.0, 0.0, 0.0; t = -1.0, elevation = deg2rad(80.0),
                                    v_kite = 0.0, v_app = 20.0, dmin = 100.0, tangent = 0.0)
        @test phase == 0
        @test cc2.pid.K ≈ 0.5 * 0.2
    end

    @testset "clamp" begin
        ccs = CourseControllerSettings(; dt = 0.02, heading_p = 1.0, heading_i = false,
                                       heading_d = false, max_steering = 0.3, entry_gain = 1.0)
        cc = CourseController(ccs)
        set_phase!(cc, 3)
        # v_kite = 0 forces pure-heading feedback (psi' = heading = 0), so a 180°
        # chi_set is a pure ±180° error with nothing to wind up (heading_i = false).
        rel, _, _ = calc_steering(cc, deg2rad(180.0), 0.0, 0.0;
                                  t = 0.0, elevation = deg2rad(80.0),
                                  v_kite = 0.0, v_app = ccs.v_app_ref,
                                  dmin = 0.0, tangent = 0.0)
        @test abs(rel) ≈ ccs.max_steering
    end

    @testset "wrapped_error" begin
        ccs = CourseControllerSettings(; dt = 0.02)
        cc = CourseController(ccs)
        set_phase!(cc, 3)
        calc_steering(cc, deg2rad(179.0), deg2rad(-179.0), 0.0;
                     t = 0.0, elevation = deg2rad(80.0), v_kite = 0.0, v_app = ccs.v_app_ref,
                     dmin = 0.0, tangent = 0.0)
        @test cc.psi_prime ≈ deg2rad(-179.0)
        @test rad2deg(cc.err) ≈ 2.0
    end

    @testset "psi_prime_endpoints" begin
        ccs = CourseControllerSettings(; dt = 0.02, v_kite_heading = 5.0, v_kite_course = 10.0,
                                       fig8_pure_course = false)
        cc = CourseController(ccs)
        set_phase!(cc, 3)
        heading = deg2rad(10.0); course = deg2rad(20.0)
        calc_steering(cc, 0.0, heading, course; t = 0.0, elevation = deg2rad(80.0),
                     v_kite = 5.0, v_app = ccs.v_app_ref, dmin = 0.0, tangent = 0.0)
        @test cc.w_course == 0.0
        @test cc.psi_prime ≈ heading
        calc_steering(cc, 0.0, heading, course; t = 0.02, elevation = deg2rad(80.0),
                     v_kite = 10.0, v_app = ccs.v_app_ref, dmin = 0.0, tangent = 0.0)
        @test cc.w_course == 1.0
        @test cc.psi_prime ≈ wrap2pi(course + ccs.course_offset)
        calc_steering(cc, 0.0, heading, course; t = 0.04, elevation = deg2rad(80.0),
                     v_kite = 7.5, v_app = ccs.v_app_ref, dmin = 0.0, tangent = 0.0)
        @test cc.w_course ≈ 0.5

        cc2 = CourseController(CourseControllerSettings(; dt = 0.02, fig8_pure_course = true))
        set_phase!(cc2, 3)
        calc_steering(cc2, 0.0, heading, course; t = 0.0, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 0.0, tangent = 0.0)
        @test cc2.w_course == 1.0   # forced despite v_kite below v_kite_heading

        # course_offset applied exactly once: zeroing it leaves psi' = course unshifted.
        cc3 = CourseController(CourseControllerSettings(; dt = 0.02, course_offset = 0.0))
        set_phase!(cc3, 3)
        calc_steering(cc3, 0.0, heading, course; t = 0.0, elevation = deg2rad(80.0),
                     v_kite = 10.0, v_app = ccs.v_app_ref, dmin = 0.0, tangent = 0.0)
        @test cc3.psi_prime ≈ course
    end

    @testset "limiter" begin
        ccs = CourseControllerSettings(; dt = 0.02, entry_d_gate = 12.0, entry_d_blend = 4.0)
        for (dmin, expected) in ((5.0, 0.0), (12.0, 0.0), (14.0, 0.5), (16.0, 1.0), (20.0, 1.0))
            cc = CourseController(ccs)
            calc_steering(cc, 0.0, 0.0, 0.0; t = -1.0, elevation = deg2rad(80.0),
                         v_kite = 0.0, v_app = ccs.v_app_ref, dmin, tangent = 1.0)
            @test cc.w_lim ≈ expected
        end
        # Hard switch at entry_d_blend = 0.
        ccs2 = CourseControllerSettings(; dt = 0.02, entry_d_gate = 12.0, entry_d_blend = 0.0)
        cc2 = CourseController(ccs2)
        calc_steering(cc2, 0.0, 0.0, 0.0; t = -1.0, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = ccs2.v_app_ref, dmin = 11.9, tangent = 1.0)
        @test cc2.w_lim == 0.0
        cc3 = CourseController(ccs2)
        calc_steering(cc3, 0.0, 0.0, 0.0; t = -1.0, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = ccs2.v_app_ref, dmin = 12.1, tangent = 1.0)
        @test cc3.w_lim == 1.0
        # entry_chi_max = 180 disables the limiter outright: chi_cmd stays raw chi_set.
        ccs4 = CourseControllerSettings(; dt = 0.02, entry_chi_max = 180.0,
                                        entry_d_gate = 0.0, entry_d_blend = 0.0)
        cc4 = CourseController(ccs4)
        calc_steering(cc4, deg2rad(170.0), 0.0, 0.0; t = -1.0, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = ccs4.v_app_ref, dmin = 100.0, tangent = 1.0)
        @test cc4.chi_cmd ≈ deg2rad(170.0)
    end

    @testset "sign_latch" begin
        ccs = CourseControllerSettings(; dt = 0.02, entry_chi_max = 90.0, entry_d_gate = 0.0,
                                       entry_d_blend = 0.0, entry_cut_margin = 30.0)
        cc = CourseController(ccs)
        calc_steering(cc, deg2rad(175.0), 0.0, 0.0; t = -1.0, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = -1.0)
        @test cc.entry_sign == -1
        # A different chi_set/tangent afterwards must not move the latch.
        calc_steering(cc, deg2rad(-175.0), 0.0, 0.0; t = -0.98, elevation = deg2rad(80.0),
                     v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = 1.0)
        @test cc.entry_sign == -1
    end

    @testset "ladder" begin
        ccs = CourseControllerSettings(; dt = 0.02, park_time = 1.0, dive_el_margin = 7.0,
                                       el_center = 26.0, hold_time = 0.5, fig8_d_gate = 5.0)
        cc = CourseController(ccs)
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 0.5, elevation = deg2rad(60.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = 0.0)
        @test p == 0
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.0, elevation = deg2rad(60.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = 0.0)
        @test p == 1
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.02, elevation = deg2rad(40.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = 0.0)
        @test p == 1
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.04, elevation = deg2rad(30.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = 0.0)
        @test p == 2
        @test cc.hold_start ≈ 1.04
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.4, elevation = deg2rad(30.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = 0.0)
        @test p == 2
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.55, elevation = deg2rad(30.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 100.0, tangent = 0.0)
        @test p == 3
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.6, elevation = deg2rad(30.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 10.0, tangent = 0.0)
        @test p == 3
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.62, elevation = deg2rad(30.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 4.0, tangent = 0.0)
        @test p == 4

        @test_throws ErrorException set_phase!(cc, 2)   # never backwards
        set_phase!(cc, 5)                                # winch-triggered, from outside
        @test cc.phase == 5
        _, _, p = calc_steering(cc, 0.0, 0.0, 0.0; t = 1.64, elevation = deg2rad(30.0),
                                v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 4.0, tangent = 0.0)
        @test p == 5   # the ladder itself never reaches 5, so it stays put

        # set_phase!(cc, 5) can fire the SAME step a 3->4 transition just happened,
        # mirroring examples/simple_reelout.jl.
        cc2 = CourseController(ccs)
        set_phase!(cc2, 3)
        _, _, p2 = calc_steering(cc2, 0.0, 0.0, 0.0; t = 10.0, elevation = deg2rad(30.0),
                                 v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 1.0, tangent = 0.0)
        @test p2 == 4
        set_phase!(cc2, 5)
        @test cc2.phase == 5
    end

    @testset "steering_sign_convention" begin
        # Pinned regression: a commanded course AHEAD of (larger than) the feedback
        # angle must yield a positive rel_steering — this plant's PID output is fed
        # to rel_steering UNNEGATED. (`err = psi' - chi_cmd` is fed to the PID as
        # `(r=0, y=err)`, whose P term is `K*(0 - err)`, so chi_cmd > psi' -> err < 0
        # -> rel_steering > 0.)
        ccs = CourseControllerSettings(; dt = 0.02, heading_p = 1.0, heading_i = false,
                                       heading_d = false, max_steering = 1.0, entry_gain = 1.0)
        cc = CourseController(ccs)
        set_phase!(cc, 3)
        rel, _, _ = calc_steering(cc, deg2rad(10.0), 0.0, 0.0;
                                  t = 0.0, elevation = deg2rad(80.0),
                                  v_kite = 0.0, v_app = ccs.v_app_ref, dmin = 0.0, tangent = 0.0)
        @test rel > 0
    end

    @testset "numeric_fixture" begin
        # Hand-checked, in the style of test_calc_steering: a refactor that changes
        # a formula should show up here as a number, not just as a green suite.
        ccs = CourseControllerSettings(; dt = 0.02, heading_p = 0.2, heading_i = false,
                                       heading_d = false, max_steering = 1.0,
                                       entry_gain = 1.0, v_app_ref = 20.0, v_app_min = 5.0)
        cc = CourseController(ccs)
        set_phase!(cc, 3)
        rel, rel_depower, phase = calc_steering(cc, deg2rad(30.0), deg2rad(10.0), 0.0;
            t = 0.0, elevation = deg2rad(80.0), v_kite = 0.0, v_app = 20.0,
            dmin = 0.0, tangent = 0.0)
        @test rel ≈ 0.06981317007977318
        @test rel_depower == ccs.depower_setpoint
        @test phase == 4   # dmin = 0.0 < fig8_d_gate advances 3 -> 4 immediately
    end
end
