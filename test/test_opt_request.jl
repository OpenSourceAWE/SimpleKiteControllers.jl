# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for what a request asks the optimizer for (`src/opt_request.jl`): the depower seed and
its conversion to V3Kite's, the minimum turn radius and the constraints of the startup solve.
Settings and the package's turn-rate table in, numbers out; no optimizer.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: depower_conversion, with_depower_conversion, awetrim_depower_to_v3kite,
    DEPOWER_SEED_BOUNDS, depower_seed, min_turn_radius_request, request_constraints,
    pattern_limits_from, elevation_min_request

@testset verbose = true "opt_request" begin
    @testset "awetrim_depower_to_v3kite" begin
        conv = depower_conversion()
        # At the calibration point: l_dp = 0.6 + 5*u_p plus the calibrated offset, as ever.
        @test awetrim_depower_to_v3kite(conv.pivot) ≈ (conv.pivot - 0.6) / 5 + conv.offset
        # Away from it: slope and curvature about that point.
        @test awetrim_depower_to_v3kite(conv.pivot + 0.2) - awetrim_depower_to_v3kite(conv.pivot) ≈
              0.2 * conv.slope + 0.04 * conv.curvature
        # A trial conversion is in force inside with_depower_conversion only, also after an error.
        plain = merge(conv, (; slope = 0.2, curvature = 0.0))
        @test with_depower_conversion(() -> awetrim_depower_to_v3kite(1.6), plain) ≈ 0.2 + conv.offset
        @test_throws ErrorException with_depower_conversion(() -> error("trial"), plain)
        @test depower_conversion() === conv
        # Beyond the identified range the tangent at its end continues the parabola.
        bent = merge(conv, (; slope = 0.1, curvature = 0.2, l_dp_min = 1.4, l_dp_max = 1.9))
        at(l) = awetrim_depower_to_v3kite(l; conv = bent)
        @test at(2.3) - at(1.9) ≈ 0.4 * (0.1 + 2 * 0.2 * (1.9 - bent.pivot))
        @test at(1.1) - at(1.4) ≈ -0.3 * (0.1 + 2 * 0.2 * (1.4 - bent.pivot))
        @test at(1.8) - at(bent.pivot) ≈ 0.1 * 0.37 + 0.2 * 0.37^2
    end

    @testset "depower_seed" begin
        lo, hi = DEPOWER_SEED_BOUNDS
        tos = (; input_depower = 1.6, input_depower_per_wind = 0.1, input_depower_wind_ref = 7.0,
               input_depower_seed_max = 0.0)
        # One-sided: flat below the reference wind, a ramp above it.
        @test depower_seed(tos, 5.0) == 1.6
        @test depower_seed(tos, 9.0) ≈ 1.8
        # The soft cap clamps short of AWETrim's own bound, silently: it is calibrated.
        capped = merge(tos, (; input_depower_seed_max = 1.7))
        @test (@test_logs depower_seed(capped, 12.0)) == 1.7
        # AWETrim's hard bound clamps too, and warns: the ramp has run out of tape.
        @test (@test_logs (:warn,) depower_seed(tos, 20.0)) == hi
        @test (@test_logs (:warn,) depower_seed(merge(tos, (; input_depower = 0.5)), 5.0)) == lo
    end

    fcs = FC_Settings()
    c1_setpoint = turn_rate_coeffs(fcs.run.body_damping, fcs.course.depower_setpoint).c1

    @testset "min_turn_radius_request" begin
        tos = (; min_feasibility_margin = 0.8)
        # margin / (c1 * max_steering), scaled; the passed c1 wins over the table's.
        @test min_turn_radius_request(fcs, tos; c1 = 0.25) ≈ 0.8 / (0.25 * fcs.course.max_steering)
        @test min_turn_radius_request(fcs, tos; c1 = 0.25, scale = 1.5, margin = 1.0) ≈
              1.5 / (0.25 * fcs.course.max_steering)
        # No c1, or an unusable one: the table's at the setpoint.
        r_table = 0.8 / (c1_setpoint * fcs.course.max_steering)
        @test min_turn_radius_request(fcs, tos) ≈ r_table
        @test min_turn_radius_request(fcs, tos; c1 = NaN) ≈ r_table
        # Margin 0 is where the gate is off: no constraint.
        @test isnothing(min_turn_radius_request(fcs, (; min_feasibility_margin = 0.0)))
        @test_throws ErrorException min_turn_radius_request(fcs, tos; scale = -1.0)
        # Off the table's grid it sends no constraint and warns, as the gate degrades.
        off_grid = deepcopy(fcs)
        off_grid.course.depower_setpoint = 0.9
        @test isnothing(@test_logs (:warn,) min_turn_radius_request(off_grid, tos))
    end

    @testset "request_constraints" begin
        tos = SimpleKiteControllers.TrajOptSettings()
        inflow = (; wind_speed = 8.0)
        l_opt = 150.0
        rc = request_constraints(tos, fcs, inflow, 10.0, l_opt)
        @test rc.turn_radius_reel == turn_radius_lap_reelout(tos, 8.0)
        @test rc.opt_r_scale ≈ (1 + rc.turn_radius_reel / l_opt) * tos.turn_radius_headroom
        # Sized at the seed's depower, converted to V3Kite's: the reply is flown at its own.
        @test rc.depower_request ≈ awetrim_depower_to_v3kite(depower_seed(tos, 8.0))
        @test rc.c1_request == turn_rate_coeffs(fcs.run.body_damping, rc.depower_request).c1
        @test rc.opt_r_min ≈ rc.opt_r_scale * tos.min_feasibility_margin /
                             (rc.c1_request * fcs.course.max_steering)
        @test rc.opt_r_on
        @test rc.opt_r_sent == rc.opt_r_min
        box = pattern_limits_from(tos; elevation_min = elevation_min_request(fcs, tos, l_opt),
                                  wind_speed = 10.0)
        @test isnothing(box) ? isnothing(rc.opt_box) :
              Tuple(getfield(rc.opt_box, f) for f in fieldnames(typeof(box))) ==
              Tuple(getfield(box, f) for f in fieldnames(typeof(box)))

        # Off the grid: no c1, no radius constraint, and the gate's warning.
        off_grid = deepcopy(fcs)
        off_grid.run.body_damping = 100 .* fcs.run.body_damping
        rc = @test_logs (:warn,) match_mode = :any request_constraints(tos, off_grid, inflow, 10.0,
                                                                       l_opt)
        @test isnothing(rc.c1_request)
        @test isnothing(rc.opt_r_min)
        @test !rc.opt_r_on
    end
end
nothing
