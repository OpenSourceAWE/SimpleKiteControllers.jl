# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the step-wise decisions of the reel-out loop (`src/loop_decisions.jl`).
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: loop_gain_scale, feedforward_step, blended_depower, stop_depower,
    final_force_extra, lift_should_start, lap_index_step, reelout_release, reelout_command,
    soft_stop_speed

@testset verbose = true "loop_decisions" begin
    fcs = (; depower_final = 0.35, depower_final_max = 0.42, depower_final_f_gain = 1e-5,
           depower_final_f_gain_stop = 4e-5, depower_final_f_target = 6000.0,
           el_offset_lead = 4.0, reelout_l_max = 380.0, reelout_delay = 2.0,
           reelout_f_trigger = 3000.0, reelout_softstart = 4.0)

    @testset "loop_gain_scale" begin
        c1(dp) = 0.3 - 0.5 * dp                          # falls with the depower
        @test loop_gain_scale(NaN, 0.3, Inf, c1) == 1.0  # setpoint's c1 unknown
        @test loop_gain_scale(0.2, 0.2, Inf, c1) ≈ 0.2 / c1(0.2)
        # Rounded to three digits for the memo, and capped where the table ends.
        @test loop_gain_scale(0.2, 0.2004, Inf, c1) == loop_gain_scale(0.2, 0.2, Inf, c1)
        @test loop_gain_scale(0.2, 0.9, 0.3, c1) ≈ 0.2 / c1(0.3)
        @test loop_gain_scale(0.2, 0.7, Inf, c1) == 1.0  # c1 <= 0 there
        @test loop_gain_scale(0.2, 0.2, Inf, dp -> NaN) == 1.0
    end

    @testset "feedforward_off_before_phase_4_or_without_gain" begin
        f = (; ff_gain = 0.7, v_app_min = 10.0, ff_lead_time = 0.35, ff_smooth = 6.0,
             ff_d_fade = 6.0, ff_err_fade = 60.0, ff_tau = 0.2)
        r = feedforward_step(f, 0.011, nothing, 3, 0.0, 25.0, 200.0, 30.0, 0.0, 0.25, 1.0, 0.1, 0.2)
        @test r == (0.0, 0.0, 0.1, 0.2)                  # phase 3: nothing, filters untouched
        f0 = merge(f, (; ff_gain = 0.0))
        @test feedforward_step(f0, 0.011, nothing, 5, 0.0, 25.0, 200.0, 30.0, 0.0, 0.25, 1.0, 0.1, 0.2) ==
              (0.0, 0.0, 0.1, 0.2)
        # Unknown turn-rate coefficient, or a kite that does not move: also nothing.
        @test feedforward_step(f, 0.011, nothing, 5, 0.0, 25.0, 200.0, 30.0, 0.0, NaN, 1.0, 0.1, 0.2)[1:2] == (0.0, 0.0)
        @test feedforward_step(f, 0.011, nothing, 5, 0.0, 25.0, 200.0, 0.0, 0.0, 0.25, 1.0, 0.1, 0.2)[1:2] == (0.0, 0.0)
    end

    @testset "depower_ramps" begin
        @test blended_depower(0.3, nothing, 0.0, 5.0, 2.0, 0.27) == (1.0, 0.27)
        @test blended_depower(0.3, 0.2, 10.0, 11.0, 2.0, 0.27) == (0.5, 0.25)
        @test blended_depower(0.3, 0.2, 10.0, 99.0, 2.0, 0.27) == (1.0, 0.2)
        @test stop_depower(fcs, 0.3, 10.0, 4.0, 10.0) == 0.3
        @test stop_depower(fcs, 0.3, 10.0, 4.0, 12.0) ≈ 0.325
        @test stop_depower(fcs, 0.3, 10.0, 4.0, 20.0) ≈ 0.35
        # Never below the depower the stop latched at, even if depower_final is lower.
        @test stop_depower(fcs, 0.4, 10.0, 4.0, 20.0) == 0.4
    end

    @testset "final_force_limiter" begin
        # Below the target force it winds down to zero, above it winds up, both within the limits.
        @test final_force_extra(fcs, 0.0, 1000.0, 20.0, 0.0, false, 0.011) == 0.0
        up = final_force_extra(fcs, 0.0, 8000.0, 20.0, 0.0, false, 0.011)
        @test 0 < up < fcs.depower_final_max - fcs.depower_final
        # The force the stopped drum will see is higher than the one now, by the wind ratio squared.
        @test final_force_extra(fcs, 0.0, 6000.0, 20.0, 5.0, false, 0.011) > 0.0
        @test final_force_extra(fcs, 0.0, 6000.0, 20.0, -5.0, false, 0.011) == 0.0   # reel-in ignored
        # Faster while the stop ramps, and capped.
        @test final_force_extra(fcs, 0.0, 8000.0, 20.0, 0.0, true, 0.011) > up
        @test final_force_extra(fcs, 0.07, 1e6, 20.0, 0.0, true, 1.0) ≈ 0.07
        @test final_force_extra(fcs, 0.0, 8000.0, 0.0, 0.0, false, 0.011) ≈ up   # no wind: no scaling
    end

    @testset "lift_and_laps" begin
        @test lift_should_start(fcs, 3.0, 4, 0.0, 100.0)             # the stop latched
        @test lift_should_start(fcs, NaN, 5, 0.0, 100.0)             # phase 5
        @test !lift_should_start(fcs, NaN, 4, 3.0, 300.0)            # 80 m left, 12 m of lead
        @test lift_should_start(fcs, NaN, 4, 3.0, 370.0)             # 10 m left <= 12 m
        @test !lift_should_start(fcs, NaN, 4, 0.0, 379.0)            # not reeling out
        # Q moving forward, across the wrap, backward, and jumping to the other branch.
        @test lap_index_step(11, 10, 100) == 1
        @test lap_index_step(1, 99, 100) == 2                        # 99 -> 1 across the wrap
        @test lap_index_step(99, 1, 100) == -2
        @test lap_index_step(11, 10, 100) + lap_index_step(60, 11, 100) == 1 + 0   # jump: no progress
    end

    @testset "reelout_gate_and_command" begin
        @test reelout_release(fcs, 12.0, 10.0, 100.0) == (true, false)   # the delay has passed
        @test reelout_release(fcs, 10.5, 10.0, 3500.0) == (false, true)  # early, by force
        @test reelout_release(fcs, 10.5, 10.0, 100.0) == (false, false)
        # Soft start ramps the command; a loaded tether releases it in proportion to the force.
        @test reelout_command(fcs, 2.0, 10.0, 10.0, 500.0, 500.0, 7000.0) == 0.0
        @test reelout_command(fcs, 2.0, 12.0, 10.0, 500.0, 500.0, 7000.0) ≈ 1.0
        @test reelout_command(fcs, 2.0, 10.0, 10.0, 3750.0, 500.0, 7000.0) ≈ 0.5 * 2.0
        @test reelout_command(merge(fcs, (; reelout_softstart = 0.0)), 2.0, 10.0, 10.0, 0.0, 500.0, 7000.0) == 2.0
        @test soft_stop_speed(2.0, 10.0, 10.0, 4.0) == 2.0
        @test soft_stop_speed(2.0, 12.0, 10.0, 4.0) ≈ 1.0
        @test soft_stop_speed(2.0, 15.0, 10.0, 4.0) == 0.0
    end
end
