# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the decisions of the startup retry ladder (`src/startup_retry.jl`): which lever
each attempt pulls and what the ladder learns from an answer. Pure numbers, no optimizer.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: RetryLadder, next_lever, record_422!, record_converged!,
    azimuth_amplitude, RETRY_GAIN_MAX, startup_seed_offsets, STARTUP_RETRY_STEP, STARTUP_RETRY_SLACK

@testset verbose = true "startup_retry" begin
    tos = (; min_feasibility_margin = 1.0)
    # An incumbent 10° wide and 4° tall, top at 22°, with a box floor at 10°: room to lower the ceiling.
    az = 10 .* sin.(range(0, 2pi; length = 100))
    el = 20 .+ 2 .* sin.(range(0, 4pi; length = 100))
    radius_for(target) = 10 * target
    ask(L; box_el_min = 10.0, floor = 12.0, r_sent = 8.0) =
        next_lever(L, tos, az, el, r_sent, box_el_min, floor, radius_for)

    @testset "amplitude_measure" begin
        # A sine of amplitude A has RMS A/sqrt(2), so the server's half-width measure is A.
        @test azimuth_amplitude(az) ≈ 10 rtol = 1e-2   # the sampled sine repeats its endpoint
        @test azimuth_amplitude(fill(3.0, 10)) == 0.0
    end

    @testset "first_attempt_corrects_the_radius" begin
        L = RetryLadder(; m_reply = 0.8)
        a = ask(L)
        @test a.lever == "radius correction"
        @test a.target == max(STARTUP_RETRY_STEP * 0.8, STARTUP_RETRY_SLACK)   # the slack floor wins
        @test a.r_ask == radius_for(a.target)
        @test a.prev_ask == 8.0                              # the radius the startup solve was sent
        @test a.el_cap ≈ maximum(el) - 2.0                   # room to lower the ceiling: taken at once
        @test a.az_min === nothing
    end

    @testset "no_ceiling_room_keeps_the_ceiling" begin
        L = RetryLadder(; m_reply = 0.8)
        # Box floor so high the incumbent barely fits: no room.
        a = ask(L; box_el_min = maximum(el) - (maximum(el) - minimum(el)) - 0.4)
        @test a.lever == "radius correction"
        @test a.el_cap === nothing
    end

    @testset "levers_in_order_after_a_converged_ask" begin
        L = RetryLadder(; m_reply = 0.8)
        a1 = ask(L)
        record_converged!(L, a1, 0.85)
        @test (L.r_asked, L.m_reply, L.cap_ok) == (a1.r_ask, 0.85, a1.el_cap)
        a2 = ask(L)
        @test a2.lever == "ceiling step" && a2.r_ask == a1.r_ask
        @test a2.el_cap < a1.el_cap
        # A ceiling that 422s is never sent again; the width comes next, then the radius.
        @test record_422!(L, a2) == :ceiling
        @test L.relax_cap && L.cap_bad == a2.el_cap
        a3 = ask(L)
        @test a3.lever == "width step"
        @test a3.az_min ≈ azimuth_amplitude(az) + 2.0
        @test a3.el_cap == L.cap_ok
        @test record_422!(L, a3) == :width
        @test L.relax_width && L.width_bad == a3.az_min
        a4 = ask(L)
        @test a4.lever == "radius step"
        @test a4.r_ask ≈ a1.r_ask * clamp(a4.target / 0.85, 1.0, RETRY_GAIN_MAX)
    end

    @testset "radius_step_is_capped_per_solve" begin
        L = RetryLadder(; m_reply = 0.3, r_asked = 8.0, relax_cap = true, relax_width = true)
        a = ask(L)
        @test a.lever == "radius step"
        @test a.r_ask ≈ 8.0 * RETRY_GAIN_MAX                 # target/measured is far above the cap
    end

    @testset "bisection_and_the_end_of_the_ladder" begin
        L = RetryLadder(; m_reply = 0.9, r_asked = 8.0, relax_cap = true, relax_width = true)
        a = ask(L)
        @test record_422!(L, a) == :radius
        @test L.bisect_hi == a.r_ask
        # 0.9 * bisect_hi / 8 still reaches the gate of 1.0: bisect between the two.
        b = ask(L)
        @test b.lever == "radius bisection"
        @test b.r_ask ≈ (8.0 + L.bisect_hi) / 2
        # A converged reply clears the bound.
        record_converged!(L, b, 0.95)
        @test isnan(L.bisect_hi) && !L.relax_cap && !L.relax_width
        # A 422 so close to the last converged ask that bisecting cannot reach the gate: stop.
        L2 = RetryLadder(; m_reply = 0.9, r_asked = 8.0, bisect_hi = 8.5, relax_cap = true,
                         relax_width = true)
        c = ask(L2)
        @test c.lever === nothing
        @test !c.bisect_room
    end

    @testset "422_kinds" begin
        L = RetryLadder(; m_reply = 0.8)
        # Only the radius moved: no ceiling or width in the ask differs from the converged ones.
        @test record_422!(L, (; el_cap = L.cap_ok, az_min = L.width_ok, r_ask = 9.0)) == :radius
        @test L.bisect_hi == 9.0
        # The highest ceiling that failed is kept.
        L = RetryLadder(; m_reply = 0.8, cap_ok = 20.0)
        record_422!(L, (; el_cap = 18.0, az_min = nothing, r_ask = 8.0))
        record_422!(L, (; el_cap = 19.0, az_min = nothing, r_ask = 8.0))
        @test L.cap_bad == 19.0
    end

    @testset "startup_seed_offsets" begin
        # No listed retries: the shipped guess only, and no walked-out tail.
        @test startup_seed_offsets(Float64[]) == [0.0]
        # The listed seeds first, in order, then the whole degrees they miss, negative first.
        @test startup_seed_offsets([2.0, -1.0]; max_abs = 3.0) ==
              [0.0, 2.0, -1.0, 1.0, -2.0, -3.0, 3.0]
        offsets = startup_seed_offsets([4.0])
        @test length(offsets) == length(unique(offsets))
        @test maximum(abs, offsets) == 10.0
    end
end
nothing
