# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for blocks of one step of the reel-out loop (`src/reelout_loop.jl`): the lap counter,
the path blend, the phase-5 fallback, the reel-out release and the compliant hold. Each takes a
hand-made `RunState`, `setup` and `plant`: no model, no optimizer.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: count_laps!, advance_blend!, phase5_fallback!, release_reelout!,
    winch_setpoint!, compliant_hold!

# `rcs.f_high` is written by `count_laps!`; a WCSettings stand-in with only that field.
mutable struct ForceLimits
    f_high::Float64
end

@testset verbose = true "reelout_loop" begin
    fcs0 = FC_Settings()
    up_loops = fcs0.up_loops
    n = 120
    s = range(0, 2pi; length = n + 1)[1:n]
    eight(a, b, c) = prepare_path(a .* sin.(s), c .+ b .* sin.(2 .* s); resample = n, up_loops)
    path_a = eight(20.0, 5.0, 25.0)
    new_fec(path) = (fec = FigureEightController(fcs0; dt = 0.02);
                     set_path!(fec, path...; up_loops);
                     fec)

    @testset "count_laps" begin
        fec = new_fec(path_a)
        np = length(fec.az_path)
        rcs = ForceLimits(7200.0)
        setup = (; fcs = (; first_lap_force_frac = 0.8), fec, rcs, f_high_nominal = 7200.0)
        plant = (; ss = (; elevation = deg2rad(24.0)))
        st = RunState(; n_path = np)
        fec.last_idx = 1
        count_laps!(st, setup, plant, 30.0)
        # The first entry: lap 1, phase 4 starts now, and lap 1 flies under the lowered limit.
        @test st.fig8_n == 1 && st.t_phase4 == 30.0
        @test rcs.f_high == 0.8 * 7200.0 && st.first_lap_f_high_applied
        for k in 1:np
            fec.last_idx = mod1(1 + k, np)   # Q one point on per step, across the wrap
            count_laps!(st, setup, plant, 30.0 + k)
            k == np - 1 && @test st.fig8_n == 1
        end
        @test st.fig8_idx_progress == np
        @test st.fig8_n == 2
        @test rcs.f_high == 7200.0 && !st.first_lap_f_high_applied   # back on lap 2
        # Q slipping back two points at an install does not count a lap down.
        fec.last_idx = mod1(fec.last_idx - 2, np)
        count_laps!(st, setup, plant, 200.0)
        @test st.fig8_n == 2 && st.fig8_idx_progress == np - 2
        @test sum(st.droop_n) == np + 2   # every step lands in one droop bin
    end

    @testset "advance_blend" begin
        path_b = (path_a[1], path_a[2] .+ 2.0)   # 2° higher, aligned point by point
        fec = new_fec(path_a)
        setup = (; tos = (; path_blend_time = 6.0), fcs = (; up_loops), fec)
        st = RunState(; blend_from = path_a, blend_to = path_b, blend_t0 = 10.0,
                      raw_from = path_a, raw_to = path_b)
        advance_blend!(st, setup, 13.0)              # half way
        @test st.raw_el ≈ path_a[2] .+ 1.0
        @test fec.el_path ≈ st.raw_el && fec.az_path ≈ path_a[1]
        @test !isnothing(st.blend_to)
        advance_blend!(st, setup, 16.5)              # past the end: lands on blend_to, then clears
        @test st.raw_el ≈ path_b[2] && fec.el_path ≈ path_b[2]
        @test isnothing(st.blend_from) && isnothing(st.blend_to)
        @test isnothing(st.raw_from) && isnothing(st.raw_to)
        el_before = copy(fec.el_path)
        advance_blend!(st, setup, 20.0)              # no blend: nothing moves
        @test fec.el_path == el_before
    end

    @testset "phase5_fallback" begin
        path_c = eight(16.0, 5.0, 25.0)    # the path in the air: narrower, tighter turns
        record(t, p, margin) = (; t, az = p[1], el = p[2], raw = p, margin, el_applied = 0.0)
        function fallback_case(; margin_now = 0.6, margin_old = 1.8, final_margin_min = 1.5)
            fec = new_fec(path_c)
            st = RunState(; n_path = n, el_applied = 1.0, raw_az = path_c[1], raw_el = path_c[2])
            push!(st.p5_history, record(0.0, path_a, margin_old), record(50.0, path_c, margin_now))
            setup = (; fcs = (; final_margin_min, up_loops),
                     tos = (; blend_fold_margin = 0.5, blend_probe_points = 21), fec)
            # Q right of the path centre, and left of it on the step before: the crossing.
            fec.last_idx = findfirst(>(1.0), fec.az_path)
            st.p5_q_az_prev = -1.0
            return st, setup, fec
        end

        st, setup, fec = fallback_case()
        phase5_fallback!(st, setup, 90.0, 4)         # before the stop latch: not looked at
        @test !st.p5_fallback_done && isnothing(st.blend_to)
        st.stop_start = 85.0
        phase5_fallback!(st, setup, 91.0, 4)         # stop latched: the crossing triggers it
        @test st.p5_fallback_done
        @test st.p5_fallback == (; t = 91.0, from_margin = 0.6, to_margin = 1.8, to_t = 0.0)
        # Blends to the old path with the lift the kite carries now, from the path in the air.
        @test st.blend_to[2] ≈ prepare_path(path_a[1], path_a[2] .+ 1.0; resample = n, up_loops)[2]
        @test st.blend_from[1] ≈ fec.az_path && st.blend_t0 == 91.0
        @test st.raw_to[1] ≈ prepare_path(path_a...; resample = n, up_loops)[1]
        n_before = length(st.p5_history)
        phase5_fallback!(st, setup, 92.0, 5)         # once only
        @test length(st.p5_history) == n_before && st.blend_t0 == 91.0

        st, setup, _ = fallback_case()
        st.p5_q_az_prev = NaN                        # first step looked at: no crossing yet
        phase5_fallback!(st, setup, 91.0, 5)
        @test !st.p5_fallback_done && st.p5_q_az_prev > 0

        st, setup, _ = fallback_case(; margin_now = 1.6)   # the path in the air is good enough
        phase5_fallback!(st, setup, 91.0, 5)
        @test st.p5_fallback_done && isnothing(st.blend_to) && isnothing(st.p5_fallback)

        st, setup, _ = fallback_case(; margin_old = 1.2)   # no earlier path is better
        @test_logs (:warn, r"no earlier install meets it") phase5_fallback!(st, setup, 91.0, 5)
        @test st.p5_fallback_done && isnothing(st.blend_to)

        st, setup, _ = fallback_case(; final_margin_min = 0.0)   # off
        phase5_fallback!(st, setup, 91.0, 5)
        @test !st.p5_fallback_done
    end

    @testset "release_reelout" begin
        setup = (; fcs = (; reelout_delay = 2.0, reelout_f_trigger = 3000.0))
        st = RunState(; transition_start = 10.0)
        release_reelout!(st, setup, (; force = 100.0, ss = nothing), 11.0)
        @test !st.reelout_started
        @test_logs (:info, r"released EARLY") release_reelout!(st, setup, (; force = 3500.0, ss = nothing), 11.5)
        @test st.reelout_started && st.reelout_start_t == 11.5 && st.reelout_trigger_fired
        st = RunState(; transition_start = 10.0)
        release_reelout!(st, setup, (; force = 3500.0, ss = nothing), 12.0)   # timer and force
        @test st.reelout_started && !st.reelout_trigger_fired
    end

    @testset "compliant_hold" begin
        hold = (; gain = 0.5, τF = 1.0, τpos = 5.0)
        setup = (; hold_compliance = hold, rcs = (; kv = 0.04))
        st = RunState(; l_set = 200.0, reelout_started = true, reelout_done = true)
        plant(f) = (; force = f, dt = 0.01, ss = nothing)
        @test winch_setpoint!(st, setup, plant(4000.0), 100.0, 5, 0.3) == 0.0   # starts at rest
        @test st.hold_l0 == 200.0 && st.hold_f_lp == 4000.0
        lp = 4000.0 + 0.01 / hold.τF * 100.0
        v = hold.gain * 0.04 / (2 * sqrt(lp)) * (4100.0 - lp)
        @test compliant_hold!(st, setup, plant(4100.0)) ≈ v      # gives with the force rise
        @test st.l_set ≈ 200.0 + v * 0.01
        st.l_set = 201.0; st.hold_f_lp = 4000.0
        @test compliant_hold!(st, setup, plant(4000.0)) ≈ -1.0 / hold.τpos   # pulled back
        # Phase 4 after the reel-out: no hold, no motion.
        l = st.l_set
        @test winch_setpoint!(st, setup, plant(4000.0), 100.0, 4, 0.3) == 0.0 && st.l_set == l
    end
end
nothing
