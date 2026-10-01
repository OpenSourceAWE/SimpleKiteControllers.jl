# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for blocks of one step of the reel-out loop (`src/reelout_loop.jl`): the lap counter,
the path blend, the in-air lift, the phase-5 fallback, the lift target, the depower, the steering
hooks, the reel-out release, speed and stop, the entry force guard and the compliant hold. Each takes a hand-made `RunState`, `setup` and `plant`: no model, no
optimizer. The winch controllers are the real ones, from `build_controllers` on a stand-in plant.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: count_laps!, advance_blend!, phase5_fallback!, release_reelout!,
    winch_setpoint!, compliant_hold!, reelout_speed!, entry_force_guard!, build_controllers,
    WCSettings, deliver_lift_in_air!, update_lift_target!, depower_command!, steering_hooks!

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

    @testset "deliver_lift_in_air" begin
        tos = (; min_feasibility_margin = 0.82, blend_fold_margin = 0.5, blend_probe_points = 21)
        function lift_case(; c1 = NaN, margin_min = 0.82)
            fec = new_fec(path_a)
            setup = (; tos = merge(tos, (; min_feasibility_margin = margin_min)),
                     fcs = (; up_loops, max_steering = fcs0.max_steering), fec,
                     feas = (; c1), c1_at_phase = (phase, st) -> c1)
            st = RunState(; chk_points = 60, fig8_n = 3)
            return st, setup, fec
        end
        plant = (; ss = (; l_tether = [200.0]))

        st, setup, fec = lift_case()                 # no turn-rate law: no margin to fail
        deliver_lift_in_air!(st, setup, plant, 50.0, 4, 1.0)
        @test st.blend_from == (fec.az_path, fec.el_path)
        @test st.blend_to[1] == fec.az_path && st.blend_to[2] ≈ fec.el_path .+ 1.0
        @test st.blend_t0 == 50.0 && st.el_applied == 1.0
        @test st.el_shift_events[end].status == "blended in"
        # Once per lap and target: the same lap again queues nothing, the next lap does.
        st.blend_to = nothing; st.el_applied = 0.0
        deliver_lift_in_air!(st, setup, plant, 51.0, 4, 1.0)
        @test isnothing(st.blend_to)
        st.fig8_n = 4
        deliver_lift_in_air!(st, setup, plant, 52.0, 4, 1.0)
        @test !isnothing(st.blend_to) && st.el_applied == 1.0
        # Nothing while a re-optimization is pending, or when the path already carries the lift.
        st, setup, _ = lift_case()
        st.reopt_pending = true
        deliver_lift_in_air!(st, setup, plant, 50.0, 4, 1.0)
        @test isnothing(st.blend_to) && isempty(st.el_shift_events)
        st, setup, _ = lift_case()
        st.el_applied = 1.0
        deliver_lift_in_air!(st, setup, plant, 50.0, 4, 1.0)
        @test isnothing(st.blend_to)

        # No rung clears the gate: held back, warned once, retried on a later lap.
        st, setup, _ = lift_case(; c1 = 0.28, margin_min = 1e6)
        @test_logs (:warn, r"held back") deliver_lift_in_air!(st, setup, plant, 50.0, 4, 1.0)
        @test isnothing(st.blend_to) && st.el_applied == 0.0
        @test st.el_shift_events[end].status == "held back" && st.el_shift_warned
        st.fig8_n = 4
        @test_logs deliver_lift_in_air!(st, setup, plant, 60.0, 4, 1.0)   # no second warning
        @test length(st.el_shift_events) == 1

        # A shift too big for the gate is rationed to the first rung that clears it. Raising the
        # path compresses its azimuth by cos(elevation), so its turns tighten as it goes up.
        st, setup, fec = lift_case(; c1 = 0.28, margin_min = 1.0)
        margin_at(fm) = check_pattern_feasible(
            prepare_path(fec.az_path, fec.el_path .+ fm * 40.0; resample = 60, up_loops)...,
            200.0, fcs0.max_steering; c1 = 0.28, prn = false).margin
        rungs = (1.0, 0.75, 0.5, 0.25)
        fm = rungs[findfirst(r -> margin_at(r) >= 1.0, rungs)]
        @test margin_at(1.0) < 1.0 && 0.25 <= fm < 1          # the case needs a rationed rung
        el0 = copy(fec.el_path)
        @test_logs (:info, r"rationed to fit the curvature gate") deliver_lift_in_air!(
            st, setup, plant, 50.0, 4, 40.0)
        @test st.el_applied == fm * 40.0 && st.blend_to[2] ≈ el0 .+ fm * 40.0
        @test st.el_shift_events[end].status == "blended in ($(round(Int, 100fm)) %)"
        @test st.el_shift_events[end].margin == margin_at(fm)
        # A shift of a hundredth of a degree or less is no delivery, and nothing held it back.
        st, setup, _ = lift_case()
        @test_logs deliver_lift_in_air!(st, setup, plant, 50.0, 4, 0.005)
        @test isnothing(st.blend_to) && st.el_applied == 0.0 && isempty(st.el_shift_events)
        @test !st.el_shift_warned
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

    @testset "update_lift_target" begin
        setup = (; fcs = (; el_offset_final = 1.0, el_offset_lead = 4.0, reelout_l_max = 380.0))
        plant = (; ss = (; v_reelout = [3.0]))
        st = RunState(; l_set = 370.0)
        @test update_lift_target!(st, setup, plant, 80.0, 3) == 0.0   # phase 4 on only
        st.l_set = 300.0
        @test update_lift_target!(st, setup, plant, 81.0, 4) == 0.0   # 80 m left: 27 s away
        st.l_set = 370.0                                              # 10 m left: within 4 s
        @test (@test_logs (:info, r"Elevation lift") update_lift_target!(st, setup, plant, 82.0, 4)) == 1.0
        @test st.lift_on && st.lift_t == 82.0 && st.lift_remaining == 10.0
        @test update_lift_target!(st, setup, plant, 90.0, 4) == 1.0 && st.lift_t == 82.0   # latched
    end

    @testset "depower_command" begin
        lim = (; depower_final = 0.35, depower_final_max = 0.35, depower_final_f_gain = 1e-5,
               depower_final_f_gain_stop = 4e-5, depower_final_f_target = 6000.0)
        cc() = CourseController(CourseControllerSettings(; dt = 0.01))
        plant(force = 5000.0) = (; force, dt = 0.01, ss = (; v_app = 25.0, v_reelout = [0.0]))
        # Under fly_opt_depower the 2 -> 3 hand-over ramps from the entry's depower to the optimizer's.
        setup = (; tos = (; fly_opt_depower = true, path_blend_time = 6.0), fcs = lim)
        st = RunState(; cc = cc(), depower_flown_opt = 0.28)
        @test depower_command!(st, setup, plant(), 10.0, 2, 3, 0.25) == (0.25, 3)
        @test depower_command!(st, setup, plant(), 13.0, 3, 3, 0.30)[1] ≈ 0.265
        @test depower_command!(st, setup, plant(), 16.0, 3, 4, 0.30)[1] ≈ 0.28
        @test isnothing(st.depower_blend_to)
        @test depower_command!(st, setup, plant(), 20.0, 4, 4, 0.30)[1] == 0.28
        # The reel-out done: phase 5 the same step, at depower_final.
        setup = (; tos = (; fly_opt_depower = false), fcs = lim)
        st = RunState(; cc = cc(), reelout_done = true)
        @test depower_command!(st, setup, plant(), 30.0, 4, 4, 0.27) == (0.35, 5)
        @test st.cc.phase == 5 && st.final_start == 30.0
        # The soft-stop ramps toward depower_final from the depower it latched at.
        st = RunState(; cc = cc(), stop_start = 100.0, stop_T = 4.0, stop_dp_entry = 0.28)
        dp, phase = depower_command!(st, setup, plant(), 102.0, 4, 4, 0.27)
        @test dp ≈ 0.28 + 0.07 / 2 && phase == 4
        # The force limiter from phase 5 on: depower above depower_final, at most depower_final_max.
        setup = (; tos = (; fly_opt_depower = false), fcs = merge(lim, (; depower_final_max = 0.42)))
        st = RunState(; cc = cc(), final_start = 0.0)
        dp, _ = depower_command!(st, setup, plant(8000.0), 30.0, 5, 5, 0.3)
        @test st.dp_final_extra ≈ 1e-5 * 2000.0 * 0.01 && dp ≈ 0.35 + st.dp_final_extra
        @test st.dp_final_extra_peak == st.dp_final_extra
        st.dp_final_extra = 0.1
        @test depower_command!(st, setup, plant(8000.0), 30.0, 5, 5, 0.3)[1] == 0.42   # capped
    end

    @testset "steering_hooks" begin
        hooks = (; steer_disturbance = nothing, extra_steer_delay = 0, hook_settle = 5.0,
                 steer_gain_feedback_only = false, steer_gain_factor = 1.0,
                 fcs = (; max_steering = 0.32))
        st = RunState()
        @test steering_hooks!(st, hooks, 1.0, 0.2, 0.0) == 0.2       # no hook set
        setup = merge(hooks, (; steer_disturbance = t -> 0.01))
        @test steering_hooks!(st, setup, 1.0, 0.2, 0.0) ≈ 0.21
        @test st.dist_t == [1.0] && st.dist_d == [0.01] && st.dist_u ≈ [0.21]
        # The gain factor, from hook_settle after phase 4, clamped to max_steering.
        setup = merge(hooks, (; steer_gain_factor = 1.1))
        st = RunState(; t_phase4 = 10.0)
        @test steering_hooks!(st, setup, 14.0, 0.2, 0.0) == 0.2       # still settling
        @test steering_hooks!(st, setup, 15.0, 0.2, 0.0) ≈ 0.22
        @test steering_hooks!(st, setup, 16.0, 0.3, 0.0) == 0.32
        # On the feedback part only: the feed-forward passes unscaled.
        setup = merge(hooks, (; steer_gain_factor = 1.5, steer_gain_feedback_only = true))
        @test steering_hooks!(RunState(; t_phase4 = 0.0), setup, 10.0, 0.2, 0.1) ≈ 0.25
        # A delay of two steps, once the buffer is full.
        setup = merge(hooks, (; extra_steer_delay = 2, hook_settle = 0.0))
        st = RunState(; t_phase4 = 0.0)
        @test [steering_hooks!(st, setup, 1.0, u, 0.0) for u in (0.1, 0.2, 0.3, 0.4, 0.5)] ≈
              [0.1, 0.2, 0.1, 0.2, 0.3]
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

    # The reel-out controller and the entry guard as a run builds them, at dt = 0.01 s.
    function winch(fcs)
        rcs = WCSettings(; dt = 0.01)
        c = build_controllers(FC_Settings(), rcs, (; dt = 0.01, sys_state = (; l_tether = [150.0])))
        return (; fcs, rc = c.rc, rcs, guard_lfc = c.guard_lfc)
    end
    wplant(v_reel, force) = (; v_reel, force, dt = 0.01, ss = nothing)

    @testset "reelout_speed_stops_at_the_length" begin
        setup = winch((; reelout_l_max = 160.0, reelout_softstop = 2.0, n_fig_eight = 0,
                       reelout_softstart = 0.0))
        st = RunState(; l_set = 150.0, reelout_started = true, reelout_start_t = 0.0)
        v, t, l_latch = Float64[], 0.0, NaN
        while !st.reelout_done && t < 60
            l_before = st.l_set
            push!(v, reelout_speed!(st, setup, wplant(isempty(v) ? 0.0 : v[end], 2000.0), t, 0.3))
            isnan(l_latch) && !isnan(st.stop_start) && (l_latch = l_before)
            t += 0.01
        end
        @test st.reelout_done && st.stop_reason == "length"
        @test st.l_set == 160.0                       # capped, never past reelout_l_max
        @test all(>=(0), v) && sum(v) * 0.01 >= 10.0 - 1e-9  # the last step is cut at the limit
        # The soft-stop latched once `reelout_softstop` seconds at the entry speed covered the rest,
        # and plans a linear ramp to zero over twice the time that rest takes at that speed.
        i = findfirst(==(st.stop_v_entry), v)
        @test !isnothing(i) && 160.0 - l_latch <= st.stop_v_entry * 2.0
        @test st.stop_T ≈ 2 * (160.0 - l_latch) / st.stop_v_entry
        @test all(diff(v[i:end]) .<= 0)               # decelerates from there on
    end

    @testset "reelout_speed_stops_after_the_laps" begin
        fcs = (; reelout_l_max = 400.0, reelout_softstop = 2.0, n_fig_eight = 2, reelout_softstart = 0.0)
        setup = winch(fcs)
        st = RunState(; l_set = 150.0, reelout_started = true, reelout_start_t = 0.0, n_path = 100,
                      fig8_idx_progress = 200.0)      # two complete laps flown
        reelout_speed!(st, setup, wplant(0.0, 2000.0), 10.0, 0.3)
        @test st.stop_reason == "laps" && st.stop_start == 10.0 && st.stop_T == 2 * 2.0
        @test !st.reelout_done
        t = 10.0
        while !st.reelout_done && t < 30
            t += 0.01
            reelout_speed!(st, setup, wplant(0.0, 2000.0), t, 0.3)
        end
        @test st.reelout_done
        @test t - st.stop_start ≈ st.stop_T atol = 0.011   # the ramp has run out
        @test st.l_set < 400.0
        # Without a soft-stop the laps end the reel-out at once.
        setup0 = winch(merge(fcs, (; reelout_softstop = 0.0)))
        st0 = RunState(; l_set = 150.0, reelout_started = true, reelout_start_t = 0.0, n_path = 100,
                       fig8_idx_progress = 200.0)
        reelout_speed!(st0, setup0, wplant(0.0, 2000.0), 10.0, 0.3)
        @test st0.reelout_done && st0.stop_reason == "laps" && isnan(st0.stop_start)
    end

    @testset "entry_force_guard" begin
        setup = merge(winch((;)), (; fcs = (; entry_f_min = 350.0)))
        st = RunState(; l_set = 150.0)
        # Force above the floor: inactive, the setpoint and the length are left alone.
        @test all(_ -> entry_force_guard!(st, setup, wplant(0.0, 2000.0), 0.0) == 0.0, 1:50)
        @test st.l_set == 150.0 && !setup.guard_lfc.active
        # A force sag: the guard reels IN, never out, and the length follows it.
        v = [entry_force_guard!(st, setup, wplant(0.0, 100.0), 0.0) for _ in 1:200]
        @test setup.guard_lfc.active && all(<=(0), v) && minimum(v) < 0
        @test st.l_set ≈ 150.0 + sum(v) * 0.01
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
