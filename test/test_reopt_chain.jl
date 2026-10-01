# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the re-optimization cycle of the reel-out loop (`src/reelout_loop.jl`):
`reoptimize!` queues a request on a lap boundary, polls for the reply, and gates and installs it
(`request_reopt!`, `collect_reopt!`, `gate_and_install!`, `evaluate_candidate!`,
`install_candidate!`). The optimizer is an `OptChain` in replay mode, which serves recorded replies
in order without a server; the requests are warm and non-blocking, the cold retries of a rejected
reply are not reached (`blend_max_retries = 0`).
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: reoptimize!, OptChain, InflowConditions, WinchParams,
    TrajOptSettings, awetrim_depower_to_v3kite

@testset verbose = true "reopt_chain" begin
    fcs = FC_Settings()
    up_loops = fcs.up_loops
    n = 120
    s = range(0, 2pi; length = n + 1)[1:n]
    eight(a, b, c; m = n) = (r = range(0, 2pi; length = m + 1)[1:m];
                             prepare_path(a .* sin.(r), c .+ b .* sin.(2 .* r); resample = m, up_loops))
    flown = eight(20.0, 5.0, 25.0)
    l_tether = 200.0

    # A reply as `/trajectory` serves it: a 100-point curve in RADIANS, its power and depower.
    function entry(; power = 18000.0, status = "converged", a = 21.0)
        az, el = eight(a, 5.0, 25.5; m = 100)
        return Dict{String, Any}("key" => "c2-$power-$a", "status" => status,
            "length_m" => l_tether, "when" => "2026-10-01",
            "table" => Dict{String, Any}(
                "table" => Dict{String, Any}("azimuth" => deg2rad.(az), "elevation" => deg2rad.(el),
                                             "distance_radial" => [l_tether, l_tether + 30]),
                "metrics" => Dict{String, Any}("avg_power_W" => power, "turn_radius_min_m" => 14.0),
                "optimized_parameters" => Dict{String, Any}("input_depower" => 1.42),
                "spline" => Dict{String, Any}("downloops" => !up_loops)),
            "reply" => Dict{String, Any}(
                "trajectory" => Dict{String, Any}("azimuth" => az, "elevation" => el),
                "state" => status, "step_index" => 1,
                "metrics" => Dict{String, Any}("energy_J" => 1e5, "total_time_s" => 100.0,
                                               "avg_power_W" => power)))
    end

    # Phase 4, one lap flown on the startup path, one reply waiting in the replay.
    function cycle(replies; max_reopt = 3)
        tos = TrajOptSettings()
        tos.use_step = true; tos.reopt_blocking = false; tos.reopt_poll_interval = 0.5
        tos.reopt_every_n_laps = 1; tos.max_reopt = max_reopt; tos.blend_max_retries = 0
        tos.optimize_k_v = false; tos.fly_opt_depower = false
        fec = FigureEightController(fcs; dt = 0.02)
        set_path!(fec, flown...; up_loops)
        oc = OptChain("http://127.0.0.1:1"; dir = mktempdir(), replay = replies)
        oc.config = SimpleKiteControllers.InitParams(; name = tos.name, length = l_tether,
            winch_params = WinchParams("reelout", 0.04, 700.0, 7200.0),
            inflow_conditions = InflowConditions(; wind_speed = 8.0, wind_direction = 270.0,
                                                 profile_law = 3),
            trajectory = SimpleKiteControllers.Trajectory(flown...))
        setup = (; tos, fcs, fec, opt_chain = oc, opt_r_on = true,
                 c1_at_phase = (phase, x) -> 0.28, cap_wind = 8.0, el_center_seed = 30.0,
                 winch_reopt = oc.config.winch_params, inflow = oc.config.inflow_conditions,
                 feas = (; c1 = NaN), el_floor = 13.0, wing_lift = (az, el) -> zero(az),
                 power_gate_off = pred -> false, margin5 = Phase5MarginState(),
                 phase5_margin_at = (az, el) -> 1.7, opt_depower_log = NamedTuple[],
                 project_set = (; v_wind = 8.0), wc = nothing, rc = nothing,
                 opt_kv_log = NamedTuple[])
        st = RunState(; n_path = length(fec.az_path), fig8_idx_progress = length(fec.az_path),
                      opt_paths_raw = [flown], opt_paths_at = [(0.0, 0)], opt_power_pred = 18000.0,
                      raw_az = flown[1], raw_el = flown[2], depower_flown = 0.27,
                      opt_r_scale = 1.1, pred_timeline = [(t = 0.0, power = 18000.0)])
        return st, setup
    end
    plant = (; ss = (; l_tether = [l_tether]))

    @testset "request_collect_install" begin
        st, setup = cycle([entry(), entry(; a = 21.5)])
        @test_logs (:info, r"Re-optimizing for L = 200 m") match_mode = :any reoptimize!(
            st, setup, plant, 30.0, 4, 0.0)
        @test st.reopt_pending && st.reopt_t_request == 30.0 && st.reopt_lap == 1.0
        @test st.reopt_next_poll == 30.5 && st.reopt_n == 0
        # Sized for this length: the turn radius under the law the reply is judged with, the box.
        @test st.opt_r_min ≈ min_turn_radius_request(fcs, setup.tos; scale = 1.1, c1 = 0.28)
        @test !isnothing(st.opt_box_now)
        reoptimize!(st, setup, plant, 30.2, 4, 0.0)          # before the poll: nothing
        @test st.reopt_pending && st.reopt_n == 0
        reoptimize!(st, setup, plant, 30.5, 4, 0.0)          # polled: converged, gated, installed
        @test !st.reopt_pending && st.reopt_n == 1
        ev = st.reopt_events[end]
        @test ev.status == "installed" && occursin("18000 W predicted", ev.detail)
        @test length(st.reopt_cycles) == 1 && st.reopt_cycles[end].status == "installed"
        # The install: a blend queued from the path in the air, the raw reply recorded, the
        # prediction, the phase-5 record, the depower to fly, and the checks at the reply's resolution.
        @test !isnothing(st.blend_to) && st.blend_t0 == 30.5
        @test length(st.opt_paths_raw) == 2 && st.opt_paths_at[end] == (30.5, 4)
        @test st.pred_timeline[end] == (t = 30.5, power = 18000.0)
        @test st.p5_history[end].margin == 1.7 && st.p5_history[end].t == 30.5
        @test st.depower_flown_opt == awetrim_depower_to_v3kite(1.42)
        @test st.depower_blend_from == 0.27 && st.depower_blend_to == st.depower_flown_opt
        @test length(setup.opt_depower_log) == 1 && st.chk_points == 99
        @test setup.fec.az_path ≈ st.blend_from[1]           # the aligned OLD path, at w = 0
        # No new request while that blend runs, even a lap later.
        st.fig8_idx_progress += 2st.n_path
        reoptimize!(st, setup, plant, 40.0, 4, 0.0)
        @test !st.reopt_pending
        st.blend_to = nothing                                # once it has run: the next one
        @test_logs (:info, r"Re-optimizing") match_mode = :any reoptimize!(st, setup, plant, 41.0, 4, 0.0)
        @test st.reopt_pending
    end

    @testset "max_reopt" begin
        st, setup = cycle([entry()]; max_reopt = 1)
        st.reopt_n = 1
        reoptimize!(st, setup, plant, 30.0, 4, 0.0)
        @test !st.reopt_pending && isempty(st.reopt_events)
    end

    @testset "rejected_on_power" begin
        st, setup = cycle([entry(; power = 100.0)])          # far below min_power_frac
        reoptimize!(st, setup, plant, 30.0, 4, 0.0)
        reoptimize!(st, setup, plant, 30.5, 4, 0.0)
        ev = st.reopt_events[end]
        @test ev.status == "rejected" && occursin("100 W predicted", ev.detail)
        @test isnothing(st.blend_to) && length(st.opt_paths_raw) == 1   # nothing flown
        @test length(st.pred_timeline) == 1 && st.reopt_n == 1
    end

    @testset "failed_solve" begin
        st, setup = cycle([entry(; status = "failed")])
        reoptimize!(st, setup, plant, 30.0, 4, 0.0)
        reoptimize!(st, setup, plant, 30.5, 4, 0.0)
        @test st.reopt_events[end].status == "failed" && st.reopt_n == 1
        @test isnothing(st.blend_to) && length(st.opt_paths_raw) == 1
    end

    @testset "request_failed" begin
        st, setup = cycle(Dict{String, Any}[])                # nothing left to serve: a 422
        @test_logs (:warn, r"flying on with the current path") match_mode = :any reoptimize!(
            st, setup, plant, 30.0, 4, 0.0)
        @test !st.reopt_pending && st.reopt_n == 1
        @test st.reopt_events[end].status == "request failed"
        @test st.reopt_cycles[end].status == "request failed"
    end
end
nothing
