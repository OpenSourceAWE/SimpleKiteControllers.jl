# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the run-summary sections the two reel-out scripts share
(`run_summary.jl`), on hand-built metrics and a synthetic log.
"""

using Test
using SimpleKiteControllers
using Statistics: mean
import Dates

# Every field `fig8_metrics_block` reads, as `print_fig8_metrics` returns them.
fake_fig8m(; failed = String[]) = (; criteria = 10, criteria_failed = failed,
    stats_start = 20.04, settle_time_used = 5.0, laps = 6.5, rms_d = 1.234, mean_d = 0.5,
    max_d = 3.0, min_elevation_settled = 21.0, min_elevation_all = 18.0, max_turn_rate = 55.4,
    az_reach_neg = 30.0, az_reach_pos = 31.0, az_fill_neg = 0.95, az_fill_pos = 0.97,
    az_reach_neg_worst = 28.0, az_reach_pos_worst = 29.0, el_span = 12.0, el_span_lap = 8.0,
    el_fill = 0.9, mean_force = 5000.4, std_force = 400.2, cv_force = 0.08,
    max_steering_used = 0.3, steering_sat_frac = 0.02, steering_hf_std = 0.001,
    turnrate_hf_std = 0.5, max_steering_delivered = 0.29, tape_rate_frac = 0.01,
    max_tape_rate = 0.2, v_steering = 0.2)

@testset verbose = true "run_summary" begin

    @testset "success_verdict and package_git_state" begin
        @test success_verdict(nothing)[1] == "not scored — no settled samples"
        @test success_verdict(fake_fig8m())[1] == "all 10 passed"
        @test success_verdict(fake_fig8m(; failed = ["a", "b"]))[1] == "FAILED: a, b"
        git = package_git_state()
        @test git.hash isa AbstractString && !isempty(git.hash)
        @test git.status in ("clean", "dirty", "unknown")
    end

    @testset "simulation_block" begin
        t = Dates.DateTime(2026, 9, 30, 14, 5, 6)
        b = simulation_block("run.jl", "system_x.yaml", 0.0, 8.25, t)
        @test collect(keys(b)) == ["script", "project", "rel_turbulence", "wind_speed", "time",
                                   "date", "hostname", "git_hash", "git_status"]
        @test b["time"][1] == "14:05:06" && b["date"][1] == "2026-09-30"
        @test b["wind_speed"][1] == 8.25
    end

    @testset "fig8_metrics_block" begin
        none = (; t_start = Float64[], dt = Float64[])
        b = fig8_metrics_block(fake_fig8m(), none)
        @test b["lap_time"][1] == "none"
        @test b["cross_track_deg"]["rms"] == (1.23, "RMS cross-track error [deg]")
        @test !haskey(b, "success_criteria")
        laps = (; t_start = [10.0, 22.0, 33.0], dt = [12.0, 11.0, 13.0])
        b = fig8_metrics_block(fake_fig8m(), laps; cross_track_ref = "the path",
                               nested_verdict = true)
        @test b["lap_time"]["fastest"][1] == 11.0 && b["lap_time"]["slowest"][1] == 13.0
        @test occursin("lap 2, starting at t = 22.0 s", b["lap_time"]["fastest"][2])
        @test b["cross_track_deg"]["rms"][2] == "RMS cross-track error vs the path [deg]"
        @test b["success_criteria"][1] == "all 10 passed"
        @test last(collect(keys(b))) == "success_criteria"
    end

    @testset "reelout_block" begin
        # Phase 3 for 20 samples, 4 for 70, 5 for 11; reel-out from sample 21 to 80.
        n = 101
        tt = collect(0.0:0.1:10.0)
        l_set = [i <= 20 ? 100.0 : (i <= 80 ? 100.0 + (i - 20) : 160.0) for i in 1:n]
        phase = [i <= 20 ? 3 : (i <= 90 ? 4 : 5) for i in 1:n]
        force = [i <= 20 ? 100.0 : 1000.0 + i for i in 1:n]
        vro = [21 <= i <= 80 ? 2.0 : 0.0 for i in 1:n]
        sl = (; time = tt, sys_state = Int16.(phase), v_app = fill(Float32(20.0), n),
              winch_force = [Float32[force[i], 0, 0, 0] for i in 1:n],
              v_reelout = [Float32[vro[i], 0, 0, 0] for i in 1:n],
              depower = fill(Float32(0.3), n), var_10 = Float32.(l_set),
              var_12 = fill(Float32(1), n))
        fcs = FC_Settings(; v_app_ref = 20.0, reelout_l_max = 160.0)
        r = reelout_block(sl, fcs, 100.0; stop_reason = "", laps_reeled = 3.456)
        @test r.block["tether"]["stop_reason"][1] == "none"
        @test r.block["tether"]["laps_reeled"][1] == 3.46
        @test r.block["v_app_phase4"]["mean_m_s"][1] == 20.0
        @test collect(keys(r.block["force"])) == ["cf_force_ro"]
        @test r.rp.n == 60
        i4 = findall(==(4), phase)
        @test r.p4.power.av ≈ mean(Float32.(force[i4]) .* Float32.(vro[i4]))
        @test r.p4.depower_av ≈ 0.3f0
        # With the window's means, which run_metrics reads, first.
        rm = reelout_block(sl, fcs, 100.0; stop_reason = "length", laps_reeled = 3.0,
                           window_means = true)
        @test collect(keys(rm.block["force"])) == ["mean_N", "peak_N", "cf_force_ro"]
        @test first(collect(keys(rm.block["power"]))) == "mean_W"
        @test rm.block["power"]["mean_W"][1] == round(Int, rm.rp.mean_power)
        # Phase 4 never reached: no apparent-wind section and no phase-4 figures.
        sl3 = merge(sl, (; sys_state = fill(Int16(3), n)))
        r3 = @test_logs (:warn, r"Phase 4 never reached") match_mode = :any reelout_block(
            sl3, fcs, 100.0; stop_reason = "", laps_reeled = 0.0)
        @test isnothing(r3.p4) && !haskey(r3.block, "v_app_phase4")
    end

    @testset "performance_block" begin
        @test (@test_logs (:warn, r"No simulated time") performance_block(0.0, 1.0, 0.05, 1)) ===
              nothing
        b = performance_block(100.0, 50.0, 0.05, 2)
        @test b["realtime_factor"] == (2.0, "sim_time / wall_time")
        @test b["steps"][1] == 2000 && b["ms_per_step"][1] == 25.0
        # Frozen time excluded from the rates, and the extra entries after wall_time.
        b = performance_block(100.0, 60.0, 0.05, 2; blocked_s = 10.0,
                              extra = Pair{String, Any}["x" => (1, "c")])
        @test collect(keys(b))[1:4] == ["sim_time", "wall_time", "x", "realtime_factor"]
        @test b["realtime_factor"][1] == 2.0
        @test occursin("excluding time frozen", b["realtime_factor"][2])
    end

    @testset "opt_cycle_max" begin
        @test opt_cycle_max(NamedTuple[], 7.0).s == 7.0
        cycles = [(t = 100.0, l = 200.0, status = "installed", wall_s = 9.0),
                  (t = 150.0, l = 250.0, status = "rejected", wall_s = 12.0)]
        c = opt_cycle_max(cycles, 7.0)
        @test c.s == 12.0 && occursin("t = 150.0 s (rejected", c.comment)
        c = opt_cycle_max(cycles, 20.0)
        @test c.s == 20.0 && occursin("slowest re-optimization took 12.0 s", c.comment)
    end
end
