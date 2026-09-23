# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the parallel figure-of-eight shape sweep: `OptSettings` loading,
grid generation, the file-lock/claim protocol, the results table's YAML I/O and
its ranking. All file-based tests run against a fresh `mktempdir`, never against
`output/`, and `pattern_margin` is the only geometry entering here — the same
fast, no-simulation computation already exercised in test_fig8_controller.jl.
"""

using Test
using SimpleKiteControllers
# Internal helpers, not part of the public API but part of the on-disk contract.
import SimpleKiteControllers: format_result_entry, _yaml_scalar, _read_claims

@testset verbose = true "optimization" begin

    @testset "OptSettings_defaults_and_yaml" begin
        os = OptSettings()
        @test os.max_processes == 6
        @test os.min_f8_a == 20.0 && os.max_f8_a == 30.0
        @test os.results_file == "optimization_results.yaml"
        @test os.max_pct_time_upper_force == 0.0

        # The package's own shipped sweep definition.
        os2 = OptSettings("optimization.yaml")
        @test os2.max_processes == 12
        @test os2.min_f8_a == 19.0
        @test os2.max_pct_time_within_2pct_of_peak == 5.0

        mktempdir() do dir
            bad = joinpath(dir, "bad.yaml")
            write(bad, "optimization:\n  not_a_field: 1\n")
            @test_throws ErrorException OptSettings(bad)

            zero_step = joinpath(dir, "zero_step.yaml")
            write(zero_step, "optimization:\n  step_size: 0.0\n")
            @test_throws ErrorException OptSettings(zero_step)

            zero_proc = joinpath(dir, "zero_proc.yaml")
            write(zero_proc, "optimization:\n  max_processes: 0\n")
            @test_throws ErrorException OptSettings(zero_proc)
        end
    end

    @testset "opt_grid" begin
        os = OptSettings()
        os.min_f8_a = 20.0; os.max_f8_a = 22.0
        os.min_f8_b = 8.0; os.max_f8_b = 8.4
        os.step_size = 1.0
        grid = opt_grid(os)
        # f8_b only ever lands on 8.0 at this step size -> f8_a alone drives length.
        @test length(grid) == 3
        @test all(t -> t.f8_b == 8.0, grid)
        @test [t.f8_a for t in grid] == [20.0, 21.0, 22.0]

        # Upper bound included when it lands exactly on the grid.
        os2 = OptSettings()
        os2.min_f8_a = 0.0; os2.max_f8_a = 2.0; os2.step_size = 1.0
        os2.min_f8_b = 0.0; os2.max_f8_b = 0.0
        @test [t.f8_a for t in opt_grid(os2)] == [0.0, 1.0, 2.0]

        # Upper bound dropped when the step overshoots it.
        os3 = OptSettings()
        os3.min_f8_a = 8.0; os3.max_f8_a = 12.0; os3.step_size = 1.5
        os3.min_f8_b = 0.0; os3.max_f8_b = 0.0
        @test unique(t.f8_a for t in opt_grid(os3)) == [8.0, 9.5, 11.0]

        # f8_b is the inner (fastest-varying) axis.
        os4 = OptSettings()
        os4.min_f8_a = 0.0; os4.max_f8_a = 1.0; os4.step_size = 1.0
        os4.min_f8_b = 0.0; os4.max_f8_b = 1.0
        @test opt_grid(os4) == [(f8_a = 0.0, f8_b = 0.0), (f8_a = 0.0, f8_b = 1.0),
                                 (f8_a = 1.0, f8_b = 0.0), (f8_a = 1.0, f8_b = 1.0)]
    end

    @testset "task_key" begin
        @test task_key(24.0, 10.0) == "a=24.000_b=10.000"
        @test task_key(24, 10) == "a=24.000_b=10.000"     # integer input works too
        @test task_key(19.00049, 8.0) == "a=19.000_b=8.000"   # rounds, doesn't truncate oddly
    end

    @testset "pattern_margin_and_filter_grid" begin
        fcs = FC_Settings()
        # A larger pattern is always less tightly curved (same invariant as
        # path_min_radius in test_fig8_controller.jl), regardless of the exact
        # numbers, which depend on fcs and are not pinned here.
        m_small = pattern_margin(fcs, 5.0, 2.0, 150.0)
        m_big = pattern_margin(fcs, 30.0, 12.0, 150.0)
        @test m_big > m_small
        @test m_small > 0 && m_big > 0

        grid = [(f8_a = 5.0, f8_b = 2.0), (f8_a = 30.0, f8_b = 12.0)]
        margins = [pattern_margin(fcs, t.f8_a, t.f8_b, 150.0) for t in grid]
        kept, dropped = filter_grid(grid, fcs, 150.0; min_margin = 1.0)
        @test kept == grid[margins .>= 1.0]
        @test length(dropped) == count(<(1.0), margins)
        for (t, m) in dropped
            @test m < 1.0
            @test m == pattern_margin(fcs, t.f8_a, t.f8_b, 150.0)
        end

        # min_margin = 0 keeps everything, whatever the shape.
        kept0, dropped0 = filter_grid(grid, fcs, 150.0; min_margin = 0.0)
        @test kept0 == grid
        @test isempty(dropped0)
    end

    @testset "with_file_lock" begin
        mktempdir() do dir
            lockp = joinpath(dir, "x.lock")

            @test with_file_lock(() -> 42, lockp) == 42
            @test !isdir(lockp)

            # Released even when the protected function throws.
            @test_throws ErrorException with_file_lock(lockp) do
                error("boom")
            end
            @test !isdir(lockp)

            # A held, non-stale lock makes a waiter time out rather than proceed.
            mkdir(lockp)
            try
                @test_throws ErrorException with_file_lock(() -> 1, lockp;
                                                            timeout = 0.2,
                                                            stale_after = 1e6,
                                                            poll = 0.02)
            finally
                isdir(lockp) && rm(lockp; recursive = true, force = true)
            end

            # A lock far older than stale_after is broken (with a warning) instead
            # of blocking the sweep forever behind a dead worker.
            mkdir(lockp)
            run(`touch -d 1970-01-01 $lockp`)
            ran = Ref(false)
            @test_logs (:warn,) match_mode = :any with_file_lock(lockp;
                                                                 stale_after = 1.0) do
                ran[] = true
            end
            @test ran[]
            @test !isdir(lockp)
        end
    end

    @testset "init_results_file_and_load_results" begin
        mktempdir() do dir
            rp = joinpath(dir, "results.yaml")
            @test load_results(rp) == Dict{String, Any}[]   # missing file

            init_results_file(rp)
            @test isfile(rp)
            @test occursin("results:", read(rp, String))
            @test load_results(rp) == Dict{String, Any}[]   # header only, no entries

            # An existing file is left untouched — that's what makes a sweep resumable.
            open(rp, "a") do io
                print(io, format_result_entry(["f8_a" => 1.0, "f8_b" => 2.0]))
            end
            before = read(rp, String)
            init_results_file(rp)
            @test read(rp, String) == before

            # A bare list (no "results:" header) is still read, not just written.
            rp2 = joinpath(dir, "bare.yaml")
            write(rp2, "- f8_a: 3.0\n  f8_b: 4.0\n")
            r = load_results(rp2)
            @test length(r) == 1
            @test r[1]["f8_a"] == 3.0
        end
    end

    @testset "record_result_and_load_results_round_trip" begin
        mktempdir() do dir
            rp = joinpath(dir, "results.yaml")
            missing_summary = joinpath(dir, "nope.yaml")
            record_result!(rp, run_metrics(missing_summary, 20.0, 8.0; status = "skipped"))
            record_result!(rp, run_metrics(missing_summary, 21.0, 9.0; worker = 2))

            results = load_results(rp)
            @test length(results) == 2
            @test results[1]["f8_a"] == 20.0 && results[1]["status"] == "skipped"
            @test results[1]["mean_power_W"] === nothing
            @test results[2]["f8_a"] == 21.0 && results[2]["worker"] == 2
        end
    end

    @testset "_yaml_scalar_and_format_result_entry" begin
        @test _yaml_scalar("hi") == "\"hi\""
        @test _yaml_scalar("a\\b\"c\nd\r") == "\"a\\\\b\\\"c\\nd\""   # escaped, not kept
        @test _yaml_scalar(true) == "true"
        @test _yaml_scalar(nothing) == "null"
        @test _yaml_scalar(1.5) == "1.5"
        @test _yaml_scalar(NaN) == ".nan"
        @test _yaml_scalar(3) == "3"

        s = format_result_entry(["a" => 1.0, "b" => "x", "c" => nothing])
        lines = split(s, '\n'; keepempty = false)
        @test lines[1] == "  - a: 1.0"
        @test lines[2] == "    b: \"x\""
        @test lines[3] == "    c: null"
    end

    @testset "run_metrics" begin
        mktempdir() do dir
            sp = joinpath(dir, "summary.yaml")
            write(sp, """
            reelout:
              power:
                mean_W: 1234.5
                energy_run_kJ: 12.0
                peak_W: 3000.0
              winch_state:
                upper_force_pct: 2.0
                lower_force_pct: 1.0
              force:
                mean_N: 500.0
                peak_N: 900.0
            fig8_metrics:
              steering:
                pct_time_within_2pct_of_peak: 3.0
                peak_abs_u_s: 0.3
              cross_track_deg:
                rms: 1.1
              laps: 5.5
              elevation_deg:
                min_whole_run: 25.0
              success_criteria: 8
            performance:
              sim_time: 60.0
              wall_time: 12.0
            """)
            d = Dict(run_metrics(sp, 24.0, 10.0; worker = 3))
            @test d["f8_a"] == 24.0 && d["f8_b"] == 10.0
            @test d["status"] == "ok" && d["worker"] == 3
            @test d["mean_power_W"] == 1234.5
            @test d["pct_time_upper_force"] == 2.0
            @test d["pct_time_lower_force"] == 1.0
            @test d["laps"] == 5.5
            @test d["min_elevation_deg"] == 25.0

            # A missing summary file: the given fields survive, everything else is
            # `null` rather than an error, so an aborted run is recorded, not lost.
            d2 = Dict(run_metrics(joinpath(dir, "missing.yaml"), 1.0, 2.0;
                                  status = "crashed"))
            @test d2["status"] == "crashed"
            @test d2["mean_power_W"] === nothing
            @test d2["laps"] === nothing

            # A partial summary (run aborted before reeling out): present branches
            # read through, an absent one (here fig8_metrics) is null, not a KeyError.
            sp2 = joinpath(dir, "partial.yaml")
            write(sp2, "reelout:\n  power:\n    mean_W: 10.0\n")
            d3 = Dict(run_metrics(sp2, 1.0, 2.0))
            @test d3["mean_power_W"] == 10.0
            @test d3["laps"] === nothing
            @test d3["pct_time_upper_force"] === nothing
        end
    end

    @testset "side_conditions" begin
        os = OptSettings()
        ok = Dict{String, Any}("status" => "ok", "mean_power_W" => 100.0,
                               "pct_time_upper_force" => 0.0,
                               "pct_time_within_2pct_of_peak" => 0.0)
        @test isempty(side_conditions(ok, os))

        has(r, needle) = any(s -> occursin(needle, s), side_conditions(r, os))
        @test has(merge(ok, Dict("status" => "crashed")), "run status")
        @test has(merge(ok, Dict("mean_power_W" => nothing)), "no reel-out power")
        @test has(merge(ok, Dict("pct_time_upper_force" => nothing)),
                  "winch state not measured")
        @test has(merge(ok, Dict("pct_time_upper_force" => 5.0)),
                  "upper force controller engaged")
        @test has(merge(ok, Dict("pct_time_within_2pct_of_peak" => nothing)),
                  "steering saturation not measured")
        @test has(merge(ok, Dict("pct_time_within_2pct_of_peak" => 10.0)),
                  "steering within")

        # Failures accumulate, checked in the order the docstring promises.
        reasons = side_conditions(Dict{String, Any}("status" => "crashed"), os)
        @test length(reasons) == 4
        @test occursin("run status", reasons[1])
    end

    @testset "unique_results" begin
        results = [
            Dict("f8_a" => 20.0, "f8_b" => 8.0, "mean_power_W" => 100.0),
            Dict("f8_a" => 21.0, "f8_b" => 8.0, "mean_power_W" => 200.0),
            Dict("f8_a" => 20.0, "f8_b" => 8.0, "mean_power_W" => 150.0),  # re-flown
        ]
        u = unique_results(results)
        @test length(u) == 2
        @test only(r for r in u if r["f8_a"] == 20.0)["mean_power_W"] == 150.0   # last wins
    end

    @testset "rank_results" begin
        os = OptSettings()
        results = [
            Dict{String, Any}("f8_a" => 20.0, "f8_b" => 8.0, "status" => "ok",
                              "mean_power_W" => 300.0, "pct_time_upper_force" => 0.0,
                              "pct_time_within_2pct_of_peak" => 0.0),
            Dict{String, Any}("f8_a" => 21.0, "f8_b" => 8.0, "status" => "ok",
                              "mean_power_W" => 500.0, "pct_time_upper_force" => 0.0,
                              "pct_time_within_2pct_of_peak" => 0.0),
            Dict{String, Any}("f8_a" => 22.0, "f8_b" => 8.0, "status" => "crashed"),
        ]
        accepted, rejected = rank_results(results, os)
        @test length(accepted) == 2 && length(rejected) == 1
        @test accepted[1]["f8_a"] == 21.0 && accepted[2]["f8_a"] == 20.0   # power, high first
        @test rejected[1][1]["f8_a"] == 22.0
        @test occursin("run status", rejected[1][2][1])
    end

    @testset "format_results_table" begin
        os = OptSettings()
        results = [
            Dict{String, Any}("f8_a" => 20.0, "f8_b" => 8.0, "status" => "ok",
                              "mean_power_W" => 300.0, "pct_time_upper_force" => 0.0,
                              "pct_time_within_2pct_of_peak" => 0.0, "laps" => 5.5),
            Dict{String, Any}("f8_a" => 18.0, "f8_b" => 8.0,
                              "status" => "skipped: below margin",
                              "feasibility_margin" => 0.5),
            Dict{String, Any}("f8_a" => 22.0, "f8_b" => 8.0, "status" => "crashed"),
        ]
        s = format_results_table(results, os)
        @test occursin("RANKED by mean reel-out power", s)
        @test occursin("1 of 3", s)   # one accepted out of three unique results
        @test occursin("REJECTED, flown (1)", s)
        @test occursin("run status: crashed", s)
        @test occursin("NOT FLOWN (1)", s)
        @test occursin("margin 0.50", s)

        # No accepted runs: the table says so instead of printing an empty list.
        s_empty = format_results_table(results[2:3], os)
        @test occursin("none — every run broke a side condition", s_empty)
    end

    @testset "claim_protocol" begin
        mktempdir() do dir
            rp = joinpath(dir, "results.yaml")
            grid = [(f8_a = 20.0, f8_b = 8.0), (f8_a = 21.0, f8_b = 8.0),
                    (f8_a = 22.0, f8_b = 8.0)]

            @test n_unclaimed(grid, rp) == 3
            t1 = claim_task!(grid, rp; worker = 1)
            @test t1 == grid[1]
            @test n_unclaimed(grid, rp) == 2
            t2 = claim_task!(grid, rp; worker = 2)
            @test t2 == grid[2]
            # The same point is never handed out twice while still claimed.
            @test claim_task!([t1], rp; worker = 9) === nothing

            # A finished run drops out of "unclaimed" through the results table,
            # not through the claim file.
            record_result!(rp, run_metrics(joinpath(dir, "x.yaml"), t1.f8_a, t1.f8_b))
            t3 = claim_task!(grid, rp; worker = 1)
            @test t3 == grid[3]
            @test claim_task!(grid, rp; worker = 3) === nothing   # nothing left
            @test n_unclaimed(grid, rp) == 0

            # release_claims! only releases the named worker's UNFINISHED claims.
            n_released = release_claims!(rp, 2)
            @test n_released == 1
            @test n_unclaimed(grid, rp) == 1
            @test claim_task!(grid, rp; worker = 4) == grid[2]

            # reset_claims! keeps only combinations that actually have a result.
            n_orphaned = reset_claims!(grid, rp)
            @test n_orphaned == 2   # grid[2] (worker 4) and grid[3] (worker 1)
            claimed_keys = Set(c.k for c in _read_claims(rp * ".claims"))
            @test claimed_keys == Set([task_key(t1.f8_a, t1.f8_b)])
        end
    end
end
