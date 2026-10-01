# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the run log (`src/run_log.jl`): `with_run_log` writes the messages it passes on, and
`startup_ladder_report` and `startup_log_lines` read them back. The messages are those of real runs.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: with_run_log, startup_ladder_report, ladder_line, startup_log_lines
using Base.CoreLogging: with_logger
using Test: TestLogger

# The `ladder` case of examples/regression_baseline.jl (Maasvlakte 8.25 m/s, margin 1.3 at headroom
# 0.4, 2026-10-01): retry 3 takes over, the others get a 422, and the run stops at the startup gate.
const LADDER_RUN = """
[ Info: Constraints sent with the request: min_turn_radius 11.85 m (min_feasibility_margin 1.30 x the kite's own)
[ Info: Received the optimized path in 13.56 s
[ Info: The startup path is at margin 0.979, below min_feasibility_margin = 1.30: retry 1/4 (radius correction) at L = 150.0 m, turn radius 7.32 m (was 11.85 m), elevation ceiling 26.8° (the incumbent spans 17.9-28.7° over a 15.5° floor), targeting margin 1.339.
[ Info: POST /step -> HTTP 422: optimization infeasible or not converged: optimization did not converge (IPOPT failure)
[ Info: Startup retry 1 (radius correction) could not converge (HTTP 422) at 7.32 m under a ceiling of 26.8°; the ceiling lever is spent, re-asking the same radius under the last converged ceiling (none).
[ Info: The startup path is at margin 0.979, below min_feasibility_margin = 1.30: retry 2/4 (radius correction) at L = 150.0 m, turn radius 7.32 m (was 11.85 m), no elevation ceiling, targeting margin 1.339.
[ Warning: This exact request failed before (422 from /step, recorded 2026-09-30 23:04:12) and is cached as bad, so it was not sent: v4 at 150.00000 m, guess centred at 30°.
[ Info: Startup retry 2 could not converge (HTTP 422) at 7.32 m; bisecting toward the last converged ask of 11.85 m.
[ Info: The startup path is at margin 0.979, below min_feasibility_margin = 1.30: retry 3/4 (width step) at L = 150.0 m, turn radius 11.85 m (was 11.85 m), no elevation ceiling, azimuth half-width >= 19.9° (the incumbent's is 17.9°), targeting margin 1.339 (the radius lever is spent under the 7.32 m that 422'd).
[ Info: Startup retry 3 measured: margin 1.041, lowest point 45.8 m (floor 40 m), elevation ok, predicted power 18624 W, 0.2 s.
[ Info: Kept retry 3 as the best-so-far; trying again.
[ Info: The startup path is at margin 1.041, below min_feasibility_margin = 1.30: retry 4/4 (ceiling step) at L = 150.0 m, turn radius 11.85 m (was 11.85 m), elevation ceiling 27.0° (the incumbent spans 17.8-28.8° over a 15.5° floor), azimuth half-width >= 19.9° (the incumbent's is 19.8°), targeting margin 1.339.
[ Info: Startup retry 4 (ceiling step) could not converge (HTTP 422) at 11.85 m under a ceiling of 27.0°; the ceiling lever is spent, the radius steps next under the last converged ceiling (none).
[ Info: Saved failed trajectory to /home/ufechner/repos/SimpleKiteControllers.jl/src/../trajectories/startup_incumbent_2026-10-01_0808.yaml (margin 1.0409472655186542).
[ Info: Pattern feasibility at the STARTING length: margin 1.04 — path radius 4.4°, kite 4.3° at L = 150 m, u_s = 0.320. BELOW the demanded min_feasibility_margin = 1.30.
[ Error: the run threw: LoadError: The optimized path asks for a turn radius of 4.4° where the kite manages 4.3°
"""

# A run whose second retry takes over and clears the gates after a first one that is no better
# (made up from the messages of src/startup_path.jl; no run has done this yet).
const TAKES_OVER = """
[ Warning: Startup solve at 30° failed: retry 1/2 from a guess centred at 29° (-1.0°, startup_retry_el_offsets).
[ Warning: Startup path solved from a RETRY seed centred at 29° (-1.0° off guess_el_center): a different optimum.
[ Info: The startup path is at margin 0.700, below min_feasibility_margin = 0.82: retry 1/4 (radius correction) at L = 150.0 m, targeting margin 0.861.
[ Info: Startup retry 1 measured: margin 0.650, lowest point 45.8 m (floor 40 m), elevation ok, predicted power 18000 W, 6.1 s.
[ Info: Startup retry 1 gave margin 0.650, no better than 0.700 — keeping the incumbent.
[ Info: The startup path is at margin 0.700, below min_feasibility_margin = 0.82: retry 2/4 (radius step) at L = 150.0 m, targeting margin 0.861.
[ Info: Startup retry 2 measured: margin 0.900 clearing all gates, lowest point 46.0 m (floor 40 m), elevation ok, predicted power 18500 W, 5.9 s.
[ Info: Startup path clears the gates at margin 0.900 after 2 solves (12.0 s of wall time).
[ Info: Pattern feasibility at the STARTING length: margin 0.90 — path radius 4.4°.
[ Info: step   200 / 13509,   2.36 times realtime, lift/drag [N]: 3802.95/ 619.33
[ Info: Phase-5 fallback at t = 91.1 s
"""

@testset verbose = true "run_log" begin
    @testset "ladder_run" begin
        r = startup_ladder_report(LADDER_RUN)
        @test r.ladder && r.margin_in == 0.979 && r.gate == 1.3
        @test r.attempts == ["radius correction" => "422", "radius correction" => "422",
                             "width step" => "took over, 1.041", "ceiling step" => "422"]
        @test !r.stopped
        @test r.margin_start == 1.04
        @test startswith(r.threw, "LoadError: The optimized path")
        @test r.seed_retries == 0 && isnothing(r.seed_offset)
        @test occursin("ladder at margin 0.979 < 1.3, 4 retries [radius correction: 422", ladder_line(r))
    end

    @testset "a_retry_takes_over" begin
        r = startup_ladder_report(TAKES_OVER)
        @test r.attempts == ["radius correction" => "no better, 0.650",
                             "radius step" => "took over, cleared, 0.900"]
        @test r.seed_retries == 1 && r.seed_offset == -1.0
        @test isnothing(r.threw)
        @test startswith(ladder_line(r), "startup: 1 seed retry, converged at -1.0°; ladder")
    end

    @testset "no_ladder" begin
        r = startup_ladder_report("[ Info: Pattern feasibility at the STARTING length: margin 1.16 — ok\n")
        @test !r.ladder && isempty(r.attempts)
        @test ladder_line(r) == "startup: no ladder; flew margin 1.16"
    end

    @testset "startup_log_lines" begin
        a = startup_log_lines(LADDER_RUN)
        # Cache and server messages dropped, wall times and stamps masked.
        @test !any(l -> occursin("POST", l) || occursin("exact request failed", l), a)
        @test "[ Info: Received the optimized path in <t> s" in a
        @test any(l -> occursin("to trajectories/startup_incumbent_<stamp>.yaml", l), a)
        @test startup_log_lines("[ Info: box SimpleKiteControllers.PatternLimits(28.0)") ==
              ["[ Info: box PatternLimits(28.0)"]
        b = startup_log_lines(TAKES_OVER)
        @test last(b) == "[ Info: Pattern feasibility at the STARTING length: margin 0.90 — path radius 4.4°."
    end

    @testset "with_run_log" begin
        path = joinpath(mktempdir(), "sub", "run.log")
        inner = TestLogger()
        with_logger(inner) do
            @test with_run_log(path) do
                @info "first" x = 1
                @debug "not written"
                @warn "two\nlines"
                42
            end == 42
        end
        @test read(path, String) == "[ Info: first\n│   x = 1\n[ Warning: two\n│ lines\n"
        @test [r.message for r in inner.logs] == ["first", "two\nlines"]   # still passed on
        with_logger(TestLogger()) do
            @test_throws ErrorException with_run_log(() -> error("boom"), path)
        end
        @test read(path, String) == "[ Error: the run threw: boom\n"
    end
end
nothing
