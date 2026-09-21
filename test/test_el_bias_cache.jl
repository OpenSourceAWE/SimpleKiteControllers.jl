# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the elevation-bias memory: the per-inflow-condition cache that
seeds the first laps of a reel-out run. File I/O on a temporary file only.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: YAML

@testset "el_bias_cache" begin
    @testset "key" begin
        # A path and a bare name agree; the extension never enters the key.
        @test el_bias_key("system_reelout_cabauw.yaml", 5.8) == "system_reelout_cabauw@5.80"
        @test el_bias_key("/some/where/system_reelout_cabauw.yaml", 5.8) ==
              el_bias_key("system_reelout_cabauw", 5.8)
        # Wind speeds differ at a hundredth; the same speed written differently agrees.
        @test el_bias_key("p.yaml", 6.25) != el_bias_key("p.yaml", 6.2)
        @test el_bias_key("p.yaml", 6.0) == el_bias_key("p.yaml", 6)
    end

    mktempdir() do dir
        file = joinpath(dir, "cache", "el_bias_cache.yaml")

        @testset "empty cache" begin
            @test isnothing(el_bias_seed("p.yaml", 6.0, 5; file))
            @test_throws ArgumentError el_bias_seed("p.yaml", 6.0, 0; file)
        end

        @testset "record and read back" begin
            profile = [0.98, 1.46, 2.18, 2.57, 1.65]
            record_el_bias_seed!("p.yaml", 6.0, profile; laps = 2, file, log = "run_a")
            @test isfile(file)
            @test el_bias_seed("p.yaml", 6.0, 5; file) ≈ profile atol = 1e-3
            # Another speed or project is another condition, but with one entry
            # every other condition is seeded from it: nearest for the same project,
            # the mean over projects for another one.
            @test isnothing(el_bias_seed("p.yaml", 6.5, 5; file, fallback = false))
            @test isnothing(el_bias_seed("q.yaml", 6.0, 5; file, fallback = false))
            @test el_bias_seed("p.yaml", 6.5, 5; file) ≈ profile atol = 1e-3
            @test el_bias_seed_info("p.yaml", 6.5, 5; file).source == "nearest"
            @test el_bias_seed_info("p.yaml", 6.0, 5; file).source == "exact"
            q = el_bias_seed_info("q.yaml", 6.0, 5; file)
            @test q.source == "mean"
            @test q.profile ≈ profile atol = 1e-3
            @test q.keys == [el_bias_key("p.yaml", 6.0)]
            # A different number of bands gets the mean as a rigid shift.
            rigid = el_bias_seed("p.yaml", 6.0, 3; file)
            @test length(rigid) == 3
            @test all(rigid .≈ sum(profile) / length(profile))
            @test el_bias_seed("p.yaml", 6.0, 1; file) ≈ [sum(profile) / length(profile)] atol = 1e-3
            # What was stored, in plain text.
            entry = YAML.load_file(file)["entries"][el_bias_key("p.yaml", 6.0)]
            @test entry["laps"] == 2
            @test entry["runs"] == 1
            @test entry["bins"] == 5
            @test entry["log"] == "run_a"
            @test entry["previous_profile_deg"] == "none"
            @test entry["project"] == "p"
            @test entry["wind_speed"] == 6.0
        end

        @testset "overwrite keeps the history" begin
            record_el_bias_seed!("p.yaml", 6.0, [1.0, 1.5, 2.0, 2.5, 1.5]; laps = 1, file)
            record_el_bias_seed!("p.yaml", 7.0, [0.5]; laps = 2, file)
            entries = YAML.load_file(file)["entries"]
            @test length(entries) == 2
            e = entries[el_bias_key("p.yaml", 6.0)]
            @test e["runs"] == 2
            @test e["laps"] == 1
            @test Float64.(e["previous_profile_deg"]) ≈ [0.98, 1.46, 2.18, 2.57, 1.65] atol = 1e-3
            @test el_bias_seed("p.yaml", 6.0, 5; file) ≈ [1.0, 1.5, 2.0, 2.5, 1.5]
            @test el_bias_seed("p.yaml", 7.0, 5; file) ≈ fill(0.5, 5)
        end

        @testset "neighbour fallback" begin
            # 6.0 and 7.0 m/s are stored: in between is interpolated (the 1-bin
            # entry as a rigid shift), beyond the ends is the nearest, and another
            # project gets the mean of everything.
            mid = el_bias_seed_info("p.yaml", 6.25, 5; file)
            @test mid.source == "interpolated"
            @test mid.keys == [el_bias_key("p.yaml", 6.0), el_bias_key("p.yaml", 7.0)]
            @test mid.profile ≈ 0.75 .* [1.0, 1.5, 2.0, 2.5, 1.5] .+ 0.25 .* fill(0.5, 5)
            @test el_bias_seed_info("p.yaml", 5.0, 5; file).source == "nearest"
            @test el_bias_seed("p.yaml", 5.0, 5; file) ≈ [1.0, 1.5, 2.0, 2.5, 1.5]
            @test el_bias_seed("p.yaml", 9.0, 5; file) ≈ fill(0.5, 5)
            q = el_bias_seed_info("q.yaml", 6.0, 5; file)
            @test q.source == "mean"
            @test length(q.keys) == 2
            @test q.profile ≈ 0.5 .* ([1.0, 1.5, 2.0, 2.5, 1.5] .+ fill(0.5, 5))
            @test_throws ArgumentError record_el_bias_seed!("p.yaml", 6.0, Float64[]; laps = 1, file)
        end

        @testset "unreadable or malformed" begin
            # An unreadable file is an empty cache, never a stopped run.
            broken = joinpath(dir, "broken.yaml")
            write(broken, "entries: [not, a, dict\n")
            @test isnothing(@test_logs (:warn, r"unreadable") el_bias_seed("p.yaml", 6.0, 5; file = broken))
            # A file without the entries map is empty too.
            write(broken, "other: 1\n")
            @test isnothing(el_bias_seed("p.yaml", 6.0, 5; file = broken))
            # An entry without a profile is skipped, with a warning.
            write(broken, "entries:\n  \"$(el_bias_key("p.yaml", 6.0))\":\n    laps: 2\n")
            @test isnothing(@test_logs (:warn, r"malformed") el_bias_seed("p.yaml", 6.0, 5; file = broken))
            # Recording over it repairs it.
            record_el_bias_seed!("p.yaml", 6.0, [2.0]; laps = 2, file = broken)
            @test el_bias_seed("p.yaml", 6.0, 1; file = broken) ≈ [2.0]
        end
    end
end
