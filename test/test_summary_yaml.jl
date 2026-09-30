# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the commented run-summary writer in `summary_yaml.jl`:
`write_yaml_commented` and `time_keyed`.
"""

using Test
using SimpleKiteControllers

@testset verbose = true "summary_yaml" begin

    @testset "write_yaml_commented" begin
        # One-entry Dicts keep the order deterministic without OrderedCollections.
        node = Dict("run" => Dict("wind" => (8.25, "mean wind [m/s]")))
        io = IOBuffer()
        write_yaml_commented(io, 0, node)
        @test String(take!(io)) == "run:\n  wind: 8.25" * " "^(36 - 12) * "# mean wind [m/s]\n"
        # Strings are quoted; a leaf without a comment gets none.
        write_yaml_commented(io, 1, Dict("status" => "ok"))
        @test String(take!(io)) == "  status: \"ok\"\n"
        # A line longer than the comment column keeps one space before the comment.
        long = "k"^40
        write_yaml_commented(io, 0, Dict(long => (1, "c")))
        @test String(take!(io)) == "$long: 1 # c\n"
    end

    @testset "time_keyed" begin
        rows = [(t = 1.0, v = "a"), (t = 12.34, v = "b"), (t = 1.0, v = "c"), (t = 1.0, v = "d")]
        pairs = time_keyed(e -> (e.t, e.v), rows)
        @test first.(pairs) == ["t_001.0_s", "t_012.3_s", "t_001.0_s_2", "t_001.0_s_3"]
        @test last.(pairs) == ["a", "b", "c", "d"]
        @test isempty(time_keyed(e -> (e.t, e.v), NamedTuple[]))
    end
end
