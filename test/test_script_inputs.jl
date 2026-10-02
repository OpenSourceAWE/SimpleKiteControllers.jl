# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for `script_inputs.jl`: `run_example` hands its keywords to the one
script it includes, and `script_inputs` merges them over the script's defaults.
"""

using Test
using SimpleKiteControllers

@testset verbose = true "script_inputs" begin
    dir = mktempdir()
    script = joinpath(dir, "echo_inputs.jl")
    # The script returns its inputs, so `run_example` hands them back.
    write(script, "script_inputs(@__FILE__, (; a = 1, b = \"two\"))\n")

    # A plain include runs with the defaults.
    @test Base.include(Main, script) == (; a = 1, b = "two")
    # run_example passes its keywords, merged over the defaults.
    @test run_example(script; b = "three") == (; a = 1, b = "three")
    # Once: the next plain include has the defaults again.
    @test Base.include(Main, script) == (; a = 1, b = "two")
    # A keyword the script does not take is an error (a LoadError, raised inside the
    # include), and is not left for the next run.
    err = try
        run_example(script; c = 3)
    catch e
        e
    end
    @test err isa LoadError && occursin("takes no input c", sprint(showerror, err.error))
    @test Base.include(Main, script) == (; a = 1, b = "two")
    # Another script than the one run_example was called on gets its defaults.
    @test script_inputs(joinpath(dir, "other.jl"), (; a = 5)) == (; a = 5)
    @test_throws ErrorException run_example(joinpath(dir, "missing.jl"))
    # A script that runs another one before reading its own inputs still gets them.
    outer = joinpath(dir, "outer.jl")
    write(outer, "run_example(\"$script\"; a = 7)\nscript_inputs(@__FILE__, (; x = 0))\n")
    @test run_example(outer; x = 2) == (; x = 2)
    @test Base.include(Main, script) == (; a = 1, b = "two")
end
nothing
