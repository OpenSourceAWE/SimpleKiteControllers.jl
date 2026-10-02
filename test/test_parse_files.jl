# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Every Julia file in `src/`, `examples/` and `test/` parses. The examples are not run by the
test suite, so without this a syntax error in one of them shows up only at the end of a
simulation that includes it.
"""

using Test

@testset "all files parse" begin
    root = dirname(@__DIR__)
    files = [joinpath(dir, file) for folder in ("src", "examples", "test")
             for (dir, _, names) in walkdir(joinpath(root, folder))
             for file in names if endswith(file, ".jl")]
    @test !isempty(files)
    for file in files
        ex = Meta.parseall(read(file, String); filename = file)
        errors = [e for e in ex.args if e isa Expr && e.head in (:error, :incomplete)]
        @test isempty(errors) || (@error "Parse error in $(relpath(file, root))" errors[1]; false)
    end
end
nothing
