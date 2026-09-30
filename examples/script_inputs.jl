# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Caller inputs of the example scripts, passed as keywords instead of globals.

    include("examples/script_inputs.jl")
    run_example("simple_opt_reelout.jl"; show_plots = false,
                tos_overrides = Dict{Symbol, Any}(:reopt_enabled => false))

`include`s the script with those inputs; the script reads them at its top with
`script_inputs(@__FILE__, defaults)`. A plain `include("examples/simple_opt_reelout.jl")`
runs with the defaults, so nothing a caller passed can survive into the next run,
and a keyword the script does not take is an error, not a silently ignored global.
"""

if !isdefined(@__MODULE__, :SCRIPT_INPUTS)
    "What `run_example` hands the script it includes: `(; file, inputs)`, or `nothing` between runs."
    const SCRIPT_INPUTS = Ref{Any}(nothing)
end

"""
    run_example(file; inputs...)

`include` the example script `file` (relative to `examples/`, or absolute) with the
keyword `inputs`, see the file's docstring. Returns what the `include` returns.
"""
function run_example(file::AbstractString; inputs...)
    path = abspath(isabspath(file) ? file : joinpath(@__DIR__, file))
    isfile(path) || error("run_example: no script $path")
    SCRIPT_INPUTS[] = (; file = path, inputs = NamedTuple(inputs))
    try
        return Base.include(@__MODULE__, path)
    finally
        SCRIPT_INPUTS[] = nothing
    end
end

"""
    script_inputs(file, defaults::NamedTuple) -> NamedTuple

The inputs `run_example` passed to the script `file` (call it with `@__FILE__`), merged
over `defaults`, which name every input the script takes. Only the script `run_example`
was called on gets them, once: another script it includes gets its own defaults.
"""
function script_inputs(file::AbstractString, defaults::NamedTuple)
    given = SCRIPT_INPUTS[]
    (isnothing(given) || given.file != abspath(file)) && return defaults
    SCRIPT_INPUTS[] = nothing
    unknown = [k for k in keys(given.inputs) if !haskey(defaults, k)]
    isempty(unknown) ||
        error("$(basename(file)) takes no input $(join(unknown, ", ")); \
               its inputs are $(join(keys(defaults), ", ")).")
    return merge(defaults, given.inputs)
end
