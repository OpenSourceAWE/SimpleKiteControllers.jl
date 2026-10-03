# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Caller inputs of the example scripts, passed as keywords instead of globals.
#
#     run_example("simple_opt_reelout.jl"; show_plots = false,
#                 tos_overrides = Dict{Symbol, Any}(:max_reopt => 0))
#
# `include`s the script with those inputs; the script reads them at its top with
# `script_inputs(@__FILE__, defaults)`. A plain `include("examples/simple_opt_reelout.jl")`
# runs with the defaults, so nothing a caller passed can survive into the next run,
# and a keyword the script does not take is an error, not a silently ignored global.

"What `run_example` hands the script it includes: `(; file, inputs)`, or `nothing` between runs."
const SCRIPT_INPUTS = Ref{Any}(nothing)

"""
    run_example(file; inputs...)

`include` the example script `file` (relative to the package's `examples/`, or
absolute) into `Main` with the keyword `inputs`, which the script reads with
[`script_inputs`](@ref); a plain `include` of the same script runs with its
defaults. Returns what the `include` returns. A script started this way may itself
`run_example` another one before it reads its own inputs: they are restored afterwards.

    run_example("simple_opt_reelout.jl"; show_plots = false,
                tos_overrides = Dict{Symbol, Any}(:max_reopt => 0))
"""
function run_example(file::AbstractString; inputs...)
    path = abspath(isabspath(file) ? file : joinpath(dirname(@__DIR__), "examples", file))
    isfile(path) || error("run_example: no script $path")
    outer = SCRIPT_INPUTS[]
    SCRIPT_INPUTS[] = (; file = path, inputs = NamedTuple(inputs))
    try
        return Base.include(Main, path)
    finally
        SCRIPT_INPUTS[] = outer
    end
end

"""
    script_inputs(file, defaults::NamedTuple) -> NamedTuple

The inputs [`run_example`](@ref) passed to the script `file` (call it with `@__FILE__`),
merged over `defaults`, which name every input the script takes. Only the script
`run_example` was called on gets them, once: another script it includes gets its own
defaults. A keyword the script does not take is an error.
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

"""
    muted(f)

Call `f()` with `stdout` and logging silenced, e.g. a `run_example` inside a sweep.
Rebinds `Base.stdout` instead of `redirect_stdout`, which can only restore a
file-backed stream and so fails, with `stdout` left on `devnull`, in a REPL whose
`stdout` is a custom IO (e.g. Kaimon's).
"""
function muted(f)
    out = stdout
    setglobal!(Base, :stdout, devnull)
    try
        return with_logger(f, Base.CoreLogging.NullLogger())
    finally
        setglobal!(Base, :stdout, out)
    end
end

"""
    latest_global(name::Symbol)

The global `name` of `Main`, read at the latest world age: after a
[`run_example`](@ref) the script's globals were (re)defined in a newer world than
the caller's code runs in.
"""
latest_global(name::Symbol) = Base.invokelatest(getglobal, Main, name)

"The first line of the message `err` would print: a one-line reason for a progress log"
first_error_line(err) = first(split(sprint(showerror, err), '\n'))
