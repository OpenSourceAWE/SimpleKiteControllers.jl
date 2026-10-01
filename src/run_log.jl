# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The log messages of a run, written to a file as they are shown (`with_run_log`), and what the
# file says about the startup's retries (`startup_ladder_report`) and how two runs compare
# (`startup_log_lines`). Base only, so an older checkout can `include` this file to log its runs.

using Base.CoreLogging: AbstractLogger, LogLevel, Info, Warn, Error, current_logger, with_logger
import Base.CoreLogging: handle_message, shouldlog, min_enabled_level, catch_exceptions

"""
    TeeLogger(inner, io, min_level)

Passes every message on to `inner` unchanged and writes the ones at `min_level` and above to `io`,
in the console's `[ Info: message` form, without the source location.
"""
struct TeeLogger{L <: AbstractLogger, I <: IO} <: AbstractLogger
    inner::L
    io::I
    min_level::LogLevel
end
min_enabled_level(l::TeeLogger) = min(min_enabled_level(l.inner), l.min_level)
shouldlog(::TeeLogger, args...) = true
catch_exceptions(l::TeeLogger) = catch_exceptions(l.inner)

function handle_message(l::TeeLogger, level, message, _module, group, id, file, line; kwargs...)
    if level >= l.min_level
        write_log_entry(l.io, level, message, kwargs)
        flush(l.io)    # a run that crashes keeps every message up to the crash
    end
    if level >= min_enabled_level(l.inner) && shouldlog(l.inner, level, _module, group, id)
        handle_message(l.inner, level, message, _module, group, id, file, line; kwargs...)
    end
    return nothing
end

function write_log_entry(io, level, message, kwargs)
    label = level == Warn ? "Warning" : string(level)
    lines = split(string(message), '\n')
    extra = String[]
    for (k, v) in kwargs
        k === :maxlog && continue
        if k === :exception
            exc = v isa Tuple ? first(v) : v
            append!(extra, split(sprint(showerror, exc), '\n'))
        else
            push!(extra, "  $k = $v")
        end
    end
    println(io, "[ $label: ", first(lines))
    for s in Iterators.flatten((lines[2:end], extra))
        println(io, "│ ", s)
    end
end

"""
    with_run_log(f, path; min_level = Info)

Run `f()` with its log messages shown as usual and also written to `path`, message by message, so
the file is complete up to a crash. A throw is written as a last `[ Error: the run threw: …` line
and rethrown. The parent folder is created. Returns what `f` returns.
"""
function with_run_log(f, path::AbstractString; min_level = Info)
    mkpath(dirname(abspath(path)))
    open(path, "w") do io
        try
            with_logger(f, TeeLogger(current_logger(), io, min_level))
        catch exc
            println(io, "[ Error: the run threw: ", first(split(sprint(showerror, exc), '\n')))
            rethrow()
        end
    end
end

read_log_text(src) = isfile(src) ? read(src, String) : String(src)

# The messages of src/startup_path.jl and src/awetrim_client.jl the report is read from; keep them
# in step with those messages.
const RE_LADDER_START = r"The startup path is at margin ([\d.]+), below min_feasibility_margin = ([\d.]+): retry 1/(\d+)"
const RE_ATTEMPT = r"The startup path is at margin [\d.]+, below min_feasibility_margin = [\d.]+: retry (\d+)/\d+ \(([^)]+)\)"
const RE_422 = r"Startup retry (\d+) (?:\([^)]+\) )?could not converge \(HTTP 422\)"
const RE_MEASURED = r"Startup retry (\d+) measured: margin ([\d.]+)"
const RE_KEPT = r"Kept retry (\d+) as the best-so-far"
const RE_CLEARED = r"Startup path clears the gates at margin ([\d.]+) after (\d+) solves"
const RE_NO_BETTER = r"Startup retry (\d+) gave margin ([\d.]+), no better than"
const RE_FLOOR = r"Startup retry (\d+) reached margin ([\d.]+) but dropped below a floor"
const RE_STOP = r"Startup retries stop at (\d+)/"
const RE_SEED_FAILED = r"Startup solve at ([\d.]+)° failed: retry (\d+)/(\d+)"
const RE_SEED_OK = r"Startup path solved from a RETRY seed centred at ([\d.]+)° \(([+-][\d.]+)°"
const RE_START_MARGIN = r"Pattern feasibility at the STARTING length: margin ([\d.]+)"
const RE_THREW = r"^\[ Error: the run threw: (.*)$"m

"""
    startup_ladder_report(log) -> NamedTuple

What a run log of `simple_opt_reelout.jl` (`log` is the file written by `with_run_log`, or its text)
says about the two retry paths of the startup:

- `seed_retries`: startup solves sent again from another seed after a 422 (`startup_retry_el_offsets`),
  and `seed_offset`, the offset the solve converged from (`nothing` for the shipped guess);
- `ladder`: whether `retry_startup!` ran, at `margin_in` against the gate `gate`, and per attempt
  `attempts`, `lever => outcome` with the outcome `"422"`, `"took over"`, `"took over, cleared"`,
  `"no better"` or `"below a floor"` (each with its margin), and `stopped` when the levers ran out;
- `margin_start`: the startup margin the run went on with, and `threw`, the first line of the error a
  run that stopped there threw (`nothing` otherwise).

`ladder_line(report)` is the one-line form.
"""
function startup_ladder_report(log)
    text = read_log_text(log)
    seed_retries = length(collect(eachmatch(RE_SEED_FAILED, text)))
    m = match(RE_SEED_OK, text)
    seed_offset = isnothing(m) ? nothing : parse(Float64, m[2])
    m = match(RE_LADDER_START, text)
    ladder = !isnothing(m)
    margin_in = ladder ? parse(Float64, m[1]) : NaN
    gate = ladder ? parse(Float64, m[2]) : NaN
    levers = Dict(parse(Int, a[1]) => String(a[2]) for a in eachmatch(RE_ATTEMPT, text))
    outcome = Dict{Int, String}()
    for a in eachmatch(RE_422, text); outcome[parse(Int, a[1])] = "422"; end
    for a in eachmatch(RE_MEASURED, text); outcome[parse(Int, a[1])] = "margin $(a[2])"; end
    for a in eachmatch(RE_NO_BETTER, text); outcome[parse(Int, a[1])] = "no better, $(a[2])"; end
    for a in eachmatch(RE_FLOOR, text); outcome[parse(Int, a[1])] = "below a floor, $(a[2])"; end
    for a in eachmatch(RE_KEPT, text)
        i = parse(Int, a[1])
        outcome[i] = "took over, " * replace(get(outcome, i, ""), "margin " => "")
    end
    m = match(RE_CLEARED, text)
    if !isnothing(m)
        outcome[parse(Int, m[2])] = "took over, cleared, $(m[1])"
    end
    attempts = [get(levers, i, "?") => get(outcome, i, "?") for i in sort!(collect(keys(levers)))]
    stopped = occursin(RE_STOP, text)
    m = match(RE_START_MARGIN, text)
    margin_start = isnothing(m) ? NaN : parse(Float64, m[1])
    m = match(RE_THREW, text)
    threw = isnothing(m) ? nothing : String(m[1])
    return (; seed_retries, seed_offset, ladder, margin_in, gate, attempts, stopped,
            margin_start, threw)
end

"""
    ladder_line(report) -> String

`startup_ladder_report` in one line, e.g. `startup: ladder at margin 0.979 < 1.30, 4 retries
[radius correction: 422, ceiling step: 422, …], levers spent; flew margin 1.04; threw …`.
"""
function ladder_line(r)
    parts = String[]
    r.seed_retries > 0 &&
        push!(parts, "$(r.seed_retries) seed retr$(r.seed_retries == 1 ? "y" : "ies"), " *
                     (isnothing(r.seed_offset) ? "none converged" :
                          "converged at $(r.seed_offset)°"))
    if r.ladder
        push!(parts, "ladder at margin $(r.margin_in) < $(r.gate), $(length(r.attempts)) retries [" *
                     join(("$k: $v" for (k, v) in r.attempts), ", ") * "]" *
                     (r.stopped ? ", levers spent" : ""))
    else
        push!(parts, "no ladder")
    end
    isnan(r.margin_start) || push!(parts, "flew margin $(r.margin_start)")
    isnothing(r.threw) || push!(parts, "threw: $(r.threw)")
    return "startup: " * join(parts, "; ")
end

# Messages of the optimizer client that differ between a run the server answered and the same run
# served from the solution or failure cache.
const RE_TRANSPORT = r"^\[ \w+: (POST |This exact request failed before|Optimizer step at L = .*(served from the solution cache|replayed)|Rebuilding the optimizer's session|Could not rebuild the optimizer's session|No AWETrim server|Something still answers|Replaying the )"

"""
    startup_log_lines(log) -> Vector{String}

The startup section of a run log (see `with_run_log`), as two runs of the same code with the same
optimizer answers write it: the messages up to the first progress line of the loop (`step …`), without
their continuation lines, without the optimizer client's transport messages (`RE_TRANSPORT`, which differ
between a server run and a cached one), and with wall times (`… s`), file time stamps and the folder
of saved trajectories (`…/trajectories/`, which differs between checkouts) masked.
"""
function startup_log_lines(log)
    out = String[]
    for line in split(read_log_text(log), '\n')
        startswith(line, "[ ") || continue
        occursin(r"^\[ Info: step\s+\d+ /", line) && break
        occursin(RE_TRANSPORT, line) && continue
        s = replace(line, r"\d+(\.\d+)? s\b" => "<t> s", r"\d{4}-\d{2}-\d{2}_\d{4}" => "<stamp>",
                    r"\S*/trajectories/" => "trajectories/")
        push!(out, s)
    end
    return out
end
