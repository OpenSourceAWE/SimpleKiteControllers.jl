# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Settings of the figure-of-eight SHAPE optimization driven by
`examples/optimize_fig8.jl`: the size of the process pool, the parameter grid
that is swept, and the side conditions a run must satisfy to be ranked at all.
Loaded from `data/optimization.yaml`, in the same way and for the same reason as
[`FC_Settings`](@ref) is loaded from `fc_settings.yaml` — a sweep is defined by a
file, not by editing a script.

The grid is the OUTER product of `f8_a` (pattern width) and `f8_b` (pattern
height), both walked in `step_size` degree increments and both inclusive of their
bounds. Everything else about a run — the plant, the tether length, the wind, the
controller tuning — comes from the system project the workers fly, exactly as an
interactive `simple_reelout.jl` run would; only `f8_a` and `f8_b` are overridden.
"""
@with_kw mutable struct OptSettings @deftype Float64
    "Number of worker processes run in parallel; one simulation each at a time"
    max_processes::Int64 = 6
    "Lower bound of the pattern width sweep [deg], inclusive"
    min_f8_a = 20.0
    "Upper bound of the pattern width sweep [deg], inclusive"
    max_f8_a = 30.0
    "Lower bound of the pattern height sweep [deg], inclusive"
    min_f8_b = 8.0
    "Upper bound of the pattern height sweep [deg], inclusive"
    max_f8_b = 12.0
    "Grid increment of BOTH axes [deg]"
    step_size = 1.0
    """
    Results table, relative to `output/`. Appended to under a lock as each run
    finishes, so it is also the sweep's resume state: a combination already in
    here is never flown again (delete the file to start over).
    """
    results_file::String = "optimization_results.yaml"
    """
    Restart a worker process after this many simulations, so a leaked model or a
    fragmented heap cannot accumulate across a long sweep. `0` never restarts;
    the cost of a restart is one package-load (~1 min), so keep it well above 1.
    """
    max_runs_per_process::Int64 = 0
    """
    SIDE CONDITION: the maximum share of the reel-out window the winch's
    UpperForceController may hold the drum [%]. `0.0` is "shall never become
    engaged" — a pattern whose power was capped by the force limiter is a
    measurement of the limiter, not of the shape. See [`winch_state_pct`](@ref).
    """
    max_pct_time_upper_force = 0.0
    """
    SIDE CONDITION: the maximum share of the settled window the steering command
    may spend within 2 % of its own peak [%], i.e. clamped against
    `max_steering`. A shape that is only flyable at full steering has no margin
    left for gusts, so its power is not bankable. See `fig8_metrics`'
    `steering_sat_frac`.
    """
    max_pct_time_within_2pct_of_peak = 5.0
end

"""
    OptSettings(filename::String; path=skc_data_path()) -> OptSettings

Load sweep settings from the YAML file `filename` under `path`, defaulting to
this package's own [`skc_data_path`](@ref). The file must have a top-level
`optimization:` mapping whose keys are field names of [`OptSettings`](@ref); a
missing key falls back to the struct default and an unknown key is an error.
Same contract as [`FC_Settings`](@ref).
"""
function OptSettings(filename::String; path = skc_data_path())
    dict = YAML.load_file(isabspath(filename) ? filename :
                          joinpath(path, filename))["optimization"]
    os = OptSettings()
    for (key, value) in dict
        sym = Symbol(key)
        hasfield(OptSettings, sym) ||
            error("Unknown key \"$key\" in $filename — not a field of OptSettings.")
        setfield!(os, sym, convert(fieldtype(OptSettings, sym), value))
    end
    os.step_size > 0 || error("step_size must be > 0, got $(os.step_size).")
    os.max_processes >= 1 || error("max_processes must be >= 1, got $(os.max_processes).")
    return os
end

"""
    opt_grid(os::OptSettings) -> Vector{@NamedTuple{f8_a::Float64, f8_b::Float64}}

Every (width, height) combination of the sweep, `f8_b` varying fastest. Both
axes are walked as `min + i * step_size` and rounded to 3 decimals rather than
built with a floating-point range, so that the values — which become the keys of
the results table — are reproducible across processes and Julia versions.

The upper bound is included when it lands on the grid and dropped when it does
not: with `step_size = 1.5` from 8 to 12 the last height is 11.0, not 12.0.
"""
function opt_grid(os::OptSettings)
    axis(lo, hi) = [round(lo + i * os.step_size; digits = 3)
                    for i in 0:floor(Int, (hi - lo) / os.step_size + 1e-9)]
    return [(; f8_a, f8_b) for f8_a in axis(os.min_f8_a, os.max_f8_a)
                           for f8_b in axis(os.min_f8_b, os.max_f8_b)]
end

"""
    task_key(f8_a, f8_b) -> String

Identity of one grid point, `"a=24.000_b=10.000"`. Fixed-width and rounded so
that the same combination produces the same string in every process, whether it
came from [`opt_grid`](@ref) or was read back out of the results YAML.
"""
task_key(f8_a, f8_b) = @sprintf("a=%.3f_b=%.3f", f8_a, f8_b)

"""
    with_file_lock(f, lock_path; timeout=300.0, stale_after=900.0) -> f()

Run `f` while holding a cross-process lock, and release it even if `f` throws.

The lock is a DIRECTORY: `mkdir` is atomic on every POSIX filesystem (and on
NFS), so exactly one of several processes can create it, which a
check-then-create on a regular file cannot promise. Waiters retry with a jittered
poll until `timeout` [s] and then error rather than proceed unlocked.

A worker killed while holding the lock would otherwise block the sweep forever,
so a lock older than `stale_after` [s] is broken with a warning. Keep that well
above the longest critical section, which here is reading and appending a few
kilobytes of YAML.
"""
function with_file_lock(f, lock_path; timeout = 300.0, stale_after = 900.0,
                        poll = 0.05)
    t_start = time()
    while true
        try
            mkdir(lock_path)
            break
        catch err
            # Anything other than "it already exists" is a real filesystem problem.
            isdir(lock_path) || rethrow()
            age = time() - _lock_mtime(lock_path)
            if age > stale_after
                @warn "Breaking a stale lock" lock_path age_s=round(age; digits = 1)
                rm(lock_path; force = true, recursive = true)
                continue
            end
            time() - t_start > timeout &&
                error("Timed out after $(timeout) s waiting for the lock $lock_path.")
            # Jittered: several workers finishing together must not retry in lockstep.
            sleep(poll * (1 + rand()))
        end
    end
    try
        return f()
    finally
        rm(lock_path; force = true, recursive = true)
    end
end

# mtime of the lock as unix seconds; "now" if it vanished under us, i.e. not stale.
_lock_mtime(path) = try mtime(path) catch; time() end

"""
    init_results_file(results_path) -> String

Create the results table with its header comment if it does not exist yet, and
return `results_path`. An existing file is left untouched — that is what makes a
sweep resumable, since [`claim_task!`](@ref) skips every combination already
recorded in it.
"""
function init_results_file(results_path)
    mkpath(dirname(results_path))
    isfile(results_path) && return results_path
    open(results_path, "w") do io
        println(io, """
        # Results of the figure-of-eight shape sweep, appended one entry per
        # finished run by examples/optimize_fig8_worker.jl under a file lock.
        # Delete this file to start a sweep over; keep it to resume one, as every
        # combination listed here is skipped rather than re-flown.
        results:""")
    end
    return results_path
end

# Newlines are escaped, not kept: an entry is appended line by line, and a status
# carrying a multi-line exception message would otherwise break the table's shape.
_yaml_scalar(v::AbstractString) =
    "\"" * replace(v, "\\" => "\\\\", "\"" => "\\\"", "\n" => "\\n", "\r" => "") * "\""
_yaml_scalar(v::Bool) = string(v)
_yaml_scalar(v::Nothing) = "null"
_yaml_scalar(v::AbstractFloat) = isfinite(v) ? string(v) : ".nan"
_yaml_scalar(v) = string(v)

"""
    format_result_entry(entry) -> String

One YAML list item for the results table, from `entry`, a vector of
`"key" => value` pairs. Written by hand rather than through `YAML.write` to keep
the key ORDER of the caller, which is what makes the table readable as a table.
"""
function format_result_entry(entry)
    io = IOBuffer()
    for (i, (key, value)) in enumerate(entry)
        println(io, i == 1 ? "  - " : "    ", key, ": ", _yaml_scalar(value))
    end
    return String(take!(io))
end

"""
    record_result!(results_path, entry; lock_path=results_path * ".lock")

Append one finished run to the results table, holding the lock of
[`with_file_lock`](@ref) across the write so that concurrent workers cannot
interleave their entries. `entry` is a vector of `"key" => value` pairs, as built
by [`run_metrics`](@ref).

Appending (rather than parsing the table, adding an entry and rewriting it) is
deliberate: the critical section stays O(1) in the number of results, and a
process killed mid-sweep can at worst leave one truncated entry behind instead of
destroying the table.
"""
function record_result!(results_path, entry; lock_path = results_path * ".lock")
    with_file_lock(lock_path) do
        open(results_path, "a") do io
            print(io, format_result_entry(entry))
        end
    end
    return nothing
end

"""
    load_results(results_path) -> Vector{Dict{String, Any}}

Every finished run recorded so far, or an empty vector for a missing or
still-empty table. Callers holding the lock already should pass through
[`with_file_lock`](@ref) themselves; a lone read needs no lock, since entries are
appended whole.
"""
function load_results(results_path)
    isfile(results_path) || return Dict{String, Any}[]
    dict = YAML.load_file(results_path)
    results = get(dict, "results", nothing)
    isnothing(results) && return Dict{String, Any}[]
    return Vector{Dict{String, Any}}(results)
end

"""
    claim_task!(grid, results_path; claims_path, lock_path) -> task or `nothing`

Hand out the next unflown grid point, or `nothing` once the sweep is complete.

Claiming and recording share ONE lock, so a combination cannot be handed to two
workers: under the lock the claim file and the results table are read, the first
grid point in neither is appended to the claim file, and only then is the lock
released. The claim file is separate from the results table because a claim is
not a result — it is dropped when the sweep restarts, which is what lets a
combination whose worker died be picked up again.
"""
function claim_task!(grid, results_path;
                     claims_path = results_path * ".claims",
                     lock_path = results_path * ".lock")
    with_file_lock(lock_path) do
        done = Set(task_key(r["f8_a"], r["f8_b"]) for r in load_results(results_path))
        claimed = isfile(claims_path) ? Set(strip.(readlines(claims_path))) : Set{String}()
        for task in grid
            key = task_key(task.f8_a, task.f8_b)
            (key in done || key in claimed) && continue
            open(claims_path, "a") do io
                println(io, key)
            end
            return task
        end
        return nothing
    end
end

"""
    reset_claims!(grid, results_path; claims_path, lock_path) -> Int

Rewrite the claim file to hold only the combinations that actually have a result,
and return how many claims were dropped. Called by the driver when the pool has
gone quiet with work outstanding: those claims belong to workers that died, and
without this their grid points would be skipped forever.
"""
function reset_claims!(grid, results_path;
                       claims_path = results_path * ".claims",
                       lock_path = results_path * ".lock")
    with_file_lock(lock_path) do
        done = Set(task_key(r["f8_a"], r["f8_b"]) for r in load_results(results_path))
        claimed = isfile(claims_path) ? Set(strip.(readlines(claims_path))) : Set{String}()
        orphaned = setdiff(claimed, done)
        open(claims_path, "w") do io
            for key in sort(collect(done))
                println(io, key)
            end
        end
        return length(orphaned)
    end
end

# Nested lookup that tolerates a missing branch: a run that never reeled out has
# no `reelout: power:` section at all, and that is a result, not an error.
function _dig(node, keys...)
    for key in keys
        node isa AbstractDict || return nothing
        haskey(node, key) || return nothing
        node = node[key]
    end
    return node
end

"""
    run_metrics(summary_path, f8_a, f8_b; status="ok", worker=0)
        -> Vector{Pair{String, Any}}

One results-table entry, read out of the run summary
`examples/simple_reelout.jl` writes next to its log. Missing sections come back
as `null` rather than throwing, so a run that aborted during the entry is still
recorded — with its `status` saying so — instead of vanishing from the sweep.

Ordered, because the entry is written as YAML in this order: the two swept
parameters first, then the objective (`mean_power_W`), then the side conditions,
then the context needed to understand a rejection without opening the log.
"""
function run_metrics(summary_path, f8_a, f8_b; status = "ok", worker = 0)
    summary = isfile(summary_path) ? YAML.load_file(summary_path) : nothing
    winch = _dig(summary, "reelout", "winch_state")
    entry = Pair{String, Any}[
        "f8_a" => f8_a,
        "f8_b" => f8_b,
        "status" => status,
        "mean_power_W" => _dig(summary, "reelout", "power", "mean_W"),
        "energy_run_kJ" => _dig(summary, "reelout", "power", "energy_run_kJ"),
        "peak_power_W" => _dig(summary, "reelout", "power", "peak_W"),
        "pct_time_upper_force" => _dig(winch, "upper_force_pct"),
        "pct_time_within_2pct_of_peak" =>
            _dig(summary, "fig8_metrics", "steering", "pct_time_within_2pct_of_peak"),
        "pct_time_lower_force" => _dig(winch, "lower_force_pct"),
        "mean_force_N" => _dig(summary, "reelout", "force", "mean_N"),
        "peak_force_N" => _dig(summary, "reelout", "force", "peak_N"),
        "peak_abs_u_s" => _dig(summary, "fig8_metrics", "steering", "peak_abs_u_s"),
        "rms_cross_track_deg" => _dig(summary, "fig8_metrics", "cross_track_deg", "rms"),
        "laps" => _dig(summary, "fig8_metrics", "laps"),
        "min_elevation_deg" =>
            _dig(summary, "fig8_metrics", "elevation_deg", "min_whole_run"),
        "success_criteria" => _dig(summary, "fig8_metrics", "success_criteria"),
        "sim_time_s" => _dig(summary, "performance", "sim_time_s"),
        "wall_time_s" => _dig(summary, "performance", "wall_time_s"),
        "worker" => worker,
    ]
    return entry
end

"""
    side_conditions(result, os::OptSettings) -> Vector{String}

Why `result` may not be ranked, empty when it may. Checked in the order a run
fails them: it has to have finished, it has to have reeled out (no power, no
objective), and then the two side conditions of [`OptSettings`](@ref).

A missing metric REJECTS rather than passes. An unmeasured side condition is not
a satisfied one, and the alternative — treating `null` as 0 — would rank exactly
the runs that broke early above the ones that flew.
"""
function side_conditions(result, os::OptSettings)
    reasons = String[]
    status = get(result, "status", "unknown")
    status == "ok" || push!(reasons, "run status: $status")

    power = get(result, "mean_power_W", nothing)
    isnothing(power) && push!(reasons, "no reel-out power measured")

    upper = get(result, "pct_time_upper_force", nothing)
    if isnothing(upper)
        push!(reasons, "winch state not measured")
    elseif upper > os.max_pct_time_upper_force
        push!(reasons, @sprintf("upper force controller engaged %.1f %% of the reel-out \
                                 window (limit %.1f %%)", upper, os.max_pct_time_upper_force))
    end

    sat = get(result, "pct_time_within_2pct_of_peak", nothing)
    if isnothing(sat)
        push!(reasons, "steering saturation not measured")
    elseif sat > os.max_pct_time_within_2pct_of_peak
        push!(reasons, @sprintf("steering within 2 %% of peak for %g %% of the time \
                                 (limit %g %%)", sat, os.max_pct_time_within_2pct_of_peak))
    end
    return reasons
end

"""
    rank_results(results, os::OptSettings) -> (accepted, rejected)

Split `results` on the side conditions of [`OptSettings`](@ref) and sort the
survivors by `mean_power_W`, highest first — the sweep's objective. `accepted` is
a vector of results, `rejected` a vector of `result => reasons` pairs, itself
sorted by power so that a near-miss is easy to spot next to the winner.
"""
function rank_results(results, os::OptSettings)
    accepted = Dict{String, Any}[]
    rejected = Pair{Dict{String, Any}, Vector{String}}[]
    for result in results
        reasons = side_conditions(result, os)
        isempty(reasons) ? push!(accepted, result) : push!(rejected, result => reasons)
    end
    power(r) = something(get(r, "mean_power_W", nothing), -Inf)
    sort!(accepted; by = power, rev = true)
    sort!(rejected; by = p -> power(first(p)), rev = true)
    return accepted, rejected
end

"""
    format_results_table(results, os::OptSettings) -> String

The sweep's verdict as printable text: the ranked feasible shapes, then the
rejected ones with the condition each one broke. Returned rather than printed so
that the driver can put the same text on the console and into a file.
"""
function format_results_table(results, os::OptSettings)
    accepted, rejected = rank_results(results, os)
    io = IOBuffer()
    # A missing metric prints as "-" rather than crashing the report: an entry can
    # legitimately lack any of these (see `run_metrics`), and it is still ranked.
    ints(x) = isnothing(x) ? "-" : string(round(Int, x))
    dec1(x) = isnothing(x) ? "-" : @sprintf("%.1f", x)
    dec2(x) = isnothing(x) ? "-" : @sprintf("%.2f", x)
    gen(x) = isnothing(x) ? "-" : @sprintf("%g", x)

    println(io, "RANKED by mean reel-out power, side conditions satisfied ",
            "($(length(accepted)) of $(length(results)) runs):")
    println(io, "     f8_a   f8_b    power    energy    force   upper     sat   d_rms  laps")
    println(io, "     [deg]  [deg]     [W]      [kJ]      [N]     [%]     [%]   [deg]      ")
    for (i, r) in enumerate(accepted)
        @printf(io, "%3d. %6.1f %6.1f %8s %9s %8s %7s %7s %7s %5s\n", i,
                r["f8_a"], r["f8_b"],
                ints(get(r, "mean_power_W", nothing)),
                dec1(get(r, "energy_run_kJ", nothing)),
                ints(get(r, "mean_force_N", nothing)),
                dec1(get(r, "pct_time_upper_force", nothing)),
                gen(get(r, "pct_time_within_2pct_of_peak", nothing)),
                dec2(get(r, "rms_cross_track_deg", nothing)),
                gen(get(r, "laps", nothing)))   # halves are real: 3.5 laps, not 4
    end
    if isempty(accepted)
        println(io, "     (none — every run broke a side condition)")
    end

    println(io)
    println(io, "REJECTED ($(length(rejected))):")
    for (r, reasons) in rejected
        @printf(io, "     f8_a = %5.1f, f8_b = %5.1f, power %s W: %s\n",
                r["f8_a"], r["f8_b"], ints(get(r, "mean_power_W", nothing)),
                join(reasons, "; "))
    end
    isempty(rejected) && println(io, "     (none)")
    return String(take!(io))
end
