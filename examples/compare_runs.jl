# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Regression check for refactors of `simple_opt_reelout.jl`: compares two runs of
it, the `.arrow` logs column by column and the `_opt.yaml` summaries key by key.

A run is named by its folder and log file, as `output/` or a scenario folder
holds them, for example

    include("compare_runs.jl")
    compare_runs("output/scenarios/maasvlakte/v08", "output/regression/v08")

`compare_runs(a, b; log = <the only *_opt.arrow in a>)` returns `true` if the
runs are identical and prints every difference otherwise. Ignored, because they
differ between any two runs of the same code: the wall-clock fields of the
summary (`time`, `date`, `hostname`, `git_hash`, `git_status`, `t_wall`, wall
times of the re-optimization cycles, `realtime_factor`, the profiling timings),
and the `time`-of-day metadata of the log columns.

Numbers are compared exactly (`===`, so `NaN` equals `NaN`) unless `rtol > 0`
is given. A pure refactor must be exact; `rtol` is for diagnosing how large a
difference is.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

import YAML
using KiteUtils
const Arrow = KiteUtils.Arrow

"Summary keys (at any depth) that hold wall-clock or machine-dependent values"
const IGNORED_KEYS = Set(["time", "date", "hostname", "git_hash", "git_status", "t_wall",
                          "wall_s", "realtime_factor", "wall_time", "total_time", "timing",
                          "cycle_wall", "total_wall_time", "optimization_time",
                          "max_optimization_time", "ms_per_step"])

"The one `*_opt.arrow` in `dir`, or an error if there is none or more than one"
function find_log(dir)
    logs = filter(f -> endswith(f, "_opt.arrow"), readdir(dir))
    length(logs) == 1 || error("Expected exactly one *_opt.arrow in $dir, found $(length(logs)); pass `log`.")
    return first(logs)
end

same(x, y, rtol) = x === y || (rtol > 0 && x isa Real && y isa Real && isapprox(x, y; rtol, atol = rtol))
same(x::AbstractArray, y::AbstractArray, rtol) =
    size(x) == size(y) && all(same(a, b, rtol) for (a, b) in zip(x, y))

"Compare the columns of two Arrow tables, push messages to `diffs`; returns the number of columns compared"
function compare_tables!(diffs, ta, tb, rtol)
    na, nb = collect(propertynames(ta)), collect(propertynames(tb))
    for n in setdiff(na, nb); push!(diffs, "log: column $n only in A"); end
    for n in setdiff(nb, na); push!(diffs, "log: column $n only in B"); end
    common = intersect(na, nb)
    for n in common
        a, b = getproperty(ta, n), getproperty(tb, n)
        if length(a) != length(b)
            push!(diffs, "log: column $n has $(length(a)) rows in A, $(length(b)) in B")
            continue
        end
        bad = findall(i -> !same(a[i], b[i], rtol), eachindex(a))
        isempty(bad) && continue
        i = first(bad)
        push!(diffs, "log: column $n differs in $(length(bad)) of $(length(a)) rows; first at row $i (A = $(a[i]), B = $(b[i]))")
    end
    return length(common)
end

"Compare two YAML documents recursively, ignoring `IGNORED_KEYS`"
function compare_yaml!(diffs, a, b, rtol, path = "")
    if a isa AbstractDict && b isa AbstractDict
        for k in union(keys(a), keys(b))
            k in IGNORED_KEYS && continue
            p = isempty(path) ? string(k) : "$path.$k"
            haskey(a, k) || (push!(diffs, "yaml: $p only in B"); continue)
            haskey(b, k) || (push!(diffs, "yaml: $p only in A"); continue)
            compare_yaml!(diffs, a[k], b[k], rtol, p)
        end
    elseif a isa AbstractVector && b isa AbstractVector
        length(a) == length(b) || (push!(diffs, "yaml: $path has $(length(a)) entries in A, $(length(b)) in B"); return)
        for i in eachindex(a)
            compare_yaml!(diffs, a[i], b[i], rtol, "$path[$i]")
        end
    elseif !same(a, b, rtol)
        push!(diffs, "yaml: $path: A = $a, B = $b")
    end
    return nothing
end

"""
    compare_runs(dir_a, dir_b; log = nothing, rtol = 0.0) -> Bool

Compare the log and the summary of the run in `dir_a` with the one in `dir_b`
(see the file's docstring). Prints every difference; returns `true` if there is none.
"""
function compare_runs(dir_a, dir_b; log = nothing, rtol = 0.0)
    log_a = isnothing(log) ? find_log(dir_a) : log
    log_b = isnothing(log) ? find_log(dir_b) : log
    diffs = String[]
    ncols = compare_tables!(diffs, Arrow.Table(joinpath(dir_a, log_a)),
                            Arrow.Table(joinpath(dir_b, log_b)), rtol)
    yaml_a, yaml_b = replace(log_a, ".arrow" => ".yaml"), replace(log_b, ".arrow" => ".yaml")
    for (fa, fb) in ((yaml_a, yaml_b), (replace(yaml_a, "_opt.yaml" => "_opt_opt_paths.yaml"),
                                        replace(yaml_b, "_opt.yaml" => "_opt_opt_paths.yaml")))
        pa, pb = joinpath(dir_a, fa), joinpath(dir_b, fb)
        if isfile(pa) != isfile(pb)
            push!(diffs, "yaml: $fa exists only in $(isfile(pa) ? "A" : "B")")
        elseif isfile(pa)
            compare_yaml!(diffs, YAML.load_file(pa), YAML.load_file(pb), rtol)
        end
    end
    if isempty(diffs)
        println("IDENTICAL: $ncols log columns and the summaries of $dir_a and $dir_b")
    else
        println("DIFFERENT: $(length(diffs)) difference(s) between $dir_a and $dir_b")
        foreach(d -> println("  ", d), first(diffs, 40))
        length(diffs) > 40 && println("  ... and $(length(diffs) - 40) more")
    end
    return isempty(diffs)
end

nothing
