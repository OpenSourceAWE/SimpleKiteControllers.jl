# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Identify the gravity coefficient `C3` of the turn-rate law

    ψ̇ = c1·v_a·u_s + c3·sin(ψ)·cos(β)

from the archived reel-out runs in `output/scenarios/<site>/vNN` (a symlink into
SimulationResults). The relay sweeps of `build_turn_rate_table.jl` cannot
identify the gravity term — their steering is fed back from the heading, so it
trades against the delay — but a figure of eight turns through every heading,
which excites it independently of the steering.

Samples: the ones `stability_opt_reelout.jl` rates, phases 3-5 with the
cross-track error (`var_01`) below `attractor_dist`. `c1` and the delay come from
the turn-rate table at each sample's depower (clamped to the table's range);
the turn rate from `calc_turn_rate`, as in V3Kite's `identify_turn_rate_law`.
Only `c3` is fitted, by linear least squares on
`ψ̇ - c1·v_a·u_s(t - τ) = c3·sin(ψ)·cos(β)`.

Per run it also prints `c2`, the coefficient of the table's form
`c2/v_a·sin(ψ)·cos(β)`, fitted the same way: it grows about in proportion to
`v_a`, while `c3` stays flat, which is why `course_loop_model.jl` uses `C3`.
Result 2026-09-28, 25 runs: pooled `c3` = 0.230 1/s, per run 0.18 – 0.30.

    include("examples/identify_c3.jl")
    r = identify_c3()          # both sites; identify_c3(sites = ["cabauw"]) for one
    b = c3_by_depower()        # c3 in bins of the depower flown
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using V3Kite
using V3Kite: calc_turn_rate, wrap_to_pi
using SimpleKiteControllers
using Statistics: mean, median, std
using Printf
import YAML

set_data_path(normpath(joinpath(@__DIR__, "..", "data")))

"Value of `key` anywhere in the nested dictionary `d`, or `nothing`"
function find_key(d, key)
    d isa AbstractDict || return nothing
    haskey(d, key) && return d[key]
    for v in values(d)
        x = find_key(v, key)
        isnothing(x) || return x
    end
    return nothing
end

"""
    rated_samples(dir) -> NamedTuple

Regressor `g = sin(ψ)·cos(β)`, residual turn rate `y = ψ̇ - c1·v_a·u_s(t - τ)`,
apparent wind speed `v_a` [m/s] and depower `dp` [-] of the rated samples of the
run archived in `dir`, as vectors.
"""
function rated_samples(dir)
    fcs = YAML.load_file(joinpath(dir, "fc_settings_reelout.yaml"))
    attractor_dist = Float64(find_key(fcs, "attractor_dist"))
    body_damping = Float64.(find_key(fcs, "body_damping"))
    dp_lo, dp_hi = turn_rate_depower_range(body_damping)
    name = only(filter(endswith(".arrow"), readdir(dir)))
    sl = load_log(splitext(name)[1]; path = dir).syslog
    dt = median(diff(collect(sl.time)))
    rate = calc_turn_rate(sl; source = :heading, dt)   # aligned to sl.time[2:end]
    g = Float64[]; y = Float64[]; v_a = Float64[]; dp = Float64[]
    for i in 2:length(sl.time)
        (3 <= sl.sys_state[i] <= 5 && sl.var_01[i] <= attractor_dist) || continue
        depower = Float64(sl.depower[i])
        tc = turn_rate_coeffs(body_damping, clamp(depower, dp_lo, dp_hi))
        k = i - round(Int, tc.delay / dt)
        k >= 1 || continue
        va = Float64(sl.v_app[i])
        push!(g, sin(wrap_to_pi(sl.heading[i])) * cos(sl.elevation[i]))
        push!(y, rate[i - 1] - tc.c1 * va * sl.steering[k])
        push!(v_a, va); push!(dp, depower)
    end
    return (; g, y, v_a, dp)
end

"""
    fit_run(dir) -> NamedTuple

Sums for the least-squares fit of `c3` (and of the table form's `c2`) over the
rated samples of the run archived in `dir`.
"""
function fit_run(dir)
    (; g, y, v_a) = rated_samples(dir)
    s3 = [sum(g .* y), sum(g .^ 2)]
    s2 = [sum(g ./ v_a .* y), sum((g ./ v_a) .^ 2)]
    return (; s3, s2, v_a = isempty(v_a) ? 0.0 : mean(v_a), n = length(g))
end

"Archived run directories of `sites`"
function run_dirs(sites)
    root = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))
    dirs = String[]
    for site in sites, v in sort(readdir(joinpath(root, site)))
        dir = joinpath(root, site, v)
        isdir(dir) && any(endswith(".arrow"), readdir(dir)) && push!(dirs, dir)
    end
    return dirs
end

"""
    identify_c3(; sites = ["maasvlakte", "cabauw"]) -> NamedTuple

Fit `c3` on every archived run of `sites`, print one row per run and the pooled
value over all rated samples. Returns `(; c3, runs)`.
"""
function identify_c3(; sites = ["maasvlakte", "cabauw"])
    runs = NamedTuple[]
    for dir in run_dirs(sites)
        r = fit_run(dir)
        r.n > 100 || continue
        push!(runs, (; name = joinpath(splitpath(dir)[end-1:end]...), r...,
                     c3 = r.s3[1] / r.s3[2], c2 = r.s2[1] / r.s2[2]))
    end
    println("run                  v_a [m/s]   c2 [-]   c3 [1/s]   samples")
    for r in runs
        @printf("%-20s %9.1f %8.2f %10.3f %9d\n", r.name, r.v_a, r.c2, r.c3, r.n)
    end
    c3 = sum(r.s3[1] for r in runs) / sum(r.s3[2] for r in runs)
    c3s = [r.c3 for r in runs]
    @printf("pooled c3 = %.3f 1/s over %d samples; per run %.3f ± %.3f (%.3f – %.3f)\n",
            c3, sum(r.n for r in runs), mean(c3s), std(c3s), extrema(c3s)...)
    return (; c3, runs)
end

"""
    c3_by_depower(; sites = ["maasvlakte", "cabauw"],
                  edges = [0.26, 0.27, 0.28, 0.30, 0.33, 0.36, 0.42]) -> Vector

Fit `c3` in bins of the depower flown (bin `j` is `edges[j] <= dp < edges[j+1]`),
pooled over the rated samples of all runs of `sites`, and print one row per bin
with its standard error, sample count and mean `v_a`. The standard error assumes
independent samples and understates the uncertainty; compare the scatter between
the bins instead. Result 2026-09-28, 25 runs, depower 0.263 – 0.410: `c3` 0.207 –
0.256 1/s without a trend (0.209, 0.207, 0.256, 0.210, 0.248, 0.237), so `c3` is
not scheduled with the depower. Returns one `(; lo, hi, c3, se, n, v_a)` per bin.
"""
function c3_by_depower(; sites = ["maasvlakte", "cabauw"],
                       edges = [0.26, 0.27, 0.28, 0.30, 0.33, 0.36, 0.42])
    runs = filter(r -> length(r.g) > 100, map(rated_samples, run_dirs(sites)))
    g, y, v_a, dp = (reduce(vcat, getfield.(runs, f)) for f in (:g, :y, :v_a, :dp))
    bins = NamedTuple[]
    println("depower          c3 [1/s]   se [1/s]   samples   v_a [m/s]")
    for (lo, hi) in zip(edges[1:end-1], edges[2:end])
        m = lo .<= dp .< hi
        n = count(m)
        n > 1 || continue
        gm, ym = g[m], y[m]
        c3 = sum(gm .* ym) / sum(gm .^ 2)
        se = sqrt(sum((ym .- c3 .* gm) .^ 2) / (n - 1) / sum(gm .^ 2))
        push!(bins, (; lo, hi, c3, se, n, v_a = mean(v_a[m])))
        @printf("%.3f – %.3f %10.3f %10.3f %9d %11.1f\n", lo, hi, c3, se, n, mean(v_a[m]))
    end
    @printf("depower of the rated samples: %.3f – %.3f over %d runs\n", extrema(dp)..., length(runs))
    return bins
end

nothing
