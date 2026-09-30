# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Retune the reel-out course loop in small steps until the worst guided disk
margin over all archived scenarios reaches a target, on the linear model of
`stability_opt_reelout.jl`.

The operating points of every archived scenario of both sites
(`output/scenarios/<site>/vNN`, selected as `stability_global.jl` does) are
collected once: for each linear tether-length bin its log samples and the tape's
lag. A trial setting is then evaluated on them with `bin_margins`, the same
worst case per bin as `stability_opt_reelout.jl` (v_a and depower corners, the
highest `ω_g` flown near each `v_a`, both signs of the gravity pole), so the
live settings reproduce `stability_overview.md`. The samples are those on the
path within the live `attractor_dist`; a trial `attractor_dist` changes `ω_g`,
not which samples are rated.

`retune` starts from the live `fc_settings_reelout.yaml` and raises or lowers
one of `heading_p`, `heading_d`, `attractor_dist` and `attractor_lead_time` per
step, by a small increment, whichever lifts the worst scenario most, until it
reaches `target`. Small steps on purpose: the model ranks settings only roughly,
and a large jump that raised the modelled margin ("C", 2026-09-25) made tracking
worse in flight. The result is a proposal: write it to `fc_settings_reelout.yaml`
by hand and fly the regression runs before adopting it.

Result 2026-09-28 (c3 = 0.23 1/s, 22 scenarios): from lead 0.88 s and
`heading_d` 0.126 s, worst α guided 0.257 (Cabauw 10 m/s) -> 0.305 with lead
0.96 s and `heading_d` 0.136 s, see docs/course_loop_stability_reelout.md.

About 1 min to collect, then about 20 s per trial setting (3 – 8 per step).

    include("examples/retune_guided.jl")
    data = collect_scenarios()                 # once
    table(data, live_settings())               # worst margins per scenario
    trail = retune(data; target = 0.3)         # greedy small steps
    table(data, trail[end].settings)           # the proposal, per scenario
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Printf
using Statistics: median
using Base.CoreLogging: with_logger, NullLogger
using SimpleKiteControllers: run_example, script_inputs

"Sites and their projects, as `stability_opt_reelout.jl` supports them"
const RETUNE_SITES = ("cabauw" => "system_reelout_cabauw.yaml",
                      "maasvlakte" => "system_reelout_maasvlakte.yaml")

"""
Step per setting: relative for `heading_p`, absolute otherwise [-, s, deg, s].
"""
const RETUNE_STEPS = (heading_p = 0.02, heading_d = 0.005, attractor_dist = 0.25,
                      attractor_lead_time = 0.02)

"Call `f()` with `stdout` and logging silenced (see `muted` in `stability_global.jl`)"
function muted(f)
    out = stdout
    setglobal!(Base, :stdout, devnull)
    try
        return with_logger(f, NullLogger())
    finally
        setglobal!(Base, :stdout, out)
    end
end

latest(name) = Base.invokelatest(getglobal, Main, name)

"""
    collect_scenario(project, dir) -> NamedTuple

Run `stability_opt_reelout.jl` for `project` on the scenario folder `dir` and
keep its linear bins: `(; bins, lag, Ts)`, each bin with its log `samples`.
"""
function collect_scenario(project, dir)
    muted(() -> run_example("stability_opt_reelout.jl"; show_plots = false, log_dir = dir, project))
    return (; bins = [r.samples for r in latest(:lin_rows)], lag = latest(:tape_lag).T,
            Ts = latest(:Ts))
end

"""
    collect_scenarios(; sites = RETUNE_SITES) -> Vector

The linear bins of every archived scenario of `sites`, one
`(; site, name, bins, lag, Ts)` per scenario.
"""
function collect_scenarios(; sites = RETUNE_SITES)
    data = NamedTuple[]
    for (site, project) in sites
        root = normpath(joinpath(@__DIR__, "..", "output", "scenarios", site))
        for name in sort(readdir(root))
            dir = joinpath(root, name)
            !occursin('_', name) && isdir(dir) && any(endswith(".arrow"), readdir(dir)) || continue
            @info "Collecting $site/$name"
            push!(data, (; site, name, collect_scenario(project, dir)...))
        end
    end
    allequal(d.Ts for d in data) || error("The scenarios differ in the controller's sample time.")
    return data
end

"The live `fc_settings_reelout.yaml`"
live_settings() = deepcopy(latest(:fcs))

"Copy of the settings `f` with the field `k` moved by one step in direction `sgn`"
function step_setting(f, k, sgn)
    g = deepcopy(f)
    δ = k == :heading_p ? RETUNE_STEPS[k] * latest(:fcs).heading_p : RETUNE_STEPS[k]
    setproperty!(g, k, getproperty(f, k) + sgn * δ)
    return g
end

"""
    scenario_margins(d, f; inner = false) -> NamedTuple

Worst guided disk margin `α_g`, its delay margin `dm_g`, tether length `L` and
`v_a`, and (with `inner = true`) the worst inner margin `α_i`, of the scenario
`d` for the settings `f`.
"""
function scenario_margins(d, f; inner = false)
    bin_margins, Ts = latest(:bin_margins), latest(:Ts)
    Ts == d.Ts || error("Collected at another sample time; collect_scenarios() again.")
    bms = [Base.invokelatest(bin_margins, s, d.lag; f, inner) for s in d.bins]
    w = argmin(b -> b.wg.m.guided.α, bms)
    return (; α_g = w.wg.m.guided.α, dm_g = w.wg.m.guided.delay_margin, L = w.L, va = w.wg.va,
            α_i = inner ? minimum(b.wi.m.inner.α for b in bms) : NaN)
end

"Worst guided disk margin over all scenarios in `data` for the settings `f`"
worst_margin(data, f) = minimum(scenario_margins(d, f).α_g for d in data)

"""
    retune(data; target = 0.3, settings_keys = keys(RETUNE_STEPS), max_steps = 20) -> Vector

Greedy small-step ascent of the worst guided disk margin from the live settings:
per step, every setting in `settings_keys` is moved one step up and one down, and the
move that lifts the worst scenario most is taken; stops at `target`, after
`max_steps`, or when no move helps. Returns the trail, one `(; settings, α,
move)` per step, the live settings first.
"""
function retune(data; target = 0.3, settings_keys = keys(RETUNE_STEPS), max_steps = 20)
    f = live_settings()
    trail = [(; settings = f, α = worst_margin(data, f), move = "live")]
    @info @sprintf("live: worst α guided = %.4f", trail[end].α)
    while trail[end].α < target && length(trail) <= max_steps
        moves = [(k, sgn) for k in settings_keys for sgn in (1, -1)]
        vals = [worst_margin(data, step_setting(trail[end].settings, k, sgn)) for (k, sgn) in moves]
        best = argmax(vals)
        vals[best] > trail[end].α || (@warn "No step lifts the worst margin; stopped."; break)
        k, sgn = moves[best]
        g = step_setting(trail[end].settings, k, sgn)
        push!(trail, (; settings = g, α = vals[best],
                      move = @sprintf("%s %s → %.5g", k, sgn > 0 ? "+" : "−", getproperty(g, k))))
        @info @sprintf("step %d: %s, worst α guided = %.4f", length(trail) - 1, trail[end].move, vals[best])
    end
    trail[end].α >= target || @warn @sprintf("Target %.2f not reached: %.4f.", target, trail[end].α)
    f0, f1 = trail[1].settings, trail[end].settings
    println("Proposal (live → retuned):")
    for k in RETUNE_STEPS |> propertynames
        println(@sprintf("  %-20s %8.5g → %8.5g", k, getproperty(f0, k), getproperty(f1, k)))
    end
    return trail
end

"Print the worst margins per scenario of `data` for the settings `f`, inner loop included"
function table(data, f)
    println("  scenario            α inner  α guided  DM guided   at L [m]  v_a [m/s]")
    for d in data
        m = scenario_margins(d, f; inner = true)
        println(@sprintf("  %-18s  %7.3f  %8.3f  %7.3f s  %8.0f  %9.1f", "$(d.site)/$(d.name)",
                         m.α_i, m.α_g, m.dm_g, m.L, m.va))
    end
end

nothing
