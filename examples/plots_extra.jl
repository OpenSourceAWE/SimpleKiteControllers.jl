# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Extra plots for the LearningControl paper that are not part of the pattern
figures of `plot_patterns_paper.jl`.

- `plot_attractor_distance`: the arc length [m] from the closest path point Q to
  the attractor point over time, for the scenarios in
  [`ATTRACTOR_SCENARIOS`](@ref) (Cabauw and Maasvlakte at low, medium and high
  wind speed) in one plot. Saved as `attractor_distance.pdf` into
  `../LearningControl/figures`.
- `plot_fig8_height`: the mean height [m] of the kite during the fig8 phase (4)
  over the wind speed, for all wind speeds of the sites in
  [`HEIGHT_SITES`](@ref), one line per site. Saved as `fig8_height.pdf` into `../LearningControl/figures`.

    include("plots_extra.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using GLMakie
using MakieControlPlots
using V3Kite
using SimpleKiteControllers

include(joinpath(@__DIR__, "plot_pattern_utils.jl"))

const PAPER_FIGURES_DIR = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))

"""
    PAPER_THEME

Makie theme for the figures that go into the LearningControl paper: TeX Gyre
Termes, the same Times design the Copernicus class sets the body text in (the
PDF embeds it as Nimbus Roman), and tick labels large enough to stay readable
once the figure is scaled down. Kept in step with the copies in
`plot_patterns_paper.jl` and `plot_powercurve.jl`, so every figure of the paper
is set in one face.
"""
const PAPER_THEME = Theme(
    fonts = (; regular = "TeX Gyre Termes", bold = "TeX Gyre Termes Bold",
               italic = "TeX Gyre Termes Italic"),
    Axis = (; xticklabelsize = 20, yticklabelsize = 20),
)

"""
    ATTRACTOR_SCENARIOS

`(site, scenario)` pairs drawn by [`plot_attractor_distance`](@ref): both sites
at 4, 7 and 10 m/s.
"""
const ATTRACTOR_SCENARIOS = [
    ("cabauw", "v04"), ("cabauw", "v07"), ("cabauw", "v10"),
    ("maasvlakte", "v04"), ("maasvlakte", "v07"), ("maasvlakte", "v10"),
]

"""
    attractor_arc(site, scenario) -> (time, arc, flown_wind)

The attractor arc `Δ·L` [m] of the run in `output/scenarios/<site>/<scenario>`
over time: [`attractor_distance`](@ref) evaluated with the scenario's own
`fc_settings_reelout.yaml` at the logged apparent wind speed and tether length,
the same value the guidance re-reads every step. `NaN` outside the phases
transition, fig8 and final (3–5); in the entry phases before them the state
machine replaces the guidance. `flown_wind` is the ground wind speed [m/s].
"""
function attractor_arc(site, scenario)
    scenario_dir = normpath(joinpath(@__DIR__, "..", "output", "scenarios", site, scenario))
    isdir(scenario_dir) || error("$scenario_dir does not exist.")
    fcs = FC_Settings(fc_settings(scenario_system_file(scenario_dir)); path = scenario_dir)

    log_name = replace(only(filter(f -> endswith(f, ".arrow"), readdir(scenario_dir))), ".arrow" => "")
    sl = load_log(log_name; path = scenario_dir).syslog
    summary_sim = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]

    rng = 2:length(sl.time)
    time = Float64.(sl.time[rng])
    l_tether = getindex.(sl.l_tether[rng], 1)
    v_app = Float64.(sl.v_app[rng])
    guided = sl.sys_state[rng] .>= 3
    arc = [guided[i] ? deg2rad(attractor_distance(fcs, v_app[i], l_tether[i])) * l_tether[i] : NaN
           for i in eachindex(time)]
    return time, arc, summary_sim["wind_speed"]
end

"""
    plot_attractor_distance(scenarios = ATTRACTOR_SCENARIOS; t_max = 150.0, disp = true, save = true)

Plot the attractor arc ([`attractor_arc`](@ref)) of each `(site, scenario)` pair
in `scenarios` over time, one line per run. The runs share one time axis, that of
the longest run; the others are sampled onto it, `NaN` beyond their own end.
The time axis runs from 0 to `t_max` [s].
"""
function plot_attractor_distance(scenarios = ATTRACTOR_SCENARIOS; t_max = 150.0, disp = true, save = true)
    runs = [attractor_arc(site, scenario) for (site, scenario) in scenarios]
    time = argmax(r -> last(r[1]), runs)[1]
    arcs = map(runs) do (t_run, arc, _)
        map(time) do t
            k = searchsortedlast(t_run, t)
            k == 0 || t > last(t_run) ? NaN : arc[k]
        end
    end
    labels = ["$(uppercasefirst(site)) $(round(r[3]; digits = 1)) m/s"
              for ((site, _), r) in zip(scenarios, runs)]

    # The save re-runs the builder, so it has to happen under the theme too.
    with_theme(PAPER_THEME) do
        p = MakieControlPlots.plot(time, arcs;
            xlabel = "time [s]", ylabel = "attractor distance [m]",
            labelsize = 22, legendsize = 16,
            labels = labels, xlims = (0.0, t_max), fig = "attractor distance", disp = disp)
        if save && disp
            pdf_file = joinpath(PAPER_FIGURES_DIR, "attractor_distance.pdf")
            savefig(pdf_file)
            @info "Saved attractor distance plot" pdf_file
        end
        return p
    end
end

"""
    HEIGHT_SITES

Sites drawn by [`plot_fig8_height`](@ref), each with all of its wind speeds.
"""
const HEIGHT_SITES = ["cabauw", "maasvlakte"]

"""
    site_scenarios(site) -> Vector{String}

The scenario names of all wind speeds flown at `site`, e.g. `"v05.5"`, sorted by
wind speed. Repeated runs such as `v10_2` are left out.
"""
function site_scenarios(site)
    site_dir = normpath(joinpath(@__DIR__, "..", "output", "scenarios", site))
    names = filter(readdir(site_dir)) do f
        occursin(r"^v\d+(\.\d+)?$", f) && any(endswith(".arrow"), readdir(joinpath(site_dir, f)))
    end
    return sort(names; by = f -> parse(Float64, f[2:end]))
end

"""
    fig8_height(site, scenario) -> (mean_height, flown_wind)

The mean height [m] of the kite above the ground during the fig8 phase (4) of
the run in `output/scenarios/<site>/<scenario>`, computed as
`l_tether * sin(elevation)`. `flown_wind` is the ground wind speed [m/s].
"""
function fig8_height(site, scenario)
    scenario_dir = normpath(joinpath(@__DIR__, "..", "output", "scenarios", site, scenario))
    isdir(scenario_dir) || error("$scenario_dir does not exist.")
    log_name = replace(only(filter(f -> endswith(f, ".arrow"), readdir(scenario_dir))), ".arrow" => "")
    sl = load_log(log_name; path = scenario_dir).syslog
    summary_sim = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]

    fig8 = findall(x -> Int(x) == 4, sl.sys_state)
    isempty(fig8) && error("$scenario_dir has no fig8 phase.")
    height = [first(sl.l_tether[i]) * sin(sl.elevation[i]) for i in fig8]
    return sum(height) / length(height), summary_sim["wind_speed"]
end

"""
    plot_fig8_height(sites = HEIGHT_SITES; disp = true, save = true)

Plot the mean height during the fig8 phase ([`fig8_height`](@ref)) over the wind
speed for all wind speeds of each site ([`site_scenarios`](@ref)), one line per
site. The sites are flown at different wind speeds, so all lines are drawn on
the union of them: each site is interpolated linearly between its own points,
which leaves its line unchanged, and is `NaN` outside its range.
"""
function plot_fig8_height(sites = HEIGHT_SITES; disp = true, save = true)
    runs = [[fig8_height(site, scenario) for scenario in site_scenarios(site)] for site in sites]
    v_wind = sort(unique(Float64(v) for r in runs for (_, v) in r))
    heights = map(runs) do r
        v_site = Float64.(last.(r))
        h_site = Float64.(first.(r))
        map(v_wind) do v
            v < first(v_site) || v > last(v_site) ? NaN :
                (k = min(searchsortedlast(v_site, v), length(v_site) - 1);
                 v == v_site[k + 1] ? h_site[k + 1] :
                 h_site[k] + (h_site[k + 1] - h_site[k]) * (v - v_site[k]) / (v_site[k + 1] - v_site[k]))
        end
    end
    labels = uppercasefirst.(sites)

    # The save re-runs the builder, so it has to happen under the theme too.
    with_theme(PAPER_THEME) do
        p = MakieControlPlots.plot(v_wind, heights;
            xlabel = "wind speed [m/s]", ylabel = "mean height fig8 [m]",
            labelsize = 22, legendsize = 16,
            labels = labels, fig = "fig8 height", disp = disp)
        if save && disp
            pdf_file = joinpath(PAPER_FIGURES_DIR, "fig8_height.pdf")
            savefig(pdf_file)
            @info "Saved fig8 height plot" pdf_file
        end
        return p
    end
end

plot_attractor_distance()
plot_fig8_height()
