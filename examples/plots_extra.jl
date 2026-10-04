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
    plot_attractor_distance(scenarios = ATTRACTOR_SCENARIOS; t_max = 180.0, disp = true, save = true)

Plot the attractor arc ([`attractor_arc`](@ref)) of each `(site, scenario)` pair
in `scenarios` over time, one line per run. The runs share one time axis, that of
the longest run; the others are sampled onto it, `NaN` beyond their own end.
The time axis runs from 0 to `t_max` [s].
"""
function plot_attractor_distance(scenarios = ATTRACTOR_SCENARIOS; t_max = 180.0, disp = true, save = true)
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

plot_attractor_distance()
