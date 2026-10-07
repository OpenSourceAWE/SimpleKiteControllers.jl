# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Bode plot of the guided course loop at the worst disk margin of all reel-out scenarios,
for the LearningControl paper.

The worst scenario is the one marked `← worst` with the lowest `α guided` in the two
sites' `stability_overview.md`, as `stability_global.jl` wrote them; run that first after
a retune. Its folder is re-analysed with `stability_opt_reelout.jl` (plots off, output
muted), which leaves the worst bin's loop `L` in `Main`. Its Bode plot, drawn with
`MakieControlPlots.bode_plot` from 0.01 to 2 Hz with the phase shifted by -360° and
reference lines at 0 dB and -180°, is shown in a window and
written to `../LearningControl/figures/worst_loop_bode.pdf`.

    include("examples/plot_worst_margin_bode.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers: run_example, muted, latest_global, set_selected_project, read_gui_field
using ControlSystemsBase, RobustAndOptimalControl
using GLMakie
using MakieControlPlots
using Printf

const SITE_PROJECTS = [
    "maasvlakte" => "system_reelout_maasvlakte.yaml",
    "cabauw" => "system_reelout_cabauw.yaml",
]
const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))
const FIG_FILE = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures", "worst_loop_bode.pdf"))
"Frequency range of the plot [Hz]"
const F_RANGE = (0.01, 2.0)
"Axis label size [pt]; `bode_plot` draws 768 px wide, scaled to one column of the paper, as in `plot_c1_c2.jl`"
const LABEL_SIZE = 26

"""
    COLUMN_THEME

TeX Gyre Termes, the face of the paper's body text, with tick labels sized for a
figure that fills one column, as in `plot_c1_c2.jl`.
"""
const COLUMN_THEME = Theme(
    fonts = (; regular = "TeX Gyre Termes", bold = "TeX Gyre Termes Bold",
               italic = "TeX Gyre Termes Italic"),
    Axis = (; xticklabelsize = 24, yticklabelsize = 24),
)

"""
    worst_scenario() -> NamedTuple

The scenario with the lowest guided disk margin over the sites of `SITE_PROJECTS`, read
from their `stability_overview.md`: `site`, `project`, `name`, `α`.
"""
function worst_scenario()
    candidates = map(SITE_PROJECTS) do (site, project)
        file = joinpath(SCENARIOS_DIR, site, "stability_overview.md")
        isfile(file) || error("No $file: run stability_global.jl first.")
        line = only(filter(contains("← worst"), readlines(file)))
        cells = strip.(split(line, '|'))
        (; site, project, name = cells[2], α = parse(Float64, cells[5]))
    end
    return argmin(c -> c.α, candidates)
end

"""
    plot_worst_margin_bode(; file = FIG_FILE)

Analyse the worst scenario, show its Bode plot and save it (see the file's docstring).
"""
function plot_worst_margin_bode(; file = FIG_FILE)
    ws = worst_scenario()
    project0 = read_gui_field("project")
    try
        set_selected_project(ws.project)
        muted(() -> run_example("stability_opt_reelout.jl"; show_plots = false, project = ws.project,
                                log_dir = joinpath(SCENARIOS_DIR, ws.site, ws.name)))
    finally
        set_selected_project(project0)
    end
    worst, L = latest_global(:worst), latest_global(:L)
    dm = diskmargin(L)
    @info @sprintf("%s/%s: α = %.3f at %.2f Hz (gain %.2f – %.2f, phase ± %.1f°), L = %.0f m, v_a = %.1f m/s, depower = %.3f",
                   ws.site, ws.name, dm.margin, dm.ω0 / 2π, dm.gainmargin..., dm.phasemargin,
                   worst.L, worst.va, worst.dp)
    # The save re-runs the builder, so it has to happen under the theme too.
    with_theme(COLUMN_THEME) do
        # No title: the caption of the paper names the operating point. `bode` unwraps the phase
        # from 0.01 Hz, where the guidance's integrator and the gravity pole put it a turn above
        # the usual range: -360° moves the margins to -180°.
        bode_plot(L; from = log10(2π * F_RANGE[1]), to = log10(2π * F_RANGE[2]), hz = true, bw = true, fontsize = LABEL_SIZE,
                  show_title = false, phase_offset = -360, ref_lines = true,
                  xticks = ([0.01, 0.1, 1.0], ["0.01", "0.1", "1"]), fig = "worst_loop_bode", disp = true)
        mkpath(dirname(file))
        savefig(file)
    end
    return nothing
end

plot_worst_margin_bode()
