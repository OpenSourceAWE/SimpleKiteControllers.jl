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
written to `../LearningControl/figures/worst_loop_bode.pdf`. Then its disk margin over the
same frequencies, `diskmargin(L, 0, ω)`: the margin `α` with the robustness threshold
0.5, and the gain and phase variations each frequency's disk tolerates, written to
`../LearningControl/figures/worst_loop_disk_margin.pdf`.

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
const FIG_DIR = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))
"Frequency range of the plot [Hz]"
const F_RANGE = (0.01, 2.0)
"Frequency ticks of both figures [Hz]"
const F_TICKS = ([0.01, 0.1, 1.0], ["0.01", "0.1", "1"])
"Disk margin from which the loop is rated robust [-]"
const α_ROBUST = 0.5
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
    plot_disk_margin(L, file)

Show the disk margin of the loop `L` over frequency and save it to `file`: `α`, and the
gain [dB] and phase [deg] variations the disk of each frequency tolerates. With skew
σ = 0 the disk is symmetric in dB, so one curve each gives ± the variation.
"""
function plot_disk_margin(L, file)
    f = exp10.(range(log10.(F_RANGE)...; length = 400))
    dms = diskmargin(L, 0, 2π .* f)
    α = [d.α for d in dms]
    # From α = 2 on the disk contains every gain increase: the upper gain margin is infinite
    # (the formula turns negative), and the curve is left out there.
    gain = [d.α < 2 ? 20log10(d.gainmargin[2]) : NaN for d in dms]
    phase = [d.phasemargin for d in dms]
    # The phase margin of the disk, at its minimum: the phase variation the loop tolerates
    # together with the gain variation of the same disk.
    i = argmin(α)
    pm_label = rich("P", subscript("m"), @sprintf(" = ±%.1f°", phase[i]))
    # Its gain margin in dB, as the axis: with σ = 0 the disk is symmetric in dB, so ± one value
    # (the factors, 0.60 – 1.68 for α = 0.506, go into the caption).
    gm_label = rich("G", subscript("m"), @sprintf(" = ±%.1f dB", gain[i]))
    with_theme(COLUMN_THEME) do
        plotx(f, [α, fill(α_ROBUST, length(f))], gain, phase;
              xlabel = "Frequency [Hz]", xscale = :log10, xticks = F_TICKS, xlims = F_RANGE,
              ylabels = ["α [-]", "gain ± [dB]", "phase ± [deg]"],
              # From 0: the threshold is not on the edge. The gain grows without bound towards α = 2.
              ylims = [(0, 2.2), (0, 20), (0, 90)],
              ann = [nothing, (f[i], (0.0, gain[i]), gm_label), (f[i], (0.0, phase[i]), pm_label)],
              linestyle = [[:solid, :dot], nothing, nothing],
              color = [[:black, :gray], :black, :black],
              labelsize = LABEL_SIZE, fig = "worst_loop_disk_margin", disp = true)
        savefig(file)
    end
    return nothing
end

"""
    plot_worst_margin_bode(; dir = FIG_DIR)

Analyse the worst scenario, show its Bode plot and its disk margin over frequency, and
save both into `dir` (see the file's docstring).
"""
function plot_worst_margin_bode(; dir = FIG_DIR)
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
                  xticks = F_TICKS, fig = "worst_loop_bode", disp = true)
        mkpath(dir)
        savefig(joinpath(dir, "worst_loop_bode.pdf"))
    end
    plot_disk_margin(L, joinpath(dir, "worst_loop_disk_margin.pdf"))
    return nothing
end

plot_worst_margin_bode()
