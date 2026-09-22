# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Generate the figure-of-eight pattern plots used in the LearningControl paper:
the Maasvlakte and Cabauw scenarios at a given ground wind speed
(`output/scenarios/maasvlakte/<v>` and `output/scenarios/cabauw/<v>`). Saves
`<site>_<v>_pattern.pdf` into `../LearningControl/figures`, next to this
package's repo.

Also generates the feedforward-comparison pair `cabauw_5.75_fb.pdf` (from
`v05.75`) and `cabauw_5.75_nofb.pdf` (from `v05.75_2`), both drawn with a
fixed azimuth range of -28 to 27 deg and elevation range of 13 to 38 deg so
the two subfigures share one axis scale.

All entries of [`PAPER_SCENARIOS`](@ref) are `enabled`; flip `enabled = false`
on any entry to skip it.

    include("plot_patterns_paper.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using GLMakie
using MakieControlPlots
using LaTeXStrings
using V3Kite
using SimpleKiteControllers

# Include utility function for pattern plotting
include(joinpath(@__DIR__, "plot_pattern_utils.jl"))

const PAPER_FIGURES_DIR = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))

const PAPER_SCENARIOS = [
    (site = "maasvlakte", scenario = "v04", project = "system_reelout_maasvlakte.yaml",
     out_file = "maasvlakte_4.0_pattern.pdf", enabled = true, xlims = nothing, ylims = nothing),
    (site = "cabauw", scenario = "v04", project = "system_reelout_cabauw.yaml",
     out_file = "cabauw_4.0_pattern.pdf", enabled = true, xlims = nothing, ylims = nothing),
    (site = "maasvlakte", scenario = "v07", project = "system_reelout_maasvlakte.yaml",
     out_file = "maasvlakte_7.0_pattern.pdf", enabled = true, xlims = nothing, ylims = nothing),
    (site = "cabauw", scenario = "v07", project = "system_reelout_cabauw.yaml",
     out_file = "cabauw_7.0_pattern.pdf", enabled = true, xlims = nothing, ylims = nothing),
    (site = "maasvlakte", scenario = "v10", project = "system_reelout_maasvlakte.yaml",
     out_file = "maasvlakte_10.0_pattern.pdf", enabled = true, xlims = nothing, ylims = nothing),
    (site = "cabauw", scenario = "v10", project = "system_reelout_cabauw.yaml",
     out_file = "cabauw_10.0_pattern.pdf", enabled = true, xlims = nothing, ylims = nothing),
    # With feedforward control (v05.75) vs. without (v05.75_2), sharing one axis
    # scale so the two subfigures in the paper compare directly.
    (site = "cabauw", scenario = "v05.75", project = "system_reelout_cabauw.yaml",
     out_file = "cabauw_5.75_fb.pdf", enabled = true, xlims = (-28, 27), ylims = (13, 38)),
    (site = "cabauw", scenario = "v05.75_2", project = "system_reelout_cabauw.yaml",
     out_file = "cabauw_5.75_nofb.pdf", enabled = true, xlims = (-28, 27), ylims = (13, 38)),
]

"""
    create_paper_pattern_plots()

For each `enabled` scenario in [`PAPER_SCENARIOS`](@ref), load its archived
log from `output/scenarios/<site>/<scenario>` and render the pattern plot
with [`plot_pattern_scenario`](@ref), saving it as a PDF in
[`PAPER_FIGURES_DIR`](@ref). A disabled entry is skipped. Each plot is shown
(`disp=true`) so `MakieControlPlots.savefig` captures the right figure, then
the window is closed immediately via `MakieControlPlots.close`.
"""
function create_paper_pattern_plots()
    isdir(PAPER_FIGURES_DIR) || error("$PAPER_FIGURES_DIR does not exist.")

    for s in PAPER_SCENARIOS
        s.enabled || continue

        scenario_dir = normpath(joinpath(@__DIR__, "..", "output", "scenarios", s.site, s.scenario))
        isdir(scenario_dir) || error("$scenario_dir does not exist.")

        p = plot_pattern_scenario(scenario_dir; disp = true,
                                  project = joinpath(scenario_dir, s.project),
                                  xlims = s.xlims, ylims = s.ylims)

        pdf_file = joinpath(PAPER_FIGURES_DIR, s.out_file)
        savefig(pdf_file)
        MakieControlPlots.close(p.fig)

        @info "Saved pattern plot" pdf_file
    end

    @info "Paper pattern plot generation complete"
end

create_paper_pattern_plots()
