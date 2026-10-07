# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Generate Fig. 14 of the LearningControl paper: the three-dimensional view of
the figure-of-eight pattern flown in the Cabauw scenario at a ground wind
speed of 5.75 m/s with feedforward control (`output/scenarios/cabauw/v05.75`,
the run of `cabauw_5.75_fb.pdf`). Saves `pattern_cabauw_5.8ms.png` into
`../LearningControl/figures`, next to this package's repo.

The figure is built by [`build_path3d_figure`](@ref), rendered with GLMakie and saved as a PNG at
twice the figure size, since the paper includes the PNG. The camera is turned
from the `Axis3` default (azimuth 1.275π, elevation π/8) to the view chosen
for the paper: from lower and more from the front, so that the figures of
eight are seen face-on. The run ends with a climb towards the zenith; the
path is cut where the kite first passes `z_max`, so the figure shows the
figures of eight and not the final climb.

    include("plot_path3d_paper.jl")
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

"""
    PAPER_THEME

Makie theme for the figures that go into the LearningControl paper, kept in
step with the copies in `plot_patterns_paper.jl`, `plot_powercurve.jl` and
`plots_extra.jl`, so every figure of the paper is set in one face.
"""
const PAPER_THEME = Theme(
    fonts = (; regular = "TeX Gyre Termes", bold = "TeX Gyre Termes Bold",
               italic = "TeX Gyre Termes Italic"),
    Axis = (; xticklabelsize = 20, yticklabelsize = 20),
)

"""
    create_paper_path3d_plot(; site = "cabauw", scenario = "v05.75",
                             out_file = "pattern_cabauw_5.8ms.png",
                             azimuth = 1.128π, elevation = 0.105π,
                             ticklabelsize = 22, labelsize = 26,
                             legendsize = 22, z_max = 320.0,
                             size = (850, 840), px_per_unit = 2) -> String

Load the archived log of `output/scenarios/<site>/<scenario>`, build the 3D
flight-path figure of the run up to the first sample where the height of the
kite exceeds `z_max` [m] under [`PAPER_THEME`](@ref), with the z axis ending
at `z_max`, so the end marker and the straight tether go to that point and
not to the end of the run. View it from `azimuth` and
`elevation` [rad], display it and save it as `out_file` in
[`PAPER_FIGURES_DIR`](@ref). Returns the path of the saved file.

The figure is set at 0.6 of the text width, so the font sizes of
[`build_path3d_figure`](@ref), chosen for a window on screen, are raised here:
`ticklabelsize` for the ticks of the axes and the colorbar, `labelsize` for
the axis and colorbar labels and `legendsize` for the legend entries. The
axis box keeps the data aspect ratio, so its width follows from the height;
`size` [px] is narrowed from the 1000 px of `build_path3d_figure` to keep the
colorbar close to the box.
"""
function create_paper_path3d_plot(; site = "cabauw", scenario = "v05.75",
                                  out_file = "pattern_cabauw_5.8ms.png",
                                  azimuth = 1.128π, elevation = 0.105π,
                                  ticklabelsize = 22, labelsize = 26,
                                  legendsize = 22, z_max = 320.0,
                                  size = (850, 840), px_per_unit = 2)
    isdir(PAPER_FIGURES_DIR) || error("$PAPER_FIGURES_DIR does not exist.")
    scenario_dir = normpath(joinpath(@__DIR__, "..", "output", "scenarios", site, scenario))
    isdir(scenario_dir) || error("$scenario_dir does not exist.")

    GLMakie.activate!()
    png_file = joinpath(PAPER_FIGURES_DIR, out_file)
    with_theme(PAPER_THEME) do
        log_name = replace(only(filter(f -> endswith(f, ".arrow"), readdir(scenario_dir))),
                           ".arrow" => "")
        sl = load_log(log_name; path = scenario_dir).syslog
        # height of the kite as in `build_path3d_figure`: the centre of pressure
        # of the mid-span wing points 10..13
        z_kite = (0.7 .* getindex.(sl.Z, 10) .+ 0.3 .* getindex.(sl.Z, 11) .+
                  0.7 .* getindex.(sl.Z, 12) .+ 0.3 .* getindex.(sl.Z, 13)) ./ 2
        i_cut = findnext(>(z_max), z_kite, 2)
        rng = 2:(isnothing(i_cut) ? length(sl.time) : i_cut - 1)
        fig, _ = build_path3d_figure(sl, rng)
        resize!(fig, size...)
        ax = content(fig[1, 1])
        zlims!(ax, nothing, z_max)
        ax.azimuth[] = azimuth
        ax.elevation[] = elevation
        for name in (:x, :y, :z)
            getproperty(ax, Symbol(name, :ticklabelsize))[] = ticklabelsize
            getproperty(ax, Symbol(name, :labelsize))[] = labelsize
            # keep the larger labels clear of the larger tick labels
            getproperty(ax, Symbol(name, :labeloffset))[] = 1.6 * labelsize + 15
        end
        # room below the axis box for the y label, which would be cut off
        # otherwise, and above it, so the frame of the box stays clear of the
        # legend; the wide left margin moves the box towards the colorbar
        ax.protrusions[] = (110, 30, 3 * labelsize, 90)
        cbar = content(fig[1, 2])
        cbar.ticklabelsize[] = ticklabelsize
        cbar.labelsize[] = labelsize
        # The legend frame does not follow a later change of the label size,
        # so the legend is replaced by one built with the paper size.
        delete!(only(filter(c -> c isa Legend, fig.content)))
        axislegend(ax; position = :lt, labelsize = legendsize)
        display(fig)
        GLMakie.save(png_file, fig; px_per_unit)
    end
    @info "Saved 3D pattern plot" png_file
    return png_file
end

create_paper_path3d_plot()
