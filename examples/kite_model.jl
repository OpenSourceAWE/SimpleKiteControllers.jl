# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Draw the structural discretisation of the V3 kite, the paper's figure of the kite model
(`LearningControl/figures/kite_model.pdf`, Sect. 3), as a vector PDF: the wing frame, the
bridle, the start of the tether, the VSM panels and the point masses, in the body frame
with its origin at the KCU.

The kite is the one the reel-out runs fly, `KITE_FIG_PROJECT`: its structural and
aerodynamic geometry and VSM settings (`struc_geometry.yaml`, `cfd_aero_geometry.yaml`,
`vsm_settings.yaml`, resolved like V3Kite does: beside the project first, V3Kite's
`data/` after), built with `load_sys_struct_from_yaml` and not settled or flown, so the
figure shows the geometry as defined. Every project of this package names the same three
files.

Drawn with CairoMakie at printed size (0.6 column width) and written to
`LearningControl/figures/` (`FIG_DIR`), next to the paper, like `plot_c1_c2.jl`. The theme
and KiteUtils' data path are restored afterwards, so the session's GLMakie plots and run
scripts are not affected.

    include("examples/kite_model.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

import CairoMakie
using CairoMakie: Figure, Axis3, Legend, LineElement, PolyElement, MarkerElement, Point3f,
                  RGBf, RGBAf, mesh!, lines!, linesegments!, scatter!, text!, rich, subscript,
                  rowgap!, with_theme, GLTriangleFace
using V3Kite
using SymbolicAWEModels
using VortexStepMethod
import KiteUtils
import YAML
using SimpleKiteControllers: skc_data_path

# Where the PDF lands: LearningControl's figures/, the document that includes it, as in
# plot_c1_c2.jl.
const FIG_DIR = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))

"The system project whose kite is drawn"
const KITE_FIG_PROJECT = "system_reelout_maasvlakte.yaml"

# 301 bp is 0.6 column width of the manuscript; drawn at printed size, so the font size holds.
const KITE_FIG_SIZE = (301, 318)
const KITE_FIG_FONT_SIZE = 10
const KITE_FIG_FONT = "Nimbus Roman"

const KITE_WING_COLOR = RGBf(0.12, 0.12, 0.12)
const KITE_BRIDLE_COLOR = RGBf(0.20, 0.45, 0.75)
const KITE_TETHER_COLOR = RGBf(0.45, 0.45, 0.45)
const KITE_PANEL_FILL = RGBAf(0.85, 0.30, 0.25, 0.22)
const KITE_PANEL_EDGE = RGBAf(0.55, 0.20, 0.16, 0.55)
const KITE_POINT_COLOR = RGBf(0.55, 0.10, 0.10)
"Index of the KCU point, the origin of the figure"
const KCU_IDX = 1

"""
    load_kite_model(project = KITE_FIG_PROJECT) -> SystemStructure

The kite of the system project `project` as a particle model, with its VSM wing. Each
geometry file is resolved as V3Kite resolves it (`V3Kite.project_file`). KiteUtils' data
path is pointed at this package's `data/` to read the settings and restored afterwards.
"""
function load_kite_model(project = KITE_FIG_PROJECT)
    path = joinpath(skc_data_path(), project)
    system = YAML.load_file(path)["system"]
    resolve(key) = V3Kite.project_file(path, system[key])
    data_path0 = KiteUtils.get_data_path()
    try
        set_data_path(skc_data_path())
        set = Settings(path)
        vsm_set = VortexStepMethod.VSMSettings(resolve("vsm_settings"); data_prefix = false)
        vsm_set.wings[1].geometry_file = resolve("aero_geometry")
        return load_sys_struct_from_yaml(resolve("structural_geometry");
            system_name = "figure", set, dynamics_type = PARTICLE_DYNAMICS, vsm_set)
    finally
        set_data_path(data_path0)
    end
end

"""
    segment_group(sys, tether_segments, idx) -> Symbol

Whether segment `idx` of `sys` belongs to the wing frame, the bridle or the tether.
"""
function segment_group(sys, tether_segments, idx)
    idx in tether_segments && return :tether
    seg = sys.segments[idx]
    all(sys.points[j].is_wing_node for j in seg.point_idxs) ? :wing : :bridle
end

"""
    plot_kite_model(sys) -> Figure

The figure of the kite `sys`: wing frame, bridle and tether segments, VSM panels and
point masses in the body frame, the tether cut at the KCU.
"""
function plot_kite_model(sys)
    wing = sys.wings[1]
    kcu_pos = sys.points[KCU_IDX].pos_w
    to_body(pos_w) = Point3f(wing.R_b_to_w' * (pos_w - kcu_pos))
    tether_segments = Set(Int.(sys.tethers[1].segment_idxs))
    group_color = Dict(:wing => KITE_WING_COLOR, :bridle => KITE_BRIDLE_COLOR,
                       :tether => KITE_TETHER_COLOR)

    # The tether runs to the ground station, far outside the frame the kite fills.
    tether_points = union(Set{Int}(),
        (Set(Int.(sys.segments[i].point_idxs)) for i in tether_segments)...)
    kite_idxs = setdiff(eachindex(sys.points), setdiff(tether_points, KCU_IDX))

    positions = [to_body(sys.points[i].pos_w) for i in kite_idxs]
    lo = reduce((a, b) -> min.(a, b), positions)
    hi = reduce((a, b) -> max.(a, b), positions)
    pad = 0.05 * maximum(hi .- lo)

    fig = Figure(size = KITE_FIG_SIZE, figure_padding = (2, 2, 2, 2))
    ax = Axis3(fig[1, 1]; aspect = :data, azimuth = -2.45, elevation = 0.12,
               xlabel = rich("x", subscript("B"), " [m]"),
               ylabel = rich("y", subscript("B"), " [m]"),
               zlabel = rich("z", subscript("B"), " [m]"),
               xlabeloffset = 21, ylabeloffset = 24, zlabeloffset = 32,
               xticks = [0, 2], yticks = [-2.5, 0.0, 2.5], zticks = [0, 5, 10],
               limits = (lo[1] - pad, hi[1] + pad, lo[2] - pad, hi[2] + pad,
                         lo[3] - pad, hi[3] + pad))

    for panel in wing.vsm_aero.panels
        corners = [to_body(wing.R_b_to_w * panel.corner_points[:, i] + wing.pos_w)
                   for i in 1:4]
        mesh!(ax, corners, [GLTriangleFace(1, 2, 3), GLTriangleFace(1, 3, 4)];
              color = KITE_PANEL_FILL, transparency = true)
        lines!(ax, [corners..., corners[1]]; color = KITE_PANEL_EDGE, linewidth = 0.25)
    end

    for group in (:tether, :bridle, :wing)
        pts = Point3f[]
        for (idx, seg) in enumerate(sys.segments)
            segment_group(sys, tether_segments, idx) == group || continue
            push!(pts, to_body(sys.points[seg.point_idxs[1]].pos_w))
            push!(pts, to_body(sys.points[seg.point_idxs[2]].pos_w))
        end
        linesegments!(ax, pts; color = group_color[group], linewidth = 0.7)
    end
    scatter!(ax, positions; color = KITE_POINT_COLOR, markersize = 3)

    text!(ax, to_body(kcu_pos); text = "KCU", color = :black,
          align = (:left, :center), offset = (5, 3))

    Legend(fig[2, 1],
           [LineElement(color = KITE_WING_COLOR, linewidth = 1.2),
            LineElement(color = KITE_BRIDLE_COLOR, linewidth = 1.2),
            LineElement(color = KITE_TETHER_COLOR, linewidth = 1.2),
            PolyElement(color = KITE_PANEL_FILL, strokecolor = KITE_PANEL_EDGE, strokewidth = 0.5),
            MarkerElement(color = KITE_POINT_COLOR, marker = :circle, markersize = 4)],
           ["wing frame", "bridle", "tether", "VSM panels", "point masses"];
           orientation = :horizontal, nbanks = 2, framevisible = false,
           patchsize = (16, 9), padding = (0, 0, 0, 0))
    rowgap!(fig.layout, 4)
    @info "Kite model" points = length(kite_idxs) segments = length(sys.segments) panels = length(wing.vsm_aero.panels)
    return fig
end

let sys = load_kite_model()
    fonts = (; regular = KITE_FIG_FONT, bold = "$KITE_FIG_FONT Bold",
               italic = "$KITE_FIG_FONT Italic", bold_italic = "$KITE_FIG_FONT Bold Italic")
    # with_theme, not set_theme!: the session's other plots keep their theme.
    with_theme(; fontsize = KITE_FIG_FONT_SIZE, fonts) do
        fig = plot_kite_model(sys)
        mkpath(FIG_DIR)
        file = joinpath(FIG_DIR, "kite_model.pdf")
        # backend = CairoMakie: a vector PDF, whichever backend the session displays with.
        CairoMakie.save(file, fig; backend = CairoMakie, pt_per_unit = 1)
        @info "Wrote $file."
    end
end
