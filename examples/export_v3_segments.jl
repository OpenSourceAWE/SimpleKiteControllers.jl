# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Export the V3 kite's segment table to `output/v3_segments.csv`.

Columns: `segment`, `point1`, `point2`, `segment_type`, one row per segment of
the model, with `segment` the segment index (1…n) and `point1`/`point2` the
indices of the two points it connects.

# Where the table comes from

The structure is built from V3Kite's `data/struc_geometry.yaml` with
`load_sys_struct_from_yaml`, which is the cheap half of `init` — the same YAML
loader, but no model build, so this runs in a second instead of minutes. What it
adds over the raw YAML matters here: `expand_auto_tethers!` turns the single
`tethers:` entry (start point 1, end point 39, `n_segments` 6) into 5
intermediate points and the 6 tether segments, so the file the sim flies has 44
points and 95 segments where the YAML lists 39 and 89. Exported from the YAML
alone, the table would contain no tether rows at all.

The kite is settled and trimmed before it is flown, and `init` loads the
resulting `struc_geometry_depower*_tip*_te*.yaml` from V3Kite's cache rather
than `data/struc_geometry.yaml`. That variant differs in point positions and
rest lengths only — the topology this script exports is the same, and the
committed file is the one that always exists.

# How the type is decided

Structurally, by `examples/v3_segments.jl` — a segment of a tether is `tether`,
one with both endpoints on the wing is `wing`, everything else is `bridle`, which
for the V3 gives 46 wing, 43 bridle and 6 tether segments. The same file feeds
the live viewer in `simple_fig8_live.jl`, so the exported table and the drawn one
cannot disagree.

Run from the menu, or by

    include("examples/export_v3_segments.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using V3Kite: v3_data_path, load_sys_struct_from_yaml, set_data_path, Settings, SymbolicAWEModels
using SimpleKiteControllers: project_file

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
include(joinpath(@__DIR__, "v3_segments.jl"))

const STRUC_YAML = joinpath(v3_data_path(), "struc_geometry.yaml")

"""
    export_segments(sys_struct, csv_file) -> Vector{String}

Write the `segment, point1, point2, segment_type` table of `sys_struct` to
`csv_file` and return the type of each segment, in the same order.
"""
function export_segments(sys_struct, csv_file)
    types = segment_types(sys_struct)
    open(csv_file, "w") do io
        println(io, "segment,point1,point2,segment_type")
        for (segment, type) in zip(sys_struct.segments, types)
            point1, point2 = segment.point_idxs
            println(io, "$(segment.idx),$point1,$point2,$type")
        end
    end
    return types
end

project_set = Settings(project_file(selected_project()))
# AeroNone: the wing's aerodynamics do not touch the topology exported here, and it is the one
# aero mode that needs no VSM geometry — so no vsm_settings/aero_geometry/polars are loaded.
sys_struct = load_sys_struct_from_yaml(STRUC_YAML; set=project_set,
                                       aero_mode=SymbolicAWEModels.AeroNone())

output_path = normpath(joinpath(@__DIR__, "..", "output"))
mkpath(output_path)
csv_file = joinpath(output_path, "v3_segments.csv")
types = export_segments(sys_struct, csv_file)

counts = join(["$(count(==(type), types)) $type" for type in ("wing", "bridle", "tether")], ", ")
@info "export_v3_segments.jl: wrote $(length(types)) segments ($counts) of \
       $(length(sys_struct.points)) points to $csv_file."
