# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Segment topology of the V3 kite: which two points each segment connects, and
whether it is `tether`, `wing` or `bridle`.

Shared by `export_v3_segments.jl`, which writes the table to
`output/v3_segments.csv`, and `simple_fig8_live.jl`, which hands it to the 3D
viewer — so the exported table and the drawn one cannot disagree.

# How the type is decided

`Segment` carries no type: the `SegmentType` enum
(`POWER_LINE`/`STEERING_LINE`/`BRIDLE`) is deprecated in SymbolicAWEModels and no
longer a constructor parameter, so the classification is structural, using the
same wing test the package's own validation uses:

1. a segment belonging to a tether (`tether.segment_idxs`) is `tether`,
2. of the rest, one with both endpoints on the wing (`point.is_wing_node`, i.e. a
   member of a twist surface) is `wing` — the leading-edge tubes, struts,
   trailing-edge wires and diagonal springs,
3. everything else is `bridle`, from the wing attachment points down to the KCU,
   including the pulley halves and the power/steering tapes.

For the V3 that gives 46 wing, 43 bridle and 6 tether segments.

Takes a built `SystemStructure` and nothing else: the caller supplies the integer
code of each type name, so this file needs neither a viewer nor GLMakie.
"""

"""
    tether_segment_idxs(sys_struct) -> Set{Int}

Indices of every segment that belongs to one of the tethers.
"""
function tether_segment_idxs(sys_struct)
    idxs = Set{Int}()
    for tether in sys_struct.tethers
        union!(idxs, tether.segment_idxs)
    end
    idxs
end

"""
    segment_type(sys_struct, segment, tether_idxs) -> String

Classify `segment` as `"tether"`, `"wing"` or `"bridle"`; `tether_idxs` is the
set of indices of all segments belonging to a tether.
"""
function segment_type(sys_struct, segment, tether_idxs)
    segment.idx in tether_idxs && return "tether"
    all(sys_struct.points[i].is_wing_node for i in segment.point_idxs) && return "wing"
    return "bridle"
end

"""
    segment_types(sys_struct) -> Vector{String}

Type of every segment of `sys_struct`, in segment order.
"""
function segment_types(sys_struct)
    tether_idxs = tether_segment_idxs(sys_struct)
    [segment_type(sys_struct, segment, tether_idxs) for segment in sys_struct.segments]
end

"""
    segment_matrix(sys_struct, type_code) -> Matrix{Int64}

The `n × 3` `(point1, point2, type)` topology matrix of `sys_struct`, with
`type_code` mapping a type name onto the integer the consumer expects —
KiteViewers' `SegmentType` values for `KiteViewers.init_segments`.
"""
function segment_matrix(sys_struct, type_code)
    types = segment_types(sys_struct)
    segments = Matrix{Int64}(undef, length(types), 3)
    for (i, (segment, type)) in enumerate(zip(sys_struct.segments, types))
        segments[i, 1], segments[i, 2] = segment.point_idxs
        segments[i, 3] = type_code[type]
    end
    segments
end
