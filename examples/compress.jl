# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Shrink a saved `.arrow` flight log by dropping the VSM panel-corner slots from
its `X`/`Y`/`Z` position arrays — about 78 % of a reel-out log, see
`docs/log_size.md`. Optionally also decimates the rows (`every = n`).

The corners are pure visualisation data written by
`SymbolicAWEModels.write_aero_log_points!`; nothing in the replay path reads
them. `KiteViewers.update_segments!` only touches slots `1:n_points`, and
`load_log` takes the position count from the file itself
(`P = length(table.Z[1])`), so a shortened log loads and replays unchanged —
no flag or version bump anywhere upstream.

The slot layout is `points | panel_corners | wing origins | body origins`
(`SymbolicAWEModels.position_slots`). This keeps the structural points and the
trailing origins and cuts the corner block out of the middle, so the origins
move down from slot 333 to 45. Only code that recomputes `position_slots` from
a live model would notice; the viewer does not.

    include("compress.jl")
    compress_log("../output/reelout_150m_opt.arrow")                # in place
    compress_log(log; outfile = "small.arrow", every = 2)           # + 45 Hz

WARNING: with no `outfile` the source file is replaced (written to a temp file
next to it first, then renamed, so a failure leaves the original intact).
Decimation aliases the fast signals — see the caveat in `docs/log_size.md`;
treat a resampled log as a viewing artefact, not as a scoring input.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
using KiteUtils
using StaticArrays
using Printf

# `selected_reelout_project`, the default source of the settings whose
# `segments` decide how many tether nodes the point block holds.
include(joinpath(@__DIR__, "gui_state.jl"))

const SA = KiteUtils.StructArrays
const Arrow = KiteUtils.Arrow

"""
    n_struct_points(; project = nothing)

How many leading `X`/`Y`/`Z` slots of the V3 model are structural points —
everything after them is panel corners, then the wing/body origins the log
itself accounts for (see [`compress_log`](@ref)).

Built the cheap way `simple_reelout_play.jl` builds its topology —
`load_sys_struct_from_yaml` with `AeroNone`, the YAML half of `init` with no
model build — because only the count is wanted. `project` names a
`system_*.yaml` under this package's `data/`; its settings decide the number of
tether interior nodes (`segments`), hence the point count.
"""
function n_struct_points(; project = nothing)
    data_path = get_data_path()   # a caller mid-workflow keeps its own data path
    try
        set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
        isnothing(project) && (project = selected_reelout_project())
        set = Settings(project_file(project))
        sys_struct = load_sys_struct_from_yaml(joinpath(v3_data_path(), "struc_geometry.yaml");
            set, aero_mode = SymbolicAWEModels.AeroNone())
        return length(sys_struct.points)
    finally
        set_data_path(data_path)
    end
end

"""
    log_colmeta(fullname, loaded_colmeta)

Column metadata for the rewritten log: the `var_01..var_16` names `load_log`
returns, back in the `["name" => ...]` shape `Arrow.write` wants, plus the
`created_at` stamp `V3Kite.timestamp_colmeta` puts on the `time` column, which
`load_log` drops on the floor.
"""
function log_colmeta(fullname::AbstractString, loaded_colmeta::AbstractDict)
    colmeta = Dict{Symbol, Union{String, Vector{Pair{String, String}}}}(
        k => (v isa AbstractString ? ["name" => v] : v) for (k, v) in loaded_colmeta)
    meta = Arrow.getmetadata(Arrow.Table(fullname).time)
    created_at = isnothing(meta) ? nothing : get(meta, "created_at", nothing)
    isnothing(created_at) ||
        (colmeta[:time] = ["name" => "time", "created_at" => created_at])
    return colmeta
end

"""
    compress_log(infile; outfile = nothing, n_points = nothing, n_origins = nothing,
                 project = nothing, every = 1, keep_origins = true, compress = true)

Rewrite the `.arrow` log at `infile` without its VSM panel corners, replacing it
unless `outfile` is given. Returns the path written.

# Keyword arguments
- `outfile`: destination; `nothing` replaces `infile` (via a temp file + rename).
- `n_points`: leading structural-point slots, by default taken from the model
  with [`n_struct_points`](@ref). Pass it to compress a log of a different
  system without loading that system.
- `n_origins`: trailing wing/body origin slots. By default read off the log —
  `SymbolicAWEModels` logs one orientation frame per wing and per body, and one
  origin slot each, so the length of the `Qw` column is the count. (The V3
  reel-out logs carry two: the wing, plus a co-located rigid body.)
- `project`: `system_*.yaml` handed to `slot_layout`; defaults to the selected
  reel-out project.
- `every`: keep every n-th row as well (`1` = all rows, full rate).
- `keep_origins`: keep the trailing wing/body origin slots. `false` drops them
  too, leaving `X`/`Y`/`Z` at exactly the structural points.
- `compress`: LZ4 the arrow file, as `save_log` does by default.
"""
function compress_log(infile::AbstractString; outfile = nothing, n_points = nothing,
                      n_origins = nothing, project = nothing, every::Int = 1,
                      keep_origins::Bool = true, compress::Bool = true)
    isfile(infile) || error("No such log: $infile")
    every >= 1 || error("every must be >= 1, got $every")
    isnothing(n_points) && (n_points = n_struct_points(; project))

    syslog = load_log(abspath(infile))
    sl = syslog.syslog
    P = length(sl.X[1])
    isnothing(n_origins) && (n_origins = length(sl.Qw[1]))
    n_corners = P - n_points - n_origins
    n_corners >= 0 || error("Log has $P position slots, fewer than the $n_points \
                             points + $n_origins origins it is taken to hold.")
    # VSM logs 4 corners per panel; anything else means the counts are wrong for
    # this log and cutting would eat real points.
    n_corners % 4 == 0 || error("$(n_corners) slots between the $n_points points and \
                                 the $n_origins origins of $infile is not a whole number \
                                 of 4-corner panels — pass n_points/n_origins explicitly.")
    if n_corners == 0 && every == 1
        @info "$infile carries no panel corners already — nothing to do."
        return infile
    end

    keep = keep_origins ? [1:n_points; (P - n_origins + 1):P] : collect(1:n_points)
    rows = 1:every:length(sl)
    P2 = length(keep)
    O = length(sl.Qw[1])
    D = length(sl.flap_angle[1])

    cols = SA.components(sl)
    thinned = every == 1 ? cols : map(c -> c[rows], cols)
    cut(c) = [MVector{P2, KiteUtils.MyFloat}(view(c[i], keep)) for i in rows]
    newcols = merge(thinned, (X = cut(cols.X), Y = cut(cols.Y), Z = cut(cols.Z)))
    small = KiteUtils.SysLog{P2}(syslog.name,
        log_colmeta(abspath(infile), syslog.colmeta),
        SA.StructArray{SysState{P2, O, D}}(newcols))

    dest = isnothing(outfile) ? infile : outfile
    endswith(dest, ".arrow") || error("Output must be an .arrow file, got $dest")
    # save_log names the file after the log; write beside `dest` under a temp
    # name so an in-place run cannot shred the file it is still mmap-reading.
    tmp_name = replace(basename(dest), r"\.arrow$" => "") * ".compress_tmp"
    tmp_path = joinpath(dirname(abspath(dest)), tmp_name * ".arrow")
    small.name = tmp_name
    save_log(small, compress; path = dirname(abspath(dest)))
    before = filesize(infile)
    mv(tmp_path, abspath(dest); force = true)
    after = filesize(dest)

    @info @sprintf("%s: %d -> %d position slots%s, %.1f MB -> %.1f MB (%.1fx).",
        basename(dest), P, P2, every == 1 ? "" : @sprintf(", every %d-th row", every),
        before / 1e6, after / 1e6, before / after)
    return dest
end

"""
    compress_scenario(dir; kwargs...)

Run [`compress_log`](@ref) over every `.arrow` file in `dir` — e.g. one
`SimulationResults` scenario folder, or all of them at once by pointing this at
the parent and mapping it yourself. `kwargs` go to `compress_log`.
"""
function compress_scenario(dir::AbstractString; n_points = nothing, project = nothing,
                           kwargs...)
    logs = filter(f -> endswith(f, ".arrow"), readdir(dir; join = true))
    isempty(logs) && @warn "No .arrow log in $dir"
    # Resolved once: every log in a scenario folder is the same system.
    if !isempty(logs) && isnothing(n_points)
        n_points = n_struct_points(; project)
    end
    return [compress_log(log; n_points, kwargs...) for log in logs]
end

nothing
