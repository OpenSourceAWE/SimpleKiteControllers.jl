# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    fig8_log_meta.jl — store the figure-eight geometry in the log file.

The syslog records the FLOWN trajectory, but not the REFERENCE path it was
flown against: F8_A/B/C/D, the pattern-center elevation and the statistics
settle time only exist as globals of test_figure_eight.jl. A fresh session
that just loads "fig8_reference" (fig_eight_plots.jl's standalone path) therefore
had to fall back to hardcoded defaults and silently drew the wrong "desired"
path whenever the run used different values.

`fig8_colmeta` stashes those parameters as Arrow column metadata on `var_01`
(a column both wing branches always write) when the log is saved;
`read_fig8_meta` reads them back. KiteUtils' `load_log` only restores the
"name" key of each var column's metadata, so the extra keys are read straight
from the .arrow file.
"""

using KiteUtils

# Column the parameters are attached to, and the metadata key belonging to
# each of plot_fig_eight_results' geometry keywords.
const FIG8_META_COLUMN = :var_01
const FIG8_META_KEYS = (
    "F8_A" => :f8_a,
    "F8_B" => :f8_b,
    "F8_C" => :f8_c,
    "F8_D" => :f8_d,
    "SET_ELEVATION_FIG_EIGHT" => :f8_el_center,
    "SETTLE_TIME" => :settle_time,
)

"""
    fig8_colmeta(; f8_a, f8_b, f8_c, f8_d, f8_el_center, settle_time)

Build the `colmeta` dictionary for `save_log(logger, name; colmeta)`: the
KiteUtils defaults (a "name" entry for every `var_XX` column — `load_log`
reads all sixteen of them, so the full set must be present) plus the
figure-eight parameters on `FIG8_META_COLUMN`. Read back with
[`read_fig8_meta`](@ref).
"""
function fig8_colmeta(; f8_a, f8_b, f8_c, f8_d, f8_el_center, settle_time)
    params = (; f8_a, f8_b, f8_c, f8_d, f8_el_center, settle_time)
    colmeta = Dict{Symbol, Vector{Pair{String, String}}}(
        Symbol("var_$(lpad(i, 2, '0'))") => ["name" => "var_$(lpad(i, 2, '0'))"]
        for i = 1:16
    )
    append!(
        colmeta[FIG8_META_COLUMN],
        [key => string(Float64(params[sym])) for (key, sym) in FIG8_META_KEYS],
    )
    return colmeta
end

"""
    read_fig8_meta(name="fig8_reference"; path=get_data_path())

Read the figure-eight parameters stored by [`fig8_colmeta`](@ref) from the
log `name` in `path`. Returns a NamedTuple with the keyword names of
`plot_fig_eight_results` (`f8_a`, `f8_b`, `f8_c`, `f8_d`, `f8_el_center`,
`settle_time`), or `nothing` if the file does not exist or was written
before this metadata was added.
"""
function read_fig8_meta(name = "fig8_reference"; path = get_data_path())
    # Arrow arrives via KiteUtils instead of being a direct dependency of
    # examples/Project.toml; getfield avoids the "not public in KiteUtils"
    # warning of KiteUtils.Arrow.
    Arrow = getfield(KiteUtils, :Arrow)
    file = isfile(name) ? name : joinpath(path, basename(name) * ".arrow")
    isfile(file) || return nothing
    meta = Arrow.getmetadata(getproperty(Arrow.Table(file), FIG8_META_COLUMN))
    meta === nothing && return nothing
    md = Dict(meta)
    all(haskey(md, key) for (key, _) in FIG8_META_KEYS) || return nothing
    return (; (sym => parse(Float64, md[key]) for (key, sym) in FIG8_META_KEYS)...)
end
