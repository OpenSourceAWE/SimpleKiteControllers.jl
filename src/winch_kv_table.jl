# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    winch_kv(v_wind; project = project_file()) -> Float64

Wind-speed-dependent `kv` for WinchControllers.jl's `v_set = kv * sqrt(force)`
reel-out law, linearly interpolated between the rows of the system project's
`winch_kv_table` file ([`winch_kv_table_file`](@ref)). Clamped to the
lowest/highest identified wind speed outside that range, never extrapolated.
Overrides the flat `kv` of `data/wc_settings.yaml`; the caller assigns the
result to `WCSettings.kv`. Only reel-out projects carry a `winch_kv_table`
entry, so calling this against a fig8 project throws.
"""
function winch_kv(v_wind; project = project_file())
    raw = YAML.load_file(joinpath(skc_data_path(), winch_kv_table_file(project)))
    entries = sort([(Float64(e["v_wind"]), Float64(e["kv"])) for e in raw["entries"]]; by = first)
    winds, kvs = first.(entries), last.(entries)
    v = Float64(v_wind)
    v <= first(winds) && return first(kvs)
    v >= last(winds) && return last(kvs)
    i = searchsortedlast(winds, v)
    v0, k0 = entries[i]
    v1, k1 = entries[i + 1]
    return k0 + (v - v0) / (v1 - v0) * (k1 - k0)
end
