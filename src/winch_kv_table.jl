# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    winch_table_lookup(v_wind, key; project = project_file()) -> Float64

Linearly interpolate column `key` of the system project's `winch_kv_table` file
([`winch_kv_table_file`](@ref)) against mean wind speed. Clamped to the
lowest/highest identified wind speed outside that range, never extrapolated.
Only reel-out projects carry a `winch_kv_table` entry, so calling this against a
fig8 project throws, as does a table missing `key` in any row.
"""
function winch_table_lookup(v_wind, key; project = project_file())
    file = joinpath(skc_data_path(), winch_kv_table_file(project))
    raw = YAML.load_file(file)
    rows = raw["entries"]
    all(r -> haskey(r, key), rows) ||
        error("$file: every entry needs a \"$key\" column.")
    entries = sort([(Float64(r["v_wind"]), Float64(r[key])) for r in rows]; by = first)
    winds = first.(entries)
    v = Float64(v_wind)
    v <= first(winds) && return last(first(entries))
    v >= last(winds) && return last(last(entries))
    i = searchsortedlast(winds, v)
    v0, y0 = entries[i]
    v1, y1 = entries[i + 1]
    return y0 + (v - v0) / (v1 - v0) * (y1 - y0)
end

"""
    winch_kv(v_wind; project = project_file()) -> Float64

Wind-speed-dependent `kv` for WinchControllers.jl's `v_set = kv * sqrt(force)`
reel-out law, from the `kv` column of the project's `winch_kv_table` file (see
[`winch_table_lookup`](@ref)). Overrides the flat `kv` of
`data/wc_settings.yaml`; the caller assigns the result to `WCSettings.kv`.
"""
winch_kv(v_wind; project = project_file()) =
    winch_table_lookup(v_wind, "kv"; project)

"""
    winch_f_low(v_wind; project = project_file()) -> Float64

Wind-speed-dependent force floor [N] below which WinchControllers.jl's
`LowerForceController` reels in, from the `f_low` column of the project's
`winch_kv_table` file (see [`winch_table_lookup`](@ref)). Overrides the flat
`f_low` of `data/wc_settings.yaml`; the caller assigns the result to
`WCSettings.f_low`.

It is wind-dependent because the two things it sits between scale differently:
the winch's rating does not move with the wind, while the force a kite can
produce goes with its square. A single value sized for strong wind takes over
the run in weak wind — measured at 3 m/s, `f_low` 700 N put the limiter in
control 63 % of the time against 21 % at 350 N. NOT the entry guard's floor,
which is `FC_Settings.entry_f_min`; see `docs/fig8_tuning_log.md`.
"""
winch_f_low(v_wind; project = project_file()) =
    winch_table_lookup(v_wind, "f_low"; project)
