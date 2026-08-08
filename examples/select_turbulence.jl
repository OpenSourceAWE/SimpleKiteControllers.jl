# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu to choose the turbulence level `init` applies in
`simple_fig8.jl`: `default` (the `use_turbulence` of the selected project's
settings file) or a specific value in `[0.0, 1.0]`, where `0.0` disables
turbulence and `1.0` is the Cabauw reference level.

Writes `default_turbulence` in this package's `data/gui.yaml` — through
V3Kite's `set_default_turbulence`, so its validation and warnings are the ones
that apply — beside the other menu selections. `simple_fig8.jl` reads it back
with [`selected_turbulence`](@ref) and passes it to `init` as `use_turbulence`;
the model's own `gui.yaml` is not touched, and is read-only for an installed
V3Kite anyway.

The wind field is loaded from this package's `data/` (`windfield_*.npz`); a
level with no matching file makes the next run generate one (~1.2 GB, tens of
seconds).
"""

using V3Kite: set_default_turbulence
using SimpleKiteControllers: skc_data_path

include(joinpath(@__DIR__, "gui_state.jl"))

"""
    select_turbulence()

Ask for the turbulence level and persist it to `data/gui.yaml`.
"""
function select_turbulence()
    ensure_gui_state_file()
    set_default_turbulence(; data_path = skc_data_path())
    return nothing
end

select_turbulence()
