# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive checkbox menu to choose which figures `simple_fig8_plots.jl`
shows: `pattern`, `time series` and `aerodynamics`.

Writes the choice to `data/menu_state.yaml` via [`set_selected_plots`](@ref)
(`examples/menu_state.jl`) rather than a `Main` global, so `simple_fig8_plots.jl`
reads the current selection straight off disk on every `include`.
"""

using REPL.TerminalMenus

include(joinpath(@__DIR__, "menu_state.jl"))

const PLOT_LABELS = ["pattern", "time series", "aerodynamics"]
const PLOT_KEYS = ["pattern", "time_series", "aerodynamics"]

"""
    select_plots()

Ask which figures to show via a checkbox menu and persist the choice to
`data/menu_state.yaml`.
"""
function select_plots()
    current = selected_plots()
    checked = Set(findall(in(current), PLOT_KEYS))
    menu = MultiSelectMenu(PLOT_LABELS; selected = checked)
    chosen = request("\nSelect plots to show: ", menu)
    plots = PLOT_KEYS[sort(collect(chosen))]
    set_selected_plots(plots)
    println("Plots set to: ", isempty(plots) ? "none" : join(plots, ", "))
    return nothing
end

select_plots()
