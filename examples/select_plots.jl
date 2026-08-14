# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive checkbox menu to choose which figures `simple_fig8_plots.jl`
shows: `pattern`, `time series` and `aerodynamics`. `power` is
`simple_reelout_plots.jl`'s winch figure; the fig8 script ignores it.

Writes the choice to `data/gui.yaml` via [`set_selected_plots`](@ref)
(`examples/gui_state.jl`) rather than a `Main` global, so `simple_fig8_plots.jl`
reads the current selection straight off disk on every `include`.
"""

using REPL.TerminalMenus

include(joinpath(@__DIR__, "gui_state.jl"))

const PLOT_LABELS = ["pattern", "time series", "power", "aerodynamics"]
const PLOT_KEYS = ["pattern", "time_series", "power", "aerodynamics"]

"""
    select_plots()

Ask which figures to show via a checkbox menu and persist the choice to
`data/gui.yaml`.
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
