# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu to choose the simulation time `simple_fig8.jl` flies:
`default` (the selected project's own `sim_time`) or a specific value in
seconds, entered numerically.

Writes the choice to `data/gui.yaml` via [`set_selected_sim_time`](@ref)
(`examples/gui_state.jl`) rather than a `Main` global, so `simple_fig8.jl`
reads the current selection straight off disk on every `include`.
"""

using REPL.TerminalMenus

include(joinpath(@__DIR__, "gui_state.jl"))

"""
    ask_sim_time_seconds() -> Union{Float64, Nothing}

Prompt for a positive number of seconds, re-asking on invalid input;
`nothing` on empty input (cancel).
"""
function ask_sim_time_seconds()
    while true
        input = Base.prompt("Simulation time in seconds")
        (isnothing(input) || isempty(strip(input))) && return nothing
        value = tryparse(Float64, strip(input))
        if !isnothing(value) && value > 0
            return value
        end
        println("Enter a positive number, e.g. 30.0.")
    end
end

"""
    select_sim_time()

Ask for `default` or a specific simulation time in seconds and persist the
choice to `data/gui.yaml`.
"""
function select_sim_time()
    current = selected_sim_time()
    current_str = isnothing(current) ? "default" : "$(current) s"
    options = ["default", "specific value in seconds...", "quit"]
    choice = request("\nSelect simulation time (current: $current_str): ",
                     RadioMenu(options, pagesize=8))

    if choice == 1
        set_selected_sim_time(nothing)
        println("Simulation time set to: default")
    elseif choice == 2
        value = ask_sim_time_seconds()
        if isnothing(value)
            println("Selection cancelled.")
        else
            set_selected_sim_time(value)
            println("Simulation time set to: $value s")
        end
    else
        println("Selection cancelled.")
    end
    return nothing
end

select_sim_time()
