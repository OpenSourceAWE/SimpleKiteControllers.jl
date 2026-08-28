# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu to choose the wind speed `init` applies in `simple_fig8.jl`:
`default` (the selected project's own `v_wind`) or a specific value in m/s at
the project's reference height.

Writes `wind_speed` in this package's `data/gui.yaml` beside the other menu
selections. `simple_fig8.jl` reads it back with
[`selected_windspeed`](@ref) and passes
`something(selected_windspeed(), project_set.v_wind)` as the mean wind speed
to `init`, so a `default` selection changes nothing and a specific value
overrides the project file for that run only.

The turbulent wind field is generated for the resulting mean speed; a value
with no matching cached field makes the next run generate one (~1.2 GB, tens
of seconds), same as picking a new turbulence level.
"""

using REPL.TerminalMenus

include(joinpath(@__DIR__, "gui_state.jl"))

"""
    ask_windspeed_ms() -> Union{Float64, Nothing}

Prompt for a positive wind speed in m/s, re-asking on invalid input;
`nothing` on empty input (cancel).
"""
function ask_windspeed_ms()
    while true
        input = Base.prompt("Wind speed in m/s")
        (isnothing(input) || isempty(strip(input))) && return nothing
        value = tryparse(Float64, strip(input))
        if !isnothing(value) && value > 0
            return value
        end
        println("Enter a positive number, e.g. 6.0.")
    end
end

"""
    select_windspeed()

Ask for `default` or a specific wind speed in m/s and persist the choice to
`data/gui.yaml`.
"""
function select_windspeed()
    current = selected_windspeed()
    current_str = isnothing(current) ? "default" : "$(current) m/s"
    options = ["default", "specific value in m/s...", "quit"]
    choice = REPL.TerminalMenus.request("\nSelect wind speed (current: $current_str): ",
                     RadioMenu(options, pagesize=8))

    if choice == 1
        set_selected_windspeed(nothing)
        println("Wind speed set to: default")
    elseif choice == 2
        value = ask_windspeed_ms()
        if isnothing(value)
            println("Selection cancelled.")
        else
            set_selected_windspeed(value)
            println("Wind speed set to: $value m/s")
        end
    else
        println("Selection cancelled.")
    end
    return nothing
end

select_windspeed()
