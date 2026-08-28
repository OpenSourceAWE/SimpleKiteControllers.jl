# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu to replot an archived scenario from `output/scenarios/`.

Each subfolder there (`v08`, `v09`, ...) is a self-contained copy of one
`simple_opt_reelout.jl` run — its log plus every settings file that produced
it — moved out of a timestamped `output/archives/` folder by hand so it
survives past the next run. This lists the non-empty ones, and on a choice
sets `SCENARIO_PATH` and `include`s `simple_reelout_plots.jl`, which reloads
`fcs`/`project_set`/the log from THAT folder's own copies rather than the
live `data/` directory or whatever a prior run left in `Main`.

    include("plot_scenario.jl")
"""

using REPL.TerminalMenus

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))

"""
    plot_scenario()

Ask which archived scenario under `output/scenarios/` to replot, then
`include` `simple_reelout_plots.jl` against it. Folders with no files in them
are left off the menu.
"""
function plot_scenario()
    if !isdir(SCENARIOS_DIR)
        println("No scenarios found — $SCENARIOS_DIR does not exist.")
        return nothing
    end
    scenarios = sort(filter(readdir(SCENARIOS_DIR)) do name
        dir = joinpath(SCENARIOS_DIR, name)
        isdir(dir) && !isempty(readdir(dir))
    end)
    if isempty(scenarios)
        println("No non-empty scenario folders found in $SCENARIOS_DIR")
        return nothing
    end

    options = [scenarios; "quit"]
    choice = REPL.TerminalMenus.request("\nSelect a scenario to plot: ", RadioMenu(options, pagesize = 8))

    if choice != -1 && choice != length(options)
        selected = options[choice]
        @info "Plotting scenario: $selected"
        global SCENARIO_PATH = joinpath(SCENARIOS_DIR, selected)
        include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
    else
        println("Selection cancelled.")
    end
    return nothing
end

plot_scenario()
