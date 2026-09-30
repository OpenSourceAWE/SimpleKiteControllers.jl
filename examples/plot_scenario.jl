# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu to replot an archived scenario from the active project's
site folder under `output/scenarios/` (`cabauw/` or `maasvlakte/`, see
`selected_scenarios_dir` in `src/gui_state.jl`).

Each subfolder there (`v08`, `v09`, ...) is a self-contained copy of one
`simple_opt_reelout.jl` run — its log plus every settings file that produced
it — moved out of a timestamped `output/archives/` folder by hand so it
survives past the next run. This lists the non-empty ones, and on a choice
runs `simple_reelout_plots.jl` with the input `scenario_path`, which reloads
`project_set` and the log from THAT folder's own copies rather than the
live `data/` directory or whatever a prior run left in `Main`.

    include("plot_scenario.jl")
"""

using REPL.TerminalMenus
# `selected_scenarios_dir`: the site folder of the active project.
using SimpleKiteControllers: selected_scenarios_dir
using SimpleKiteControllers: run_example, script_inputs
using SimpleKiteControllers: write_yaml_commented

"""
    plot_scenario()

Ask which archived scenario under the active project's site folder (see
`selected_scenarios_dir`) to replot, then
`include` `simple_reelout_plots.jl` against it, and print that scenario's
`summary:` block (from its `reelout_150m_opt.yaml`) to the console with the
same syntax highlighting as the one printed at the end of a live
`simple_opt_reelout.jl` run. Folders with no files in them are left off the
menu.
"""
function plot_scenario()
    scenarios_dir = selected_scenarios_dir()
    if !isdir(scenarios_dir)
        println("No scenarios found — $scenarios_dir does not exist.")
        return nothing
    end
    scenarios = sort(filter(readdir(scenarios_dir)) do name
        dir = joinpath(scenarios_dir, name)
        isdir(dir) && !isempty(readdir(dir))
    end)
    if isempty(scenarios)
        println("No non-empty scenario folders found in $scenarios_dir")
        return nothing
    end

    options = [scenarios; "quit"]
    choice = TerminalMenus.request("\nSelect a scenario to plot ($(basename(scenarios_dir))): ",
                                   RadioMenu(options, pagesize = 8))

    if choice != -1 && choice != length(options)
        selected = options[choice]
        @info "Plotting scenario: $selected"
        dir = joinpath(scenarios_dir, selected)
        run_example("simple_reelout_plots.jl"; scenario_path = dir)
        # `load_run_summary` and `scenario_log_name` are fresh from the include above; call
        # them at the latest world age or a first-time call throws a world age error.
        Base.invokelatest() do
            run_summary = load_run_summary(dir, scenario_log_name(dir))
            if !isnothing(run_summary) && haskey(run_summary, "summary")
                printstyled("\nSummary:\n"; bold = true)
                write_yaml_commented(stdout, 1, run_summary["summary"]; color = true)
            else
                @warn "No summary: block in $selected's run summary."
            end
        end
    else
        println("Selection cancelled.")
    end
    return nothing
end

plot_scenario()
