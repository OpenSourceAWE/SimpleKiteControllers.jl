# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu to choose which system project (`system_*.yaml`) the
example scripts fly.

Writes the choice to `data/gui.yaml` via [`set_selected_project`](@ref)
(`examples/gui_state.jl`) rather than a `Main` global, so `simple_fig8.jl`
and `simple_fig8_plots.jl` read the current selection straight off disk on
every `include` instead of relying on a REPL variable.

The `simple_reelout*.jl` scripts read the same key through
[`selected_reelout_project`](@ref), which ignores a fig8 selection and flies
`system_reelout_150m.yaml` instead, so the two families need no re-selection in
between; a `system_reelout_*.yaml` chosen here does reach them.
"""

using REPL
using REPL.TerminalMenus
using SimpleKiteControllers: skc_data_path

include(joinpath(@__DIR__, "gui_state.jl"))

"""
    select_project()

Ask which `system_*.yaml` project to fly and persist the choice to
`data/gui.yaml`.
"""
function select_project()
    data_dir = skc_data_path()
    projects = sort(filter(f -> startswith(f, "system") && endswith(f, ".yaml"), readdir(data_dir)))

    if isempty(projects)
        println("No system_*.yaml project files found in $data_dir")
        return nothing
    end

    current = selected_project()
    options = [projects; "quit"]
    choice = REPL.TerminalMenus.request("\nSelect a project (current: $current): ", RadioMenu(options, pagesize=8))

    if choice != -1 && choice != length(options)
        selected = options[choice]
        set_selected_project(selected)
        println("Project set to: $selected")
    else
        println("Selection cancelled.")
    end
    return nothing
end

select_project()
