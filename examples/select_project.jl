# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu to choose which system project (`system_fig8_*.yaml`) the
next `simple_fig8.jl` run flies.

Sets the global `PROJECT` in `Main`, which `simple_fig8.jl` picks up if
already defined — same session-global pattern as `fcs` and `SHOW_PLOTS`.
"""

using REPL.TerminalMenus
using SimpleKiteControllers: skc_data_path

"""
    select_project()

Ask which `system_fig8_*.yaml` project to fly and set the global `PROJECT`
to the chosen file name.
"""
function select_project()
    data_dir = skc_data_path()
    projects = sort(filter(f -> startswith(f, "system") && endswith(f, ".yaml"), readdir(data_dir)))

    if isempty(projects)
        println("No system_*.yaml project files found in $data_dir")
        return nothing
    end

    current = @isdefined(PROJECT) ? PROJECT : ""
    prompt = isempty(current) ? "\nSelect a project: " : "\nSelect a project (current: $current): "

    options = [projects; "quit"]
    choice = request(prompt, RadioMenu(options, pagesize=8))

    if choice != -1 && choice != length(options)
        global PROJECT = options[choice]
        println("Project set to: $PROJECT")
    else
        println("Selection cancelled.")
    end
    return nothing
end

select_project()
