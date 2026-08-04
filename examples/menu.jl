# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu for the example scripts.

The chosen script is `include`d, so it runs exactly as it would by hand: it
activates `examples/` itself. `select_project()`, `select_sim_time()` and
`select_plots()` pick which system project (150m/200m/300m), how long a run
lasts (the project's own `sim_time`, or a specific value in seconds) and
which figures get shown; all three are persisted to `data/menu_state.yaml`
and read fresh on every run rather than cached in a `Main` global. Selecting
the figure-eight run means minutes of
wall time; its plots come up on their own at the end, so the plotting entry
is for a log that is already on disk.

Started by `menu()` in a REPL from `bin/run_julia`, or by

    include("examples/menu.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using REPL.TerminalMenus

const EXAMPLES_DIR = @__DIR__

const EXAMPLES = [
    "select_project.jl     - choose which system project (150m/200m/300m) to fly" => "select_project.jl",
    "select_sim_time.jl    - choose the simulation time (default or a specific value)" => "select_sim_time.jl",
    "select_plots.jl       - choose which figures to show (pattern/time series/aerodynamics)" => "select_plots.jl",
    "simple_fig8.jl        - fly the figure-of-eight pattern (minutes!)" => "simple_fig8.jl",
    "simple_fig8_plots.jl  - plot the last logged run of active project" => "simple_fig8_plots.jl",
]

"""
    example_menu()

Ask which example to run, `include` it, and ask again until `quit` or `q`.
"""
function example_menu()
    options = [first(e) for e in EXAMPLES]
    push!(options, "quit")
    while true
        choice = request("\nChoose example to run or `q` to quit: ",
                         RadioMenu(options, pagesize=8))
        if choice == -1 || choice == length(options)
            println("Left menu. Press <ctrl><d> to quit Julia!")
            return nothing
        end
        include(joinpath(EXAMPLES_DIR, last(EXAMPLES[choice])))
    end
end

example_menu()
