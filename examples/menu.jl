# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu for the example scripts.

The chosen script is `include`d, so it runs exactly as it would by hand: it
activates `examples/` itself. `select_project()`, `select_sim_time()`,
`select_plots()` and `select_turbulence()` pick which system project
(150m/200m/300m), how long a run lasts (the project's own `sim_time`, or a
specific value in seconds), which figures get shown and how much turbulence
`init` applies; all four are persisted to `data/gui.yaml` and read fresh on
every run rather than cached in a `Main` global. Once the model
and settling caches exist, the figure-eight run simulates at about twice
realtime, so its wall time is roughly half the selected `sim_time`; the first
run of a fresh cache takes minutes longer. Its plots come up on their own at
the end, so the plotting entry is for a log that is already on disk.

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
    "select_turbulence.jl    - choose the turbulence level init() applies (default or 0.0…1.0)" => "select_turbulence.jl",
    "select_project.jl       - choose which system project (150m/200m/300m) to fly" => "select_project.jl",
    "select_sim_time.jl      - choose the simulation time (default or a specific value)" => "select_sim_time.jl",
    "select_plots.jl         - choose which figures to show (pattern/time series/aerodynamics)" => "select_plots.jl",
    "simple_fig8.jl          - fly the figure-of-eight pattern (minutes!)" => "simple_fig8.jl",
    "simple_fig8_live.jl     - the same run, shown live in the 3D viewer (minutes!)" => "simple_fig8_live.jl",
    "simple_fig8_plots.jl    - plot the last logged run of active project" => "simple_fig8_plots.jl",
    "simple_reelout.jl       - fly the pattern, then reel out to reelout_l_max (minutes!)" => "simple_reelout.jl",
    "simple_reelout_plots.jl - plot the last logged reel-out run" => "simple_reelout_plots.jl",
    "optimize_path.jl        - Julia client for the AWETrim reelout flight-path optimizer" => "optimize_path.jl",
    "export_v3_segments.jl   - write the V3 segment table to output/v3_segments.csv" => "export_v3_segments.jl",
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
                         RadioMenu(options, pagesize=10))
        if choice == -1 || choice == length(options)
            println("Left menu. Press <ctrl><d> to quit Julia!")
            return nothing
        end
        include(joinpath(EXAMPLES_DIR, last(EXAMPLES[choice])))
    end
end

example_menu()
