# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu for the example scripts.

The chosen script is `include`d, so it runs exactly as it would by hand: it
activates `examples/` itself. `select_project()`, `select_sim_time()`,
`select_plots()`, `select_turbulence()` and `select_windspeed()` pick which
system project (150m/200m/300m), how long a run lasts (the project's own
`sim_time`, or a specific value in seconds), which figures get shown, how much
turbulence `init` applies and the mean wind speed it flies; all five are
persisted to `data/gui.yaml` and read fresh on every run rather than cached in
a `Main` global. Once the model
and settling caches exist, the figure-eight run simulates at about twice
realtime, so its wall time is roughly half the selected `sim_time`; the first
run of a fresh cache takes minutes longer. Its plots come up on their own at
the end, so the plotting entries are for a log that is already on disk —
`simple_reelout_plots.jl` for the last one, `plot_scenario.jl` for an archived
run moved into `output/scenarios/`.

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
    "select_windspeed.jl     - choose the wind speed init() applies (default or a specific m/s)" => "select_windspeed.jl",
    "select_project.jl       - choose which system project (150m/200m/300m) to fly" => "select_project.jl",
    "select_sim_time.jl      - choose the simulation time (default or a specific value)" => "select_sim_time.jl",
    "select_plots.jl         - choose figures: pattern/3d path/time series/power/aerodynamics" => "select_plots.jl",
    "plot_scenario.jl        - replot an archived run from output/scenarios/" => "plot_scenario.jl",
    "move_scenario.jl        - move the last reel-out run into output/scenarios/vNN" => "move_scenario.jl",
    "copy_scenario.jl        - same, but keeps vNN_2/vNN_3/... instead of overwriting" => "copy_scenario.jl",
    "simple_opt_reelout.jl   - reel out along an externally optimized path (minutes!)" => "simple_opt_reelout.jl",
    "simple_reelout_plots.jl - plot the last logged reel-out run" => "simple_reelout_plots.jl",
    "simple_fig8.jl          - fly the figure-of-eight pattern (minutes!)" => "simple_fig8.jl",
    "simple_fig8_live.jl     - the same run, shown live in the 3D viewer (minutes!)" => "simple_fig8_live.jl",
    "simple_fig8_plots.jl    - plot the last logged run of active project" => "simple_fig8_plots.jl",
    "simple_opt_fig8.jl      - fly an externally optimized path at constant length (minutes!)" => "simple_opt_fig8.jl",
    "simple_reelout.jl       - fly the pattern, then reel out to reelout_l_max (minutes!)" => "simple_reelout.jl",
    "simple_reelout_play.jl  - replay the last logged reel-out run in the 3D viewer" => "simple_reelout_play.jl",
    "simple_auto_parking.jl  - fly heading-stabilized parking of the V3 kite" => "simple_auto_parking.jl",
    "simple_auto_parking_plots.jl - plot the last logged parking run" => "simple_auto_parking_plots.jl",
    "optimize_fig8.jl        - sweep the pattern shape in parallel processes (HOURS!)" => "optimize_fig8.jl",
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
