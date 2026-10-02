# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interactive menu for the model identification.

The chosen script is `include`d, so it runs exactly as it would by hand: it
activates `examples/` itself. `select_project.jl` picks the system project, which
is persisted to `data/gui.yaml`; `build_turn_rate_table.jl` then re-identifies the
turn-rate law of that project's kite and rewrites the turn-rate table the project
names (about 12 minutes for the whole grid); `plot_c1_c2.jl` plots the result.
`identify_kite_delay_scaling.jl` identifies how the dead time and lag scale with
`v_a` and writes the two exponents into the project's course-loop model file. See
"Re-identifying after a change of the kite" on the documentation page
"Examples - identification".

Started by `menu2()` in a REPL from `bin/run_julia`, or by

    include("examples/menu2.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using REPL.TerminalMenus

const IDENTIFICATION_DIR = @__DIR__

const IDENTIFICATION_SCRIPTS = [
    "select_project.jl              - choose the system project (the kite) to identify" => "select_project.jl",
    "build_turn_rate_table.jl       - identify the turn-rate law of every depower (12 min!)" => "build_turn_rate_table.jl",
    "plot_c1_c2.jl                  - plot c1, c2, the dead time and the lag over depower" => "plot_c1_c2.jl",
    "identify_kite_delay_scaling.jl - scaling of dead time and lag over v_a (5 min!)" => "identify_kite_delay_scaling.jl",
]

"""
    identification_menu()

Ask which identification script to run, `include` it, and ask again until `quit` or `q`.
"""
function identification_menu()
    options = [[first(e) for e in IDENTIFICATION_SCRIPTS]; "quit"]
    while true
        choice = TerminalMenus.request("\nChoose identification script to run or `q` to quit: ",
                                       RadioMenu(options, pagesize = 12))
        if choice == -1 || choice == length(options)
            println("Left menu. Press <ctrl><d> to quit Julia!")
            return nothing
        end
        include(joinpath(IDENTIFICATION_DIR, last(IDENTIFICATION_SCRIPTS[choice])))
    end
end

identification_menu()
