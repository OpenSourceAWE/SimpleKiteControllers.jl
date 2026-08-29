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
    write_yaml_commented(io, indent, node; comment_col = 36, color = false)

Serialize a nested `OrderedDict` as YAML, recursing into `OrderedDict` values
and appending a trailing `# comment` for leaves given as `(value, comment)`
pairs, aligned to `comment_col` where the line is short enough. `color = true`
adds ANSI syntax highlighting (keys, string/number values, comments); copied
from `reelout_results.jl` so the console summary here matches the one printed
at the end of a live run.
"""
function write_yaml_commented(io, indent, node; comment_col = 36, color = false)
    pad = "  "^indent
    for (k, v) in node
        if v isa AbstractDict
            print(io, pad)
            color ? printstyled(io, k; color = :cyan, bold = true) : print(io, k)
            println(io, ":")
            write_yaml_commented(io, indent + 1, v; comment_col, color)
        else
            value, comment = v isa Tuple ? v : (v, "")
            val = value isa AbstractString ? "\"$value\"" : string(value)
            prefix = string(pad, k, ": ", val)
            print(io, pad)
            if color
                printstyled(io, k; color = :cyan)
                print(io, ": ")
                printstyled(io, val; color = value isa AbstractString ? :green : :yellow)
            else
                print(io, k, ": ", val)
            end
            if isempty(comment)
                println(io)
            else
                print(io, " "^max(1, comment_col - length(prefix)))
                color ? printstyled(io, "# ", comment; color = :light_black) :
                    print(io, "# ", comment)
                println(io)
            end
        end
    end
end

"""
    plot_scenario()

Ask which archived scenario under `output/scenarios/` to replot, then
`include` `simple_reelout_plots.jl` against it, and print that scenario's
`summary:` block (from its `reelout_150m_opt.yaml`) to the console with the
same syntax highlighting as the one printed at the end of a live
`simple_opt_reelout.jl` run. Folders with no files in them are left off the
menu.
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
    choice = TerminalMenus.request("\nSelect a scenario to plot: ", RadioMenu(options, pagesize = 8))

    if choice != -1 && choice != length(options)
        selected = options[choice]
        @info "Plotting scenario: $selected"
        global SCENARIO_PATH = joinpath(SCENARIOS_DIR, selected)
        include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
        if haskey(run_summary, "summary")
            printstyled("\nSummary:\n"; bold = true)
            write_yaml_commented(stdout, 1, run_summary["summary"]; color = true)
        else
            @warn "No summary: block in $selected's run summary."
        end
    else
        println("Selection cancelled.")
    end
    return nothing
end

plot_scenario()
