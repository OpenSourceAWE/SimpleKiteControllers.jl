# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Baselines for refactors of `simple_opt_reelout.jl` (see `Plan_refactor_opt_reelout.md`).

`fly_replay(site, wind, scenario, out)` flies `simple_opt_reelout.jl` at the
site and wind speed with the optimizer's answers replayed from the scenario
folder `scenario` (`REPLAY_PATHS`, so no optimizer server is needed and any
change of a request fails loudly), writes the log and summary to `out` and
leaves the archive and the plots alone. The `gui.yaml` selection is restored
afterwards, also after an error.

Compare the result with `compare_runs`; record a baseline with the script
before a change and a second run after it:

    include("regression_baseline.jl")
    fly_replay("maasvlakte", 8.25, "output/scenarios/maasvlakte/v08.25",
               "output/regression/maasvlakte_8.25")
    compare_runs("output/regression/maasvlakte_8.25", "output/scenarios/maasvlakte/v08.25")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

include(joinpath(@__DIR__, "gui_state.jl"))
include(joinpath(@__DIR__, "compare_runs.jl"))

const SITE_PROJECT = Dict("maasvlakte" => "system_reelout_maasvlakte.yaml",
                          "cabauw" => "system_reelout_cabauw.yaml")

"""
    fly_replay(site, wind, scenario, out)

Fly one replayed run, see the file's docstring. Relative `scenario` and `out`
are taken from the package root. With `scenario = nothing` the optimizer is asked (a LIVE
run, which needs the server; how a scenario that replays optimizer answers is recorded), and
`tos_overrides` are the `TOS_OVERRIDES` of the run, e.g. to force a code path. Returns `out`.
"""
function fly_replay(site::AbstractString, wind::Real, scenario::Union{AbstractString, Nothing},
                    out::AbstractString; tos_overrides = Dict{Symbol, Any}())
    root = normpath(joinpath(@__DIR__, ".."))
    out = abspath(root, out)
    if !isnothing(scenario)
        scenario = abspath(root, scenario)
        isdir(scenario) || error("No scenario folder $scenario")
    end
    project0, wind0 = read_gui_field("project"), read_gui_field("wind_speed")
    try
        set_selected_project(SITE_PROJECT[site])
        write_gui_field("wind_speed", Float64(wind))
        Core.eval(Main, :(SHOW_PLOTS = false; RUN_ARCHIVE = false;
                          REPLAY_PATHS = $scenario; OUTPUT_PATH = $out;
                          TOS_OVERRIDES = $(Dict{Symbol, Any}(tos_overrides))))
        Base.include(Main, joinpath(@__DIR__, "simple_opt_reelout.jl"))
    finally
        set_selected_project(project0)
        write_gui_field("wind_speed", something(wind0, "default") == "default" ? "default" :
                                      parse(Float64, wind0))
    end
    return out
end

"The replay baselines recorded before the refactor: `(site, wind, scenario folder)`"
const REGRESSION_CASES = (("maasvlakte", 8.25, "v08.25"), ("maasvlakte", 3.5, "v03.5"))

"""
    check_regression(tag; cases = REGRESSION_CASES) -> Vector{String}

Fly each case with `fly_replay` into `output/regression/<tag>_<wind>` and compare it with
its baseline `output/regression/<site>_<wind>`; one line per case, `IDENTICAL`,
`DIFFERENT` (the differences are printed) or the error.
"""
function check_regression(tag; cases = REGRESSION_CASES)
    root = normpath(joinpath(@__DIR__, "..", "output"))
    lines = String[]
    for (site, wind, folder) in cases
        try
            fly_replay(site, wind, "output/scenarios/$site/$folder", "output/regression/$(tag)_$wind")
            same = compare_runs("$root/regression/$(site)_$wind", "$root/regression/$(tag)_$wind")
            push!(lines, "$site $wind: " * (same ? "IDENTICAL" : "DIFFERENT"))
        catch err
            push!(lines, "$site $wind: " * first(split(sprint(showerror, err), '\n')))
        end
    end
    return lines
end

nothing
