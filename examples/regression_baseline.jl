# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Baselines for refactors of `simple_opt_reelout.jl` (see `Plan_refactor_opt_reelout.md`).

`fly_replay(site, wind, scenario, out)` flies `simple_opt_reelout.jl` at the
site and wind speed with the optimizer's answers replayed from the scenario
folder `scenario` (the input `replay_paths`, so no optimizer server is needed and any
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

using SimpleKiteControllers: read_gui_field, write_gui_field, set_selected_project
using SimpleKiteControllers: run_example, script_inputs
include(joinpath(@__DIR__, "compare_runs.jl"))

const SITE_PROJECT = Dict("maasvlakte" => "system_reelout_maasvlakte.yaml",
                          "cabauw" => "system_reelout_cabauw.yaml")

"""
    fly_replay(site, wind, scenario, out)

Fly one replayed run, see the file's docstring. Relative `scenario` and `out`
are taken from the package root. With `scenario = nothing` the optimizer is asked (a LIVE
run, which needs the server; how a scenario that replays optimizer answers is recorded), and
`tos_overrides` are the run's input of that name, e.g. to force a code path. Further keywords
are passed on as run inputs (see `run_input_defaults`), e.g. the test inputs of `HOOK_INPUTS`.
Returns `out`.
"""
function fly_replay(site::AbstractString, wind::Real, scenario::Union{AbstractString, Nothing},
                    out::AbstractString; tos_overrides = Dict{Symbol, Any}(), inputs...)
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
        run_example("simple_opt_reelout.jl"; show_plots = false, run_archive = false,
                    replay_paths = scenario, output_path = out,
                    tos_overrides = Dict{Symbol, Any}(tos_overrides), inputs...)
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

"""
The LIVE runs that cover branches no replay reaches (see `Plan_refactor_opt_reelout.md`),
all at Maasvlakte 8.25 m/s: `name => run inputs`. `reject` forces gate retries, cold retries,
rejections and a cold retry that does not converge; `hooks` sets every test input.
"""
const LIVE_CASES = (
    "reject" => (; tos_overrides = Dict{Symbol, Any}(:min_power_frac_prev => 1.2,
                                                     :blend_max_retries => 1)),
    "hooks" => (; steer_disturbance = t -> 0.01 * sin(t),
                xtrack_offset = τ -> τ > 5 ? 1.0 : 0.0, xtrack_phase = 4,
                hold_compliance = (gain = 0.5, τF = 1.0, τpos = 5.0),
                steer_gain_factor = 1.1, extra_steer_delay = 2),
)

"""
    check_live(tag; ref = "pkg", cases = LIVE_CASES) -> Vector{String}

Fly each LIVE case (the optimizer server must be up, or `autostart_server` set) into
`output/regression/<name>_<tag>_8.25` and compare it with `output/regression/<name>_<ref>_8.25`.
The run is served from the solution cache where the reference was, so only the cache counters and
`traj_opt.reopt.blocked` may differ; `compare_runs` prints those, and the line says `DIFFERENT`.
"""
function check_live(tag; ref = "pkg", cases = LIVE_CASES)
    root = normpath(joinpath(@__DIR__, "..", "output"))
    lines = String[]
    for (name, inputs) in cases
        try
            fly_replay("maasvlakte", 8.25, nothing, "output/regression/$(name)_$(tag)_8.25"; inputs...)
            same = compare_runs("$root/regression/$(name)_$(ref)_8.25",
                                "$root/regression/$(name)_$(tag)_8.25")
            push!(lines, "$name: " * (same ? "IDENTICAL" : "DIFFERENT"))
        catch err
            push!(lines, "$name: " * first(split(sprint(showerror, err), '\n')))
        end
    end
    return lines
end

nothing
