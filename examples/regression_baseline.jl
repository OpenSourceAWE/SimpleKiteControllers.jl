# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Baselines for refactors of `simple_opt_reelout.jl` (see `oldplans/Plan_refactor_opt_reelout.md`).

`fly_replay(site, wind, scenario, out)` flies `simple_opt_reelout.jl` at the
site and wind speed with the optimizer's answers replayed from the scenario
folder `scenario` (the input `replay_paths`, so no optimizer server is needed and any
change of a request fails loudly), writes the log and summary to `out` and
leaves the archive and the plots alone. The `gui.yaml` selection is restored
afterwards, also after an error. The run's log messages go to `out/run.log` as well
(`with_run_log`), so a run that stops in the startup still leaves its startup to compare.

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
using SimpleKiteControllers: run_example, script_inputs, with_run_log
using SimpleKiteControllers: startup_ladder_report, ladder_line
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
The run files a previous flight left in `out` are removed first. Returns `out`.
"""
function fly_replay(site::AbstractString, wind::Real, scenario::Union{AbstractString, Nothing},
                    out::AbstractString; tos_overrides = Dict{Symbol, Any}(), inputs...)
    root = normpath(joinpath(@__DIR__, ".."))
    out = abspath(root, out)
    if !isnothing(scenario)
        scenario = abspath(root, scenario)
        isdir(scenario) || error("No scenario folder $scenario")
    end
    if isdir(out)
        for f in readdir(out)
            (f in ("run.log", "last_run_done.txt") || endswith(f, r"_opt(\.arrow|\.yaml|_opt_paths\.yaml)")) &&
                rm(joinpath(out, f))
        end
    end
    project0, wind0 = read_gui_field("project"), read_gui_field("wind_speed")
    try
        set_selected_project(SITE_PROJECT[site])
        write_gui_field("wind_speed", Float64(wind))
        with_run_log(joinpath(out, "run.log")) do
            run_example("simple_opt_reelout.jl"; show_plots = false, run_archive = false,
                        replay_paths = scenario, output_path = out,
                        tos_overrides = Dict{Symbol, Any}(tos_overrides), inputs...)
        end
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
its baseline `output/regression/<site>_<wind>` (`compare_flights`); one line per case.
"""
function check_regression(tag; cases = REGRESSION_CASES)
    root = normpath(joinpath(@__DIR__, "..", "output"))
    return [check_line("$site $wind", "$root/regression/$(site)_$wind", "$root/regression/$(tag)_$wind",
                       () -> fly_replay(site, wind, "output/scenarios/$site/$folder",
                                        "output/regression/$(tag)_$wind"))
            for (site, wind, folder) in cases]
end

first_line(err) = first(split(sprint(showerror, err), '\n'))
has_run(dir) = isdir(dir) && any(endswith("_opt.arrow"), readdir(dir))

"""
    compare_flights(dir_a, dir_b) -> String

`IDENTICAL` or `DIFFERENT` (the differences are printed), with what was compared: the runs
(`compare_runs`, when they flew past the startup) and the startup logs (`compare_startup_logs`, when
both have a `run.log`). A run that stopped in the startup is compared on its startup log alone.
"""
function compare_flights(dir_a, dir_b)
    isdir(dir_a) || return "no reference $dir_a"
    what, same = String[], true
    run_a, run_b = has_run(dir_a), has_run(dir_b)
    if run_a != run_b
        return "DIFFERENT: only $(run_a ? "the reference" : "the new run") flew past the startup"
    elseif run_a
        same &= compare_runs(dir_a, dir_b)
        push!(what, "run")
    end
    if isfile(joinpath(dir_a, "run.log")) && isfile(joinpath(dir_b, "run.log"))
        same &= compare_startup_logs(dir_a, dir_b)
        push!(what, "startup log")
    end
    isempty(what) && return "nothing to compare in $dir_a and $dir_b"
    return (same ? "IDENTICAL" : "DIFFERENT") * " (" * join(what, " and ") * ")"
end

# One line of a check: the comparison, what the new run's startup did (the error it threw included),
# and the error when there is no run log to read it from.
function check_line(name, dir_ref, dir_new, fly)
    threw = try
        fly()
        nothing
    catch err
        first_line(err)
    end
    log = joinpath(dir_new, "run.log")
    return "$name: " * compare_flights(dir_ref, dir_new) *
           (isfile(log) ? "; " * ladder_line(startup_ladder_report(log)) :
                          isnothing(threw) ? "" : "; the run threw: $threw")
end

"""
The LIVE runs that cover branches no replay reaches (see `oldplans/Plan_refactor_opt_reelout.md`),
all at Maasvlakte 8.25 m/s: `name => run inputs`. `reject` forces gate retries, cold retries,
rejections and a cold retry that does not converge; `hooks` sets every test input; `ladder`
reaches `retry_startup!`: the startup path misses `min_feasibility_margin`. The margin alone
does not do it, since it sizes the request too, and neither does a low headroom alone, since
the path then clears 0.82 on its own (0.96 at headroom 0.6); a margin of 1.3 sent at a
headroom of 0.4 asks for about 0.5 of it.
"""
const LIVE_CASES = (
    "reject" => (; tos_overrides = Dict{Symbol, Any}(:min_power_frac_prev => 1.2,
                                                     :blend_max_retries => 1)),
    "ladder" => (; tos_overrides = Dict{Symbol, Any}(:min_feasibility_margin => 1.3,
                                                     :turn_radius_headroom => 0.4)),
    "hooks" => (; steer_disturbance = t -> 0.01 * sin(t),
                xtrack_offset = τ -> τ > 5 ? 1.0 : 0.0, xtrack_phase = 4,
                hold_compliance = (gain = 0.5, τF = 1.0, τpos = 5.0),
                steer_gain_factor = 1.1, extra_steer_delay = 2),
)

"""
    check_live(tag; ref = "d002", cases = LIVE_CASES) -> Vector{String}

Fly each LIVE case (the optimizer server must be up, or `autostart_server` set) into
`output/regression/<name>_<tag>_8.25` and compare it with `output/regression/<name>_<ref>_8.25`.
The run is served from the solution cache where the reference was, so only the cache counters and
`traj_opt.reopt.blocked` may differ; `compare_runs` prints those, and the line says `DIFFERENT`.
Select cases by name, e.g. `cases = filter(c -> first(c) == "ladder", LIVE_CASES)`.
"""
function check_live(tag; ref = "d002", cases = LIVE_CASES)
    root = normpath(joinpath(@__DIR__, "..", "output"))
    return [check_line(name, "$root/regression/$(name)_$(ref)_8.25", "$root/regression/$(name)_$(tag)_8.25",
                       () -> fly_replay("maasvlakte", 8.25, nothing,
                                        "output/regression/$(name)_$(tag)_8.25"; inputs...))
            for (name, inputs) in cases]
end

nothing
