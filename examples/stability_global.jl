# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Disk-margin stability analysis of the reel-out course loop over every archived
scenario of the active project's site (`output/scenarios/<site>/vNN`, see
`selected_scenarios_dir` in `gui_state.jl`).

For each non-empty scenario folder this runs `stability_opt_reelout.jl` with
`LOG_DIR` set to that folder: the operating points (tether length, `v_a`, kite
speed, depower, centre elevation, the tape's lag and the kite's dead time) come
from the scenario's log, the controller from the LIVE `data/` settings. So the
table answers "is the current tuning stable at every operating point flown so
far", not "was each scenario stable as it was flown" — a scenario flown before
a retune may have used other gains.

Prints one row per scenario with the worst disk margin over its linear
tether-length bins, for the inner loop and the loop with the guidance, and
where the guided worst case sits (tether length, `v_a`, depower). The guided
margin is the one rated. Then re-runs the worst scenario with plots on, which
shows its Bode plot and the margins over tether length, and leaves its `L` in
`Main` for `diskmargin(L)`.

About 35 s per scenario. Set `VERBOSE = true` first to see each scenario's full per-bin output.

    include("stability_global.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Printf
using Base.CoreLogging: with_logger, NullLogger
import YAML

include(joinpath(@__DIR__, "gui_state.jl"))
verbose = @isdefined(VERBOSE) ? VERBOSE : false
VERBOSE = false

scenarios_dir = selected_scenarios_dir()
isdir(scenarios_dir) || error("No scenarios: $scenarios_dir does not exist.")
scenario_names = sort(filter(readdir(scenarios_dir)) do name
    dir = joinpath(scenarios_dir, name)
    isdir(dir) && any(endswith(".arrow"), readdir(dir))
end)
isempty(scenario_names) && error("No scenario folder with a log in $scenarios_dir.")

"Wind speed [m/s] of the scenario in `dir`, from its run summary; `NaN` if it has none"
function scenario_wind(dir)
    for f in filter(f -> endswith(f, ".yaml") && !startswith(f, "system_"), readdir(dir))
        y = YAML.load_file(joinpath(dir, f))
        y isa AbstractDict && haskey(y, "simulation") || continue
        return Float64(get(y["simulation"], "wind_speed", NaN))
    end
    return NaN
end

"""
    muted(f)

Call `f()` with `stdout` and logging silenced. Rebinds `Base.stdout` instead of
`redirect_stdout`, which can only restore a file-backed stream and so fails, with
`stdout` left on `devnull`, in a REPL whose `stdout` is a custom IO (e.g. Kaimon's).
"""
function muted(f)
    out = stdout
    setglobal!(Base, :stdout, devnull)
    try
        return with_logger(f, NullLogger())
    finally
        setglobal!(Base, :stdout, out)
    end
end

"Run `stability_opt_reelout.jl` on the scenario folder `dir`, muted unless `verbose`"
function analyse_scenario(dir; plots = false, quiet = true)
    Main.SHOW_PLOTS = plots
    Main.LOG_DIR = dir
    analyse() = Base.include(Main, joinpath(@__DIR__, "stability_opt_reelout.jl"))
    quiet ? muted(analyse) : analyse()
    # Globals the include just (re)defined: read them at the latest world age.
    latest(name) = Base.invokelatest(getglobal, Main, name)
    lin_rows, rows, worst = latest(:lin_rows), latest(:rows), latest(:worst)
    return (; α_inner = minimum(r.α_inner for r in lin_rows), α_guided = worst.α_guided,
            dm_guided = worst.dm_guided, L = worst.L, va = worst.va, dp = worst.dp,
            not_rated = length(rows) - length(lin_rows), not_flown = length(latest(:uncovered)),
            τ = latest(:τ_log))
end

results = []
for name in scenario_names
    dir = joinpath(scenarios_dir, name)
    print(rpad("  $name ", 12))
    try
        r = analyse_scenario(dir; quiet = !verbose)
        push!(results, (; name, dir, wind = scenario_wind(dir), r...))
        println(@sprintf("α guided = %.3f", r.α_guided))
    catch e
        push!(results, (; name, dir, wind = scenario_wind(dir), error = sprint(showerror, e)))
        println("failed: ", first(split(sprint(showerror, e), '\n')))
    end
end

ok = filter(r -> !haskey(r, :error), results)
isempty(ok) && error("The stability analysis failed for every scenario in $scenarios_dir.")
worst_scenario = argmin(r -> r.α_guided, ok)

verdict(α) = α < 0.3 ? "fragile" : α < 0.5 ? "marginal" : "robust"
println()
printstyled(@sprintf("Worst disk margin per scenario, %s (live controller settings):\n",
                     basename(scenarios_dir)); bold = true)
println("  scenario  wind [m/s]  α inner  α guided  verdict   at L [m]  v_a [m/s]  depower  DM guided  τ_kite [s]  bins not rated/flown")
for r in results
    if haskey(r, :error)
        println(@sprintf("  %-8s  %10.2f  failed: %s", r.name, r.wind, first(split(r.error, '\n'))))
        continue
    end
    line = @sprintf("  %-8s  %10.2f  %7.3f  %8.3f  %-8s  %8.0f  %9.1f  %7.3f  %7.3f s  %10.3f  %d/%d",
                    r.name, r.wind, r.α_inner, r.α_guided, verdict(r.α_guided), r.L, r.va, r.dp,
                    r.dm_guided, r.τ, r.not_rated, r.not_flown)
    color = r.α_guided < 0.3 ? :red : r.α_guided < 0.5 ? :yellow : :normal
    printstyled(line, r === worst_scenario ? "   <- worst\n" : "\n"; color)
end
println()

# The worst scenario again, with its plots and its full console output; leaves its `L` in Main.
@info @sprintf("Worst scenario: %s (wind %.2f m/s), α guided = %.3f at L = %.0f m. Plotting it.",
               worst_scenario.name, worst_scenario.wind, worst_scenario.α_guided, worst_scenario.L)
analyse_scenario(worst_scenario.dir; plots = true, quiet = false)
nothing
