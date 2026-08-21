# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plot mean reel-out power against wind speed across every archived scenario in
`output/scenarios/` (see `move_scenario.jl`) — one point per `vNN` folder,
read from that scenario's own run-summary YAML (`reelout.power.mean_W` and
`simulation.wind_speed`) rather than parsed out of the folder name, the same
way `scenario_name` in `move_scenario.jl` and `scan_row` in `wind_scan.jl` do.
Folders with no files in them are skipped.

    include("plot_powercurve.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using YAML, MakieControlPlots

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))

"""
    scenario_power(dir)

`(wind_speed, mean_power)` for the scenario archived in `dir`, read from that
run's own summary YAML — found via its one `.arrow` log, as `scenario_name`
in `move_scenario.jl` does.
"""
function scenario_power(dir::AbstractString)
    arrow_files = filter(f -> endswith(f, ".arrow"), readdir(dir))
    isempty(arrow_files) && error("No .arrow log found in $dir")
    log_name = replace(only(arrow_files), ".arrow" => "")
    y = YAML.load_file(joinpath(dir, log_name * ".yaml"))
    return (y["simulation"]["wind_speed"], y["reelout"]["power"]["mean_W"])
end

"""
    plot_powercurve()

Scan `output/scenarios/` for non-empty scenario folders, read each one's mean
reel-out power and wind speed, and plot power [kW] against wind speed [m/s].
"""
function plot_powercurve()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")
    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && !isempty(readdir(dir))
    end
    isempty(dirs) && error("No non-empty scenario folders found in $SCENARIOS_DIR")

    rows = sort(scenario_power.(dirs))
    v_wind = first.(rows)
    p_mean_kw = last.(rows) ./ 1000

    plotxy(v_wind, p_mean_kw;
           xlabel = "wind speed [m/s]", ylabel = "mean reel-out power [kW]",
           title = "V3 reel-out power curve", scatter = true, disp = true,
           fig = "powercurve")
end

plot_powercurve()
