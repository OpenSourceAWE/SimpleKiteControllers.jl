# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plot mean, max and min reel-out power, tether force and reel-out speed against
wind speed across every archived scenario in `output/scenarios/` (see
`move_scenario.jl`) — one point per `vNN` folder, read from that scenario's
own run-summary YAML (`summary.av_power_ro`/`max_power_ro`/`min_power_ro`,
`summary.av_force_ro`/`max_force_ro`/`min_force_ro`,
`summary.v_ro_av`/`v_ro_max`/`v_ro_min`, all over the figure-eight flying
phase, and `simulation.wind_speed`) rather than parsed out of the folder name,
the same way `scenario_name` in `move_scenario.jl` does. Folders with no files
in them are skipped. In every panel, max is dashed and min is dash-dotted
(both grey); the force panel also carries a dotted black reference line at
`MAX_TETHER_FORCE_N`, the winch's rated `max_force`, read from
`settings_reelout_150m.yaml`.

    include("plot_powercurve.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using YAML, MakieControlPlots
using SimpleKiteControllers: skc_data_path

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))
const MAX_TETHER_FORCE_N =
    YAML.load_file(joinpath(skc_data_path(), "settings_reelout_150m.yaml"))["winch"]["max_force"]

"""
    scenario_metrics(dir)

`(wind_speed, av_power, max_power, min_power, av_force, max_force, min_force,
av_speed, max_speed, min_speed)` for the scenario archived in `dir`, read from
that run's own summary YAML — found via its one `.arrow` log, as
`scenario_name` in `move_scenario.jl` does. All power/force/speed values are
over the figure-eight flying phase (`summary.av_power_ro`/`max_power_ro`/
`min_power_ro`, `av_force_ro`/`max_force_ro`/`min_force_ro`,
`v_ro_av`/`v_ro_max`/`v_ro_min`).
"""
function scenario_metrics(dir::AbstractString)
    arrow_files = filter(f -> endswith(f, ".arrow"), readdir(dir))
    isempty(arrow_files) && error("No .arrow log found in $dir")
    log_name = replace(only(arrow_files), ".arrow" => "")
    y = YAML.load_file(joinpath(dir, log_name * ".yaml"))
    s = y["summary"]
    return (y["simulation"]["wind_speed"],
            s["av_power_ro"], s["max_power_ro"], s["min_power_ro"],
            s["av_force_ro"], s["max_force_ro"], s["min_force_ro"],
            s["v_ro_av"], s["v_ro_max"], s["v_ro_min"])
end

"""
    plot_powercurve()

Scan `output/scenarios/` for non-empty scenario folders, read each one's mean,
max and min reel-out power, tether force and reel-out speed (all over the
figure-eight flying phase) and wind speed, and plot them stacked against wind
speed [m/s]: power [kW], force [kN], speed [m/s] — mean, max and min together
in each row, max dashed grey and min dash-dotted grey throughout, plus a
dotted black `MAX_TETHER_FORCE_N` reference line in the force row.
"""
function plot_powercurve()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")
    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && !isempty(readdir(dir))
    end
    isempty(dirs) && error("No non-empty scenario folders found in $SCENARIOS_DIR")

    rows = sort(scenario_metrics.(dirs))
    v_wind = getindex.(rows, 1)
    p_kw = getindex.(rows, 2) ./ 1000
    p_max_kw = getindex.(rows, 3) ./ 1000
    p_min_kw = getindex.(rows, 4) ./ 1000
    f_kn = getindex.(rows, 5) ./ 1000
    f_max_kn = getindex.(rows, 6) ./ 1000
    f_min_kn = getindex.(rows, 7) ./ 1000
    v_ro = getindex.(rows, 8)
    v_ro_max = getindex.(rows, 9)
    v_ro_min = getindex.(rows, 10)
    f_limit_kn = fill(MAX_TETHER_FORCE_N / 1000, length(v_wind))

    plotx(v_wind, [p_kw, p_max_kw, p_min_kw], [f_kn, f_max_kn, f_min_kn, f_limit_kn],
          [v_ro, v_ro_max, v_ro_min];
          xlabel = "wind speed [m/s]",
          ylabels = ["power [kW]", "force [kN]", "speed [m/s]"],
          labels = [["mean", "max", "min"], ["mean", "max", "min", "limit"], ["mean", "max", "min"]],
          linestyle = [[nothing, :dash, :dashdot], [nothing, :dash, :dashdot, :dot],
                       [nothing, :dash, :dashdot]],
          color = [[nothing, :grey, :grey], [nothing, :grey, :grey, :black],
                   [nothing, :grey, :grey]],
          legend_position = [:auto, :lt, :auto],
          title = "V3 reel-out power curve", scatter = true, disp = true,
          fig = "powercurve")
    
    # Save the plot to PNG file in notebooks/images folder
    notebooks_dir = normpath(joinpath(@__DIR__, "..", "notebooks", "images"))
    ispath(notebooks_dir) || mkpath(notebooks_dir)
    png_file = joinpath(notebooks_dir, "powercurve.png")
    savefig(png_file)
end

plot_powercurve()
