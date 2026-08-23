# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Utility functions for pattern plotting — extracted from simple_reelout_plots.jl
to enable batch processing in create_plots.jl while keeping the interactive
script's structure intact.
"""

# These imports are assumed to be available in the including script's context
# (simple_reelout_plots.jl or create_plots.jl), but we list them here for clarity
# using MakieControlPlots
# using LaTeXStrings
# using V3Kite
# using SimpleKiteControllers

"""
    plot_pattern_scenario(scenario_dir::AbstractString; disp::Bool = true,
                         opt_raw::Union{Nothing, Tuple} = nothing,
                         project::Union{Nothing, AbstractString} = nothing,
                         log_name::Union{Nothing, AbstractString} = nothing) -> Figure

Plot the azimuth/elevation flight pattern for a scenario in `scenario_dir`
(flown path vs. attractor reference, or vs. optimizer-raw if available).
Loads the flight log from `scenario_dir`. The system project defaults to
`scenario_dir`'s own `system_reelout_150m.yaml` copy, present in an archived
scenario folder (`output/scenarios/<name>` or `output/archives/<run>`); pass
`project` explicitly for a live run in `output/`, whose project file lives in
`data/` and is never copied there. When `disp=false`, the figure is created
but not displayed (suitable for batch processing). `opt_raw` may be passed as
`(az_array, el_array)` to overlay the optimizer's uncorrected paths; if
absent, uses the attractor reference instead. Pass `log_name` explicitly to
avoid ambiguity when multiple logs exist in the directory (common in `output/`
after multiple runs); if absent and `project` is provided, searches for the
one `.arrow` file.
"""
function plot_pattern_scenario(scenario_dir::AbstractString; disp::Bool = true,
                               opt_raw::Union{Nothing, Tuple} = nothing,
                               project::Union{Nothing, AbstractString} = nothing,
                               log_name::Union{Nothing, AbstractString} = nothing)
    # Load settings and log; the system project is either the caller's own
    # resolved path, or the scenario folder's own copy
    scenario_project = something(project, joinpath(scenario_dir, "system_reelout_150m.yaml"))
    project_set = Settings(scenario_project)
    fcs = FC_Settings(fc_settings(scenario_project); path = scenario_dir)

    # Find and load the .arrow log file
    if isnothing(log_name)
        arrow_files = filter(f -> endswith(f, ".arrow"), readdir(scenario_dir))
        isempty(arrow_files) && error("No .arrow log found in scenario folder $scenario_dir")
        log_name = replace(only(arrow_files), ".arrow" => "")
    end
    syslog = load_log(log_name; path = scenario_dir)
    sl = syslog.syslog

    # Compute wind speed from the run summary YAML
    flown_wind = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]["wind_speed"]
    fig_name = "V3 Kite Reel-out – $(round(flown_wind; digits = 1)) m/s"
    project_name = replace(basename(scenario_project), ".yaml" => "")

    # Skip t=0 (guidance slots filled from first step! onward)
    rng = 2:length(sl.time)

    # Azimuth and elevation over the flight phase
    az_deg = rad2deg.(sl.azimuth[rng])
    el_deg = rad2deg.(sl.elevation[rng])

    # Reference path: the logged attractor (live, walking every path under re-opt)
    # or fallback to the lemniscate
    live = [i for i in rng if sl.sys_state[i] >= 3 &&
            !(iszero(sl.var_02[i]) && iszero(sl.var_03[i]))]
    ref_az, ref_el = if !isempty(live)
        Float64.(sl.var_02[live]), Float64.(sl.var_03[live])
    else
        figure_eight_path(fcs.f8_a, fcs.f8_b, fcs.f8_c, fcs.f8_d, 0.0,
                          Float64(sl.var_04[end]), 0.0, 361)
    end

    # Build and return the pattern plot
    p = plotxy(
        isnothing(opt_raw) ? [az_deg, ref_az] : [az_deg, opt_raw[1]],
        isnothing(opt_raw) ? [el_deg, ref_el] : [el_deg, opt_raw[2]];
        xlabel = L"\mathrm{azimuth}~[°]",
        ylabel = L"\mathrm{elevation}~[°]",
        legend = isnothing(opt_raw) ?
                 [L"\mathrm{flown}", L"\mathrm{attractor}"] :
                 [L"\mathrm{flown}", L"\mathrm{optimizer,~uncorrected}"],
        fig = replace(fig_name, "Reel-out" => project_name) * " – pattern",
        disp = disp,
    )
    return p
end
