# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Utility functions for pattern, time-series, power and aerodynamics plotting —
extracted from simple_reelout_plots.jl to enable batch processing in
create_plots.jl while keeping the interactive script's structure intact.
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

"""
    plot_time_series_scenario(scenario_dir::AbstractString; disp::Bool = true,
                             project::Union{Nothing, AbstractString} = nothing,
                             log_name::Union{Nothing, AbstractString} = nothing) -> Figure

Plot the time-series panels (cross-track error, elevation, course/heading,
tracking error, steering, tether force, tether length vs. lap count, reel-out
speed, depower vs. winch-controller state, and the entry phase) for a scenario
in `scenario_dir`. Loads the flight log from `scenario_dir`, the same way
`plot_pattern_scenario` does — see its docstring for `project`/`log_name`/
`disp` semantics.
"""
function plot_time_series_scenario(scenario_dir::AbstractString; disp::Bool = true,
                                   project::Union{Nothing, AbstractString} = nothing,
                                   log_name::Union{Nothing, AbstractString} = nothing)
    scenario_project = something(project, joinpath(scenario_dir, "system_reelout_150m.yaml"))

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

    el_deg = rad2deg.(sl.elevation[rng])

    # --- angles for the psi/chi panel, plotted UNWRAPPED ---------------------- #
    unwrap_angle(a) = first(a) .+ cumsum(vcat(0.0, wrap_to_pi.(diff(a))))
    onto(ref_u, ref_w, a) = ref_u .+ wrap_to_pi.(a .- ref_w)

    psi    = Float64.(sl.heading[rng])
    # +π puts the logged course into the same convention as heading (0 = zenith).
    chi    = wrap_to_pi.(Float64.(sl.course[rng]) .+ pi)
    chiset = Float64.(sl.bearing[rng])   # chi_cmd, the course actually tracked
    chi_u  = unwrap_angle(chi)

    # Both wrapped to ±180°; the offset between them is the kite's drift angle.
    err_course  = rad2deg.(wrap_to_pi.(chi .- chiset))
    err_heading = rad2deg.(wrap_to_pi.(psi .- chiset))

    # `getindex` because l_tether/v_reelout are one entry per tether and the V3 has one.
    l_tether = getindex.(sl.l_tether[rng], 1)
    v_reelout = getindex.(sl.v_reelout[rng], 1)
    v_set = Float64.(sl.var_11[rng])
    u_d = Float64.(sl.var_14[rng])            # commanded rel_depower, filled by step! itself
    # WinchController state (0 lower-force, 1 speed, 2 upper-force). Logged every
    # step, but `rc` is only stepped from phase 3 on, so everything before that is
    # an unstepped controller's state and says nothing — NaN blanks it rather than
    # drawing a flat line the eye reads as a measurement.
    wc_state = [sl.sys_state[i] >= 3 ? Float64(sl.var_12[i]) : NaN for i in rng]
    # Entry state machine; the codes stay 0-based, other scripts search for `>= 3`.
    state = Float64.(sl.sys_state[rng])
    fig8 = Float64.(sl.fig_8[rng])

    p = plotx(
        sl.time[rng],
        sl.var_01[rng],
        [el_deg, Float64.(sl.var_03[rng])],
        [rad2deg.(onto(chi_u, chi, psi)), rad2deg.(chi_u),
         rad2deg.(onto(chi_u, chi, chiset))],
        [err_course, err_heading, Float64.(sl.var_06[rng])],
        (100.0 .* sl.steering[rng], 100.0 .* sl.set_steering[rng]),
        getindex.(sl.winch_force[rng], 1),
        [l_tether, fig8],
        [v_reelout, v_set],
        [u_d, wc_state],
        state;
        xlabel = L"\mathrm{time}~[\mathrm{s}]",
        ysize = 18,
        legendsize = 16,
        ylabels = [
            L"d~[°]",
            L"\mathrm{elevation}~[°]",
            L"\psi,~\chi~[°]",
            L"\Delta\chi~[°]",
            L"u_{\mathrm{s}}~[\%]",
            L"F_{\mathrm{tether}}~[\mathrm{N}]",
            [L"l_{\mathrm{tether}}~[\mathrm{m}]", L"\mathrm{cycle}~[-]"],
            L"v_{\mathrm{ro}}~[\mathrm{m/s}]",
            [L"u_{\mathrm{d}}~[-]", L"\mathrm{wc~state}~[-]"],
            L"\mathrm{state}~[-]",
        ],
        labels = [
            nothing,
            [L"\mathrm{kite}", L"\mathrm{attractor}"],
            [L"\psi", L"\chi", L"\chi_{\mathrm{set}}"],
            [L"\chi - \chi_{\mathrm{set}}", L"\psi - \chi_{\mathrm{set}}",
             L"\psi' - \psi'_{\mathrm{set}}"],
            [L"u_{\mathrm{s}}", L"u_{\mathrm{s,set}}"],
            nothing,
            [L"l_{\mathrm{tether}}", L"\mathrm{cycle}"],
            [L"v_{\mathrm{ro}}", L"v_{\mathrm{set}}"],
            [L"u_{\mathrm{d}}",
             L"\mathrm{wc}:~0=f_{\mathrm{low}},~1=v,~2=f_{\mathrm{high}}"],
            # A bare label, not a vector: plotx only reads a scalar one for a plain vector.
            L"0=\mathrm{park},~1=\mathrm{dive},~2=\mathrm{hold},~3=\mathrm{transition},~4=\mathrm{fig8},~5=\mathrm{final}",
        ],
        fig = replace(fig_name, "Reel-out" => project_name) * " – time series",
        disp = disp,
    )
    return p
end

"""
    plot_power_scenario(scenario_dir::AbstractString; disp::Bool = true,
                       project::Union{Nothing, AbstractString} = nothing,
                       log_name::Union{Nothing, AbstractString} = nothing,
                       opt_depower_log = nothing) -> Figure

Plot the winch power panels (`F_tether`, `v_reelout`, their product
`P_mech = F * v_ro` and its running integral `E_mech`) for a scenario in
`scenario_dir`. The legends carry the scores from `reelout_power`: mean power
and energy over the reel-out window, and the whole-run energy `E_mech` ends
on. Loads the flight log the same way `plot_pattern_scenario` does — see its
docstring for `project`/`log_name`/`disp` semantics. Pass `opt_depower_log`
(as recorded by `simple_opt_reelout.jl`) to add a fifth panel comparing flown
`rel_depower` against the optimizer's own depower.
"""
function plot_power_scenario(scenario_dir::AbstractString; disp::Bool = true,
                             project::Union{Nothing, AbstractString} = nothing,
                             log_name::Union{Nothing, AbstractString} = nothing,
                             opt_depower_log = nothing)
    scenario_project = something(project, joinpath(scenario_dir, "system_reelout_150m.yaml"))

    if isnothing(log_name)
        arrow_files = filter(f -> endswith(f, ".arrow"), readdir(scenario_dir))
        isempty(arrow_files) && error("No .arrow log found in scenario folder $scenario_dir")
        log_name = replace(only(arrow_files), ".arrow" => "")
    end
    syslog = load_log(log_name; path = scenario_dir)
    sl = syslog.syslog

    flown_wind = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]["wind_speed"]
    fig_name = "V3 Kite Reel-out – $(round(flown_wind; digits = 1)) m/s"
    project_name = replace(basename(scenario_project), ".yaml" => "")

    rng = 2:length(sl.time)

    f_tether = getindex.(sl.winch_force[rng], 1)
    v_ro = getindex.(sl.v_reelout[rng], 1)
    # Sign included: reeling in against the tether is negative mechanical power.
    p_mech = f_tether .* v_ro ./ 1000
    # Running integral of the SAME series, so the curve ends on `energy_run`.
    dt_log = Float64(sl.time[2]) - Float64(sl.time[1])
    e_mech = cumsum(p_mech) .* dt_log
    # Mean and the two totals; the window is the one `reelout_power` scores.
    pm = reelout_power(sl)
    p_label = isnothing(pm) ? L"P_{\mathrm{mech}}" :
              latexstring("\\overline{P}_{\\mathrm{reel-out}} = " *
                          string(round(pm.mean_power / 1000, digits = 1)) *
                          "~\\mathrm{kW~over~}" *
                          string(round(pm.duration, digits = 1)) * "~\\mathrm{s}")
    e_label = isnothing(pm) ? L"E" :
              latexstring("E_{\\mathrm{run}} = " *
                          string(round(pm.energy_run / 1000)) * "~\\mathrm{kJ},~" *
                          "E_{\\mathrm{reel-out}} = " *
                          string(round(pm.energy / 1000)) * "~\\mathrm{kJ}")
    panels = Any[f_tether, v_ro, p_mech, e_mech]
    ylabels = Any[
        L"F_{\mathrm{tether}}~[\mathrm{N}]",
        L"v_{\mathrm{ro}}~[\mathrm{m/s}]",
        L"P_{\mathrm{mech}}~[\mathrm{kW}]",
        L"E_{\mathrm{mech}}~[\mathrm{kJ}]",
    ]
    labels = Any[
        nothing,
        nothing,
        p_label,
        e_label,
    ]
    if !isnothing(opt_depower_log) && !isempty(opt_depower_log)
        t_dp = Float64[e.t for e in opt_depower_log]
        u_dp = Float64[e.u_p_equiv for e in opt_depower_log]
        # Right-continuous step hold: the optimizer's value applies from the
        # reopt that produced it until the next one.
        idx = searchsortedlast.(Ref(t_dp), Float64.(sl.time[rng]))
        u_p_opt = [i == 0 ? u_dp[1] : u_dp[i] for i in idx]
        push!(panels, [Float64.(sl.var_14[rng]), u_p_opt])
        push!(ylabels, L"u_d~[-]")
        push!(labels, [L"\mathrm{flown}", L"\mathrm{optimizer~(equiv.)}"])
    end

    p = plotx(
        sl.time[rng],
        panels...;
        xlabel = L"\mathrm{time}~[\mathrm{s}]",
        ysize = 18,
        legendsize = 16,
        ylabels = ylabels,
        labels = labels,
        fig = replace(fig_name, "Reel-out" => project_name) * " – power",
        disp = disp,
    )
    return p
end

"""
    plot_aerodynamics_scenario(scenario_dir::AbstractString; disp::Bool = true,
                              project::Union{Nothing, AbstractString} = nothing,
                              log_name::Union{Nothing, AbstractString} = nothing) -> Figure

Plot the aerodynamics panels (angle of attack at centre vs. span-mean, L/D of
the wing vs. effective, and apparent vs. kite speed) for a scenario in
`scenario_dir`. Loads the flight log the same way `plot_pattern_scenario`
does — see its docstring for `project`/`log_name`/`disp` semantics.
"""
function plot_aerodynamics_scenario(scenario_dir::AbstractString; disp::Bool = true,
                                    project::Union{Nothing, AbstractString} = nothing,
                                    log_name::Union{Nothing, AbstractString} = nothing)
    scenario_project = something(project, joinpath(scenario_dir, "system_reelout_150m.yaml"))

    if isnothing(log_name)
        arrow_files = filter(f -> endswith(f, ".arrow"), readdir(scenario_dir))
        isempty(arrow_files) && error("No .arrow log found in scenario folder $scenario_dir")
        log_name = replace(only(arrow_files), ".arrow" => "")
    end
    syslog = load_log(log_name; path = scenario_dir)
    sl = syslog.syslog

    flown_wind = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]["wind_speed"]
    fig_name = "V3 Kite Reel-out – $(round(flown_wind; digits = 1)) m/s"
    project_name = replace(basename(scenario_project), ".yaml" => "")

    rng = 2:length(sl.time)

    # Written out rather than `norm.` so this function needs no LinearAlgebra import.
    v_kite = [sqrt(sum(abs2, v)) for v in sl.vel_kite[rng]]

    p = plotx(
        sl.time[rng],
        [rad2deg.(Float64.(sl.AoA[rng])), Float64.(sl.var_09[rng])],
        [Float64.(sl.var_15[rng]), Float64.(sl.var_16[rng])],
        [Float64.(sl.v_app[rng]), v_kite];
        xlabel = L"\mathrm{time}~[\mathrm{s}]",
        ysize = 18,
        legendsize = 20,
        ylabels = [
            L"\alpha~[°]",
            L"L/D~[-]",
            L"v~[\mathrm{m/s}]",
        ],
        labels = [
            [L"\alpha_{\mathrm{centre}}", L"\alpha_{\mathrm{span~mean}}"],
            [L"\mathrm{wing}", L"\mathrm{effective}"],
            [L"v_{\mathrm{app}}", L"|v_{\mathrm{kite}}|"],
        ],
        fig = replace(fig_name, "Reel-out" => project_name) * " – aerodynamics",
        disp = disp,
    )
    return p
end
