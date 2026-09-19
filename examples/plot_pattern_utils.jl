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
    load_opt_paths(scenario_dir, log_name; phases = (0, 3, 4)) -> Union{Nothing, Tuple{Vector{Float64}, Vector{Float64}}}

The optimizer's uncorrected paths of an optimized reel-out run, from the
`<log_name>_opt_paths.yaml` that `reelout_results.jl` writes next to the log, as
one `(az, el)` pair of series [deg] — the curves joined by `NaN` so Makie draws
the family in one colour under one legend entry, each closed by repeating its
first point. Only the paths installed in one of `phases` (the startup path counts
as phase 0): a path is flown from its install on, so one installed in phase 5 is
flown by phase 5 alone, which the pattern plot masks out of the flown curve too.
`nothing` when the file is absent (a lemniscate run, or a log from before it was
written) or no path is left.
"""
function load_opt_paths(scenario_dir::AbstractString, log_name::AbstractString;
                        phases = (0, 3, 4))
    file = joinpath(scenario_dir, log_name * "_opt_paths.yaml")
    isfile(file) || return nothing
    paths = get(V3Kite.YAML.load_file(file), "paths", nothing)
    paths isa AbstractVector || return nothing
    # A file from before the phase was recorded keeps every path.
    paths = [p for p in paths if get(p, "installed_phase", 0) in phases]
    isempty(paths) && return nothing
    oaz = Float64[]; oel = Float64[]
    for (k, p) in enumerate(paths)
        paz, pel = Float64.(p["azimuth"]), Float64.(p["elevation"])
        k > 1 && (push!(oaz, NaN); push!(oel, NaN))
        append!(oaz, paz); push!(oaz, first(paz))
        append!(oel, pel); push!(oel, first(pel))
    end
    return (oaz, oel)
end

"""
    plot_pattern_scenario(scenario_dir::AbstractString; disp::Bool = true,
                         opt_raw::Union{Nothing, Tuple} = nothing,
                         project::Union{Nothing, AbstractString} = nothing,
                         log_name::Union{Nothing, AbstractString} = nothing,
                         hide_before_final::Union{Nothing, Real} = nothing) -> Figure

Plot the azimuth/elevation flight pattern for a scenario in `scenario_dir`
(flown path vs. attractor reference, or vs. optimizer-raw if available).
Loads the flight log from `scenario_dir`. The system project defaults to
`scenario_dir`'s own `system_reelout_150m.yaml` copy, present in an archived
scenario folder (`output/scenarios/<name>` or `output/archives/<run>`); pass
`project` explicitly for a live run in `output/`, whose project file lives in
`data/` and is never copied there. When `disp=false`, the figure is created
but not displayed (suitable for batch processing). `opt_raw` may be passed as
`(az_array, el_array)` to overlay the optimizer's uncorrected paths; if absent,
they are read from `<log_name>_opt_paths.yaml` in `scenario_dir` when an
optimized reel-out run wrote one (see [`load_opt_paths`](@ref)), and the
attractor reference is the fallback for a run without it. Pass `log_name` explicitly to
avoid ambiguity when multiple logs exist in the directory (common in `output/`
after multiple runs); if absent and `project` is provided, searches for the
one `.arrow` file. The flown curve leaves out phase 5 and the
`hide_before_final` seconds [s] before it, where the pattern is already lifted
by `el_offset_final` ahead of the end of reel-out; the default is the scenario's
own `fcs.el_offset_lead`, which is exactly that window, and `0` hides phase 5
alone.
"""
function plot_pattern_scenario(scenario_dir::AbstractString; disp::Bool = true,
                               opt_raw::Union{Nothing, Tuple} = nothing,
                               project::Union{Nothing, AbstractString} = nothing,
                               log_name::Union{Nothing, AbstractString} = nothing,
                               hide_before_final::Union{Nothing, Real} = nothing)
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

    # Falls back to the project's own base wind speed for a summary written
    # before this field existed (or one from a script that never logged the
    # override actually flown).
    summary_sim = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]
    flown_wind = get(summary_sim, "wind_speed", project_set.v_wind)
    fig_name = "V3 Kite Reel-out – $(round(flown_wind; digits = 1)) m/s"
    project_name = replace(basename(scenario_project), ".yaml" => "")

    # Skip t=0 (guidance slots filled from first step! onward)
    rng = 2:length(sl.time)

    # Azimuth and elevation over the flight phase; phase 5 (final descent after
    # the winch stops) is masked out so it does not distort the pattern plot,
    # and so are the `hide_before_final` seconds before it: `el_offset_final`
    # latches `fcs.el_offset_lead` seconds ahead of the end of reel-out, so the
    # last part of phase 4 already flies the lifted pattern and ends a lap
    # ~5° above the others at the crossing (measured 2026-09-20).
    i5 = findfirst(==(5), sl.sys_state)
    hide_s = something(hide_before_final, fcs.el_offset_lead)
    t_hide = isnothing(i5) ? Inf : sl.time[i5] - hide_s
    hidden(i) = sl.sys_state[i] == 5 || sl.time[i] >= t_hide
    az_deg = [hidden(i) ? NaN : rad2deg(sl.azimuth[i]) for i in rng]
    el_deg = [hidden(i) ? NaN : rad2deg(sl.elevation[i]) for i in rng]

    # The optimizer's uncorrected curves, when the run wrote them and the caller
    # passed none: the reference the correction is meant to land the kite on.
    isnothing(opt_raw) && (opt_raw = load_opt_paths(scenario_dir, log_name))

    # Reference path: the logged attractor (live, walking every path under re-opt)
    # or fallback to the lemniscate. Phase 5 is left out, matching the flown mask.
    live = [i for i in rng if sl.sys_state[i] in (3, 4) &&
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

    # Falls back to the project's own base wind speed for a summary written
    # before this field existed (or one from a script that never logged the
    # override actually flown).
    summary_sim = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]
    flown_wind = get(summary_sim, "wind_speed", Settings(scenario_project).v_wind)
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

    # Falls back to the project's own base wind speed for a summary written
    # before this field existed (or one from a script that never logged the
    # override actually flown).
    summary_sim = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]
    flown_wind = get(summary_sim, "wind_speed", Settings(scenario_project).v_wind)
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

    # Falls back to the project's own base wind speed for a summary written
    # before this field existed (or one from a script that never logged the
    # override actually flown).
    summary_sim = V3Kite.YAML.load_file(joinpath(scenario_dir, log_name * ".yaml"))["simulation"]
    flown_wind = get(summary_sim, "wind_speed", Settings(scenario_project).v_wind)
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

"""
    wind_tag(v_wind::Real) -> String

Scenario-style tag for a wind speed, matching `output/scenarios/vNN` naming:
`"v03"` for an integer m/s, `"v03.5"` when it is not. Used to name a live
run's own exported plots the way an archived scenario's folder already is,
so the two land under the same naming scheme in `notebooks/images/`.
"""
function wind_tag(v_wind::Real)
    r = round(v_wind; digits = 1)
    whole = floor(Int, r)
    isapprox(r, whole) ? "v" * lpad(whole, 2, '0') :
                         "v" * lpad(whole, 2, '0') * "." * string(round(Int, 10 * (r - whole)))
end

"""
    build_path3d_figure(sl, rng; static_export = false) -> (fig, path)

3D flight-path figure: the flown trajectory in the ENU world frame (raw
Makie, not MakieControlPlots, which has no 3D plot), the ground track
underneath it and the straight line to the ground station at the origin for
depth, coloured by the measured mechanical winch power
`P_mech = F_tether * v_ro` [kW] — signed, so the entry phase's reel-in is the
dark end of the colorbar and the reeling-out figure-of-eights the bright one.
The kite position is no logged field of its own: X/Y/Z hold ALL particle
positions (`ss.X[point.idx] = point.pos_w[1]`), so it is reconstructed here
exactly as V3Kite's `pos_kite` does — the centre of pressure of the four
mid-span wing points 10..13 (10/12 leading edge, 11/13 trailing edge), each
pair weighted 0.7 LE + 0.3 TE for the ~30 % chord position and then averaged
over both sides. `path` is the coloured line, returned for a caller that
wants a colorbar on its own figure layout. Renders through whichever Makie
backend is currently `activate!`d.

The axis block depends on the destination. By default an `Axis3` with
`aspect = :data`, so the pattern is not stretched by the tether length
dominating the x range — the right choice for a live window, GLMakie or
WGLMakie. With `static_export = true` an `LScene` instead: `Axis3` drives its
camera from Julia, so in an HTML file exported with
[`save_path3d_html`](@ref) (no Julia behind it) the view cannot be turned at
all, whereas `LScene`'s `Camera3D` is the one camera WGLMakie's JS bundle
re-implements browser-side, which keeps rotate/zoom working offline. The
`LScene` axis has plainer labels and no `:data` aspect setting — its
perspective camera has data aspect anyway.
"""
function build_path3d_figure(sl, rng; static_export::Bool = false)
    cop(C) = (0.7 .* getindex.(C, 10) .+ 0.3 .* getindex.(C, 11) .+
              0.7 .* getindex.(C, 12) .+ 0.3 .* getindex.(C, 13)) ./ 2
    x_kite = Float64.(cop(sl.X[rng]))
    y_kite = Float64.(cop(sl.Y[rng]))
    z_kite = Float64.(cop(sl.Z[rng]))
    p_kite = Float64.(getindex.(sl.winch_force[rng], 1) .*
                      getindex.(sl.v_reelout[rng], 1)) ./ 1000
    fig = Figure(size = (1000, 780))
    ax = if static_export
        LScene(fig[1, 1]; show_axis = true)
    else
        Axis3(fig[1, 1];
            xlabel = L"x~[\mathrm{m}]",
            ylabel = L"y~[\mathrm{m}]",
            zlabel = L"z~[\mathrm{m}]",
            xlabelsize = 18, ylabelsize = 18, zlabelsize = 18,
            aspect = :data,
        )
    end
    lines!(ax, x_kite, y_kite, zeros(length(z_kite));
        color = (:gray, 0.4), linewidth = 1, label = L"\mathrm{ground~track}")
    # The ground station sits at the origin (that is the frame `calc_elevation`
    # and `calc_azimuth` measure in); a straight line to it, sag ignored — the
    # tether particles ARE logged, but at indices that depend on the tether's
    # `n_segments`, which the log alone does not carry.
    lines!(ax, [0.0, x_kite[end]], [0.0, y_kite[end]], [0.0, z_kite[end]];
        color = (:black, 0.5), linewidth = 1, linestyle = :dash,
        label = L"\mathrm{tether~(straight)}")
    path = lines!(ax, x_kite, y_kite, z_kite;
        color = p_kite, colormap = :viridis, linewidth = 2)
    scatter!(ax, [x_kite[1]], [y_kite[1]], [z_kite[1]];
        color = :green, markersize = 14, label = L"\mathrm{start}")
    scatter!(ax, [x_kite[end]], [y_kite[end]], [z_kite[end]];
        color = :red, markersize = 14, label = L"\mathrm{end}")
    scatter!(ax, [0.0], [0.0], [0.0];
        color = :black, marker = :rect, markersize = 12,
        label = L"\mathrm{ground~station}")
    Colorbar(fig[1, 2], path; label = L"P_{\mathrm{mech}}~[\mathrm{kW}]",
        labelsize = 18)
    axislegend(ax; position = :rt, labelsize = 16)
    if static_export
        # The old 3D axis renders its names through UnicodeFun, not MathTeXEngine,
        # so plain strings here rather than the L"" labels of the Axis3 branch.
        names = ax.scene[OldAxis].names[]
        names.axisnames[] = ("x [m]", "y [m]", "z [m]")
        names.fontsize[] = (9.0, 9.0, 9.0)
    end
    return fig, path
end

"""
    save_path3d_html(html_file::AbstractString, fig)

Save `fig` (from [`build_path3d_figure`](@ref)) as a single self-contained
interactive HTML file at `html_file`, via WGLMakie/Bonito's `export_static`.
Plain `save(path, fig)` writes a page that loads its JS bundle from
`http://localhost:<port>/...` instead — fine for the live `display(fig)` in
the same session, but broken the moment that Bonito server (or the Julia
process) is gone, which is the normal state of a file meant to sit in
`notebooks/images/`. Requires WGLMakie to be `import`ed (not merely
installed) in the calling script and its backend `activate!`d before this is
called — `WGLMakie` is resolved as a global at call time, so the order
relative to this function's own definition does not matter.

The page scales the canvas down to the viewport width with a CSS transform
(a few lines of inline JS), so on a phone or in a narrow iframe the whole
figure is visible instead of its left edge. A transform rather than
WGLMakie's `resize_to`: that one asks Julia to re-layout the figure on every
resize, and the exported page has no Julia behind it.
"""
function save_path3d_html(html_file::AbstractString, fig)
    mkpath(dirname(html_file))
    w, h = size(fig.scene)
    DOM = WGLMakie.Bonito.DOM
    # A transform shrinks what is drawn, not the element's layout box, so the
    # scaled canvas sits in a clipping box that is resized to the scaled size —
    # otherwise the 1000 px box still gives the page a horizontal scrollbar.
    app = WGLMakie.Bonito.App() do
        DOM.div(
            DOM.style("""
                html, body { margin: 0; overflow: hidden; }
                #skc-fit { overflow: hidden; width: $(w)px; height: $(h)px; }
                #skc-scaler { transform-origin: top left; width: $(w)px; height: $(h)px; }
            """),
            DOM.div(DOM.div(fig; id = "skc-scaler"); id = "skc-fit"),
            DOM.script("""
                (function () {
                  const box = document.getElementById("skc-fit");
                  const el = document.getElementById("skc-scaler");
                  const fit = () => {
                    const k = Math.min(1, document.documentElement.clientWidth / $w);
                    el.style.transform = "scale(" + k + ")";
                    box.style.width = ($w * k) + "px";
                    box.style.height = ($h * k) + "px";
                  };
                  fit();
                  window.addEventListener("resize", fit);
                })();
            """),
        )
    end
    WGLMakie.Bonito.export_static(html_file, app)
    return html_file
end

"""
    plot_path3d_scenario(scenario_dir::AbstractString;
                        project::Union{Nothing, AbstractString} = nothing,
                        log_name::Union{Nothing, AbstractString} = nothing,
                        static_export::Bool = false) -> Figure

[`build_path3d_figure`](@ref) for a scenario in `scenario_dir`, loading the
flight log the same way `plot_pattern_scenario` does — see its docstring for
`project`/`log_name` semantics, and `build_path3d_figure`'s for
`static_export`. Renders through whichever backend is currently active; the
caller picks GLMakie or WGLMakie before calling this.
"""
function plot_path3d_scenario(scenario_dir::AbstractString;
                              project::Union{Nothing, AbstractString} = nothing,
                              log_name::Union{Nothing, AbstractString} = nothing,
                              static_export::Bool = false)
    if isnothing(log_name)
        arrow_files = filter(f -> endswith(f, ".arrow"), readdir(scenario_dir))
        isempty(arrow_files) && error("No .arrow log found in scenario folder $scenario_dir")
        log_name = replace(only(arrow_files), ".arrow" => "")
    end
    syslog = load_log(log_name; path = scenario_dir)
    sl = syslog.syslog
    rng = 2:length(sl.time)
    fig, _ = build_path3d_figure(sl, rng; static_export)
    return fig
end
