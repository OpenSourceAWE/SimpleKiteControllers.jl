# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plotting for `simple_reelout.jl` results.

A copy of `simple_fig8_plots.jl` with the time-series figure's tether-length
panel changed from a flat `l0` line to the actual flown length against `fig_8`
(the live lap count) on a secondary right-hand y-axis — `plotx`'s twin-axis
form, triggered by a 2-element `ylabels` entry. Two panels added: reel-out
speed (measured `v_reelout` vs the setpoint `var_11`) and the commanded
depower `u_d` (`var_14`, `rel_depower` — filled by `step!` itself, since
`SysState` has no `set_depower` field) — worth watching here because
`depower_final` steps it up once phase 5 (final) begins. That `u_d` panel
carries the WinchController state (`var_12`, 0 lower-force / 1 speed /
2 upper-force) on a secondary right-hand y-axis, the same twin-axis form as the
tether-length panel: the two belong together because depower is what decides
whether the winch ever leaves speed control, and at high wind it does not —
measured at 9 m/s, 87 % of the reel-out window sat in the upper-force limiter.
It is blanked before phase 3, where `rc` is not yet stepped. The bottom panel is
the ENTRY state machine (0 park … 4 fig8, plus 5 final once reel-out reaches
`reelout_l_max`). See `simple_fig8_plots.jl`'s docstring for everything else,
which is unchanged here.

Plus two figures that script has no counterpart for.

`path_3d` draws the flown trajectory in the ENU world frame with GLMakie's
`Axis3` (raw Makie, not MakieControlPlots, which has no 3D plot), with the
ground track underneath it and the straight line to the ground station at the
origin for depth. The curve is coloured by the measured mechanical winch power
`P_mech = F_tether * v_ro` [kW], so it shows WHERE in the pattern the energy is
made — signed, so the entry phase's reel-in is the dark end of the colorbar and
the reeling-out figure-of-eights the bright one. The loops flown after reel-out
reaches `reelout_l_max` are dark again for the OTHER reason: `v_ro` is zero
there, so the phase-5 pattern carries no mechanical power at all. The kite position itself is NOT a
logged field: it is reconstructed here from the logged particle positions
exactly as V3Kite's `pos_kite` does.

`path_webgl` is the same figure (`build_path3d_figure`), rendered through
WGLMakie instead of GLMakie so it opens in the browser rather than a native
window — useful over a remote/SSH session with no GLFW display, or for a plot
worth rotating in a tab of its own. Independent of `path_3d`: pick either,
both, or neither. Also saved to `notebooks/images/path_webgl_<tag>.html` via
`save_path3d_html` (Bonito's `export_static`, not plain `save`, which would
point at a `localhost` URL that dies with the Julia session) — `<tag>` is the
scenario folder's name for an archived replot, or `wind_tag(flown_wind)` for
a live run, matching `create_plots.jl`'s own `<plottype>_<scenario>` naming
for its batch-exported PNGs.

`power` shows the winch triple
`F_tether`, `v_reelout` and their product `P_mech = F * v_ro` [kW], all
measured, over a running integral `E_mech` [kJ]. The legends carry the scores
from `reelout_power`: mean power and energy over the reel-out window, and the
whole-run energy the `E_mech` curve ends on — which is the number to compare
across runs, since a change to when reel-out engages moves mean power and window
length in opposite directions. Both are selectable plots like the others, so
`select_plots()` shows them in the menu; `simple_fig8_plots.jl` ignores the keys.
A fifth panel, flown `rel_depower` (`var_14`) against the optimizer's own
depower converted to the same units (`awetrim_depower_to_v3kite`, step-held
between reopt solves), is added when the run has one — silently skipped for a
plain `simple_reelout.jl` run. A sixth carries `k_v`, the winch gain the run
flew, against the seed it was bracketed around when an archived run made it a
design variable; that one is ALWAYS drawn, flat when the gain was fixed.

Neither rides a `var_` slot — all sixteen are taken. Both are read from the RUN
SUMMARY on a scenario replot (`traj_opt.guess.depower_optimized_rel`,
`traj_opt.guess.k_v_optimized`, `traj_opt.winch.k_v_flown`) and from the live
run's `opt_depower_log`/`rc` (`live_global`) only for the run that just flew —
see `depower_series`/`kv_series` for why a replot must never touch them.
An archive from before `depower_optimized_rel` was written gets no `u_d` panel.

Run from the REPL after (or instead of, if the log already exists) running
`simple_reelout.jl`:

    include("simple_reelout_plots.jl")

`run_example("simple_reelout_plots.jl"; scenario_path = "output/scenarios/<name>")`
(`src/script_inputs.jl`) replots an archived run instead — `project_set` and the log are reloaded from that
folder's own copies rather than the live `data/` directory. `plot_scenario.jl`
is the menu-driven front end for that.

Everything happens in [`draw_reelout_plots`](@ref) and the one function per
figure it calls, so an include leaves no globals behind but its functions.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using GLMakie
using MakieControlPlots
using LaTeXStrings
using V3Kite                  # the log, its data path, wrap_to_pi
using OrderedCollections: OrderedDict
using SimpleKiteControllers   # reelout_power, apply_windspeed_override!
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own

include(joinpath(@__DIR__, "plot_pattern_utils.jl"))

# Where simple_reelout.jl saved the log; `init` no longer moves the data path.
set_data_path(skc_data_path())

"""
    live_global(name) -> value or nothing

What the live run left under `name`: a `simple_opt_reelout.jl` run that just finished
keeps it in its `setup` (and names its log in `LOG_NAME`), `simple_reelout.jl` as a
global of that name; `nothing` when neither has one.
"""
function live_global(name::Symbol)
    if @isdefined(LOG_NAME) && LOG_NAME isa AbstractString && @isdefined(setup) &&
       hasproperty(setup, name)
        return getproperty(setup, name)
    end
    return isdefined(@__MODULE__, name) ? getfield(@__MODULE__, name) : nothing
end


"The system project a scenario folder was FLOWN with, as its run summary names it, or `nothing`"
function flown_project(dir)
    # The run summary is the one YAML in the folder with a `simulation` section.
    for f in filter(f -> endswith(f, ".yaml") && !startswith(f, "system_"),
                    readdir(dir))
        y = V3Kite.YAML.load_file(joinpath(dir, f))
        y isa AbstractDict && haskey(y, "simulation") &&
            haskey(y["simulation"], "project") || continue
        return y["simulation"]["project"]
    end
    return nothing
end

"""
    plot_settings(scenario_path) -> (; project_set, pattern_project)

The settings the plots are drawn against. For a scenario folder every setting
comes from ITS OWN copies inside the folder, not the live data/ directory or
whatever a prior run left in `Main` — a scenario exists to freeze exactly the
conditions it was flown under. Otherwise the live run's (`live_global`), and only
a standalone include with no live run around reads the files of the project
selected in `data/gui.yaml`.
"""
function plot_settings(scenario_path)
    if !isnothing(scenario_path)
        # The file's own name varies by project family (`system_reelout_maasvlakte.yaml` at
        # maasvlakte, `system_reelout_cabauw.yaml` at cabauw), so it is read off the run
        # summary's `simulation.project`, which is what was FLOWN. A folder can hold more than
        # one — a project renamed between runs left the old copy behind wherever the
        # scenario was overwritten in place — and only the summary says which is right.
        project_files = filter(f -> startswith(f, "system_reelout_") && endswith(f, ".yaml"),
                               readdir(scenario_path))
        isempty(project_files) && error("No system_reelout_*.yaml in $scenario_path")
        flown = flown_project(scenario_path)
        scenario_project = joinpath(scenario_path,
            if !isnothing(flown) && flown in project_files
                flown
            elseif length(project_files) == 1
                only(project_files)
            else
                error("$scenario_path holds $(length(project_files)) project files                    ($(join(project_files, ", "))) and the run summary does not                    name one of them; remove the stale copy.")
            end)
        return (; project_set = Settings(scenario_project), pattern_project = scenario_project)
    end
    project = project_file(selected_reelout_project())
    # The wind speed actually flown: a live run's `project_set` (with any override
    # already applied by `apply_windspeed_override!`) wins (`live_global`); only a
    # standalone re-include with no live run around reads the file.
    project_set = let p = live_global(:project_set)
        p isa Settings ? p : begin
            p = Settings(project)
            apply_windspeed_override!(p, selected_windspeed())
            p
        end
    end
    return (; project_set, pattern_project = project)
end

"The log of a scenario folder: its one `.arrow` file, without the extension"
function scenario_log_name(dir)
    arrow_files = filter(f -> endswith(f, ".arrow"), readdir(dir))
    isempty(arrow_files) && error("No .arrow log found in scenario folder $dir")
    return replace(only(arrow_files), ".arrow" => "")
end

"""
    live_log_name(project_set, output_path) -> String

A run that flew an externally optimized path (`simple_opt_reelout.jl`) logs under
`<log_file>_opt` and leaves the name in `LOG_NAME`, so the two runs of one
project keep separate logs and can be plotted against each other. A standalone
re-include with no `LOG_NAME` around (a fresh session, or the plots having
failed at the end of the run that set it) takes whichever of the project's two
logs was written last — a project only ever flown by `simple_opt_reelout.jl`
(`system_reelout_cabauw.yaml`) has no plain `<log_file>.arrow` at all.
"""
function live_log_name(project_set, output_path)
    @isdefined(LOG_NAME) && LOG_NAME isa AbstractString && return LOG_NAME
    base = basename(project_set.log_file)
    candidates = filter([base, base * "_opt"]) do name
        isfile(joinpath(output_path, name * ".arrow"))
    end
    isempty(candidates) &&
        error("No $base.arrow or $(base)_opt.arrow in $output_path; run \
               simple_reelout.jl or simple_opt_reelout.jl first.")
    return argmax(name -> mtime(joinpath(output_path, name * ".arrow")), candidates)
end

"""
    load_run_summary(dir, log_name) -> OrderedDict | nothing

The run summary, written by `reelout_results.jl` BEFORE the plots and copied
into every archive. It is the durable home of everything the Arrow cannot hold:
all sixteen `var_` slots are taken, so `k_v` and the optimizer's depower are not
columns and a replot has to read them from here.
"""
function load_run_summary(dir, log_name)
    summary_file = joinpath(dir, log_name * ".yaml")
    return isfile(summary_file) ?
        V3Kite.YAML.load_file(summary_file; dicttype = OrderedDict{String, Any}) : nothing
end

"""
    kv_series(summary, times; live) -> (k_v, seed, moved)

The winch gain over `times`, step-held between optimizer replies, for the power
plot's `k_v` panel.

Parsed out of the summary's `traj_opt.guess.k_v_optimized`, which only runs from
before `optimize_k_v` was removed (2026-10-03) carry, since `k_v` is not a log
column. `moved` is false when the optimizer never retuned the gain, and the series
is then the flat gain the run actually flew, read back from the winch the law used.

`live = false` (a scenario replot) ignores `rc`/`winch` (see `live_global`) entirely and
reads only the summary. Without that, replotting an archive from the REPL that
just flew something else would draw THAT run's gain onto this run's plot — the
globals outlive the run that set them.

Entries sharing a timestamp are the two call sites of one re-optimization; they
are ordered by their summary key so the retry wins, as it does live.
"""
function kv_series(summary, times; live::Bool)
    traj = isnothing(summary) ? nothing : get(summary, "traj_opt", nothing)
    wsum = isnothing(traj) ? nothing : get(traj, "winch", nothing)
    winch = live ? live_global(:winch) : nothing
    rc = live ? live_global(:rc) : nothing
    seed = if !isnothing(winch) && hasproperty(winch, :k_v)
        Float64(winch.k_v)
    elseif !isnothing(wsum)
        Float64(wsum["k_v"])
    else
        NaN
    end
    flown = if !isnothing(rc) && hasproperty(rc, :wcs)
        Float64(rc.wcs.kv)
    elseif !isnothing(wsum)
        Float64(get(wsum, "k_v_flown", seed))
    else
        seed
    end
    guess = isnothing(traj) ? nothing : get(traj, "guess", nothing)
    block = isnothing(guess) ? nothing : get(guess, "k_v_optimized", nothing)
    rows = block isa AbstractDict ?
        [(parse(Float64, m[1]), String(k), Float64(v))
         for (k, v) in block
         for m in (match(r"^t_([\d.]+)_s", String(k)),) if m !== nothing] :
        Tuple{Float64, String, Float64}[]
    sort!(rows; by = r -> (r[1], r[2]))
    t_kv, k_kv = Float64[r[1] for r in rows], Float64[r[3] for r in rows]
    isempty(t_kv) && return fill(flown, length(times)), seed, false
    idx = searchsortedlast.(Ref(t_kv), times)
    ([i == 0 ? k_kv[1] : k_kv[i] for i in idx], seed, true)
end

"""
    depower_series(summary, times; live) -> Union{Vector{Float64}, Nothing}

The optimizer's depower over `times` as V3Kite `rel_depower`, step-held between
replies, for the power plot's `u_d` panel; `nothing` when the run has none.

Sourced from `opt_depower_log` for a live run and from the summary's
`traj_opt.guess.depower_optimized_rel` for a scenario replot (`live = false`),
the same split as `kv_series` and for the same reason: `opt_depower_log`
outlives the run that set it, so a replot reading it would draw the LAST live
run's replies against this archive's flown depower. Archives written before
that key existed get no panel rather than a wrong one.
"""
function depower_series(summary, times; live::Bool)
    opt_depower_log = live ? live_global(:opt_depower_log) : nothing
    t_dp, u_dp = if !isnothing(opt_depower_log) && !isempty(opt_depower_log)
        (Float64[e.t for e in opt_depower_log], Float64[e.u_p_equiv for e in opt_depower_log])
    else
        traj = isnothing(summary) ? nothing : get(summary, "traj_opt", nothing)
        guess = isnothing(traj) ? nothing : get(traj, "guess", nothing)
        block = isnothing(guess) ? nothing : get(guess, "depower_optimized_rel", nothing)
        rows = block isa AbstractDict ?
            [(parse(Float64, m[1]), String(k), Float64(v))
             for (k, v) in block
             for m in (match(r"^t_([\d.]+)_s", String(k)),) if m !== nothing] :
            Tuple{Float64, String, Float64}[]
        sort!(rows; by = r -> (r[1], r[2]))
        (Float64[r[1] for r in rows], Float64[r[3] for r in rows])
    end
    isempty(t_dp) && return nothing
    idx = searchsortedlast.(Ref(t_dp), times)
    [i == 0 ? u_dp[1] : u_dp[i] for i in idx]
end

"""
    reelout_plot_data(scenario_path) -> NamedTuple

Everything the figures share: the settings (`plot_settings`), the log and its run
summary, the flown wind speed, the figure-name stem, the plotted sample range
`rng` and the series more than one figure draws.
"""
function reelout_plot_data(scenario_path)
    (; project_set, pattern_project) = plot_settings(scenario_path)
    output_path = isnothing(scenario_path) ?
                  normpath(joinpath(@__DIR__, "..", "output")) : scenario_path
    # A scenario archive is identified by its one `.arrow` file, since it was moved out
    # of `output/archives/` by hand and may hold any project's log.
    log_name = isnothing(scenario_path) ? live_log_name(project_set, output_path) :
                                          scenario_log_name(scenario_path)
    sl = load_log(log_name; path = output_path).syslog
    created_at = log_created_at(log_name; path = output_path)
    run_summary = load_run_summary(output_path, log_name)
    # `project_set.v_wind` is the PROJECT's base value, not necessarily what was
    # actually flown if a WIND_SPEED override was in effect — a scenario's own run
    # summary is the only place the true value survives.
    flown_wind = if isnothing(scenario_path)
        project_set.v_wind
    elseif isnothing(run_summary)
        error("No run summary at $(joinpath(output_path, log_name * ".yaml")); the flown \
               wind speed only survives there.")
    else
        run_summary["simulation"]["wind_speed"]
    end
    fig_name = "Reel-out – $(round(flown_wind; digits = 1)) m/s"
    if !isnothing(created_at)
        fig_name *= " – " * replace(first(split(created_at, '.')), "T" => "_")
    end
    # Skip t=0: the guidance slots are filled from the first `step!` onward.
    rng = 2:length(sl.time)
    # Phase 5 (final descent after the winch stops) is masked out so it does not
    # distort the elevation panel.
    el_deg = [sl.sys_state[i] == 5 ? NaN : rad2deg(sl.elevation[i]) for i in rng]
    return (; scenario_path, pattern_project, output_path, log_name, sl, run_summary,
            flown_wind, fig_name, rng, el_deg)
end

"The flown pattern against every path the optimizer returned for it (`plot_pattern_scenario`)"
function plot_pattern(d)
    @info "Plotting the pattern..."
    # Every path the optimizer returned that the run went on to fly, BEFORE
    # `el_offset_final` and `el_offset_wing` were added to it, is drawn
    # against the flown curve. Every other curve here carries that pre-distortion —
    # the logged attractor included, since it walks the corrected path — so without
    # these there is nothing in the figure to compare the correction against. Read
    # inside `plot_pattern_scenario` from the `<log>_opt_paths.yaml` that
    # reelout_results.jl writes next to the log, so the paths installed only in the
    # window it hides from the flown curve are left out too; the attractor is the
    # fallback for a lemniscate run or a log from before it was written.
    p1 = plot_pattern_scenario(d.output_path; disp = true,
                               project = d.pattern_project, log_name = d.log_name)
    display(p1)
    sleep(0.1)
end

"The 3D flight path in a GLMakie window of its own, see the file's docstring"
function plot_path_3d(d)
    @info "Plotting the 3D flight path..."
    fig3, _ = build_path3d_figure(d.sl, d.rng)
    # Same window handling as MakieControlPlots' figures: a named GLMakie
    # screen, so this plot does not steal or reuse one of theirs.
    screen3 = GLMakie.Screen(title = d.fig_name * " – 3D path")
    display(screen3, fig3)
    sleep(0.1)
end

"The same 3D flight path in the browser (WGLMakie), also saved as an interactive HTML file"
function plot_path_webgl(d)
    @info "Plotting the 3D flight path (WGLMakie, opens in the browser)..."
    # WGLMakie is imported (not `using`d, at the bottom of this file) because it exports
    # names that clash with GLMakie's; `Figure`/`Axis3`/`lines!`/... stay the GLMakie ones
    # from the top of this file, and only WHICH BACKEND RENDERS THEM is switched by
    # `activate!`, Makie's normal multi-backend mechanism. Switched back to
    # GLMakie right after so the plots below (time_series, power,
    # aerodynamics) keep rendering in their own windows rather than the
    # browser.
    WGLMakie.activate!()
    fig4, _ = build_path3d_figure(d.sl, d.rng)
    display(fig4)
    # Also saved as a self-contained interactive HTML file into
    # notebooks/images/, alongside the PNGs create_plots.jl generates for the
    # other figures — named after the scenario folder when replotting an
    # archive (matching create_plots.jl's own `<plottype>_<scenario>` naming),
    # or after the flown wind speed for a live run in output/, which has no
    # scenario folder of its own.
    html_tag = isnothing(d.scenario_path) ? wind_tag(d.flown_wind) : basename(d.scenario_path)
    html_file = normpath(joinpath(@__DIR__, "..", "notebooks", "images",
                                  "path_webgl_$(html_tag).html"))
    # Built a second time with `static_export`: the Axis3 shown above cannot be
    # turned once no Julia session is behind the page.
    fig5, _ = build_path3d_figure(d.sl, d.rng; static_export = true)
    save_path3d_html(html_file, fig5)
    @info "Saved interactive 3D plot" html_file
    GLMakie.activate!()
    sleep(0.1)
end

# --- angles for the psi/chi panel, plotted UNWRAPPED (unwrap_angle, unwrap_onto) --- #

"The time series: guidance, course, steering, tether, reel-out, depower and the entry state machine"
function plot_time_series(d)
    (; sl, rng, el_deg) = d
    @info "Plotting the time series..."
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
    # drawing a flat line the eye reads as a measurement. Same window `fig8_metrics`
    # scores over, for the same reason.
    wc_state = [sl.sys_state[i] >= 3 ? Float64(sl.var_12[i]) : NaN for i in rng]
    # Entry state machine; the codes stay 0-based, other scripts search for `>= 3`.
    state = Float64.(sl.sys_state[rng])
    fig8 = Float64.(sl.fig_8[rng])
    p2 = plotx(
        sl.time[rng],
        sl.var_01[rng],
        # var_03, the ATTRACTOR's elevation — what the guidance is steering at, so
        # this panel reads as demanded vs actual. var_04 is the pattern centre, a
        # constant, which said nothing about tracking.
        [el_deg, Float64.(sl.var_03[rng])],
        [rad2deg.(unwrap_onto(chi_u, chi, psi)), rad2deg.(chi_u),
         rad2deg.(unwrap_onto(chi_u, chi, chiset))],
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
        fig = d.fig_name * " – time series",
    )
    display(p2)
    sleep(0.1)
end

"""
    plot_power(d)

The winch triple `F_tether`, `v_reelout`, `P_mech` and the running `E_mech`, plus
the optimizer's depower (`depower_series`) when the run has one and the winch gain
it flew (`kv_series`).
"""
function plot_power(d)
    (; sl, rng, run_summary, scenario_path) = d
    @info "Plotting the winch power..."
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
        # A bare label, not a vector: plotx only reads a scalar one for a plain vector.
        p_label,
        e_label,
    ]
    u_p_opt = depower_series(run_summary, Float64.(sl.time[rng]);
                             live = isnothing(scenario_path))
    if !isnothing(u_p_opt)
        push!(panels, [Float64.(sl.var_14[rng]), u_p_opt])
        push!(ylabels, L"u_d~[-]")
        push!(labels, [L"\mathrm{flown}", L"\mathrm{optimizer~(equiv.)}"])
    end
    # The winch gain the run flew. ALWAYS drawn — a flat line is the honest
    # picture of a fixed-gain run, and with `force_limit: "soft"` k_v sets the
    # whole reel-out law, so which value was flown belongs on the plot either way.
    # Against the seed only when the optimizer actually moved it; see `kv_series`
    # for where the numbers come from on a replot.
    kv_flown, kv_seed, kv_moved = kv_series(run_summary, Float64.(sl.time[rng]);
                                            live = isnothing(scenario_path))
    if any(isfinite, kv_flown)
        push!(panels, kv_moved ? [kv_flown, fill(kv_seed, length(kv_flown))] : kv_flown)
        push!(ylabels, L"k_v~[-]")
        # A bare label for the plain vector, as with p_label/e_label above.
        push!(labels, kv_moved ? [L"\mathrm{optimized}", L"\mathrm{seed}"] :
                                 L"k_v~\mathrm{flown~(fixed)}")
    end

    p4 = plotx(
        sl.time[rng],
        panels...;
        xlabel = L"\mathrm{time}~[\mathrm{s}]",
        ysize = 18,
        legendsize = 16,
        ylabels = ylabels,
        labels = labels,
        fig = d.fig_name * " – power",
    )
    display(p4)
    sleep(0.1)
end

"The aerodynamics: angle of attack, lift-to-drag ratio and speeds"
function plot_aerodynamics(d)
    (; sl, rng) = d
    # Written out rather than `norm.` so this script needs no LinearAlgebra import.
    v_kite = [sqrt(sum(abs2, v)) for v in sl.vel_kite[rng]]
    @info "Plotting the aerodynamics..."
    p3 = plotx(
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
        fig = d.fig_name * " – aerodynamics",
    )
    display(p3)
    sleep(0.1)
end

"""
    draw_reelout_plots(scenario_path = nothing)

Draw every figure `selected_plots()` asks for, of the live run or, with
`scenario_path`, of the archived one in that folder.
"""
function draw_reelout_plots(scenario_path = nothing)
    @info "Loading simulation results..."
    d = reelout_plot_data(scenario_path)
    plots = selected_plots()
    "pattern" in plots && plot_pattern(d)
    "path_3d" in plots && plot_path_3d(d)
    "path_webgl" in plots && plot_path_webgl(d)
    "time_series" in plots && plot_time_series(d)
    "power" in plots && plot_power(d)
    "aerodynamics" in plots && plot_aerodynamics(d)
    return nothing
end

# At top level, where an `import` must be, and only when it is needed: see `plot_path_webgl`.
if "path_webgl" in selected_plots()
    import WGLMakie
end
# A scenario folder (`output/scenarios/<name>`) to replot instead of `output/`, the input
# `scenario_path` (`plot_scenario.jl` passes it). It holds for that one run: a stale value
# must not silently redirect a LATER live run's plots at an old archive.
draw_reelout_plots(script_inputs(@__FILE__, (; scenario_path = nothing)).scenario_path)
