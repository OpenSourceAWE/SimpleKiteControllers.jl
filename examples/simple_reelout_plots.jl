# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plotting for simple_reelout.jl results.

A copy of `simple_fig8_plots.jl` with the time-series figure's tether-length
panel changed from a flat `l0` line to the actual flown length against `fig_8`
(the live lap count) on a secondary right-hand y-axis — `plotx`'s twin-axis
form, triggered by a 2-element `ylabels` entry. Two panels added: reel-out
speed (measured `v_reelout` vs the setpoint `var_11`) and the commanded
depower `u_d` (`var_14`, `rel_depower` — filled by `step!` itself, since
`SysState` has no `set_depower` field) — worth watching here because
`depower_final` steps it up once phase 5 (final) begins. The bottom panel is
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

`power` shows the winch triple
`F_tether`, `v_reelout` and their product `P_mech = F * v_ro` [kW], all
measured, over a running integral `E_mech` [kJ]. The legends carry the scores
from `reelout_power`: mean power and energy over the reel-out window, and the
whole-run energy the `E_mech` curve ends on — which is the number to compare
across runs, since a change to when reel-out engages moves mean power and window
length in opposite directions. Both are selectable plots like the others, so
`select_plots()` shows them in the menu; `simple_fig8_plots.jl` ignores the keys.

Run from the REPL after (or instead of, if the log already exists) running
simple_reelout.jl:

    include("simple_reelout_plots.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using GLMakie
using MakieControlPlots
using LaTeXStrings
using V3Kite                  # the log, its data path, wrap_to_pi
using SimpleKiteControllers   # FC_Settings, figure_eight_path

@info "Loading simulation results..."
# Where simple_reelout.jl saved the log; `init` no longer moves the data path.
set_data_path(skc_data_path())

# Read fresh from data/gui.yaml on every include, same as simple_reelout.jl (a
# fig8 selection falls back to the reel-out default there and here alike), so a
# manual re-include never plots against a stale project or fcs.
include(joinpath(@__DIR__, "gui_state.jl"))
project = project_file(selected_reelout_project())
# Same rule as simple_reelout.jl: an `fcs` already in `Main` wins. Included at the
# end of a run that is the fcs actually FLOWN, so rebuilding it from the YAML
# here would both draw the wrong reference path and throw away the caller's
# assignments; only a standalone re-include with no `fcs` around reads the file.
if !(@isdefined(fcs) && fcs isa FC_Settings)
    fcs = FC_Settings(fc_settings(project))
end
plots = selected_plots()
output_path = normpath(joinpath(@__DIR__, "..", "output"))
# A run that flew an externally optimized path (simple_opt_reelout.jl) logs under
# `<log_file>_opt` and leaves the name in `LOG_NAME`, so the two runs of one
# project keep separate logs and can be plotted against each other.
log_name = (@isdefined(LOG_NAME) && LOG_NAME isa AbstractString) ? LOG_NAME :
           basename(Settings(project).log_file)
syslog = load_log(log_name; path = output_path)
sl = syslog.syslog

created_at = log_created_at(log_name; path = output_path)
fig_name = "V3 Kite Reel-out"
if !isnothing(created_at)
    fig_name *= " – " * replace(first(split(created_at, '.')), "T" => "_")
end

# Skip t=0: the guidance slots are filled from the first `step!` onward.
rng = 2:length(sl.time)

az_deg = rad2deg.(sl.azimuth[rng])
el_deg = rad2deg.(sl.elevation[rng])

# Written out rather than `norm.` so this script needs no LinearAlgebra import.
v_kite = [sqrt(sum(abs2, v)) for v in sl.vel_kite[rng]]

# The reference as it was actually steered at: the logged attractor point, which
# walks every path the run flew. A static curve is the LAST path only, and under
# re-optimization a run flies several. From phase 3 on, where the guidance is what
# steers — before that Q is still switching lobes under a kite that is parked, and
# the first step's slots are zero because they are written after `step!` runs.
# `REF_PATH`/`fcs.f8_*` stay as the fallback for a log without those slots.
live = [i for i in rng if sl.sys_state[i] >= 3 &&
        !(iszero(sl.var_02[i]) && iszero(sl.var_03[i]))]
ref_az, ref_el = if !isempty(live)
    Float64.(sl.var_02[live]), Float64.(sl.var_03[live])
elseif @isdefined(REF_PATH) && REF_PATH isa Tuple
    REF_PATH
else
    figure_eight_path(fcs.f8_a, fcs.f8_b, fcs.f8_c, fcs.f8_d, 0.0,
                      Float64(sl.var_04[end]), 0.0, 361)
end

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

if "pattern" in plots
    @info "Plotting the pattern..."
    project_name = replace(basename(project), ".yaml" => "")
    p1 = plotxy(
        [az_deg, ref_az],
        [el_deg, ref_el];
        xlabel = L"\mathrm{azimuth}~[°]",
        ylabel = L"\mathrm{elevation}~[°]",
        legend = [L"\mathrm{flown}", L"\mathrm{attractor}"],
        fig = replace(fig_name, "Reel-out" => project_name) * " – pattern",
    )
    display(p1)
    sleep(0.1)
end

if "path_3d" in plots
    @info "Plotting the 3D flight path..."
    # The kite position is no logged field of its own: X/Y/Z hold ALL particle
    # positions (`ss.X[point.idx] = point.pos_w[1]`), so reconstruct it the way
    # V3Kite's `pos_kite` does — the centre of pressure of the four mid-span
    # wing points 10..13 (10/12 leading edge, 11/13 trailing edge), each pair
    # weighted 0.7 LE + 0.3 TE for the ~30 % chord position and then averaged
    # over both sides.
    cop(C) = (0.7 .* getindex.(C, 10) .+ 0.3 .* getindex.(C, 11) .+
              0.7 .* getindex.(C, 12) .+ 0.3 .* getindex.(C, 13)) ./ 2
    x_kite = Float64.(cop(sl.X[rng]))
    y_kite = Float64.(cop(sl.Y[rng]))
    z_kite = Float64.(cop(sl.Z[rng]))
    # Mechanical winch power, the same product the `power` figure below plots and
    # recomputed here so the two blocks stay independent of each other's
    # selection. Sign included: the entry phase's reel-IN (the force-floor guard)
    # is negative, and the colorbar's 0 tick separates the two.
    p_kite = Float64.(getindex.(sl.winch_force[rng], 1) .*
                      getindex.(sl.v_reelout[rng], 1)) ./ 1000
    fig3 = Figure(size = (1000, 780))
    # `aspect = :data` keeps the three axes on one scale, so the pattern is not
    # stretched by the tether length dominating the x range.
    ax3 = Axis3(fig3[1, 1];
        xlabel = L"x~[\mathrm{m}]",
        ylabel = L"y~[\mathrm{m}]",
        zlabel = L"z~[\mathrm{m}]",
        xlabelsize = 18, ylabelsize = 18, zlabelsize = 18,
        aspect = :data,
    )
    # Ground track: the same path at z = 0, which is what makes the height
    # readable in a projection that has no depth cues of its own.
    lines!(ax3, x_kite, y_kite, zeros(length(z_kite));
        color = (:gray, 0.4), linewidth = 1, label = L"\mathrm{ground~track}")
    # The ground station sits at the origin (that is the frame `calc_elevation`
    # and `calc_azimuth` measure in); a straight line to it, sag ignored — the
    # tether particles ARE logged, but at indices that depend on the tether's
    # `n_segments`, which the log alone does not carry.
    lines!(ax3, [0.0, x_kite[end]], [0.0, y_kite[end]], [0.0, z_kite[end]];
        color = (:black, 0.5), linewidth = 1, linestyle = :dash,
        label = L"\mathrm{tether~(straight)}")
    path = lines!(ax3, x_kite, y_kite, z_kite;
        color = p_kite, colormap = :viridis, linewidth = 2)
    scatter!(ax3, [x_kite[1]], [y_kite[1]], [z_kite[1]];
        color = :green, markersize = 14, label = L"\mathrm{start}")
    scatter!(ax3, [x_kite[end]], [y_kite[end]], [z_kite[end]];
        color = :red, markersize = 14, label = L"\mathrm{end}")
    scatter!(ax3, [0.0], [0.0], [0.0];
        color = :black, marker = :rect, markersize = 12,
        label = L"\mathrm{ground~station}")
    Colorbar(fig3[1, 2], path; label = L"P_{\mathrm{mech}}~[\mathrm{kW}]",
        labelsize = 18)
    axislegend(ax3; position = :rt, labelsize = 16)
    # Same window handling as MakieControlPlots' figures: a named GLMakie
    # screen, so this plot does not steal or reuse one of theirs.
    screen3 = GLMakie.Screen(title = fig_name * " – 3D path")
    display(screen3, fig3)
    sleep(0.1)
end

if "time_series" in plots
    @info "Plotting the time series..."
    # `getindex` because l_tether/v_reelout are one entry per tether and the V3 has one.
    l_tether = getindex.(sl.l_tether[rng], 1)
    v_reelout = getindex.(sl.v_reelout[rng], 1)
    v_set = Float64.(sl.var_11[rng])
    u_d = Float64.(sl.var_14[rng])            # commanded rel_depower, filled by step! itself
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
        [rad2deg.(onto(chi_u, chi, psi)), rad2deg.(chi_u),
         rad2deg.(onto(chi_u, chi, chiset))],
        [err_course, err_heading, Float64.(sl.var_06[rng])],
        (100.0 .* sl.steering[rng], 100.0 .* sl.set_steering[rng]),
        getindex.(sl.winch_force[rng], 1),
        [l_tether, fig8],
        [v_reelout, v_set],
        u_d,
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
            L"u_{\mathrm{d}}~[-]",
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
            nothing,
            # A bare label, not a vector: plotx only reads a scalar one for a plain vector.
            L"0=\mathrm{park},~1=\mathrm{dive},~2=\mathrm{hold},~3=\mathrm{transition},~4=\mathrm{fig8},~5=\mathrm{final}",
        ],
        fig = fig_name * " – time series",
    )
    display(p2)
    sleep(0.1)
end

if "power" in plots
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
    p4 = plotx(
        sl.time[rng],
        f_tether,
        v_ro,
        p_mech,
        e_mech;
        xlabel = L"\mathrm{time}~[\mathrm{s}]",
        ysize = 18,
        legendsize = 16,
        ylabels = [
            L"F_{\mathrm{tether}}~[\mathrm{N}]",
            L"v_{\mathrm{ro}}~[\mathrm{m/s}]",
            L"P_{\mathrm{mech}}~[\mathrm{kW}]",
            L"E_{\mathrm{mech}}~[\mathrm{kJ}]",
        ],
        labels = [
            nothing,
            nothing,
            # A bare label, not a vector: plotx only reads a scalar one for a plain vector.
            p_label,
            e_label,
        ],
        fig = fig_name * " – power",
    )
    display(p4)
    sleep(0.1)
end

if "aerodynamics" in plots
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
        fig = fig_name * " – aerodynamics",
    )
    display(p3)
    sleep(0.1)
end

nothing
