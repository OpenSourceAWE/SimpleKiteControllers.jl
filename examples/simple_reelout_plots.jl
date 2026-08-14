# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plotting for simple_reelout.jl results.

A copy of `simple_fig8_plots.jl` with the time-series figure's tether-length
panel changed from a flat `l0` line to the actual REEL_OUT setpoint (`var_10`),
and two panels added: reel-out speed (measured `v_reelout` vs the setpoint
`var_11`) and the WinchController's own state (`var_12`: 0 lower-force limit, 1
speed control, 2 upper-force limit) — distinct from the bottom panel's ENTRY
state machine (0 park … 4 settled). See `simple_fig8_plots.jl`'s docstring for
everything else, which is unchanged here.

Plus one figure that script has no counterpart for: `power`, the winch triple
`F_tether`, `v_reelout` and their product `P_mech = F * v_ro` [kW], all
measured, over a running integral `E_mech` [kJ]. The legends carry the scores
from `reelout_power`: mean power and energy over the pay-out window, and the
whole-run energy the `E_mech` curve ends on — which is the number to compare
across runs, since a change to when reel-out engages moves mean power and window
length in opposite directions. It is a selectable plot like the others, so
`select_plots()` shows it in the menu; `simple_fig8_plots.jl` ignores the key.

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

# Read fresh from data/gui.yaml on every include, same as simple_reelout.jl,
# so a manual re-include never plots against a stale project or fcs.
include(joinpath(@__DIR__, "gui_state.jl"))
project = project_file(selected_project())
# Same rule as simple_reelout.jl: an `fcs` already in `Main` wins. Included at the
# end of a run that is the fcs actually FLOWN, so rebuilding it from the YAML
# here would both draw the wrong reference path and throw away the caller's
# assignments; only a standalone re-include with no `fcs` around reads the file.
if !(@isdefined(fcs) && fcs isa FC_Settings)
    fcs = FC_Settings(fc_settings(project))
end
plots = selected_plots()
output_path = normpath(joinpath(@__DIR__, "..", "output"))
log_name = basename(Settings(project).log_file)
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

# From the LAST row, so a log whose centre moved still overlays where it ended up.
el_c_end = Float64(sl.var_04[end])
ref_az, ref_el = figure_eight_path(fcs.f8_a, fcs.f8_b, fcs.f8_c, fcs.f8_d,
                                   0.0, el_c_end, 0.0, 361)

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
        legend = [L"\mathrm{flown}", L"\mathrm{reference}"],
        fig = replace(fig_name, "Reel-out" => project_name) * " – pattern",
    )
    display(p1)
    sleep(0.1)
end

if "time_series" in plots
    @info "Plotting the time series..."
    # `getindex` because l_tether/v_reelout are one entry per tether and the V3 has one.
    l_tether = getindex.(sl.l_tether[rng], 1)
    l_set = Float64.(sl.var_10[rng])          # REEL_OUT setpoint, not flat like simple_fig8
    v_reelout = getindex.(sl.v_reelout[rng], 1)
    v_set = Float64.(sl.var_11[rng])
    rc_state = Float64.(sl.var_12[rng])       # WinchController state: 0/1/2
    # Entry state machine; the codes stay 0-based, other scripts search for `>= 3`.
    state = Float64.(sl.sys_state[rng])
    p2 = plotx(
        sl.time[rng],
        sl.var_01[rng],
        [el_deg, Float64.(sl.var_04[rng])],
        [rad2deg.(onto(chi_u, chi, psi)), rad2deg.(chi_u),
         rad2deg.(onto(chi_u, chi, chiset))],
        [err_course, err_heading, Float64.(sl.var_06[rng])],
        (100.0 .* sl.steering[rng], 100.0 .* sl.set_steering[rng]),
        getindex.(sl.winch_force[rng], 1),
        [l_tether, l_set],
        [v_reelout, v_set],
        rc_state,
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
            L"l_{\mathrm{tether}}~[\mathrm{m}]",
            L"v_{\mathrm{ro}}~[\mathrm{m/s}]",
            L"\mathrm{rc~state}~[-]",
            L"\mathrm{state}~[-]",
        ],
        labels = [
            nothing,
            [L"\mathrm{kite}", L"\mathrm{pattern~centre}"],
            [L"\psi", L"\chi", L"\chi_{\mathrm{set}}"],
            [L"\chi - \chi_{\mathrm{set}}", L"\psi - \chi_{\mathrm{set}}",
             L"\mathrm{regulated}"],
            [L"u_{\mathrm{s}}", L"u_{\mathrm{s,set}}"],
            nothing,
            [L"l_{\mathrm{tether}}", L"l_{\mathrm{set}}"],
            [L"v_{\mathrm{ro}}", L"v_{\mathrm{set}}"],
            L"0=\mathrm{lower~force},~1=\mathrm{speed},~2=\mathrm{upper~force}",
            # A bare label, not a vector: plotx only reads a scalar one for a plain vector.
            L"0=\mathrm{park},~1=\mathrm{dive},~2=\mathrm{hold},~3=\mathrm{fig8},~4=\mathrm{settled}",
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
        legendsize = 16,
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
