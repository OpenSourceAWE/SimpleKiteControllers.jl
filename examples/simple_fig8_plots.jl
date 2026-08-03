# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plotting for simple_fig8.jl results.

Loads the "fig8_run" log saved by simple_fig8.jl and produces three figures:

1. the flown pattern in the (azimuth, elevation) plane against the reference
   lemniscate. The attractor track is deliberately not drawn — it hugs the
   reference path by construction and obscures the two curves that matter;
2. a stacked time series: cross-track error, elevation with the pattern centre,
   heading and course vs the commanded course, the course-tracking error,
   steering command vs the KCU's tape-lagged value, tether force, and tether
   length against its setpoint. The bottom panel is the entry state machine
   (`sys_state`: 0 park, 1 dive, 2 hold, 3 fig8), which dates every feature
   above it — an anomaly in phase 1-2 is the open-loop entry, one in phase 3 is
   the path controller, and they are not diagnosed the same way;
3. an aerodynamics figure: angle of attack and lift-to-drag ratio, over the
   apparent wind speed and the kite speed `|vel_kite|`.

Two curves per panel, where the pair is the point:

- **AoA**: `AoA` is the VSM's CENTRE PANEL angle, `var_09` the span mean. They
  agree while the wing is loaded symmetrically and separate in a turn, so their
  gap reads how hard the wing is being worked.
- **L/D**: `var_15` is the wing alone, `var_16` includes tether, bridle and KCU
  drag and is what sets the achievable speed. A GAP is an unloaded wing, not
  missing data — `step!` logs `NaN` below its `drag_floor`.
- **angles**: the guidance commands a COURSE while the inner loop regulates
  HEADING, so `chi - psi` is the kite's drift angle (~13°). Plotted UNWRAPPED;
  the error panel below carries the same errors wrapped to ±180°.
- **speeds**: `v_app` and `|vel_kite|` differ by roughly the wind speed, and the
  heading/course blend is scheduled on the latter, so a mid-manoeuvre crossing
  of the `v_kite_heading`/`v_kite_course` band shows up here.

Judge path following by `chi - chi_set`. The third curve in the error panel is
the error the PID actually regulated (`var_06`), which rides on `psi - chi_set`
at low kite speed and on `chi - chi_set` at high.

Run from the REPL after (or instead of, if "fig8_run" already exists) running
simple_fig8.jl:

    include("simple_fig8_plots.jl")
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
# Where simple_fig8.jl saved the log; `init` no longer moves the data path.
set_data_path(skc_data_path())

# Reuses the run's own `fcs` when included from it, so the overlay cannot drift.
@isdefined(fcs) || (fcs = FC_Settings("fc_settings.yaml"))
syslog = load_log("fig8_run")
sl = syslog.syslog

created_at = log_created_at("fig8_run")
fig_name = "V3 Kite Figure-of-Eight"
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

@info "Plotting the pattern..."
p1 = plotxy(
    [az_deg, ref_az],
    [el_deg, ref_el];
    xlabel = L"\mathrm{azimuth}~[°]",
    ylabel = L"\mathrm{elevation}~[°]",
    legend = [L"\mathrm{flown}", L"\mathrm{reference}"],
    fig = fig_name * " – pattern",
)
display(p1)
sleep(0.1)

@info "Plotting the time series..."
# `getindex` because l_tether is one entry per tether and the V3 has one.
l_tether = getindex.(sl.l_tether[rng], 1)
l_set = fill(Float64(sl.l_tether[1][1]), length(rng))
# Entry state machine; the codes stay 0-based, other scripts search for `== 3`.
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
        [L"l_{\mathrm{tether}}", L"l_{0}"],
        # A bare label, not a vector: plotx only reads a scalar one for a plain vector.
        L"0=\mathrm{park},~1=\mathrm{dive},~2=\mathrm{hold},~3=\mathrm{fig8}",
    ],
    fig = fig_name * " – time series",
)
display(p2)
sleep(0.1)

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

nothing
