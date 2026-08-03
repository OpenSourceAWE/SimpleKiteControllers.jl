# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plotting for simple_fig8.jl results.

Loads the "fig8_run" log saved by simple_fig8.jl. Produces three figures:

1. the flown pattern in the (azimuth, elevation) plane against the reference
   lemniscate — the plot that shows at a glance whether the pattern is being
   flown or only approximated. The attractor track (`var_02`/`var_03`) is
   deliberately NOT drawn: it hugs the reference path by construction, so it
   added a third near-coincident curve and obscured the two that matter;
2. a stacked time-series figure: cross-track error, elevation (with the pattern
   centre), heading and course vs the commanded course, the course-tracking
   error, steering command vs the KCU's actual tape-lagged value, tether force,
   and tether length against the length setpoint. The last two belong together:
   at `compliance > 0` the drum holds a low-passed force rather than a length,
   so it pays out on every dive and hauls in on every climb, and the length
   panel is where that shows — the excursion scales with `compliance` itself.
   The setpoint line is the length at `t = 0`, which is exactly the `l0` the run
   trims back to (both come from the first logged row). At `compliance = 0` the
   two lines should sit on top of each other to within a centimetre. The bottom
   panel is the entry state machine (`sys_state`, 0-based as logged: 0 park,
   1 dive, 2 hold, 3 fig8 — the legend repeats the mapping), which dates every
   feature above it — an anomaly inside phase 1-2 is
   the open-loop entry, one inside phase 3 is the path controller, and they are
   not diagnosed the same way;
3. an aerodynamics figure: angle of attack and lift-to-drag ratio, with the
   apparent wind speed and the kite speed `|vel_kite|` underneath to read both
   against. Those two speeds are plotted in one panel on purpose: they differ by
   roughly the wind speed, and `|vel_kite|` is the signal the heading/course
   blend is scheduled on, so this is where a mid-manoeuvre crossing of the
   `v_kite_heading`/`v_kite_course` band shows up.

The AoA panel carries two curves because they answer different questions.
`AoA` is the VSM's CENTRE PANEL angle, the one `update_sys_state!` logs; the
span-mean (`var_09`, written by simple_fig8.jl) averages every panel. They
agree while the wing is loaded symmetrically and separate in a turn, where
steering twists the two halves the opposite way — so their gap is a direct read
on how hard the wing is being worked, which matters here because the steering
sits on its clamp for most of the pattern. Logs written before `var_09` existed
show that curve as a flat zero.

The L/D panel likewise shows two: `var_15` is the wing alone and `var_16` the
effective value with tether, bridle and KCU drag included (both filled by
`step!`). The second is what actually sets the achievable speed. A GAP in either
curve is an unloaded wing, not missing data: drag is a signed projection onto
the apparent wind, so it passes through zero whenever the wing unloads, and
V3Kite's `step!` logs `NaN` below its `drag_floor` rather than the arbitrarily
large ratio a vanishing denominator would produce. The `warmup_time` in
`FC_Settings` keeps the settling-to-dynamics transient, which is where this
happened most reliably, out of the log altogether.

The guidance commands a COURSE (direction of travel) while the inner loop
regulates HEADING (where the nose points), so the angle panel shows all three:
`chi` is what the kite actually flew, `chi_set` the command, and the
`chi - psi` gap is the kite's drift angle (~13° on the V3, see
`src/figure_eight_controller.jl`). Those three are plotted UNWRAPPED (a lap of the
pattern crosses ±180°, and the raw traces jump there); the error panel below
carries the same errors wrapped to ±180°.

Judge path following by `chi - chi_set`. The third curve in the error panel is
the error the PID actually regulated (`var_06`): the loop feeds back heading at
low kite speed and course at high, blending in between
(`v_kite_heading`/`v_kite_course` in `FC_Settings`, weight logged in `var_08`),
so that curve rides on `psi - chi_set` at low speed and on `chi - chi_set` at
high. On logs written before the blend existed it coincides with
`psi - chi_set`.

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
set_data_path(v3_data_path())

# Pattern geometry, used only to redraw the reference path. simple_fig8.jl
# `include`s this file at the end of a run with its `fcs` still in scope, so the
# overlay cannot drift out of sync with the run; standalone against an existing
# "fig8_run" log the settings file is read instead, which is the same source the
# run itself used unless it was overridden in the REPL.
@isdefined(fcs) || (fcs = FC_Settings("fc_settings.yaml"))
syslog = load_log("fig8_run")
sl = syslog.syslog

created_at = log_created_at("fig8_run")
fig_name = "V3 Kite Figure-of-Eight"
if !isnothing(created_at)
    fig_name *= " – " * replace(first(split(created_at, '.')), "T" => "_")
end

# Skip the t=0 initial log entry (the guidance slots are only filled from the
# first `step!` call onward).
rng = 2:length(sl.time)

az_deg = rad2deg.(sl.azimuth[rng])
el_deg = rad2deg.(sl.elevation[rng])

# Kite speed |vel_kite|, plotted against v_app because the two are far apart on
# this kite and it is the SLOW one that the heading/course blend is scheduled on
# (v_kite_heading/v_kite_course in FC_Settings): parked, v_app is already ~9
# m/s of ambient wind while the kite itself barely moves. Written out rather
# than `norm.` so this script needs no LinearAlgebra import.
v_kite = [sqrt(sum(abs2, v)) for v in sl.vel_kite[rng]]

# Reference path at the pattern centre the run logged (var_04). Read from the
# LAST row rather than assumed constant, so a log whose centre moved during the
# run still gets an overlay that matches where it ended up.
el_c_end = Float64(sl.var_04[end])
ref_az, ref_el = figure_eight_path(fcs.f8_a, fcs.f8_b, fcs.f8_c, fcs.f8_d,
                                   0.0, el_c_end, 0.0, 361)

# --- angles for the psi/chi panel ---------------------------------------- #
# Plotted UNWRAPPED: the raw ±180° traces jump at the branch cut, the three
# curves cross it at different times, and the pattern makes them cross it every
# lap. The course is unwrapped by integrating its wrapped increments; heading
# and the command are then placed on the branch nearest the unwrapped course at
# each sample, so all three stay continuous *and* their vertical distances are
# still exactly the wrapped errors plotted in the panel below.
unwrap_angle(a) = first(a) .+ cumsum(vcat(0.0, wrap_to_pi.(diff(a))))
onto(ref_u, ref_w, a) = ref_u .+ wrap_to_pi.(a .- ref_w)

psi    = Float64.(sl.heading[rng])
# +π puts the logged course into the same convention as heading and the
# guidance (0 = towards zenith); see the note in simple_fig8.jl. Without it the
# course trace sits 180° away from everything else in the panel.
chi    = wrap_to_pi.(Float64.(sl.course[rng]) .+ pi)
chiset = Float64.(sl.bearing[rng])   # chi_cmd, the course actually tracked
chi_u  = unwrap_angle(chi)

# Tracking errors, both wrapped to ±180°. chi - chi_set is the path-following
# error the guidance cares about; psi - chi_set is what the PID regulates, and
# the offset between them is the kite's drift angle (~13° on the V3).
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
# Tether length, and the setpoint the run trims back to. `l_tether` is a vector
# per sample (one entry per tether) and the V3 has one, hence the `getindex`,
# as for `winch_force`. The setpoint is the t = 0 length — the same row the run
# reads its `l0` from — drawn as a constant so payout and haul-in are readable
# as displacement from it rather than as an absolute number.
l_tether = getindex.(sl.l_tether[rng], 1)
l_set = fill(Float64(sl.l_tether[1][1]), length(rng))
# Entry state machine, logged by simple_fig8.jl to the `sys_state` field as
# 0 = park, 1 = dive, 2 = hold, 3 = fig8 guidance engaged. Plotted as its own
# panel because it shares units with nothing else, and drawn LAST so it sits
# against the time axis every other panel is read against: the three step edges
# are what date the dive, the handover and the start of the pattern.
#
# The legend spells the mapping out at the LOGGED values, which are 0-based.
# They are not a display choice: `fig_eight_plots.jl` finds the handover with
# `findfirst(==(3), sl.sys_state)`, and the codes are shared with the reference
# controller's log so the two can be read with the same scripts. Renumbering
# the panel 1..4 would make it disagree with both.
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
        # A bare label, not a vector: this channel is passed as a plain vector,
        # and plotx only reads a scalar label in that case (see plotx.jl:169).
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
