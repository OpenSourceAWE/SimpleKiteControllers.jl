# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Figure-of-eight path following of the V3 kite via V3Kite's `init`/`step!`
interface.

The kite starts parked at ~73° and reaches the pattern through a four-phase
entry (park -> dive -> hold -> fig8). Once engaged, the L0
attractor guidance (`src/figure_eight_controller.jl`) commands a course and a PID
(as in V3Kite's `simple_sinus.jl` / `simple_auto_parking.jl`) tracks it with the
steering tape.

# What comes from where

The guidance, its settings, the run metrics and the turn-rate table are this
package (`src/`). Everything that touches the kite — the model, the simulation
loop, both winch modes, the discarded warm-up and the span-mean AoA — is V3Kite,
used through its public API. Nothing here reaches into a `V3KITE` itself, which
is what keeps this package free of a kite-model dependency.

The CONDITIONS the model is run under come from here too: `project_file` returns
this package's `data/system_fig8_200m.yaml`, so the simulation settings it names
(`data/settings_fig8_200m.yaml`) are the ones flown, not the model's copy. That
file, not `fc_settings.yaml`, sets how long the run is (`sim_time`) and the
timestep (`1/sample_freq`); `init` falls back to both and the loop reads
`s.steps` and `s.dt` from the model. The winch-controller settings come from here
too (`data/wc_settings.yaml`, found because the data path points at this
package's `data/` until `init` moves it back). Only the geometry, the polars and
the VSM settings stay with the model.

# Why the pattern is large

The V3's turn-rate law fixes the smallest angular turn radius it can fly,
`rho = 1/(L*c1*u_s)`, and the tightest curvature of a lemniscate collapses as the
pattern is raised, because the azimuth axis is compressed by `cos(elevation)`. A
figure-eight near zenith is therefore geometrically impossible at any PID tuning:
the pattern must be flown low and wide, and the kite descends onto it.
`check_pattern_feasible` prints the margin at startup; below ~1 the tracking
error is curvature-limited, so enlarge the pattern, lower its centre, lower the
damping, or raise `max_steering`. The measured margins behind the values used
here are in `docs/fig8_tuning_log.md`.

Logs the run to "fig8_run" and `include`s `simple_fig8_plots.jl` at the end, so
the figures come up without a second call. Even a 30 s simulation is several
thousand `step!` calls and takes minutes of wall time.

Log slot mapping (`step!` already fills `var_14`/`var_15`/`var_16`):

| slot     | quantity                                  |
|:---------|:------------------------------------------|
| `var_01` | cross-track error d [deg]                 |
| `var_02` | attractor azimuth [deg]                   |
| `var_03` | attractor elevation [deg]                 |
| `var_04` | pattern-centre elevation [deg]            |
| `var_05` | raw guidance course chi_set [rad]         |
| `var_06` | regulated error (feedback - chi_cmd) [deg] |
| `var_07` | entry descent limiter weight (0 = raw guidance, 1 = fully limited) |
| `var_08` | course/heading blend weight (0 = heading, 1 = course) |
| `var_09` | span-mean geometric AoA [deg]             |

`bearing` carries `chi_cmd`, the course the loop actually tracks, so
`course - bearing` is the path-following error; the unmodified guidance course
is kept in `var_05`. The feedback angle the PID regulates is heading at low
kite speed and course at high (`v_kite_heading`/`v_kite_course`, scheduled on
`|vel_kite|`), so `var_06` equals `heading - bearing` at low speed and
`course - bearing` at high. In FIG8 mode that schedule is bypassed and the
course is fed back at any speed (`fig8_pure_course`), so `var_08` is 1
throughout phase 3 and the schedule governs the entry only.

The `sys_state` field carries the ENTRY STATE MACHINE (0 park, 1 dive, 2 hold,
3 fig8), using the same codes as the reference controller's log so both can be
read with the same scripts; `simple_fig8_plots.jl` draws it as the bottom panel
of the time-series figure.

# Parameters

Every tuning parameter of the run is a field of `FC_Settings`
(`src/fc_settings.jl`), loaded from this package's `data/fc_settings.yaml` into
the global `fcs`; each field is documented there. A run with different values
needs no edit of this script — define `fcs` first and it is used as-is:

    fcs = FC_Settings(fc_settings(project_file()))
    fcs.el_center = 30.0
    fcs.attractor_dist = 12.0
    include("examples/simple_fig8.jl")

Shortening a run is not among them: `sim_time` and `sample_freq` are in
`data/settings_fig8_200m.yaml`, so a 30 s run means editing that file.

The dated record of how these parameters were arrived at — sweeps, reverted
attempts and the failures behind each closed lever — is in
`docs/fig8_tuning_log.md`. Add new findings there, not here.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using DiscretePIDs: set_K!
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using LinearAlgebra: norm
using Statistics: mean
using Printf

@info "simple_fig8.jl: figure-of-eight path following of the V3 kite."

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
PROJECT = "system_fig8_200m.yaml" # defined for 150m, 200m and 300m
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
l_tether = Settings(project).l_tether

# ======================== INIT =========================== #

# Decided BEFORE init: the warm-up must relax against the winch the loop commands.
fcs.compliance >= 0 || error("compliance must be >= 0, got $(fcs.compliance)")
# Passed to init in both modes; force mode simply never asks it for a torque.
wc = WC_Settings(wc_settings(project))
wfc = nothing
if fcs.compliance > 0
    # winch_force_gains returns plain numbers; the controller object is V3Kite's.
    wfc = WinchForceController(; winch_force_gains(fcs)...)
    @info @sprintf("Winch: FORCE mode at compliance = %.2f — len_kp %.0f N/m, \
                    damp %.0f N·s/m, tau %.1f s.",
                   fcs.compliance, wfc.len_kp, wfc.damp, wfc.force_tau)
else
    # Perfectly stiff: the position feed-forward cancels the measured load exactly.
    @info "Winch: POSITION mode at compliance = 0 — constant unstretched length."
end

# For the mismatch check in turn_rate_coeffs below; the bare name is what the table records.
set_turn_rate_conditions!(v_wind = fcs.v_wind, l_tether = l_tether,
                          system_yaml = basename(project))
@info "System project: $project"

# No sim_time/dt: init takes them from the project's settings (sim_time, sample_freq).
# cache_path takes everything V3Kite GENERATES; delete it to force a rebuild.
s = init(fcs.v_wind, l_tether; body_damping = fcs.body_damping,
    elevation = fcs.elevation, depower_setpoint = fcs.depower_setpoint,
    system_yaml = project, wc, cache_path = joinpath(@__DIR__, "cache"),
    warmup_time = fcs.warmup_time, warmup_wfc = wfc)
@info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)

# Constant-length setpoint: the tether length after settling and warm-up.
l0 = s.sys_state.l_tether[1]

fec = FigureEightController(FigureEightSettings(;
    dt = s.dt, A = fcs.f8_a, B = fcs.f8_b, C = fcs.f8_c, D = fcs.f8_d,
    az_center = 0.0, el_center = fcs.el_center,
    attractor_distance = fcs.attractor_dist, up_loops = fcs.up_loops))

# Never hardcode these: both arguments move them a lot.
coeffs = turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
c1, c2, delay = coeffs.c1, coeffs.c2, coeffs.delay
@info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
               fcs.body_damping, fcs.depower_setpoint,
               coeffs.interpolated ? " (INTERPOLATED, not identified)" : "",
               c1, c2, delay)

# c1 must match the body damping in use; that is what makes this check meaningful.
feas = check_pattern_feasible(fec, l_tether, fcs.max_steering; c1)
feas.feasible ||
    @warn "Pattern is tighter than the kite's minimum turn radius — expect \
           curvature-limited tracking, not a tuning problem."

# Dead-time context for attractor_dist: how long the lead arc takes to fly.
lead_time = deg2rad(fcs.attractor_dist) * l_tether / fcs.v_app_ref
@info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                vs %.2f s steering dead time (ratio %.1f).",
               fcs.attractor_dist, lead_time, fcs.v_app_ref, delay, lead_time / delay)

# N is the derivative filter's maximum gain; the DiscretePIDs default of 10 rings.
heading_pid = create_heading_pid(;
    K = fcs.heading_p, Ti = fcs.heading_i, Td = fcs.heading_d, N = fcs.heading_d_n,
    dt = s.dt, umin = -fcs.max_steering, umax = fcs.max_steering)

entry_sign = 0              # latched sign of the entry descent limiter (0 = unset)

# 0 = park, 1 = dive, 2 = hold, 3 = figure-eight guidance engaged.
phase = 0
hold_start = NaN            # [s] time the hold began

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

try
    for _ in 1:s.steps
        t = s.sys_state.time

        # L0 attractor guidance -> commanded course [rad].
        chi_set, az_attr, el_attr, dmin =
            navigate_fig8(fec, Float64(s.sys_state.azimuth),
                          Float64(s.sys_state.elevation))

        # Entry state machine: advances on elevation and time, never backwards.
        local el_deg = rad2deg(Float64(s.sys_state.elevation))
        if phase == 0 && t >= fcs.park_time
            global phase = 1
        elseif phase == 1 && el_deg <= fcs.el_center + fcs.dive_el_margin
            global phase = 2
            global hold_start = t
        elseif phase == 2 && t - hold_start >= fcs.hold_time
            global phase = 3
        end

        # Entry descent limiter, active only while the kite is far off the path.
        heading = Float64(s.sys_state.heading)
        chi_cmd = chi_set
        # 1 = fully limited, 0 = raw guidance, linear over entry_d_blend above the gate.
        w_lim = fcs.entry_d_blend > 0 ?
                clamp((dmin - fcs.entry_d_gate) / fcs.entry_d_blend, 0.0, 1.0) :
                (dmin > fcs.entry_d_gate ? 1.0 : 0.0)
        if w_lim > 0 && abs(chi_set) > deg2rad(fcs.entry_chi_max)
            # Only the STEEPNESS is limited; near the ±180° cut the sign is noise.
            tang = path_tangent(fec)
            entry_sign == 0 && (global entry_sign = tang >= 0 ? 1 : -1)
            sgn = abs(chi_set) < pi - deg2rad(fcs.entry_cut_margin) ?
                  (chi_set >= 0 ? 1 : -1) : entry_sign
            chi_lim = sgn * deg2rad(fcs.entry_chi_max)
            # Wrapped difference: a plain convex combination sweeps the long way at ±180°.
            chi_cmd = wrap_to_pi(chi_set + w_lim * wrap_to_pi(chi_lim - chi_set))
        end

        # Open-loop entry: overrides the guidance for the dive and the hold.
        if phase == 1
            chi_cmd = deg2rad(fcs.chi_dive)
        elseif phase == 2
            chi_cmd = deg2rad(fcs.chi_hold)
        end

        # Feedback angle: heading at low kite speed, course at high (see FC_Settings).
        v_kite = norm(s.sys_state.vel_kite)
        w_course = if fcs.fig8_pure_course && phase == 3
            1.0
        else
            clamp((v_kite - fcs.v_kite_heading) /
                  (fcs.v_kite_course - fcs.v_kite_heading), 0.0, 1.0)
        end
        # +π: the raw tangent-frame course has its zero pointing AWAY from zenith.
        course = wrap_to_pi(Float64(s.sys_state.course) + pi)
        fb = heading + w_course * wrap_to_pi(course - heading)

        # DiscretePID does not wrap, so the error is formed here against a zero reference.
        err = wrap_to_pi(fb - chi_cmd)
        # Turn rate ~ u_s * v_app, so K ~ 1/v_app, on APPARENT wind: that is the plant gain.
        v_app = max(Float64(s.sys_state.v_app), fcs.v_app_min)
        K_phase = phase == 3 ? fcs.heading_p : fcs.entry_gain * fcs.heading_p
        set_K!(heading_pid, K_phase * fcs.v_app_ref / v_app, 0.0, err)
        # Park: zero steering, but the PID is still stepped so engagement is bumpless.
        rel_steering = if phase == 0
            heading_pid(0.0, 0.0, 0.0)
            0.0
        else
            heading_pid(0.0, err, 0.0)
        end

        # entry_depower during the dive and hold, depower_setpoint elsewhere.
        rel_depower = (phase == 1 || phase == 2) ? fcs.entry_depower :
                                                   fcs.depower_setpoint

        # Force mode pays out under load; compliance = 0 holds the length outright.
        if isnothing(wfc)
            step!(s; rel_depower, rel_steering, set_length = l0,
                  vsm_interval = fcs.vsm_interval)
        else
            step!(s; rel_depower, rel_steering,
                  set_torque = winch_force_torque!(wfc, s, l0),
                  vsm_interval = fcs.vsm_interval)
        end

        # Report the overspeed rather than the opaque solver abort it causes later.
        if Float64(s.sys_state.v_app) > fcs.v_app_abort
            @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                             Stopping before the solver diverges.",
                            s.sys_state.time, s.sys_state.v_app, fcs.v_app_abort,
                            rad2deg(s.sys_state.elevation), rad2deg(s.sys_state.AoA))
            break
        end

        # After step!, which overwrites parts of sys_state.
        s.sys_state.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 fig8
        s.sys_state.bearing = chi_cmd          # the course actually tracked
        s.sys_state.attractor .= (deg2rad(az_attr), deg2rad(el_attr))
        s.sys_state.var_01 = dmin              # cross-track error [deg]
        s.sys_state.var_02 = az_attr           # attractor azimuth [deg]
        s.sys_state.var_03 = el_attr           # attractor elevation [deg]
        s.sys_state.var_04 = fcs.el_center     # pattern-centre elevation [deg]
        s.sys_state.var_05 = chi_set           # RAW guidance course [rad]
        s.sys_state.var_06 = rad2deg(err)      # REGULATED error [deg]
        # A weight, not a flag: a step here means entry_d_blend is too narrow.
        s.sys_state.var_07 = abs(chi_set) > deg2rad(fcs.entry_chi_max) ? w_lim : 0.0
        s.sys_state.var_08 = w_course          # course/heading blend weight [-]
        # Whole wing; sys_state.AoA is the centre panel only, which a turn twists away from.
        s.sys_state.var_09 = rad2deg(span_mean_aoa(s.sys))
    end
catch exc
    # `exc`, not `e`: a stray global `e` in the REPL makes the catch binding warn.
    @error "Simulation stopped early at t≈$(round(s.sys_state.time, digits=2))s" exception=(exc, catch_backtrace())
end

@info "Save the log"
save_log(s.logger, "fig8_run"; colmeta = timestamp_colmeta())

# ==================== RESULTS ==================== #

syslog = load_log("fig8_run")
sl = syslog.syslog
# The geometry is passed in too: without it the criteria are blind to pattern SIZE.
print_fig8_metrics(sl; t_start = fcs.park_time, settle_time = fcs.entry_time,
                   min_elevation = fcs.min_elevation, az_center = 0.0,
                   az_amplitude = fcs.f8_a, el_height = fcs.f8_b,
                   min_span_frac = fcs.min_span_frac)

# On the LOGGED PHASE, not a time window; the mean is what v_app_ref should be.
let fig8 = findall(x -> Int(x) == 3, sl.sys_state)
    if isempty(fig8)
        @warn "Phase 3 never reached — no figure-eight apparent wind speed."
    else
        va = Float64.(sl.v_app[fig8])
        @printf("  v_app over phase 3 (%.1f s): mean %.2f m/s, range %.2f … %.2f m/s \
                 | v_app_ref = %.1f (%+.1f%%)\n",
                sl.time[fig8[end]] - sl.time[fig8[1]],
                mean(va), minimum(va), maximum(va),
                fcs.v_app_ref, 100 * (mean(va) / fcs.v_app_ref - 1))
    end
end

# SHOW_PLOTS = false in the REPL suppresses them, which is what makes a sweep bearable.
@isdefined(SHOW_PLOTS) || (SHOW_PLOTS = true)
SHOW_PLOTS && include(joinpath(@__DIR__, "simple_fig8_plots.jl"))

nothing
