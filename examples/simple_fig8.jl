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
package (`src/`). The kite model, the simulation loop and the winch position
controller are V3Kite, used exactly as released on its main branch. The three
things this run needs that released V3Kite does not provide — the discarded
warm-up, the force-mode winch and the span-mean AoA — are in
`v3kite_support.jl` next to this file, written against V3Kite's public API only.

# Why the pattern is large

Neither is a free choice. The V3's identified turn-rate law fixes the smallest
angular turn radius the kite can fly:

    rho = 1 / (L * c1 * u_s)        [rad]   -- apparent wind speed cancels

`c1` depends strongly on the `body_damping` passed to `init` (see
`V3_TURN_RATE_COEFFS`): across the identified range it changes the achievable
turn radius by 5.6x, from 6.9° at `[0, 0, 40]` to 38.5° at `[20, 20, 40]`. 

The tightest curvature of a lemniscate in (azimuth, elevation) is not at the
lobe tip but on the lobe's upper shoulder, and it collapses as the pattern is
raised, because the azimuth axis is compressed by `cos(elevation)`. At
`c1 = 0.3159` and `u_s = 0.175`:

    A=45 B=20, centre 73° -> tightest path radius  0.4°   (margin 0.06)
    A=45 B=20, centre 60° ->                       3.1°   (margin 0.45)
    A=45 B=20, centre 35° ->                       8.5°   (margin 1.23)
    A=50 B=25, centre 30° ->                      10.5°   (margin 1.52)

So a figure-eight near zenith is geometrically impossible for this kite at any
PID tuning: the pattern must be flown low and wide, and the kite descends onto
it. `check_pattern_feasible` prints the margin at startup. Below ~1 the tracking
error is curvature-limited and no PID tuning will fix it — enlarge the pattern,
lower its centre, lower the damping, or raise `max_steering`.

Logs the run to "fig8_run" (a dedicated name, not the shared "tmp_run"). For
verification run `include("examples/simple_fig8.jl")` — even a 30 s simulation
is several thousand `step!` calls and takes minutes of wall time. The script
`include`s `simple_fig8_plots.jl` itself at the end, so the pattern and
time-series figures come up without a second call.

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

    fcs = FC_Settings("fc_settings.yaml")
    fcs.sim_time = 30.0
    fcs.el_center = 30.0
    include("examples/simple_fig8.jl")

The dated record of how these parameters were arrived at — sweeps, reverted
attempts and the failures behind each closed lever — is in V3Kite.jl's
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

# `warmup!`, the force-mode winch and `span_mean_aoa`: the parts of this run
# that reach into a `V3KITE` and therefore cannot live in the (model-agnostic)
# package. See the file's own docstring.
include(joinpath(@__DIR__, "v3kite_support.jl"))

@info "simple_fig8.jl: figure-of-eight path following of the V3 kite."

# ==================== USER PARAMETERS ==================== #

# Every tuning parameter of the run lives in this package's
# `data/fc_settings.yaml` and is documented field by field in
# `src/fc_settings.jl` (`FC_Settings`). `set_data_path` points KiteUtils at
# V3Kite's data directory — the model's geometry, settings and settled-state
# cache — which is a different directory from the controller's own; `FC_Settings`
# reads `skc_data_path()` regardless of it.
set_data_path(v3_data_path())
fcs = FC_Settings("fc_settings.yaml")

fcs.vsm_interval == 1 || error("vsm_interval = $(fcs.vsm_interval): V3Kite's \
    released `step!` hardwires 1 and takes no `vsm_interval` keyword, so this \
    script cannot honour any other value. See `FC_Settings`.")

# ======================== INIT =========================== #

# Winch mode (see `fcs.compliance`). Decided BEFORE `init`, because the warm-up
# below has to relax against the same winch the loop commands — warming up
# against the other one hands the run the discontinuity the warm-up exists to
# remove.
fcs.compliance >= 0 || error("compliance must be >= 0, got $(fcs.compliance)")
# Position-mode gains, from V3Kite's own wc-settings file (the one named in the
# project YAML). Passed to `init` in both modes: it builds the winch position
# controller either way, and force mode simply never asks it for a torque.
wc = WC_Settings(wc_settings(fcs.project))
wfc = nothing
if fcs.compliance > 0
    # Compliance is 1/winch_len_kp; winch_damp goes with it so the length loop's
    # time constant (damp/len_kp) does not move.
    wfc = WinchForceController(fcs)
    @info @sprintf("Winch: FORCE mode at compliance = %.2f — len_kp %.0f N/m, \
                    damp %.0f N·s/m, tau %.1f s.",
                   fcs.compliance, wfc.len_kp, wfc.damp, wfc.force_tau)
else
    # Perfectly stiff: V3Kite's `winch_position_torque!` feed-forward cancels the
    # measured load exactly, so the drum has nothing to accelerate it whatever
    # the PI gains are.
    @info "Winch: POSITION mode at compliance = 0 — constant unstretched length."
end

# Conditions of this run, for the mismatch check in `turn_rate_coeffs` below:
# the table's coefficients were identified at the conditions in its own
# `conditions:` block and do not necessarily carry over to others.
set_turn_rate_conditions!(v_wind = fcs.v_wind, l_tether = fcs.tether_length,
                          system_yaml = fcs.project)

s = init(fcs.v_wind, fcs.tether_length; body_damping = fcs.body_damping,
    elevation = fcs.elevation,
    depower_setpoint = fcs.depower_setpoint, sim_time = fcs.sim_time, dt = fcs.dt,
    system_yaml = fcs.project, wc)

# Hand the drum a holding torque for the load it is already carrying. `init`
# releases the brake, but the settled state was found with the length
# KINEMATICALLY FIXED, so the serialized `set_value` is whatever settling left
# there — without this the first consistent-initial-condition solve sees a free
# drum holding nothing, which is a torque step at t = 0 even though the winch
# controller starts from the measured force one step later.
init_winch_torque!(s.sys)

# Discarded warm-up: relax the settled geometry into an equilibrium of the model
# actually being integrated, so that transient is not in the log at all.
warmup!(s, fcs.warmup_time; depower = fcs.depower_setpoint, wfc)

# Constant-length setpoint: the tether length after settling and warm-up.
l0 = s.sys_state.l_tether[1]

fec = FigureEightController(FigureEightSettings(;
    dt = s.dt, A = fcs.f8_a, B = fcs.f8_b, C = fcs.f8_c, D = fcs.f8_d,
    az_center = 0.0, el_center = fcs.el_center,
    attractor_distance = fcs.attractor_dist, up_loops = fcs.up_loops))

# Turn-rate law of the plant actually being flown. `turn_rate_coeffs` is the
# single source for all three coefficients of
#
#     psi_dot = c1 * v_app * u_s + c2 / v_app * sin(psi) * cos(beta)
#
# at this `body_damping` and `depower_setpoint` — never hardcode them, both
# arguments move them a lot (see the function's docstring). They are printed
# every run because every setting in `fcs` is argued against them:
#   c1     -> steering authority, hence the curvature feasibility margin
#   c2     -> the gravity/turn term the heading loop has to fight
#   delay  -> steering dead time, the limit on how fast the commanded course
#             may rotate (the lever behind `attractor_dist` and `heading_d`)
# turn_rate_coeffs interpolates for a `depower_setpoint` between identified grid
# points; a run using interpolated values says so rather than
# reporting the margin as if it came from an identified one.
coeffs = turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
c1, c2, delay = coeffs.c1, coeffs.c2, coeffs.delay
@info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
               fcs.body_damping, fcs.depower_setpoint,
               coeffs.interpolated ? " (INTERPOLATED, not identified)" : "",
               c1, c2, delay)

# Curvature feasibility: a pattern tighter than the kite's minimum turn radius
# cannot be tracked at any PID tuning (see the docstring). c1 must match the
# body damping actually in use — that is what makes this check meaningful.
feas = check_pattern_feasible(fec, fcs.tether_length, fcs.max_steering; c1)
feas.feasible ||
    @warn "Pattern is tighter than the kite's minimum turn radius — expect \
           curvature-limited tracking, not a tuning problem."

# Dead-time context for `attractor_dist`. The attractor sits that much arc ahead
# of the kite, so the commanded course turns over roughly the time the kite
# needs to cover that arc; `delay` is how long the plant takes to react at all.
# `v_app_ref` is the crosswind speed actually flown, which is what this
# kinematic estimate needs.
lead_time = deg2rad(fcs.attractor_dist) * fcs.tether_length / fcs.v_app_ref
@info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                vs %.2f s steering dead time (ratio %.1f).",
               fcs.attractor_dist, lead_time, fcs.v_app_ref, delay, lead_time / delay)

heading_pid = create_fig8_pid(;
    K = fcs.heading_p, Ti = fcs.heading_i, Td = fcs.heading_d, N = fcs.heading_d_n,
    dt = s.dt, umin = -fcs.max_steering, umax = fcs.max_steering)

entry_sign = 0              # latched sign of the entry descent limiter (0 = unset)

# Entry state machine (see `FC_Settings`). Codes match the reference
# controller's log so both can be read with the same plotting scripts:
#   0 = park, 1 = dive, 2 = hold, 3 = figure-eight guidance engaged.
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

        # ---- Entry state machine (park -> dive -> hold -> fig8) ------------ #
        # Advances on elevation and time, never backwards. The guidance above
        # keeps running through all phases so `dmin`/the attractor stay logged,
        # but in phases 1-2 its course is discarded in favour of the open-loop
        # entry command below — the point of the phases is that the descent is
        # NOT flown by the path controller.
        local el_deg = rad2deg(Float64(s.sys_state.elevation))
        if phase == 0 && t >= fcs.park_time
            global phase = 1
        elseif phase == 1 && el_deg <= fcs.el_center + fcs.dive_el_margin
            global phase = 2
            global hold_start = t
        elseif phase == 2 && t - hold_start >= fcs.hold_time
            global phase = 3
        end

        # Entry descent limiter (see `FC_Settings`). Active only while the kite
        # is far off the path; on the path the raw guidance course passes
        # through, because the pattern needs steep courses of its own.
        heading = Float64(s.sys_state.heading)
        chi_cmd = chi_set
        # Limiter weight: 1 = fully limited, 0 = raw guidance, linear in between
        # over `entry_d_blend` above the gate. With `entry_d_blend` = 0 this is
        # the old hard switch (the `>` keeps d == gate on the raw side either way).
        w_lim = fcs.entry_d_blend > 0 ?
                clamp((dmin - fcs.entry_d_gate) / fcs.entry_d_blend, 0.0, 1.0) :
                (dmin > fcs.entry_d_gate ? 1.0 : 0.0)
        if w_lim > 0 && abs(chi_set) > deg2rad(fcs.entry_chi_max)
            # chi_set is the HOMING law (great-circle course to the attractor);
            # only its steepness is limited, never its homing intent. An earlier
            # version commanded the path tangent instead — pure feed-forward
            # with no reference to the path's POSITION — and the kite flew off
            # to azimuth -65° and sat in a limit cycle for 180 s with the
            # cross-track error frozen at 24°. A course limiter must clamp the
            # homing course, not replace it.
            #
            # Sign: chi_set's own, except near the ±180° branch cut, where the
            # kite sits almost directly above the pattern, every attractor point
            # is "straight down", and the sign is numerical noise. There the
            # latched tangent-at-Q sign is used instead — it is ~±108° at the
            # park, nowhere near the cut, and encodes which way round the path
            # is traversed, so the kite arrives moving in the right direction.
            # Latching arbitrarily and then reversing mid-descent is what broke
            # the two earlier runs.
            tang = path_tangent(fec)
            entry_sign == 0 && (global entry_sign = tang >= 0 ? 1 : -1)
            sgn = abs(chi_set) < pi - deg2rad(fcs.entry_cut_margin) ?
                  (chi_set >= 0 ? 1 : -1) : entry_sign
            chi_lim = sgn * deg2rad(fcs.entry_chi_max)
            # Blend on the WRAPPED difference, exactly as the heading/course
            # feedback blend below: chi_lim and chi_set can straddle the ±180°
            # cut (the limiter exists partly because chi_set hunts across it),
            # and a plain convex combination would then sweep the command the
            # long way round through zero. This form is continuous there and
            # still returns chi_lim exactly at w_lim = 1 and chi_set exactly
            # at w_lim = 0, so the blend adds no offset in either limit.
            chi_cmd = wrap_to_pi(chi_set + w_lim * wrap_to_pi(chi_lim - chi_set))
        end

        # Open-loop entry command. Overrides the guidance (and its limiter) for
        # the dive and the hold; positive = towards positive azimuth, matching
        # the reference controller, which enters the pattern from the right.
        if phase == 1
            chi_cmd = deg2rad(fcs.chi_dive)
        elseif phase == 2
            chi_cmd = deg2rad(fcs.chi_hold)
        end

        # Feedback angle: heading at low kite speed, course at high (see
        # `FC_Settings`). Blended on the WRAPPED difference so the transition
        # stays continuous across the ±180° cut, and so the two endpoints are
        # exactly `heading` and `course`.
        #
        # In FIG8 mode the speed schedule is bypassed and the course is fed back
        # unconditionally (`fig8_pure_course`). Path following is a course problem:
        # what must lie on the path is where the kite GOES, and the ~13-15° drift
        # angle means heading feedback tracks the path with a standing offset. On
        # the pattern the kite is also fast enough that the schedule asks for
        # course anyway — it just dips into the band during the slow part of a
        # turn and swaps the feedback signal mid-manoeuver, which is the failure
        # mode recorded in V3Kite.jl's SmallPlan.md (|v_kite| crossed the 10 m/s edge twice
        # in the 15-27 s window of the 2026-08-02 run, 9.8% of it inside the
        # band). The entry phases keep the schedule: during park and dive the
        # kite really can be too slow for a meaningful course.
        v_kite = norm(s.sys_state.vel_kite)
        w_course = if fcs.fig8_pure_course && phase == 3
            1.0
        else
            clamp((v_kite - fcs.v_kite_heading) /
                  (fcs.v_kite_course - fcs.v_kite_heading), 0.0, 1.0)
        end
        # +π: `SysState.course` is SymbolicAWEModels' raw tangent-frame course,
        # whose zero points AWAY from zenith, while `SysState.heading` and the
        # guidance both use 0 = towards zenith. The flip is not symmetric
        # between the two fields — the V3's body x-axis is reversed w.r.t. the
        # sensor convention, which cancels the frame flip for the heading but
        # cannot for a velocity direction. MEASURED on this model over the
        # samples where the kite flies (>2°/s): with the correction,
        # course - heading is the +13..15° drift angle documented in
        # src/figure_eight_controller.jl; without it, -165°. Feeding the raw field back
        # is positive feedback and diverged the run at t = 19.7 s.
        course = wrap_to_pi(Float64(s.sys_state.course) + pi)
        fb = heading + w_course * wrap_to_pi(course - heading)

        # Heading PID on the WRAPPED course error. DiscretePID computes a plain
        # (r - y) difference with no ±π wrapping, so the error is formed here
        # and passed as the measurement against a zero reference; otherwise the
        # loop commands a full turn the long way round at every wrap crossing.
        err = wrap_to_pi(fb - chi_cmd)
        # Gain scheduling: turn rate ~ u_s * v_app, so K ~ 1/v_app. Still on
        # APPARENT wind speed — that is the plant gain; only the choice of
        # feedback angle above schedules on kite speed.
        # The entry phases run at `entry_gain * heading_p`; the pattern itself
        # (phase 3) at the full gain.
        v_app = max(Float64(s.sys_state.v_app), fcs.v_app_min)
        K_phase = phase == 3 ? fcs.heading_p : fcs.entry_gain * fcs.heading_p
        set_K!(heading_pid, K_phase * fcs.v_app_ref / v_app, 0.0, err)
        # Park: hold zero steering while the settling transients decay. The PID
        # is still stepped (with zero error) so its derivative state is current
        # and engagement is bumpless.
        rel_steering = if phase == 0
            heading_pid(0.0, 0.0, 0.0)
            0.0
        else
            heading_pid(0.0, err, 0.0)
        end

        # Depower: `entry_depower` during the dive and the hold,
        # `depower_setpoint` during the park and on the pattern.
        rel_depower = (phase == 1 || phase == 2) ? fcs.entry_depower :
                                                   fcs.depower_setpoint

        # Winch: force mode pays out under load and trims the mean length back
        # slowly, at the stiffness `compliance` scaled the gains to;
        # `compliance` = 0 holds the length outright (see `FC_Settings`).
        if isnothing(wfc)
            step!(s; rel_depower, rel_steering, set_length = l0)
        else
            step!(s; rel_depower, rel_steering,
                  set_torque = winch_force_torque!(wfc, s, l0))
        end

        # Overspeed guard: report the cause instead of letting it surface as an
        # opaque solver dt_epsilon abort a few steps later.
        if Float64(s.sys_state.v_app) > fcs.v_app_abort
            @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                             Stopping before the solver diverges.",
                            s.sys_state.time, s.sys_state.v_app, fcs.v_app_abort,
                            rad2deg(s.sys_state.elevation), rad2deg(s.sys_state.AoA))
            break
        end

        # Logged after step! (which overwrites parts of sys_state). bearing is
        # the commanded course, so course - bearing is the path-following error
        # and heading - bearing is what the loop sees while w_course = 0.
        s.sys_state.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 fig8
        s.sys_state.bearing = chi_cmd          # the course actually tracked
        s.sys_state.attractor .= (deg2rad(az_attr), deg2rad(el_attr))
        s.sys_state.var_01 = dmin              # cross-track error [deg]
        s.sys_state.var_02 = az_attr           # attractor azimuth [deg]
        s.sys_state.var_03 = el_attr           # attractor elevation [deg]
        s.sys_state.var_04 = fcs.el_center     # pattern-centre elevation [deg]
        s.sys_state.var_05 = chi_set           # RAW guidance course [rad]
        s.sys_state.var_06 = rad2deg(err)      # REGULATED error [deg]
        # Entry limiter weight, not a flag: 0 = raw guidance course, 1 = fully
        # limited, fractional inside the `entry_d_blend` band. Logged as the
        # weight so the handover is visible as the ramp it now is — a plot that
        # still shows a step here means `entry_d_blend` is too narrow for the
        # rate d is closing at.
        s.sys_state.var_07 = abs(chi_set) > deg2rad(fcs.entry_chi_max) ? w_lim : 0.0
        s.sys_state.var_08 = w_course          # course/heading blend weight [-]
        # Whole-wing AoA. `sys_state.AoA` is the CENTRE PANEL only, which is
        # representative while the wing is loaded symmetrically but not in a
        # turn, where steering twists the two halves in opposite directions —
        # and this pattern is flown with the steering on its clamp nearly all
        # the time. The plots show both so the gap between them is visible.
        s.sys_state.var_09 = rad2deg(span_mean_aoa(s.sys))
    end
catch exc
    # `exc`, not `e`: a stray global `e` in the REPL turns the catch binding
    # into a soft-scope local and emits a confusing warning on every run.
    @error "Simulation stopped early at t≈$(round(s.sys_state.time, digits=2))s" exception=(exc, catch_backtrace())
end

@info "Save the log"
save_log(s.logger, "fig8_run"; colmeta = timestamp_colmeta())

# ==================== RESULTS ==================== #

syslog = load_log("fig8_run")
sl = syslog.syslog
# The pattern geometry is passed in, not just the log: without it the tracking
# criteria are blind to SIZE. Cross-track error is measured to the closest point
# of the path, so a kite flying a small eight — or one lobe's worth of it in half
# the wind window — is close to the path at every instant and scores a low RMS d.
# `az_amplitude`/`el_height` add the reach criteria that catch that (see
# `min_span_frac`).
print_fig8_metrics(sl; t_start = fcs.park_time, settle_time = fcs.entry_time,
                   min_elevation = fcs.min_elevation, az_center = 0.0,
                   az_amplitude = fcs.f8_a, el_height = fcs.f8_b,
                   min_span_frac = fcs.min_span_frac)

# Apparent wind speed over the pattern. Selected on the LOGGED PHASE (== 3),
# not on a time window: phase 3 begins when the entry state machine hands over,
# which depends on how the dive went, so a fixed window would mix entry samples
# into the average on a slow entry and drop pattern samples on a fast one.
# The mean is the value `v_app_ref` should be set to — it anchors the 1/v_app
# gain schedule, and only when it matches the speed actually flown does
# `heading_p` read as the gain the kite really flies at (see `FC_Settings`). The
# applied gain is `heading_p * v_app_ref / v_app` either way, so a mismatch
# misreports the tuning rather than changing this run.
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

# Plots come up with the run — the plotting script reads the pattern geometry
# from `fcs`, so the reference overlay always matches the pattern flown.
# `SHOW_PLOTS = false` in the REPL suppresses them, which is what makes a sweep
# bearable: three GLMakie windows per run adds up fast.
@isdefined(SHOW_PLOTS) || (SHOW_PLOTS = true)
SHOW_PLOTS && include(joinpath(@__DIR__, "simple_fig8_plots.jl"))

nothing
