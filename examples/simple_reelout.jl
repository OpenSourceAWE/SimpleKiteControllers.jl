# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Figure-of-eight path following of the V3 kite, extended with a REEL_OUT winch: the
tether starts at `l_tether` (150 m by default), flies the same four-phase entry as
`simple_fig8.jl` (park -> dive -> hold -> fig8), and once the pattern is
first tracked closely (phase 4) reels out under WinchControllers.jl's
`v_set = kv * sqrt(force)` law until `fcs.reelout_l_max`, then holds that length
for the rest of the run. No pumping cycle: there is no reel-in phase here.

# What comes from where

The guidance and its settings are unchanged from `simple_fig8.jl` — see that
script's docstring for the guidance, the entry state machine and the plant/data
path conventions, all of which apply here too. What differs is the winch: instead
of V3Kite's own FORCE/POSITION winch (`fcs.compliance`), this script layers
WinchControllers.jl's `WinchController` on top of V3Kite's POSITION mode. Each
step it turns the measured `reel_out_speed(s)`/`winch_force(s)` into a speed
`v_set`, integrates that into a length setpoint `l_set`, and passes `l_set` to
`step!`'s `set_length` — the same mechanism the constant-length run uses, just
with a setpoint that grows. `step!` accepts a length or a torque, never a speed
directly, which is why the integration happens here rather than inside V3Kite.
`v_set` itself is ALSO passed, as `step!`'s `v_ff`: V3Kite's position winch is a
P loop on the length error, so integrating a speed here and differentiating it
back out there is a first-order lag of `1/winch_pos_kp` = 2 s — measured at 1.16 s
of delay and 0.49 of the commanded amplitude on the 5.7 s reel-out oscillation
before `v_ff` existed, plus a standing `v_ro/winch_pos_kp` ≈ 5 m length error.
Feeding the speed forward leaves the P loop only the error to correct.
`fcs.compliance` must be `0` (POSITION mode): REEL_OUT and V3Kite's own FORCE mode
both drive `set_length`/`set_torque`, and only one winch can hold the drum at a
time — this script errors at startup otherwise, rather than silently picking
one. That is why `fcs` here is loaded from `data/fc_settings_reelout.yaml`, a
copy of `simple_fig8.jl`'s `fc_settings.yaml` with `compliance: 0` and the
REEL_OUT keys below, not the shared file (whose `compliance: 0.5` is fig8's
FORCE-mode tuning) — `system_reelout_150m.yaml`'s `fc_settings:` key names it.

# Feasibility

`check_pattern_feasible` is printed at both `l_tether` (the START of the run,
before any pay-out — the worst case, since a longer tether only ever shrinks the
kite's minimum angular turn radius) and `fcs.reelout_l_max` (the end). Measured
with `fc_settings_reelout.yaml`'s defaults at 150 m the margin is 1.22, growing
to 1.63 at 200 m and 2.04 at 250 m, so no pattern change is needed to start at
150 m — see `Plan.md`.

Logs the run to `output/<log_file>.arrow` and `include`s `simple_reelout_plots.jl`
at the end, exactly like `simple_fig8.jl`.

Log slot mapping (`step!` already fills `var_14`/`var_15`/`var_16`; `var_01`
through `var_09` are as in `simple_fig8.jl`):

| slot     | quantity                                          |
|:---------|:---------------------------------------------------|
| `var_10` | tether length setpoint `l_set` [m]                |
| `var_11` | REEL_OUT speed setpoint `v_set` [m/s]             |
| `var_12` | WinchController state (0 lower-force, 1 speed, 2 upper-force) |
| `var_13` | force error of the active force limiter [N], NaN in speed control |

`sys_state` carries the same entry state machine as `simple_fig8.jl` (0 park,
1 dive, 2 hold, 3 fig8, 4 settled); reel-out begins the first time phase 4 is
reached and stops once `l_set` reaches `fcs.reelout_l_max`.

# Parameters

`fcs` works exactly as in `simple_fig8.jl` — define it before the `include` to
override a value, e.g. `fcs.reelout_l_max = 300.0`. The REEL_OUT-specific fields
are `reelout_kv`, `reelout_f_low`, `reelout_f_high`, `reelout_v_max` (all feed
WinchControllers.jl's `WCSettings`) and `reelout_l_max`, the stop length.

Which system project is flown, the run length and the turbulence level are read
from `data/gui.yaml` exactly as in `simple_fig8.jl` — run `select_project()` to
choose `system_reelout_150m.yaml` (or any other `system*.yaml`; a shorter tether
means less margin to cover, see Feasibility above).
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using WinchControllers: WCSettings, WinchController, calc_v_set, on_timer,
    get_state, get_f_err
using DiscretePIDs: set_K!
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: calc_wind_factor
using LinearAlgebra: norm
using Statistics: mean
using Printf

@info "simple_reelout.jl: figure-of-eight path following with REEL_OUT of the tether."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
# Read and cleared HERE, so a `SHOW_PLOTS = false` never survives into the next run.
show_plots = @isdefined(SHOW_PLOTS) ? SHOW_PLOTS : true
SHOW_PLOTS = true
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Structural damping of the tether and bridle segments, as a ratio of their
# stiffness: unit_damping = ratio * unit_stiffness [s]. See simple_fig8.jl's docstring.
DAMPING_PER_STIFFNESS = 0.001
PROJECT = selected_project() # e.g. system_reelout_150m.yaml, set via select_project()
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
@info "simple_reelout.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))

project_set = Settings(project)
l_tether = project_set.l_tether

# Log files are arrow files, named after the project's `log_file`, kept out of git.
output_path = normpath(joinpath(@__DIR__, "..", "output"))
mkpath(output_path)
log_name = basename(project_set.log_file)

# ======================== INIT =========================== #

fcs.compliance >= 0 ||
    error("compliance must be >= 0, got $(fcs.compliance)")
fcs.compliance == 0 ||
    error("REEL_OUT needs compliance = 0 (POSITION mode) — REEL_OUT and V3Kite's own \
           FORCE mode both drive the winch and only one can hold the drum at a time.")
wc = WC_Settings(wc_settings(project))   # POSITION-mode gains; step! still takes set_length
wfc = nothing                            # no FORCE-mode winch; warm-up settles at constant length
@info @sprintf("Winch: REEL_OUT mode — v_set = %.3f * sqrt(force), stopping at %.0f m.",
               fcs.reelout_kv, fcs.reelout_l_max)

# No dt: init takes it from the project's settings (sample_freq). sim_time falls
# back to the project's own value when SIM_TIME is `nothing` (the `default` choice).
# The wind speed comes from the same file: it is a plant condition, and passing the
# project's own value keeps the mean wind and the turbulent field (which init builds
# for `set.v_wind`) at the same speed.
# No cache_path either: V3Kite's default is where its own precompile workload
# compiled the model, and a different model binary costs 40 s of re-JIT in init.
s = init(project_set.v_wind, l_tether; body_damping = fcs.body_damping,
    damping_per_stiffness = DAMPING_PER_STIFFNESS,
    elevation = fcs.elevation, depower_setpoint = fcs.depower_setpoint,
    system_yaml = project, wc, use_turbulence = TURBULENCE, aero_mode = AERO_MODE,
    sim_time = SIM_TIME, warmup_time = fcs.warmup_time, warmup_wfc = wfc)
@info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)

# REEL_OUT controller: built fresh here so its soft-start ramp (t_startup) begins
# the moment reel-out actually starts (phase 4), not at t = 0. `update(rcs)` is
# NOT called: it would read this package's wc_settings.yaml, which is in V3Kite's
# WC_Settings schema, not WinchControllers'. The tuning comes from fcs instead.
rcs = WCSettings(dt = s.dt)
rcs.mode = "reelout"
rcs.kv = fcs.reelout_kv
rcs.f_low = fcs.reelout_f_low
rcs.f_high = fcs.reelout_f_high
rcs.v_sat = fcs.reelout_v_max
rc = WinchController(rcs)

# Length setpoint: starts at the tether length after settling and warm-up, and
# grows from the first step of phase 4 onward until it reaches `reelout_l_max`.
l_set = s.sys_state.l_tether[1]

fec = FigureEightController(FigureEightSettings(;
    dt = s.dt, A = fcs.f8_a, B = fcs.f8_b, C = fcs.f8_c, D = fcs.f8_d,
    az_center = 0.0, el_center = fcs.el_center,
    attractor_distance = fcs.attractor_dist, up_loops = fcs.up_loops))

# Never hardcode these: both arguments move them a lot. The lookup key is
# `body_damping`, the value `init` was given: the damping the model FLIES the
# pattern with is the floor that decays out of it (0.8x by default), so the one
# value identifies both the settling transient and the flown damping c1 belongs
# to.
#
# The coefficients are DIAGNOSTIC here — they feed the feasibility check and the
# dead-time context below, no gain and no control law — so a damping/depower the
# table cannot serve costs the diagnosis, not the run. `turn_rate_coeffs` refuses
# to extrapolate (by design: c1 moves violently with both arguments), so catch
# that and fly on unadvised rather than aborting a deliberate off-grid run.
coeffs = try
    turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
catch e
    e isa ArgumentError || rethrow()
    @warn "No turn-rate coefficients for body_damping = $(fcs.body_damping), \
           depower = $(fcs.depower_setpoint) — flying WITHOUT the feasibility \
           check. Identify this cell with V3Kite.jl's steering_test_v3.jl to get \
           it back.\n$(e.msg)"
    nothing
end

if isnothing(coeffs)
    c1 = c2 = delay = NaN
else
    c1, c2, delay = coeffs.c1, coeffs.c2, coeffs.delay
    @info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                    c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
                   fcs.body_damping, fcs.depower_setpoint,
                   coeffs.interpolated ? " (INTERPOLATED)" : "",
                   c1, c2, delay)

    # c1 must match the damping in use; that is what makes this check meaningful.
    # At l_tether (the START, before any pay-out) this is the WORST case: a longer
    # tether only ever shrinks the kite's minimum angular turn radius.
    feas_start = check_pattern_feasible(fec, l_tether, fcs.max_steering; c1)
    feas_start.feasible ||
        @warn "Pattern is tighter than the kite's minimum turn radius AT THE START \
               (l_tether = $l_tether m) — expect curvature-limited tracking during \
               the entry, not a tuning problem."
    feas_end = check_pattern_feasible(fec, fcs.reelout_l_max, fcs.max_steering; c1)

    # Dead-time context for attractor_dist: how long the lead arc takes to fly.
    lead_time = deg2rad(fcs.attractor_dist) * l_tether / fcs.v_app_ref
    @info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   fcs.attractor_dist, lead_time, fcs.v_app_ref, delay, lead_time / delay)
end

# N is the derivative filter's maximum gain; the DiscretePIDs default of 10 rings.
heading_pid = create_heading_pid(;
    K = fcs.heading_p, Ti = fcs.heading_i, Td = fcs.heading_d, N = fcs.heading_d_n,
    dt = s.dt, umin = -fcs.max_steering, umax = fcs.max_steering)

entry_sign = 0              # latched sign of the entry descent limiter (0 = unset)

# 0 = park, 1 = dive, 2 = hold, 3 = figure-eight guidance engaged, 4 = settled
# (phase 3 with cross-track error below settled_d_gate for the first time).
phase = 0
hold_start = NaN            # [s] time the hold began

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

# Assigned OUTSIDE the try: the loop's wall time must survive an early break.
t_wall_start = time()
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
        elseif phase == 3 && dmin < fcs.settled_d_gate
            global phase = 4
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
        local v_kite = norm(s.sys_state.vel_kite)
        w_course = if fcs.fig8_pure_course && phase >= 3
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
        K_phase = phase >= 3 ? fcs.heading_p : fcs.entry_gain * fcs.heading_p
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

        # REEL_OUT: only while phase 4 has been reached and l_set has not yet hit
        # reelout_l_max. Once it does, l_set simply stops growing and the rest of
        # the run is flown exactly like the constant-length example.
        local v_set = 0.0
        if phase == 4 && l_set < fcs.reelout_l_max
            v_set = calc_v_set(rc, reel_out_speed(s), winch_force(s), rcs.f_low)
            global l_set = min(l_set + v_set * s.dt, fcs.reelout_l_max)
            on_timer(rc)
        end

        # `v_ff = v_set`: the winch's outer P loop is told the speed being
        # commanded instead of having to rediscover it from a length error. See
        # the docstring — without it the pair (integrate here, differentiate
        # there) is a 1/winch_pos_kp = 2 s lag. Zero outside the pay-out window,
        # where `l_set` is constant and there is nothing to feed forward.
        step!(s; rel_depower, rel_steering, set_length = l_set, v_ff = v_set,
              speed_limit = fcs.reelout_v_max, acceleration_limit = rcs.max_acc,
              vsm_interval = fcs.vsm_interval)

        # Report the overspeed rather than the opaque solver abort it causes later.
        if Float64(s.sys_state.v_app) > fcs.v_app_abort
            @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                             Stopping before the solver diverges.",
                            s.sys_state.time, s.sys_state.v_app, fcs.v_app_abort,
                            rad2deg(s.sys_state.elevation), rad2deg(s.sys_state.AoA))
            break
        end

        # After step!, which overwrites parts of sys_state.
        s.sys_state.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 fig8, 4 settled
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
        s.sys_state.var_10 = l_set             # tether length setpoint [m]
        s.sys_state.var_11 = v_set             # REEL_OUT speed setpoint [m/s]
        s.sys_state.var_12 = get_state(rc)     # WinchController state (0/1/2)
        s.sys_state.var_13 = get_f_err(rc)     # force error [N], NaN in speed control
        # Not filled anywhere in the model chain: without this the log and the viewer read 0.
        s.sys_state.v_wind_200m .= calc_wind_factor(s.am, 200.0) .* s.sys_state.v_wind_gnd
    end
catch exc
    # `exc`, not `e`: a stray global `e` in the REPL makes the catch binding warn.
    @error "Simulation stopped early at t≈$(round(s.sys_state.time, digits=2))s" exception=(exc, catch_backtrace())
end
# The loop ALONE: saving the log and scoring it below are not simulation.
t_wall = time() - t_wall_start
t_sim = Float64(s.sys_state.time)

@info "Save the log"
save_log(s.logger, log_name; path = output_path, colmeta = timestamp_colmeta())

# ==================== RESULTS ==================== #

syslog = load_log(log_name; path = output_path)
sl = syslog.syslog
# The geometry is passed in too: without it the criteria are blind to pattern SIZE.
print_fig8_metrics(sl; t_start = fcs.park_time, settle_time = fcs.entry_time,
                   min_elevation = fcs.min_elevation, az_center = 0.0,
                   az_amplitude = fcs.f8_a, el_height = fcs.f8_b,
                   min_span_frac = fcs.min_span_frac)

# On the LOGGED PHASE, not a time window; the mean is what v_app_ref should be.
let settled = findall(x -> Int(x) == 4, sl.sys_state)
    if isempty(settled)
        @warn "Phase 4 never reached — no settled apparent wind speed, no reel-out."
    else
        va = Float64.(sl.v_app[settled])
        @printf("  v_app over phase 4 (%.1f s): mean %.2f m/s, range %.2f … %.2f m/s \
                 | v_app_ref = %.1f (%+.1f%%)\n",
                sl.time[settled[end]] - sl.time[settled[1]],
                mean(va), minimum(va), maximum(va),
                fcs.v_app_ref, 100 * (mean(va) / fcs.v_app_ref - 1))
        @printf("  Tether: %.1f m -> %.1f m (target %.1f m).\n",
                l_tether, sl.var_10[settled[end]], fcs.reelout_l_max)
    end
end

let rp = reelout_power(sl)
    if isnothing(rp)
        @warn "Tether never reeled out — no reel-out power to report."
    else
        @printf("  Reel-out power: mean %.0f W over %d samples.\n", rp.mean_power, rp.n)
    end
end

# Speed of the SIMULATED time against the wall clock; > 1 is faster than realtime.
if t_sim > 0
    @printf("  Performance: %.1f s sim in %.1f s wall = %.2f x realtime \
             (%.1f ms/step over %d steps at dt = %.4f s, vsm_interval = %d)\n",
            t_sim, t_wall, t_sim / t_wall, 1000 * t_wall / round(Int, t_sim / s.dt),
            round(Int, t_sim / s.dt), s.dt, fcs.vsm_interval)
else
    @warn "No simulated time elapsed — no performance figure."
end

if show_plots
    include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
else
    @info "Plots suppressed by SHOW_PLOTS = false; it is back to true for the next run."
end

nothing
