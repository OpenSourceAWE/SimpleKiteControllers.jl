# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Reel out along a path an EXTERNAL OPTIMIZER produced, instead of along a
lemniscate.

`simple_reelout.jl` with one block added — read that file's docstring for the
winch, the entry state machine, the log slots and the two stop criteria, all of
which apply here unchanged. What differs is the reference path: the AWETrim
optimizer is asked for the power-optimal reel-out path under THIS run's wind and
winch, and `set_path!` installs it. This is the run
[`simple_opt_fig8.jl`](simple_opt_fig8.jl) was the rehearsal for — that one flies
the same path at constant length, which cannot exercise the winch coupling the
path was optimized for.

# The number this run exists to produce

The optimizer predicts a mean reel-out power for the path it returns. Reeling out
along it measures one. Both end up in the `traj_opt:` section of the run summary,
with their ratio: that comparison is the point of the exercise, and a large gap is
the finding, not a bug to hide. Everything else here is `simple_reelout.jl`.

# Re-optimizing while the tether grows

`reopt_enabled` in `data/traj_opt.yaml` (off by default, so a run stays
reproducible without a server) re-anchors the path to the length actually being
flown. A request goes out on a lap boundary every `reopt_every_n_laps`, at most
`max_reopt` times, and it repeats the STARTUP solve at the current length: `/init`
with the parametric guess of `traj_opt.yaml`, then `opt_step(...; wait = false)`.
`/status` is polled every `reopt_poll_interval` and the result collected from
`opt_trajectory` — whose table is in RADIANS, unlike the degrees of every struct.
While one runs, and after one that fails, the server keeps serving the previous
path, so "not ready" needs no special case.

The flown path is deliberately NOT fed back as the seed. It is an (azimuth,
elevation) curve optimized for a much shorter radius, so it degrades as a starting
point the further the run walks from its anchor, and the solve escapes to
near-zenith where `C_beta` jams on its 0.9 rad bound, tension collapses to ~1 kN
and the winch law cannot be met. Measured 2026-08-18: re-optimizations seeded that
way fail repeatedly from ~210 m out, while the guess converges at 182, 200, 246,
278, 282, 286 and 317 m — 282 m being one that failed from the flown path and
solves to 9014 W from the guess.

`reopt_blocking` decides what the loop does during the 7-13 s solve. `true`
(default) holds the loop at the request until `/status` leaves `"solving"`, so the
reply matches the length it was asked for; the frozen wall time is reported as
`traj_opt.reopt.blocked_s` and excluded from `performance.realtime_factor`.
`false` flies on instead, which keeps the run realtime-ish but anchors the reply to
a radius the run has already left — at 2.4 m/s of reel-out, a 10 s solve is ~24 m
of lag.

A candidate is checked for curvature, clearance and elevation AT THE CURRENT
LENGTH before it is installed; one that fails any of them is dropped and the run
keeps flying what it has. The checks run at the reply's OWN resolution while the
flown path is resampled to the run's `n_path`, and the split matters both ways:
`/trajectory` serves fewer points than `/step` echoes, upsampling a coarse
polyline makes the curvature check read tighter than the curve is, and a flown
path whose point count changes rescales the lap counter under itself. What passes is blended in over `path_blend_time` ([`blend_paths`](@ref)),
because installing a new reference in one step is a step in the cross-track error
and the guidance answers a step with steering. Both paths go through
[`prepare_path`](@ref) first — same point count, same traversal direction, same
starting point — since interpolating two paths that are not aligned sweeps the
reference around the pattern instead of morphing it. Installing the aligned old
path re-bases the point indices, so the lap counter is re-based in the same step.

Every solve is recorded in the `traj_opt.reopt` section of the run summary with
its time, tether length and outcome.

# Why the curvature margin is checked where it is

For ONE path flown all the way out, the start is the worst case. The kite's
minimum ANGULAR turn radius is `1/(L*c1*u_s)`, so a fixed (azimuth, elevation)
curve only ever gets easier to fly: measured 2026-08-18, the path returned for
150 m has margin 0.93 there and 2.18 by the time the tether is at 350 m. That is
why the startup check runs at the starting length.

With `reopt_enabled` that reasoning does NOT carry over, and the start is not the
worst case — every re-optimization is. Cancel the `L` in the angular radius and
the kite's minimum PHYSICAL turn radius is `1/(c1*u_s)`, INDEPENDENT of tether
length: 11.35 m at `c1 = 0.2752`, `u_s = 0.32`. The optimizer returns a pattern
whose tightest physical radius is 11.0-11.5 m at every length (measured
2026-08-18 at 182, 200 and 246 m), because its own `input_steering` (±0.35) and
`input_steering_rate` (±0.29) bounds are binding in every solve and maximum power
wants the tightest loops they allow. Two near-equal constants, so the margin sits
at ~1.0 at EVERY length and a solve landing at 0.85 or 1.01 is which optimum the
seed reached, not how long the tether is.

So a candidate is checked at the CURRENT length, every time, and rejections are
the normal case rather than a fault. The optimizer knows nothing of the V3's
turn-rate law; the real fix is to give it one, not to move the gate.

# What comes from where

The optimizer's own settings — server, initial guess, solver knobs, resampling
and the margin — are `data/traj_opt.yaml` ([`TrajOptSettings`](@ref)). The
CONDITIONS are not: the wind comes from the system project's settings file and
the winch law from its `wc_settings`, through `inflow_from_settings` and
`winch_from_wc`, so the path is optimized for the ground station that flies it.
The guess is not a formality — it decides whether the solve converges and which
of several optima it converges to, so this script never retries with a different
one behind your back; see `simple_opt_fig8.jl`'s docstring for the measurements.

`fcs.f8_a`/`f8_b`/`el_center` describe a lemniscate that is NOT flown here. The
pattern's centre (the dive target, and `var_04`) and its extent (the size criteria
of `fig8_metrics`) are measured off the installed path instead.

Logs to `output/<log_file>_opt.arrow` and `_opt.yaml` — the `_opt` suffix keeps
the lemniscate run's log and summary intact, so the two are comparable after the
fact. `REF_PATH` and `LOG_NAME` carry the flown curve and that name to
`simple_reelout_plots.jl`.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using WinchControllers: WCSettings, WinchController, calc_v_set, on_timer,
    get_state, get_f_err, wcsLowerForceLimit,
    LowerForceController, set_f_set, set_reset, set_v_sw, set_v_act,
    set_tracking, set_force, get_v_set_out, calc_vro
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: calc_wind_factor
using LinearAlgebra: norm
using Statistics: mean
using Printf
import Dates
using OrderedCollections: OrderedDict

@info "simple_opt_reelout.jl: reeling out along an externally optimized path."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
# The optimizer client: opt_init/opt_step/opt_trajectory and ensure_server.
include(joinpath(@__DIR__, "awetrim_client.jl"))
# Read and cleared HERE, so a `SHOW_PLOTS = false` never survives into the next run.
show_plots = @isdefined(SHOW_PLOTS) ? SHOW_PLOTS : true
SHOW_PLOTS = true
# The reference curve and the log name simple_reelout_plots.jl reads; both set
# below, once they are known, and cleared here for SHOW_PLOTS's reason.
REF_PATH = nothing
LOG_NAME = nothing
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Structural damping of the tether and bridle segments, as a ratio of their
# stiffness: unit_damping = ratio * unit_stiffness [s]. See simple_fig8.jl's docstring.
DAMPING_PER_STIFFNESS = 0.001
PROJECT = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
@info "simple_opt_reelout.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
# The optimizer's own settings: server, initial guess, solver knobs, margin.
tos = TrajOptSettings("traj_opt.yaml")

# Per-run overrides of the settings just loaded, for a SWEEP: read and cleared
# here like SHOW_PLOTS above, so a leftover value can never silently change the
# next interactive run. `examples/optimize_fig8.jl` sets it before each include.
fcs_overrides = @isdefined(FCS_OVERRIDES) ? FCS_OVERRIDES : Dict{Symbol, Any}()
FCS_OVERRIDES = Dict{Symbol, Any}()
for (key, value) in fcs_overrides
    hasfield(FC_Settings, key) ||
        error("FCS_OVERRIDES: \"$key\" is not a field of FC_Settings.")
    setfield!(fcs, key, convert(fieldtype(FC_Settings, key), value))
end
isempty(fcs_overrides) ||
    @info "fcs overrides in force: " * join(("$k = $v" for (k, v) in fcs_overrides), ", ")

project_set = Settings(project)
l_tether = project_set.l_tether

# Log files are arrow files, named after the project's `log_file`, kept out of git.
# OUTPUT_PATH redirects them, so that parallel runs of this script (the sweep)
# cannot overwrite each other's log, summary and archive. Read and cleared like
# SHOW_PLOTS; `nothing` is the default output/.
output_path = (@isdefined(OUTPUT_PATH) && !isnothing(OUTPUT_PATH)) ? OUTPUT_PATH :
              normpath(joinpath(@__DIR__, "..", "output"))
OUTPUT_PATH = nothing
mkpath(output_path)
# `_opt`: this run must not overwrite the lemniscate run's log and summary — the
# two are each other's baseline.
log_name = basename(project_set.log_file) * "_opt"

# ======================== INIT =========================== #

fcs.compliance >= 0 ||
    error("compliance must be >= 0, got $(fcs.compliance)")
fcs.compliance == 0 ||
    error("REEL_OUT needs compliance = 0 (POSITION mode) — REEL_OUT and V3Kite's own \
           FORCE mode both drive the winch and only one can hold the drum at a time.")
# ONE settings object for BOTH winch loops, and BOTH are ours now: V3Kite's
# `step!` takes a torque. Since the 2026-08-16 merge `WCSettings` carries the
# POSITION-mode torque gains (`winch_*`, which `wpc` below reads) AND
# WinchControllers.jl's speed-controller tuning (`kv`, `f_low`, ... which `rc`
# reads). `dt` is the file's one placeholder; the plant's timestep wins.
dt0 = 1 / project_set.sample_freq
wc = load_wc_settings(wc_settings(project); dt = dt0)
rcs = wc                                 # same object, two controllers read it
wpc = WinchPosController(wc; dt = dt0)   # the length loop `step!` used to own

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
    system_yaml = project, use_turbulence = TURBULENCE, aero_mode = AERO_MODE,
    sim_time = SIM_TIME, warmup_time = fcs.warmup_time,
    # The warm-up relaxes at constant length, against the same loop the run uses.
    warmup_torque = (m, l) -> winch_torque!(wpc, m, l))
@info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)

# REEL_OUT controller: built fresh here so its soft-start ramp (t_startup) begins
# the moment reel-out actually starts (phase 3), not at t = 0. `rcs` is `wc`, the
# object loaded above from the project's `wc_settings:` file — the two winches
# used to need two files in two schemas, and since the merge they share one.
# The file's `dt` is a placeholder; the plant's own timestep is the real one.
rcs.dt = s.dt
rc = WinchController(rcs)
stop_criteria = fcs.n_fig_eight > 0 ?
    @sprintf("%.0f m or after %d figures of eight", fcs.reelout_l_max, fcs.n_fig_eight) :
    @sprintf("%.0f m", fcs.reelout_l_max)
@info @sprintf("Winch: REEL_OUT mode — v_set = %.3f * sqrt(force), stopping at %s.",
               rcs.kv, stop_criteria)

# Standalone force-floor guard for phases 0-2 (park/dive/hold) — deliberately
# NOT `rc`. `WinchController`'s own SpeedController is ACTIVE (its integrator
# accumulating v_set_in - v_act = kv*sqrt(force) - v_act) whenever the force
# limiters are off, i.e. essentially the whole entry, since nothing before
# phase 3 is meant to move the drum. Left running "live" while its output is
# ignored, that integrator winds up over tens of seconds and dumps a large,
# stale command once something changes — MEASURED: v_reelout spiking to
# +8 m/s (`rcs.v_sat`, saturated) instead of the intended reel-IN, once `rc`
# was stepped throughout the entry to fix the force floor below. A bare
# `LowerForceController` has no SpeedController and no mixer to wind up, so
# the guard uses one of those instead; `rc` itself stays untouched (and its
# soft-start clock at 0) until phase 3, exactly as before this guard existed.
guard_lfc = LowerForceController(rcs)

# Length setpoint: starts at the tether length after settling and warm-up, and
# grows from the first step of phase 3 onward until it reaches `reelout_l_max`.
l_set = s.sys_state.l_tether[1]

fec = FigureEightController(FigureEightSettings(;
    dt = s.dt, A = fcs.f8_a, B = fcs.f8_b, C = fcs.f8_c, D = fcs.f8_d,
    az_center = 0.0, el_center = fcs.el_center,
    attractor_distance = fcs.attractor_dist, up_loops = fcs.up_loops))

# ================= OPTIMIZED REFERENCE PATH ================== #

# The conditions of THIS run, read off the files the plant was built from. `rcs`
# is the same WCSettings object the reel-out WinchController is built from, so
# the optimizer is given the winch law that is actually flown.
inflow = inflow_from_settings(project_set)
winch = winch_from_wc(rcs)
@info @sprintf("Optimizer conditions: %.1f m/s at 6 m from %.0f°, profile_law %d, \
                z0 = %g m | winch kv = %.4f, i.e. %.1f m/s at f_high = %.0f N.",
               inflow.wind_speed, inflow.wind_direction, inflow.profile_law, inflow.z0,
               winch.k_v, winch.k_v * sqrt(winch.f_max), winch.f_max)

# The seed, from data/traj_opt.yaml: it decides whether the solve converges and
# which optimum it converges to. `figure_eight_path` closes the curve itself.
guess_az, guess_el = figure_eight_path(tos.guess_a, tos.guess_b, tos.guess_c,
                                       tos.guess_d, 0.0, tos.guess_el_center,
                                       0.0, tos.guess_points)
@info @sprintf("Initial guess: %.0f° x %.0f° at %.0f°, %d points.",
               tos.guess_a, tos.guess_b, tos.guess_el_center, tos.guess_points)

# Anchored to the STARTING length. The pattern is optimal there and drifts off it
# as the tether grows; re-optimizing during the run is stage 4, not this script.
ensure_server(tos.base_url; autostart = tos.autostart_server)
opt_reply = opt_init(InitParams(; name = tos.name, length = l_set,
                                winch_params = winch, inflow_conditions = inflow,
                                trajectory = Trajectory(collect(guess_az), collect(guess_el)),
                                input_depower = tos.input_depower,
                                reg_weight = tos.reg_weight,
                                detect_simple_bounds = tos.detect_simple_bounds);
                     url = tos.base_url)
# No automatic retry with a different guess, deliberately: a guess that merely
# converges is not the same answer, and picking one silently would hide which
# optimum was flown.
opt_result = try
    opt_step(StepParams(l_set, winch, opt_reply.trajectory); url = tos.base_url)
catch exc
    exc isa HTTP.StatusError && exc.status == 422 || rethrow()
    error("""
          The optimizer returned no path: $(String(copy(exc.response.body)))

          Three candidates, most likely first:
            * the INITIAL GUESS is too far from the optimum for IPOPT to reach \
              it. Here that is guess_a = $(tos.guess_a)°, guess_b = \
              $(tos.guess_b)°, guess_el_center = $(tos.guess_el_center)° of \
              data/traj_opt.yaml, which seeds the request and nothing else — \
              widening or raising it changes the guess, not the flown path. \
              Measured at 150 m and 6 m/s: 20°/11° at 18° does not converge, \
              30°/12° and 20°/11°-at-26° do.
            * the winch is too stiff to reel out at the optimum: \
              kv*sqrt(f_high) = $(round(winch.k_v * sqrt(winch.f_max); digits = 1)) \
              m/s against $(inflow.wind_speed) m/s of wind at 6 m.
            * these conditions genuinely have no solution.

          `bin/run_server log` carries the solver's own output.""")
end
toc("Received the optimized path in: ")

# Resample, but NEVER upsample: the reply is a polyline, and interpolating extra
# points onto it concentrates each vertex's turn into one short segment, which
# makes path_radius_profile report a far tighter pattern than the curve is.
n_opt = length(opt_result.trajectory.azimuth) - 1
set_path!(fec, opt_result.trajectory.azimuth, opt_result.trajectory.elevation;
          resample = min(tos.resample_points, n_opt))

# The pattern's own geometry: fcs.f8_* and fcs.el_center describe the GUESS now.
# Captured now, not read off `fec` in the RESULTS block: with reopt_enabled the
# path in `fec` at the end of the run is not the one these describe.
n_path_initial = length(fec.az_path)
path_min_h_start = path_min_height(fec, l_tether)
az_c_path = 0.5 * (maximum(fec.az_path) + minimum(fec.az_path))
el_c_path = 0.5 * (maximum(fec.el_path) + minimum(fec.el_path))
az_amp_path = 0.5 * (maximum(fec.az_path) - minimum(fec.az_path))
el_height_path = maximum(fec.el_path) - minimum(fec.el_path)

# set_path! REVERSES a path that does not match up_loops, so a mismatch here is
# not caught by anything downstream: the kite would fly the optimizer's curve,
# but backwards, which is not the trajectory that was optimized.
opt_table = opt_trajectory(; url = tos.base_url)
opt_downloops = opt_table["spline"]["downloops"]
opt_power_pred = Float64(opt_table["metrics"]["avg_power_W"])
# Which path was flown when, so the run can be scored against the prediction of the
# path ACTUALLY in the air rather than only the first one. A re-optimization that
# installs appends to this; `path_blend_time` makes the swap gradual, but 4 s of
# blend against a ~100 s reeling window is inside the rounding.
pred_timeline = [(t = 0.0, power = opt_power_pred)]
opt_downloops == !fcs.up_loops ||
    error("The optimizer returned a downloops = $opt_downloops path while this run \
           flies up_loops = $(fcs.up_loops). Change fcs.up_loops or the guess; do \
           not fly it reversed.")
@info @sprintf("Optimized path: %d points, azimuth %.1f°…%.1f°, elevation \
                %.1f°…%.1f° (centre %.1f°), predicted mean reel-out power %.0f W.",
               length(fec.az_path), minimum(fec.az_path), maximum(fec.az_path),
               minimum(fec.el_path), maximum(fec.el_path), el_c_path, opt_power_pred)

# The ELEVATION floor, which is not the clearance floor and does not follow from
# it: `min_height` is satisfied at ever lower elevations as the tether grows, so a
# path that clears 50 m at 350 m of tether is at 8.2°. This repo's own criterion is
# an angle, and `fig8_metrics` fails a run that breaks it.
#
# Checked with `candidate_elevation_margin` on top, because this compares the
# REFERENCE path while the criterion scores the FLOWN one, and the kite flies below
# its reference near the lobe tips.
el_floor = fcs.min_elevation + tos.candidate_elevation_margin
minimum(fec.el_path) >= el_floor ||
    error(@sprintf("The optimized path descends to %.1f°, below min_elevation \
                    %.1f° + candidate_elevation_margin %.1f° = %.1f°. AWETrim \
                    constrains height and not elevation, so this is not something \
                    the solve avoids on its own.",
                   minimum(fec.el_path), fcs.min_elevation,
                   tos.candidate_elevation_margin, el_floor))

# The clearance floor, checked at the tether length this run flies. Independent of
# the turn-rate table above, so it runs whether or not `coeffs` was available.
if tos.min_height > 0
    clr = check_pattern_height(fec, l_tether, tos.min_height)
    clr.ok ||
        error(@sprintf("The optimized path's lowest point is %.1f m above ground at \
                        L = %.0f m (elevation %.1f°), below the min_height = %.0f m \
                        of data/traj_opt.yaml. The optimizer's own floor is a HEIGHT \
                        and it earns part of it by reeling out within the lap, which \
                        an installed (azimuth, elevation) curve does not. Raise the \
                        guess elevation, fly a longer tether, or lower min_height.",
                       clr.height, l_tether, clr.elevation, tos.min_height))
end

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
    # At l_tether (the START, before any reel-out) this is the WORST case: a longer
    # tether only ever shrinks the kite's minimum angular turn radius.
    feas_start = check_pattern_feasible(fec, l_tether, fcs.max_steering; c1)
    # A refusal, not a warning: the optimizer knows nothing of the V3's turn-rate
    # law, so a path the kite cannot turn along is a plausible thing for it to
    # return, and flying it measures the steering clamp instead of the path.
    feas_start.margin >= tos.min_feasibility_margin ||
        error(@sprintf("The optimized path asks for a turn radius of %.1f° where the \
                        kite manages %.1f° at the STARTING length %.0f m: margin \
                        %.2f, below min_feasibility_margin = %.2f. Lower \
                        body_damping, raise max_steering, or lower the margin in \
                        data/traj_opt.yaml to fly it anyway.",
                       feas_start.path_radius, feas_start.kite_radius, l_tether,
                       feas_start.margin, tos.min_feasibility_margin))
    feas_end = check_pattern_feasible(fec, fcs.reelout_l_max, fcs.max_steering; c1)

    # Dead-time context for attractor_dist: how long the lead arc takes to fly.
    lead_time = deg2rad(fcs.attractor_dist) * l_tether / fcs.v_app_ref
    @info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   fcs.attractor_dist, lead_time, fcs.v_app_ref, delay, lead_time / delay)
end

# The dive aims at the pattern centre, which is the OPTIMIZED path's now.
ccs = CourseControllerSettings(fcs; dt = s.dt)
ccs.el_center = el_c_path
cc = CourseController(ccs)

transition_start = NaN            # [s] time phase 3 began; `reelout_delay` counts from it
stop_start = NaN            # [s] time the soft-stop deceleration latched; NaN = not yet
stop_v_entry = NaN          # [m/s] v_set at the moment it latched
stop_T = NaN                # [s] duration of the linear decel to reach 0 at reelout_l_max
reelout_done = false        # true once either stop criterion has ended reel-out
stop_reason = ""            # "length", "laps", or "" if reel-out never stopped
e_mech = 0.0                # [Wh] running mechanical energy, logged for the viewer

# fig_8 (SysState field, live lap count): 0 before phase >= 4, 1 at first entry,
# +1 per full traversal of the reference path after. Named apart from the
# post-run `fig8` (phase-4 index list, below) which reuses that name.
fig8_n = 0
fig8_idx_prev = fec.last_idx
fig8_idx_progress = 0.0
n_path = length(fec.az_path)

# ---- Re-optimization (stage 4). The path is anchored to ONE radius and the run
# walks away from it; each re-optimization re-anchors it to the length being
# flown. The request is queued (`wait = false`), `/status` is polled, and the
# result is collected when it is there. While a solve runs, and after one that
# failed, the server keeps serving the previous path, so "not ready yet" needs no
# special case here either.
#
# `tos.reopt_blocking` decides what the loop does meanwhile. `false` flies on
# through the 7-13 s solve, so the reply is anchored to a radius the run has
# already left. `true` freezes the loop at the request, so the reply matches the
# length it was asked for; the frozen wall time is accumulated in
# `reopt_blocked_s` and kept out of the realtime figure.
reopt_pending = false       # a solve is queued on the server
reopt_n = 0                 # solves completed, accepted or rejected
reopt_lap = 0.0             # lap count at which the last request went out
reopt_next_poll = 0.0       # [s] next /status poll
reopt_t_request = NaN       # [s] when the pending request went out
reopt_blocked_s = 0.0       # [s] wall time spent frozen waiting for a reply
reopt_last_solve_s = NaN    # [s] wall time the last blocking wait took
reopt_events = NamedTuple[] # one row per solve, for the run summary
# The blend in progress: two canonical paths of equal length and a start time.
blend_from = nothing
blend_to = nothing
blend_t0 = NaN

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

        # Entry state machine, descent limiter, open-loop entry override, feedback
        # fusion, PID and rel_depower: see CourseController.
        heading = Float64(s.sys_state.heading)
        local v_kite = norm(s.sys_state.vel_kite)
        phase_before = cc.phase
        local rel_steering, rel_depower, phase = calc_steering(cc, chi_set, heading,
            Float64(s.sys_state.course);
            t, elevation = Float64(s.sys_state.elevation),
            v_kite, v_app = Float64(s.sys_state.v_app),
            dmin, tangent = path_tangent(fec))
        phase_before == 2 && phase == 3 && (global transition_start = t)
        # Separate from the ladder inside calc_steering so it can fire the SAME
        # step as a 3->4 transition: reel-out finishing does not wait for settling.
        if phase in (3, 4) && reelout_done
            set_phase!(cc, 5)
            phase = 5
            rel_depower = fcs.depower_final
        end
        chi_cmd = cc.chi_cmd
        w_lim = cc.w_lim
        w_course = cc.w_course
        err = cc.err

        # fig_8: jumps to 1 the instant phase first reaches >= 4 (a direct 3->5
        # reel-out finish can skip 4 entirely), then +1 per full traversal of the
        # reference path, unwrapped so a single lap never double-counts across
        # the index's `mod1` wrap.
        if phase >= 4
            if fig8_n == 0
                global fig8_n = 1
                global fig8_idx_prev = fec.last_idx
            else
                delta = fec.last_idx - fig8_idx_prev
                delta < -(n_path ÷ 2) && (delta += n_path)
                delta > n_path ÷ 2 && (delta -= n_path)
                global fig8_idx_progress += delta
                global fig8_idx_prev = fec.last_idx
                global fig8_n = 1 + floor(Int, fig8_idx_progress / n_path)
            end
        end

        # ---- Re-optimize the path for the length now being flown -------- #
        if tos.reopt_enabled && phase >= 4
            l_now = Float64(s.sys_state.l_tether[1])

            # Queue: on a lap boundary, never while a solve or a blend is running,
            # and never past max_reopt.
            if !reopt_pending && isnothing(blend_to) && reopt_n < tos.max_reopt &&
               fig8_idx_progress >= (reopt_lap + tos.reopt_every_n_laps) * n_path
                try
                    # RE-INIT at the current length and seed from the parametric
                    # guess, exactly as the startup solve does — do NOT feed the
                    # flown path back. That path is in (azimuth, elevation) and was
                    # optimized for a much shorter radius, so the further the run
                    # walks from its anchor the worse a starting point it is, and
                    # the solve escapes to near-zenith: measured 2026-08-18 at
                    # 282 m, C_beta jams on its 0.9 rad bound at ~50° of elevation
                    # where tension collapses to ~1 kN and the winch law cannot be
                    # met. The same 282 m solves to 9014 W from the guess.
                    #
                    # `reopt_retry_el_offset` adds ONE retry from a shifted guess
                    # when the first attempt fails, which only the blocking mode can
                    # do — see its docstring for why a re-optimization may retry
                    # where the startup solve deliberately may not.
                    el_seeds = (tos.reopt_blocking && tos.reopt_retry_el_offset != 0) ?
                        (tos.guess_el_center,
                         tos.guess_el_center + tos.reopt_retry_el_offset) :
                        (tos.guess_el_center,)
                    for (attempt, el_seed) in enumerate(el_seeds)
                        guess_az_r, guess_el_r =
                            figure_eight_path(tos.guess_a, tos.guess_b, tos.guess_c,
                                              tos.guess_d, 0.0, el_seed,
                                              0.0, tos.guess_points)
                        reopt_reply = opt_init(InitParams(; name = tos.name, length = l_now,
                                                          winch_params = winch,
                                                          inflow_conditions = inflow,
                                                          trajectory = Trajectory(collect(guess_az_r),
                                                                                  collect(guess_el_r)),
                                                          input_depower = tos.input_depower,
                                                          reg_weight = tos.reg_weight,
                                                          detect_simple_bounds = tos.detect_simple_bounds);
                                               url = tos.base_url)
                        opt_step(StepParams(l_now, winch, reopt_reply.trajectory);
                                 url = tos.base_url, wait = false)
                        global reopt_pending = true
                        global reopt_t_request = t
                        global reopt_lap = fig8_idx_progress / n_path
                        global reopt_next_poll = t + tos.reopt_poll_interval
                        @info @sprintf("Re-optimizing for L = %.0f m at t = %.1f s \
                                        (lap %.1f, request %d of %d, guess el %.0f°)%s.",
                                       l_now, t, reopt_lap, reopt_n + 1, tos.max_reopt,
                                       el_seed,
                                       tos.reopt_blocking ? " — holding the simulation" : "")
                        # Freeze here rather than in the collect branch, so the reply
                        # is anchored to `l_now` and not to a length the run drifted to.
                        tos.reopt_blocking || break
                        t_block = time()
                        while (try
                                   opt_status(tos.base_url)["state"]
                               catch exc
                                   @warn "Could not reach the optimizer while \
                                          holding; will retry." exception = exc
                                   "solving"
                               end) == "solving"
                            sleep(tos.reopt_poll_interval)
                        end
                        global reopt_last_solve_s = time() - t_block
                        global reopt_blocked_s += reopt_last_solve_s
                        # Collect on THIS step: the reply is already on the server.
                        global reopt_next_poll = t
                        # Retry only a solver failure, and only while a seed is left.
                        failed = (try
                                      opt_status(tos.base_url)["state"]
                                  catch; "failed"; end) == "failed"
                        (failed && attempt < length(el_seeds)) || break
                        @info @sprintf("  ... failed from guess el %.0f°; retrying \
                                        from %.0f°.", el_seed,
                                       tos.guess_el_center + tos.reopt_retry_el_offset)
                    end
                catch exc
                    # A refused request must not take the run with it: the path in
                    # the air is still the one from /init and is still flyable.
                    global reopt_n += 1
                    push!(reopt_events, (; t, l = l_now, status = "request failed",
                                         detail = first(sprint(showerror, exc), 120)))
                    @warn "Re-optimization request failed; flying on with the \
                           current path." exception = exc
                end
            end

            # Collect: poll rather than block, and validate before installing.
            if reopt_pending && t >= reopt_next_poll
                global reopt_next_poll = t + tos.reopt_poll_interval
                state = try
                    opt_status(tos.base_url)["state"]
                catch exc
                    @warn "Could not reach the optimizer; will retry." exception = exc
                    "solving"
                end
                if state != "solving"
                    global reopt_pending = false
                    global reopt_n += 1
                    event = (; t, l = l_now, status = state, detail = "")
                    if state == "converged"
                        tab = opt_trajectory(; url = tos.base_url)
                        # /trajectory is in RADIANS, unlike the degrees of the structs.
                        new_az = rad2deg.(Float64.(tab["table"]["azimuth"]))
                        new_el = rad2deg.(Float64.(tab["table"]["elevation"]))
                        # TWO resolutions of the same reply, on purpose.
                        #
                        # The CHECKS run at the reply's own resolution: /trajectory
                        # serves the server's n_points (99) where /step echoed the
                        # guess (360), and interpolating a coarse polyline up
                        # concentrates each vertex's turn into one short segment,
                        # which makes path_radius_profile read far tighter than the
                        # curve is. A curvature check on an upsampled path measures
                        # the sampling.
                        #
                        # What is FLOWN is resampled to the run's own `n_path`, so
                        # the point count never changes mid-run: `fig8_idx_progress`
                        # counts points, and a path with a different count silently
                        # rescales the lap counter. The guidance itself is
                        # indifferent — it walks arc length, and the extra points sit
                        # on the same curve.
                        n_native = min(tos.resample_points, length(new_az) - 1)
                        chk_az, chk_el = prepare_path(new_az, new_el;
                            resample = n_native, up_loops = fcs.up_loops)
                        cand_az, cand_el = prepare_path(new_az, new_el;
                            resample = n_path, up_loops = fcs.up_loops)
                        # At the CURRENT length, which is what it will be flown at.
                        margin = isnothing(coeffs) ? Inf :
                            check_pattern_feasible(chk_az, chk_el, l_now,
                                fcs.max_steering; c1, prn = false).margin
                        clearance = path_min_height(chk_az, chk_el, l_now)
                        if margin < tos.min_feasibility_margin
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("curvature margin %.2f", margin))
                        elseif tos.min_height > 0 && clearance < tos.min_height
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("clearance %.1f m", clearance))
                        elseif minimum(chk_el) < el_floor
                            # The clearance floor does NOT imply this one: at 318 m
                            # of tether, 50 m of height is 9° of elevation. The
                            # margin is there because the kite flies BELOW its
                            # reference — ~3° measured, twice.
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("descends to %.1f°, below \
                                                        min_elevation + margin = %.1f°",
                                                       minimum(chk_el), el_floor))
                        else
                            global blend_from = prepare_path(fec.az_path, fec.el_path;
                                resample = n_path, up_loops = fcs.up_loops)
                            global blend_to = (cand_az, cand_el)
                            global blend_t0 = t
                            # Install the aligned OLD path (w = 0, same curve, new
                            # point indices) and re-base the lap counter on it in
                            # the same step. `prepare_path` rotates the start point
                            # to the azimuth extreme, so without this the next
                            # `fec.last_idx - fig8_idx_prev` reads the re-indexing
                            # as a fraction of a lap the kite never flew.
                            set_path!(fec, blend_from[1], blend_from[2];
                                      up_loops = fcs.up_loops)
                            global fig8_idx_prev = fec.last_idx
                            # A no-op while the path above is resampled to
                            # `n_path`, and kept as the guard that says why it must
                            # be: `fig8_idx_progress` counts POINTS, so a path with a
                            # different count rescales the lap counter under it —
                            # 4 -> 13 in the run of 2026-08-18, before the flown path
                            # was pinned to one resolution.
                            n_path_new = length(fec.az_path)
                            global fig8_idx_progress *= n_path_new / n_path
                            global n_path = n_path_new
                            new_pred = Float64(tab["metrics"]["avg_power_W"])
                            push!(pred_timeline, (t = t, power = new_pred))
                            event = (; t, l = l_now, status = "installed",
                                     detail = @sprintf("margin %.2f, clearance %.1f m, \
                                                        %.0f W predicted", margin, clearance,
                                                       new_pred))
                        end
                    end
                    push!(reopt_events, event)
                    # Blocking collects on the SAME step as the request, so the
                    # simulated gap is 0 by construction; the wall time is the figure
                    # that means something there.
                    @info @sprintf("Re-optimization %d: %s%s (%s).",
                                   reopt_n, event.status,
                                   isempty(event.detail) ? "" : " — " * event.detail,
                                   tos.reopt_blocking ?
                                       @sprintf("%.1f s of wall time, held", reopt_last_solve_s) :
                                       @sprintf("%.1f s of sim after the request",
                                                t - reopt_t_request))
                end
            end

            # Blend: a step in the reference is a step in the cross-track error,
            # and the guidance answers a step with steering.
            if !isnothing(blend_to)
                w = (t - blend_t0) / tos.path_blend_time
                b_az, b_el = blend_paths(blend_from[1], blend_from[2],
                                         blend_to[1], blend_to[2], w)
                set_path!(fec, b_az, b_el; up_loops = fcs.up_loops)
                if w >= 1
                    global blend_from = nothing
                    global blend_to = nothing
                end
            end
        end

        # REEL_OUT: `reelout_delay` seconds after phase 3 (guidance engaged), not
        # at phase 4 (fig8), and only while l_set has not yet hit
        # reelout_l_max. Once it does, l_set simply stops growing and the rest of
        # the run is flown exactly like the constant-length example.
        local v_set = 0.0
        if phase >= 3 && t - transition_start >= fcs.reelout_delay &&
           !reelout_done
            # The INSTANTANEOUS force: reeling out faster exactly when the kite
            # pulls harder is what regulates the force. Lagging it is closed, see
            # docs/fig8_tuning_log.md.
            v_raw = calc_v_set(rc, reel_out_speed(s), winch_force(s), rcs.f_low)
            # Ramps the COMMAND, not the law: rc's internal state (integrators,
            # force limiters) sees the true v_raw throughout, only the value
            # handed to l_set/v_ff is scaled. `t_startup` does not do this — see
            # its docstring in `src/fc_settings.jl`.
            ramp = fcs.reelout_softstart > 0 ?
                clamp((t - transition_start - fcs.reelout_delay) / fcs.reelout_softstart,
                      0.0, 1.0) : 1.0
            v_cmd = ramp * v_raw

            remaining = fcs.reelout_l_max - l_set
            # Soft-stop: a hard cut of v_set to 0 the instant l_set clamps to
            # reelout_l_max leaves the drum with the old command's momentum —
            # the POSITION loop then brakes it with a transient reel-IN (a power
            # undershoot). LOOSENING the acceleration limit instead (tried, see
            # `docs/fig8_tuning_log.md`, "soft-stop") makes it WORSE: the drum
            # coasts further past l_max before turning around, so the error the
            # position loop corrects is BIGGER, not smaller (measured: 0.36 m
            # overshoot / -3 kW at 8 m/s² becomes 3.5 m / -6.7 kW at 1 m/s²).
            # The fix has to be in v_set/l_set's own trajectory, matching the
            # CURRENT commanded speed at the moment braking starts (not 0 — that
            # would just move the discontinuity to the start of the ramp) and
            # landing on exactly 0 at reelout_l_max: latch once the remaining
            # distance would be covered within `reelout_softstop` seconds AT THE
            # CURRENT RATE, then decelerate LINEARLY from `v_cmd` to 0. A linear
            # ramp's area is `v_entry*T/2`, so `stop_T` (usually ~2x
            # `reelout_softstop`) is solved for exactly, not just guessed.
            if isnan(stop_start) && fcs.reelout_softstop > 0 && v_cmd > 0 &&
               remaining <= v_cmd * fcs.reelout_softstop
                global stop_start = t
                global stop_v_entry = v_cmd
                global stop_T = 2 * remaining / v_cmd
            end
            # Second stop criterion: N COMPLETE laps since the counter started at phase 4.
            # `fig8_idx_progress`, not `fig8_n`, which reads 1 during the first lap.
            if isnan(stop_start) && fcs.n_fig_eight > 0 &&
               fig8_idx_progress >= fcs.n_fig_eight * n_path
                global stop_reason = "laps"
                if fcs.reelout_softstop > 0 && v_cmd > 0
                    global stop_start = t
                    global stop_v_entry = v_cmd
                    # No remaining distance to solve T from — unlike the reelout_l_max
                    # latch, the length is the free variable here. Same nominal duration.
                    global stop_T = 2 * fcs.reelout_softstop
                else
                    global reelout_done = true   # hard stop, as reelout_l_max does today
                end
            end
            v_set = isnan(stop_start) ? v_cmd :
                stop_v_entry * (1 - clamp((t - stop_start) / stop_T, 0.0, 1.0))
            global l_set = min(l_set + v_set * s.dt, fcs.reelout_l_max)
            on_timer(rc)
            if l_set >= fcs.reelout_l_max
                global reelout_done = true
                isempty(stop_reason) && (global stop_reason = "length")
            elseif !isnan(stop_start) && stop_reason == "laps" && t - stop_start >= stop_T
                global reelout_done = true   # the soft-stop ramp has run out
            end
        elseif phase < 3
            # Force floor BEFORE reel-out starts. `l_set` is otherwise held flat
            # at the settled length here, but the dive can sag tether force well
            # below `rcs.f_low` (measured: ~50 N at t ~ 5.2 s, entry_depower
            # unloading the wing) with nothing to catch it. `guard_lfc` (built
            # above, deliberately NOT `rc` — see the comment there) is stepped
            # by hand through the same setters `calc_v_set` uses internally.
            set_reset(guard_lfc, false)
            set_f_set(guard_lfc, rcs.f_low)
            set_v_sw(guard_lfc, calc_vro(rcs, rcs.f_low) * 1.05)
            set_v_act(guard_lfc, reel_out_speed(s))
            set_tracking(guard_lfc, 0.0)   # bumpless: l_set is otherwise flat here
            set_force(guard_lfc, winch_force(s))
            # Reel-IN only: guard_lfc's own saturation allows v_sat (reel-out) on
            # the upper side too, meant for the main `rc` controller it shares a
            # type with. Before reel-out starts this guard exists to catch a force
            # SAG, never to reel out, so its output is clamped here.
            v_guard = min(get_v_set_out(guard_lfc), 0.0)
            on_timer(guard_lfc)
            if guard_lfc.active
                v_set = v_guard
                global l_set = l_set + v_set * s.dt
            end
        end

        # `v_ff = v_set`: the winch's outer P loop is told the speed being
        # commanded instead of having to rediscover it from a length error. See
        # the docstring — without it the pair (integrate here, differentiate
        # there) is a 1/winch_pos_kp = 2 s lag. Zero outside the reel-out window,
        # where `l_set` is constant and there is nothing to feed forward.
        # `acceleration_limit` is `rcs.max_acc` (8 m/s²), NOT the plant's own
        # `winch: max_acc:` = 4 which V3Kite's `step!` would otherwise default to.
        # Deliberate, and measured: the total setpoint asks for more than 4 m/s² in
        # ~1 % of the steps and more than 8 in 0.03 %, so the limiter is nearly
        # never the binding constraint, and tightening it to 4 only made the drum
        # lag the `t_startup` ramp harder — the engagement ring grew from 0.88 to
        # 0.98 m/s and the peak force rose slightly. See Plan.md.
        step!(s; rel_depower, rel_steering, vsm_interval = fcs.vsm_interval,
              set_torque = winch_torque!(wpc, s, l_set; v_ff = v_set,
                                         speed_limit = rcs.v_sat,
                                         acceleration_limit = rcs.max_acc))

        # Report the overspeed rather than the opaque solver abort it causes later.
        if Float64(s.sys_state.v_app) > fcs.v_app_abort
            @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                             Stopping before the solver diverges.",
                            s.sys_state.time, s.sys_state.v_app, fcs.v_app_abort,
                            rad2deg(s.sys_state.elevation), rad2deg(s.sys_state.AoA))
            break
        end

        # After step!, which overwrites parts of sys_state.
        s.sys_state.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 transition, 4 fig8, 5 final
        s.sys_state.bearing = chi_cmd          # the course actually tracked
        s.sys_state.attractor .= (deg2rad(az_attr), deg2rad(el_attr))
        s.sys_state.var_01 = dmin              # cross-track error [deg]
        s.sys_state.var_02 = az_attr           # attractor azimuth [deg]
        s.sys_state.var_03 = el_attr           # attractor elevation [deg]
        s.sys_state.var_04 = el_c_path         # pattern-centre elevation [deg]
        s.sys_state.var_05 = chi_set           # RAW guidance course [rad]
        s.sys_state.var_06 = rad2deg(err)      # REGULATED error [deg]
        # A weight, not a flag: a step here means entry_d_blend is too narrow.
        s.sys_state.var_07 = abs(chi_set) > deg2rad(fcs.entry_chi_max) ? w_lim : 0.0
        s.sys_state.var_08 = w_course          # course/heading blend weight [-]
        # Whole wing; sys_state.AoA is the centre panel only, which a turn twists away from.
        s.sys_state.var_09 = rad2deg(span_mean_aoa(s.sys))
        s.sys_state.fig_8 = Int16(fig8_n)      # live lap count
        s.sys_state.var_10 = l_set             # tether length setpoint [m]
        s.sys_state.var_11 = v_set             # REEL_OUT speed setpoint [m/s]
        s.sys_state.var_12 = get_state(rc)     # WinchController state (0/1/2)
        s.sys_state.var_13 = get_f_err(rc)     # force error [N], NaN in speed control
        # Not filled anywhere in the model chain: without this the log and the viewer read 0.
        s.sys_state.v_wind_200m .= calc_wind_factor(s.am, 200.0) .* s.sys_state.v_wind_gnd
        # Same story for e_mech, which KiteViewers' status text prints in Wh: the
        # running integral of the SAME p_mech the viewer computes per frame.
        global e_mech += s.sys_state.winch_force[1] * s.sys_state.v_reelout[1] *
                         s.dt / 3600
        s.sys_state.e_mech = e_mech
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

"""
    write_yaml_commented(io, indent, node)

Serialize a nested `OrderedDict` as YAML, recursing into `OrderedDict` values
and appending a trailing `# comment` for leaves given as `(value, comment)`
pairs, aligned to `comment_col` where the line is short enough.
`YAML.write_file` has no concept of comments, hence this by hand.
"""
function write_yaml_commented(io, indent, node; comment_col = 36)
    pad = "  "^indent
    for (k, v) in node
        if v isa AbstractDict
            println(io, pad, k, ":")
            write_yaml_commented(io, indent + 1, v; comment_col)
        else
            value, comment = v isa Tuple ? v : (v, "")
            val = value isa AbstractString ? "\"$value\"" : string(value)
            prefix = string(pad, k, ": ", val)
            if isempty(comment)
                println(io, prefix)
            else
                println(io, prefix, " "^max(1, comment_col - length(prefix)), "# ", comment)
            end
        end
    end
end

syslog = load_log(log_name; path = output_path)
sl = syslog.syslog
# The geometry is passed in too: without it the criteria are blind to pattern SIZE.
# require_final: this script's own phase 5, unlike simple_fig8.jl's sys_state
# (which never goes past 4) — checks reel-out actually finished within the run.
# From the flown path, not from fcs.f8_*. `az_center` is in RADIANS here (it is
# compared against the logged azimuth), the two extents in degrees.
fig8m = print_fig8_metrics(sl; t_start = fcs.park_time, settle_time = fcs.entry_time,
                   min_elevation = fcs.min_elevation, az_center = deg2rad(az_c_path),
                   az_amplitude = az_amp_path, el_height = el_height_path,
                   min_span_frac = fcs.min_span_frac, require_final = true)

summary = OrderedDict{String, Any}()
run_time = Dates.now()
# The package repo, not `examples/` — this reports the controller code, not the script's own project.
pkg_dir = pkgdir(SimpleKiteControllers)
git_hash, git_status = try
    hash = strip(read(`git -C $pkg_dir rev-parse --short HEAD`, String))
    dirty = !isempty(strip(read(`git -C $pkg_dir status --porcelain`, String)))
    (hash, dirty ? "dirty" : "clean")
catch
    ("unknown", "unknown")
end
summary["simulation"] = OrderedDict{String, Any}(
    "script" => (basename(@__FILE__), "script that produced this run"),
    "project" => (PROJECT, "system project flown"),
    "rel_turbulence" => (TURBULENCE, "turbulence level in [0, 1] passed to init"),
    "time" => (Dates.format(run_time, "HH:MM:SS"), "wall-clock time the run finished"),
    "date" => (Dates.format(run_time, "yyyy-mm-dd"), "wall-clock date the run finished"),
    "hostname" => (gethostname(), "machine the run executed on"),
    "git_hash" => (git_hash, "SimpleKiteControllers.jl commit hash"),
    "git_status" => (git_status, "SimpleKiteControllers.jl working tree: clean or dirty"))
if fig8m !== nothing
    summary["fig8_metrics"] = OrderedDict{String, Any}(
        "settled_from_s" => (round(fig8m.stats_start; digits = 1), "sim time the scoring window begins [s]"),
        "settle_time_s" => (round(fig8m.settle_time_used; digits = 1), "time after t_start to converge [s]"),
        "laps" => (fig8m.laps, "figure-eight laps completed"),
        "cross_track_deg" => OrderedDict(
            "rms" => (round(fig8m.rms_d; digits = 2), "RMS cross-track error [deg]"),
            "mean" => (round(fig8m.mean_d; digits = 2), "mean cross-track error [deg]"),
            "max" => (round(fig8m.max_d; digits = 2), "max cross-track error [deg]")),
        "elevation_deg" => OrderedDict(
            "min_settled" => (round(fig8m.min_elevation_settled; digits = 1), "min elevation, settled window [deg]"),
            "min_whole_run" => (round(fig8m.min_elevation_all; digits = 1), "min elevation, whole run [deg]")),
        "peak_turn_rate_deg_s" => (round(Int, fig8m.max_turn_rate), "peak heading rate [deg/s]"),
        "extent" => OrderedDict(
            "azimuth_deg" => OrderedDict(
                "min" => (round(-fig8m.az_reach_neg; digits = 1), "flown azimuth reach, negative side [deg]"),
                "max" => (round(fig8m.az_reach_pos; digits = 1), "flown azimuth reach, positive side [deg]")),
            "azimuth_pct_of_amplitude" => OrderedDict(
                "min" => (round(Int, 100 * fig8m.az_fill_neg), "negative reach as % of the pattern's ±A"),
                "max" => (round(Int, 100 * fig8m.az_fill_pos), "positive reach as % of the pattern's ±A")),
            "worst_lobe_deg" => OrderedDict(
                "min" => (round(-fig8m.az_reach_neg_worst; digits = 1), "weakest lobe, negative side [deg]"),
                "max" => (round(fig8m.az_reach_pos_worst; digits = 1), "weakest lobe, positive side [deg]")),
            "elevation_span_deg" => (round(fig8m.el_span; digits = 1), "flown elevation span [deg]"),
            "elevation_span_pct_of_b" => (round(Int, 100 * fig8m.el_fill), "flown span as % of the pattern's B")),
        "tether_force_N" => OrderedDict(
            "mean" => (round(Int, fig8m.mean_force), "mean tether force, settled window [N]"),
            "std" => (round(Int, fig8m.std_force), "std tether force, settled window [N]"),
            "cv_pct" => (round(100 * fig8m.cv_force; digits = 1), "coefficient of variation [%]")),
        "steering" => OrderedDict(
            "peak_abs_u_s" => (round(fig8m.max_steering_used; digits = 3), "peak |rel_steering| commanded"),
            "pct_time_within_2pct_of_peak" => (round(Int, 100 * fig8m.steering_sat_frac),
                "% of time within 2% of peak (saturation)"),
            "hf_std_steering" => (round(fig8m.steering_hf_std; digits = 4), "high-frequency std of steering (chatter)"),
            "hf_std_turnrate_deg_s" => (round(fig8m.turnrate_hf_std; digits = 2),
                "high-frequency std of heading rate [deg/s]")),
        "tape" => OrderedDict(
            "delivered_peak_abs_u_s" => (round(fig8m.max_steering_delivered; digits = 3),
                "peak steering the KCU tape delivered"),
            "commanded_peak_abs_u_s" => (round(fig8m.max_steering_used; digits = 3), "peak steering commanded"),
            "rate_limited_pct_time" => (round(Int, 100 * fig8m.tape_rate_frac),
                "% of time the tape's rate limit was hit"),
            "rate_limited_peak_per_s" => (round(fig8m.max_tape_rate; digits = 3), "peak tape rate reached [1/s]"),
            "rate_limit_per_s" => (fig8m.v_steering, "KCU's configured rate limit [1/s]")),
        "success_criteria" => (isempty(fig8m.criteria_failed) ? "all $(fig8m.criteria) passed" :
            "FAILED: " * join(fig8m.criteria_failed, ", "), "pass/fail verdict vs V3Kite's success criteria"))
end

reelout_summary = OrderedDict{String, Any}()
# On the LOGGED PHASE, not a time window; the mean is what v_app_ref should be.
fig8 = findall(x -> Int(x) == 4, sl.sys_state)
if isempty(fig8)
    # Reel-out runs from phase 3, so this is about the anchor only.
    @warn "Phase 4 never reached — no fig8 apparent wind speed."
else
    va = Float64.(sl.v_app[fig8])
    duration = sl.time[fig8[end]] - sl.time[fig8[1]]
    @printf("  v_app over phase 4 (%.1f s): mean %.2f m/s, range %.2f … %.2f m/s \
             | v_app_ref = %.1f (%+.1f%%)\n",
            duration, mean(va), minimum(va), maximum(va),
            fcs.v_app_ref, 100 * (mean(va) / fcs.v_app_ref - 1))
    reelout_summary["v_app_phase4"] = OrderedDict(
        "duration_s" => (round(duration; digits = 1), "phase-4 window length [s]"),
        "mean_m_s" => (round(mean(va); digits = 2), "mean apparent wind speed [m/s]"),
        "min_m_s" => (round(minimum(va); digits = 2), "min apparent wind speed [m/s]"),
        "max_m_s" => (round(maximum(va); digits = 2), "max apparent wind speed [m/s]"),
        "v_app_ref_m_s" => (fcs.v_app_ref, "reference apparent wind speed [m/s]"),
        "deviation_pct" => (round(100 * (mean(va) / fcs.v_app_ref - 1); digits = 1),
            "mean v_app deviation from v_app_ref [%]"))
end
# Unconditional: the winch is gated on phase 3, which phase 4 may never follow.
# stop_reason is "" when the run ended with reel-out still going.
reelout_stop_reason = isempty(stop_reason) ? "none" : stop_reason
@printf("  Tether: %.1f m -> %.1f m (target %.1f m, stopped by: %s).\n",
        l_tether, sl.var_10[end], fcs.reelout_l_max, reelout_stop_reason)
reelout_summary["tether"] = OrderedDict(
    "start_m" => (l_tether, "tether length at run start [m]"),
    "end_m" => (round(Float64(sl.var_10[end]); digits = 1), "tether length at run end [m]"),
    "target_m" => (fcs.reelout_l_max, "reelout_l_max target [m]"),
    "stop_reason" => (reelout_stop_reason, "criterion that ended reel-out: length, laps, or none"),
    "laps_reeled" => (round(fig8_idx_progress / n_path; digits = 2),
        "figure-eight laps completed by the time reel-out ended"))

rp = reelout_power(sl)
if isnothing(rp)
    @warn "Tether never reeled out — no reel-out power to report."
else
    @printf("  Reel-out force: mean %.0f N, peak %.0f N (cf=%.2f).\n",
            rp.mean_force, rp.peak_force, rp.cf_force_ro)
    # Both totals: an earlier engagement trades mean power for a longer window.
    @printf("  Reel-out power: mean %.0f W, peak %.0f W (cf=%.2f) over %.1f s (%d samples), \
             E = %.1f kJ; whole run E = %.1f kJ.\n",
            rp.mean_power, rp.peak_power, rp.cf_power_ro, rp.duration, rp.n,
            rp.energy / 1000, rp.energy_run / 1000)
    reelout_summary["force"] = OrderedDict(
        "mean_N" => (round(Int, rp.mean_force), "mean tether force over the reeling window [N]"),
        "peak_N" => (round(Int, rp.peak_force), "peak tether force over the reeling window [N]"),
        "cf_force_ro" => (round(rp.cf_force_ro; digits = 2), "crest factor: peak_N / mean_N"))
    reelout_summary["power"] = OrderedDict(
        "mean_W" => (round(Int, rp.mean_power), "mean reel-out power over the reeling window [W]"),
        "peak_W" => (round(Int, rp.peak_power), "peak reel-out power over the reeling window [W]"),
        "cf_power_ro" => (round(rp.cf_power_ro; digits = 2), "crest factor: peak_W / mean_W"),
        "duration_s" => (round(rp.duration; digits = 1), "reeling window length [s]"),
        "n_samples" => (rp.n, "sample count in the reeling window"),
        "energy_kJ" => (round(rp.energy / 1000; digits = 1), "energy over the reeling window [kJ]"),
        "energy_run_kJ" => (round(rp.energy_run / 1000; digits = 1), "energy over the whole run [kJ]"))
end

ws = winch_state_pct(sl)
if isnothing(ws)
    @warn "Tether never reeled out — no winch controller states to report."
else
    @printf("  Winch states over the reeling window: speed %.1f %%, lower force %.1f %%, \
             upper force %.1f %%.\n",
            ws.speed_pct, ws.lower_force_pct, ws.upper_force_pct)
    reelout_summary["winch_state"] = OrderedDict(
        "speed_pct" => (round(ws.speed_pct; digits = 1),
            "% of the reeling window in speed control (state 1)"),
        "lower_force_pct" => (round(ws.lower_force_pct; digits = 1),
            "% in the LowerForceController, reeling in (state 0)"),
        "upper_force_pct" => (round(ws.upper_force_pct; digits = 1),
            "% in the UpperForceController, force capped at f_high (state 2)"),
        "n_samples" => (ws.n, "sample count in the reeling window"))
end

rr = reelout_ringing(sl)
if isnothing(rr)
    @warn "Tether never reeled out — no ringing to report."
elseif rr.n_peaks == 0
    @printf("  Reel-out ring: none detected (peak %.2f m/s vs steady %.2f m/s).\n",
            rr.peak_v_reelout_m_s, rr.steady_v_reelout_m_s)
    reelout_summary["ringing"] = OrderedDict(
        "n_peaks" => (0, "ring peaks detected above peak_floor"),
        "peak_v_reelout_m_s" => (round(rr.peak_v_reelout_m_s; digits = 2), "raw v_reelout max within ring_span [m/s]"),
        "steady_v_reelout_m_s" => (round(rr.steady_v_reelout_m_s; digits = 2), "mean v_reelout after ring_span [m/s]"))
else
    @printf("  Reel-out ring: period %.2f s, zeta %.2f, overshoot %.2f m/s, decays in %.1f s \
             (peak %.2f m/s vs steady %.2f m/s).\n",
            rr.period_s, rr.zeta, rr.overshoot_m_s, rr.duration_s,
            rr.peak_v_reelout_m_s, rr.steady_v_reelout_m_s)
    reelout_summary["ringing"] = OrderedDict(
        "n_peaks" => (rr.n_peaks, "ring peaks detected above peak_floor"),
        "period_s" => (round(rr.period_s; digits = 2), "mean peak-to-peak ring period [s]"),
        "zeta" => (round(rr.zeta; digits = 3), "damping ratio from the peak log decrement"),
        "overshoot_m_s" => (round(rr.overshoot_m_s; digits = 2), "first ring peak's amplitude above the local trend [m/s]"),
        "duration_s" => (round(rr.duration_s; digits = 1), "time until the ring decays below settle_frac of overshoot_m_s [s]"),
        "peak_v_reelout_m_s" => (round(rr.peak_v_reelout_m_s; digits = 2), "raw v_reelout max within ring_span [m/s]"),
        "steady_v_reelout_m_s" => (round(rr.steady_v_reelout_m_s; digits = 2), "mean v_reelout after ring_span [m/s]"))
end
summary["reelout"] = reelout_summary

# The comparison this script exists for: what the optimizer promised against what
# reeling out delivered. `rp` is `nothing` when the tether never reeled out, and
# then there is nothing to compare.
#
# The prediction is a WEIGHTED one whenever a re-optimization installed a path. Each
# path carries its own `avg_power_W`, and the run flies each for part of the reeling
# window, so scoring the whole window against the FIRST path's number compares the
# measurement to a path that was not in the air for some of it. `rp.idx` is that
# window's samples, so every share below is measured over exactly the samples
# `measured_W` averages.
opt_power_meas = isnothing(rp) ? nothing : rp.mean_power
pred_shares = NamedTuple[]
opt_power_pred_eff = opt_power_pred
if !isnothing(rp)
    t_ro = Float64.(sl.time[rp.idx])
    # Which timeline entry was current at each reeling sample.
    which = [findlast(e -> e.t <= tq, pred_timeline) for tq in t_ro]
    for (k, e) in enumerate(pred_timeline)
        share = count(==(k), which) / length(which)
        share > 0 && push!(pred_shares, (; from_s = e.t, power = e.power, share))
    end
    opt_power_pred_eff = sum(p.power * p.share for p in pred_shares)
end
if !isnothing(opt_power_meas)
    # Built once and repeated as an `@info` after the archive line: this is the
    # comparison the script exists for, and here it is buried in the results block.
    global power_summary = @sprintf("Optimizer predicted %.0f W of mean reel-out \
                                     power%s; the run measured %.0f W (%.2f x).",
        opt_power_pred_eff,
        length(pred_shares) > 1 ?
            @sprintf(" (weighted over %d paths: %s)", length(pred_shares),
                     join((@sprintf("%.0f W for %.0f%%", p.power, 100 * p.share)
                           for p in pred_shares), ", ")) : " for this path",
        opt_power_meas, opt_power_meas / opt_power_pred_eff)
    println("  ", power_summary)
end
# A key is omitted rather than written empty when its measurement does not exist:
# `string(nothing)` would put the bare word `nothing` into the file, which YAML
# reads back as a string.
power_block = OrderedDict{String, Any}(
    "predicted_W" => (round(Int, opt_power_pred_eff),
        "predicted mean reel-out power of the paths actually flown, weighted by \
         their share of the reeling window [W]"),
    "predicted_initial_W" => (round(Int, opt_power_pred),
        "predicted mean reel-out power of the path installed before the run [W]"))
if length(pred_shares) > 1
    power_block["predicted_paths"] = OrderedDict(
        @sprintf("t_%05.1f_s", p.from_s) =>
            (round(Int, p.power), @sprintf("%.0f%% of the reeling window", 100 * p.share))
        for p in pred_shares)
end
if !isnothing(opt_power_meas)
    power_block["measured_W"] = (round(Int, opt_power_meas),
        "mean reel-out power the run harvested [W]")
    power_block["ratio"] = (round(opt_power_meas / opt_power_pred_eff; digits = 2),
        "measured / predicted, against the weighted prediction")
end
feasibility_block = OrderedDict{String, Any}(
    "min_required" => (tos.min_feasibility_margin,
        "min_feasibility_margin of data/traj_opt.yaml [-]"))
if !isnothing(coeffs)
    feasibility_block["margin_start"] = (round(feas_start.margin; digits = 2),
        "curvature margin at the starting length; the worst case only when one \
         path is flown throughout — see the docstring [-]")
    feasibility_block["margin_end"] = (round(feas_end.margin; digits = 2),
        "curvature margin at reelout_l_max [-]")
end
summary["traj_opt"] = OrderedDict{String, Any}(
    "power" => power_block,
    "guess" => OrderedDict(
        "a_deg" => (tos.guess_a, "width of the guess lemniscate; azimuth spans ±a [deg]"),
        "b_deg" => (tos.guess_b, "height of the guess, peak to peak [deg]"),
        "el_center_deg" => (tos.guess_el_center, "centre elevation of the guess [deg]"),
        "points" => (tos.guess_points, "points the guess was sent with")),
    "path" => OrderedDict(
        "points" => (n_path_initial, "points of the path installed before the run"),
        "points_final" => (length(fec.az_path),
            "points of the path at the end; differs when reopt installed one"),
        "az_center_deg" => (round(az_c_path; digits = 1), "centre azimuth of the path [deg]"),
        "az_amplitude_deg" => (round(az_amp_path; digits = 1), "half-width of the path [deg]"),
        "el_center_deg" => (round(el_c_path; digits = 1), "centre elevation of the path [deg]"),
        "el_height_deg" => (round(el_height_path; digits = 1),
            "elevation span of the path, peak to peak [deg]"),
        "downloops" => (opt_downloops, "traversal direction the optimizer solved for")),
    "feasibility" => feasibility_block,
    "reopt" => OrderedDict(
        "enabled" => (tos.reopt_enabled, "re-optimization during the run"),
        "requests" => (reopt_n, "solves that completed, accepted or rejected"),
        "blocking" => (tos.reopt_blocking,
            "simulation held while a solve ran"),
        "blocked_s" => (round(reopt_blocked_s; digits = 1),
            "wall time the simulation was frozen waiting for replies [s]"),
        "installed" => (count(e -> e.status == "installed", reopt_events),
            "new paths actually flown"),
        "events" => OrderedDict(
            @sprintf("t_%05.1f_s", e.t) =>
                (string(e.status, isempty(e.detail) ? "" : " — " * e.detail),
                 @sprintf("at L = %.0f m", e.l))
            for e in reopt_events)),
    "clearance" => OrderedDict(
        "path_min_m" => (round(path_min_h_start; digits = 1),
            "lowest point of the PRE-FLIGHT path at the starting length [m]"),
        "flown_min_m" => (round(minimum(first(lt) * sin(el)
                                        for (lt, el) in zip(sl.l_tether, sl.elevation));
                                digits = 1),
            "lowest point the kite actually reached over the run [m]"),
        "min_required_m" => (tos.min_height, "min_height of data/traj_opt.yaml [m]")),
    "inflow" => OrderedDict(
        "wind_speed_m_s" => (inflow.wind_speed, "wind speed at 6 m sent to the optimizer [m/s]"),
        "wind_direction_deg" => (inflow.wind_direction, "direction the wind comes from [deg]"),
        "profile_law" => (inflow.profile_law, "0=CONST, 1=EXP, 2=LOG, 3=EXPLOG, 4-6=CUSTOM_*")),
    "winch" => OrderedDict(
        "k_v" => (winch.k_v, "v_set = k_v * sqrt(force) sent to the optimizer [-]"),
        "f_min_N" => (winch.f_min, "minimum winch force sent [N]"),
        "f_max_N" => (winch.f_max, "maximum winch force sent [N]")))

# Speed of the SIMULATED time against the wall clock; > 1 is faster than realtime.
if t_sim > 0
    steps = round(Int, t_sim / s.dt)
    # Rates exclude the frozen time, as in the summary; the raw wall time is still
    # shown, and the gap between the two is `reopt_blocked_s`.
    t_run = max(t_wall - reopt_blocked_s, eps())
    @printf("  Performance: %.1f s sim in %.1f s wall%s = %.2f x realtime \
             (%.1f ms/step over %d steps at dt = %.4f s, vsm_interval = %d)\n",
            t_sim, t_wall,
            reopt_blocked_s > 0 ? @sprintf(" (%.1f s held for re-optimization)",
                                           reopt_blocked_s) : "",
            t_sim / t_run, 1000 * t_run / steps,
            steps, s.dt, fcs.vsm_interval)
    summary["performance"] = OrderedDict(
        "sim_time_s" => (round(t_sim; digits = 1), "simulated time [s]"),
        "wall_time_s" => (round(t_wall; digits = 1), "wall-clock time [s]"),
        "realtime_factor" => (round(t_sim / max(t_wall - reopt_blocked_s, eps()); digits = 2),
            "sim_time / wall_time, excluding time frozen for re-optimization"),
        "ms_per_step" => (round(1000 * (t_wall - reopt_blocked_s) / steps; digits = 1),
            "wall time per step, excluding time frozen for re-optimization [ms]"),
        "steps" => (steps, "step count"),
        "dt_s" => (s.dt, "simulation timestep [s]"),
        "vsm_interval" => (fcs.vsm_interval, "VSM aerodynamic update interval [steps]"))
else
    @warn "No simulated time elapsed — no performance figure."
end

open(joinpath(output_path, log_name * ".yaml"), "w") do io
    println(io, "# Run summary for output/", log_name,
            ".arrow, written by examples/simple_reelout.jl at the end of the run.")
    write_yaml_commented(io, 0, summary)
end

# ==================== ARCHIVE ==================== #

# Repeated here, as the last thing the run says: the results block above is a wall
# of printf and this is the comparison the script exists for. `@info`, not `printf`,
# so it lands on the same stream as the archive line it precedes.
isnothing(opt_power_meas) || @info power_summary

# One timestamped folder per run under output/archives/, so the exact config
# that produced a log survives even after the next run overwrites output/*.
# RUN_ARCHIVE = false skips it, read and cleared like SHOW_PLOTS: a sweep writes
# one folder per grid point otherwise, each with a copy of the 40 MB arrow log,
# and its own results table already records what distinguished the runs.
run_archive = @isdefined(RUN_ARCHIVE) ? RUN_ARCHIVE : true
RUN_ARCHIVE = true
if run_archive
    archive_dir = joinpath(output_path, "archives",
                           Dates.format(run_time, "yyyy-mm-dd_HHMMSS"))
    mkpath(archive_dir)
    input_yaml_files = [
        project,                                              # system project
        joinpath(dirname(project), project_set.sim_settings), # plant/solver settings
        joinpath(skc_data_path(), wc_settings(project)),      # winch gains
        joinpath(skc_data_path(), fc_settings(project)),      # flight-controller tuning
        joinpath(skc_data_path(), "gui.yaml"),                # project/sim_time/turbulence choice
        # Without this the archive cannot reproduce its own run: the guess decides
        # WHICH optimum the solve converges to, and reopt_*/min_feasibility_margin
        # decide what is re-anchored and what is flown.
        joinpath(skc_data_path(), "traj_opt.yaml"),           # optimizer guess and knobs
    ]
    output_files = [
        joinpath(output_path, log_name * ".arrow"),
        joinpath(output_path, log_name * ".yaml"),
    ]
    for f in unique(vcat(input_yaml_files, output_files))
        isfile(f) && cp(f, joinpath(archive_dir, basename(f)); force = true)
    end
    @info "Archived run inputs and outputs to $archive_dir"
else
    @info "Archiving suppressed by RUN_ARCHIVE = false; it is back to true for the next run."
end

if show_plots
    # The flown curve and this run's log name, so the plots draw the optimized
    # path and load the _opt log instead of the lemniscate run's.
    REF_PATH = (fec.az_path, fec.el_path)
    LOG_NAME = log_name
    include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
else
    @info "Plots suppressed by SHOW_PLOTS = false; it is back to true for the next run."
end

nothing
