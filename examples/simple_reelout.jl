# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Figure-of-eight path following of the V3 kite, extended with a REEL_OUT winch: the
tether starts at `l_tether` (150 m by default), flies the same four-phase entry as
`simple_fig8.jl` (park -> dive -> hold -> transition), and from the moment the
guidance engages (phase 3) reels out under WinchControllers.jl's
`v_set = kv * sqrt(force)` law until `fcs.reelout_l_max` or `fcs.n_fig_eight`
figures of eight, whichever comes first, then holds that length for the rest of
the run. Once reel-out stops, a fifth phase (final) takes over:
the pattern keeps flying, but depower switches from `depower_setpoint` to
`depower_final`, meant to hold roughly the force reel-out was regulating away.
No pumping cycle: there is no reel-in phase here.

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

**Two winch controllers, ONE settings file.** Both read the same `WCSettings`
object (`wc`, and `rcs` which is the same object), loaded from the file the
project's `wc_settings:` key names. V3Kite's torque loop takes the `winch_*`
fields; WinchControllers.jl's `WinchController` takes `kv`/`f_low`/`f_high`/
`v_sat`/`t_startup` and the force-limiter gains. Before the 2026-08-16 merge
(PlanWinchcontrol.md) these were two structs of nearly the same name in two
packages, needing two files in two schemas — and the second, `wc_settings_reelout.yaml`,
had to be loaded by HARDCODED NAME because the project key was already claimed
by the first, so switching projects could not switch the reel-out winch tuning.
It can now.

# Feasibility

`check_pattern_feasible` is printed at both `l_tether` (the START of the run,
before any reel-out — the worst case, since a longer tether only ever shrinks the
kite's minimum angular turn radius) and `fcs.reelout_l_max` (the end). Measured
with `fc_settings_reelout.yaml`'s defaults at 150 m the margin is 1.22, growing
to 1.63 at 200 m and 2.04 at 250 m, so no pattern change is needed to start at
150 m — see `Plan.md`.

Logs the run to `output/<log_file>.arrow` and `include`s `simple_reelout_plots.jl`
at the end, exactly like `simple_fig8.jl`. The printed RESULTS summary is also
written to `output/<log_file>.yaml`, structured the same way and with each value
commented, for later comparison across runs without re-parsing console output.

Log slot mapping (`step!` already fills `var_14`/`var_15`/`var_16`; `var_01`
through `var_09` are as in `simple_fig8.jl`):

| slot     | quantity                                          |
|:---------|:---------------------------------------------------|
| `var_10` | tether length setpoint `l_set` [m]                |
| `var_11` | speed setpoint `v_set` [m/s] — REEL_OUT's law from phase 3, or the standalone force-floor guard's reel-IN before that |
| `var_12` | WinchController state (0 lower-force, 1 speed, 2 upper-force) |
| `var_13` | force error of the active force limiter [N], NaN in speed control |

`var_12`/`var_13` only mean anything from phase 3 on, when `rc` (the full
`WinchController`) is actually stepped. Before that, `l_set` is otherwise held
at the settled length, but the dive can sag tether force well below `rcs.f_low`
(measured: ~50 N) — a SEPARATE, standalone `LowerForceController` (`guard_lfc`,
deliberately not `rc`, see its construction comment) monitors force throughout
phases 0-2 and reels in when needed, logged into `var_10`/`var_11` same as the
main reel-out law.

Not a `var_XX` slot: `fig_8`, `SysState`'s live lap count, as in `simple_fig8.jl`
but gated on `phase >= 4` rather than `== 4` — a fast reel-out can jump straight
from phase 3 to 5 without ever touching 4, and the counter must still start.
`cycle` (the pumping-cycle number) is left at its default: no reel-in phase here.

`sys_state` carries the same entry state machine as `simple_fig8.jl` (0 park,
1 dive, 2 hold, 3 transition, 4 fig8), plus a fifth phase this script adds: 5 final,
entered from either 3 or 4 the moment reel-out stops, i.e. `l_set` reaches
`fcs.reelout_l_max` OR `fcs.n_fig_eight` laps have been flown, whichever fires
first. Reel-out begins `fcs.reelout_delay` seconds after phase 3 is reached and
stops at whichever of the two triggers phase 5. Phase 4 still marks the first
close tracking of the pattern, it just no longer gates the winch, and does not
gate phase 5 either — a run that never settles still reaches final once
reel-out is done.

# Parameters

`fcs` works exactly as in `simple_fig8.jl`: rebuilt from `data/fc_settings_reelout.yaml`
unconditionally on every `include`, so a value is overridden by editing that file,
not by pre-defining or mutating `fcs` in the REPL. The one exception is
`FCS_OVERRIDES`, a `Dict{Symbol, Any}` of field => value applied on top of the file
and then CLEARED, which is how the shape sweep (`examples/optimize_fig8.jl`) flies
one `f8_a`/`f8_b` pair per `include`; `OUTPUT_PATH` and `RUN_ARCHIVE` are read and
cleared the same way, and let those parallel runs keep their logs apart and skip
the per-run archive. All three are `SHOW_PLOTS`'s convention, for its reason: a
value left over from a sweep must never change an interactive run. The
REEL_OUT-specific fields are `reelout_l_max`, the stop length, `n_fig_eight`,
the second, independent stop criterion counted in laps (`0` disables it, its
own docstring in `src/fc_settings.jl` has the counting details), `reelout_delay`,
how long after phase 3 the winch waits before it starts reeling out, and
`reelout_softstart`, which ramps the COMMANDED speed (`v_ff` and the `l_set`
integration together, not the law inside `WinchController`) linearly from 0 to
the computed `v_set` over that many seconds — `rcs.t_startup`
(`data/wc_settings.yaml`) does not do this, see the comment on that key.
`reelout_softstop` is the OTHER end of reel-out: once the remaining distance
would finish within that many seconds at the current rate, `v_set` decelerates
LINEARLY to 0 at `reelout_l_max` instead of stepping there, continuous with the
speed already being flown — avoiding the reel-in transient (a power undershoot)
a hard stop leaves the POSITION loop to correct. The same soft-stop ramp applies
when `n_fig_eight` fires first: with no remaining distance to solve a duration
from, it latches for exactly `2 * reelout_softstop` seconds instead, the same
nominal duration as the length case.
`depower_final` is flown once phase 5 (final) is reached; its own docstring in
`src/fc_settings.jl` has the tuned value and where it came from.

WinchControllers.jl's OWN tuning — `kv`, `f_low`, `f_high`, `v_sat`,
`t_startup`, `mode`, and every LowerForceController/UpperForceController gain
that used to be invisible package defaults — is `data/wc_settings.yaml`
now, not here; see that file's header comment.

The run length and the turbulence level are read from `data/gui.yaml` exactly as
in `simple_fig8.jl`. The system project is too, but only when the selection is a
reel-out one: `selected_reelout_project()` (`examples/gui_state.jl`) ignores a
fig8 selection left over from `simple_fig8.jl` — whose FORCE-mode `fc_settings`
this script rejects at startup — and flies `system_reelout_150m.yaml` instead, so
no `select_project()` call is needed in between. Selecting another
`system_reelout_*.yaml` still switches this script to it (a shorter tether means
less margin to cover, see Feasibility above).
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
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

@info "simple_reelout.jl: figure-of-eight path following with REEL_OUT of the tether."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
# Read and cleared HERE, so a `SHOW_PLOTS = false` never survives into the next run.
show_plots = @isdefined(SHOW_PLOTS) ? SHOW_PLOTS : true
SHOW_PLOTS = true
# Cleared for the same reason: a lemniscate run must not plot the optimized
# reference, or load the `_opt` log, that a simple_opt_reelout.jl run left behind.
REF_PATH = nothing
LOG_NAME = nothing
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Structural damping of the tether and bridle segments, as a ratio of their
# stiffness: unit_damping = ratio * unit_stiffness [s]. See simple_fig8.jl's docstring.
DAMPING_PER_STIFFNESS = 0.001
PROJECT = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
WIND_SPEED = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
@info "simple_reelout.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE, wind_speed = $(isnothing(WIND_SPEED) ? "default" : "$WIND_SPEED m/s")."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))

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
apply_windspeed_override!(project_set, WIND_SPEED)
l_tether = project_set.l_tether

# Log files are arrow files, named after the project's `log_file`, kept out of git.
# OUTPUT_PATH redirects them, so that parallel runs of this script (the sweep)
# cannot overwrite each other's log, summary and archive. Read and cleared like
# SHOW_PLOTS; `nothing` is the default output/.
output_path = (@isdefined(OUTPUT_PATH) && !isnothing(OUTPUT_PATH)) ? OUTPUT_PATH :
              normpath(joinpath(@__DIR__, "..", "output"))
OUTPUT_PATH = nothing
mkpath(output_path)
log_name = basename(project_set.log_file)

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
# The wind speed comes from the same file unless WIND_SPEED overrides it above: it is
# a plant condition, and project_set.v_wind keeps the mean wind and the turbulent
# field (which init builds for it) at the same speed.
# No cache_path either: V3Kite's default is where its own precompile workload
# compiled the model, and a different model binary costs 40 s of re-JIT in init.
s = init(project_set.v_wind, l_tether; body_start_damping = fcs.body_damping,
    body_sim_damping = 0.8 .* fcs.body_damping,
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

cc = CourseController(CourseControllerSettings(fcs; dt = s.dt))

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
                # A step moves Q by a fraction of a point; a jump is Q changing branch.
                abs(delta) > n_path ÷ 8 && (delta = 0)
                global fig8_idx_progress += delta
                global fig8_idx_prev = fec.last_idx
                global fig8_n = 1 + floor(Int, fig8_idx_progress / n_path)
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
            # ...but the soft-start must not override the tether's own protection:
            # at 8 m/s ground wind the entry swoop drives the force to 10.7 kN
            # (41 % over f_high) while the UpperForceController sits pinned at
            # v_sat = 8 m/s and this ramp, still only 0.21 at t = 28.5 s, hands
            # the drum 1.7 m/s of it. An OPEN-LOOP timer beating a CLOSED-LOOP
            # force limiter. Release the ramp in proportion to tether load
            # instead: inert below f_low (so the engagement transient the ramp
            # exists for is unchanged at 5-6 m/s, where the limiter never fires
            # during entry), fully bypassed at f_high. Continuous in the force,
            # so there is no jump when the limiter latches.
            force_release = clamp((winch_force(s) - rcs.f_low) /
                                  (rcs.f_high - rcs.f_low), 0.0, 1.0)
            v_cmd = max(ramp, force_release) * v_raw

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
        s.sys_state.var_04 = fcs.el_center     # pattern-centre elevation [deg]
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
fig8m = print_fig8_metrics(sl; t_start = fcs.park_time, settle_time = fcs.entry_time,
                   min_elevation = fcs.min_elevation, az_center = 0.0,
                   az_amplitude = fcs.f8_a, el_height = fcs.f8_b,
                   min_span_frac = fcs.min_span_frac, require_final = true)

summary = OrderedDict{String, Any}()
# The FIRST key of the file, the same words the console logs as "Success criteria:
# …". It is the one line a reader looks for, so it is not buried at the end of
# `fig8_metrics:` — that section keeps the numbers the verdict was computed from.
# A failure names the criteria that broke, exactly as the console does. The nested
# key stays as well here; see the comment on it below.
summary["success_criteria"] = (fig8m === nothing ? "not scored — no settled samples" :
    isempty(fig8m.criteria_failed) ? "all $(fig8m.criteria) passed" :
    "FAILED: " * join(fig8m.criteria_failed, ", "),
    "pass/fail verdict vs V3Kite's success criteria")
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
    "wind_speed" => (project_set.v_wind, "mean wind speed in m/s passed to init"),
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
        # Deliberately the same string as the top-level verdict above, which is
        # where a reader should look. This copy stays because `run_metrics` reads
        # the sweep's runs from here — a shape sweep flies THIS script — and
        # dropping it would put `success_criteria: null` in every row of
        # output/optimization_results.yaml.
        "success_criteria" => (isempty(fig8m.criteria_failed) ? "all $(fig8m.criteria) passed" :
            "FAILED: " * join(fig8m.criteria_failed, ", "), "same verdict as the top-level key"))
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

# Speed of the SIMULATED time against the wall clock; > 1 is faster than realtime.
if t_sim > 0
    steps = round(Int, t_sim / s.dt)
    @printf("  Performance: %.1f s sim in %.1f s wall = %.2f x realtime \
             (%.1f ms/step over %d steps at dt = %.4f s, vsm_interval = %d)\n",
            t_sim, t_wall, t_sim / t_wall, 1000 * t_wall / steps,
            steps, s.dt, fcs.vsm_interval)
    summary["performance"] = OrderedDict(
        "sim_time_s" => (round(t_sim; digits = 1), "simulated time [s]"),
        "wall_time_s" => (round(t_wall; digits = 1), "wall-clock time [s]"),
        "realtime_factor" => (round(t_sim / t_wall; digits = 2), "sim_time / wall_time"),
        "ms_per_step" => (round(1000 * t_wall / steps; digits = 1), "wall time per step [ms]"),
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
    include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
else
    @info "Plots suppressed by SHOW_PLOTS = false; it is back to true for the next run."
end

nothing
