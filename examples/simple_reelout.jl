# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Figure-of-eight path following of the V3 kite, extended with a `REEL_OUT` winch: the
tether starts at `l_tether` (150 m by default), flies the same four-phase entry as
`simple_fig8.jl` (park -> dive -> hold -> transition), and from the moment the
guidance engages (phase 3) reels out under WinchControllers.jl's
`v_set = kv * sqrt(force)` law until `fcs.reelout.reelout_l_max` or `fcs.reelout.n_fig_eight`
figures of eight, whichever comes first, then holds that length for the rest of
the run. Once reel-out stops, a fifth phase (final) takes over:
the pattern keeps flying, but depower switches from `depower_setpoint` to
`depower_final`, meant to hold roughly the force reel-out was regulating away.
No pumping cycle: there is no reel-in phase here.

# What comes from where

The guidance and its settings are unchanged from `simple_fig8.jl` — see that
script's docstring for the guidance, the entry state machine and the plant/data
path conventions, all of which apply here too. What differs is the winch: instead
of V3Kite's own FORCE/POSITION winch (`fcs.winch.compliance`), this script layers
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
`fcs.winch.compliance` must be `0` (POSITION mode): `REEL_OUT` and V3Kite's own FORCE mode
both drive `set_length`/`set_torque`, and only one winch can hold the drum at a
time — this script errors at startup otherwise, rather than silently picking
one. That is why `fcs` here is loaded from `data/fc_settings_reelout.yaml`, a
copy of `simple_fig8.jl`'s `fc_settings.yaml` with `compliance: 0` and the
`REEL_OUT` keys below, not the shared file (whose `compliance: 0.5` is fig8's
FORCE-mode tuning) — `system_reelout_maasvlakte.yaml`'s `fc_settings:` key names it.

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
kite's minimum angular turn radius) and `fcs.reelout.reelout_l_max` (the end). Measured
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
| `var_11` | speed setpoint `v_set` [m/s] — `REEL_OUT`'s law from phase 3, or the standalone force-floor guard's reel-IN before that |
| `var_12` | WinchController state (0 lower-force, 1 speed, 2 upper-force) |
| `var_13` | force error of the active force limiter [N], NaN in speed control |

`var_12`/`var_13` only mean anything from phase 3 on, when `rc` (the full
`WinchController`) is actually stepped. Before that, `l_set` is otherwise held
at the settled length, but the dive can sag tether force well below `entry_f_min`
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
`fcs.reelout.reelout_l_max` OR `fcs.reelout.n_fig_eight` laps have been flown, whichever fires
first. Reel-out begins `fcs.reelout.reelout_delay` seconds after phase 3 is reached and
stops at whichever of the two triggers phase 5. Phase 4 still marks the first
close tracking of the pattern, it just no longer gates the winch, and does not
gate phase 5 either — a run that never settles still reaches final once
reel-out is done.

# Parameters

`fcs` works exactly as in `simple_fig8.jl`: rebuilt from `data/fc_settings_reelout.yaml`
unconditionally on every `include`, so a value is overridden by editing that file,
not by pre-defining or mutating `fcs` in the REPL. The one exception is the input
`fcs_overrides`, a `Dict{Symbol, Any}` of field => value applied on top of the file,
for example one `f8_a`/`f8_b` pair per run: `run_example("simple_reelout.jl";
fcs_overrides, output_path, run_archive = false, show_plots = false)`
(`src/script_inputs.jl`). `output_path` and `run_archive` let parallel runs keep
their logs apart and skip the per-run archive. The inputs hold for that one run: a plain `include` flies
with the defaults, so a value from a sweep never changes an interactive run. The
`REEL_OUT`-specific fields are `reelout_l_max`, the stop length, `n_fig_eight`,
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
reel-out one: `selected_reelout_project()` (`src/gui_state.jl`) ignores a
fig8 selection left over from `simple_fig8.jl` — whose FORCE-mode `fc_settings`
this script rejects at startup — and flies `system_reelout_maasvlakte.yaml` instead, so
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
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
include(joinpath(@__DIR__, "model_setup.jl"))
# The caller's inputs, `run_example("simple_reelout.jl"; show_plots = false, ...)`, see the
# docstring; a plain `include` flies with these defaults.
(; show_plots, fcs_overrides, output_path, run_archive) =
    script_inputs(@__FILE__, (; show_plots = true, fcs_overrides = Dict{Symbol, Any}(),
                               output_path = nothing, run_archive = true))
# Cleared for the same reason: a lemniscate run must not plot the optimized
# reference, or load the `_opt` log, that a simple_opt_reelout.jl run left behind.
REF_PATH = nothing
LOG_NAME = nothing
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Structural damping of the tether and bridle segments, as a ratio of their
# stiffness: unit_damping = ratio * unit_stiffness [s]. See simple_fig8.jl's docstring.
DAMPING_PER_STIFFNESS = 0.002
PROJECT = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
WIND_SPEED = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
@info "simple_reelout.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE, wind_speed = $(isnothing(WIND_SPEED) ? "default" : "$WIND_SPEED m/s")."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))

# Per-run overrides of the settings just loaded, for a SWEEP (the input `fcs_overrides`).
apply_overrides!(fcs, fcs_overrides, "fcs_overrides", "FC_Settings", "fcs")

project_set = Settings(project)
apply_windspeed_override!(project_set, WIND_SPEED)
l_tether = project_set.l_tether

# Log files are arrow files, named after the project's `log_file`, kept out of git.
# The input `output_path` redirects them, so that parallel runs of this script (the sweep)
# cannot overwrite each other's log, summary and archive; `nothing` is the default output/.
output_path = something(output_path, normpath(joinpath(@__DIR__, "..", "output")))
mkpath(output_path)
log_name = basename(project_set.log_file)

# ======================== INIT =========================== #

fcs.winch.compliance >= 0 ||
    error("compliance must be >= 0, got $(fcs.winch.compliance)")
fcs.winch.compliance == 0 ||
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

# `init_model` (model_setup.jl): sim_time falls back to the project's own value when
# SIM_TIME is `nothing` (the `default` choice); the wind speed comes from the project
# file unless WIND_SPEED overrides it above.
s = init_model(project, project_set, fcs, wpc, SIM_TIME; turbulence = TURBULENCE,
    aero_mode = AERO_MODE, damping_per_stiffness = DAMPING_PER_STIFFNESS, pad_final_time = false)

# The controllers, built on the settled model (`build_controllers`, src/winch_setup.jl):
# the REEL_OUT controller `rc`, built fresh here so its soft-start ramp (t_startup)
# begins the moment reel-out actually starts (phase 3), not at t = 0; the standalone
# force-floor guard `guard_lfc` for phases 0-2 — deliberately NOT `rc`, whose own
# SpeedController would wind up while its output is ignored (MEASURED: v_reelout
# spiking to +8 m/s instead of the intended reel-IN); the length setpoint `l_set`,
# the settled length, growing from phase 3 until it reaches `reelout_l_max`; and `fec`.
# `rcs` is `wc`, the one `WCSettings` of both winches; `rcs.dt` becomes the plant's.
(; rc, guard_lfc, l_set, fec) = build_controllers(fcs, rcs, s)

# Never hardcode these: both arguments move them a lot. The lookup key is
# `body_damping`, the value `init` was given: the damping the model FLIES the
# pattern with is the floor that decays out of it (0.8x by default), so the one
# value identifies both the settling transient and the flown damping c1 belongs
# to.
#
# The coefficients are DIAGNOSTIC here — they feed the feasibility check and the
# dead-time context below, no gain and no control law — so a damping/depower the
# table cannot serve costs the diagnosis, not the run, so `try_turn_rate_coeffs`
# flies on unadvised rather than aborting a deliberate off-grid run.
coeffs = try_turn_rate_coeffs(fcs)

if isnothing(coeffs)
    c1 = c2 = delay = NaN
else
    (; c1, c2, delay) = coeffs

    # c1 must match the damping in use; that is what makes this check meaningful.
    # At l_tether (the START, before any reel-out) this is the WORST case: a longer
    # tether only ever shrinks the kite's minimum angular turn radius.
    feas_start = check_pattern_feasible(fec, l_tether, fcs.course.max_steering; c1)
    feas_start.feasible ||
        @warn "Pattern is tighter than the kite's minimum turn radius AT THE START \
               (l_tether = $l_tether m) — expect curvature-limited tracking during \
               the entry, not a tuning problem."
    feas_end = check_pattern_feasible(fec, fcs.reelout.reelout_l_max, fcs.course.max_steering; c1)

    # Dead-time context for attractor_dist: how long the lead arc takes to fly.
    lead_time = deg2rad(fcs.pattern.attractor_dist) * l_tether / fcs.course.v_app_ref
    @info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   fcs.pattern.attractor_dist, lead_time, fcs.course.v_app_ref, delay, lead_time / delay)
end

cc = CourseController(CourseControllerSettings(fcs; dt = s.dt))

transition_start = NaN            # [s] time phase 3 began; `reelout_delay` counts from it
stop_start = NaN            # [s] time the soft-stop deceleration latched; NaN = not yet
stop_v_entry = NaN          # [m/s] v_set at the moment it latched
stop_T = NaN                # [s] duration of the linear decel to reach 0 at reelout_l_max
reelout_done = false        # true once either stop criterion has ended reel-out
stop_reason = ""            # "length", "laps", or "" if reel-out never stopped
final_start = NaN           # [s] time phase 5 began; the run ends `fcs.reelout.final_time` after it
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
        t - final_start >= fcs.reelout.final_time && break

        # L0 attractor guidance -> commanded course [rad]. The lead is a flight
        # TIME when attractor_lead_time is set, so it is re-read every step.
        fec.fes.attractor_distance = attractor_distance(fcs, Float64(s.sys_state.v_app),
                                                        Float64(s.sys_state.l_tether[1]))
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
            dmin, tangent = path_tangent(fec),
            park = SimpleKiteControllers.park_should_start(fcs, cc.phase,
                       Float64(s.sys_state.v_reelout[1]), l_set))
        phase_before == 2 && phase == 3 && (global transition_start = t)
        # Separate from the ladder inside calc_steering so it can fire the SAME
        # step as a 3->4 transition: reel-out finishing does not wait for settling.
        if phase in (3, 4) && reelout_done
            set_phase!(cc, 5)
            phase = 5
            isnan(final_start) && (global final_start = t)
            rel_depower = fcs.reelout.depower_final
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
        if phase >= 3 && t - transition_start >= fcs.reelout.reelout_delay &&
           !reelout_done
            # The INSTANTANEOUS force: reeling out faster exactly when the kite
            # pulls harder is what regulates the force. Lagging it is closed, see
            # docs/fig8_tuning_log.md.
            v_raw = calc_v_set(rc, reel_out_speed(s), winch_force(s), rcs.f_low)
            # Ramps the COMMAND, not the law: rc's internal state (integrators,
            # force limiters) sees the true v_raw throughout, only the value
            # handed to l_set/v_ff is scaled. `t_startup` does not do this — see
            # its docstring in `src/fc_settings.jl`.
            ramp = fcs.reelout.reelout_softstart > 0 ?
                clamp((t - transition_start - fcs.reelout.reelout_delay) / fcs.reelout.reelout_softstart,
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

            remaining = fcs.reelout.reelout_l_max - l_set
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
            if isnan(stop_start) && fcs.reelout.reelout_softstop > 0 && v_cmd > 0 &&
               remaining <= v_cmd * fcs.reelout.reelout_softstop
                global stop_start = t
                global stop_v_entry = v_cmd
                global stop_T = 2 * remaining / v_cmd
            end
            # Second stop criterion: N COMPLETE laps since the counter started at phase 4.
            # `fig8_idx_progress`, not `fig8_n`, which reads 1 during the first lap.
            if isnan(stop_start) && fcs.reelout.n_fig_eight > 0 &&
               fig8_idx_progress >= fcs.reelout.n_fig_eight * n_path
                global stop_reason = "laps"
                if fcs.reelout.reelout_softstop > 0 && v_cmd > 0
                    global stop_start = t
                    global stop_v_entry = v_cmd
                    # No remaining distance to solve T from — unlike the reelout_l_max
                    # latch, the length is the free variable here. Same nominal duration.
                    global stop_T = 2 * fcs.reelout.reelout_softstop
                else
                    global reelout_done = true   # hard stop, as reelout_l_max does today
                end
            end
            v_set = isnan(stop_start) ? v_cmd :
                stop_v_entry * (1 - clamp((t - stop_start) / stop_T, 0.0, 1.0))
            global l_set = min(l_set + v_set * s.dt, fcs.reelout.reelout_l_max)
            on_timer(rc)
            if l_set >= fcs.reelout.reelout_l_max
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
            set_f_set(guard_lfc, fcs.winch.entry_f_min)
            set_v_sw(guard_lfc, calc_vro(rcs, fcs.winch.entry_f_min) * 1.05)
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
        step!(s; rel_depower, rel_steering, vsm_interval = fcs.run.vsm_interval,
              set_torque = winch_torque!(wpc, s, l_set; v_ff = v_set,
                                         speed_limit = rcs.v_sat,
                                         acceleration_limit = rcs.max_acc))

        # Report the overspeed rather than the opaque solver abort it causes later.
        if Float64(s.sys_state.v_app) > fcs.run.v_app_abort
            @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                             Stopping before the solver diverges.",
                            s.sys_state.time, s.sys_state.v_app, fcs.run.v_app_abort,
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
        s.sys_state.var_04 = fcs.pattern.el_center     # pattern-centre elevation [deg]
        s.sys_state.var_05 = chi_set           # RAW guidance course [rad]
        s.sys_state.var_06 = rad2deg(err)      # REGULATED error [deg]
        # A weight, not a flag: a step here means entry_d_blend is too narrow.
        s.sys_state.var_07 = abs(chi_set) > deg2rad(fcs.course.entry_chi_max) ? w_lim : 0.0
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

# `write_yaml_commented` is the package's.

syslog = load_log(log_name; path = output_path)
sl = syslog.syslog
# The geometry is passed in too: without it the criteria are blind to pattern SIZE.
# require_final: this script's own phase 5, unlike simple_fig8.jl's sys_state
# (which never goes past 4) — checks reel-out actually finished within the run.
fig8m = print_fig8_metrics(sl; t_start = fcs.course.park_time, settle_time = fcs.run.entry_time,
                   min_elevation = fcs.run.min_elevation, az_center = 0.0,
                   az_amplitude = fcs.pattern.f8_a, el_height = fcs.pattern.f8_b,
                   min_span_frac = fcs.run.min_span_frac, require_final = true,
                   max_force = project_set.max_force,
                   # A parking kite leaves the path on purpose: the pattern ends where it parks.
                   t_end = something(SimpleKiteControllers.park_start_time(fcs, sl), Inf))

summary = OrderedDict{String, Any}()
# The FIRST key of the file, the same words the console logs as "Success criteria:
# …". It is the one line a reader looks for, so it is not buried at the end of
# `fig8_metrics:` — that section keeps the numbers the verdict was computed from.
# A failure names the criteria that broke, exactly as the console does. The nested
# key stays as well here.
summary["success_criteria"] = success_verdict(fig8m)
run_time = Dates.now()
# The sections this script shares with simple_opt_reelout.jl are the package's.
summary["simulation"] = simulation_block(basename(@__FILE__), PROJECT, TURBULENCE,
                                         project_set.v_wind, run_time)
fig8m === nothing ||
    (summary["fig8_metrics"] = fig8_metrics_block(fig8m, lap_durations(sl); nested_verdict = true))
# The window's mean and peak force and power too.
reelout_sections = reelout_block(sl, fcs, l_tether; stop_reason,
                                 laps_reeled = fig8_idx_progress / n_path, window_means = true,
                                 t_end = something(SimpleKiteControllers.park_start_time(fcs, sl), Inf))
rp = reelout_sections.rp
summary["reelout"] = reelout_sections.block
performance_section = performance_block(t_sim, t_wall, s.dt, fcs.run.vsm_interval)
isnothing(performance_section) || (summary["performance"] = performance_section)

# A recap of the numbers scattered above; last key, so `success_criteria` stays
# the first thing a reader sees. No optimizer here, so no power_ratio or
# optimization_requests/installed — those only exist in reelout_results.jl.
summary_block = OrderedDict{String, Any}(
    "date" => (Dates.format(run_time, "yyyy-mm-dd"), "wall-clock date the run finished"),
    "time" => (Dates.format(run_time, "HH:MM:SS"), "wall-clock time the run finished"),
    "wind_speed_m_s" => (project_set.v_wind, "mean wind speed passed to init [m/s]"))
isnothing(rp) || (summary_block["mean_power_W"] =
    (round(Int, rp.mean_power), "mean reel-out power over the reeling window [W]"))
t_sim > 0 && (summary_block["realtime_factor"] =
    (round(t_sim / t_wall; digits = 2), "sim_time / wall_time"))
summary["summary"] = summary_block

open(joinpath(output_path, log_name * ".yaml"), "w") do io
    println(io, "# Run summary for output/", log_name,
            ".arrow, written by examples/simple_reelout.jl at the end of the run.")
    write_yaml_commented(io, 0, summary)
end

# ==================== ARCHIVE ==================== #

# One timestamped folder per run under output/archives/, so the exact config
# that produced a log survives even after the next run overwrites output/*.
# The input `run_archive = false` skips it: a sweep writes
# one folder per grid point otherwise, each with a copy of the 40 MB arrow log,
# and its own results table already records what distinguished the runs.
if run_archive
    archive_run_files(output_path, run_time, run_input_files(project, project_set),
                      [joinpath(output_path, log_name * ".arrow"),
                       joinpath(output_path, log_name * ".yaml")])
else
    @info "Archiving suppressed by run_archive = false."
end

if show_plots
    include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
else
    @info "Plots suppressed by show_plots = false."
end

nothing
