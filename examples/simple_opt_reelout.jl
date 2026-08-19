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

# Learning the elevation bias

The kite tracks BELOW the path it is given, in every segment of every run measured
so far: 1-2 deg, and 3.5 deg at 380 m. `fcs.el_bias_gain` (`0` = off) closes that
by learning. Once per completed lap the mean signed elevation error is taken, the
correction moves by that fraction of it (clamped to `fcs.el_bias_max`), and every
path installed afterwards is RAISED by the correction — so what is flown converges
on the curve the optimizer solved for, which is also the curve its power number
was computed for.

`fcs.el_offset_final` lifts the path by a further fixed amount once reel-out ends,
which is a setpoint move rather than an error: there is no reel-out power left to
trade for height then, and height is clearance. It cancels out of the learning —
the sag is measured against the path in the air and the update closes on the
optimizer's curve PLUS this offset — so the two compose instead of fighting. The
lift starts at the STOP LATCH, where the winch begins decelerating, not at the
phase 4 -> 5 transition: measured 2026-08-18, the run's lowest point falls between
the two, and at the transition the 4 s blend and the climb after it are still
ahead. With `reelout_softstop` at 0 there is no latch before phase 5 at all, so
`fcs.el_offset_lead` anticipates the end instead — the lift goes in once the length
left is under `v_reelout * el_offset_lead`. Whichever fires first latches it, and
it is never cleared.

State 5 gets its own gain (`fcs.el_bias_gain_final`): the sag deepens once the
winch stops and there is a lap or two left to learn in. A correction only reaches
the kite through an install, and phase 5 usually has none left — `max_reopt` is
spent by then — so a lap whose correction moved installs the DIFFERENCE on the
path in the air through the same blend, gated on the curvature margin at the
current length. A fresh candidate is raised by the whole correction, the path in
the air by what has changed since it was installed; both end up at the optimizer's
curve plus the correction.

`fcs.el_bias_bins` learns the correction as a PROFILE instead of one number: the
sag is deeper at the lobes than at the crossing (~2.3° against ~0.4° at 380 m,
`traj_opt.droop_profile`), so a single number is that profile's mean and lifts the
crossing it does not need to lift. The same per-lap update then runs per azimuth
band, keyed on |azimuth| as a fraction of the path's OWN amplitude so the bands
keep their place on a pattern that shrinks by a third over the run, and what is
applied to a path is [`bias_lift`](@ref) — the bands interpolated with a
smoothstep between their centres, never a staircase, because a corner in the
reference moves the tightest turn of the pattern onto the corner. After every
update the bands are pulled towards each other by `fcs.el_bias_smooth`
([`smooth_bins`](@ref)), which leaves the profile's mean alone and only damps the
differences: a band sees a fifth of the samples the scalar learner had, and the
noise that buys is what the curvature gate refuses. `el_bias_bins = 1` is the
scalar learner exactly.

The error that is fed back is the one against the OPTIMIZER's curve, i.e. the
measured sag plus the correction already in the flown path. Closing on the sag
itself does not converge and is what the first version did (measured 2026-08-18):
the kite drops below whatever path it is given, so raising the reference raises
the kite with it and leaves the sag unchanged, and the correction integrates to
the clamp in 7 laps while the flown curve sails past the optimizer's. The sag
therefore stays in the cross-track error by construction — the correction moves
WHERE the pattern is flown, not how well.

The correction is applied before the three gates, which have to score the curve
that will actually be flown, and it rides on the same blend as the install, so it
never enters as a step. A whole lap is averaged on purpose: the error's SHAPE
cancels there and only its offset survives, and the shape is the part the kite
cannot follow anyway — it is bounded by turn authority at the tight shoulder, not
by the reference. Recorded per lap in `traj_opt.el_bias` of the run summary.

Every lift is paid for on the other axis: a pattern raised is a pattern flown
narrower, and the pattern-SIZE criteria are what breaks first — the 2.5 s
`el_offset_lead` failed the azimuth reach by hundredths of a degree while passing
everything else. `traj_opt.lift_budget` in the summary puts the two halves of that
trade next to each other: the correction the path in the air actually carries and
the elevation it bought, against how much room each size criterion has left, with
`tightest` naming the one the next lift has to spend.

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

Two blocks are `include`d rather than written out here, both at the point they
used to sit at: `reelout_feasibility.jl` (the gates on the installed path and the
turn-rate coefficients they are read against, defining `el_floor`, `c1_at` and
`phase5_margin`) and `reelout_results.jl` (scoring, the summary YAML, the
archive, the plots and the finished-run marker). Both run at top level in the
same scope, so they see and set this script's globals exactly as inline code did.

Logs to `output/<log_file>_opt.arrow` and `_opt.yaml` — the `_opt` suffix keeps
the lemniscate run's log and summary intact, so the two are comparable after the
fact. `REF_PATH` and `LOG_NAME` carry the flown curve and that name to
`simple_reelout_plots.jl`.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

# Wall clock of the WHOLE script, package loading included, for
# `performance.total_wall_time_s`; `tic`/`toc` below time the phases inside it.
t_script_start = time()
# Named here rather than read off `@__FILE__` where the summary is built: that
# code lives in `reelout_results.jl` now and would report its own name.
run_script = basename(@__FILE__)

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

# Finished-run marker for anything watching this script from OUTSIDE the REPL —
# a sweep, a shell `until [ -f output/last_run_done.txt ]`, an agent. Removed
# here and written as the very last thing the script does, so its presence means
# "this run is over", never "a previous run was". It carries the archive path
# because that, not output/, is where the run's own numbers survive.
const RUN_DONE_FILE = joinpath(output_path, "last_run_done.txt")
rm(RUN_DONE_FILE; force = true)
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
start_params = InitParams(; name = tos.name, length = l_set,
                          winch_params = winch, inflow_conditions = inflow,
                          trajectory = Trajectory(collect(guess_az), collect(guess_el)),
                          input_depower = tos.input_depower,
                          reg_weight = tos.reg_weight,
                          detect_simple_bounds = tos.detect_simple_bounds)
# Known bad? Then say so in a second rather than in the 80 s the solver needs to
# reach its iteration cap and fail again. See the cache in awetrim_client.jl for
# why a recorded failure can be trusted.
let cached = tos.opt_failure_cache ? opt_failed_before(start_params) : nothing
    isnothing(cached) ||
        error(@sprintf("This exact request failed before (%s, recorded %s) and is \
                        cached as bad, so it was not sent: %s at %.5f m.\n\nRetry it \
                        with `clear_opt_failures()`, drop its entry from %s, or set \
                        opt_failure_cache: false in data/traj_opt.yaml.",
                       get(cached, "reason", "no reason recorded"),
                       get(cached, "when", "at an unknown time"), tos.name, l_set,
                       OPT_FAILURE_CACHE))
end
t_solve_start = time()
opt_reply = opt_init(start_params; url = tos.base_url)
# No automatic retry with a different guess, deliberately: a guess that merely
# converges is not the same answer, and picking one silently would hide which
# optimum was flown.
opt_result = try
    opt_step(StepParams(l_set, winch, opt_reply.trajectory); url = tos.base_url)
catch exc
    exc isa HTTP.StatusError && exc.status == 422 || rethrow()
    tos.opt_failure_cache && record_opt_failure!(start_params, "422 from /step")
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
# The startup solve, which holds the script rather than the loop; the blocking
# re-optimizations hold the loop and land in `reopt_blocked_s`.
opt_startup_solve_s = time() - t_solve_start
toc("Received the optimized path in: ")

# The LOBE lift: extra elevation where the sag is deepest, none in the middle of
# the pattern. Baked into every path this run installs — the startup one here,
# each re-optimized reply below — and always evaluated on the reply AS IT
# ARRIVED, before `el_bias` and `el_offset_final`, so it never compounds with a
# correction and needs no delta bookkeeping of its own.
wing_lift(az, el) = lobe_lift(az, el; lift = fcs.el_offset_wing,
                              mode = fcs.el_offset_wing_mode,
                              az_full = fcs.el_offset_wing_az,
                              az_blend = fcs.el_offset_wing_blend,
                              depth = fcs.el_offset_wing_depth)
if fcs.el_offset_wing != 0 && fcs.el_offset_wing_mode == "azimuth"
    @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.1f°, ramped over %.1f°, \
                    zero inside %.1f°.",
                   fcs.el_offset_wing, fcs.el_offset_wing_az, fcs.el_offset_wing_blend,
                   fcs.el_offset_wing_az - fcs.el_offset_wing_blend)
elseif fcs.el_offset_wing != 0 && fcs.el_offset_wing_mode == "azimuth_frac"
    # In fractions of each path's own amplitude, so the degrees below are the
    # STARTUP pattern's; they shrink with it at every install, which is the point.
    amp0 = 0.5 * (maximum(opt_result.trajectory.azimuth) -
                  minimum(opt_result.trajectory.azimuth))
    @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.2f of the pattern's own \
                    amplitude, ramped over %.2f of it — %.1f° and %.1f° on the \
                    startup path's ±%.1f°.",
                   fcs.el_offset_wing, fcs.el_offset_wing_az, fcs.el_offset_wing_blend,
                   fcs.el_offset_wing_az * amp0, fcs.el_offset_wing_blend * amp0, amp0)
elseif fcs.el_offset_wing != 0
    # The fold bound of `lobe_lift`, read on the path that is about to be flown:
    # the pattern shrinks with the tether, so the half-span here is the LOOSEST
    # this run ever sees and a startup that clears it can still fold at 380 m.
    half0 = 0.5 * (maximum(opt_result.trajectory.elevation) -
                   minimum(opt_result.trajectory.elevation))
    @info @sprintf("Lobe lift: %+.2f° at the bottom of the pattern, ramped in from \
                    %.2f half-spans below its centre (%.2f° of the %.2f° half-span \
                    at startup); folds above %.2f° of lift there.",
                   fcs.el_offset_wing, fcs.el_offset_wing_depth,
                   fcs.el_offset_wing_depth * half0, half0,
                   (1 - fcs.el_offset_wing_depth) * half0 / 1.5)
end

# Resample, but NEVER upsample: the reply is a polyline, and interpolating extra
# points onto it concentrates each vertex's turn into one short segment, which
# makes path_radius_profile report a far tighter pattern than the curve is.
n_opt = length(opt_result.trajectory.azimuth) - 1
# Every optimizer answer this run flies, as it arrived — before `el_bias`,
# `el_offset_final` and `el_offset_wing` are added to it. The pattern plot draws
# the lot: a run installs a new path per lap and they shrink INWARD and DOWNWARD
# in angle as the tether grows — measured over a 150 -> 380 m run, azimuth
# ±19/20° -> ±13°, elevation 16-29° -> 10-17°, since the same physical pattern
# subtends less angle on a longer tether. The LAST one alone therefore says
# nothing about what was asked for over the run. Every other curve in that plot
# carries the pre-distortion.
opt_paths_raw = [(collect(Float64.(opt_result.trajectory.azimuth)),
                  collect(Float64.(opt_result.trajectory.elevation)))]
set_path!(fec, opt_result.trajectory.azimuth,
          opt_result.trajectory.elevation .+
              wing_lift(opt_result.trajectory.azimuth, opt_result.trajectory.elevation);
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

# The three gates on the path just installed (elevation floor, ground clearance,
# curvature) and the turn-rate coefficients they are read against. Defines
# `el_floor`, `c1_at` and `phase5_margin`, which the loop below gates every
# re-optimized reply with.
include(joinpath(@__DIR__, "reelout_feasibility.jl"))

@info @sprintf("Elevation lift: el_offset_final = %+.2f°, el_offset_lead = %.1f s \
                (%s), reelout_softstop = %.1f s.",
               fcs.el_offset_final, fcs.el_offset_lead,
               fcs.el_offset_lead > 0 ? "anticipates the end of reel-out" :
                                        "starts at the stop latch / phase 5",
               fcs.reelout_softstop)

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
# Points the path in the air is WORTH checking at — the resolution of the reply
# it came from, not the count it is flown at. The startup path is native (the
# first reply is resampled down, never up); every re-optimization reply is 99
# points upsampled to `n_path`, and a curvature check on an upsampled polyline
# measures the sampling. Updated at each install.
chk_points = n_path

# Elevation bias (`fcs.el_bias_gain`): the kite tracks BELOW the path it is given,
# by 1-2° and up to 3.5° at 380 m, in every segment of every run measured so far.
# One update per lap from the mean signed elevation error, added to the path a
# re-optimization installs afterwards, so what is FLOWN is the curve the optimizer
# solved for rather than a curve 2° under it. The correction is a PROFILE over
# `fcs.el_bias_bins` azimuth bands (1 = one number, i.e. a rigid shift), since the
# sag is deeper at the lobes than at the crossing.
n_el_bins = max(1, fcs.el_bias_bins)
el_bias = zeros(n_el_bins)      # [deg] learnt correction, per azimuth band
el_applied = zeros(n_el_bins)   # [deg] profile the path in the air actually carries
lift_on = false             # `el_offset_final` latched in; never cleared once set
el_shift_events = NamedTuple[]  # in-air shift attempts, one entry per outcome CHANGE
lift_t = NaN                # [s] when it latched; NaN = never
lift_remaining = NaN        # [m] of reel-out left at that moment
el_lap_skip = false         # skip the learning update for the lap the lift landed in
el_shift_warned = false     # a held-back shift warns once; re-armed by the next delivery
el_bias_sum = zeros(n_el_bins)  # [deg] running sum of the sag over the current lap
el_bias_prof = zeros(n_el_bins)  # [deg] sum of the correction the path carried there
el_bias_n = zeros(Int, n_el_bins)  # samples in both sums
el_bias_events = NamedTuple[]

# The pattern the kite is being ASKED to fly, sampled per step. Every
# re-optimization returns a smaller one — a third narrower in azimuth and half as
# tall by 380 m — so the size criteria have to be scored lap by lap against what
# was commanded there, not against the startup path (`fig8_metrics`, below).
geom_t = Float64[]
geom_az_c = Float64[]
geom_az_amp = Float64[]
geom_el_h = Float64[]

"The elevation correction as one number per azimuth band, crossing first [deg]."
prof_str(p) = length(p) == 1 ? @sprintf("%+.2f°", p[1]) :
              string("[", join((@sprintf("%+.2f", v) for v in p), " "),
                     "]° (crossing … lobe)")

# Where in the pattern the kite ends up low — the profile a shaped lift is aimed
# at, and the one thing a whole-lap mean like `el_bias` cannot see. Binned on
# |azimuth| as a fraction of the path's OWN amplitude and measured against its own
# elevation centre, so the bins pool over a run whose pattern shrinks by a third
# in azimuth and a half in elevation as the tether grows.
n_droop_bins = 5
droop_n = zeros(Int, n_droop_bins)
droop_flown = zeros(n_droop_bins)   # [deg] kite below the path's elevation centre
droop_ref = zeros(n_droop_bins)     # [-] depth of the path at Q, in half-spans
droop_sag = zeros(n_droop_bins)     # [deg] kite below the path at Q

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

        # What the flown path's elevation should be shifted by: the learnt sag
        # correction, plus the fixed lift `el_offset_final`. The lift is a setpoint
        # move, not an error, so it cancels out of `err_opt`. It starts at the
        # reel-out STOP LATCH rather than at phase 5, which is `path_blend_time`
        # plus a climb later — the kite's lowest point of the whole run is in that
        # gap. `stop_start` stays NaN when `reelout_softstop` is 0, hence phase 5
        # as the fallback.
        if !lift_on && phase >= 4
            v_ro_now = Float64(s.sys_state.v_reelout[1])
            if !isnan(stop_start) || phase >= 5 ||
               (fcs.el_offset_lead > 0 && v_ro_now > 0 &&
                fcs.reelout_l_max - l_set <= v_ro_now * fcs.el_offset_lead)
                global lift_on = true
                global lift_t = t
                global lift_remaining = fcs.reelout_l_max - l_set
                # The lap the lift lands in is a CLIMB towards the new reference,
                # not a bias: learning from it reads the transient as sag and
                # over-corrects, then unwinds it the lap after (measured: -1.4 ->
                # +1.9 deg of error, correction 1.31 -> 2.72 -> 0.83).
                global el_lap_skip = true
                @info @sprintf("Elevation lift of %+.2f° starting at t = %.1f s \
                                (%.1f m of reel-out left, phase %d).",
                               fcs.el_offset_final, t, fcs.reelout_l_max - l_set, phase)
            end
        end
        el_target = lift_on ? el_bias .+ fcs.el_offset_final : copy(el_bias)

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
                lap_before = fig8_n
                global fig8_n = 1 + floor(Int, fig8_idx_progress / n_path)
                # One update per completed lap, per azimuth band: a whole lap
                # averages out the part of the error the kite cannot follow and
                # leaves what the reference can be moved to fix.
                if fcs.el_bias_gain > 0 && fig8_n > lap_before && any(>(0), el_bias_n) &&
                   el_lap_skip
                    global el_lap_skip = false
                    fill!(el_bias_sum, 0.0)
                    fill!(el_bias_prof, 0.0)
                    fill!(el_bias_n, 0)
                    @info @sprintf("Lap %d skipped for learning: the lift landed in it.",
                                   fig8_n - 1)
                elseif fcs.el_bias_gain > 0 && fig8_n > lap_before && any(>(0), el_bias_n)
                    # The kite sags below WHATEVER path it is given, so the error
                    # against the path in the air is the sag and never converges.
                    # What has to converge is the error against the OPTIMIZER's
                    # curve, which is the flown path less the correction in it.
                    # The sag grows once the winch stops, and phase 5 has few laps
                    # left to learn in, so it gets its own gain.
                    gain = phase >= 5 ? fcs.el_bias_gain_final : fcs.el_bias_gain
                    raw = copy(el_bias)
                    err_bin = fill(NaN, n_el_bins)
                    for b in 1:n_el_bins
                        # A band with no samples this lap keeps its value and is
                        # carried by its neighbours through the smoothing.
                        el_bias_n[b] == 0 && continue
                        err_bin[b] = (el_bias_sum[b] + el_bias_prof[b]) / el_bias_n[b]
                        raw[b] = el_bias[b] - gain * err_bin[b]
                    end
                    global el_bias = clamp.(smooth_bins(raw, fcs.el_bias_smooth),
                                            -fcs.el_bias_max, fcs.el_bias_max)
                    n_tot = sum(el_bias_n)
                    err_el = sum(el_bias_sum) / n_tot
                    err_opt = err_el + sum(el_bias_prof) / n_tot
                    push!(el_bias_events,
                          (; t, lap = fig8_n - 1, err_el, err_opt, bias = copy(el_bias),
                           err_bin, n = copy(el_bias_n)))
                    @info @sprintf("Elevation bias after lap %d: kite %.2f° under the \
                                    path it flies, %+.2f° vs the optimizer's; \
                                    correction now %s.",
                                   fig8_n - 1, err_el, err_opt, prof_str(el_bias))
                    fill!(el_bias_sum, 0.0)
                    fill!(el_bias_prof, 0.0)
                    fill!(el_bias_n, 0)
                end
            end

            az_lo, az_hi = extrema(fec.az_path)
            el_lo, el_hi = extrema(fec.el_path)
            az_amp, el_half = 0.5 * (az_hi - az_lo), 0.5 * (el_hi - el_lo)
            el_kite = rad2deg(Float64(s.sys_state.elevation))
            if fcs.el_bias_gain > 0
                # Binned where the correction is APPLIED, and the correction the
                # path carries at this azimuth is the profile's own interpolation
                # rather than the band's value — so the two agree at the fixed
                # point even inside a band the profile ramps across.
                b = azimuth_bin(fec.az_path[fec.last_idx], az_lo, az_hi, n_el_bins)
                el_bias_sum[b] += el_kite - fec.el_path[fec.last_idx]
                el_bias_prof[b] += bias_lift(azimuth_frac(fec.az_path[fec.last_idx],
                                                          az_lo, az_hi), el_bias)
                el_bias_n[b] += 1
            end

            if az_amp > 0 && el_half > 0
                el_c = 0.5 * (el_hi + el_lo)
                b = azimuth_bin(fec.az_path[fec.last_idx], az_lo, az_hi, n_droop_bins)
                droop_n[b] += 1
                droop_flown[b] += el_c - el_kite
                droop_ref[b] += (el_c - fec.el_path[fec.last_idx]) / el_half
                droop_sag[b] += el_kite - fec.el_path[fec.last_idx]
            end

            # The shift reaches the kite at an install, or — when none is due, which
            # is every lap of phase 5 once `max_reopt` is spent — as a blend of the
            # difference onto the path in the air, while the curvature still passes.
            el_delta = el_target .- el_applied
            if maximum(abs, el_delta) > 1e-6 && isnothing(blend_to) && !reopt_pending
                # Per point, not rigidly: with more than one band the shift owed to
                # the lobes is not the one owed to the crossing.
                shifted = fec.el_path .+ bias_lift(fec.az_path, el_delta)
                # Scored at `chk_points`, the resolution the path in the air came
                # at, exactly as the install gate scores its own reply — because a
                # curvature check on an UPSAMPLED polyline measures the sampling,
                # not the curve. Measured 2026-08-18 on the seven curves this gate
                # refused during a 150 -> 380 m run: 0.25 … 0.70 at the flown 359
                # points, 0.92 … 1.10 at the reply's 98, and stable at 60 — while
                # the install gate scored the same replies 0.83 … 1.03. The 359 is
                # the outlier. That artefact refused EVERY in-air shift of every run
                # before this, which left `el_bias` after `max_reopt` spent, and the
                # whole of `el_offset_lead`, with no way to reach the kite. Only the
                # CHECK is downsampled; what is flown keeps `n_path` points, which
                # the lap counter depends on.
                chk_n = min(chk_points, length(fec.az_path) - 1)
                shift_az, shift_el = prepare_path(fec.az_path, shifted;
                                                  resample = chk_n,
                                                  up_loops = fcs.up_loops)
                margin = isnothing(coeffs) ? Inf :
                    check_pattern_feasible(shift_az, shift_el,
                        Float64(s.sys_state.l_tether[1]), fcs.max_steering;
                        c1 = c1_at(phase), prn = false).margin
                if margin >= tos.min_feasibility_margin
                    global blend_from = (copy(fec.az_path), copy(fec.el_path))
                    global blend_to = (copy(fec.az_path), shifted)
                    global blend_t0 = t
                    push!(el_shift_events, (; t, delta = mean(el_delta),
                                            margin, status = "blended in"))
                    global el_applied = copy(el_target)
                    global el_shift_warned = false
                elseif !el_shift_warned
                    push!(el_shift_events, (; t, delta = mean(el_delta),
                                            margin, status = "held back"))
                    global el_shift_warned = true
                    @warn @sprintf("Elevation shift of %s held back: the curvature \
                                    margin would be %.2f. Retrying as the tether grows.",
                                   prof_str(el_delta), margin)
                end
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
                        reopt_params = InitParams(; name = tos.name, length = l_now,
                                                  winch_params = winch,
                                                  inflow_conditions = inflow,
                                                  trajectory = Trajectory(collect(guess_az_r),
                                                                          collect(guess_el_r)),
                                                  input_depower = tos.input_depower,
                                                  reg_weight = tos.reg_weight,
                                                  detect_simple_bounds = tos.detect_simple_bounds)
                        # A cached failure costs the run nothing but the lap it
                        # would have re-optimized on: no request goes out, the
                        # kite keeps the path it has, and `reopt_n` is NOT spent,
                        # so the budget still buys `max_reopt` real solves.
                        cached = tos.opt_failure_cache ? opt_failed_before(reopt_params) :
                                 nothing
                        if !isnothing(cached)
                            global reopt_lap = fig8_idx_progress / n_path
                            push!(reopt_events,
                                  (; t, l = l_now, status = "skipped",
                                   detail = @sprintf("cached failure from %s (%s)",
                                                     get(cached, "when", "an earlier run"),
                                                     get(cached, "reason", "no reason recorded"))))
                            @info @sprintf("Re-optimization at L = %.0f m skipped: this \
                                            exact request is cached as failing (%s). \
                                            clear_opt_failures() to retry it.",
                                           l_now, get(cached, "reason", "no reason recorded"))
                            attempt == length(el_seeds) && break
                            continue
                        end
                        reopt_reply = opt_init(reopt_params; url = tos.base_url)
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
                        failed && tos.opt_failure_cache &&
                            record_opt_failure!(reopt_params,
                                                @sprintf("solver failed at L = %.1f m, \
                                                          guess el %.0f°", l_now, el_seed))
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
                        # Raised BEFORE the gates below, which have to score the
                        # curve that will be flown: lifting it relieves both height
                        # gates and compresses the azimuth axis by cos(elevation),
                        # which the curvature margin must be re-read for.
                        cand_raw = (copy(new_az), copy(new_el))
                        new_el = new_el .+ bias_lift(new_az, el_target) .+
                                 wing_lift(new_az, new_el)
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
                        global chk_points = n_native
                        cand_az, cand_el = prepare_path(new_az, new_el;
                            resample = n_path, up_loops = fcs.up_loops)
                        # At the CURRENT length, which is what it will be flown at.
                        margin = isnothing(coeffs) ? Inf :
                            check_pattern_feasible(chk_az, chk_el, l_now,
                                fcs.max_steering; c1 = c1_at(phase), prn = false).margin
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
                            # Here, not before the gates: a REJECTED reply is not
                            # a path the kite ever flies, and the pattern plot
                            # draws these as what it flew, undistorted.
                            push!(opt_paths_raw, cand_raw)
                            global blend_t0 = t
                            # The candidate carries the whole shift, so the path in
                            # the air does too the moment it lands.
                            maximum(abs, el_target .- el_applied) > 1e-6 &&
                                push!(el_shift_events,
                                      (; t, delta = mean(el_target .- el_applied),
                                       margin, status = "carried by an install"))
                            global el_applied = copy(el_target)
                            # Arm the in-air warning again: it is one warning per
                            # SHIFT, not per run. Without this, the first hold-back
                            # silences every later one — and since the in-air route
                            # is refused far more often than it passes, that hid the
                            # lift's own refusal at t = 122.2 s in the runs of
                            # 2026-08-18.
                            global el_shift_warned = false
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
                            global margin5_flown = phase5_margin(chk_az, chk_el)
                            # Not a rejection reason: this path is accepted for the
                            # reel-out laps it was solved for, and refusing the
                            # best-power curve over the handful of laps after it
                            # would cost more than it buys. Said out loud instead,
                            # once, so a phase 5 flown on the clamp is not a surprise.
                            if !isnan(margin5_flown) &&
                               margin5_flown < tos.min_feasibility_margin && !margin5_warned
                                global margin5_warned = true
                                @warn @sprintf("The path installed at %.0f m has a \
                                                curvature margin of %.2f at \
                                                depower_final (%.2f here at \
                                                depower_setpoint), below \
                                                min_feasibility_margin = %.2f: phase 5 \
                                                will fly it with less turn authority \
                                                than any gate has checked.",
                                               l_now, margin5_flown, margin,
                                               tos.min_feasibility_margin)
                            end
                            event = (; t, l = l_now, status = "installed",
                                     detail = @sprintf("margin %.2f%s, clearance %.1f m, \
                                                        %.0f W predicted", margin,
                                                       isnan(margin5_flown) ? "" :
                                                           @sprintf(" (phase 5: %.2f at %.0f m)",
                                                                    margin5_flown,
                                                                    fcs.reelout_l_max),
                                                       clearance, new_pred))
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
        az_lo_g, az_hi_g = extrema(fec.az_path)
        el_lo_g, el_hi_g = extrema(fec.el_path)
        push!(geom_t, t)
        push!(geom_az_c, 0.5 * (az_hi_g + az_lo_g))
        push!(geom_az_amp, 0.5 * (az_hi_g - az_lo_g))
        push!(geom_el_h, el_hi_g - el_lo_g)
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

# Scoring, the summary YAML, the archive, the plots and the finished-run marker.
include(joinpath(@__DIR__, "reelout_results.jl"))
