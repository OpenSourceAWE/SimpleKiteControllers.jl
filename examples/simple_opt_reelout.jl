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
`max_reopt` times, sent with `wait = false`; `/status` is polled every
`reopt_poll_interval` and the result collected from `opt_trajectory` — whose table
is in RADIANS, unlike the degrees of every struct. While one runs, and after one
that fails, the server keeps serving the previous path, so "not ready" needs no
special case.

What the request IS depends on `use_step`. With `use_step: false` it repeats the
STARTUP solve at the current length — `/init` with the parametric guess of
`traj_opt.yaml`, then the step — so every solve is cold and independent. With
`use_step: true` (the shipped setting) `/init` is sent ONCE, before the run, and a
re-optimization is `/step` alone: the server keeps the session and WARM-STARTS
from the previous optimum, which it re-anchors to the length sent (moving `r0` and
shifting its node-wise warm start with it). That is the cheap solve, and under
`reopt_blocking` it is wall time the simulation is not frozen for. Its price is in
`TrajOptSettings.use_step`: the failure cache cannot key a warm request, and the
run follows one branch of a multi-modal problem instead of re-drawing from the
guess at every length. A warm step that fails falls back to one cold `/init`.

What is NOT done either way is feeding the FLOWN path back as the seed. It is an
(azimuth, elevation) curve optimized for a much shorter radius, so it degrades as a
starting point the further the run walks from its anchor, and the solve escapes to
near-zenith where `C_beta` jams on its 0.9 rad bound, tension collapses to ~1 kN
and the winch law cannot be met. Measured 2026-08-18: re-optimizations seeded that
way fail repeatedly from ~210 m out, while the guess converges at 182, 200, 246,
278, 282, 286 and 317 m — 282 m being one that failed from the flown path and
solves to 9014 W from the guess. A warm start is a different thing: the seed is the
optimizer's own iterate in its own variables and never leaves the server.

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

The profile's SPREAD is rationed, its mean is not. A binned correction is a bend in
the reference where the bands differ, and the pattern shrinks as the tether grows
until it has no curvature to spare: measured 2026-08-20 at 380 m, a reply that was
fine on its own (margin 0.95) kept 0.97 under a uniform lift of the profile's 3.06°
mean and fell to 0.41 under its 1.74° of band-to-band spread — a rejection of the
optimizer's own path over a correction to it. So a candidate is lifted by the whole
mean and by the largest quarter-step of the deviation that still clears
`min_feasibility_margin` at the current length; what is withheld stays in
`el_target - el_applied` and goes in later through the same in-air route as any
other shift, on the same gate. `el_bias_bins = 1` has no spread and is unaffected.

The IN-AIR route rations on the same ladder, and did not always: it was
all-or-nothing, so a shift whose spread failed the gate took its mean down with it.
That is the run's minimum elevation — the last shift owed before the winch stops is
`el_offset_final` plus the converged bias, ~1.6°, and two runs at identical settings
split on it by 0.04 of margin for a degree of ground clearance (see the tuning log,
"The whole-run minimum is the LAST in-air shift missing the gate by 0.04"). The
mean now goes in whole wherever the spread cannot, and only the remainder waits.

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

PROJECT's `kite_settings:` is always redirected to
`data/kite_settings_psm_monolith.yaml` in this repo (a `backend: monolith`
copy of V3Kite's own `kite_settings_psm.yaml`), regardless of which backend
the upstream package ships — see the patched-project block before `init`.

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
using YAML

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
WIND_SPEED = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
# Debug toggle: disables the RUNTIME UpperForceController only (`rc` below), by
# building it from a copy of `wc_settings.yaml`'s f_high raised far out of reach.
# `rcs`/`wc` itself is untouched, so `winch_from_wc(rcs)` still sends the
# optimizer the f_high the file actually ships — this does not loosen what the
# path is optimized for, only what the runtime winch is allowed to do with it.
DISABLE_UFC = true
@info "simple_opt_reelout.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE, wind_speed = $(isnothing(WIND_SPEED) ? "default" : "$WIND_SPEED m/s")."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
# The optimizer's own settings: server, initial guess, solver knobs, margin.
tos = TrajOptSettings(traj_opt_settings_file(project))

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
default_v_wind = project_set.v_wind
apply_windspeed_override!(project_set, WIND_SPEED)
l_tether = project_set.l_tether

# Reel-out speed scales with the wind (more force -> faster reel-out), so a
# slower override needs proportionally more time to cover the same tether
# length and a faster one needs less: effective sim_time (the selected value,
# or the project's own default) scales by default_v_wind / WIND_SPEED, steepened
# by BELOW_DEFAULT_EXPONENT below default_v_wind since that ratio undershoots
# there (see docs/fig8_tuning_log.md, 2026-08-20). Without an override sim_time
# is unaffected.
BELOW_DEFAULT_EXPONENT = 1.66 # 1.5 undershot 3 m/s; see docs/fig8_tuning_log.md, 2026-08-20
EFFECTIVE_SIM_TIME = if isnothing(WIND_SPEED)
    SIM_TIME
else
    wind_ratio = default_v_wind / WIND_SPEED
    scale = wind_ratio <= 1 ? wind_ratio : wind_ratio^BELOW_DEFAULT_EXPONENT
    something(SIM_TIME, project_set.sim_time) * scale
end
isnothing(WIND_SPEED) ||
    @info "simple_opt_reelout.jl: wind-speed override active, sim_time scaled to \
           $(round(EFFECTIVE_SIM_TIME; digits = 1)) s."

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

"""
    write_run_done(status; err = nothing)

Write the finished-run marker. Defined HERE, before anything that can throw, and
called from a `catch` as well as from the normal tail of `reelout_results.jl`, so
the marker's absence means "still running" and never "it died" — a watcher that
only ever sees the success path waits forever on a run that crashed (measured: a
GLMakie main-thread error in the plots, long after the simulation had finished
and the log was safely written).

`status` is `ok`, `ok (plots failed)` or `FAILED`; on failure the exception's
first line follows on an `error:` line. Every other field is read defensively —
a crash early enough leaves `archive_dir`/`fig8m` undefined, and the marker still
has to be writable.
"""
function write_run_done(status::AbstractString; err = nothing)
    get_global(name, default) = isdefined(Main, name) ? getfield(Main, name) : default
    open(RUN_DONE_FILE, "w") do io
        println(io, Dates.format(Dates.now(), "yyyy-mm-dd HH:MM:SS"))
        println(io, "status: ", status)
        isnothing(err) || println(io, "error: ", first(split(sprint(showerror, err), "\n")))
        println(io, "archive: ", get_global(:archive_dir, "none"))
        println(io, "log: ", joinpath(output_path, log_name * ".yaml"))
        fig8m = get_global(:fig8m, nothing)
        println(io, "criteria: ", isnothing(fig8m) ? "n/a" :
                                  isempty(fig8m.criteria_failed) ?
                                  "all $(fig8m.criteria) passed" :
                                  "FAILED: " * join(fig8m.criteria_failed, ", "))
        power = get_global(:opt_power_meas, nothing)
        println(io, "power: ", isnothing(power) ? "n/a" :
                               @sprintf("%.0f W measured", power))
    end
end
# `_opt`: this run must not overwrite the lemniscate run's log and summary — the
# two are each other's baseline.
log_name = basename(project_set.log_file) * "_opt"

# ======================== INIT =========================== #

# Force the monolith backend: patch a copy of `project` whose `kite_settings:`
# always points at this repo's data/kite_settings_psm_monolith.yaml, never at
# V3Kite's own (kernel-backend) kite_settings_psm.yaml. The copy is written to
# `output_path`, so the two kinds of path in it need different treatment:
# `sim_settings` resolves relative to the project file itself and so needs to
# become absolute, and so do the six V3Kite-only keys, which always resolve
# against `v3_data_path()` regardless of the project file's own location. The
# rest (`wc_settings`, `fc_settings`, ...) are bare names that resolve through
# the active data path set above, unaffected by where the project file lives.
project_dict = YAML.load_file(project)
project_dict["system"]["sim_settings"] =
    joinpath(skc_data_path(), project_dict["system"]["sim_settings"])
for key in ("kite_settings", "structural_geometry", "aero_geometry",
            "vsm_settings", "settle_settings", "heading_settings")
    haskey(project_dict["system"], key) || continue
    project_dict["system"][key] = joinpath(v3_data_path(), project_dict["system"][key])
end
project_dict["system"]["kite_settings"] =
    joinpath(skc_data_path(), "kite_settings_psm_monolith.yaml")
PROJECT_MONOLITH = joinpath(output_path, "system_reelout_monolith.yaml")
YAML.write_file(PROJECT_MONOLITH, project_dict)

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
wc.kv = winch_kv(project_set.v_wind; project) # overrides the file's flat kv, see data/winch_kv_table.yaml
rcs = wc                                 # same object, two controllers read it
wpc = WinchPosController(wc; dt = dt0)   # the length loop `step!` used to own

# No dt: init takes it from the project's settings (sample_freq). sim_time falls
# back to the project's own value when SIM_TIME is `nothing` (the `default` choice),
# and is rescaled by EFFECTIVE_SIM_TIME when a wind-speed override is active (above).
# The wind speed comes from the same file unless WIND_SPEED overrides it above: it is
# a plant condition, and project_set.v_wind keeps the mean wind, the turbulent field
# and the optimizer's inflow_from_settings query (below) at the same speed.
# No cache_path either: V3Kite's default is where its own precompile workload
# compiled the model, and a different model binary costs 40 s of re-JIT in init.
s = init(project_set.v_wind, l_tether; body_start_damping = fcs.body_damping,
    body_sim_damping = 0.8 .* fcs.body_damping,
    damping_per_stiffness = DAMPING_PER_STIFFNESS,
    elevation = fcs.elevation, depower_setpoint = fcs.depower_setpoint,
    system_yaml = PROJECT_MONOLITH, use_turbulence = TURBULENCE, aero_mode = AERO_MODE,
    sim_time = EFFECTIVE_SIM_TIME, warmup_time = fcs.warmup_time,
    # The warm-up relaxes at constant length, against the same loop the run uses.
    warmup_torque = (m, l) -> winch_torque!(wpc, m, l), remake_model = false)
@info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)

# REEL_OUT controller: built fresh here so its soft-start ramp (t_startup) begins
# the moment reel-out actually starts (phase 3), not at t = 0. `rcs` is `wc`, the
# object loaded above from the project's `wc_settings:` file — the two winches
# used to need two files in two schemas, and since the merge they share one.
# The file's `dt` is a placeholder; the plant's own timestep is the real one.
rcs.dt = s.dt
if DISABLE_UFC
    rc_settings = deepcopy(rcs)
    rc_settings.f_high = 1.0e6   # never trips; rcs itself keeps the real f_high
    rc = WinchController(rc_settings)
    @warn "DISABLE_UFC is set: the runtime UpperForceController will not engage \
           at any force. The optimizer still solves under the real f_high = \
           $(rcs.f_high) N — this is a runtime-only override."
else
    rc = WinchController(rcs)
end
stop_criteria = fcs.n_fig_eight > 0 ?
    @sprintf("%.0f m or after %d figures of eight", fcs.reelout_l_max, fcs.n_fig_eight) :
    @sprintf("%.0f m", fcs.reelout_l_max)
@info @sprintf("Winch: REEL_OUT mode — v_set = %.3f * sqrt(force), stopping at %s.",
               rcs.kv, stop_criteria)

# Standalone force-floor guard for phases 0-2 (park/dive/hold)
# `WinchController`'s own SpeedController is ACTIVE (its integrator
# accumulating v_set_in - v_act = kv*sqrt(force) - v_act) whenever the force
# limiters are off, i.e. essentially the whole entry, since nothing before
# phase 3 is meant to move the drum. Left running "live" while its output is
# ignored, that integrator winds up over tens of seconds and dumps a large,
# stale command once something changes. 
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
                z0 = %g m | winch kv = %.4f, i.e. %.1f m/s at f_high = %.0f N | \
                depower seed %.3f m%s.",
               inflow.wind_speed, inflow.wind_direction, inflow.profile_law, inflow.z0,
               winch.k_v, winch.k_v * sqrt(winch.f_max), winch.f_max,
               depower_seed(tos, inflow.wind_speed),
               inflow.wind_speed > tos.input_depower_wind_ref ?
                   @sprintf(" (%.2f + %.3f per m/s above %.1f m/s)", tos.input_depower,
                            tos.input_depower_per_wind, tos.input_depower_wind_ref) : "")

# The seed, from data/traj_opt.yaml: it decides whether the solve converges and
# which optimum it converges to. `figure_eight_path` closes the curve itself.
el_center_seed = guess_el_center_seed(tos, inflow.wind_speed)
guess_az, guess_el = figure_eight_path(tos.guess_a, tos.guess_b, tos.guess_c,
                                       tos.guess_d, 0.0, el_center_seed,
                                       0.0, tos.guess_points)
@info @sprintf("Initial guess: %.0f° x %.0f° at %.0f°, %d points.",
               tos.guess_a, tos.guess_b, el_center_seed, tos.guess_points)

# Anchored to the STARTING length. The pattern is optimal there and drifts off it
# as the tether grows; re-optimizing during the run is stage 4, not this script.
ensure_server(tos.base_url; autostart = tos.autostart_server)
# What the solve must respect, as opposed to what the gates below score it
# against afterwards: both are off unless data/traj_opt.yaml turns them on.
#
# The turn radius is asked for with HEADROOM, and re-derived per request rather
# than set once: the optimizer measures `R = r/|kappa|` at each node's own radius,
# which grows through the lap, while the run flies the reply at the ANCHOR, where
# the same angular curvature is tighter by `L/r`. `opt_r_scale` carries both
# corrections — `reelout_anchor_ratio` off the last reply for the geometry, and
# `tos.turn_radius_headroom` for what the GATE adds on top (its finite-difference
# curvature estimate, the elevation lift).
#
# The startup request has no reply to measure the geometry off, so it ASSUMES it:
# `turn_radius_lap_reelout_m` over the starting length. That is the largest ratio of
# the run (35 m is 1.23 at 150 m against 1.09 at 380 m), which is why leaving it at
# 0 shows up as a startup path flown near its curvature limit.
opt_r_scale = (1 + tos.turn_radius_lap_reelout_m / l_set) * tos.turn_radius_headroom
opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale)
opt_r_on = !isnothing(opt_r_min)   # off for margin 0, or an off-grid turn-rate cell
opt_box = pattern_limits_from(tos;
                              elevation_min = elevation_min_request(fcs, tos, l_set))
isnothing(opt_r_min) && isnothing(opt_box) ||
    @info @sprintf("Constraints sent with the request: min_turn_radius %s, \
                    pattern box %s.",
                   isnothing(opt_r_min) ? "unset" :
                       @sprintf("%.2f m (min_feasibility_margin %.2f x the kite's \
                                own, x %.3f for %.0f m of assumed reel-out per lap \
                                and %.2f of headroom)",
                                opt_r_min, tos.min_feasibility_margin, opt_r_scale,
                                tos.turn_radius_lap_reelout_m, tos.turn_radius_headroom),
                   isnothing(opt_box) ? "unset" : string(opt_box))

# One row per depower value the optimizer reports back (startup solve, each
# reopt), for the run summary and the power plot: `t` [s], `l_dp` [m] on
# AWETrim's own scale, and `u_p_equiv`, the V3Kite `rel_depower` expected to fly
# at the same power (`awetrim_depower_to_v3kite`, docs/steering_depower.md).
opt_depower_log = NamedTuple[]
# What phase 4 flies when `tos.fly_opt_depower` is on, kept in step with
# `opt_depower_log`'s latest `u_p_equiv`; falls back to the fixed setpoint
# before the first optimizer answer.
depower_flown_opt = fcs.depower_setpoint

start_params = InitParams(; name = tos.name, length = l_set,
                          winch_params = winch, inflow_conditions = inflow,
                          trajectory = Trajectory(collect(guess_az), collect(guess_el)),
                          input_depower = depower_seed(tos, inflow.wind_speed),
                          reg_weight = tos.reg_weight,
                          detect_simple_bounds = tos.detect_simple_bounds,
                          min_turn_radius = opt_r_min,
                          pattern_limits = opt_box)
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
              $(tos.guess_b)°, guess_el_center = $(el_center_seed)° of \
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

# What the reply was optimized AT, which is not what this run flies. The server
# varies `l_dp` unless it is pinned (`DepowerSpec("fixed", ...)`), and
# `awetrim_depower_to_v3kite` converts it to the V3Kite rel_depower expected to
# fly at the SAME power — the full correction (calibration + measured aero
# offset), not just the two scales' 0.4 m zero difference. See
# docs/steering_depower.md.
if !isnothing(opt_result.depower)
    u_p_equiv = awetrim_depower_to_v3kite(opt_result.depower.value)
    @info @sprintf("Optimized at depower l_dp = %.3f m (mode %s) = rel_depower \
                    %.3f equivalent, against the flown depower_setpoint = %.3f — \
                    a %+.3f gap.",
                   opt_result.depower.value, opt_result.depower.mode, u_p_equiv,
                   fcs.depower_setpoint, u_p_equiv - fcs.depower_setpoint)
end
# The optimizer's own curvature diagnostic, in metres and physical: comparable
# with the kite's 1/(c1*u_s) and with `min_feasibility_margin`, and read
# before this repo's angular gates ever see the path.
isnothing(opt_result.metrics.turn_radius_min_m) ||
    @info @sprintf("Tightest physical turn radius of the reply: %.2f m%s.",
                   opt_result.metrics.turn_radius_min_m,
                   isnothing(opt_r_min) ? "" :
                       @sprintf(" (asked for >= %.2f m)", opt_r_min))

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

# The turn-rate law the gates read a path against, looked up here because the retry
# below has to know whether the startup path clears it. `reelout_feasibility.jl`
# looks it up again and reports it; a cell the table cannot serve costs the retry,
# not the run, so this one stays quiet.
coeffs_startup = try
    turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
catch exc
    exc isa ArgumentError || rethrow()
    nothing
end
# Resample, but NEVER upsample: the reply is a polyline, and interpolating extra
# points onto it concentrates each vertex's turn into one short segment, which
# makes path_radius_profile report a far tighter pattern than the curve is.
#
# A function because the startup path can be installed twice: once from the first
# solve, once from the corrected re-solve below.
install_optimized_path!(reply) = begin
    az = collect(Float64.(reply.trajectory.azimuth))
    el = collect(Float64.(reply.trajectory.elevation))
    set_path!(fec, az, el .+ wing_lift(az, el);
              resample = min(tos.resample_points, length(az) - 1))
    return (az, el)
end
# Every optimizer answer this run flies, as it arrived — before `el_bias`,
# `el_offset_final` and `el_offset_wing` are added to it. The pattern plot draws
# the lot: a run installs a new path per lap and they shrink INWARD and DOWNWARD
# in angle as the tether grows — measured over a 150 -> 380 m run, azimuth
# ±19/20° -> ±13°, elevation 16-29° -> 10-17°, since the same physical pattern
# subtends less angle on a longer tether. The LAST one alone therefore says
# nothing about what was asked for over the run. Every other curve in that plot
# carries the pre-distortion.
opt_paths_raw = [install_optimized_path!(opt_result)]

# set_path! REVERSES a path that does not match up_loops, so a mismatch here is
# not caught by anything downstream: the kite would fly the optimizer's curve,
# but backwards, which is not the trajectory that was optimized.
opt_table = opt_trajectory(; url = tos.base_url)
opt_downloops = opt_table["spline"]["downloops"]
opt_power_pred = Float64(opt_table["metrics"]["avg_power_W"])
# The geometric half of the request correction, now that there is a reply to
# measure it off: how much longer the tether gets over the lap this path was
# solved for. Every re-optimization below asks for `L/r` more turn radius than the
# gate demands, re-measured from the reply that comes back, because the ratio
# shrinks as the tether grows (1.19 at 180 m, 1.09 at 380 m for the same reel-out).
# Guarded, not unconditional: a request that is off (margin 0, or a
# `body_damping`/`depower` the turn-rate table refuses) stays off, and asking
# `min_turn_radius_request` again would only repeat its warning once per lap.
if opt_r_on
    opt_r_scale = reelout_anchor_ratio(opt_table) * tos.turn_radius_headroom
    opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale)
end
isnothing(opt_r_min) ||
    @info @sprintf("Turn-radius request for the re-optimizations: %.2f m — the \
                    gate's %.2f m at margin %.2f, x %.3f for the lap's reel-out \
                    (%.1f -> %.1f m) and x %.2f of headroom.",
                   opt_r_min, opt_r_min / opt_r_scale, tos.min_feasibility_margin,
                   reelout_anchor_ratio(opt_table),
                   minimum(Float64.(opt_table["table"]["distance_radial"])),
                   maximum(Float64.(opt_table["table"]["distance_radial"])),
                   tos.turn_radius_headroom)

# ---- One corrected retry of the STARTUP solve -------------------------- #
# The startup request is the only one that goes out with an ASSUMED reel-out per
# lap (`turn_radius_lap_reelout_m`); the ratio is now MEASURED, off the reply that
# just landed. When the assumption was short the installed path is one the run
# flies below its own gate — measured 2026-08-20 at 150 m: 9.20 m asked for where
# 11.23 m was right, 11.91 m delivered, and 0.90 of margin at the anchor.
#
# So: one warm `/step` at the same length under the corrected radius, and only when
# the installed path is short of `min_feasibility_margin`. Not a different SEED —
# the startup deliberately never retries with one of those, since a guess that
# merely converges is a different optimum flown silently — the same solve under the
# constraint it should have carried. A retry that fails or comes back no better
# changes nothing: the first path stays installed and the gates below judge it as
# they always did.
#
# The retry asks for margin_startup*1.05, not the full min_feasibility_margin: the
# request already carries `turn_radius_headroom` on top of the target, so jumping
# straight to the gate's margin overshoots it at the reply (measured 2026-08-21,
# 150 m: 0.72 -> 0.90 against a target of 0.82) and the wider turn radius narrows
# the installed pattern enough to fail ground clearance instead. A 5 % step keeps
# the correction close to what was actually missing.
if opt_r_on && !isnothing(coeffs_startup)
    margin_startup = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                            c1 = coeffs_startup.c1, prn = false).margin
    if margin_startup < tos.min_feasibility_margin
        margin_retry_target = min(1.05 * margin_startup, tos.min_feasibility_margin)
        opt_r_min_retry = min_turn_radius_request(fcs, tos; scale = opt_r_scale,
                                                  margin = margin_retry_target)
        @info @sprintf("The startup path is at margin %.2f, below \
                        min_feasibility_margin = %.2f: re-solving once at L = %.1f m \
                        under the corrected turn radius (%.2f m, was %.2f m), \
                        targeting margin %.2f (+5 %%).",
                       margin_startup, tos.min_feasibility_margin, l_set,
                       opt_r_min_retry,
                       opt_r_min / opt_r_scale *
                           (1 + tos.turn_radius_lap_reelout_m / l_set) *
                           tos.turn_radius_headroom,
                       margin_retry_target)
        t_retry = time()
        try
            retry_result = opt_step(StepParams(; length = l_set, winch_params = winch,
                                               min_turn_radius = opt_r_min_retry);
                                    url = tos.base_url)
            retry_table = opt_trajectory(; url = tos.base_url)
            retry_raw = install_optimized_path!(retry_result)
            margin_retry = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                                  c1 = coeffs_startup.c1,
                                                  prn = false).margin
            if margin_retry > margin_startup
                global opt_result = retry_result
                global opt_table = retry_table
                global opt_downloops = retry_table["spline"]["downloops"]
                global opt_power_pred = Float64(retry_table["metrics"]["avg_power_W"])
                global opt_paths_raw = [retry_raw]
                global opt_r_scale = reelout_anchor_ratio(retry_table) *
                                     tos.turn_radius_headroom
                global opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale)
                @info @sprintf("Startup re-solve: margin %.2f -> %.2f, predicted power \
                                %.0f W, %.1f s of wall time.",
                               margin_startup, margin_retry, opt_power_pred,
                               time() - t_retry)
            else
                install_optimized_path!(opt_result)   # put the first path back
                @info @sprintf("Startup re-solve gave margin %.2f, no better than \
                                %.2f — keeping the first path (%.1f s).",
                               margin_retry, margin_startup, time() - t_retry)
            end
        catch exc
            exc isa HTTP.StatusError && exc.status == 422 || rethrow()
            @warn "The corrected startup re-solve was refused; flying the first \
                   path." exception = exc
        end
    end
end

if !isnothing(opt_result.depower)
    global depower_flown_opt = awetrim_depower_to_v3kite(opt_result.depower.value)
    push!(opt_depower_log,
          (; t = 0.0, l_dp = opt_result.depower.value, u_p_equiv = depower_flown_opt))
end

# The pattern's own geometry: fcs.f8_* and fcs.el_center describe the GUESS now.
# Captured now, not read off `fec` in the RESULTS block: with reopt_enabled the
# path in `fec` at the end of the run is not the one these describe.
n_path_initial = length(fec.az_path)
path_min_h_start = path_min_height(fec, l_tether)
az_c_path = 0.5 * (maximum(fec.az_path) + minimum(fec.az_path))
el_c_path = 0.5 * (maximum(fec.el_path) + minimum(fec.el_path))
az_amp_path = 0.5 * (maximum(fec.az_path) - minimum(fec.az_path))
el_height_path = maximum(fec.el_path) - minimum(fec.el_path)

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
stop_dp_entry = NaN         # [-] rel_depower at the moment it latched
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
blend_retries_total = 0     # cold-restart attempts spent on a rejected reply
# [deg] how far the last reply that was gated out for clearance or elevation fell
# below the floor it was asked for. Carried across cycles, not reset per request:
# the shortfall is structural (an angle curve installed at the anchor, where the
# optimizer earns part of its height by reeling out within the lap) and the next
# length has it too, so paying a retry for it once is enough.
el_min_extra = 0.0
# The blend in progress: two canonical paths of equal length and a start time.
# Both are guaranteed fold-free across the whole w in [0, 1] before either is
# ever assigned — see the accept gate's fold-check below.
blend_from = nothing
blend_to = nothing
blend_t0 = NaN

"""
    blend_folds(az0, el0, az1, el1) -> Bool

Does `blend_paths` between these two closed curves collapse `path_min_radius`
anywhere across `w` in `[0, 1]`, relative to the smaller of the two endpoints'
own radius? Sampled at `tos.blend_probe_points` points; a fold shows up as a
near-zero radius against endpoints that are not, so a coarse sweep catches it —
see the tuning log entry on why this replaced a runtime hold/jump-cap instead.
"""
function blend_folds(az0, el0, az1, el1)
    r0 = min(path_min_radius(az0, el0), path_min_radius(az1, el1))
    r0 <= 0 && return false   # degenerate endpoint; not this check's job
    any(w -> path_min_radius(blend_paths(az0, el0, az1, el1, w)...) <
             tos.blend_fold_margin * r0,
        range(0.0, 1.0; length = tos.blend_probe_points))
end

"""
    retried(n) -> String

How a rejection reports the retries that preceded it, `""` for the first try.
"""
retried(n) = n > 0 ? @sprintf(", after %d retries", n) : ""

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
        # Overrides calc_steering's fixed fcs.depower_setpoint with the optimizer's
        # own converted depower, kept current by the push!(opt_depower_log, ...)
        # sites above; phase 5 below still wins with fcs.depower_final.
        tos.fly_opt_depower && phase == 4 && (rel_depower = depower_flown_opt)
        # Separate from the ladder inside calc_steering so it can fire the SAME
        # step as a 3->4 transition: reel-out finishing does not wait for settling.
        if phase in (3, 4) && reelout_done
            set_phase!(cc, 5)
            phase = 5
        end
        # Ramps depower toward depower_final in step with v_set's own soft-stop
        # decay, so shedding lift offsets the tension the slowing winch would
        # otherwise raise; the clamp holds it at depower_final once reached.
        if !isnan(stop_start)
            rel_depower = stop_dp_entry +
                (fcs.depower_final - stop_dp_entry) * clamp((t - stop_start) / stop_T, 0.0, 1.0)
        elseif phase == 5
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
                # RATIONED, exactly as the install ladder rations a candidate, and
                # for the same reason: the shift's MEAN is a rigid lift the pattern
                # barely notices, its SPREAD is a bend in a curve that has no
                # curvature to spare (measured 2026-08-20 at 380 m: 0.95 -> 0.97
                # under the mean, 0.95 -> 0.41 under the spread). All-or-nothing
                # here threw the mean away with the spread whenever the whole shift
                # missed the gate, and the shift that matters most is the last one —
                # `el_offset_final` plus the converged bias, ~1.6°, owed just before
                # the winch stops and the kite loses 23 % of its turn authority to
                # `depower_final`. Measured on two runs at the same settings whose
                # only difference was the Q rate limit: at t = 117.8 s the whole
                # 1.60° passed at margin 0.75 and the run's floor was 9.66°; the
                # next run's 1.58° read 0.71 at t = 117.1 s, went in NOTHING, and
                # only arrived at the 380 m install at t = 127.6 s — 4.5 s after
                # reel-out had already stopped, blending past the run's minimum at
                # t = 129.1 s, floor 8.70°. A 0.04 miss on one threshold cost a
                # degree of ground clearance, which is what a ladder exists to stop.
                #
                # Per point, not rigidly: with more than one band the shift owed to
                # the lobes is not the one owed to the crossing. What goes in is
                # added to `el_applied`; the remainder stays in `el_delta` and is
                # retried on the next lap, on the same gate, as before.
                d_mean = mean(el_delta)
                d_dev = el_delta .- d_mean
                shifted_by(fm, fd) = fec.el_path .+
                    bias_lift(fec.az_path, fm * d_mean .+ fd .* d_dev)
                function shift_margin(e)
                    isnothing(coeffs) && return Inf
                    a, b = prepare_path(fec.az_path, e; resample = chk_n,
                                        up_loops = fcs.up_loops)
                    check_pattern_feasible(a, b, Float64(s.sys_state.l_tether[1]),
                        fcs.max_steering; c1 = c1_at(phase), prn = false).margin
                end
                # Spread first, then the mean — the spread is what costs margin, so
                # giving it up buys the most height per rung. The mean is never
                # rationed below a quarter: a shift that small is not worth a 4 s
                # blend, and it is retried whole next lap anyway.
                rungs = ((1.0, 1.0), (1.0, 0.75), (1.0, 0.5), (1.0, 0.25),
                         (1.0, 0.0), (0.75, 0.0), (0.5, 0.0), (0.25, 0.0))
                # A rung that clears the endpoint margin can still fold `blend_paths`
                # between the CURRENT elevation and this one somewhere in between
                # (`lobe_lift`'s own docstring: it FOLDS the path once the lift is
                # large enough) — checked here too, not just for reopt installs,
                # since a rung that folds is skipped for the SAME reason a rung
                # short of margin is: the next, smaller one is strictly safer.
                hit = nothing
                margin = NaN
                for (fm, fd) in rungs
                    e = shifted_by(fm, fd)
                    m = shift_margin(e)
                    isnan(margin) && (margin = m)   # the WHOLE shift's margin, reported
                    if m >= tos.min_feasibility_margin &&
                       !blend_folds(fec.az_path, fec.el_path, fec.az_path, e)
                        hit = (fm, fd, e, m)
                        break
                    end
                end
                # A rung that moves no band by more than a hundredth of a degree is
                # not a delivery; let it fall through to the hold-back and retry.
                if !isnothing(hit) &&
                   maximum(abs, hit[1] * d_mean .+ hit[2] .* d_dev) <= 0.01
                    hit = nothing
                end
                if !isnothing(hit)
                    fm, fd, shifted, hit_margin = hit
                    global blend_from = (copy(fec.az_path), copy(fec.el_path))
                    global blend_to = (copy(fec.az_path), shifted)
                    global blend_t0 = t
                    went_in = fm * d_mean .+ fd .* d_dev
                    push!(el_shift_events, (; t, delta = mean(went_in),
                                            margin = hit_margin,
                                            status = (fm < 1 || fd < 1) ?
                                                @sprintf("blended in (%.0f %% of the \
                                                          mean, %.0f %% of the spread)",
                                                         100 * fm, 100 * fd) :
                                                "blended in"))
                    global el_applied = el_applied .+ went_in
                    global el_shift_warned = false
                    (fm < 1 || fd < 1) &&
                        @info @sprintf("Elevation shift rationed to fit the curvature \
                                        gate: %.0f %% of the %+.2f° mean, %.0f %% of \
                                        the %.2f° spread; the rest is retried next \
                                        lap.", 100 * fm, d_mean, 100 * fd,
                                       maximum(d_dev) - minimum(d_dev))
                elseif !el_shift_warned
                    push!(el_shift_events, (; t, delta = mean(el_delta),
                                            margin, status = "held back"))
                    global el_shift_warned = true
                    @warn @sprintf("Elevation shift of %s held back: the curvature \
                                    margin would be %.2f even rationed to a quarter \
                                    of its mean. Retrying as the tether grows.",
                                   prof_str(el_delta), margin)
                end
            end
        end

        # ---- Re-optimize the path for the length now being flown -------- #
        # phase 4 only: l_set is frozen at reelout_l_max in phase 5, so a
        # lap-boundary trigger there would keep re-asking for a length already solved.
        if tos.reopt_enabled && phase == 4
            l_now = Float64(s.sys_state.l_tether[1])

            # Queue: on a lap boundary, never while a solve or a blend is running,
            # and never past max_reopt.
            if !reopt_pending && isnothing(blend_to) && reopt_n < tos.max_reopt &&
               fig8_idx_progress >= (reopt_lap + tos.reopt_every_n_laps) * n_path
                try
                    # A re-optimization is one of two things, chosen by
                    # `use_step` in data/traj_opt.yaml.
                    #
                    # COLD (`use_step: false`): RE-INIT at the current length and
                    # seed from the parametric guess, exactly as the startup solve
                    # does — do NOT feed the flown path back. That path is in
                    # (azimuth, elevation) and was optimized for a much shorter
                    # radius, so the further the run walks from its anchor the worse
                    # a starting point it is, and the solve escapes to near-zenith:
                    # measured 2026-08-18 at 282 m, C_beta jams on its 0.9 rad bound
                    # at ~50° of elevation where tension collapses to ~1 kN and the
                    # winch law cannot be met. The same 282 m solves to 9014 W from
                    # the guess.
                    #
                    # WARM (`use_step: true`): `/step` alone, keeping the session
                    # `/init` built before the run. The seed is then the optimizer's
                    # OWN previous solution, re-anchored to `l_now` by the server
                    # (it moves `r0` and shifts its node-wise warm start with it) —
                    # not an angle curve refitted at a radius it was not solved for,
                    # which is the failure above. The failure cache cannot key such
                    # a request, since what it seeds from is every solve before it.
                    #
                    # Either way ONE retry follows a failure, and only in blocking
                    # mode — see `reopt_retry_el_offset` for why a re-optimization
                    # may retry where the startup solve deliberately may not. A
                    # `nothing` seed below is the warm attempt; a number is a cold
                    # `/init` from the guess at that centre elevation, which is also
                    # what a failed warm step falls back to (and which restarts the
                    # warm-start chain, since `/init` resets the session).
                    el_seeds = if tos.use_step
                        tos.reopt_blocking ? (nothing, el_center_seed) :
                                             (nothing,)
                    elseif tos.reopt_blocking && tos.reopt_retry_el_offset != 0
                        (el_center_seed,
                         el_center_seed + tos.reopt_retry_el_offset)
                    else
                        (el_center_seed,)
                    end
                    # Asked for under the turn authority the reply will be JUDGED
                    # with, which from phase 5 is `depower_final`'s c1 — ~23 % below
                    # the pattern's. A request made at the pattern's c1 there is
                    # short by exactly that: measured 2026-08-20, a 12.15 m reply
                    # answering a 10.37 m request, rejected at margin 0.61.
                    opt_r_on && (global opt_r_min =
                        min_turn_radius_request(fcs, tos; scale = opt_r_scale,
                                                c1 = c1_at(phase)))
                    # The elevation floor MOVES with the length the same way the
                    # turn radius does, and in the other direction: `min_height` is
                    # met at ever lower angles as the tether grows, so a box built
                    # once at the starting length over-asks by hundreds of metres
                    # of tether. Rebuilt per request, raised by whatever the last
                    # gated-out reply fell short by.
                    global opt_box_now = pattern_limits_from(tos;
                        elevation_min = elevation_min_request(fcs, tos, l_now;
                                                              extra = el_min_extra))
                    for (attempt, el_seed) in enumerate(el_seeds)
                        # Set only on a cold attempt: it is what the failure cache
                        # keys on, and a warm step has no such key.
                        reopt_params = nothing
                        if isnothing(el_seed)
                            # `min_turn_radius` travels with the step, not because
                            # the session forgets it — an omitted field keeps what
                            # `/init` set — but because it MOVES with the length:
                            # the anchor correction is `L/r`, and the lap's reel-out
                            # is a bigger fraction of a short tether than of a long
                            # one. `nothing` (the request is off) still means "keep".
                            opt_step(StepParams(; length = l_now, winch_params = winch,
                                                min_turn_radius = opt_r_min,
                                                pattern_limits = opt_box_now);
                                     url = tos.base_url, wait = false)
                        else
                            guess_az_r, guess_el_r =
                                figure_eight_path(tos.guess_a, tos.guess_b, tos.guess_c,
                                                  tos.guess_d, 0.0, el_seed,
                                                  0.0, tos.guess_points)
                            reopt_params = InitParams(; name = tos.name, length = l_now,
                                                      winch_params = winch,
                                                      inflow_conditions = inflow,
                                                      trajectory = Trajectory(collect(guess_az_r),
                                                                              collect(guess_el_r)),
                                                      input_depower = depower_seed(tos, inflow.wind_speed),
                                                      reg_weight = tos.reg_weight,
                                                      detect_simple_bounds = tos.detect_simple_bounds,
                                                      min_turn_radius = opt_r_min,
                                                      pattern_limits = opt_box_now)
                            # A cached failure costs the run nothing but the lap it
                            # would have re-optimized on: no request goes out, the
                            # kite keeps the path it has, and `reopt_n` is NOT spent,
                            # so the budget still buys `max_reopt` real solves.
                            cached = tos.opt_failure_cache ?
                                     opt_failed_before(reopt_params) : nothing
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
                        end
                        global reopt_pending = true
                        global reopt_t_request = t
                        global reopt_lap = fig8_idx_progress / n_path
                        global reopt_next_poll = t + tos.reopt_poll_interval
                        @info @sprintf("Re-optimizing for L = %.0f m at t = %.1f s \
                                        (lap %.1f, request %d of %d, %s)%s.",
                                       l_now, t, reopt_lap, reopt_n + 1, tos.max_reopt,
                                       isnothing(el_seed) ? "warm start" :
                                           @sprintf("guess el %.0f°", el_seed),
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
                        # Only a cold request can be recorded: the cache key is the
                        # request as the SERVER sees it, and a warm step's seed is
                        # the whole history of solves before it, which no key covers.
                        # A failed step also mutates nothing on the server, so the
                        # next warm start still departs from the last good optimum.
                        failed && tos.opt_failure_cache && !isnothing(reopt_params) &&
                            record_opt_failure!(reopt_params,
                                                @sprintf("solver failed at L = %.1f m, \
                                                          guess el %.0f°", l_now, el_seed))
                        (failed && attempt < length(el_seeds)) || break
                        @info @sprintf("  ... failed from %s; retrying from %s.",
                                       isnothing(el_seed) ? "the warm start" :
                                           @sprintf("guess el %.0f°", el_seed),
                                       @sprintf("guess el %.0f°", el_seeds[attempt + 1]))
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
                local state = try
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
                        let l_dp = Float64(tab["optimized_parameters"]["input_depower"])
                            global depower_flown_opt = awetrim_depower_to_v3kite(l_dp)
                            push!(opt_depower_log, (; t, l_dp, u_p_equiv = depower_flown_opt))
                        end
                    # The whole prospective blend to a converged reply is checked
                    # (`blend_folds`) before it is ever installed, and a folded one
                    # is not flown at all: a fresh reply is requested instead,
                    # holding the simulation, up to `blend_max_retries` times. This
                    # replaced three successive runtime-side attempts to react to a
                    # fold DURING an already-started blend, each of which traded one
                    # failure mode for another — see the tuning log entry "Why
                    # retry, not react" for the measurements behind the choice.
                    #
                    # COLD each time, not a repeated warm `/step`: a warm step
                    # re-anchors the server's OWN previous optimum, and asking for
                    # the same length again from the same warm-start state solves
                    # to the same local optimum every time — measured 2026-08-20,
                    # three installs logged "rejected — blend folds after 3
                    # retries" with nothing varied between attempts, so the three
                    # retries almost certainly re-asked the identical question and
                    # got the identical folding answer back. `el_seeds`/
                    # `reopt_retry_el_offset` exist for the same reason on a
                    # solver FAILURE; reused here with the same "one attempt at
                    # most, per install" cost, since a retry that folds again is
                    # a normal rejection, not a run-ending one.
                    reject_reason = ""
                    # A rejection for clearance or elevation is retried too, but it
                    # is a different retry: what is varied is the FLOOR asked for,
                    # not the guess it is solved from, and the guess only ever moves
                    # up. `el_min_extra` carries the shortfall.
                    reject_low = false
                    # `pred_timeline` grows only on an INSTALL, so this is frozen
                    # for the whole retry chain below; its first entry is the
                    # startup solve, which `min_power_frac_prev` does not gate on.
                    prev_install_pred = length(pred_timeline) > 1 ?
                        pred_timeline[end].power : NaN
                    for blend_attempt in 0:tos.blend_max_retries
                        if blend_attempt > 0
                            global blend_retries_total += 1
                            # Alternating +/- `reopt_retry_el_offset`, not scaled
                            # UP by `blend_attempt`: that grew as far as 3x the
                            # one offset this magnitude is actually validated at
                            # (`reopt_retry_el_offset`'s own docstring, tuned for
                            # solver failures) — measured 2026-08-20, it walked
                            # the guess far enough to converge on a technically
                            # non-folding but near-collapsed pattern (2094 W
                            # predicted against ~22000 W everywhere else, margin
                            # 1.71 — trivially "safe" because there is almost no
                            # pattern left to fold), which the run then flew,
                            # unwrapped ψ reaching 693° start to end.
                            retry_el_seed = el_center_seed +
                                (reject_low || isodd(blend_attempt) ? 1 : -1) *
                                tos.reopt_retry_el_offset
                            retry_el_min = elevation_min_request(fcs, tos, l_now;
                                                                 extra = el_min_extra)
                            @info @sprintf("  ... candidate at L = %.0f m rejected \
                                            (%s); cold-restarting from guess el \
                                            %.0f°%s (retry %d of %d), holding the \
                                            simulation.",
                                           l_now, reject_reason, retry_el_seed,
                                           isnothing(retry_el_min) ? "" :
                                               @sprintf(", floor %.1f°%s", retry_el_min,
                                                        el_min_extra > 0 ?
                                                            @sprintf(" (+%.1f° for the \
                                                                      shortfall)",
                                                                     el_min_extra) : ""),
                                           blend_attempt, tos.blend_max_retries)
                            retry_az, retry_el = figure_eight_path(tos.guess_a,
                                tos.guess_b, tos.guess_c, tos.guess_d, 0.0,
                                retry_el_seed, 0.0, tos.guess_points)
                            retry_params = InitParams(; name = tos.name, length = l_now,
                                winch_params = winch, inflow_conditions = inflow,
                                trajectory = Trajectory(collect(retry_az),
                                                        collect(retry_el)),
                                input_depower = depower_seed(tos, inflow.wind_speed),
                                reg_weight = tos.reg_weight,
                                detect_simple_bounds = tos.detect_simple_bounds,
                                min_turn_radius = opt_r_min,
                                pattern_limits = pattern_limits_from(tos;
                                    elevation_min = retry_el_min))
                            retry_reply = opt_init(retry_params; url = tos.base_url)
                            opt_step(StepParams(l_now, winch, retry_reply.trajectory);
                                     url = tos.base_url, wait = false)
                            t_retry = time()
                            retry_state = "solving"
                            while retry_state == "solving"
                                sleep(tos.reopt_poll_interval)
                                retry_state = try
                                    opt_status(tos.base_url)["state"]
                                catch exc
                                    @warn "Could not reach the optimizer while \
                                           retrying a folded blend; will retry." exception = exc
                                    "solving"
                                end
                            end
                            global reopt_blocked_s += time() - t_retry
                            if retry_state != "converged"
                                @warn @sprintf("Blend-fold retry %d of %d for L = \
                                                %.0f m did not converge (%s); giving \
                                                up on this cycle.",
                                               blend_attempt, tos.blend_max_retries,
                                               l_now, retry_state)
                                event = (; t, l = l_now, status = "rejected",
                                         detail = @sprintf("blend-fold retry %d did \
                                                            not converge (%s)",
                                                           blend_attempt, retry_state))
                                break
                            end
                            tab = opt_trajectory(; url = tos.base_url)
                            let l_dp = Float64(tab["optimized_parameters"]["input_depower"])
                                global depower_flown_opt = awetrim_depower_to_v3kite(l_dp)
                                push!(opt_depower_log, (; t, l_dp, u_p_equiv = depower_flown_opt))
                            end
                        end
                        # Re-measure the anchor correction for the NEXT request off
                        # the reply that just landed: the lap's reel-out is a
                        # shrinking fraction of a growing tether, so a factor fixed
                        # at the startup length over-asks by the end of the run. Only
                        # the SCALE is settled here — the radius itself is derived
                        # where the request goes out, since it also depends on the
                        # phase's turn authority.
                        opt_r_on && (global opt_r_scale = reelout_anchor_ratio(tab) *
                                                          tos.turn_radius_headroom)
                        # What the optimizer itself measured, in the same metres the
                        # request was made in, next to the radii it measured them at.
                        # The gate below reads the same curve AT THE ANCHOR, so the
                        # two differ by `L/r` — that is what this line is here to
                        # show when a reply is rejected for curvature.
                        opt_r_reply = opt_float(tab["metrics"], "turn_radius_min_m")
                        r_span = extrema(Float64.(tab["table"]["distance_radial"]))
                        # /trajectory is in RADIANS, unlike the degrees of the structs.
                        new_az = rad2deg.(Float64.(tab["table"]["azimuth"]))
                        new_el = rad2deg.(Float64.(tab["table"]["elevation"]))
                        # Raised BEFORE the gates below, which have to score the
                        # curve that will be flown: lifting it relieves both height
                        # gates and compresses the azimuth axis by cos(elevation),
                        # which the curvature margin must be re-read for.
                        cand_raw = (copy(new_az), copy(new_el))
                        # As much of the learnt droop profile as the curvature gate
                        # can take, instead of all of it or none.
                        #
                        # The profile's MEAN is free and its SPREAD is not. Measured
                        # 2026-08-20 on the reply this run rejected at 380 m: the
                        # curve itself was fine (margin 0.95 at the anchor, the
                        # optimizer's own 12.15 m against the 10.36 m asked for), a
                        # uniform lift of the profile's mean (3.06°) left it at 0.97,
                        # and the profile's 1.74° of band-to-band spread took it to
                        # 0.41. The pattern is only 4.7° tall and ±8.6° wide by then,
                        # so a spread of that size is a bend in a curve that has none
                        # to spare, and it lands the tightest radius on the bend.
                        #
                        # Dropping the reply over that is the wrong trade: the path is
                        # what the run exists to fly, the droop correction is a
                        # refinement of it. So the deviation from the mean is scaled
                        # back until the candidate passes, and `el_applied` records
                        # what was actually applied — the rest stays in `el_delta` and
                        # goes in through the in-air shift above once the pattern can
                        # carry it, on the same gate.
                        wing_delta = wing_lift(new_az, new_el)
                        el_mean = mean(el_target)
                        el_dev = el_target .- el_mean
                        n_native = min(tos.resample_points, length(new_az) - 1)
                        lifted(fs, fw) = new_el .+
                                         bias_lift(new_az, el_mean .+ fs .* el_dev) .+
                                         fw .* wing_delta
                        function lifted_margin(e)
                            isnothing(coeffs) && return Inf
                            a, b = prepare_path(new_az, e; resample = n_native,
                                                up_loops = fcs.up_loops)
                            check_pattern_feasible(a, b, l_now, fcs.max_steering;
                                                   c1 = c1_at(phase), prn = false).margin
                        end
                        # Ration order: the LOBE LIFT first, then the droop spread.
                        # Both are the RUN's additions to the optimizer's curve; the
                        # curve is what the run exists to fly, so it is never the
                        # thing given up. Between the two additions the spread is
                        # the one that was MEASURED — `el_bias` is what the kite's
                        # own tracking error asked for, per band, and the learner
                        # re-closes on whatever is left standing — while
                        # `el_offset_wing` is a fixed 1.5° guess at the same shape.
                        # Giving up the measurement to keep the guess is backwards.
                        #
                        # It is the more expensive half of the trade, and knowingly:
                        # measured 2026-08-20 at 380 m, the spread costs 0.54 of
                        # margin (0.95 -> 0.41) against the lobe lift's 0.14
                        # (0.95 -> 0.81), so a spread that fits now needs the lift
                        # rationed to 0 first and often more of the gate besides
                        # (see `min_feasibility_margin`). What the old order bought
                        # was cheap margin and an undelivered correction: on the run
                        # of 2026-08-20 01:36 the phase-5 install took 0 % of the
                        # spread and the +1.57° shift behind it was refused three
                        # times, so the learner's whole profile reached the kite as
                        # its mean.
                        #
                        # Measured on the run of 2026-08-20 01:46: the 358 m install
                        # went from 75 % of the spread to the spread WHOLE (the lobe
                        # lift rationed to 75 % instead), 8 of 8 criteria and 8641 W.
                        # The 380 m install still takes 0 % — 0.14 of freed margin
                        # against a spread that costs 0.54 — and lowering the gate
                        # does not help there either; see the tuning-log entry
                        # "The learnt SPREAD reaches the kite everywhere but phase 5".
                        # The ladder stops at what the kite is ALREADY flying. An
                        # install rations from `el_target`, which is the correction
                        # the learner wants, and knows nothing about `el_applied`,
                        # the one the in-air route already got onto the path — so a
                        # reply that could not carry the spread took it back off,
                        # and the run flew the flat mean again. Measured 2026-08-20
                        # at gate 0.75: the whole +1.60° per-band shift blended in at
                        # t = 117.8 s, and the 380 m install 11 s later put
                        # `delivered_profile_deg` back to [2.35 x5].
                        #
                        # The floor is that applied spread expressed in the new
                        # target's shape (a least-squares fraction, since the two
                        # profiles are different vectors), and the rungs above it are
                        # tried first, the floor itself last among them. Below it the
                        # ladder still runs: a reply whose curve cannot carry even
                        # what is flying is taken anyway — the path is what the run
                        # exists to fly — but it says so, where it used to be silent.
                        applied_dev = el_applied .- mean(el_applied)
                        dev_norm = sum(abs2, el_dev)
                        bias_floor = dev_norm > 1e-12 ?
                            clamp(sum(applied_dev .* el_dev) / dev_norm, 0.0, 1.0) : 0.0
                        rungs = ((1.0, 1.0), (1.0, 0.75), (1.0, 0.5),
                                 (1.0, 0.25), (1.0, 0.0), (0.75, 0.0),
                                 (0.5, 0.0), (0.25, 0.0), (0.0, 0.0))
                        ladder = vcat([r for r in rungs if r[1] >= bias_floor],
                                      [(bias_floor, 0.0)],
                                      [r for r in rungs if r[1] < bias_floor])
                        bias_frac, wing_frac = 1.0, 1.0
                        for (fs, fw) in ladder
                            bias_frac, wing_frac = fs, fw
                            lifted_margin(lifted(fs, fw)) >= tos.min_feasibility_margin &&
                                break
                        end
                        new_el = lifted(bias_frac, wing_frac)
                        (bias_frac < 1 || wing_frac < 1) &&
                            @info @sprintf("Lifts held back on the path for L = %.0f m \
                                            to fit the curvature gate: droop spread at \
                                            %.0f %% (%.2f° band to band), lobe lift at \
                                            %.0f %% of %.2f°. The %.2f° mean of the \
                                            droop goes in whole.",
                                           l_now, 100 * bias_frac,
                                           maximum(el_target) - minimum(el_target),
                                           100 * wing_frac, fcs.el_offset_wing, el_mean)
                        # Louder than the line above, and a different event: this
                        # install does not just withhold a correction, it TAKES BACK
                        # one the kite was already flying.
                        bias_frac < bias_floor - 1e-9 &&
                            @warn @sprintf("The path for L = %.0f m cannot carry the \
                                            spread already flying: %.0f %% installed \
                                            against %.0f %% applied. The in-air route \
                                            will retry it as the tether grows.",
                                           l_now, 100 * bias_frac, 100 * bias_floor)
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
                        chk_az, chk_el = prepare_path(new_az, new_el;
                            resample = n_native, up_loops = fcs.up_loops)
                        global chk_points = n_native
                        cand_az, cand_el = prepare_path(new_az, new_el;
                            resample = n_path, up_loops = fcs.up_loops)
                        # Canonicalized the SAME way as `cand_az`/`cand_el` — same
                        # resample count, same start-point rotation — so
                        # `blend_folds` and the real `blend_paths` both interpolate
                        # the same point correspondence `prepare_path` establishes.
                        # An uncanonicalized `fec.az_path` folds EVERY candidate,
                        # including a plain, ordinary one: measured 2026-08-20.
                        cand_from = prepare_path(fec.az_path, fec.el_path;
                            resample = n_path, up_loops = fcs.up_loops)
                        # At the CURRENT length, which is what it will be flown at.
                        margin = isnothing(coeffs) ? Inf :
                            check_pattern_feasible(chk_az, chk_el, l_now,
                                fcs.max_steering; c1 = c1_at(phase), prn = false).margin
                        clearance = path_min_height(chk_az, chk_el, l_now)
                        # Gated against BOTH the startup prediction and the previous
                        # install's: a collapsed pattern and a worse local optimum
                        # both clear margin, clearance and `blend_folds`, and neither
                        # floor alone sees the other (`min_power_frac`,
                        # `min_power_frac_prev`).
                        new_pred = Float64(tab["metrics"]["avg_power_W"])
                        cand_folds = blend_folds(cand_from..., cand_az, cand_el)
                        if margin < tos.min_feasibility_margin
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("curvature margin %.2f%s",
                                                       margin,
                                                       isnothing(opt_r_reply) ? "" :
                                                           @sprintf(" (the optimizer \
                                                                     measured %.2f m at \
                                                                     r = %.0f-%.0f m, \
                                                                     asked for >= %.2f m)",
                                                                    opt_r_reply, r_span[1],
                                                                    r_span[2],
                                                                    something(opt_r_min, 0.0))))
                            break
                        elseif tos.min_height > 0 && clearance < tos.min_height
                            reason = @sprintf("clearance %.1f m", clearance)
                            # In DEGREES, which is the currency the request is made
                            # in: how far the reply's lowest point sits below the
                            # elevation this gate demands at this length.
                            deficit = asind(min(1.0, tos.min_height / l_now)) -
                                      minimum(chk_el)
                            if tos.elevation_min_from_gates &&
                               blend_attempt < tos.blend_max_retries
                                global el_min_extra += deficit +
                                                       tos.elevation_min_retry_margin
                                reject_reason = reason
                                reject_low = true
                                continue   # re-asked at a raised floor, at the top
                            end
                            event = (; t, l = l_now, status = "rejected",
                                     detail = reason * retried(blend_attempt))
                            break
                        elseif minimum(chk_el) < el_floor
                            # The clearance floor does NOT imply this one: at 318 m
                            # of tether, 50 m of height is 9° of elevation. The
                            # margin is there because the kite flies BELOW its
                            # reference — ~3° measured, twice.
                            reason = @sprintf("descends to %.1f°, below \
                                               min_elevation + margin = %.1f°",
                                              minimum(chk_el), el_floor)
                            if tos.elevation_min_from_gates &&
                               blend_attempt < tos.blend_max_retries
                                global el_min_extra += el_floor - minimum(chk_el) +
                                                       tos.elevation_min_retry_margin
                                reject_reason = reason
                                reject_low = true
                                continue
                            end
                            event = (; t, l = l_now, status = "rejected",
                                     detail = reason * retried(blend_attempt))
                            break
                        elseif cand_folds ||
                               new_pred < tos.min_power_frac * opt_power_pred ||
                               new_pred < tos.min_power_frac_prev * prev_install_pred
                            reason = if cand_folds
                                "blend folds"
                            elseif new_pred < tos.min_power_frac * opt_power_pred
                                @sprintf("%.0f W predicted, below %.0f%% of the \
                                          startup prediction (%.0f W)",
                                         new_pred, 100 * tos.min_power_frac,
                                         opt_power_pred)
                            else
                                @sprintf("%.0f W predicted, below %.0f%% of the \
                                          previous install's (%.0f W)",
                                         new_pred, 100 * tos.min_power_frac_prev,
                                         prev_install_pred)
                            end
                            if blend_attempt < tos.blend_max_retries
                                reject_reason = reason
                                reject_low = false   # this one is not about height
                                continue   # a fresh reply is requested at the top
                            end
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("%s, after %d retries",
                                                       reason, tos.blend_max_retries))
                            break
                        else
                            global blend_from = cand_from
                            global blend_to = (cand_az, cand_el)
                            # Here, not before the gates: a REJECTED reply is not
                            # a path the kite ever flies, and the pattern plot
                            # draws these as what it flew, undistorted.
                            push!(opt_paths_raw, cand_raw)
                            global blend_t0 = t
                            # What went in, which is the target only when the
                            # spread survived the gate above; the remainder stays
                            # in `el_delta` for the in-air route to retry.
                            el_installed = el_mean .+ bias_frac .* el_dev
                            maximum(abs, el_installed .- el_applied) > 1e-6 &&
                                push!(el_shift_events,
                                      (; t, delta = mean(el_installed .- el_applied),
                                       margin,
                                       status = bias_frac < 1 ?
                                           @sprintf("carried by an install (%.0f %% of \
                                                     the spread)", 100 * bias_frac) :
                                           "carried by an install"))
                            global el_applied = el_installed
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
                                     detail = @sprintf("margin %.2f%s%s, clearance %.1f m, \
                                                        %.0f W predicted", margin,
                                                       (bias_frac < 1 || wing_frac < 1) ?
                                                           @sprintf(" (droop spread at \
                                                                     %.0f %%, lobe lift \
                                                                     at %.0f %%)",
                                                                    100 * bias_frac,
                                                                    100 * wing_frac) : "",
                                                       isnan(margin5_flown) ? "" :
                                                           @sprintf(" (phase 5: %.2f at %.0f m)",
                                                                    margin5_flown,
                                                                    fcs.reelout_l_max),
                                                       clearance, new_pred))
                            break
                        end
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
            #
            # `blend_paths` interpolates two closed curves point BY INDEX, and
            # that can fold the curve IN BETWEEN even when neither endpoint does.
            # Rather than react to that HERE — three attempts at it, 2026-08-20,
            # each traded one failure mode for another (see the tuning log: an
            # absolute-margin hold checked the wrong resolution and starved every
            # later reopt; a relative check fixed that but a wide fold zone meant
            # a big single-step jump on recovery; not reverting on abandon avoided
            # the jump but could leave the kite stuck on a barely-blended shape
            # for the rest of the run once retries cascaded) — the WHOLE
            # prospective blend is now checked once, before `blend_to` is ever
            # set, in the accept gate below. `blend_to` is guaranteed fold-free
            # across all of `w`, so this loop needs no guard at all: a plain
            # linear ramp.
            if !isnothing(blend_to)
                w = clamp((t - blend_t0) / tos.path_blend_time, 0.0, 1.0)
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
                global stop_dp_entry = rel_depower
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
                    global stop_dp_entry = rel_depower
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
# Wrapped so that a throw in scoring or archiving still leaves a marker behind:
# `reelout_results.jl` writes the `ok` one itself as its last statement, and only
# a path that never reaches it lands here. Rethrown — the marker reports the
# failure, it does not swallow it.
try
    include(joinpath(@__DIR__, "reelout_results.jl"))
catch exc
    write_run_done("FAILED"; err = exc)
    rethrow()
end
