# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Figure-of-eight path following of the V3 kite, shown LIVE in the KiteViewers 3D
window while it flies.

The live twin of `simple_fig8.jl`: identical flight, logging and scoring, plus a
viewer the simulation loop drives directly. Everything below describes both;
the section "The live viewer" is what the two files differ by, and a diff against
`simple_fig8.jl` should show nothing else.

The kite starts parked at ~73° and reaches the pattern through a four-phase
entry (park -> dive -> hold -> transition). Once engaged, the L0 attractor
guidance (`src/figure_eight_controller.jl`) commands a course and the inner
loop (`src/course_controller.jl`'s `CourseController`) tracks it with the
steering tape — that file also owns the entry state machine and `rel_depower`;
this script calls it once per step and applies what it returns.

guidance (`src/figure_eight_controller.jl`) commands a course and the inner
loop (`src/course_controller.jl`'s `CourseController`) tracks it with the
steering tape — that file also owns the entry state machine and `rel_depower`;
this script calls it once per step and applies what it returns.

# The live viewer

A `Viewer3D` opens once the model is built and is updated from inside the loop,
every `VIEWER_INTERVAL`-th step, with the point positions `step!` has just
written into `s.sys_state` — not by replaying the log afterwards, which is what
KiteViewers' own `park_v3.jl` does. The point/segment topology comes from
`examples/v3_segments.jl`, classified from the running structure `s.sys`, so no
`v3_segments.csv` has to exist or be current.

The viewer is a window on the run, not a control over it: `STOP` (and `PAUSE`,
which shares the same flag) ends the simulation early, at which point the log is
still saved, scored and plotted, exactly as after the overspeed abort. Closing
the window mid-run is not supported — press `STOP` instead.

Once the run is over the same window becomes a replay: `RUN` plays the saved log
back from the start, paced by `REPLAY_TIME_LAPSE` (1 = realtime), and `PAUSE`
(the same button, relabelled) or `STOP` ends it. The playback reads the log that
was just loaded for the metrics, not the live model, so it can be repeated as
often as wanted and costs nothing but the drawing. The REPL is free while it
runs — the replay is an `@async` task driven by the button, so a second click
never starts a second, overlapping playback.

`record_video()` writes that replay to `output/<log_file>.mp4` instead of pacing
it, and `run_example("simple_fig8_live.jl"; record_video = true)` does it
automatically once the run is scored. It records what is on the window, buttons and all, and runs
unpaced — the encoded frame rate is what carries `REPLAY_TIME_LAPSE`, so the file
plays at the same speed the button would, in a fraction of the wall time. Above
`VIDEO_MAX_FPS` the frames are thinned by an integer factor, which keeps that
speed exact rather than encoding a 150 fps file. Recording a run therefore costs
no second simulation: any saved log can be filmed again by loading it into `sl`
and calling `record_video()` a second time. Like `show_plots`, the input holds
for that one run only.

The frame is the window's framebuffer, so `VIDEO_SIZE` gives the file a fixed
resolution independently of how the window has been dragged: the window is
resized to it for the recording and put back afterwards. `nothing` records it as
it is.

`VIEWER_TIME_LAPSE` caps the playback speed: each drawn frame is held to
`VIEWER_INTERVAL * dt / VIEWER_TIME_LAPSE` seconds of wall time, so 1 flies at
realtime and 2 at twice that (measured on a replay: 1.00x and 2.00x). Only a
CAP — the solver runs at 1x to 5x realtime depending on how hard the step is, and
the wait absorbs the fast stretches while the slow ones simply run slow. The
deadline is taken from the clock after every wait rather than accumulated, so a
slow frame is never paid back by a burst, which is what would otherwise make the
motion jerky. Set it to `Inf` for the unpaced run.

The waiting is where the time goes, not the drawing: an unpaced replay of the
same frames costs 0.1 ms each, because `update_segments!` only writes observables
and GLMakie renders them on its own.

Waiting is what the frames are spent on, so the `Performance: … x realtime` line
at the end reports the PACING, not the solver — `simple_fig8.jl` is where that
number means something.

`VIEWER_SCALE` shrinks the world into scene units and `VIEWER_KITE_SCALE` blows
up bridle and wing about the KCU, leaving the tether alone: the V3's 5 m wing on
a 200 m tether is a dot at true scale, and 6 is where its structure reads without
swamping the tether. Neither is an `FC_Settings` field — they change the picture,
not the flight.

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

Logs the run to `output/<log_file>.arrow`, where `log_file` is the project's
`system.log_file` setting (e.g. `fig8_200m`), and `include`s
`simple_fig8_plots.jl` at the end, so the figures come up without a second
call. Unpaced and with the model and settling caches in place, the simulation
runs at about twice realtime, so a 150 s run costs roughly 75 s of wall time;
`VIEWER_TIME_LAPSE` caps it below that, and the first run of a fresh cache takes
minutes longer.

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
| `var_11` | curvature feed-forward steering `u_ff` [-] |
| `var_13` | feed-forward chord correction `chi_ff` [deg] |

`var_10` and `var_12` stay unused: `fig8_metrics.jl` reads them as the reel-out
length setpoint and winch state and relies on a figure-eight run leaving them at 0.

Not a `var_XX` slot: `fig_8` (0 before phase 4, 1 at first entry, +1 per lap
after) carries the live lap count, `SysState`'s field of that name. `cycle`
(the pumping-cycle number) is left at its default — this script has no
reel-in phase to count cycles over.

`bearing` carries `chi_cmd`, the course the loop actually tracks, so
`course - bearing` is the path-following error; the unmodified guidance course
is kept in `var_05`. `var_06`/`var_08` are `CourseController`'s regulated error
and heading/course blend weight — see that file's `calc_steering` docstring for
the feedback-angle fusion and gain schedule behind them.

# Steering feed-forward

With `ff_gain > 0` in `data/fc_settings.yaml` the PID is helped by a curvature
feed-forward from phase 4 on, the same law `simple_opt_reelout.jl` flies: the
path's own course rate `ff_lead_time` ahead of Q, inverted through the turn-rate
law, `u_ff = ff_gain * psi_dot_path / (c1 * v_app)`, plus the chord correction
`chi_ff` subtracted from the commanded course so the guidance does not ask the
PD for the same turn a second time. Both are faded out off the path
(`ff_d_fade`, `ff_err_fade`) and low-passed over `ff_tau`; see
`FC_Settings.ff_gain` for the rationale. It needs `c1`, so it is OFF when
`turn_rate_coeffs` has no cell for this `body_damping`/`depower_setpoint`.

The `sys_state` field carries `CourseController`'s ENTRY STATE MACHINE (0 park,
1 dive, 2 hold, 3 transition, 4 fig8 — this script never reaches 5, which is
`simple_reelout.jl`'s winch-triggered addition), using the same codes as the
reference controller's log so both can be read with the same scripts. Control
is unaffected by the 3 -> 4 step — both are flown identically — it only marks
when the pattern was first tracked closely. `simple_fig8_plots.jl` draws it as
the bottom panel of the time-series figure.

# Parameters

Every tuning parameter of the run is a field of `FC_Settings`
(`src/fc_settings.jl`), loaded from this package's `data/fc_settings.yaml` into
the global `fcs`; each field is documented there. `fcs = FC_Settings(fc_settings(project))`
runs unconditionally near the top of this script, with no `@isdefined` guard —
a pre-defined or hand-mutated `fcs` left in `Main` does NOT
survive the next `include`: it is discarded and rebuilt from the YAML file before
the run that was meant to use it even starts. There is no REPL-side override; a
run with different values means editing `data/fc_settings.yaml` itself (or, for a
sweep, editing it between iterations — see `docs/fig8_tuning_log.md` for how past
sweeps did this). `body_damping` is among the fields; settling starts there and
decays to `init`'s floor of 0.8x it, so the one value fixes both the settling
transient and what is flown. A `body_damping` the cache has not seen before makes
V3Kite re-settle the wing, since the settled-geometry filename encodes it — one
slow `init`, then it is cached like any other.

`sample_freq` is not among them: it is in `data/settings_fig8_200m.yaml`, so
changing the timestep means editing that file.

The aerodynamics model is not among them either: `AERO_MODE` is set at the head of
the USER PARAMETERS block and is `ContinuousAero()`, which integrates the VSM load
instead of holding it frozen over a step. V3Kite's own default is `AeroDirect()`,
the cheaper one — the continuous mode carries its own model binary and settled
geometry and costs more per step.

`DAMPING_PER_STIFFNESS` sits beside it and is not an `FC_Settings` field either.
It sets the structural damping of the TETHER AND BRIDLE segments as a ratio of
their stiffness (`unit_damping = ratio * unit_stiffness` [s]), overriding the
material value in the model's `struc_geometry.yaml`; the wing frame keeps the
damping given there, and `body_damping` (which acts on point velocity relative to
the wing) is unaffected. V3Kite applies it from the start of settling, with a
floor: below `MIN_SETTLE_DAMPING_PER_STIFFNESS` (0.0015) settling would diverge,
so a lower ratio settles at the floor and is set on the settled structure
afterwards. The FLOORED value is what enters the settled-geometry cache key, so
every ratio below the floor shares one `settled_*.bin` — changing this value
within that range costs no re-settle. Passing `nothing` instead restores V3Kite's
default: bridles at the material value, main tether undamped.

The turn-rate table is not indexed by it. `turn_rate_coeffs` is keyed on
`body_damping` and `depower_setpoint` alone, so the `c1` printed below — and the
feasibility margin built from it — was identified without tether damping and is
only an estimate here. Both are diagnostic, so this costs the diagnosis, not the
run.

`run_example("simple_fig8_live.jl"; show_plots = false)` (`src/script_inputs.jl`)
suppresses the figures at the end, which is what makes a sweep bearable. The
inputs hold for that one run: a plain `include` flies with the defaults, so a
sweep can never silently swallow the plots of a later run. Because `fcs` is rebuilt from the YAML file on every
`include` (see above), a sweep over an `FC_Settings` field cannot mutate `fcs` in
the loop — it must rewrite the YAML file itself between iterations, e.g. with
KiteUtils' `update_yaml_scalar` (`src/gui_state.jl` uses it the same way for
`data/gui.yaml`).

Which system project is flown (150m/200m/300m pattern), the run length
(`sim_time`, `default` for the project's own value or a specific number of
seconds), the turbulence level (`use_turbulence`, `default` to leave the
settings YAML in charge) and the mean wind speed (`default` for the project's
own `v_wind`) are read fresh on every `include` from `data/gui.yaml`
(`src/gui_state.jl`), not `Main` globals: run `select_project()`
(`examples/select_project.jl`), `select_sim_time()`
(`examples/select_sim_time.jl`), `select_turbulence()`
(`examples/select_turbulence.jl`) and `select_windspeed()`
(`examples/select_windspeed.jl`) beforehand to change them.

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
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: calc_wind_factor
using LinearAlgebra: norm
using Statistics: mean
using Printf
# Selectively: KiteViewers exports `init` too, which would make V3Kite's ambiguous.
import KiteViewers
using KiteViewers: Viewer3D, update_segments!, update_status_text!, clear_viewer, stop,
                   set_status, bring_viewer_to_front, on
# `GLMakie.save` stays qualified: `save` is a name several packages here export.
import GLMakie
using GLMakie: VideoStream, recordframe!

@info "simple_fig8_live.jl: figure-of-eight path following of the V3 kite."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# V3Kite is torque-only; the winch loops are ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
include(joinpath(@__DIR__, "model_setup.jl"))
include(joinpath(@__DIR__, "v3_segments.jl"))
# The caller's inputs, `run_example("simple_fig8_live.jl"; show_plots = false, record_video = true, ...)`;
# a plain `include` flies with these defaults. The V1 hooks and `steer_injection` are explained where
# they act, below.
(; show_plots, steer_gain_factor, steer_gain_feedback_only, extra_steer_delay, hook_settle,
   steer_injection) = inputs =
    script_inputs(@__FILE__, (; show_plots = true, record_video = false, steer_gain_factor = 1.0,
                               steer_gain_feedback_only = false, extra_steer_delay = 0,
                               hook_settle = 15.0, steer_injection = nothing))
record_video_ = inputs.record_video   # `record_video` is the function below
# Cleared for the same reason: a lemniscate run must not plot the optimized
# reference a previous simple_opt_fig8.jl left behind.
REF_PATH = nothing
VIEWER_INTERVAL = 3     # draw every n-th step
VIEWER_TIME_LAPSE = 1.0 # playback speed cap: 1 = realtime, N = N times faster
REPLAY_TIME_LAPSE = 2.0 # speed of the post-run replay on RUN: 1 = realtime
VIDEO_MAX_FPS = 60      # frames above this are dropped, at the same playback speed
VIDEO_SIZE = (1260, 1350) # window size to record at, restored after; `nothing` = as it is
VIEWER_SCALE = 0.08     # world -> scene units, as in KiteViewers' park_v3.jl
VIEWER_KITE_SCALE = 3.0 # bridle and wing only: a 5 m wing on a 200 m tether is a dot at 1
TEXT_UPDATE_HZ = 5    # cap the on-screen status text to this many refreshes per second,
                        # independent of the (much higher) geometry redraw rate
VIEWER_PX_PER_UNIT = 2.0 # GLMakie screen supersampling; smooths thin tether/segment cylinders
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Structural damping of the tether and bridle segments, as a ratio of their
# stiffness: unit_damping = ratio * unit_stiffness [s]. See the docstring above.
DAMPING_PER_STIFFNESS = 0.002
PROJECT = selected_fig8_project() # system_fig8_{150,200,300}m.yaml, set via select_project()
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
WIND_SPEED = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
@info "simple_fig8_live.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE, wind_speed = $(isnothing(WIND_SPEED) ? "default" : "$WIND_SPEED m/s")."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
# The turn-rate table PROJECT names, not whatever an earlier script left in the session.
reload_turn_rate_table!(project)

project_set = Settings(project)
apply_windspeed_override!(project_set, WIND_SPEED)
l_tether = project_set.l_tether
4.0 <= project_set.v_wind <= 10.0 ||
    @warn "v_wind = $(project_set.v_wind) m/s is outside 4-10 m/s, the range the fig8 \
           settings were tuned for (docs/fig8_tuning_log.md). Below, the entry may fly \
           into the ground; above, the run may stop on v_app_abort."
# Before anything reads fcs: init settles at the depower, the turn-rate lookup, the
# guidance and the controller are built from it. At high wind more depower keeps
# v_app under v_app_abort and a larger pattern wins back the turn-radius margin.
apply_wind_schedule!(fcs, project_set.v_wind)
@info @sprintf("Wind schedule at v_wind = %.1f m/s: depower %.2f, f8_a %.1f°, f8_b %.1f°.",
               project_set.v_wind, fcs.depower_setpoint, fcs.f8_a, fcs.f8_b)

# Log files are arrow files, named after the project's `log_file`, kept out of git.
output_path = normpath(joinpath(@__DIR__, "..", "output"))
mkpath(output_path)
log_name = basename(project_set.log_file)

# ======================== INIT =========================== #

# Decided BEFORE init: the warm-up must relax against the winch the loop commands.
fcs.compliance >= 0 || error("compliance must be >= 0, got $(fcs.compliance)")
# Both winch loops are the CALLER's now: V3Kite's `step!` takes a torque.
wcs = load_wc_settings(wc_settings(project); dt = 1 / project_set.sample_freq)
wpc = nothing
wfc = nothing
if fcs.compliance > 0
    # winch_force_gains returns plain numbers; the controller object is V3Kite's.
    wfc = WinchForceController(; winch_force_gains(fcs)...)
    @info @sprintf("Winch: FORCE mode at compliance = %.2f — len_kp %.0f N/m, \
                    damp %.0f N·s/m, tau %.1f s.",
                   fcs.compliance, wfc.len_kp, wfc.damp, wfc.force_tau)
else
    # Perfectly stiff: the position feed-forward cancels the measured load exactly.
    wpc = WinchPosController(wcs; dt = 1 / project_set.sample_freq)
    @info "Winch: POSITION mode at compliance = 0 — constant unstretched length."
end

# `init_model` (model_setup.jl): sim_time falls back to the project's own value when
# SIM_TIME is `nothing` (the `default` choice); the wind speed comes from the project
# file unless WIND_SPEED overrides it above.
s = init_model(project, project_set, fcs, wpc, SIM_TIME; turbulence = TURBULENCE,
    aero_mode = AERO_MODE, damping_per_stiffness = DAMPING_PER_STIFFNESS, pad_final_time = false,
    # The warm-up must relax against the winch the loop will command.
    warmup_torque = isnothing(wfc) ? (m, l) -> winch_torque!(wpc, m, l) :
                                     (m, l) -> winch_force_hold!(wfc, m, l))

# Constant-length setpoint: the tether length after settling and warm-up.
l0 = s.sys_state.l_tether[1]

fec = FigureEightController(fcs; dt = s.dt)

# Never hardcode these: both arguments move them a lot. The lookup key is
# `body_damping`, the value `init` was given: the damping the model FLIES the
# pattern with is the floor that decays out of it (0.8x by default), so the one
# value identifies both the settling transient and the flown damping c1 belongs
# to.
#
# The coefficients feed the feasibility check, the dead-time context below and,
# through `c1`, the curvature feed-forward; no PD gain is scaled by them, so a
# damping/depower the table cannot serve costs the diagnosis and the
# feed-forward, not the run, so `try_turn_rate_coeffs` flies on unadvised
# rather than aborting a deliberate off-grid run.
coeffs = try_turn_rate_coeffs(fcs)

if isnothing(coeffs)
    c1 = c2 = delay = NaN
else
    (; c1, c2, delay) = coeffs

    # c1 must match the damping in use; that is what makes this check meaningful.
    feas = check_pattern_feasible(fec, l_tether, fcs.max_steering; c1)
    feas.feasible ||
        @warn "Pattern is tighter than the kite's minimum turn radius — expect \
               curvature-limited tracking, not a tuning problem."

    # Dead-time context for the attractor lead: how long the lead arc takes to fly.
    lead_deg = attractor_distance(fcs, fcs.v_app_ref, l_tether)
    lead_time = deg2rad(lead_deg) * l_tether / fcs.v_app_ref
    @info @sprintf("Attractor lead %.1f°%s ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   lead_deg, fcs.attractor_lead_time > 0 ? " (lead time)" : "",
                   lead_time, fcs.v_app_ref, delay, lead_time / delay)
end

cc = CourseController(CourseControllerSettings(fcs; dt = s.dt))

# Live lap counter, logged to SysState's `fig_8`: 0 before the pattern is
# tracked, 1 at first entry into phase 4, then +1 each time `fec.last_idx` has
# advanced one full lemniscate (`n_path` points) since then — the guidance's
# own path parametrization, not a re-derived azimuth threshold. Refs, not plain
# variables: the top-level `for` below is a soft scope, so a plain variable
# reassigned only inside a conditional would rebind to a fresh, unassigned
# local every iteration instead of carrying its value forward.
fig8 = Ref(0)
idx_prev = Ref(fec.last_idx)
idx_progress = Ref(0.0)
n_path = length(fec.az_path)

# Low-pass state of the steering feed-forward; Refs for the same soft-scope reason.
ff_u_filt = Ref(0.0)    # [-]   feed-forward steering
ff_chi_filt = Ref(0.0)  # [rad] chord correction
if fcs.ff_gain > 0 && !(isfinite(c1) && c1 > 0)
    @warn "ff_gain = $(fcs.ff_gain), but there is no turn-rate coefficient c1 — \
           flying WITHOUT steering feed-forward."
end

# V1 model-validation test inputs (oldplans/Plan_model_validation.md, V1), read at the top:
# `steer_gain_factor` multiplies rel_steering, and
# `extra_steer_delay` adds a FIFO delay to it, in samples. Both act only from
# `hook_settle` seconds after phase 4 is first reached, so the entry and phase 3
# fly identically in every run of a sweep. With ff_gain = 0 (as V1 requires),
# scaling rel_steering is the same as scaling heading_p, except at the
# max_steering clamp.
# `steer_gain_feedback_only` true: scale only the feedback part, rel_steering - u_ff. The
# feed-forward lies outside the loop, so this scales the loop gain alone; it differs from the
# default only with ff_gain > 0.
(steer_gain_factor == 1.0 && extra_steer_delay == 0) ||
    @info @sprintf("V1 stability hook in force: gain factor %.3g%s, extra delay %d \
                    samples (%.3f s), active %.1f s after phase 4 begins.",
                   steer_gain_factor, steer_gain_feedback_only ? " (feedback only)" : "",
                   extra_steer_delay, extra_steer_delay * s.dt, hook_settle)
# Kept full of the last `extra_steer_delay` raw commands from the start of the run,
# so it is already primed with real history by the time the hook switches on
# (extra_steer_delay * s.dt is well under hook_settle at every V1 point). The
# feed-forward goes through a FIFO of its own, so the two stay aligned.
steer_delay_buf = Float64[]
ff_delay_buf = Float64[]
t_phase4 = Ref(NaN)    # [s] time phase 4 was first reached this run; NaN before that
# Test input for V2 (oldplans/Plan_model_validation.md): a function τ -> Δu added to
# rel_steering, τ the time since the V1 hooks switched on (t_phase4 + hook_settle), the
# input `steer_injection`. It is not logged: it is a function of time, so
# the analysis recomputes it from the log's phase-4 start (validate_margins.jl).
isnothing(steer_injection) || @info "Steering injection in force (test input)."

# ==================== LIVE VIEWER ======================== #

# The run's own settings, not `Viewer3D(false)`: the zero-argument constructor calls
# `se()`, which wants a `system.yaml` this package's data/ deliberately does not have.
# show_kite = false — the wing is drawn from the topology below, not from kite.obj.
viewer = Viewer3D(project_set, false; px_per_unit = VIEWER_PX_PER_UNIT)
# Classified from the RUNNING structure, so no v3_segments.csv has to exist or be current.
KiteViewers.init_segments(viewer, segment_matrix(s.sys,
    Dict("tether" => Int(KiteViewers.TETHER), "bridle" => Int(KiteViewers.BRIDLE),
         "wing" => Int(KiteViewers.WING))))
clear_viewer(viewer; stop_ = false) # stop_ = false: a stopped viewer breaks the loop at once

# Wall time one drawn frame is allowed to take at most; the deadline is reset from the
# clock after every wait, so a slow frame is never made up for by a fast burst.
frame_ns = VIEWER_INTERVAL * s.dt / VIEWER_TIME_LAPSE * 1e9
frame_deadline = time_ns() + frame_ns

# Geometry is redrawn every VIEWER_INTERVAL-th step, far more often than the status text
# needs refreshing; gate text updates to TEXT_UPDATE_HZ by wall clock, not by step count,
# so the cap holds regardless of solver speed or REPLAY_TIME_LAPSE. Live only — recorded
# video keeps a fresh text per captured frame (see replay_run).
# `update_status_text!` has its OWN internal 1-in-`mod_text` throttle (default 4); leave that
# at 1 (every call passes) so TEXT_UPDATE_HZ is the only throttle actually in effect.
viewer.mod_text = 1
text_update_ns = round(UInt64, 1e9 / TEXT_UPDATE_HZ)
last_text_update = Ref(zero(UInt64))
function maybe_update_status_text!(state; height)
    now = time_ns()
    if now - last_text_update[] >= text_update_ns
        update_status_text!(viewer, state; height)
        last_text_update[] = now
        # Setting the Observable only queues the new text; GLMakie still needs a scheduling
        # point to actually paint it. The caller's own yield() (once per drawn frame) is usually
        # close enough, but this guarantees one right when the text itself changes.
        yield()
    end
end

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

# Assigned OUTSIDE the try: the loop's wall time must survive an early break.
t_wall_start = time()
# No GC pause is allowed to eat a frame deadline; re-enabled in the `finally` below, so an
# early break or an exception can never leave the REPL with collection switched off.
GC.enable(false)
try
    for i in 1:s.steps
        t = s.sys_state.time

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

        # Curvature feed-forward plus chord correction, low-passed over ff_tau; see FC_Settings.ff_gain.
        local u_ff = 0.0
        local chi_ff = 0.0
        if fcs.ff_gain > 0 && cc.phase >= 4 && isfinite(c1) && c1 > 0
            local v_app_ff = max(Float64(s.sys_state.v_app), fcs.v_app_min)
            local speed_ff = rad2deg(v_kite / Float64(s.sys_state.l_tether[1]))  # [deg/s]
            if speed_ff > 0
                local psi_dot_ff = path_turn_rate(fec, fcs.ff_lead_time * speed_ff, speed_ff;
                                                  smooth = fcs.ff_smooth)
                # Faded out when the kite is not on this branch (a Q swap hands it the other lobe's curvature).
                local fade_d = clamp((fcs.ff_d_fade - dmin) / (0.5 * fcs.ff_d_fade), 0.0, 1.0)
                local fade_e = clamp((deg2rad(fcs.ff_err_fade) - abs(cc.err)) /
                                     (0.5 * deg2rad(fcs.ff_err_fade)), 0.0, 1.0)
                local g_ff = fcs.ff_gain * fade_d * fade_e
                local alpha_ff = fcs.ff_tau > 0 ? s.dt / (s.dt + fcs.ff_tau) : 1.0
                ff_u_filt[] += alpha_ff * (g_ff * psi_dot_ff / (c1 * v_app_ff) - ff_u_filt[])
                ff_chi_filt[] += alpha_ff * (g_ff * path_chord_offset(fec) - ff_chi_filt[])
                u_ff = ff_u_filt[]
                chi_ff = ff_chi_filt[]
            end
        end
        local rel_steering, rel_depower, phase = calc_steering(cc, chi_set, heading,
            Float64(s.sys_state.course);
            t, elevation = Float64(s.sys_state.elevation),
            v_kite, v_app = Float64(s.sys_state.v_app),
            dmin, tangent = path_tangent(fec), u_ff, chi_ff)
        chi_cmd = cc.chi_cmd
        w_lim = cc.w_lim
        w_course = cc.w_course
        err = cc.err

        # fig8: jumps to 1 the instant phase 4 is first reached, then +1 per
        # full traversal of the reference path, unwrapped so a single lap
        # never double-counts across the index's `mod1` wrap.
        if phase == 4
            if fig8[] == 0
                fig8[] = 1
                idx_prev[] = fec.last_idx
                t_phase4[] = t   # V1 hook: this run's phase-4 start
            else
                delta = fec.last_idx - idx_prev[]
                delta < -(n_path ÷ 2) && (delta += n_path)
                delta > n_path ÷ 2 && (delta -= n_path)
                idx_progress[] += delta
                idx_prev[] = fec.last_idx
                fig8[] = 1 + floor(Int, idx_progress[] / n_path)
            end
        end

        # V1 stability hooks: see the steer_gain_factor/extra_steer_delay setup above.
        push!(steer_delay_buf, rel_steering)
        push!(ff_delay_buf, u_ff)
        local delayed_u = length(steer_delay_buf) > extra_steer_delay ?
            popfirst!(steer_delay_buf) : rel_steering
        local delayed_ff = length(ff_delay_buf) > extra_steer_delay ?
            popfirst!(ff_delay_buf) : u_ff
        if !isnan(t_phase4[]) && t - t_phase4[] >= hook_settle
            local u_scaled = steer_gain_feedback_only ?
                delayed_ff + steer_gain_factor * (delayed_u - delayed_ff) :
                delayed_u * steer_gain_factor
            isnothing(steer_injection) ||
                (u_scaled += steer_injection(t - t_phase4[] - hook_settle))
            # calc_steering already clamped its own output to ±max_steering;
            # re-clamp here too, or a gain factor > 1 commands the tape angles
            # it was never calibrated for instead of just saturating earlier,
            # as scaling heading_p itself would.
            rel_steering = clamp(u_scaled, -fcs.max_steering, fcs.max_steering)
        end

        # Force mode reels out under load; compliance = 0 holds the length outright.
        if isnothing(wfc)
            step!(s; rel_depower, rel_steering,
                  set_torque = winch_torque!(wpc, s, l0),
                  vsm_interval = fcs.vsm_interval)
        else
            step!(s; rel_depower, rel_steering,
                  set_torque = winch_force_hold!(wfc, s, l0),
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
        # Same for a kite that flies into the ground: nothing stops it there, and the
        # solver only gives up tens of seconds later, well underground.
        if s.sys_state.elevation < 0
            @error @sprintf("Ground contact at t=%.2fs: elevation %.1f° (phase %d, azimuth %.1f°, \
                             v_app %.1f m/s). Stopping.",
                            s.sys_state.time, rad2deg(s.sys_state.elevation), cc.phase,
                            rad2deg(s.sys_state.azimuth), s.sys_state.v_app)
            break
        end

        # After step!, which overwrites parts of sys_state.
        s.sys_state.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 transition, 4 fig8
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
        s.sys_state.var_11 = u_ff              # feed-forward steering [-]
        s.sys_state.var_13 = rad2deg(chi_ff)   # feed-forward chord correction [deg]
        s.sys_state.fig_8 = Int16(fig8[])      # live lap count
        # Not filled anywhere in the model chain: without this the log and the viewer read 0.
        s.sys_state.v_wind_200m .= calc_wind_factor(s.am, 200.0) .* s.sys_state.v_wind_gnd

        if i % VIEWER_INTERVAL == 0
            update_segments!(viewer, s.sys_state; scale = VIEWER_SCALE,
                             kite_scale = VIEWER_KITE_SCALE)
            # Z[1] is the kite end of the tether: the V3's runs point 1 -> 39, 39 is the winch.
            maybe_update_status_text!(s.sys_state; height = s.sys_state.Z[1])
            if viewer.stop
                @info @sprintf("Stopped from the viewer at t = %.2f s.", s.sys_state.time)
                break
            end
            # Default always_sleep=false: the bulk of the wait is a real sleep, which is
            # what lets GLMakie draw; only its last 10 ms spin.
            wait_until(frame_deadline; always_sleep = true)
            global frame_deadline = time_ns() + frame_ns
        end
    end
catch exc
    # `exc`, not `e`: a stray global `e` in the REPL makes the catch binding warn.
    @error "Simulation stopped early at t≈$(round(s.sys_state.time, digits=2))s" exception=(exc, catch_backtrace())
finally
    GC.enable(true)
    GC.gc()
end
# The loop ALONE: saving the log and scoring it below are not simulation.
t_wall = time() - t_wall_start
t_sim = Float64(s.sys_state.time)

# Last state drawn whatever the step count ended on; the window stays open afterwards.
update_segments!(viewer, s.sys_state; scale = VIEWER_SCALE, kite_scale = VIEWER_KITE_SCALE)
update_status_text!(viewer, s.sys_state; height = s.sys_state.Z[1])
stop(viewer)

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
let fig8 = findall(x -> Int(x) == 4, sl.sys_state)
    if isempty(fig8)
        @warn "Phase 4 never reached — no fig8 apparent wind speed."
    else
        va = Float64.(sl.v_app[fig8])
        @printf("  v_app over phase 4 (%.1f s): mean %.2f m/s, range %.2f … %.2f m/s \
                 | v_app_ref = %.1f (%+.1f%%)\n",
                sl.time[fig8[end]] - sl.time[fig8[1]],
                mean(va), minimum(va), maximum(va),
                fcs.v_app_ref, 100 * (mean(va) / fcs.v_app_ref - 1))
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

# ==================== REPLAY ==================== #

# Guards against a second replay starting while one is in flight. `viewer.stop` cannot
# serve as that guard: the viewer's own RUN/PAUSE handler flips it on EVERY click, and
# being wired up in the constructor it always runs before the one below.
replaying = Ref(false)

"""
    replay_run(; video = nothing)

Play the flown log back in the 3D viewer, from the first row to the last, drawing
every `VIEWER_INTERVAL`-th one and holding each frame for as long as it covers in
simulated time divided by `REPLAY_TIME_LAPSE` (1 = realtime). The frame rate comes
from the log's own time column, so a run logged at a different `sample_freq` still
plays at its true speed. Returns when the log is exhausted or `viewer.stop` is set
by `PAUSE`/`STOP`, leaving the viewer stopped and ready for the next click.

`video` is a path to write the same frames to as an MP4 instead of pacing them:
the encoded frame rate carries the playback speed, so the file plays at
`REPLAY_TIME_LAPSE` while the recording itself runs as fast as the window draws.
Use [`record_video`](@ref) rather than this keyword.
"""
function replay_run(; video = nothing)
    # Set before the try: a viewer that has been closed throws below, and the flag
    # would stay latched, with it the button dead for the rest of the session.
    replaying[] = true
    frame = 0
    try
        viewer.stop = false
        clear_viewer(viewer; stop_ = false) # stop_ = false: a stopped viewer breaks the loop at once
        set_status(viewer, isnothing(video) ? "Replay" : "Recording")
        bring_viewer_to_front()
        log_dt = length(sl.time) > 1 ? Float64(sl.time[2] - sl.time[1]) : s.dt
        replay_frame_ns = VIEWER_INTERVAL * log_dt / REPLAY_TIME_LAPSE * 1e9
        deadline = time_ns() + replay_frame_ns
        # Drawn frames can outrun any sane frame rate (dt = 10 ms at VIEWER_INTERVAL = 2
        # and 3x is 150/s), so the video takes every stride-th one — an INTEGER factor,
        # which is what keeps the encoded rate a true REPLAY_TIME_LAPSE.
        stride = max(1, ceil(Int, 1e9 / replay_frame_ns / VIDEO_MAX_FPS))
        # visible = true: the config is applied to the window that is ALREADY open, and
        # VideoStream's own default (false) would hide it for the rest of the session.
        stream = isnothing(video) ? nothing :
                 VideoStream(viewer.fig; visible = true,
                             framerate = max(1, round(Int, 1e9 / replay_frame_ns / stride)))
        # Paced playback only: a GC pause does not fit in a frame budget that is often under
        # 10 ms. Recording keeps GC on — it holds every captured frame and is unpaced anyway.
        isnothing(stream) && GC.enable(false)
        for (i, state) in enumerate(sl)
            viewer.stop && break
            if i % (VIEWER_INTERVAL*REPLAY_TIME_LAPSE) == 0 || i == length(sl)
                update_segments!(viewer, state; scale = VIEWER_SCALE,
                                 kite_scale = VIEWER_KITE_SCALE)
                frame += 1
                if isnothing(stream)
                    maybe_update_status_text!(state; height = state.Z[1])
                    wait_until(deadline; always_sleep = true)
                    # `wait_until`'s last ~10ms is a non-yielding busy-spin (Timers.jl), and at
                    # REPLAY_TIME_LAPSE = 3 a frame's whole budget is often under that — with no
                    # `yield()` here, the GLFW/render thread never gets scheduled and the window
                    # is reported "not responding" by the window manager after a few seconds.
                    deadline = time_ns() + replay_frame_ns
                elseif frame % stride == 0
                    # Recorded video: keep text fresh on every captured frame, not throttled by
                    # TEXT_UPDATE_HZ — the encoded rate is already capped by stride/VIDEO_MAX_FPS.
                    update_status_text!(viewer, state; height = state.Z[1])
                    recordframe!(stream)
                    yield()
                end
            end
        end
        isnothing(stream) || GLMakie.save(video, stream)
    catch exc
        # An @async task that dies silently would just leave the button dead.
        @error "Replay stopped early" exception=(exc, catch_backtrace())
    finally
        # `video`, not `stream`: the try body is its own scope, and the two are nothing together.
        if isnothing(video)
            GC.enable(true)
            GC.gc()
        end
        replaying[] = false
        stop(viewer)
    end
end

"""
    record_video(file = joinpath(output_path, log_name * ".mp4"); window_size = VIDEO_SIZE)

Record the replay of the flown log to `file` and return its path. Runs the same
loop as [`replay_run`](@ref) but unpaced, so it finishes well inside the flown
time; the MP4 still plays at `REPLAY_TIME_LAPSE`, because that is what sets the
encoded frame rate. Needs the viewer window open — it captures what is on it,
buttons and all — and refuses while a replay is in flight.

The video is as many pixels as the window's framebuffer, so `window_size`
resizes the window for the recording and puts it back afterwards; `nothing`
records it as it currently is, whatever the user has dragged it to. Sizes are
rounded UP to even numbers, which h264 requires. A display that scales (HiDPI)
gives a file that many times larger, since the framebuffer is what is read.

The input `record_video = true` does this automatically at the end of
the run. The first call in a session also pays for compiling Makie's video path,
which is tens of seconds and is not repeated.
"""
function record_video(file = joinpath(output_path, log_name * ".mp4");
                      window_size = VIDEO_SIZE)
    replaying[] && error("A replay is running — press PAUSE before recording.")
    mkpath(dirname(abspath(file)))
    old_size = size(viewer.fig.scene)
    rec_size = isnothing(window_size) ? old_size : Tuple(2 .* cld.(window_size, 2))
    if rec_size != old_size
        resize!(viewer.fig, rec_size...)
        sleep(0.3) # GLFW resizes on its next poll, and VideoStream fixes the frame size at once
    end
    t_rec = @elapsed try
        replay_run(; video = file)
    finally
        rec_size == old_size || resize!(viewer.fig, old_size...)
    end
    @info @sprintf("Video: %s at %d x %d (%.1f s of flight recorded in %.1f s).",
                   file, rec_size[1], rec_size[2], sl.time[end] - sl.time[1], t_rec)
    file
end

on(viewer.btn_PLAY.clicks) do _
    # The built-in handler has already toggled `viewer.stop`: clicking RUN during a replay
    # therefore reads as the PAUSE it is labelled, breaking the loop above instead of
    # starting a second playback on top of it.
    replaying[] || @async replay_run()
end
@info "Press RUN in the viewer to replay the run at \
       $(REPLAY_TIME_LAPSE == 1 ? "realtime" : "$(REPLAY_TIME_LAPSE)x") speed, \
       or call record_video() to write it to output/$(log_name).mp4."

record_video_ && record_video()

if show_plots
    include(joinpath(@__DIR__, "simple_fig8_plots.jl"))
else
    @info "Plots suppressed by show_plots = false."
end

nothing
