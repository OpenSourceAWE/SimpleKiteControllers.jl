# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Reel out along a path an EXTERNAL OPTIMIZER produced, instead of along a
lemniscate.

`simple_reelout.jl` with one block added — read that file's docstring for the
winch, the entry state machine, the log slots and the stop criteria, which all
apply unchanged. What differs is the reference path: the AWETrim optimizer is
asked for the power-optimal reel-out path under THIS run's wind and winch, and
`set_path!` installs it. [`simple_opt_fig8.jl`](simple_opt_fig8.jl) was the
rehearsal, flying the same path at constant length.

# The number this run exists to produce

The optimizer predicts a mean reel-out power for its path; reeling out along it
measures one. Both land in the `traj_opt:` section of the run summary with their
ratio. A large gap is the finding, not a bug to hide.

# Re-optimizing while the tether grows

`reopt_enabled` in `data/traj_opt.yaml` (off by default, so a run is
reproducible without a server) re-anchors the path to the length actually flown:
a request every `reopt_every_n_laps` laps, at most `max_reopt` times, polled via
`/status` and collected from `opt_trajectory` (its table is in RADIANS). While a
solve runs or after one fails, the server keeps serving the previous path.

`use_step: false` repeats the cold STARTUP solve (`/init` from the parametric
guess, then `/step`) at each length. `use_step: true` (shipped) sends `/init`
once and re-optimizes with `/step` alone, WARM-STARTING from the previous optimum
re-anchored to the new length — cheaper, but it follows one branch of a
multi-modal problem and the failure cache cannot key it; a failed warm step falls
back to one cold `/init`. The FLOWN path is never fed back as a seed: optimized
for a much shorter radius, it degrades as the run walks out and the solve escapes
to near-zenith (measured 2026-08-18: repeated failures from ~210 m, while the
guess converges at every length tried).

`reopt_blocking = true` (default) freezes the loop for the 7-13 s solve so the
reply matches the length asked for; the frozen time is reported as
`traj_opt.reopt.blocked` and excluded from `performance.realtime_factor`.
`false` flies on, anchoring the reply ~24 m behind at 2.4 m/s.

A candidate is checked for curvature, clearance and elevation AT THE CURRENT
LENGTH, at its OWN resolution (the flown path is resampled to `n_path`; upsampling
a coarse polyline makes the curvature check read too tight). A rejected candidate
is dropped and the run keeps flying what it has. One that passes is aligned with
[`prepare_path`](@ref) and blended in over `path_blend_time`
([`blend_paths`](@ref)), so the reference never steps and the lap counter is
re-based in the same move. Every solve is recorded in `traj_opt.reopt`.

# Lifting the path

The kite tracks BELOW the path it is given (1-2 deg, 3.5 deg at 380 m).
`fcs.el_offset_final` adds a fixed lift once reel-out ends — a setpoint move for
clearance. It latches at the STOP LATCH (the run's lowest point falls between
latch and phase 4 -> 5), or, with `reelout_softstop` at 0, once the length left
is under `v_reelout * el_offset_lead`. It reaches the kite through the next path
install or, when none is due, as a blend onto the path in the air, gated on the
curvature margin and rationed down to a quarter of it if the whole lift is
refused; the remainder waits in `el_target - el_applied` and is retried next lap.

`fcs.el_offset_wing` lifts the LOBES only, because the sag is deeper there than
at the crossing; it is baked into every installed path and rationed the same way.

Every lift is flown narrower, so the pattern-SIZE criteria break first;
`traj_opt.lift_budget` in the summary puts the lift bought against the room each
size criterion has left.

# Why the curvature margin is checked where it is

For ONE path flown all the way out, the start is the worst case: the angular
turn radius `1/(L*c1*u_s)` only shrinks with length, so the startup check runs
at the starting length. With `reopt_enabled` every re-optimization is the worst
case instead: the PHYSICAL radius `1/(c1*u_s)` = 11.35 m is length-independent,
and the optimizer's steering bounds put its tightest loop at 11.0-11.5 m at
every length, so the margin sits at ~1.0 and rejections are the normal case.
The real fix is to teach the optimizer the V3's turn-rate law, not to move the
gate.

# What comes from where

Optimizer settings — server, initial guess, solver knobs, resampling, margin —
are `data/traj_opt.yaml` ([`TrajOptSettings`](@ref)). The CONDITIONS come from
the system project's settings file (wind) and its `wc_settings` (winch law) via
`inflow_from_settings` and `winch_from_wc`. The guess decides whether and where
the solve converges, so this script never retries with a different one; see
`simple_opt_fig8.jl` for the measurements. `fcs.f8_a`/`f8_b`/`el_center` are NOT
flown here; the pattern's centre and extent are measured off the installed path.

`reelout_feasibility.jl` (abort/warn policy on the gates of
[`check_reelout_feasibility`](@ref), defining `feas`, `c1_at`, `phase5_margin`)
and `reelout_results.jl` (scoring, summary YAML, archive, plots, finished-run
marker) are `include`d at top level and read the run's state by name (`publish_run_state!`).

Logs to `output/<log_file>_opt.arrow` and `_opt.yaml`, leaving the lemniscate
run's files intact for comparison. `REF_PATH` and `LOG_NAME` carry the flown
curve and that name to `simple_reelout_plots.jl`.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

# Wall clock of the whole script, packages included; `tic`/`toc` time the phases inside it.
t_script_start = time()
# Named here, not in `reelout_results.jl` where the summary is built and `@__FILE__` is that file.
run_script = basename(@__FILE__)

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
using SimpleKiteControllers: startup_seed_offsets, opt_length, blend_folds, request_constraints
using SimpleKiteControllers: with_elevation_max, with_azimuth_amplitude_min, with_size_box
# The decisions of the startup retries (src/startup_retry.jl), the reopt gate and the loop.
using SimpleKiteControllers: RetryLadder, next_lever, record_422!, record_converged!,
    azimuth_amplitude, elevation_amplitude, gate_candidate, retried
using SimpleKiteControllers: loop_gain_scale, feedforward_step, blended_depower, stop_depower,
    final_force_extra, lift_should_start, lap_index_step, reelout_release, reelout_command,
    soft_stop_speed
# For reelout_feasibility.jl: its thin wrappers ADD METHODS to the package's `c1_at`/`phase5_margin`,
# so they must be imported, not merely used: `using` binds them read-only.
import SimpleKiteControllers: c1_at, phase5_margin
using SimpleKiteControllers: check_reelout_feasibility, ReeloutFeasibility, Phase5MarginState
import WinchControllers   # module name, for the WC_OVERRIDES refresh (calc_vro)
using WinchControllers: WCSettings, WinchController, calc_v_set, on_timer,
    get_state, get_f_err, wcsLowerForceLimit,
    LowerForceController, set_f_set, set_reset, set_v_sw, set_v_act,
    set_tracking, set_force, get_v_set_out, calc_vro
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: AtmosphericModel, calc_wind_factor
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
# The functions moved out of this script, see Plan_refactor_opt_reelout.md.
include(joinpath(@__DIR__, "opt_reelout_lib.jl"))
# The caller's inputs (SHOW_PLOTS, the *_OVERRIDES, the test inputs, REPLAY_PATHS, OUTPUT_PATH), read and
# cleared HERE, so a `SHOW_PLOTS = false` never survives into the next run; see `read_run_inputs`.
# The setup-only ones (the *_OVERRIDES, TR_PATH_PROJECT, OUTPUT_PATH) are read as `inputs.<name>`.
inputs = read_run_inputs()
(; show_plots, steer_disturbance, xtrack_offset, xtrack_phase, hold_compliance, steer_gain_factor,
   steer_gain_feedback_only, extra_steer_delay, hook_settle, replay_paths) = inputs
# Reference curve and log name for simple_reelout_plots.jl; set below, cleared here like SHOW_PLOTS.
REF_PATH = nothing
LOG_NAME = nothing
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Tether/bridle structural damping as a ratio of stiffness [s]; see simple_fig8.jl's docstring.
DAMPING_PER_STIFFNESS = 0.001
PROJECT = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
@assert PROJECT in ("system_reelout_cabauw.yaml", "system_reelout_maasvlakte.yaml") "simple_opt_reelout.jl \
    supports only system_reelout_cabauw.yaml and system_reelout_maasvlakte.yaml, got $PROJECT"
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
WIND_SPEED = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
@info "simple_opt_reelout.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE, wind_speed = $(isnothing(WIND_SPEED) ? "default" : "$WIND_SPEED m/s")."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
# The turn-rate table the PROJECT names (its `turn_rate_coeffs`), not the one `__init__` loaded
# through system_fig8_200m.yaml. Two consumers: the CONTROLLER (gain schedule, curvature
# feed-forward) reads `ctrl_tr_table`; the PATH side (turn-radius requests to the planner, the
# startup gates, the feasibility checks) reads the session's table. Both are the project's unless
# `TR_PATH_PROJECT` (`inputs.path_tr_project`) names another system project, whose table
# then sizes the path: an A/B of the controller's table with the planned path held fixed.
# Session-wide: a later script that does not reload keeps the path side's table.
ctrl_tr_table = SimpleKiteControllers._load_turn_rate_table(project)
reload_turn_rate_table!(isnothing(inputs.path_tr_project) ? project : project_file(inputs.path_tr_project))
isnothing(inputs.path_tr_project) ||
    @info "Turn-rate tables: controller $(turn_rate_coeffs_file(project)), path side \
           $(turn_rate_coeffs_file(project_file(inputs.path_tr_project))) (TR_PATH_PROJECT)."
# The optimizer's own settings: server, initial guess, solver knobs, margin.
tos = TrajOptSettings(traj_opt_settings_file(project))

# Sweep overrides (examples/optimize_fig8.jl: FCS_OVERRIDES), and the same for the optimizer's settings
# (TOS_OVERRIDES), e.g. `reopt_enabled = false` for a test run.
apply_overrides!(fcs, inputs.fcs_overrides, "FCS_OVERRIDES", "FC_Settings", "fcs")
apply_overrides!(tos, inputs.tos_overrides, "TOS_OVERRIDES", "TrajOptSettings", "tos")
# Test input: a steering disturbance `t -> Δu` added after the controller (STEER_DISTURBANCE);
# `stability_opt_reelout.jl`'s model is validated against the loop's response to it.
isnothing(steer_disturbance) || @info "Steering disturbance in force (test input)."
# Test input: a cross-track offset `τ -> δ` [deg], τ the time since phase `XTRACK_PHASE` (default 5)
# began (`xtrack_offset`, `xtrack_phase`). The attractor is moved δ along the path's right-hand normal, so the pursuit
# aims at the parallel curve δ to the right: a reference step for the guided loop alone. The run
# keeps (in `RunState`) `xt_t`, `xt_delta`, `xt_d`, the signed cross-track error to the UNSHIFTED path, and `xt_q`,
# the index of its closest point Q: a reference run's d as a function of Q removes the lap forcing.
# Test input: a COMPLIANT hold in phase 5, `(gain, τF, τpos)` (`hold_compliance`). Instead
# of freezing l_set, the length setpoint moves at gain·kv/(2√F̄)·(F − F̄) − (l_set − l_hold)/τpos: the
# reel-out law's force slope around the force F̄ low-passed over τF [s], zero mean speed, and a slow
# pull back to the length the hold began at. `nothing` holds the length rigidly, as flown.
isnothing(hold_compliance) || @info "Compliant hold in phase 5 (test input): $hold_compliance"
# V1 model-validation test inputs (oldplans/Plan_model_validation.md, V1, point C):
# STEER_GAIN_FACTOR (`steer_gain_factor`) multiplies rel_steering, and
# EXTRA_STEER_DELAY (`extra_steer_delay`) adds a FIFO delay to it, in samples. Both act only from
# HOOK_SETTLE (`hook_settle`) seconds after phase 4 is first reached, so entry, phase 3 and the
# early part of phase 4 fly identically in every run of a sweep.
# STEER_GAIN_FEEDBACK_ONLY (`steer_gain_feedback_only`) true: scale only the feedback part,
# rel_steering - u_ff. The feed-forward lies outside the loop, so this scales the loop gain
# alone; it differs from the default only with ff_gain > 0.
(steer_gain_factor == 1.0 && extra_steer_delay == 0) ||
    @info @sprintf("V1 stability hook in force: gain factor %.3g%s, extra delay %d \
                    samples, active %.1f s after phase 4 begins.",
                   steer_gain_factor, steer_gain_feedback_only ? " (feedback only)" : "",
                   extra_steer_delay, hook_settle)
isnothing(xtrack_offset) || @info "Cross-track offset in force (test input)."

project_set = Settings(project)
default_v_wind = project_set.v_wind
apply_windspeed_override!(project_set, WIND_SPEED)
l_tether = project_set.l_tether

"""
    power_gate_off(pred) -> Bool

Whether `min_power_frac`/`min_power_frac_prev` are bypassed for a candidate
predicting `pred` watts: only below `tos.power_gate_wind_min` mean wind AND only
for a NEGATIVE prediction, where the number reports the optimizer's winch model
leaving its own domain rather than a bad path. Defined after the wind override,
so it gates on the speed actually flown.
"""
power_gate_off(pred) = pred < 0 && project_set.v_wind < tos.power_gate_wind_min

# Simulated time to ask `init` for: the reel-out budget under a wind-speed override, see `sim_budget`.
EFFECTIVE_SIM_TIME = sim_budget(project, project_set, fcs, SIM_TIME, WIND_SPEED, default_v_wind)

# Arrow log files named after the project's `log_file`; OUTPUT_PATH redirects them for parallel sweep runs.
output_path = something(inputs.output_path_arg, normpath(joinpath(@__DIR__, "..", "output")))
mkpath(output_path)

# Finished-run marker for outside watchers: removed here, written last, so its presence means "this run is over".
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
# `_opt`: never overwrite the lemniscate run's log and summary, the two are each other's baseline.
log_name = basename(project_set.log_file) * "_opt"

# ======================== INIT =========================== #

# ONE WCSettings for BOTH winch loops: the POSITION-mode torque gains (`wpc`) and the speed-controller tuning (`rc`).
(; wc, wpc, dt0) = build_winch(project, project_set, fcs)
# Winch overrides for a test run, e.g. `v_sat` or `kv` (WC_OVERRIDES, `inputs.wc_overrides`): applied
# just before the simulation loop, so the optimizer plans the path with the unchanged winch.
rcs = wc                                 # same object, two controllers read it

# Plant overrides for a diagnostic run (SET_OVERRIDES, `inputs.set_overrides`) are applied inside.
s = init_model(project, project_set, fcs, wpc, EFFECTIVE_SIM_TIME; turbulence = TURBULENCE,
               aero_mode = AERO_MODE, damping_per_stiffness = DAMPING_PER_STIFFNESS,
               set_overrides = inputs.set_overrides)

# The controllers, built after `init` so the soft-start ramp begins when reel-out starts; see `build_controllers`.
(; rc, f_high_nominal, guard_lfc, l_set, fec) = build_controllers(fcs, rcs, s)
const F_HIGH_NOMINAL = f_high_nominal

# ================= OPTIMIZED REFERENCE PATH ================== #

# The conditions of THIS run and the winches the optimizer is sent, see `optimizer_conditions`.
(; inflow, cap_wind, winch, winch_first_lap, winch_reopt) =
    optimizer_conditions(tos, fcs, project_set, rcs, F_HIGH_NOMINAL)
# Every reply's optimized gain, so the summary reports what was flown, not only what was sent.
opt_kv_log = NamedTuple{(:t, :l, :k_v, :at_bound), Tuple{Float64, Float64, Float64, Bool}}[]

"""
    apply_optimized_kv!(tab, t, l)

Move the winch gain the optimizer chose out of a `/trajectory` reply and into the
`WCSettings` the run reads, so it flies the `k_v` the path was solved for. `wc`,
`rcs` and `rc.wcs` are one object, and every sub-controller of `rc` holds a
reference to it, so a single assignment reaches all of them. A reply that did not
optimize the gain carries no `k_v` under `optimized_parameters` and this is then a
no-op. A gain that ran into its own bracket is reported: the value is the edge of
the box, not an optimum.
"""
function apply_optimized_kv!(tab, t, l)
    tos.optimize_k_v || return
    params = get(tab, "optimized_parameters", nothing)
    raw = params === nothing ? nothing : get(params, "k_v", nothing)
    raw === nothing && return
    k_v = Float64(raw)
    k_v > 0 || return
    at_bound = something(get(params, "k_v_at_bound", false), false)
    if isempty(opt_kv_log) || abs(k_v - last(opt_kv_log).k_v) > 1e-9
        @info @sprintf("  ... optimizer chose k_v = %.5f at L = %.0f m (was %.5f)%s",
                       k_v, l, wc.kv, at_bound ? " — AT ITS BRACKET EDGE" : "")
        at_bound && @warn "k_v hit the K_V_BRACKET_FACTOR bound: the optimizer wanted \
                           to retune further than it was allowed, so this is the edge \
                           of the box rather than an optimum."
    end
    wc.kv = k_v
    @assert rc.wcs === wc "the reel-out controller must read the WCSettings the gain is written to"
    # EVERY accepted install with a gain, repeats included; a rejected candidate never reaches this function.
    push!(opt_kv_log, (; t, l, k_v, at_bound))
    return
end

# The seed of the startup solve and the connection to the optimizer, see `optimizer_session`; anchored to the
# STARTING length, re-optimizing during the run is stage 4, below.
(; el_center_seed_base, el_center_seed, startup_seed_offset, guess_az, guess_el, opt_chain) =
    optimizer_session(tos, inflow, replay_paths, log_name)
# Constraints the solve must respect; the turn radius carries the anchor ratio `L/r` and the gate's headroom.
(; opt_r_scale, opt_r_min, opt_r_on, opt_r_sent, opt_box) =
    request_constraints(tos, fcs, inflow, cap_wind, opt_length(tos, l_set))

# One row per depower value the optimizer reports back (startup, each ACCEPTED reopt), for summary and plot.
opt_depower_log = NamedTuple[]
# What phases 3+ fly under `fly_opt_depower`; the fixed setpoint until the first optimizer answer.
"""
    RunState

Everything the startup functions and the simulation loop WRITE, in one place, so they take it as an
argument (`st`) instead of rebinding script globals. Comments give the meaning and the unit; the
loop-only bookkeeping is grouped as in the loop. [`publish_run_state!`](@ref) copies the fields
into the script's globals under the same names, for `reelout_feasibility.jl`, `reelout_results.jl`
and the plots, which read them by name.
"""
Base.@kwdef mutable struct RunState
    # ---- the optimizer's answer and what the retries make of it ----
    opt_result::Any = nothing               # the reply the run flies (startup, or the retry that took over)
    opt_table::Any = nothing                # its /trajectory table
    opt_downloops::Any = nothing
    opt_power_pred::Float64 = NaN           # [W] predicted mean reel-out power of the installed path
    opt_paths_raw::Vector{Any} = Any[]      # every optimizer answer as it arrived, before any lift
    opt_paths_at::Vector{Any} = Any[]       # (sim time [s], phase) each of those was installed at
    opt_r_scale::Any = nothing              # anchor ratio x headroom of the turn-radius request
    opt_r_min::Any = nothing                # [m] turn-radius request, or nothing
    opt_box_now::Any = nothing              # pattern limits sent with the last re-optimization request
    incumbent_score::Any = nothing          # score of the best startup path so far
    inc_result::Any = nothing
    inc_table::Any = nothing
    inc_raw::Any = nothing
    startup_wing_frac::Float64 = 1.0        # share of the lobe lift the startup path could carry
    c1_startup::Float64 = NaN               # [-] turn-rate gain the startup path is checked against
    depower_flown_opt::Float64 = NaN        # [-] rel_depower the optimizer asked for
    # ---- the startup pattern's geometry ----
    n_path_initial::Int = 0
    path_min_h_start::Any = NaN
    az_c_path::Any = NaN
    el_c_path::Any = NaN
    az_amp_path::Any = NaN
    el_height_path::Any = NaN
    pred_timeline::Vector{Any} = Any[]      # (; t, power): which path was flown when
    p5_history::Vector{Any} = Any[]         # every path flown, for the phase-5 fallback
    p5_fallback_done::Bool = false          # checked once, from the stop latch on, at the next crossing
    p5_q_az_prev::Float64 = NaN             # [deg] Q's azimuth from the path centre, last step
    p5_fallback::Any = nothing              # (; t, from_margin, to_margin, to_t) when a fallback was blended in
    ccs::Any = nothing                      # course controller settings
    cc::Any = nothing                       # course controller
    # ---- winch and reel-out ----
    l_set::Float64 = NaN                    # [m] tether length setpoint
    transition_start::Float64 = NaN         # [s] time phase 3 began; `reelout_delay` counts from it
    stop_start::Float64 = NaN               # [s] time the soft-stop deceleration latched; NaN = not yet
    stop_v_entry::Float64 = NaN             # [m/s] v_set at the moment it latched
    stop_dp_entry::Float64 = NaN            # [-] rel_depower at the moment it latched
    stop_T::Float64 = NaN                   # [s] duration of the linear decel to reach 0 at reelout_l_max
    reelout_started::Bool = false           # true once the gate has opened; LATCHED, never re-closes
    reelout_start_t::Float64 = NaN          # [s] time it opened; the soft-start ramp counts from here
    reelout_trigger_fired::Bool = false     # true if the FORCE trigger opened it, not the timer
    reelout_done::Bool = false              # true once either stop criterion has ended reel-out
    stop_reason::String = ""                # "length", "laps", or "" if reel-out never stopped
    final_start::Float64 = NaN              # [s] time phase 5 began; the run ends `fcs.final_time` after it
    e_mech::Float64 = 0.0                   # [Wh] running mechanical energy, logged for the viewer
    first_lap_f_high_applied::Bool = false
    # ---- feed-forward, depower ----
    ff_log::Vector{Float64} = Float64[]     # [-] feed-forward steering per step
    ff_chi_log::Vector{Float64} = Float64[] # [rad] chord correction per step
    ff_u_filt::Float64 = 0.0                # [-] low-passed feed-forward steering
    ff_chi_filt::Float64 = 0.0              # [rad] low-passed chord correction
    dp_final_extra::Float64 = 0.0           # [-] phase-5 force limiter's depower above depower_final
    dp_final_extra_peak::Float64 = 0.0      # [-] the most it asked for, for the summary
    rel_depower_prev::Float64 = NaN         # [-] depower commanded last step; the gain reads c1 there
    depower_flown::Float64 = NaN            # [-] current blended output
    depower_blend_from::Float64 = NaN
    depower_blend_to::Union{Nothing, Float64} = nothing
    depower_blend_t0::Float64 = NaN
    # ---- lap counter and scored reference ----
    fig8_n::Int = 0                         # live lap count: 0 before phase 4, 1 at first entry, +1 per traversal
    fig8_idx_prev::Int = 0
    fig8_idx_progress::Float64 = 0.0
    n_path::Int = 0
    raw_az::Any = nothing                   # the reference TRACKING is scored against
    raw_el::Any = nothing
    chk_points::Int = 0                     # resolution the path in the air is checked at
    # ---- elevation lift ----
    el_applied::Float64 = 0.0               # [deg] lift the path in the air actually carries
    lift_on::Bool = false                   # `el_offset_final` latched in; never cleared once set
    el_shift_events::Vector{NamedTuple} = NamedTuple[]  # in-air shift attempts, one per outcome CHANGE
    lift_t::Float64 = NaN                   # [s] when it latched; NaN = never
    lift_remaining::Float64 = NaN           # [m] of reel-out left at that moment
    el_shift_warned::Bool = false           # a held-back shift warns once
    el_shift_lap::Int = 0                   # lap and target of the last in-air shift attempt
    el_shift_target::Float64 = NaN
    # ---- per-step logs of the pattern asked for ----
    geom_t::Vector{Float64} = Float64[]
    geom_az_c::Vector{Float64} = Float64[]
    geom_az_amp::Vector{Float64} = Float64[]
    geom_el_h::Vector{Float64} = Float64[]
    geom_d_raw::Vector{Float64} = Float64[] # cross-track error to the scored reference
    n_droop_bins::Int = 5                   # where in the pattern the kite ends up low, binned on |azimuth|
    droop_n::Vector{Int} = zeros(Int, 5)
    droop_flown::Vector{Float64} = zeros(5) # [deg] kite below the path's elevation centre
    droop_ref::Vector{Float64} = zeros(5)   # [-] depth of the path at Q, in half-spans
    droop_sag::Vector{Float64} = zeros(5)   # [deg] kite below the path at Q
    # ---- re-optimization (stage 4) and blend ----
    reopt_pending::Bool = false             # a solve is queued on the server
    reopt_n::Int = 0                        # solves completed, accepted or rejected
    reopt_lap::Float64 = 0.0                # lap count at which the last request went out
    reopt_next_poll::Float64 = 0.0          # [s] next /status poll
    reopt_t_request::Float64 = NaN          # [s] when the pending request went out
    reopt_blocked_s::Float64 = 0.0          # [s] wall time spent frozen waiting for a reply
    reopt_last_solve_s::Float64 = NaN       # [s] wall time the last blocking wait took
    reopt_events::Vector{NamedTuple} = NamedTuple[]  # one row per solve, for the run summary
    reopt_t_wall_request::Float64 = NaN     # [s] time() when the cycle's first request went out
    reopt_cycles::Vector{NamedTuple} = NamedTuple[]  # (; t, l, status, wall_s) per completed cycle
    blend_retries_total::Int = 0            # cold-restart attempts spent on a rejected reply
    el_min_extra::Float64 = 0.0             # [deg] shortfall of the last reply gated out; carried across cycles
    blend_from::Any = nothing               # the blend in progress; fold-free across w in [0, 1]
    blend_to::Any = nothing
    blend_t0::Float64 = NaN
    raw_from::Any = nothing                 # the scored reference's endpoints of the SAME blend
    raw_to::Any = nothing
    # ---- test inputs ----
    t_phase4::Float64 = NaN                 # [s] time phase 4 was first reached this run; NaN before that
    xt_start::Float64 = NaN                 # [s] first step of phase `xtrack_phase`; τ counts from here
    hold_f_lp::Float64 = NaN                # [N] low-passed force of the compliant hold
    hold_l0::Float64 = NaN                  # [m] length the compliant hold began at
    dist_t::Vector{Float64} = Float64[]     # [s] time of each disturbed step
    dist_d::Vector{Float64} = Float64[]     # [-] disturbance added
    dist_u::Vector{Float64} = Float64[]     # [-] steering sent to the model, controller plus disturbance
    # Kept full of the last EXTRA_STEER_DELAY raw commands from the start of the run,
    # so it is already primed with real history by the time the hook switches on. The
    # feed-forward goes through a FIFO of its own, so the two stay aligned.
    steer_delay_buf::Vector{Float64} = Float64[]
    ff_delay_buf::Vector{Float64} = Float64[]
    xt_t::Vector{Float64} = Float64[]       # [s] time of each phase-5 step
    xt_delta::Vector{Float64} = Float64[]   # [deg] offset commanded
    xt_d::Vector{Float64} = Float64[]       # [deg] signed cross-track error to the unshifted path, right of travel > 0
    xt_q::Vector{Int} = Int[]               # [-] index of the closest path point Q
    xt_phase::Vector{Int} = Int[]           # [-] flight phase, and the operating point for the model:
    xt_L::Vector{Float64} = Float64[]       # [m] tether length
    xt_va::Vector{Float64} = Float64[]      # [m/s] apparent wind speed
    xt_vk::Vector{Float64} = Float64[]      # [m/s] kite speed normal to the tether
    xt_dp::Vector{Float64} = Float64[]      # [-] depower
end

"""
    publish_run_state!(mod, st)

Copy every field of `st` into a global of `mod` under the same name. Called once the startup is done
and again after the loop, because `reelout_feasibility.jl`, `reelout_results.jl` and the plots read the
run's state by name. The functions themselves never touch these globals.
"""
function publish_run_state!(mod::Module, st::RunState)
    for name in fieldnames(RunState)
        # `eval`, not `setglobal!`: Julia >= 1.12 refuses to assign a binding that does not exist yet.
        Core.eval(mod, :($name = $(QuoteNode(getfield(st, name)))))
    end
    return nothing
end

st = RunState(; l_set, opt_r_scale, opt_r_min, depower_flown_opt = fcs.depower_setpoint)

"""
    startup_params(el_center) -> InitParams

The startup `/init` request seeded with the guess lemniscate centred at
`el_center` [deg]; everything else comes from `tos`, the inflow and the
first-lap winch.
"""
function startup_params(el_center)
    az, el = figure_eight_path(tos.guess_a, tos.guess_b,
                               0.0, el_center, 0.0, tos.guess_points)
    InitParams(; name = tos.name, length = opt_length(tos, l_set),
               winch_params = winch_first_lap, inflow_conditions = inflow,
               trajectory = Trajectory(collect(az), collect(el)),
               input_depower = depower_seed(tos, inflow.wind_speed),
               reg_weight = tos.reg_weight,
               detect_simple_bounds = tos.detect_simple_bounds,
               min_turn_radius = opt_r_min, pattern_limits = opt_box)
end

"""
    startup_solve(params) -> (result, seed_trajectory)

`/init` with `params`, the optional seeding solve at `opt_warm_start_awe_trim`,
then the `/step` under the first-lap winch, the one lap 1 flies. Throws the `HTTP.StatusError` of a
422 unchanged; the caller decides whether that ends the run.
"""
function startup_solve(params)
    reply = chain_init(opt_chain, params)
    # Seeding solve: the cold first request fails at low winch force, so it is warmed at `opt_warm_start_awe_trim`.
    seed_trajectory = reply.trajectory
    if tos.opt_warm_start_awe_trim > winch.use_awe_trim
        @info @sprintf("Seeding solve at use_awe_trim %.3f before the startup \
                        request at %.3f; see opt_warm_start_awe_trim.",
                       tos.opt_warm_start_awe_trim, winch.use_awe_trim)
        warm_winch = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v,
                                   use_awe_trim = tos.opt_warm_start_awe_trim)
        seed_trajectory = chain_step(opt_chain, StepParams(opt_length(tos, l_set), warm_winch,
                                                           reply.trajectory)).trajectory
    end
    result = chain_step(opt_chain, StepParams(opt_length(tos, l_set), winch_first_lap,
                                              seed_trajectory))
    return result, seed_trajectory
end

t_solve_start = time()
# A 422 is retried from `startup_retry_el_offsets` in order, see `solve_startup`.
(; opt_result, el_center_seed, startup_seed_offset, guess_az, guess_el) =
    solve_startup(tos, startup_params, startup_solve, startup_params(el_center_seed),
                  el_center_seed_base, l_set, winch, inflow)
st.opt_result = opt_result
startup_seed_offset == 0 ||
    @warn @sprintf("Startup path solved from a RETRY seed centred at %.0f° \
                    (%+.1f° off guess_el_center): a different optimum than the \
                    shipped guess would have given.", el_center_seed, startup_seed_offset)
# The startup solve holds the script; blocking re-optimizations hold the loop (`reopt_blocked_s`).
opt_startup_solve_s = time() - t_solve_start
toc("Received the optimized path in: ")

# What the reply was optimized AT, converted to the V3Kite rel_depower flying at the SAME power.
"""
    log_startup_reply(fcs, opt_result, opt_r_min)

Say what the startup reply was optimized at: its depower converted to the V3Kite `rel_depower` flying at
the SAME power, against the flown setpoint, and the optimizer's own curvature diagnostic, physical and
comparable with `min_feasibility_margin`.
"""
function log_startup_reply(fcs, opt_result, opt_r_min)
    if !isnothing(opt_result.depower)
        u_p_equiv = awetrim_depower_to_v3kite(opt_result.depower.value)
        @info @sprintf("Optimized at depower l_dp = %.3f m (mode %s) = rel_depower \
                        %.3f equivalent, against the flown depower_setpoint = %.3f — \
                        a %+.3f gap.",
                       opt_result.depower.value, opt_result.depower.mode, u_p_equiv,
                       fcs.depower_setpoint, u_p_equiv - fcs.depower_setpoint)
    end
    # The optimizer's own curvature diagnostic, physical and comparable with `min_feasibility_margin`.
    isnothing(opt_result.metrics.turn_radius_min_m) ||
        @info @sprintf("Tightest physical turn radius of the reply: %.2f m%s.",
                       opt_result.metrics.turn_radius_min_m,
                       isnothing(opt_r_min) ? "" :
                           @sprintf(" (asked for >= %.2f m)", opt_r_min))
end
log_startup_reply(fcs, opt_result, opt_r_min)

# The LOBE lift, applied to every installed path on the reply AS IT ARRIVED, before `el_offset_final`.
wing_lift(az, el) = lobe_lift(az, el; lift = fcs.el_offset_wing,
                              mode = fcs.el_offset_wing_mode,
                              az_full = fcs.el_offset_wing_az,
                              az_blend = fcs.el_offset_wing_blend)
"""
    log_lobe_lift(fcs, opt_result)

Say how the lobe lift reads on the startup pattern: in degrees of azimuth, or, for `azimuth_frac`, as
fractions of each path's own amplitude, so the degrees are the STARTUP pattern's.
"""
function log_lobe_lift(fcs, opt_result)
    if fcs.el_offset_wing != 0 && fcs.el_offset_wing_mode == "azimuth"
        @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.1f°, ramped over %.1f°, \
                        zero inside %.1f°.",
                       fcs.el_offset_wing, fcs.el_offset_wing_az, fcs.el_offset_wing_blend,
                       fcs.el_offset_wing_az - fcs.el_offset_wing_blend)
    elseif fcs.el_offset_wing != 0 && fcs.el_offset_wing_mode == "azimuth_frac"
        # Fractions of each path's own amplitude, so the degrees below are the STARTUP pattern's.
        amp0 = 0.5 * (maximum(opt_result.trajectory.azimuth) -
                      minimum(opt_result.trajectory.azimuth))
        @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.2f of the pattern's own \
                        amplitude, ramped over %.2f of it — %.1f° and %.1f° on the \
                        startup path's ±%.1f°.",
                       fcs.el_offset_wing, fcs.el_offset_wing_az, fcs.el_offset_wing_blend,
                       fcs.el_offset_wing_az * amp0, fcs.el_offset_wing_blend * amp0, amp0)
    end
end
log_lobe_lift(fcs, opt_result)

# The turn-rate gain at a depower, NaN off the table; memoized because a blend asks every step.
const c1_memo = Dict{Float64, Float64}()
c1_at_depower(depower) = get!(c1_memo, Float64(depower)) do
    try
        turn_rate_coeffs(fcs.body_damping, depower).c1
    catch exc
        exc isa ArgumentError || rethrow()
        # Once per run: a blend ramping off the grid would repeat it every step.
        any(isnan, values(c1_memo)) ||
            @warn @sprintf("No turn-rate coefficients at depower %.3f (body_damping \
                            %s): gates and gain fall back to the startup law there.",
                           depower, fcs.body_damping)
        NaN
    end
end
# The controller's turn-rate gain at a depower, from `ctrl_tr_table`; NaN off it, memoized like c1_at_depower.
const c1_ctrl_memo = Dict{Float64, Float64}()
c1_ctrl_at(depower) = get!(c1_ctrl_memo, Float64(depower)) do
    try
        turn_rate_coeffs(fcs.body_damping, depower; table = ctrl_tr_table).c1
    catch exc
        exc isa ArgumentError || rethrow()
        NaN
    end
end
# The turn authority the loop was TUNED at; the sim loop rescales heading_p by c1_setpoint/c1(u_d) in every phase.
c1_setpoint = c1_ctrl_at(fcs.depower_setpoint)
# Phase 4 must fly with the curvature feed-forward, which silently drops out on either of these.
fcs.ff_gain > 0 || @warn "simple_opt_reelout.jl needs the curvature feed-forward in phase 4, \
    but ff_gain = $(fcs.ff_gain)"
@assert isfinite(c1_setpoint) && c1_setpoint > 0 "the curvature feed-forward needs the turn-rate \
    coefficient c1 at depower_setpoint = $(fcs.depower_setpoint), got $c1_setpoint"
c1_depower_max = try
    last(turn_rate_depower_range(fcs.body_damping; table = ctrl_tr_table))
catch exc
    exc isa ArgumentError || rethrow()
    NaN
end
# The depower a reply is FLOWN at, which is what every gate and request must read c1 at.
pattern_depower(reply) =
    tos.fly_opt_depower && !isnothing(reply.depower) ?
        awetrim_depower_to_v3kite(reply.depower.value) : fcs.depower_setpoint
# The turn-rate law the retry reads a path against; `reelout_feasibility.jl` looks it up again later.
st.c1_startup = c1_at_depower(fcs.depower_setpoint)
# Resample but never upsample; the lobe lift is rationed to fit the curvature gate.
install_optimized_path!(st::RunState, reply) = begin
    az = collect(Float64.(reply.trajectory.azimuth))
    el = collect(Float64.(reply.trajectory.elevation))
    lift = wing_lift(az, el)
    resample = min(tos.resample_points, length(az) - 1)
    st.c1_startup = c1_at_depower(pattern_depower(reply))
    st.startup_wing_frac = 1.0
    if !isnan(st.c1_startup) && tos.min_feasibility_margin > 0 && any(!=(0), lift)
        for fw in (1.0, 0.75, 0.5, 0.25, 0.0)
            st.startup_wing_frac = fw
            set_path!(fec, az, el .+ fw .* lift; resample)
            check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                   c1 = st.c1_startup, prn = false).margin >=
                tos.min_feasibility_margin && break
        end
        st.startup_wing_frac < 1 &&
            @info @sprintf("Lobe lift held back on the startup path to fit the \
                            curvature gate: %.0f %% of %.2f°.",
                           100 * st.startup_wing_frac, fcs.el_offset_wing)
    else
        set_path!(fec, az, el .+ lift; resample)
    end
    return (az, el)
end
# Every optimizer answer as it arrived, before any lift; installed paths shrink as the tether grows.
"""
    adopt_startup_path!(st)

Take the startup reply as the path of the run: install it (every optimizer answer is kept as it arrived,
before any lift, with where and when it was installed), record its success for a rerun, apply the winch
gain it chose, and re-measure the anchor ratio and the turn-radius request off it.
"""
function adopt_startup_path!(st::RunState)
    st.opt_paths_raw = [install_optimized_path!(st, st.opt_result)]
    # Where each of those was installed: (sim time [s], phase); the startup path goes in before the run.
    st.opt_paths_at = [(0.0, 0)]

    # set_path! REVERSES a path that does not match up_loops, so a mismatch must be caught here.
    st.opt_table = chain_trajectory(opt_chain)
    # Installed above, so applied: stored for a rerun that sends the same requests.
    record_opt_success!(opt_chain)
    apply_optimized_kv!(st.opt_table, 0.0, l_tether)
    st.opt_downloops = st.opt_table["spline"]["downloops"]
    st.opt_power_pred = Float64(st.opt_table["metrics"]["avg_power_W"])
    # The anchor ratio, now measured off the reply; guarded so a request that is off stays off.
    if opt_r_on
        st.opt_r_scale = reelout_anchor_ratio(st.opt_table) * tos.turn_radius_headroom
        st.opt_r_min = min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                            c1 = st.c1_startup)
    end
    isnothing(st.opt_r_min) ||
        @info @sprintf("Turn-radius request for the re-optimizations: %.2f m — the \
                        gate's %.2f m at margin %.2f, x %.3f for the lap's reel-out \
                        (%.1f -> %.1f m) and x %.2f of headroom.",
                       st.opt_r_min, st.opt_r_min / st.opt_r_scale, tos.min_feasibility_margin,
                       reelout_anchor_ratio(st.opt_table),
                       minimum(Float64.(st.opt_table["table"]["distance_radial"])),
                       maximum(Float64.(st.opt_table["table"]["distance_radial"])),
                       tos.turn_radius_headroom)
end
adopt_startup_path!(st)

# ---- Corrected retries of the STARTUP solve: one lever per attempt (ceiling, width, radius) ---- #
# The decisions (which lever, what the answers imply) are `next_lever` & co. of src/startup_retry.jl.

# Read at TOP level: the retry block defines `incumbent_score` only when it runs.
margin_startup = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                        c1 = st.c1_startup, prn = false).margin

# All three startup gates and the record of a rejected curve, defined at top level so the retries below and the
# incumbent's record after them share them.
el_floor_start = fcs.min_elevation + tos.candidate_elevation_margin
score_installed(st::RunState) = begin
    margin = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                    c1 = st.c1_startup, prn = false).margin
    el_ok = minimum(fec.el_path) >= el_floor_start
    height = NaN
    clr_ok = true
    if tos.min_height > 0
        clr = check_pattern_height(fec, l_tether, tos.min_height; prn = false)
        height = clr.height
        clr_ok = clr.ok
    end
    (; margin, el_ok, clr_ok, height,
       ok = margin >= tos.min_feasibility_margin && el_ok && clr_ok)
end
# Defined BEFORE the loop (soft scope); saves rejected curves for examples/plot_trajectory.jl.
save_failed_trajectory(name, az, el; margin = NaN, power = NaN) = begin
    dir = joinpath(@__DIR__, "..", "trajectories")
    mkpath(dir)
    stamp = replace(string(now()), r"[:.]" => "", "T" => "_")[1:15]
    file = joinpath(dir, "$(name)_$stamp.yaml")
    YAML.write_file(file, Dict(
        "name" => name,
        "date" => string(now()),
        "l_tether" => l_tether,
        "min_feasibility_margin" => tos.min_feasibility_margin,
        "margin" => margin,
        "predicted_power_W" => power,
        "azimuth_deg" => collect(Float64.(az)),
        "elevation_deg" => collect(Float64.(el)),
    ))
    @info "Saved failed trajectory to $file (margin $margin)."
end
"""
    retry_startup!(st)

Corrected retries of the STARTUP solve, for a startup path whose turn margin is below
`min_feasibility_margin`: one lever per attempt (`next_lever`), the best path so far installed
in `fec` and in the `opt_*` fields of `st` the rest of the run reads, `incumbent_score` and `inc_*` (the
incumbent, which `startup_incumbent` records afterwards) included.
"""
function retry_startup!(st::RunState)
    st.incumbent_score = score_installed(st)
    st.inc_result, st.inc_table, st.inc_raw = st.opt_result, st.opt_table, st.opt_paths_raw[1]
    ladder = RetryLadder(; m_reply = st.incumbent_score.margin)  # what the answers so far imply
    t_retries = time()
    for attempt in 1:max(Int(tos.startup_retries_max), 0)
        ask = next_lever(ladder, tos, st.inc_raw[1], st.inc_raw[2], opt_r_sent,
                         isnothing(opt_box) ? nothing : opt_box.elevation_min, el_floor_start,
                         margin -> min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                                           margin, c1 = st.c1_startup))
        if isnothing(ask.lever)
            @info @sprintf("Startup retries stop at %d/%d: no radius under the \
                            %.2f m that 422'd can reach past margin %.3f (the \
                            gate wants %.2f), and the ceiling and width levers \
                            are spent.",
                           attempt, Int(tos.startup_retries_max), ladder.bisect_hi,
                           ladder.m_reply * ladder.bisect_hi / ask.prev_ask,
                           tos.min_feasibility_margin)
            break
        end
        (; lever, r_ask, el_cap, az_min, target, prev_ask, inc_top, inc_height, inc_amp,
           el_min_box, bisect_room) = ask
        # `nothing` keeps the session's limits; only a changed side builds a box.
        box_ask = nothing
        isnothing(el_cap) || (box_ask = with_elevation_max(opt_box, el_cap))
        isnothing(az_min) ||
            (box_ask = with_azimuth_amplitude_min(isnothing(box_ask) ? opt_box : box_ask,
                                                  az_min))
        @info @sprintf("The startup path is at margin %.3f, below \
                        min_feasibility_margin = %.2f: retry %d/%d (%s) at \
                        L = %.1f m, turn radius %.2f m (was %.2f m)%s, targeting \
                        margin %.3f%s.",
                       st.incumbent_score.margin, tos.min_feasibility_margin,
                       attempt, Int(tos.startup_retries_max), lever, st.l_set,
                       r_ask, prev_ask,
                       (isnothing(el_cap) ? ", no elevation ceiling" :
                           @sprintf(", elevation ceiling %.1f° (the incumbent \
                                    spans %.1f-%.1f° over a %.1f° floor)",
                                    el_cap, inc_top - inc_height, inc_top,
                                    el_min_box)) *
                       (isnothing(az_min) ? "" :
                           @sprintf(", azimuth half-width >= %.1f° (the \
                                    incumbent's is %.1f°)", az_min, inc_amp)),
                       target,
                       isnan(ladder.bisect_hi) ? "" :
                           @sprintf(" (%s the %.2f m that 422'd)",
                                    bisect_room ? "bisecting below" :
                                        "the radius lever is spent under",
                                    ladder.bisect_hi))
        t_attempt = time()
        local att_result, att_table, att_raw, att_score
        try
            att_result = chain_step(opt_chain,
                                    StepParams(; length = opt_length(tos, st.l_set),
                                               winch_params = winch_first_lap,
                                               min_turn_radius = r_ask,
                                               pattern_limits = box_ask))
            att_table = chain_trajectory(opt_chain)
            att_raw = install_optimized_path!(st, att_result)
            att_score = score_installed(st)
        catch exc
            exc isa HTTP.StatusError && exc.status == 422 || rethrow()
            kind = record_422!(ladder, ask)
            if kind == :ceiling
                # The ceiling moved and failed: never send it (or lower) again; the radius steps next.
                @info @sprintf("Startup retry %d (%s) could not converge (HTTP \
                                422) at %.2f m under a ceiling of %s; the \
                                ceiling lever is spent, %s under the last \
                                converged ceiling (%s).",
                               attempt, lever, r_ask,
                               isnothing(el_cap) ? "none" : @sprintf("%.1f°", el_cap),
                               isnan(ladder.r_asked) ? "re-asking the same radius" :
                                                       "the radius steps next",
                               isnothing(ladder.cap_ok) ? "none" : @sprintf("%.1f°", ladder.cap_ok))
            elseif kind == :width
                # Only the width floor moved and failed: never ask for it (or wider) again.
                @info @sprintf("Startup retry %d (%s) could not converge (HTTP \
                                422) at %.2f m with an azimuth half-width >= \
                                %.1f°; the width lever is spent, the radius \
                                steps next at the last converged floor (%s).",
                               attempt, lever, r_ask, az_min,
                               isnothing(ladder.width_ok) ? "none" :
                                   @sprintf("%.1f°", ladder.width_ok))
            else
                @info @sprintf("Startup retry %d could not converge (HTTP 422) at \
                                %.2f m; bisecting toward the last converged ask \
                                of %.2f m.", attempt, r_ask, prev_ask)
            end
            continue
        end
        @info @sprintf("Startup retry %d measured: margin %.3f%s, lowest point \
                        %.1f m (floor %.0f m), elevation %s, predicted power \
                        %.0f W, %.1f s.",
                       attempt, att_score.margin,
                       att_score.ok ? " clearing all gates" : "",
                       att_score.height, tos.min_height,
                       att_score.el_ok ? "ok" : "BELOW FLOOR",
                       Float64(att_table["metrics"]["avg_power_W"]),
                       time() - t_attempt)
        takes_over = att_score.ok ||
                     (att_score.el_ok && att_score.clr_ok &&
                      att_score.margin > st.incumbent_score.margin)
        if !takes_over && att_score.margin > st.incumbent_score.margin &&
           (!att_score.el_ok || !att_score.clr_ok)
            install_optimized_path!(st, st.inc_result)     # incumbent stays flown
            @warn @sprintf("Startup retry %d reached margin %.3f but dropped \
                            below a floor (clearance %s, elevation %s); wider \
                            cannot recover clearance — retries stop here.",
                           attempt, att_score.margin,
                           att_score.clr_ok ? "ok" : "MISSED",
                           att_score.el_ok ? "ok" : "MISSED")
            break
        elseif takes_over
            record_opt_success!(opt_chain)
            st.incumbent_score = att_score
            st.inc_result, st.inc_table, st.inc_raw = att_result, att_table, att_raw
            apply_optimized_kv!(st.inc_table, 0.0, st.l_set)
            # Only adoption moves these: a discarded retry leaves the `opt_*` state untouched.
            st.opt_result = st.inc_result
            st.opt_table = st.inc_table
            st.opt_downloops = st.inc_table["spline"]["downloops"]
            st.opt_power_pred = Float64(st.inc_table["metrics"]["avg_power_W"])
            st.opt_paths_raw = [st.inc_raw]
            st.opt_paths_at = [(0.0, 0)]
            st.opt_r_scale = reelout_anchor_ratio(st.inc_table) *
                          tos.turn_radius_headroom
            st.opt_r_min = min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                                c1 = st.c1_startup)
            if att_score.ok
                @info @sprintf("Startup path clears the gates at margin %.3f \
                                after %d solves (%.1f s of wall time).",
                               st.incumbent_score.margin, attempt, time() - t_retries)
                break
            end
            @info "Kept retry $attempt as the best-so-far; trying again."
        else
            install_optimized_path!(st, st.inc_result)     # put the incumbent back
            save_failed_trajectory("startup_retry$attempt", att_raw[1],
                                   att_raw[2]; margin = att_score.margin,
                                   power = Float64(att_table["metrics"]["avg_power_W"]))
            @info @sprintf("Startup retry %d gave margin %.3f, no better than \
                            %.3f — keeping the incumbent.",
                           attempt, att_score.margin, st.incumbent_score.margin)
        end
        record_converged!(ladder, ask, att_score.margin)
    end
end
if opt_r_on && !isnan(st.c1_startup) && margin_startup < tos.min_feasibility_margin
    retry_startup!(st)
end

if margin_startup < tos.min_feasibility_margin
    # The incumbent is what the gates will refuse; `incumbent_score` exists exactly when this fires.
    save_failed_trajectory("startup_incumbent", st.inc_raw[1], st.inc_raw[2];
                           margin = st.incumbent_score.margin,
                           power = st.opt_power_pred)
end

if !isnothing(st.opt_result.depower)
    st.depower_flown_opt = awetrim_depower_to_v3kite(st.opt_result.depower.value)
    push!(opt_depower_log,
          (; t = 0.0, l_dp = st.opt_result.depower.value, u_p_equiv = st.depower_flown_opt))
end

# The pattern's own geometry, captured now: with reopt_enabled `fec` holds another path at the end.
"""
    capture_startup_geometry!(st)

The startup pattern's own geometry, captured now (with `reopt_enabled`, `fec` holds another path at the
end), and the prediction timeline that says which path was flown when, so the run is scored against the
path in the air. Refuses a path that flies against `up_loops`.
"""
function capture_startup_geometry!(st::RunState)
    st.n_path_initial = length(fec.az_path)
    st.path_min_h_start = path_min_height(fec, l_tether)
    st.az_c_path = 0.5 * (maximum(fec.az_path) + minimum(fec.az_path))
    st.el_c_path = 0.5 * (maximum(fec.el_path) + minimum(fec.el_path))
    st.az_amp_path = 0.5 * (maximum(fec.az_path) - minimum(fec.az_path))
    st.el_height_path = maximum(fec.el_path) - minimum(fec.el_path)

    # Which path was flown when, so the run is scored against the prediction of the path in the air.
    st.pred_timeline = [(t = 0.0, power = st.opt_power_pred)]
    st.opt_downloops == !fcs.up_loops ||
        error("The optimizer returned a downloops = $(st.opt_downloops) path while this run \
               flies up_loops = $(fcs.up_loops). Change fcs.up_loops or the guess; do \
               not fly it reversed.")
end
capture_startup_geometry!(st)

@info @sprintf("Optimized path: %d points, azimuth %.1f°…%.1f°, elevation \
                %.1f°…%.1f° (centre %.1f°), predicted mean reel-out power %.0f W.",
               length(fec.az_path), minimum(fec.az_path), maximum(fec.az_path),
               minimum(fec.el_path), maximum(fec.el_path), st.el_c_path, st.opt_power_pred)

# The three gates on the installed path; defines `el_floor`, `c1_at` and `phase5_margin` for the loop.
# They read the run's state by name, so it is published first.
publish_run_state!(@__MODULE__, st)
include(joinpath(@__DIR__, "reelout_feasibility.jl"))

# Every path the kite has flown, as installed (lobe lift included), with its phase-5 margin and the
# elevation lift it carried: the candidates `final_margin_min` falls back to for phase 5.
"""
    init_phase5_and_controller!(st)

The record of every path flown, for the phase-5 fallback (`final_margin_min`), and the course controller,
whose dive aims at the pattern centre, which is the OPTIMIZED path's now.
"""
function init_phase5_and_controller!(st::RunState)
    st.p5_history = [(t = 0.0, az = copy(fec.az_path), el = copy(fec.el_path), raw = st.opt_paths_raw[end],
                   margin = phase5_margin(fec.az_path, fec.el_path), el_applied = 0.0)]
    st.p5_fallback_done = false    # checked once, from the stop latch on, at the next crossing
    st.p5_q_az_prev = NaN          # [deg] Q's azimuth from the path centre, last step; arms the crossing gate
    st.p5_fallback = nothing       # (; t, from_margin, to_margin, to_t) when a fallback was blended in

    @info @sprintf("Elevation lift: el_offset_final = %+.2f°, el_offset_lead = %.1f s \
                    (%s), reelout_softstop = %.1f s.",
                   fcs.el_offset_final, fcs.el_offset_lead,
                   fcs.el_offset_lead > 0 ? "anticipates the end of reel-out" :
                                            "starts at the stop latch / phase 5",
                   fcs.reelout_softstop)

    # The dive aims at the pattern centre, which is the OPTIMIZED path's now.
    st.ccs = CourseControllerSettings(fcs; dt = s.dt)
    st.ccs.el_center = st.el_c_path
    st.cc = CourseController(st.ccs)
end
init_phase5_and_controller!(st)


"""
    init_loop_state!(st)

The loop's state that cannot be a `RunState` default because it depends on the run: the lap counter's
starting index, the scored reference, the path resolution, the last commanded depower and the depower
ramp's start. The rest starts at the defaults of `RunState`.
"""
function init_loop_state!(st::RunState)
    st.rel_depower_prev = fcs.depower_setpoint  # the gain reads c1 there
    # fig_8 live lap count: 0 before phase 4, 1 at first entry, +1 per traversal; the post-run `fig8` is another thing.
    st.fig8_idx_prev = fec.last_idx
    st.n_path = length(fec.az_path)
    # The reference TRACKING is scored against: the optimizer's curve, canonicalized and blended like the flown one, never lifted.
    st.raw_az, st.raw_el = prepare_path(st.opt_paths_raw[1]...;
                                        resample = min(tos.resample_points, length(st.opt_paths_raw[1][1]) - 1),
                                        up_loops = fcs.up_loops)
    length(st.raw_az) == st.n_path ||
        error("scored reference has $(length(st.raw_az)) points, the flown path $(st.n_path)")
    # Resolution the path in the air is worth checking at (the reply's own, not `n_path`); updated per install.
    st.chk_points = st.n_path
    # Same mechanism as the path blend, scalar, for the optimizer's rel_depower override.
    st.depower_flown = st.depower_flown_opt    # current blended output
    st.depower_blend_from = st.depower_flown
end
init_loop_state!(st)

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

apply_overrides!(wc, inputs.wc_overrides, "WC_OVERRIDES", string(typeof(wc)), "winch (simulation only)")
# The upper force controller's switching speed was derived from kv when `rc` was built.
isempty(inputs.wc_overrides) ||
    WinchControllers.set_v_sw(rc.ufc, WinchControllers.calc_vro(wc, rc.ufc.f_set))

"""
    run_loop!(st, setup)

The simulation loop: steps the model `s` until its steps or the reel-out and phase 5 are over.
Everything it writes lives in `st`; the run's settings and controllers (`fcs`, `tos`, `s`, `rc`, ...)
come in `setup`, a NamedTuple of the script's globals under the same names, unchanged during the
loop. Passed as an argument, not read as globals, so the loop compiles against their concrete types. `publish_run_state!` hands `st` to
`reelout_results.jl` and the plots afterwards. The `try` stays at the call, so the wall time survives
an early `break` or a throw.
"""
function run_loop!(st::RunState, setup::NamedTuple)
    (; EFFECTIVE_SIM_TIME, F_HIGH_NOMINAL, c1_depower_max, c1_setpoint, cap_wind, dt0,
       el_center_seed, el_floor, extra_steer_delay, fcs, feas, fec, guard_lfc, hold_compliance,
       hook_settle, inflow, margin5, opt_chain, opt_depower_log, opt_r_on, project_set, rc, rcs,
       s, steer_disturbance, steer_gain_factor, steer_gain_feedback_only, tos, winch_reopt, wpc,
       xtrack_offset, xtrack_phase) = setup
    for _ in 1:s.steps
        t = s.sys_state.time
        t - st.final_start >= fcs.final_time && break
        isnan(st.final_start) && t >= EFFECTIVE_SIM_TIME && break

        # L0 attractor guidance -> commanded course [rad]; the lead is re-read every step.
        fec.fes.attractor_distance = attractor_distance(fcs, Float64(s.sys_state.v_app),
                                                        Float64(s.sys_state.l_tether[1]))
        chi_set, az_attr, el_attr, dmin =
            navigate_fig8(fec, Float64(s.sys_state.azimuth),
                          Float64(s.sys_state.elevation))
        if !isnothing(xtrack_offset) && st.cc.phase >= xtrack_phase
            isnan(st.xt_start) && (st.xt_start = t)
            local δ = xtrack_offset(t - st.xt_start)
            if δ != 0
                local na, ne = path_normal(fec, attractor_index(fec))
                az_attr += δ * na / cosd(el_attr)
                el_attr += δ * ne
                chi_set = SimpleKiteControllers._bearing(Float64(s.sys_state.azimuth),
                                                         Float64(s.sys_state.elevation),
                                                         deg2rad(az_attr), deg2rad(el_attr))
            end
            push!(st.xt_t, t); push!(st.xt_delta, δ)
            push!(st.xt_d, signed_cross_track(fec, rad2deg(Float64(s.sys_state.azimuth)),
                                           rad2deg(Float64(s.sys_state.elevation))))
            push!(st.xt_q, fec.last_idx)
            push!(st.xt_phase, st.cc.phase); push!(st.xt_L, Float64(s.sys_state.l_tether[1]))
            push!(st.xt_va, Float64(s.sys_state.v_app)); push!(st.xt_dp, st.rel_depower_prev)
            push!(st.xt_vk, sqrt(max(norm(s.sys_state.vel_kite)^2 - Float64(s.sys_state.v_reelout[1])^2, 0.0)))
        end

        # Entry state machine, descent limiter, feedback fusion, PID and rel_depower: see CourseController.
        heading = Float64(s.sys_state.heading)
        local v_kite = norm(s.sys_state.vel_kite)
        phase_before = st.cc.phase
        # Loop gain is heading_p * c1, so every phase flies heading_p * c1(setpoint)/c1(u_d), u_d rounded for the memo.
        local gain_scale = loop_gain_scale(c1_setpoint, st.rel_depower_prev, c1_depower_max, c1_ctrl_at)
        # Curvature feed-forward plus chord correction, low-passed over ff_tau; see FC_Settings.ff_gain.
        local u_ff, chi_ff, ff_u_next, ff_chi_next =
            feedforward_step(fcs, dt0, fec, st.cc.phase, st.cc.err, Float64(s.sys_state.v_app),
                             Float64(s.sys_state.l_tether[1]), v_kite, dmin, c1_setpoint,
                             gain_scale, st.ff_u_filt, st.ff_chi_filt)
        st.ff_u_filt = ff_u_next
        st.ff_chi_filt = ff_chi_next
        push!(st.ff_log, u_ff)
        push!(st.ff_chi_log, chi_ff)
        local rel_steering, rel_depower, phase = calc_steering(st.cc, chi_set, heading,
            Float64(s.sys_state.course);
            t, elevation = Float64(s.sys_state.elevation),
            v_kite, v_app = Float64(s.sys_state.v_app),
            dmin, tangent = path_tangent(fec), gain_scale, u_ff, chi_ff)
        phase_before == 2 && phase == 3 && (st.transition_start = t)
        # The optimizer's depower from phase 3 on, ramped over path_blend_time; phase 5 below still wins.
        if tos.fly_opt_depower && phase in (3, 4)
            # The entry ladder's depower is the FROM endpoint the first time, so the 2->3 hand-over ramps too.
            if phase_before < 3 && isnothing(st.depower_blend_to)
                st.depower_blend_from = rel_depower
                st.depower_blend_to = st.depower_flown_opt
                st.depower_blend_t0 = t
            end
            local w_dp, dp_flown = blended_depower(st.depower_blend_from, st.depower_blend_to,
                                                   st.depower_blend_t0, t, tos.path_blend_time,
                                                   st.depower_flown_opt)
            st.depower_flown = dp_flown
            w_dp >= 1.0 && (st.depower_blend_to = nothing)
            rel_depower = st.depower_flown
        end
        # Separate from calc_steering's ladder so it can fire the SAME step as a 3->4 transition.
        if phase in (3, 4) && st.reelout_done
            set_phase!(st.cc, 5)
            phase = 5
            isnan(st.final_start) && (st.final_start = t)
        end
        # Ramps depower toward depower_final with the soft-stop, never BELOW the depower the stop latched at.
        if !isnan(st.stop_start)
            rel_depower = stop_depower(fcs, st.stop_dp_entry, st.stop_start, st.stop_T, t)
        elseif phase == 5
            rel_depower = fcs.depower_final
        end
        # Force limiter from the STOP LATCH on: integrates on the force the stopped drum is about to see.
        if fcs.depower_final_max > fcs.depower_final && (phase == 5 || !isnan(st.stop_start))
            ramping = !isnan(st.stop_start) && t - st.stop_start < st.stop_T
            st.dp_final_extra = final_force_extra(fcs, st.dp_final_extra, winch_force(s),
                                                      Float64(s.sys_state.v_app),
                                                      Float64(s.sys_state.v_reelout[1]),
                                                      ramping, s.dt)
            rel_depower = min(rel_depower + st.dp_final_extra, fcs.depower_final_max)
            st.dp_final_extra > st.dp_final_extra_peak && (st.dp_final_extra_peak = st.dp_final_extra)
        end
        chi_cmd = st.cc.chi_cmd
        w_lim = st.cc.w_lim
        w_course = st.cc.w_course
        err = st.cc.err

        # Elevation shift target: `el_offset_final`, latched at the stop latch (or phase 5).
        if !st.lift_on && phase >= 4
            if lift_should_start(fcs, st.stop_start, phase, Float64(s.sys_state.v_reelout[1]), st.l_set)
                st.lift_on = true
                st.lift_t = t
                st.lift_remaining = fcs.reelout_l_max - st.l_set
                @info @sprintf("Elevation lift of %+.2f° starting at t = %.1f s \
                                (%.1f m of reel-out left, phase %d).",
                               fcs.el_offset_final, t, fcs.reelout_l_max - st.l_set, phase)
            end
        end
        el_target = st.lift_on ? fcs.el_offset_final : 0.0

        # fig_8: 1 the instant phase first reaches >= 4, then +1 per traversal, unwrapped across the `mod1` wrap.
        if phase >= 4
            if st.fig8_n == 0
                st.fig8_n = 1
                st.fig8_idx_prev = fec.last_idx
                st.t_phase4 = t   # V1 hook: this run's phase-4 start
                if fcs.first_lap_force_frac < 1
                    rcs.f_high = F_HIGH_NOMINAL * fcs.first_lap_force_frac
                    st.first_lap_f_high_applied = true
                    @info @sprintf("Lap 1: upper force limit held at %.0f N \
                                    (%.0f %% of %.0f N) for this lap.",
                                   rcs.f_high, 100 * fcs.first_lap_force_frac,
                                   F_HIGH_NOMINAL)
                end
            else
                # A step moves Q by a fraction of a point; a jump is Q changing branch.
                st.fig8_idx_progress += lap_index_step(fec.last_idx, st.fig8_idx_prev, st.n_path)
                st.fig8_idx_prev = fec.last_idx
                # Never counted DOWN: Q can slip a fraction of a point backwards at an install.
                st.fig8_n = max(st.fig8_n, 1 + floor(Int, st.fig8_idx_progress / st.n_path))
                # Lap 1 only: the upper force limit is held down; `F_HIGH_NOMINAL` goes back on lap 2.
                if fcs.first_lap_force_frac < 1 && st.fig8_n > 1 && st.first_lap_f_high_applied
                    rcs.f_high = F_HIGH_NOMINAL
                    st.first_lap_f_high_applied = false
                    @info @sprintf("Lap %d: upper force limit back to %.0f N.",
                                   st.fig8_n, F_HIGH_NOMINAL)
                end
            end

            az_lo, az_hi = extrema(fec.az_path)
            el_lo, el_hi = extrema(fec.el_path)
            az_amp, el_half = 0.5 * (az_hi - az_lo), 0.5 * (el_hi - el_lo)
            el_kite = rad2deg(Float64(s.sys_state.elevation))
            if az_amp > 0 && el_half > 0
                el_c = 0.5 * (el_hi + el_lo)
                b = azimuth_bin(fec.az_path[fec.last_idx], az_lo, az_hi, st.n_droop_bins)
                st.droop_n[b] += 1
                st.droop_flown[b] += el_c - el_kite
                st.droop_ref[b] += (el_c - fec.el_path[fec.last_idx]) / el_half
                st.droop_sag[b] += el_kite - fec.el_path[fec.last_idx]
            end
        end

        # ---- Re-optimize the path for the length now being flown (phase 4 only) -------- #
        if tos.reopt_enabled && phase == 4
            l_now = Float64(s.sys_state.l_tether[1])

            # Queue on a lap boundary, never while a solve or a blend is running, never past max_reopt.
            if !st.reopt_pending && isnothing(st.blend_to) && st.reopt_n < tos.max_reopt &&
               st.fig8_idx_progress >= (st.reopt_lap + tos.reopt_every_n_laps) * st.n_path
                try
                    # Clocked from here, so a request that fails while being BUILT still has a start.
                    st.reopt_t_wall_request = time()
                    # Seeds: `nothing` is a warm `/step`, a number a cold `/init` from the guess (see `use_step`).
                    el_seeds = if tos.use_step
                        tos.reopt_blocking ? (nothing, el_center_seed) :
                                             (nothing,)
                    elseif tos.reopt_blocking && tos.reopt_retry_el_offset != 0
                        (el_center_seed,
                         el_center_seed + tos.reopt_retry_el_offset)
                    else
                        (el_center_seed,)
                    end
                    # Asked for under the turn authority the reply will be JUDGED with (depower_final's c1 from phase 5).
                    opt_r_on && (st.opt_r_min =
                        min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                                c1 = c1_at(phase, st)))
                    # The floor moves with the length: box rebuilt per request, `size_box_growth` x the previous install.
                    st.opt_box_now = with_size_box(
                        pattern_limits_from(tos;
                            elevation_min = elevation_min_request(fcs, tos, opt_length(tos, l_now);
                                                                  extra = st.el_min_extra),
                            wind_speed = cap_wind),
                        st.opt_paths_raw[end]..., tos.size_box_growth)
                    for (attempt, el_seed) in enumerate(el_seeds)
                        if isnothing(el_seed)
                            # `min_turn_radius` is re-sent because it MOVES with the length; `nothing` means "keep".
                            chain_step(opt_chain,
                                       StepParams(; length = opt_length(tos, l_now), winch_params = winch_reopt,
                                                  min_turn_radius = st.opt_r_min,
                                                  pattern_limits = st.opt_box_now);
                                       wait = false)
                        else
                            guess_az_r, guess_el_r =
                                figure_eight_path(tos.guess_a, tos.guess_b,
                                                  0.0, el_seed,
                                                  0.0, tos.guess_points)
                            reopt_params = InitParams(; name = tos.name, length = opt_length(tos, l_now),
                                                      winch_params = winch_reopt,
                                                      inflow_conditions = inflow,
                                                      trajectory = Trajectory(collect(guess_az_r),
                                                                              collect(guess_el_r)),
                                                      input_depower = depower_seed(tos, inflow.wind_speed),
                                                      reg_weight = tos.reg_weight,
                                                      detect_simple_bounds = tos.detect_simple_bounds,
                                                      min_turn_radius = st.opt_r_min,
                                                      pattern_limits = st.opt_box_now)
                            # A known failure is served by `opt_chain`, not skipped here: skipping would leave
                            # the chain on the warm lineage, and every later step would miss the cache.
                            reopt_reply = chain_init(opt_chain, reopt_params)
                            chain_step(opt_chain,
                                       StepParams(opt_length(tos, l_now), winch_reopt, reopt_reply.trajectory);
                                       wait = false)
                        end
                        st.reopt_pending = true
                        st.reopt_t_request = t
                        st.reopt_lap = st.fig8_idx_progress / st.n_path
                        st.reopt_next_poll = t + tos.reopt_poll_interval
                        @info @sprintf("Re-optimizing for L = %.0f m at t = %.1f s \
                                        (lap %.1f, request %d of %d, %s)%s%s.",
                                       l_now, t, st.reopt_lap, st.reopt_n + 1, tos.max_reopt,
                                       isnothing(el_seed) ? "warm start" :
                                           @sprintf("guess el %.0f°", el_seed),
                                       isnothing(st.opt_box_now) ? "" :
                                           @sprintf(", box |az| <= %s, el %s..%s, half-span <= %s",
                                                    isnothing(st.opt_box_now.azimuth_max) ? "-" :
                                                        @sprintf("%.1f°", st.opt_box_now.azimuth_max),
                                                    isnothing(st.opt_box_now.elevation_min) ? "-" :
                                                        @sprintf("%.1f°", st.opt_box_now.elevation_min),
                                                    isnothing(st.opt_box_now.elevation_max) ? "-" :
                                                        @sprintf("%.1f°", st.opt_box_now.elevation_max),
                                                    isnothing(st.opt_box_now.elevation_amplitude_max) ? "-" :
                                                        @sprintf("%.1f°", st.opt_box_now.elevation_amplitude_max)),
                                       tos.reopt_blocking ? " — holding the simulation" : "")
                        # Freeze here, so the reply is anchored to `l_now` and not to a length the run drifted to.
                        tos.reopt_blocking || break
                        t_block = time()
                        while (try
                                   chain_status(opt_chain)["state"]
                               catch exc
                                   @warn "Could not reach the optimizer while \
                                          holding; will retry." exception = exc
                                   "solving"
                               end) == "solving"
                            sleep(tos.reopt_poll_interval)
                        end
                        st.reopt_last_solve_s = time() - t_block
                        st.reopt_blocked_s += st.reopt_last_solve_s
                        # Collect on THIS step: the reply is already on the server.
                        st.reopt_next_poll = t
                        # Retry only a solver failure, and only while a seed is left.
                        failed = (try
                                      chain_status(opt_chain)["state"]
                                  catch; "failed"; end) == "failed"
                        (failed && attempt < length(el_seeds)) || break
                        @info @sprintf("  ... failed from %s; retrying from %s.",
                                       isnothing(el_seed) ? "the warm start" :
                                           @sprintf("guess el %.0f°", el_seed),
                                       @sprintf("guess el %.0f°", el_seeds[attempt + 1]))
                    end
                catch exc
                    # A refused request must not take the run with it: the path in the air is still flyable.
                    st.reopt_n += 1
                    push!(st.reopt_events, (; t, l = l_now, status = "request failed",
                                         detail = first(sprint(showerror, exc), 120)))
                    push!(st.reopt_cycles, (; t, l = l_now, status = "request failed",
                                         wall_s = time() - st.reopt_t_wall_request))
                    @warn "Re-optimization request failed; flying on with the \
                           current path." exception = exc
                end
            end

            # Collect: poll rather than block, and validate before installing.
            if st.reopt_pending && t >= st.reopt_next_poll
                st.reopt_next_poll = t + tos.reopt_poll_interval
                local state = try
                    chain_status(opt_chain)["state"]
                catch exc
                    @warn "Could not reach the optimizer; will retry." exception = exc
                    "solving"
                end
                if state != "solving"
                    st.reopt_pending = false
                    st.reopt_n += 1
                    event = (; t, l = l_now, status = state, detail = "")
                    if state == "converged"
                        tab = chain_trajectory(opt_chain)
                        # k_v and input_depower are applied only in the accept gate below, from the `tab` that passes it.
                    # A reply whose blend folds is not flown: a fresh COLD reply is requested, `blend_max_retries` times at most.
                    reject_reason = ""
                    # A clearance/elevation rejection retries the FLOOR, not the guess; `el_min_extra` carries the shortfall.
                    reject_low = false
                    # Frozen for the retry chain; the first entry is the startup solve, which `min_power_frac_prev` skips.
                    prev_install_pred = length(st.pred_timeline) > 1 ?
                        st.pred_timeline[end].power : NaN
                    for blend_attempt in 0:tos.blend_max_retries
                        if blend_attempt > 0
                            st.blend_retries_total += 1
                            # Alternating +/- `reopt_retry_el_offset`, never scaled UP by `blend_attempt`.
                            retry_el_seed = el_center_seed +
                                (reject_low || isodd(blend_attempt) ? 1 : -1) *
                                tos.reopt_retry_el_offset
                            retry_el_min = elevation_min_request(fcs, tos, opt_length(tos, l_now);
                                                                 extra = st.el_min_extra)
                            @info @sprintf("  ... candidate at L = %.0f m rejected \
                                            (%s); cold-restarting from guess el \
                                            %.0f°%s (retry %d of %d), holding the \
                                            simulation.",
                                           l_now, reject_reason, retry_el_seed,
                                           isnothing(retry_el_min) ? "" :
                                               @sprintf(", floor %.1f°%s", retry_el_min,
                                                        st.el_min_extra > 0 ?
                                                            @sprintf(" (+%.1f° for the \
                                                                      shortfall)",
                                                                     st.el_min_extra) : ""),
                                           blend_attempt, tos.blend_max_retries)
                            retry_az, retry_el = figure_eight_path(tos.guess_a,
                                tos.guess_b, 0.0,
                                retry_el_seed, 0.0, tos.guess_points)
                            retry_params = InitParams(; name = tos.name, length = opt_length(tos, l_now),
                                winch_params = winch_reopt, inflow_conditions = inflow,
                                trajectory = Trajectory(collect(retry_az),
                                                        collect(retry_el)),
                                input_depower = depower_seed(tos, inflow.wind_speed),
                                reg_weight = tos.reg_weight,
                                detect_simple_bounds = tos.detect_simple_bounds,
                                min_turn_radius = st.opt_r_min,
                                pattern_limits = with_size_box(
                                    pattern_limits_from(tos;
                                        elevation_min = retry_el_min,
                                        wind_speed = cap_wind),
                                    st.opt_paths_raw[end]..., tos.size_box_growth))
                            retry_reply = chain_init(opt_chain, retry_params)
                            chain_step(opt_chain,
                                       StepParams(opt_length(tos, l_now), winch_reopt, retry_reply.trajectory);
                                       wait = false)
                            t_retry = time()
                            retry_state = "solving"
                            while retry_state == "solving"
                                sleep(tos.reopt_poll_interval)
                                retry_state = try
                                    chain_status(opt_chain)["state"]
                                catch exc
                                    @warn "Could not reach the optimizer while \
                                           retrying a folded blend; will retry." exception = exc
                                    "solving"
                                end
                            end
                            st.reopt_blocked_s += time() - t_retry
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
                            tab = chain_trajectory(opt_chain)
                            # k_v and input_depower are applied only in the accept gate below, see above.
                        end
                        # Re-measure the anchor SCALE off the reply; the radius itself is derived where the request goes out.
                        opt_r_on && (st.opt_r_scale = reelout_anchor_ratio(tab) *
                                                          tos.turn_radius_headroom)
                        # What the optimizer measured, in the request's metres; the gate reads the same curve AT THE ANCHOR.
                        opt_r_reply = opt_float(tab["metrics"], "turn_radius_min_m")
                        r_span = extrema(Float64.(tab["table"]["distance_radial"]))
                        # /trajectory is in RADIANS, unlike the degrees of the structs.
                        new_az = rad2deg.(Float64.(tab["table"]["azimuth"]))
                        new_el = rad2deg.(Float64.(tab["table"]["elevation"]))
                        # Lifted BEFORE the gates, which must score the curve that will be flown.
                        cand_raw = (copy(new_az), copy(new_el))
                        # The rigid lift goes in whole; the lobe lift is rationed to fit the curvature gate.
                        wing_delta = wing_lift(new_az, new_el)
                        n_native = min(tos.resample_points, length(new_az) - 1)
                        # The turn authority THIS reply will be flown with, read off `tab`.
                        cand_c1 = c1_at(phase, tos.fly_opt_depower ?
                            awetrim_depower_to_v3kite(
                                Float64(tab["optimized_parameters"]["input_depower"])) :
                            fcs.depower_setpoint)
                        lifted(fw) = new_el .+ el_target .+ fw .* wing_delta
                        function lifted_margin(e)
                            isnan(feas.c1) && return Inf
                            a, b = prepare_path(new_az, e; resample = n_native,
                                                up_loops = fcs.up_loops)
                            check_pattern_feasible(a, b, l_now, fcs.max_steering;
                                                   c1 = cand_c1, prn = false).margin
                        end
                        wing_frac = 1.0
                        for fw in (1.0, 0.75, 0.5, 0.25, 0.0)
                            wing_frac = fw
                            lifted_margin(lifted(fw)) >= tos.min_feasibility_margin &&
                                break
                        end
                        new_el = lifted(wing_frac)
                        wing_frac < 1 &&
                            @info @sprintf("Lobe lift held back on the path for L = %.0f m \
                                            to fit the curvature gate: %.0f %% of %.2f°.",
                                           l_now, 100 * wing_frac, fcs.el_offset_wing)
                        # TWO resolutions: the CHECKS at the reply's own, what is FLOWN at `n_path` so the lap counter holds.
                        chk_az, chk_el = prepare_path(new_az, new_el;
                            resample = n_native, up_loops = fcs.up_loops)
                        st.chk_points = n_native
                        cand_az, cand_el = prepare_path(new_az, new_el;
                            resample = st.n_path, up_loops = fcs.up_loops)
                        # Canonicalized like `cand_az`/`cand_el`, so `blend_folds` and `blend_paths` pair the same points.
                        cand_from = prepare_path(fec.az_path, fec.el_path;
                            resample = st.n_path, up_loops = fcs.up_loops)
                        # At the CURRENT length, which is what it will be flown at.
                        margin = isnan(feas.c1) ? Inf :
                            check_pattern_feasible(chk_az, chk_el, l_now,
                                fcs.max_steering; c1 = cand_c1, prn = false).margin
                        clearance = path_min_height(chk_az, chk_el, l_now)
                        # Gated against BOTH the startup prediction and the previous install's (`min_power_frac*`).
                        new_pred = Float64(tab["metrics"]["avg_power_W"])
                        cand_folds = blend_folds(tos, cand_from..., cand_az, cand_el)
                        # Raw against raw: the reply's curve against the previous install's, before either carries a lift.
                        cand_size = pattern_size_growth(st.opt_paths_raw[end]..., cand_raw...)
                        # The accept gate: turn margin, clearance, elevation floor, blend fold and power, size growth.
                        gate = gate_candidate(tos, (; margin, clearance, l_now,
                                                    chk_el_min = minimum(chk_el), el_floor,
                                                    folds = cand_folds, new_pred, st.opt_power_pred,
                                                    prev_install_pred,
                                                    power_gate_off = power_gate_off(new_pred),
                                                    size = cand_size, blend_attempt, opt_r_reply,
                                                    r_span, st.opt_r_min))
                        if gate.verdict == :retry
                            # A height shortfall is re-asked at a raised floor; any other retry gets a fresh reply.
                            isnothing(gate.raise) || (st.el_min_extra += gate.raise)
                            reject_reason = gate.reason
                            reject_low = gate.low
                            continue   # at the top
                        elseif gate.verdict == :reject
                            event = (; t, l = l_now, status = "rejected", detail = gate.detail)
                            break
                        else
                            power_gate_off(new_pred) &&
                                @info @sprintf("  ... power gate bypassed at L = %.0f m \
                                                (%.0f W predicted, %.1f m/s < \
                                                power_gate_wind_min %.1f): installing anyway.",
                                               l_now, new_pred, project_set.v_wind,
                                               tos.power_gate_wind_min)
                            st.blend_from = cand_from
                            st.blend_to = (cand_az, cand_el)
                            # The scored reference follows the same ramp, unlifted curve to unlifted curve.
                            st.raw_from = prepare_path(st.raw_az, st.raw_el;
                                resample = st.n_path, up_loops = fcs.up_loops)
                            st.raw_to = prepare_path(cand_raw[1], cand_raw[2];
                                resample = st.n_path, up_loops = fcs.up_loops)
                            st.raw_az, st.raw_el = st.raw_from
                            # Here, not before the gates: a REJECTED reply is not a path the kite ever flies.
                            push!(st.opt_paths_raw, cand_raw)
                            push!(st.opt_paths_at, (t, phase))
                            st.blend_t0 = t
                            # k_v and input_depower move only for the `tab` that made it here.
                            let l_dp = Float64(tab["optimized_parameters"]["input_depower"])
                                st.depower_flown_opt = awetrim_depower_to_v3kite(l_dp)
                                st.depower_blend_from = st.depower_flown
                                st.depower_blend_to = st.depower_flown_opt
                                st.depower_blend_t0 = t
                                push!(opt_depower_log, (; t, l_dp, u_p_equiv = st.depower_flown_opt))
                            end
                            apply_optimized_kv!(tab, t, l_now)
                            record_opt_success!(opt_chain)
                            abs(el_target - st.el_applied) > 1e-6 &&
                                push!(st.el_shift_events,
                                      (; t, delta = el_target - st.el_applied, margin,
                                       status = "carried by an install"))
                            st.el_applied = el_target
                            # Arm the in-air warning again: one warning per SHIFT, not per run.
                            st.el_shift_warned = false
                            # Install the aligned OLD path (w = 0, new point indices) and re-base the lap counter on it.
                            set_path!(fec, st.blend_from[1], st.blend_from[2];
                                      up_loops = fcs.up_loops)
                            st.fig8_idx_prev = fec.last_idx
                            # A no-op while paths are resampled to `n_path`; `fig8_idx_progress` counts POINTS.
                            n_path_new = length(fec.az_path)
                            st.fig8_idx_progress *= n_path_new / st.n_path
                            st.n_path = n_path_new
                            push!(st.pred_timeline, (t = t, power = new_pred))
                            margin5.margin = phase5_margin(chk_az, chk_el)
                            push!(st.p5_history, (t, az = copy(cand_az), el = copy(cand_el), raw = cand_raw,
                                               margin = margin5.margin, st.el_applied))
                            # Not a rejection reason: said once, so a phase 5 flown on the clamp is not a surprise.
                            if !isnan(margin5.margin) &&
                               margin5.margin < tos.min_feasibility_margin && !margin5.warned
                                margin5.warned = true
                                @warn @sprintf("The path installed at %.0f m has a \
                                                curvature margin of %.2f at \
                                                depower_final (%.2f here at \
                                                depower_setpoint), below \
                                                min_feasibility_margin = %.2f: phase 5 \
                                                will fly it with less turn authority \
                                                than any gate has checked.",
                                               l_now, margin5.margin, margin,
                                               tos.min_feasibility_margin)
                            end
                            event = (; t, l = l_now, status = "installed",
                                     detail = @sprintf("margin %.2f%s%s, clearance %.1f m, \
                                                        size x%.2f, %.0f W predicted", margin,
                                                       wing_frac < 1 ?
                                                           @sprintf(" (lobe lift at %.0f %%)",
                                                                    100 * wing_frac) : "",
                                                       isnan(margin5.margin) ? "" :
                                                           @sprintf(" (phase 5: %.2f at %.0f m)",
                                                                    margin5.margin,
                                                                    fcs.reelout_l_max),
                                                       clearance, cand_size.growth, new_pred))
                            break
                        end
                    end
                        end
                    push!(st.reopt_events, event)
                    # Non-blocking: an upper bound on the solve, by at most one `reopt_poll_interval`.
                    push!(st.reopt_cycles, (; t, l = l_now, status = event.status,
                                         wall_s = time() - st.reopt_t_wall_request))
                    # Blocking collects on the SAME step as the request, so the wall time is the figure that counts.
                    @info @sprintf("Re-optimization %d: %s%s (%s).",
                                   st.reopt_n, event.status,
                                   isempty(event.detail) ? "" : " — " * event.detail,
                                   tos.reopt_blocking ?
                                       @sprintf("%.1f s of wall time, held", st.reopt_last_solve_s) :
                                       @sprintf("%.1f s of sim after the request",
                                                t - st.reopt_t_request))
                end
            end
        end

        # Blend: `blend_to` is guaranteed fold-free across all of w by the accept gate, so a plain linear ramp.
        # OUTSIDE the re-optimization block: the in-air lift queues blends too, with re-optimization off and
        # into phase 5; inside it they were reported as delivered but never ran (2026-09-26).
        if phase >= 4 && !isnothing(st.blend_to)
            w = clamp((t - st.blend_t0) / tos.path_blend_time, 0.0, 1.0)
            b_az, b_el = blend_paths(st.blend_from[1], st.blend_from[2],
                                     st.blend_to[1], st.blend_to[2], w)
            set_path!(fec, b_az, b_el; up_loops = fcs.up_loops)
            if !isnothing(st.raw_to)
                st.raw_az, st.raw_el = blend_paths(st.raw_from[1], st.raw_from[2],
                                                    st.raw_to[1], st.raw_to[2], w)
            end
            if w >= 1
                st.blend_from = nothing
                st.blend_to = nothing
                st.raw_from = nothing
                st.raw_to = nothing
            end
        end

        # ---- Deliver the elevation shift in the air, AFTER the re-optimizer, which has first claim on `blend_to` ---- #
        if phase >= 4
            # The shift reaches the kite at an install or, when none is due, as a blend onto the path in the air.
            el_delta = el_target - st.el_applied
            if abs(el_delta) > 1e-6 && isnothing(st.blend_to) && !st.reopt_pending &&
               !(st.fig8_n == st.el_shift_lap && el_target == st.el_shift_target)
                st.el_shift_lap = st.fig8_n
                st.el_shift_target = el_target
                # Scored at `chk_points`, the resolution the path in the air came at; only the CHECK is downsampled.
                chk_n = min(st.chk_points, length(fec.az_path) - 1)
                function shift_margin(e)
                    isnan(feas.c1) && return Inf
                    a, b = prepare_path(fec.az_path, e; resample = chk_n,
                                        up_loops = fcs.up_loops)
                    check_pattern_feasible(a, b, Float64(s.sys_state.l_tether[1]),
                        fcs.max_steering; c1 = c1_at(phase, st), prn = false).margin
                end
                # Rationed down to a quarter of the shift, never below.
                # A rung that clears the margin can still fold `blend_paths` in between, so that is checked too.
                hit = nothing
                margin = NaN
                for fm in (1.0, 0.75, 0.5, 0.25)
                    e = fec.el_path .+ fm * el_delta
                    m = shift_margin(e)
                    isnan(margin) && (margin = m)   # the WHOLE shift's margin, reported
                    if m >= tos.min_feasibility_margin &&
                       !blend_folds(tos, fec.az_path, fec.el_path, fec.az_path, e)
                        hit = (fm, e, m)
                        break
                    end
                end
                # A rung that moves the path by no more than a hundredth of a degree is not a delivery.
                if !isnothing(hit) && abs(hit[1] * el_delta) <= 0.01
                    hit = nothing
                end
                if !isnothing(hit)
                    fm, shifted, hit_margin = hit
                    st.blend_from = (copy(fec.az_path), copy(fec.el_path))
                    st.blend_to = (copy(fec.az_path), shifted)
                    st.blend_t0 = t
                    went_in = fm * el_delta
                    push!(st.el_shift_events, (; t, delta = went_in,
                                            margin = hit_margin,
                                            status = fm < 1 ?
                                                @sprintf("blended in (%.0f %%)", 100 * fm) :
                                                "blended in"))
                    st.el_applied = st.el_applied + went_in
                    st.el_shift_warned = false
                    fm < 1 &&
                        @info @sprintf("Elevation shift rationed to fit the curvature \
                                        gate: %.0f %% of %+.2f°; the rest is retried \
                                        next lap.", 100 * fm, el_delta)
                elseif !st.el_shift_warned
                    push!(st.el_shift_events, (; t, delta = el_delta,
                                            margin, status = "held back"))
                    st.el_shift_warned = true
                    @warn @sprintf("Elevation shift of %+.2f° held back: the curvature \
                                    margin would be %.2f even rationed to a quarter. \
                                    Retrying as the tether grows.", el_delta, margin)
                end
            end
        end

        # ---- Phase-5 path: fall back to an install that phase 5 can fly (`final_margin_min`) ---- #
        # From the stop latch, once no other blend is running; checked once. Phase 5 makes no power,
        # so the smaller, later paths that saturate the steering there buy nothing.
        # Only as Q passes the crossing (its azimuth changes sign about the path centre): the
        # startup path is far taller at full length (attractor up to ~36° vs ~18°), and blended in
        # mid-lobe the kite fell 14° behind it and spun an extra loop (Maasvlakte 8.25 m/s, 2026-09-26).
        local p5_crossing = false
        if fcs.final_margin_min > 0 && !st.p5_fallback_done && (!isnan(st.stop_start) || phase >= 5)
            local az_q = fec.az_path[fec.last_idx] - (minimum(fec.az_path) + maximum(fec.az_path)) / 2
            p5_crossing = !isnan(st.p5_q_az_prev) && signbit(az_q) != signbit(st.p5_q_az_prev)
            st.p5_q_az_prev = az_q
        end
        if fcs.final_margin_min > 0 && !st.p5_fallback_done && p5_crossing &&
           isnothing(st.blend_to) && !st.reopt_pending
            st.p5_fallback_done = true
            # Native margins, as each install computed them: on the 360-point resampled path a
            # 100-point reply reads about half its margin, an artefact of the resampling's kinks.
            local m_now = st.p5_history[end].margin
            if !isnan(m_now) && m_now < fcs.final_margin_min
                local k = findlast(h -> !isnan(h.margin) && h.margin >= fcs.final_margin_min, st.p5_history)
                if isnothing(k)
                    @warn @sprintf("Phase-5 margin %.2f < final_margin_min %.2f and no earlier \
                                    install meets it: flying phase 5 on the current path.",
                                   m_now, fcs.final_margin_min)
                else
                    local h = st.p5_history[k]
                    # The lift the kite carries now, not the one that install was made with.
                    local to = prepare_path(h.az, h.el .+ (st.el_applied - h.el_applied);
                                            resample = st.n_path, up_loops = fcs.up_loops)
                    local from = prepare_path(fec.az_path, fec.el_path;
                                              resample = st.n_path, up_loops = fcs.up_loops)
                    local m_to = h.margin
                    if blend_folds(tos, from..., to...)
                        @warn @sprintf("Phase-5 fallback to the path installed at t = %.1f s \
                                        skipped: the blend would fold.", h.t)
                    else
                        st.blend_from = from
                        st.blend_to = to
                        st.blend_t0 = t
                        st.raw_from = prepare_path(st.raw_az, st.raw_el; resample = st.n_path,
                                                       up_loops = fcs.up_loops)
                        st.raw_to = prepare_path(h.raw[1], h.raw[2]; resample = st.n_path,
                                                     up_loops = fcs.up_loops)
                        st.raw_az, st.raw_el = st.raw_from
                        # Install the aligned current path (w = 0) and re-base the lap counter, as an install does.
                        set_path!(fec, st.blend_from[1], st.blend_from[2]; up_loops = fcs.up_loops)
                        st.fig8_idx_prev = fec.last_idx
                        st.p5_fallback = (; t, from_margin = m_now, to_margin = m_to, to_t = h.t)
                        @info @sprintf("Phase-5 fallback at t = %.1f s: the flown path has a \
                                        phase-5 margin of %.2f, below final_margin_min = %.2f; \
                                        blending to the path installed at t = %.1f s \
                                        (margin %.2f).", t, m_now, fcs.final_margin_min,
                                       h.t, m_to)
                    end
                end
            end
        end

        # REEL_OUT: `reelout_delay` seconds after phase 3, and only until l_set reaches reelout_l_max.
        local v_set = 0.0
        # The gate LATCHES: `reelout_f_trigger` opens it early, and once open it never re-closes.
        if phase >= 3 && !st.reelout_started
            by_timer, by_force = reelout_release(fcs, t, st.transition_start, winch_force(s))
            if by_timer || by_force
                st.reelout_started = true
                st.reelout_start_t = t
                st.reelout_trigger_fired = by_force && !by_timer
                by_force && !by_timer &&
                    @info @sprintf("  ... reel-out released EARLY at t = %.1f s by \
                                    force %.0f N >= %.0f N (%.1f s before the \
                                    %.1f s delay would have).",
                                   t, winch_force(s), fcs.reelout_f_trigger,
                                   st.transition_start + fcs.reelout_delay - t,
                                   fcs.reelout_delay)
            end
        end
        if st.reelout_started && !st.reelout_done
            # The INSTANTANEOUS force: reeling out faster when the kite pulls harder is what regulates the force.
            v_raw = calc_v_set(rc, reel_out_speed(s), winch_force(s), rcs.f_low)
            # Ramps the COMMAND, not the law, from when the gate OPENED; `t_startup` does not do this,
            # but released in proportion to tether load, so the soft-start never overrides the force limiter.
            v_cmd = reelout_command(fcs, v_raw, t, st.reelout_start_t, winch_force(s),
                                    rcs.f_low, rcs.f_high)

            remaining = fcs.reelout_l_max - st.l_set
            # Soft-stop: latch once `reelout_softstop` seconds would cover the rest, then decelerate linearly to 0.
            if isnan(st.stop_start) && fcs.reelout_softstop > 0 && v_cmd > 0 &&
               remaining <= v_cmd * fcs.reelout_softstop
                st.stop_start = t
                st.stop_v_entry = v_cmd
                st.stop_dp_entry = rel_depower
                st.stop_T = 2 * remaining / v_cmd
            end
            # Second stop criterion: N COMPLETE laps by `fig8_idx_progress` (`fig8_n` reads 1 during the first lap).
            if isnan(st.stop_start) && fcs.n_fig_eight > 0 &&
               st.fig8_idx_progress >= fcs.n_fig_eight * st.n_path
                st.stop_reason = "laps"
                if fcs.reelout_softstop > 0 && v_cmd > 0
                    st.stop_start = t
                    st.stop_v_entry = v_cmd
                    st.stop_dp_entry = rel_depower
                    # No remaining distance to solve T from: same nominal duration instead.
                    st.stop_T = 2 * fcs.reelout_softstop
                else
                    st.reelout_done = true   # hard stop, as reelout_l_max does today
                end
            end
            v_set = isnan(st.stop_start) ? v_cmd :
                soft_stop_speed(st.stop_v_entry, t, st.stop_start, st.stop_T)
            st.l_set = min(st.l_set + v_set * s.dt, fcs.reelout_l_max)
            on_timer(rc)
            if st.l_set >= fcs.reelout_l_max
                st.reelout_done = true
                isempty(st.stop_reason) && (st.stop_reason = "length")
            elseif !isnan(st.stop_start) && st.stop_reason == "laps" && t - st.stop_start >= st.stop_T
                st.reelout_done = true   # the soft-stop ramp has run out
            end
        elseif phase < 3
            # Force floor BEFORE reel-out: `guard_lfc` (NOT `rc`, see above) stepped by hand through calc_v_set's setters.
            set_reset(guard_lfc, false)
            set_f_set(guard_lfc, fcs.entry_f_min)
            set_v_sw(guard_lfc, calc_vro(rcs, fcs.entry_f_min) * 1.05)
            set_v_act(guard_lfc, reel_out_speed(s))
            set_tracking(guard_lfc, 0.0)   # bumpless: l_set is otherwise flat here
            set_force(guard_lfc, winch_force(s))
            # Reel-IN only: before reel-out this guard exists to catch a force SAG, never to reel out.
            v_guard = min(get_v_set_out(guard_lfc), 0.0)
            on_timer(guard_lfc)
            if guard_lfc.active
                v_set = v_guard
                st.l_set = st.l_set + v_set * s.dt
            end
        end
        if !isnothing(hold_compliance) && st.reelout_done && phase == 5
            local f_now_h = winch_force(s)
            if isnan(st.hold_f_lp)
                st.hold_f_lp = f_now_h
                st.hold_l0 = st.l_set
            end
            st.hold_f_lp += s.dt / hold_compliance.τF * (f_now_h - st.hold_f_lp)
            local slope = rcs.kv / (2 * sqrt(max(st.hold_f_lp, 1.0)))     # [m/s per N], of v = kv·√F
            v_set = hold_compliance.gain * slope * (f_now_h - st.hold_f_lp) -
                    (st.l_set - st.hold_l0) / hold_compliance.τpos
            st.l_set = st.l_set + v_set * s.dt
        end

        if !isnothing(steer_disturbance)
            local du = steer_disturbance(t)
            rel_steering += du
            push!(st.dist_t, t); push!(st.dist_d, du); push!(st.dist_u, rel_steering)
        end

        # V1 stability hooks: see the STEER_GAIN_FACTOR/EXTRA_STEER_DELAY setup above.
        push!(st.steer_delay_buf, rel_steering)
        push!(st.ff_delay_buf, u_ff)
        local delayed_u = length(st.steer_delay_buf) > extra_steer_delay ?
            popfirst!(st.steer_delay_buf) : rel_steering
        local delayed_ff = length(st.ff_delay_buf) > extra_steer_delay ?
            popfirst!(st.ff_delay_buf) : u_ff
        if !isnan(st.t_phase4) && t - st.t_phase4 >= hook_settle
            local u_scaled = steer_gain_feedback_only ?
                delayed_ff + steer_gain_factor * (delayed_u - delayed_ff) :
                delayed_u * steer_gain_factor
            # calc_steering already clamped its own output to ±max_steering;
            # re-clamp here too, or a gain factor > 1 commands the tape angles
            # it was never calibrated for instead of just saturating earlier,
            # as scaling heading_p itself would.
            rel_steering = clamp(u_scaled, -fcs.max_steering, fcs.max_steering)
        end
        # `v_ff = v_set` removes the position loop's 2 s lag; `acceleration_limit` is `rcs.max_acc`, not the plant's own.
        step!(s; rel_depower, rel_steering, vsm_interval = fcs.vsm_interval,
              set_torque = winch_torque!(wpc, s, st.l_set; v_ff = v_set,
                                         speed_limit = rcs.v_sat,
                                         acceleration_limit = rcs.max_acc))
        st.rel_depower_prev = rel_depower

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
        s.sys_state.var_04 = st.el_c_path         # pattern-centre elevation [deg]
        s.sys_state.var_05 = chi_set           # RAW guidance course [rad]
        s.sys_state.var_06 = rad2deg(err)      # REGULATED error [deg]
        # A weight, not a flag: a step here means entry_d_blend is too narrow.
        s.sys_state.var_07 = abs(chi_set) > deg2rad(fcs.entry_chi_max) ? w_lim : 0.0
        s.sys_state.var_08 = w_course          # course/heading blend weight [-]
        # Whole wing; sys_state.AoA is the centre panel only, which a turn twists away from.
        s.sys_state.var_09 = rad2deg(span_mean_aoa(s.sys))
        az_lo_g, az_hi_g = extrema(fec.az_path)
        el_lo_g, el_hi_g = extrema(fec.el_path)
        push!(st.geom_t, t)
        push!(st.geom_az_c, 0.5 * (az_hi_g + az_lo_g))
        push!(st.geom_az_amp, 0.5 * (az_hi_g - az_lo_g))
        push!(st.geom_el_h, el_hi_g - el_lo_g)
        push!(st.geom_d_raw, path_distance(st.raw_az, st.raw_el,
                                        rad2deg(Float64(s.sys_state.azimuth)),
                                        rad2deg(Float64(s.sys_state.elevation))))
        s.sys_state.fig_8 = Int16(st.fig8_n)      # live lap count
        s.sys_state.var_10 = st.l_set             # tether length setpoint [m]
        s.sys_state.var_11 = v_set             # REEL_OUT speed setpoint [m/s]
        s.sys_state.var_12 = get_state(rc)     # WinchController state (0/1/2)
        s.sys_state.var_13 = get_f_err(rc)     # force error [N], NaN in speed control
        # Not filled anywhere in the model chain: without this the log and the viewer read 0.
        s.sys_state.v_wind_200m .= calc_wind_factor(s.am, 200.0) .* s.sys_state.v_wind_gnd
        # Same for e_mech, which KiteViewers prints in Wh: the running integral of the viewer's p_mech.
        st.e_mech += s.sys_state.winch_force[1] * s.sys_state.v_reelout[1] *
                         s.dt / 3600
        s.sys_state.e_mech = st.e_mech
    end
    return nothing
end
# The loop ALONE: saving the log and scoring it below are not simulation. The `try` is inside
# `@elapsed`, so the loop's wall time survives an early break.
t_wall = @elapsed try
    run_loop!(st, (; EFFECTIVE_SIM_TIME, F_HIGH_NOMINAL, c1_depower_max, c1_setpoint, cap_wind, dt0,
        el_center_seed, el_floor, extra_steer_delay, fcs, feas, fec, guard_lfc, hold_compliance,
        hook_settle, inflow, margin5, opt_chain, opt_depower_log, opt_r_on, project_set, rc,
        rcs, s, steer_disturbance, steer_gain_factor, steer_gain_feedback_only, tos,
        winch_reopt, wpc, xtrack_offset, xtrack_phase))
catch exc
    # `exc`, not `e`: a stray global `e` in the REPL makes the catch binding warn.
    @error "Simulation stopped early at t≈$(round(s.sys_state.time, digits=2))s" exception=(exc, catch_backtrace())
end
# The results file and the plots read the run's state by name; an early break or a throw is published too.
publish_run_state!(@__MODULE__, st)
t_sim = Float64(s.sys_state.time)

@info "Save the log"
save_log(s.logger, log_name; path = output_path, colmeta = timestamp_colmeta())

# Scoring, summary, archive, plots and marker; wrapped so a throw still leaves a FAILED marker, then rethrown.
try
    include(joinpath(@__DIR__, "reelout_results.jl"))
catch exc
    write_run_done("FAILED"; err = exc)
    rethrow()
end
