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

# Globals

The run keeps two: `setup` (see `setup_run`), everything it reads, and `st`, the
`RunState` everything it writes. `startup_feasibility` (the gates of
[`check_startup_path`](@ref), adding `feas`, `margin5`, `c1_at_phase` and
`phase5_margin_at` to `setup`) and `reelout_results.jl`
(scoring, summary YAML, archive, plots, finished-run marker) are functions of
both. Besides them only `REF_PATH` and `LOG_NAME` (for the plots) and the timers
`t_script_start`, `run_script`, `t_wall` and `t_sim` are left in `Main`.

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
using SimpleKiteControllers: opt_length, request_constraints, with_elevation_max,
    with_azimuth_amplitude_min, optimizer_conditions
# The decisions of the startup retries (src/startup_retry.jl).
using SimpleKiteControllers: RetryLadder, next_lever, record_422!, record_converged!
import WinchControllers   # module name, for the wc_overrides refresh (calc_vro)
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: calc_wind_factor
using Statistics: mean   # for reelout_results.jl
using Printf
import Dates
using OrderedCollections: OrderedDict

@info "simple_opt_reelout.jl: reeling out along an externally optimized path."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
# The optimizer client, part of the package but not exported (src/awetrim_client.jl),
# `HTTP.StatusError` for a 422, and YAML for the saved trajectories and optimizer paths.
using HTTP, YAML
using SimpleKiteControllers: Trajectory, InitParams, StepParams, chain_init, chain_step,
    chain_trajectory, record_opt_success!, optimizer_session, solve_startup, reelout_anchor_ratio,
    free_speed_reference, winch_from_wc, depower_seed, awetrim_depower_to_v3kite,
    min_turn_radius_request
# The functions moved out of this script, see Plan_refactor_opt_reelout.md.
include(joinpath(@__DIR__, "opt_reelout_lib.jl"))
# Reference curve and log name for simple_reelout_plots.jl; set by reelout_results.jl, cleared here.
REF_PATH = nothing
LOG_NAME = nothing

"""
    setup_run(inputs) -> NamedTuple

Everything the run READS and never rebinds, built in order: the settings and their overrides, the
plant `s`, the winch and its controllers, the optimizer's conditions and session, the turn-rate laws
(`c1_at_depower`, `c1_ctrl_at`) and the lobe lift. The script keeps the result in the one global
`setup` and hands it to every function below; the loop destructures it (see `run_loop!`).
Mutable members (`fcs`, `wc`, `s`, the `opt_*_log` vectors, ...) are still changed in place. The
startup solve and `startup_feasibility` add their results with `merge`.
"""
function setup_run(inputs)
    (; show_plots, steer_disturbance, xtrack_offset, xtrack_phase, hold_compliance, steer_gain_factor,
       steer_gain_feedback_only, extra_steer_delay, hook_settle, replay_paths) = inputs
    aero_mode = ContinuousAero() # ContinuousAero() or AeroDirect()
    # Tether/bridle structural damping as a ratio of stiffness [s]; see simple_fig8.jl's docstring.
    damping_per_stiffness = 0.001
    project_name = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
    @assert project_name in ("system_reelout_cabauw.yaml", "system_reelout_maasvlakte.yaml") "simple_opt_reelout.jl \
        supports only system_reelout_cabauw.yaml and system_reelout_maasvlakte.yaml, got $project_name"
    sim_time = selected_sim_time() # seconds, or `nothing` for the project's own default
    turbulence = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
    wind_speed = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
    @info "simple_opt_reelout.jl: project = $project_name, sim_time = $(isnothing(sim_time) ? "default" : "$sim_time s"), \
           turbulence = $turbulence, wind_speed = $(isnothing(wind_speed) ? "default" : "$wind_speed m/s")."
    project = project_file(project_name)
    fcs = FC_Settings(fc_settings(project))
    # The turn-rate table the PROJECT names (its `turn_rate_coeffs`), not the one `__init__` loaded
    # through system_fig8_200m.yaml. Two consumers: the CONTROLLER (gain schedule, curvature
    # feed-forward) reads `ctrl_tr_table`; the PATH side (turn-radius requests to the planner, the
    # startup gates, the feasibility checks) reads the session's table. Both are the project's unless
    # the input `path_tr_project` names another system project, whose table
    # then sizes the path: an A/B of the controller's table with the planned path held fixed.
    # Session-wide: a later script that does not reload keeps the path side's table.
    ctrl_tr_table = SimpleKiteControllers._load_turn_rate_table(project)
    reload_turn_rate_table!(isnothing(inputs.path_tr_project) ? project : project_file(inputs.path_tr_project))
    isnothing(inputs.path_tr_project) ||
        @info "Turn-rate tables: controller $(turn_rate_coeffs_file(project)), path side \
               $(turn_rate_coeffs_file(project_file(inputs.path_tr_project))) (path_tr_project)."
    # The optimizer's own settings: server, initial guess, solver knobs, margin.
    tos = TrajOptSettings(traj_opt_settings_file(project))

    # Sweep overrides (the input `fcs_overrides`), and the same for the optimizer's settings
    # (`tos_overrides`), e.g. `reopt_enabled = false` for a test run.
    apply_overrides!(fcs, inputs.fcs_overrides, "fcs_overrides", "FC_Settings", "fcs")
    apply_overrides!(tos, inputs.tos_overrides, "tos_overrides", "TrajOptSettings", "tos")
    # Test input: a steering disturbance `t -> Δu` added after the controller (`steer_disturbance`);
    # `stability_opt_reelout.jl`'s model is validated against the loop's response to it.
    isnothing(steer_disturbance) || @info "Steering disturbance in force (test input)."
    # Test input: a cross-track offset `τ -> δ` [deg], τ the time since phase `xtrack_phase` (default 5)
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
    # `steer_gain_factor` multiplies rel_steering, and
    # `extra_steer_delay` adds a FIFO delay to it, in samples. Both act only from
    # `hook_settle` seconds after phase 4 is first reached, so entry, phase 3 and the
    # early part of phase 4 fly identically in every run of a sweep.
    # `steer_gain_feedback_only` true: scale only the feedback part,
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
    apply_windspeed_override!(project_set, wind_speed)
    l_tether = project_set.l_tether

    # Whether `min_power_frac`/`min_power_frac_prev` are bypassed for a candidate predicting `pred`
    # watts: only below `tos.power_gate_wind_min` mean wind AND only for a NEGATIVE prediction, where
    # the number reports the optimizer's winch model leaving its own domain rather than a bad path.
    # Defined after the wind override, so it gates on the speed actually flown.
    power_gate_off(pred) = pred < 0 && project_set.v_wind < tos.power_gate_wind_min

    # Simulated time to ask `init` for: the reel-out budget under a wind-speed override, see `sim_budget`.
    effective_sim_time = sim_budget(project, project_set, fcs, sim_time, wind_speed, default_v_wind)

    # Arrow log files named after the project's `log_file`; `output_path` redirects them for parallel sweep runs.
    output_path = something(inputs.output_path, normpath(joinpath(@__DIR__, "..", "output")))
    mkpath(output_path)

    # Finished-run marker for outside watchers: removed here, written last, so its presence means "this run is over".
    run_done_file = joinpath(output_path, "last_run_done.txt")
    rm(run_done_file; force = true)
    # `_opt`: never overwrite the lemniscate run's log and summary, the two are each other's baseline.
    log_name = basename(project_set.log_file) * "_opt"

    # ======================== INIT =========================== #

    # ONE WCSettings for BOTH winch loops: the POSITION-mode torque gains (`wpc`) and the speed-controller tuning (`rc`).
    (; wc, wpc, dt0) = build_winch(project, project_set, fcs)
    # Winch overrides for a test run, e.g. `v_sat` or `kv` (`inputs.wc_overrides`): applied
    # just before the simulation loop, so the optimizer plans the path with the unchanged winch.
    rcs = wc                                 # same object, two controllers read it

    # Plant overrides for a diagnostic run (`inputs.set_overrides`) are applied inside.
    s = init_model(project, project_set, fcs, wpc, effective_sim_time; turbulence,
                   aero_mode, damping_per_stiffness, set_overrides = inputs.set_overrides)

    # The controllers, built after `init` so the soft-start ramp begins when reel-out starts; see `build_controllers`.
    (; rc, f_high_nominal, guard_lfc, l_set, fec) = build_controllers(fcs, rcs, s)

    # ================= OPTIMIZED REFERENCE PATH ================== #

    # The conditions of THIS run and the winches the optimizer is sent, see `optimizer_conditions`.
    (; inflow, cap_wind, winch, winch_first_lap, winch_reopt) =
        optimizer_conditions(tos, fcs, project_set, rcs, f_high_nominal)
    # Every reply's optimized gain, so the summary reports what was flown, not only what was sent.
    opt_kv_log = NamedTuple{(:t, :l, :k_v, :at_bound), Tuple{Float64, Float64, Float64, Bool}}[]

    # The seed of the startup solve and the connection to the optimizer, see `optimizer_session`; anchored to the
    # STARTING length, re-optimizing during the run is stage 4, below. `solve_startup_path!` replaces the seed.
    (; el_center_seed_base, el_center_seed, opt_chain) =
        optimizer_session(tos, inflow, replay_paths, log_name)
    # Constraints the solve must respect; the turn radius carries the anchor ratio `L/r` and the gate's headroom.
    (; opt_r_scale, opt_r_min, opt_r_on, opt_r_sent, opt_box) =
        request_constraints(tos, fcs, inflow, cap_wind, opt_length(tos, l_set))

    # One row per depower value the optimizer reports back (startup, each ACCEPTED reopt), for summary and plot.
    opt_depower_log = NamedTuple[]

    # The LOBE lift, applied to every installed path on the reply AS IT ARRIVED, before `el_offset_final`.
    wing_lift(az, el) = lobe_lift(az, el; lift = fcs.el_offset_wing,
                                  mode = fcs.el_offset_wing_mode,
                                  az_full = fcs.el_offset_wing_az,
                                  az_blend = fcs.el_offset_wing_blend)

    # The turn-rate gain at a depower, NaN off the table; memoized because a blend asks every step.
    c1_memo = Dict{Float64, Float64}()
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
    c1_ctrl_memo = Dict{Float64, Float64}()
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
    # The elevation floor of every candidate path, the startup gates' and `check_startup_path`'s.
    el_floor = fcs.min_elevation + tos.candidate_elevation_margin

    return (; inputs, show_plots, steer_disturbance, xtrack_offset, xtrack_phase, hold_compliance,
            steer_gain_factor, steer_gain_feedback_only, extra_steer_delay, hook_settle, replay_paths,
            project_name, turbulence, project, fcs, tos, project_set, l_tether, effective_sim_time,
            output_path, run_done_file, log_name, wc, wpc, dt0, rcs, s, rc, f_high_nominal, guard_lfc,
            l_set, fec, inflow, cap_wind, winch, winch_first_lap, winch_reopt, el_center_seed_base,
            el_center_seed, opt_chain, opt_r_scale, opt_r_min, opt_r_on, opt_r_sent, opt_box,
            opt_kv_log, opt_depower_log, power_gate_off, wing_lift, c1_at_depower, c1_ctrl_at,
            c1_setpoint, c1_depower_max, pattern_depower, el_floor)
end

"""
    write_run_done(setup, st, status; err = nothing)

Write the finished-run marker. Defined HERE, before anything that can throw, and
called from a `catch` as well as from the normal tail of `reelout_results`, so
the marker's absence means "still running" and never "it died" — a watcher that
only ever sees the success path waits forever on a run that crashed (measured: a
GLMakie main-thread error in the plots, long after the simulation had finished
and the log was safely written).

`status` is `ok`, `ok (plots failed)` or `FAILED`; on failure the exception's
first line follows on an `error:` line. The other fields come from `st`, where
`reelout_results` keeps them as soon as they are known: a crash early enough leaves
them at their defaults, and the marker still has to be writable.
"""
function write_run_done(setup, st, status::AbstractString; err = nothing)
    open(setup.run_done_file, "w") do io
        println(io, Dates.format(Dates.now(), "yyyy-mm-dd HH:MM:SS"))
        println(io, "status: ", status)
        isnothing(err) || println(io, "error: ", first(split(sprint(showerror, err), "\n")))
        println(io, "archive: ", st.archive_dir)
        println(io, "log: ", joinpath(setup.output_path, setup.log_name * ".yaml"))
        fig8m = st.fig8m
        println(io, "criteria: ", isnothing(fig8m) ? "n/a" :
                                  isempty(fig8m.criteria_failed) ?
                                  "all $(fig8m.criteria) passed" :
                                  "FAILED: " * join(fig8m.criteria_failed, ", "))
        power = st.opt_power_meas
        println(io, "power: ", isnothing(power) ? "n/a" :
                               @sprintf("%.0f W measured", power))
    end
end

# The caller's inputs, passed as `run_example("simple_opt_reelout.jl"; show_plots = false, ...)`
# (src/script_inputs.jl); a plain `include` runs with the defaults, see `run_input_defaults`.
# The setup-only ones (the *_overrides, path_tr_project, output_path, run_archive) are read as `inputs.<name>`.
setup = setup_run(script_inputs(@__FILE__, run_input_defaults()))

# What phases 3+ fly under `fly_opt_depower`; the fixed setpoint until the first optimizer answer.
st = RunState(; l_set = setup.l_set, opt_r_scale = setup.opt_r_scale, opt_r_min = setup.opt_r_min,
              depower_flown_opt = setup.fcs.depower_setpoint)

"""
    startup_params(setup, el_center) -> InitParams

The startup `/init` request seeded with the guess lemniscate centred at
`el_center` [deg]; everything else comes from `tos`, the inflow and the
first-lap winch.
"""
function startup_params(setup, el_center)
    (; tos, l_set, winch_first_lap, inflow, opt_r_min, opt_box) = setup
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
    startup_solve(setup, params) -> (result, seed_trajectory)

`/init` with `params`, the optional seeding solve at `opt_warm_start_awe_trim`,
then the `/step` under the first-lap winch, the one lap 1 flies. Throws the `HTTP.StatusError` of a
422 unchanged; the caller decides whether that ends the run.
"""
function startup_solve(setup, params)
    (; opt_chain, tos, winch, rcs, l_set, winch_first_lap) = setup
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

"""
    solve_startup_path!(setup, st) -> NamedTuple

The startup solve (`solve_startup`, a 422 retried from `startup_retry_el_offsets` in order); its reply
goes to `st.opt_result`. Returns what `setup` gains from it: the seed it converged from
(`el_center_seed`, `startup_seed_offset`, `guess_az`, `guess_el`) and its wall time,
`opt_startup_solve_s`. The startup solve holds the script; blocking re-optimizations hold the loop
(`reopt_blocked_s`).
"""
function solve_startup_path!(setup, st::RunState)
    (; tos, el_center_seed_base, l_set, winch, inflow) = setup
    t_solve_start = time()
    (; opt_result, el_center_seed, startup_seed_offset, guess_az, guess_el) =
        solve_startup(tos, el -> startup_params(setup, el), params -> startup_solve(setup, params),
                      startup_params(setup, setup.el_center_seed), el_center_seed_base, l_set, winch,
                      inflow)
    st.opt_result = opt_result
    startup_seed_offset == 0 ||
        @warn @sprintf("Startup path solved from a RETRY seed centred at %.0f° \
                        (%+.1f° off guess_el_center): a different optimum than the \
                        shipped guess would have given.", el_center_seed, startup_seed_offset)
    return (; el_center_seed, startup_seed_offset, guess_az, guess_el,
            opt_startup_solve_s = time() - t_solve_start)
end
setup = merge(setup, solve_startup_path!(setup, st))
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
log_startup_reply(setup.fcs, st.opt_result, setup.opt_r_min)

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
log_lobe_lift(setup.fcs, st.opt_result)

# The turn-rate law the retry reads a path against; `startup_feasibility` looks it up again later.
st.c1_startup = setup.c1_at_depower(setup.fcs.depower_setpoint)
# Resample but never upsample; the lobe lift is rationed to fit the curvature gate.
function install_optimized_path!(setup, st::RunState, reply)
    (; tos, fcs, fec, l_tether, wing_lift, c1_at_depower, pattern_depower) = setup
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
    adopt_startup_path!(setup, st)

Take the startup reply as the path of the run: install it (every optimizer answer is kept as it arrived,
before any lift, with where and when it was installed), record its success for a rerun, apply the winch
gain it chose, and re-measure the anchor ratio and the turn-radius request off it.
"""
function adopt_startup_path!(setup, st::RunState)
    (; opt_chain, l_tether, opt_r_on, tos, fcs) = setup
    st.opt_paths_raw = [install_optimized_path!(setup, st, st.opt_result)]
    # Where each of those was installed: (sim time [s], phase); the startup path goes in before the run.
    st.opt_paths_at = [(0.0, 0)]

    # set_path! REVERSES a path that does not match up_loops, so a mismatch must be caught here.
    st.opt_table = chain_trajectory(opt_chain)
    # Installed above, so applied: stored for a rerun that sends the same requests.
    record_opt_success!(opt_chain)
    apply_optimized_kv!(setup, st.opt_table, 0.0, l_tether)
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
adopt_startup_path!(setup, st)

# ---- Corrected retries of the STARTUP solve: one lever per attempt (ceiling, width, radius) ---- #
# The decisions (which lever, what the answers imply) are `next_lever` & co. of src/startup_retry.jl.

# All three startup gates on the path installed in `fec`, shared by the retries and the incumbent's record after them.
function score_installed(setup, st::RunState)
    (; fec, l_tether, fcs, tos, el_floor) = setup
    margin = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                    c1 = st.c1_startup, prn = false).margin
    el_ok = minimum(fec.el_path) >= el_floor
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
# Saves a rejected curve for examples/plot_trajectory.jl.
function save_failed_trajectory(setup, name, az, el; margin = NaN, power = NaN)
    dir = joinpath(@__DIR__, "..", "trajectories")
    mkpath(dir)
    stamp = replace(string(now()), r"[:.]" => "", "T" => "_")[1:15]
    file = joinpath(dir, "$(name)_$stamp.yaml")
    YAML.write_file(file, Dict(
        "name" => name,
        "date" => string(now()),
        "l_tether" => setup.l_tether,
        "min_feasibility_margin" => setup.tos.min_feasibility_margin,
        "margin" => margin,
        "predicted_power_W" => power,
        "azimuth_deg" => collect(Float64.(az)),
        "elevation_deg" => collect(Float64.(el)),
    ))
    @info "Saved failed trajectory to $file (margin $margin)."
end

"""
    retry_startup!(setup, st)

Corrected retries of the STARTUP solve, for a startup path whose turn margin is below
`min_feasibility_margin`: one lever per attempt (`next_lever`), the best path so far installed
in `fec` and in the `opt_*` fields of `st` the rest of the run reads, `incumbent_score` and `inc_*` (the
incumbent, which `startup_incumbent` records afterwards) included.
"""
function retry_startup!(setup, st::RunState)
    (; tos, fcs, opt_chain, opt_r_sent, opt_box, el_floor, winch_first_lap) = setup
    st.incumbent_score = score_installed(setup, st)
    st.inc_result, st.inc_table, st.inc_raw = st.opt_result, st.opt_table, st.opt_paths_raw[1]
    ladder = RetryLadder(; m_reply = st.incumbent_score.margin)  # what the answers so far imply
    t_retries = time()
    for attempt in 1:max(Int(tos.startup_retries_max), 0)
        ask = next_lever(ladder, tos, st.inc_raw[1], st.inc_raw[2], opt_r_sent,
                         isnothing(opt_box) ? nothing : opt_box.elevation_min, el_floor,
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
            att_raw = install_optimized_path!(setup, st, att_result)
            att_score = score_installed(setup, st)
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
            install_optimized_path!(setup, st, st.inc_result)     # incumbent stays flown
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
            apply_optimized_kv!(setup, st.inc_table, 0.0, st.l_set)
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
            install_optimized_path!(setup, st, st.inc_result)     # put the incumbent back
            save_failed_trajectory(setup, "startup_retry$attempt", att_raw[1],
                                   att_raw[2]; margin = att_score.margin,
                                   power = Float64(att_table["metrics"]["avg_power_W"]))
            @info @sprintf("Startup retry %d gave margin %.3f, no better than \
                            %.3f — keeping the incumbent.",
                           attempt, att_score.margin, st.incumbent_score.margin)
        end
        record_converged!(ladder, ask, att_score.margin)
    end
end

"""
    finish_startup!(setup, st)

The corrected retries (`retry_startup!`) for a startup path whose turn margin is below
`min_feasibility_margin`, the record of the incumbent the gates will then refuse, and the depower the
optimizer asked for, logged for the summary.
"""
function finish_startup!(setup, st::RunState)
    (; fec, l_tether, fcs, tos, opt_r_on, opt_depower_log) = setup
    margin_startup = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                            c1 = st.c1_startup, prn = false).margin
    if opt_r_on && !isnan(st.c1_startup) && margin_startup < tos.min_feasibility_margin
        retry_startup!(setup, st)
    end

    if margin_startup < tos.min_feasibility_margin
        # The incumbent is what the gates will refuse; `incumbent_score` exists exactly when this fires.
        save_failed_trajectory(setup, "startup_incumbent", st.inc_raw[1], st.inc_raw[2];
                               margin = st.incumbent_score.margin,
                               power = st.opt_power_pred)
    end

    if !isnothing(st.opt_result.depower)
        st.depower_flown_opt = awetrim_depower_to_v3kite(st.opt_result.depower.value)
        push!(opt_depower_log,
              (; t = 0.0, l_dp = st.opt_result.depower.value, u_p_equiv = st.depower_flown_opt))
    end
end
finish_startup!(setup, st)

# The pattern's own geometry, captured now: with reopt_enabled `fec` holds another path at the end.
"""
    capture_startup_geometry!(setup, st)

The startup pattern's own geometry, captured now (with `reopt_enabled`, `fec` holds another path at the
end), and the prediction timeline that says which path was flown when, so the run is scored against the
path in the air. Refuses a path that flies against `up_loops`.
"""
function capture_startup_geometry!(setup, st::RunState)
    (; fec, l_tether, fcs) = setup
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

    @info @sprintf("Optimized path: %d points, azimuth %.1f°…%.1f°, elevation \
                    %.1f°…%.1f° (centre %.1f°), predicted mean reel-out power %.0f W.",
                   length(fec.az_path), minimum(fec.az_path), maximum(fec.az_path),
                   minimum(fec.el_path), maximum(fec.el_path), st.el_c_path, st.opt_power_pred)
end
capture_startup_geometry!(setup, st)


"""
    startup_feasibility(setup, st) -> (; feas, margin5, c1_at_phase, phase5_margin_at)

The gates that refuse the run (`check_startup_path`) on the installed startup path, at
the depower the pattern is FLOWN at (`pattern_depower`): with fly_opt_depower the kite
flies the optimizer's u_d from phase 3 on. Returns the verdicts `feas`, `margin5`, the
`Phase5MarginState` of the in-air phase-5 check, and the two laws the loop reads off
`feas`: `c1_at_phase(phase, depower | st)`, the c1 to check a path against at time t,
and `phase5_margin_at(az, el)`, what phase 5 will fly a candidate path with.
"""
function startup_feasibility(setup, st::RunState)
    (; fec, fcs, tos, l_tether, c1_at_depower, pattern_depower) = setup
    feas = check_startup_path(fec, fcs, tos; l_tether, depower = pattern_depower(st.opt_result))
    # A cell the table cannot serve falls back to the startup law, see `c1_at`; `st` reads it
    # at the depower currently flown (`st.depower_flown_opt`), when sizing a request.
    c1_at_phase(phase::Integer, depower::Real) =
        c1_at(feas, phase, phase >= 5 ? NaN : c1_at_depower(depower))
    c1_at_phase(phase::Integer, st::RunState) =
        c1_at_phase(phase, tos.fly_opt_depower ? st.depower_flown_opt : fcs.depower_setpoint)
    # NaN when the table could not serve depower_final. See phase5_margin's docstring for
    # why this is NOT comparable to the install's own margin early in the reel-out.
    phase5_margin_at(az, el) = phase5_margin(feas, az, el, fcs.reelout_l_max, fcs.max_steering)
    return (; feas, margin5 = Phase5MarginState(), c1_at_phase, phase5_margin_at)
end
setup = merge(setup, startup_feasibility(setup, st))


# Every path the kite has flown, as installed (lobe lift included), with its phase-5 margin and the
# elevation lift it carried: the candidates `final_margin_min` falls back to for phase 5.
"""
    init_phase5_and_controller!(setup, st)

The record of every path flown, for the phase-5 fallback (`final_margin_min`), and the course controller,
whose dive aims at the pattern centre, which is the OPTIMIZED path's now.
"""
function init_phase5_and_controller!(setup, st::RunState)
    (; fec, fcs, s, phase5_margin_at) = setup
    st.p5_history = [(t = 0.0, az = copy(fec.az_path), el = copy(fec.el_path), raw = st.opt_paths_raw[end],
                   margin = phase5_margin_at(fec.az_path, fec.el_path), el_applied = 0.0)]
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
init_phase5_and_controller!(setup, st)


"""
    init_loop_state!(setup, st)

The loop's state that cannot be a `RunState` default because it depends on the run: the lap counter's
starting index, the scored reference, the path resolution, the last commanded depower and the depower
ramp's start. The rest starts at the defaults of `RunState`.
"""
function init_loop_state!(setup, st::RunState)
    (; fcs, fec, tos) = setup
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
init_loop_state!(setup, st)

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

apply_overrides!(setup.wc, setup.inputs.wc_overrides, "wc_overrides", string(typeof(setup.wc)),
                 "winch (simulation only)")
# The upper force controller's switching speed was derived from kv when `rc` was built.
isempty(setup.inputs.wc_overrides) ||
    WinchControllers.set_v_sw(setup.rc.ufc, WinchControllers.calc_vro(setup.wc, setup.rc.ufc.f_set))


"""
    run_loop!(st, setup)

The simulation loop: steps the model `s` until its steps or the reel-out and phase 5 are over.
Everything it writes lives in `st`; the run's settings and controllers (`fcs`, `tos`, `s`, `rc`, ...)
come in `setup` (see `setup_run`), unchanged during the loop. Passed as an argument, not read as a global, so the loop compiles against their concrete
types. `reelout_results.jl` reads `st` afterwards. The `try` stays at the call, so the wall time survives
an early `break` or a throw. What happens before and after `step!` is the package's
(`step_commands!`, `record_step!`, src/reelout_loop.jl); only the model calls are here.
"""
function run_loop!(st::RunState, setup::NamedTuple)
    (; effective_sim_time, fcs, rcs, wpc) = setup
    model = setup.s
    for _ in 1:model.steps
        t = model.sys_state.time
        t - st.final_start >= fcs.final_time && break
        isnan(st.final_start) && t >= effective_sim_time && break
        # What the package's blocks need of the model, read once: none of them changes it before `step!`.
        plant = (; ss = model.sys_state, dt = model.dt, force = winch_force(model),
                 v_reel = reel_out_speed(model))
        commands = step_commands!(st, setup, plant, t)
        (; rel_depower, rel_steering, v_set) = commands
        # `v_ff = v_set` removes the position loop's 2 s lag; `acceleration_limit` is `rcs.max_acc`, not the plant's own.
        step!(model; rel_depower, rel_steering, vsm_interval = fcs.vsm_interval,
              set_torque = winch_torque!(wpc, model, st.l_set; v_ff = v_set,
                                         speed_limit = rcs.v_sat,
                                         acceleration_limit = rcs.max_acc))
        st.rel_depower_prev = rel_depower
        # After step!, which overwrites parts of sys_state.
        plant = (; ss = model.sys_state, dt = model.dt, aoa = span_mean_aoa(model.sys),
                 wind_factor_200 = calc_wind_factor(model.am, 200.0))
        check_overspeed(setup, plant) && break
        record_step!(st, setup, plant, t, commands)
    end
    return nothing
end
# The loop ALONE: saving the log and scoring it below are not simulation. The `try` is inside
# `@elapsed`, so the loop's wall time survives an early break.
t_wall = @elapsed try
    run_loop!(st, setup)
catch exc
    # `exc`, not `e`: a stray global `e` in the REPL makes the catch binding warn.
    @error "Simulation stopped early at t≈$(round(setup.s.sys_state.time, digits=2))s" exception=(exc, catch_backtrace())
end
t_sim = Float64(setup.s.sys_state.time)

@info "Save the log"
save_log(setup.s.logger, setup.log_name; path = setup.output_path, colmeta = timestamp_colmeta())

# Scoring, summary, archive, plots and marker; wrapped so a throw still leaves a FAILED marker, then rethrown.
include(joinpath(@__DIR__, "reelout_results.jl"))
try
    reelout_results(setup, st, (; t_script_start, run_script, t_wall, t_sim))
catch exc
    write_run_done(setup, st, "FAILED"; err = exc)
    rethrow()
end
nothing
