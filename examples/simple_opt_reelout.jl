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

# Predicted versus measured power

The optimizer predicts a mean reel-out power for its path; reeling out along it
measures one. Both land in the `traj_opt:` section of the run summary with their
ratio. A large gap is the finding, not a bug to hide. With `fly_opt_depower`
(on in the shipped yaml) phases 3 and 4 also fly the optimizer's depower, ramped
in over `path_blend_time`; phase 5 always flies `fcs.depower_final`.

# Re-optimizing while the tether grows

`reopt_enabled` (on in the shipped `data/traj_opt.yaml`; the field defaults to
off, so a run is reproducible without a server) re-anchors the path to the length actually flown:
a request every `reopt_every_n_laps` laps, at most `max_reopt` times, polled via
`/status` and collected from `opt_trajectory` (its table is in RADIANS). While a
solve runs or after one fails, the server keeps serving the previous path.

`use_step: false` repeats the cold STARTUP solve (`/init` from the parametric
guess, then `/step`) at each length. `use_step: true` (shipped) sends `/init`
once and re-optimizes with `/step` alone, WARM-STARTING from the previous optimum
re-anchored to the new length — cheaper, but it follows one branch of a
multi-modal problem and the failure cache cannot key it; with `reopt_blocking`,
a failed warm step falls back to one cold `/init`. The FLOWN path is never fed back as a seed: optimized
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
at the starting length, and phase 5 is checked separately at `reelout_l_max`
under `depower_final` (`margin5`, `c1_at_phase`). With `reopt_enabled` every
re-optimization is the worst case instead: the PHYSICAL radius `1/(c1*u_s)` =
11.35 m is length-independent, so each candidate is checked against the c1 of
the phase it will be flown in.

The margin is no longer only a gate. `min_feasibility_margin` is also SENT with
every request as a minimum turn radius (`min_turn_radius_request`, scaled up by
`turn_radius_headroom` and the lap's reel-out), so the optimizer solves under the
limit its reply will be judged by. A 422 from the server, not a rejected reply,
is now the expected failure. A startup reply that still falls short is re-solved
by `retry_startup!`, up to `startup_retries_max` times: each attempt is a warm
`/step` under one changed lever (`next_lever`: turn radius, elevation ceiling or
width), never a different seed.

# What comes from where

Optimizer settings — server, initial guess, solver knobs, resampling, margin —
are `data/traj_opt.yaml` ([`TrajOptSettings`](@ref)). The CONDITIONS come from
the system project's settings file (wind) and its `wc_settings` (winch law) via
`inflow_from_settings` and `winch_from_wc`. The guess decides whether and where
the solve converges, so it is only moved when the server refuses: a startup 422
is re-sent from guess centres shifted by `startup_retry_el_offsets` (see
[`solve_startup`](@ref)), and a failed blocking COLD re-optimization
(`use_step: false`) once from `guess_el_center + reopt_retry_el_offset`. A
startup path that converges but is too tight keeps its seed (`retry_startup!`).
See `simple_opt_fig8.jl` for the measurements. `fcs.f8_a`/`f8_b`/`el_center` are NOT
flown here; the pattern's centre and extent are measured off the installed path.

# Globals

The run keeps two: `setup` (see `setup_run`), everything it reads, and `st`, the
`RunState` everything it writes. The setup and the startup are the package's
(src/run_setup.jl, src/startup_path.jl): the script hands `setup_run` its
`init_model` (opt_reelout_lib.jl), since the package does not depend on the model,
and calls the startup steps in order. `startup_feasibility` (the gates of
[`check_startup_path`](@ref)) adds `feas`, `margin5`, `c1_at_phase` and
`phase5_margin_at` to `setup`. Left here are the loop around the model's `step!`
(`run_loop!`) and `reelout_results.jl` (scoring, summary YAML, archive, plots,
finished-run marker). Besides `setup` and `st` only `REF_PATH` and `LOG_NAME`
(for the plots) and the timers `t_script_start`, `run_script`, `t_wall` and
`t_sim` are left in `Main`.

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

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
# The run's setup and startup (src/run_setup.jl, src/startup_path.jl), not exported.
using SimpleKiteControllers: setup_run, write_run_done, solve_startup_path!, log_startup_reply,
    log_lobe_lift, adopt_startup_path!, finish_startup!, capture_startup_geometry!,
    startup_feasibility, init_phase5_and_controller!, init_loop_state!
import WinchControllers   # module name, for the wc_overrides refresh (calc_vro)
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: calc_wind_factor
using Statistics: mean   # for reelout_results.jl
using Printf
import Dates
using OrderedCollections: OrderedDict
using YAML   # for reelout_results.jl: the optimizer paths
# The optimizer client's helpers (src/awetrim_client.jl), used by reelout_results.jl, not exported.
using SimpleKiteControllers: free_speed_reference, depower_seed, awetrim_depower_to_v3kite

run_script = basename(@__FILE__)

@info "simple_opt_reelout.jl: reeling out along an externally optimized path."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# V3Kite is torque-only; the winch adapter uses WinchControllers.jl to implement the force dependent speed control.
include(joinpath(@__DIR__, "winch_adapter.jl"))
# The model, which the package cannot build: `init_model`, passed to `setup_run`.
include(joinpath(@__DIR__, "opt_reelout_lib.jl"))
# Reference curve and log name for simple_reelout_plots.jl; set by reelout_results.jl, cleared here.
REF_PATH = nothing
LOG_NAME = nothing

# `run_example(file; kwargs)` includes this script; `script_inputs` reads its kwargs, else the defaults.
setup = setup_run(script_inputs(@__FILE__, run_input_defaults()); init_model)

# What phases 3+ fly under `fly_opt_depower`; the fixed setpoint until the first optimizer answer.
st = RunState(; l_set = setup.l_set, opt_r_scale = setup.opt_r_scale, opt_r_min = setup.opt_r_min,
              depower_flown_opt = setup.fcs.depower_setpoint)

setup = merge(setup, solve_startup_path!(setup, st))
toc("Received the optimized path in: ")
log_startup_reply(setup.fcs, st.opt_result, setup.opt_r_min)
log_lobe_lift(setup.fcs, st.opt_result)
# The turn-rate law the retry reads a path against; `startup_feasibility` looks it up again later.
st.c1_startup = setup.c1_at_depower(setup.fcs.depower_setpoint)
adopt_startup_path!(setup, st)
finish_startup!(setup, st)
capture_startup_geometry!(setup, st)
setup = merge(setup, startup_feasibility(setup, st))
init_phase5_and_controller!(setup, st)
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
