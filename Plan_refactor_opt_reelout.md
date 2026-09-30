# Plan: move `simple_opt_reelout.jl` into functions

Goal: at least 80 % of the code lines of `examples/simple_opt_reelout.jl` inside
functions, with **identical output**: same log, same summary, same optimizer
requests. This is a pure refactor.

Starting point (2026-09-29): 1968 non-blank, non-comment lines, of which 228
(11.6 %) were inside functions, and about 205 globals. The simulation loop, the
startup retry ladder and the loop state (about 70 variables) were top level and
updated with `global x = …`, and `reelout_results.jl` and
`simple_reelout_plots.jl` read about 81 of those globals.

Do not edit the script while `build_all_scenarios.jl` runs: it `include`s the
script again for every wind speed.

## Status (2026-09-30): the goal is reached

Measured by parsing the files (non-blank, non-comment, non-docstring lines;
"top level" is everything outside a `function` or `struct` definition):

| File                      | Code lines | In functions | In `RunState` | Top level |
|---------------------------|-----------:|-------------:|--------------:|----------:|
| `simple_opt_reelout.jl`   | 589        | 522 (89 %)   | –             | 67 (11 %) |
| `reelout_results.jl`      | 573        | 571 (100 %)  | –             | 2         |
| `simple_reelout_plots.jl` | 363        | 346 (95 %)   | –             | 17        |

`RunState` and the step of the loop are in the package now
(`src/run_state.jl`, `src/reelout_loop.jl`). The remaining top level of the script is the header, the `using` lines, and
about ten orchestration calls: `setup_run`, `RunState(...)`,
`solve_startup_path!`, `adopt_startup_path!`, `finish_startup!`,
`capture_startup_geometry!`, `startup_feasibility`, `init_loop_state!`,
`run_loop!`, `save_log`, `reelout_results`.

Every step was checked against the replay baselines
(`examples/regression_baseline.jl`, `check_regression(tag)`, comparing with
`examples/compare_runs.jl`), with identical log and summary. The baselines for
each step are in `output/regression/` (gitignored: `preglobals_*`,
`postglobals_*`, `postresults_*`, `postfeas_*`, …, all at Maasvlakte 3.5 and
8.25 m/s). Before a new refactor, record them again with `fly_replay` from a
scenario folder.

Branches the replays cannot reach are checked by flying the old and the new
code LIVE (optimizer server up) with the same inputs. The second run is served
from the solution cache, so its answers are the same, and only the cache
counters and `traj_opt.reopt.blocked` may differ:

- `reject_old_8.25` / `reject_new_8.25`:
  `fly_replay("maasvlakte", 8.25, nothing, out; tos_overrides =
  Dict(:min_power_frac_prev => 1.2, :blend_max_retries => 1))`. Covers gate
  retries, cold retries, rejections and a cold retry that did not converge.
- `hooks_old_8.25` / `hooks_new_8.25`: a live run with every test input set
  (`steer_disturbance`, `xtrack_offset` from phase 4, `hold_compliance`,
  `steer_gain_factor = 1.1`, `extra_steer_delay = 2`).

Speed did not change (about 3.9 ms/step before and after). The physics
dominates, so the refactor bought clarity, not speed.

## What was done, by stage

- **Stage 0, regression check.** `compare_runs(a, b)` compares two `.arrow`
  logs column by column plus the `_opt.yaml` summaries, ignoring wall-clock
  fields. Replay baselines exist for Maasvlakte 3.5 m/s (power-gate bypass) and
  8.25 m/s (phase-5 fallback). The planned Cabauw baseline (startup retry plus
  rejected re-optimization) was **not** recorded. The live old/new pairs above
  cover rejected re-optimizations, but not the startup retry ladder.
- **Stage 1, inputs.** Caller inputs are passed as keywords
  (`run_example("simple_opt_reelout.jl"; show_plots = false, ...)`) and read
  by `script_inputs` (`src/script_inputs.jl`), with the defaults in
  `run_input_defaults` (`src/run_inputs.jl`). The planned
  `take_global!`/`read_run_inputs` helpers were not needed. `apply_overrides!`
  is in the package.
- **Stage 2, setup.** `setup_run(inputs)` builds everything once and returns
  the `setup` NamedTuple, using `sim_budget`, `build_winch`,
  `build_controllers`, `optimizer_conditions`, `optimizer_session` and
  `request_constraints` (package) and `init_model` (`opt_reelout_lib.jl`).
  `setup` is extended twice with `merge` (after the startup solve and after
  the feasibility check). It plays the role of the planned `ctx`.
- **Stage 3, startup.** `solve_startup` (package) and
  `solve_startup_path!`/`retry_startup!` (script). The ladder's decisions are
  pure and unit-tested (`src/startup_retry.jl`: `RetryLadder`, `next_lever`,
  `record_422!`, `record_converged!`, checked against the old inline code on
  110,000 random steps).
- **Stage 4, loop.** `mutable struct RunState` (`src/run_state.jl`) holds
  everything the startup functions and the loop write: 115 fields, one struct
  instead of the planned `LoopState` + `RunLogs`, plain data without a model
  type. The script is down to 8 globals, and `st` is the only global holding
  run state. The accept/reject gate is pure (`src/reopt_gate.jl`:
  `gate_candidate`, checked against the old chain on 100,000 random
  candidates), as are the step-wise decisions (`src/loop_decisions.jl`:
  `loop_gain_scale`, `feedforward_step`, `reelout_command`, `soft_stop_speed`,
  …).

  `run_loop!(st, setup)` in the script is 26 lines and holds only the model
  calls: it reads the model into `plant = (; ss, dt, force, v_reel)`, calls
  `step_commands!(st, setup, plant, t)`, then `step!` and `winch_torque!`,
  then reads `plant = (; ss, dt, aoa, wind_factor_200)` again for
  `check_overspeed` and `record_step!`. Everything else is in
  `src/reelout_loop.jl`, one function per block of the step, in the order
  `step_commands!` calls them: `xtrack_input!`, `steering_command!` (with
  `depower_command!`), `update_lift_target!`, `count_laps!`, `reoptimize!`
  (`request_reopt!`, `collect_reopt!` → `gate_and_install!` → `cold_retry!`,
  `evaluate_candidate!`, `install_candidate!`, `apply_optimized_kv!`),
  `advance_blend!`, `deliver_lift_in_air!`, `phase5_fallback!`,
  `winch_setpoint!` (`release_reelout!`, `reelout_speed!`,
  `entry_force_guard!`, `compliant_hold!`) and `steering_hooks!`. None of
  them touches the model, so the package still does not depend on V3Kite.
  Exported: `RunState`, `step_commands!`, `record_step!`, `check_overspeed`,
  `apply_optimized_kv!`. The script's imports lost 42 names, the four re-imported exports included.

  Done in three steps (split in the script, `plant` instead of the model,
  move into `src/`), each checked IDENTICAL on both replay baselines and on
  the live old/new pairs above.
- **Stage 5, results interface.** Not needed in the planned form (copying
  fields back to globals). `reelout_results(setup, st, timing)` and the
  analysis scripts read `st.<field>` and `setup.<field>` directly. The
  feasibility check is `startup_feasibility(setup, st)`, built on
  `check_startup_path`/`check_reelout_feasibility` in
  `src/reelout_feasibility.jl`.
- **Stage 6, `reelout_results.jl`.** Done: split into `score_log`,
  `power_comparison`, `feasibility_block`, `traj_opt_block`,
  `opt_performance`, `recap_block`, `write_summary_files`, `archive_run`,
  `draw_plots` and the entry point `reelout_results`. The summary sections
  shared with `simple_reelout.jl` are in `src/run_summary.jl`. It leaves only
  `REF_PATH` and `LOG_NAME` behind, the hand-over to the plots.
  `simple_reelout_plots.jl` is functions too.

## Open

Optional work. None of it is needed for the goal. Each item needs the replay
check, as before.

1. **Concrete types in `RunState`.** 26 fields are `::Any` and 4 are
   `Vector{Any}` (`opt_result`, `opt_table`, `inc_*`, `az_c_path`, …). This is
   for readability and error detection, not speed (see above).
2. **Where the functions live.** The plan put them in
   `examples/opt_reelout_lib.jl`. In fact that file holds only `init_model`.
   The setup and startup functions (`setup_run`, `solve_startup_path!`,
   `retry_startup!`, …) stayed in the script, which is about 1000 lines, not
   the planned 250.
3. **Setup and startup into the package.** The next candidates for `src/`
   are the startup functions: `retry_startup!` and its helpers use the
   remaining imported internals (`RetryLadder`, `next_lever`, `chain_init`,
   …). `setup_run` calls `init_model` and so needs V3Kite. The model would
   have to be passed in (e.g. an `init_model` function argument), since the
   package must not depend on V3Kite.
4. **Unit tests for the step blocks.** The functions in `src/reelout_loop.jl`
   take hand-made `st`, `setup` and `plant`, so blends, the phase-5 fallback,
   the lap counter and the winch setpoint could be tested without a model,
   like `test_loop_decisions.jl`. Not written yet.
5. **A typed `setup`.** The block functions read about 40 fields of the
   `setup` NamedTuple that `setup_run` builds. A documented struct would make
   that interface explicit.

A first `runtests.jl` after a flown script used to fail once
(`turn_rate_coeffs` expected the default table, but the script had left the
project's table loaded). The test now resets it.
