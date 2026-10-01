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

## Status (2026-09-30): the goal is reached, and the script is only the list of steps

Measured by parsing the files (non-blank, non-comment, non-docstring lines;
"top level" is everything outside a `function` or `struct` definition):

| File                      | Lines | Code lines | In functions | Top level |
|---------------------------|------:|-----------:|-------------:|----------:|
| `simple_opt_reelout.jl`   | 260   | 87         | 23           | 64        |
| `reelout_results.jl`      |       | 572        | 570 (100 %)  | 2         |
| `simple_reelout_plots.jl` |       | 356        | 339 (95 %)   | 17        |
| `src/run_setup.jl`        | 229   | 120        | 120          | 0         |
| `src/startup_path.jl`     | 521   | 377        | 377          | 0         |

For the script the percentage no longer measures anything: its 23 function
lines are `run_loop!`, and the 64 at top level are the `using` lines and the
orchestration calls. Before the setup and the startup moved out (stage 9) it
was 589 code lines, 522 (89 %) of them in functions.

`RunState`, the step of the loop, the setup and the startup are in the package
(`src/run_state.jl`, `src/reelout_loop.jl`, `src/run_setup.jl`,
`src/startup_path.jl`). The script keeps what needs the model: `init_model`
(`examples/opt_reelout_lib.jl`), handed to `setup_run`, and `run_loop!` around
the model's `step!`. Its top level is the header, the `using` lines, and the
orchestration calls, in order: `setup_run`, `RunState(...)`,
`solve_startup_path!`, `log_startup_reply`, `log_lobe_lift`,
`adopt_startup_path!`, `finish_startup!`, `capture_startup_geometry!`,
`startup_feasibility`, `init_phase5_and_controller!`, `init_loop_state!`,
`run_loop!`, `save_log` and `reelout_results` (with `write_run_done` on a
throw). Two pieces of plain code are left between them: `st.c1_startup = …`
after the startup solve, and the `wc_overrides` refresh (`apply_overrides!`
plus `set_v_sw`) before the loop.

Run the script from a fresh REPL once after pulling stage 9: a REPL that ran
the old script has `setup_run` and the startup functions defined in `Main`,
which blocks the script's imports of the package's versions.

## How each step was checked

Every step was checked against the replay baselines
(`examples/regression_baseline.jl`, `check_regression(tag)`, comparing with
`examples/compare_runs.jl`), with identical log and summary. The baselines for
each step are in `output/regression/` (gitignored: `preglobals_*`,
`postglobals_*`, `postresults_*`, `postfeas_*`, …, `posttypes_*`,
`poststartup_*`, all at Maasvlakte 3.5 and 8.25 m/s). Before a new refactor,
run `check_regression` on the unchanged tree first; if it is no longer
IDENTICAL, record the baselines again with `fly_replay` from a scenario folder.
The step baselines above were flown at `damping_per_stiffness = 0.001`. Since
the default is 0.002 (see `CHANGELOG.md`), the references are recorded at
0.002: `maasvlakte_3.5` and `maasvlakte_8.25` (reproduced IDENTICAL by
`checkd002_*`), and the live `reject_d002_8.25` and `hooks_d002_8.25`, the
default `ref` of `check_live`. The 0.001 replay baselines are kept as
`maasvlakte_*_d0.001`.

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
- `reject_pkg_8.25` and `hooks_pkg_8.25`: the same two runs after the loop
  step moved into the package (`src/reelout_loop.jl`), compared with `*_new_*`.
- `reject_poststartup_8.25` and `hooks_poststartup_8.25`: after stages 8 and 9,
  compared with `*_pkg_*`. `hooks` IDENTICAL; `reject` differs only in
  `traj_opt.reopt.blocked` (50.7 against 50.2 s), 4 of its 8 steps sent to the
  server again.

Both cases and their inputs are `LIVE_CASES` in `examples/regression_baseline.jl`;
`check_live(tag; ref)` flies them and compares with the reference tag.

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
- **Stage 2, setup.** `setup_run` builds everything once and returns the
  `setup` NamedTuple, using `sim_budget`, `build_winch`, `build_controllers`,
  `optimizer_conditions`, `optimizer_session` and `request_constraints`
  (package) and `init_model` (`opt_reelout_lib.jl`). `setup` is extended twice
  with `merge` (after the startup solve and after the feasibility check). It
  plays the role of the planned `ctx`. Since stage 9 it is in the package.
- **Stage 3, startup.** `solve_startup` (package) and
  `solve_startup_path!`/`retry_startup!` (in the script until stage 9). The
  ladder's decisions are pure and unit-tested (`src/startup_retry.jl`:
  `RetryLadder`, `next_lever`, `record_422!`, `record_converged!`, checked
  against the old inline code on 110,000 random steps).
- **Stage 4, loop.** `mutable struct RunState` (`src/run_state.jl`) holds
  everything the startup functions and the loop write: 117 fields, one struct
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
  `apply_optimized_kv!`. The script's imports lost 42 names, the four
  re-imported exports included.

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
- **Stage 7, where the functions live.** The plan put them in
  `examples/opt_reelout_lib.jl`. After stage 9 that file holds only the model
  side, `init_model`, with `aero_mode = ContinuousAero()` and
  `damping_per_stiffness = 0.001` as its keyword defaults (they were locals of
  `setup_run`). The script is 260 lines, about the planned 250.
- **Stage 8, concrete types in `RunState`** (2026-09-30). The 26 `::Any` and
  4 `Vector{Any}` fields have the types measured on a run, mostly
  `Union{Nothing, T}`, with the aliases `AzElPath`, `StartupScore`,
  `PowerMark`, `P5Record` and `P5Fallback` in `src/run_state.jl`. Only
  `fig8m` stays abstract (`Union{Nothing, NamedTuple}`, 37 fields read once).
  No baseline reaches the startup retry ladder, so the `inc_*` and
  `incumbent_score` types are taken from the code (`chain_step`,
  `score_installed`), not from a run. Replays `posttypes_*` IDENTICAL; live
  pairs checked together with stage 9.
- **Stage 9, setup and startup into the package** (2026-09-30).
  `setup_run(inputs; init_model)` and `write_run_done` are in
  `src/run_setup.jl`; `startup_params` … `init_loop_state!`, `retry_startup!`
  included, in `src/startup_path.jl`. None is exported; the script imports
  the ones it calls. The model comes in as the `init_model` argument, so the
  package still does not depend on V3Kite; the startup reads only
  `setup.s.dt` of it. The script's imports lost 22 names (`chain_init`,
  `RetryLadder`, `HTTP`, …). The line links of `docs/CacheDesign.md`, stale
  since stage 4, point at the new places. Replays `poststartup_*` IDENTICAL,
  live pairs as above.

## Open

Optional work. None of it is needed for the goal. Each item needs the replay
check, as before.

1. **A baseline for the startup retry ladder.** Done (2026-10-01): the
   startup of stages 8 and 9 is IDENTICAL to the code before them
   (`3a78dc6`) on the `ladder` case, compared on the startup log.

   No scenario reaches the ladder: the full build of 2026-10-01 (22 runs,
   empty caches) had no 422, no seed retry and no ladder. Every startup path
   cleared 0.82 at the first solve; Cabauw 7-10 m/s came closest (0.85-0.91).
   `min_feasibility_margin` alone does not reach it, since it sizes the request
   too (1.3 at headroom 0.85 flew margin 1.74), and neither does a low
   `turn_radius_headroom` alone (0.6 flew 0.96 against 0.82). Both together
   do: the `ladder` entry of `LIVE_CASES`, margin 1.3 at headroom 0.4, at
   Maasvlakte 8.25 m/s. The startup path comes back at 0.979; retries 1, 2
   (radius correction) and 4 (ceiling step) get a 422, and retry 3 (width step)
   converges at 1.041 and takes over (`takes_over`, `record_converged!`). The run
   then stops at the startup gate (1.04 < 1.3), so it leaves no log or summary,
   only its startup.

   Run logs, `src/run_log.jl`: `with_run_log` writes a run's log messages to a
   file as well. `fly_replay` writes `out/run.log`; `build_all_scenarios.jl`
   writes `output/run_logs/<site>_v<wind>.log` and adds `ladder_line` to
   `build_all_scenarios.txt`. `check_live` and `check_regression` compare the
   startup logs too (`compare_startup_logs`, `startup_log_lines`: wall times,
   time stamps, the trajectories folder and the optimizer client's cache and
   server messages left out), so a run that stops in the startup is still
   compared. `ladder_d002_8.25` (only `run.log`) is the reference of the default
   `check_live`.

   The old code had a bug only the ladder reaches: `save_failed_trajectory`,
   then in the script, called `now()` with only `import Dates` in scope, so the
   ladder threw `UndefVarError: now` before recording the incumbent. Stage 9
   fixed it in passing (the package has `using Dates: now`). The old side was
   flown from a worktree at `3a78dc6` with that fix, `damping_per_stiffness =
   0.002`, the run log, and `output/` linked to this repo's, so the new run was
   served from the old run's solution cache.
2. **Unit tests for the step blocks.** The functions in `src/reelout_loop.jl`
   take hand-made `st`, `setup` and `plant`, so blends, the phase-5 fallback,
   the lap counter and the winch setpoint could be tested without a model,
   like `test_loop_decisions.jl`. The same now holds for most of
   `src/startup_path.jl`. Partly done (2026-10-01): `test/test_reelout_loop.jl`
   covers `count_laps!` (lap 1 force limit, the wrap, no counting down),
   `advance_blend!`, `phase5_fallback!` (the crossing, once only, no better
   path, off), `release_reelout!`, `reelout_speed!` (the soft-stop at the
   length, the stop after the laps, with and without a soft-stop),
   `entry_force_guard!` (reel-in only, on a force sag) and the compliant hold
   through `winch_setpoint!`; the winch controllers are the real ones, from
   `build_controllers` on a stand-in plant. Also `deliver_lift_in_air!` (the
   whole shift, once per lap and target, held back and warned once),
   `update_lift_target!`, `depower_command!` (the 2 -> 3 ramp, phase 5 when the
   reel-out is done, the soft-stop ramp, the force limiter) and
   `steering_hooks!` (disturbance, gain factor, feedback only, delay). Not
   yet: the rationed rungs of `deliver_lift_in_air!`, `steering_command!` and
   `xtrack_input!` (they need a full `SysState` and a tracking `fec`), the
   re-optimization chain (`reoptimize!` … `install_candidate!`, which talks to
   `opt_chain`), and `src/startup_path.jl`.
3. **A typed `setup`.** The block functions read about 40 fields of the
   `setup` NamedTuple that `setup_run` builds. A documented struct would make
   that interface explicit. `setup_run` being in the package makes this
   easier: the struct can live next to it.

A first `runtests.jl` after a flown script used to fail once
(`turn_rate_coeffs` expected the default table, but the script had left the
project's table loaded). The test now resets it.
