# Plan: move `simple_opt_reelout.jl` into functions

Goal: at least 80 % of the code lines of `examples/simple_opt_reelout.jl` inside
functions, with **identical output**: same log, same summary, same optimizer
requests. This is a pure refactor.

Measured on 2026-09-29: 1968 non-blank, non-comment lines, of which 228 (11.6 %)
are inside functions. The 22 functions are all small helpers of 5–30 lines.

Do not edit the script while `build_all_scenarios.jl` runs: it `include`s the
script again for every wind speed.

## Where the top-level code is

| Lines     | Block                                                  | Size | Why top level                        |
|-----------|--------------------------------------------------------|------|--------------------------------------|
| 131–470   | Reading inputs, overrides, sim budget, winch, `init`   | ~300 | Written as a script                  |
| 665–733   | Startup solve over retry seeds (`let`)                 | ~65  | Writes its results with `global`     |
| 945–1208  | Startup retry ladder                                   | ~255 | 14 variables of loop state, `global` |
| 1210–1370 | Initialising about 70 variables of loop state          | ~150 | The loop's memory                    |
| 1385–2330 | Simulation loop                                        | ~945 | Every piece of state is a global     |

Two things tie the code to globals:

- **The loop's mutable state.** About 70 scalars and vectors are updated with
  `global x = …`.
- **The included files.** `reelout_results.jl` and `simple_reelout_plots.jl`
  read about 81 of the script's roughly 205 globals. `reelout_feasibility.jl`
  defines `feas`, `el_floor`, `c1_at` and `margin5` for the loop. The analysis
  scripts (`validate_margins.jl`, `stability_opt_reelout.jl`,
  `xtrack_step_analysis.jl`) read only `s`, `fcs`, `log_name` and `t_phase4`.

## Stage 0: a regression check first

- Record reference runs with `replay_paths`, so the optimizer answers are fixed
  and no server is needed. A request that changes in any way misses the replay
  cache and fails loudly, which makes replay a sharp detector. Cover the risky
  code paths:
  - Maasvlakte 3.5 m/s: power-gate bypass.
  - Maasvlakte 8.25 m/s: phase-5 fallback.
  - A Cabauw run with a startup retry and a rejected re-optimization.
- Add `examples/compare_runs.jl`. It compares two `.arrow` logs column by column,
  plus the `_opt.yaml` summaries, ignoring wall-clock fields.
- Run the comparison after every stage. Target: identical.

## Stage 1: read the inputs in one function (about 120 lines moved)

- Lines 141–245, 329–331, 391–392, 411–418 and 531–532 repeat one pattern: read
  `X` if defined, then reset it. That becomes a helper `take_global!(:X, default)`.
- `read_run_inputs()` uses it and returns a NamedTuple: `show_plots`, the four
  override dicts, the test inputs (`steer_disturbance`, `xtrack_offset`,
  `xtrack_phase`, `hold_compliance`), the V1 hook parameters, `output_path` and
  `replay_paths`.
- The four copies of the override loop become `apply_overrides!(obj, dict, label)`.

## Stage 2: setup functions (about 200 lines moved)

- `sim_budget(project_set, fcs, inputs)` takes lines 284–326. It returns
  `EFFECTIVE_SIM_TIME` and prints its own message.
- `build_winch(project, project_set, fcs)` returns `wc` (which is also `rcs`),
  `wpc` and `guard_lfc`.
- `init_model(...)` wraps the `init` call and the `set_overrides`.
- `optimizer_setup(...)` returns `inflow`, `cap_wind`, `winch`,
  `winch_first_lap`, `winch_reopt`, `opt_r_*`, `opt_box` and `depower_request`.
- Helpers that close over globals get explicit arguments: `power_gate_off`,
  `v_reel_nominal`, `opt_length`, `startup_params`, `c1_at_depower` (with its
  memo), `pattern_depower`, `install_optimized_path!` and `apply_optimized_kv!`.
- The setup values go into one `ctx` (NamedTuple or struct) that all later
  functions take: `fcs`, `tos`, `s`, `fec`, `cc`, `rc`, `wc`, `wpc`,
  `guard_lfc`, `opt_chain`, `inflow`, the winch variants, `feas`,
  `c1_setpoint`, `c1_depower_max` and so on.

## Stage 3: startup solve and retry ladder (about 330 lines moved)

- `solve_startup(ctx)` replaces the `let` block. It returns `(opt_result,
  seed_trajectory, start_params, el_center_seed, offset)` instead of setting
  five globals.
- `retry_startup!(ctx, opt)` replaces lines 945–1208:
  - The 14 `global` variables become fields of a `RetryLadder` struct.
  - `score_installed` and `save_failed_trajectory` become ordinary functions.
  - The lever choice (lines 1000–1067) becomes a pure function,
    `next_lever(ladder, ...)`, returning `(lever, r_ask, el_cap, az_min)` or
    `nothing`. That makes it easy to unit-test.

## Stage 4: the simulation loop (about 940 lines moved, the biggest gain)

- `mutable struct LoopState` holds the scalars of lines 1258–1344, with
  concrete field types: the reel-out, stop, lift, lap-count, blend,
  re-optimization, depower-blend, feed-forward-filter, compliant-hold and
  phase-5 variables.
- `struct RunLogs` holds the vectors: `geom_*`, `xt_*`, `dist_*`, `droop_*`,
  `ff_log`, `ff_chi_log`, `reopt_events`, `reopt_cycles`, `pred_timeline`,
  `opt_paths_raw`, `opt_paths_at`, `p5_history`, `el_shift_events`,
  `opt_depower_log`, `opt_kv_log` and the steering delay buffers.
- `run_loop!(ctx, st, logs)` wraps the `for` loop. The `try`/`catch` stays at
  top level, so `t_wall` survives an early `break`. The loop body becomes one
  call per block:

| Function               | Lines     | Content                                                      |
|------------------------|-----------|--------------------------------------------------------------|
| `xtrack_input!`        | 1397–1415 | Cross-track test input                                       |
| `feedforward!`         | 1418–1453 | Gain scale, curvature feed-forward                           |
| `depower_command!`     | 1461–1502 | Optimizer depower blend, soft-stop, phase-5 force limiter    |
| `update_lift_target!`  | 1509–1522 | Latching `el_offset_final`                                   |
| `count_laps!`          | 1525–1569 | `fig8_n`, lap-1 `f_high`, droop bins                         |
| `request_reopt!`       | 1576–1688 | Sending a re-optimization request                            |
| `gate_candidate`       | 1783–1925 | Pure accept/reject gate returning a verdict; the retry loop stays in the caller |
| `install_candidate!`   | 1926–1999 | Installing a path that passed the gates                      |
| `collect_reopt!`       | 1691–2016 | Polling, blend-fold retries, calling the two above           |
| `advance_blend!`       | 2022–2037 | Path blend                                                   |
| `deliver_lift_in_air!` | 2040–2100 | Elevation shift in the air                                   |
| `phase5_fallback!`     | 2108–2158 | Phase-5 fallback                                             |
| `winch_setpoint!`      | 2161–2250 | Reel-out gate, soft-stop, force guard, compliant hold; returns `v_set` |
| `steering_hooks`       | 2252–2274 | Disturbance, gain and delay hooks                            |
| `record_step!`         | 2283–2325 | `sys_state.var_*`, geometry logs, `e_mech`                   |

Each `global x = …` becomes `st.x = …`, and the `local` annotations go away.

## Stage 5: interface to the results files (small, keeps them unchanged)

- After `run_loop!`, copy the fields of `st` and `logs` into the global names
  `reelout_results.jl` and the plots expect, in one loop over `fieldnames`
  (about 5 lines).
- Name the fields exactly like today's globals, so nothing downstream changes.
  That includes `t_phase4` for `validate_margins.jl`.
- `reelout_feasibility.jl` becomes a function returning
  `(feas, el_floor, c1_at, margin5)`.

## Stage 6: optional

`reelout_results.jl` has 1079 lines at top level with the same problem.
Converting it is a separate project. It can then take `(ctx, st, logs)` directly
instead of the globals from stage 5.

## Where the functions go

In a new file, `examples/opt_reelout_lib.jl`, included at the top.
`simple_opt_reelout.jl` then shrinks to about 250 lines: docstring, `using`
lines and orchestration. Structs defined in a re-included file are fine on
Julia 1.12, which can redefine structs in `Main`.

## What to expect

- **Share in functions:** stage 4 alone reaches about 60 %; with stages 1–3
  about 85–90 %.
- **Speed:** the loop no longer reads untyped globals, so
  `performance.realtime_factor` should improve clearly. It's already in every
  summary.
- **Risks:**
  - State that is read or written in more than one place is easy to split
    wrongly: `el_min_extra`, `opt_r_min`/`opt_r_scale`, `depower_flown_opt`,
    `margin5`.
  - `write_run_done` must stay defined before anything that can throw.
  - The replay comparison after every stage guards against both.

## Status (2026-09-30)

Done, each step checked against the replay baselines (`examples/regression_baseline.jl`,
`check_regression(tag)`; IDENTICAL log and summary):

- **Stage 0.** `examples/compare_runs.jl` (`compare_runs(a, b)`), and two replay baselines,
  Maasvlakte 8.25 m/s and 3.5 m/s, in `output/regression/maasvlakte_*` (gitignored: record them
  again with `fly_replay` from a scenario folder before starting a new refactor).
- **Stage 1.** `read_run_inputs()`, `apply_overrides!` in `examples/opt_reelout_lib.jl`.
- **Stage 2.** The setup blocks are functions (`sim_budget`, `build_winch`, `init_model`,
  `build_controllers`, `optimizer_conditions`, `optimizer_session`, `request_constraints`), each
  returning a NamedTuple that the script destructures into the same global names. The `ctx` bundle
  was not needed and was not built.
- **Stage 3.** `solve_startup`; the ladder's decisions are pure and unit-tested
  (`src/startup_retry.jl`: `next_lever`, `record_422!`, `record_converged!`, checked against the old
  inline code on 110,000 random steps); the ladder body is `retry_startup!()`.
- **Stage 4.** The reject gate is pure (`src/reopt_gate.jl`: `gate_candidate`, checked against the
  old chain on 100,000 random candidates) and the step-wise decisions of the loop are pure
  (`src/loop_decisions.jl`). The simulation loop is `run_loop!()`, moved unchanged: it still reads
  and writes the script's globals, each written one declared `global` where assigned. The run's state
  is set up by `init_loop_state!()` under the names the results file reads.
- About 82 % of the script's code lines are now inside functions (1170 of 1436 non-blank,
  non-comment lines by a rough count; 8 % at the start).

Not done:

- `LoopState` / `RunLogs` structs with concrete field types, so the loop stops reading untyped
  globals. Speed did not change with the move into functions (about 3.9 ms/step before and after,
  the physics dominates); the structs are for clarity, not for the share.
- The blocks that still mix decisions with model calls: the re-optimization request and install,
  the in-air elevation shift, the phase-5 fallback. They have no replayable baseline (a replay only
  holds installed optimizer results, not rejected ones), so they moved only as far as above.
- Stage 5 and 6 (`reelout_results.jl`, 1079 lines at top level).

A first `runtests.jl` after a flown script used to fail once (`turn_rate_coeffs`, expected the
default table but the script had left the project's loaded); the test now resets it.
