---
name: reidentify-kite
description: Re-identify the course-loop plant model (turn-rate table and course_loop_model.yaml) after a change of the kite, in order, before any margin is rated or the controller retuned. Use when the kite changed (mass, wing_drag_coeff, geometry, bridle, body damping, aerodynamics, kite_settings file), when check_model_provenance refuses ("does not belong to the kite it flies"), when guided or inner disk margins dropped after a plant change, or before running stability_global.jl / retune_guided.jl after any change to data/kite_settings_*.yaml or the kite: section of a settings file.
---

# Re-identify the plant after a change of the kite

The stability analyses rate the controller on a linear model whose parameters are
identified on the simulated kite. They live in two files, both named in the system
project's `system:` section:

- `turn_rate_coeffs` → `data/turn_rate_coeffs.yaml` (step 1);
- `course_loop_model` → `data/course_loop_model.yaml` (steps 2 – 5) and the kite-correction
  table `data/kite_correction_measured.csv` (step 5).

Every step records the `kite_id` of the kite it flew (`src/kite_fingerprint.jl`): step 1
in each row of the turn-rate table, steps 2 – 5 in the `provenance:` section of the
course-loop model file. `check_model_provenance(project)` compares them with the kite the
project flies. The rating scripts (`stability_opt_reelout.jl`, `stability_global.jl`,
`stability_fig8.jl`, `retune_guided.jl`) call it and refuse to run on a mixed model. Step
`k` calls it with `through = k - 1`, so the steps can only run in order.

The procedure is also on the documentation page `docs/src/examples_identification.md`,
"Re-identifying after a change of the kite". Keep the two in step.

## The rule

**Never retune and never rate margins until steps 1 – 5 are all done.** A margin drop seen
after a plant change is a reason to finish the identification, not to retune. On
2026-10-05 the guided loop was retuned (lead time 1.05 → 1.21 s) on a model with the new
turn-rate law but the old course-loop model. Once steps 2 – 5 were done, the retune was
reverted (back to 1.05 s, commit 8217ddc).

Never bypass `check_model_provenance`. Don't edit a `kite_id` by hand, don't stub out the
call, and don't suggest either one. If it refuses, a step is stale: re-run that step. The
one exception is a backfill that the git history proves. It needs an explicit "yes" from
the user and a comment in the file that names the commits.

## Step 0: What changed, and is it the kite?

1. Run `kite_fingerprint(project)` for the project in question and compare it with
   `git diff` on `data/kite_settings_*.yaml`, `data/settings_*.yaml` (sections `kite:` and
   `kcu:`) and `run.body_damping` in the `fc_settings` files. If a value in the fingerprint
   changed, the kite changed.
2. Run `stale_identification_steps(project)` for every project in `data/system_*.yaml`.
   The first stale step is where to start. If every list is empty, the model is current
   and there is nothing to re-identify. Look for the margin drop elsewhere.
3. Run `kite_id` on all `system_*.yaml` projects. Step 3 flies the fig8 projects and
   `system_reelout_maasvlakte.yaml`; steps 4 and 5 fly `system_fig8_300m.yaml`. They must
   all fly the same kite. If they don't, the change was made to one kite settings file
   only (for example `kite_settings_psm.yaml` but not `kite_settings_psm_kernel.yaml`).
   Ask the user whether that was intended before flying anything.

If the change is a new kite rather than a change of the existing one, copy both files under
new names, enter the new names in the kite's project and set `conditions: system` and
`conditions: dt` (`1/sample_freq`) of the copied turn-rate table.

## Running the steps

Run everything in the kaimon REPL with `include(...)` or `run_example(...)`, from the
project root, without `s=true`: the user watches the runs. Never `cd`. Never use a
separate `julia` process. The run times below come from the docstrings and may be stale,
so time the first step and extrapolate.

**Steps 1 and 2 fly the project selected in the example menu** (`select_project()`), not
the one named in the turn-rate table. Before step 1, call
`set_selected_project("<the project in conditions: system>")` and check it with
`selected_project()`. `build_turn_rate_table.jl` refuses a `conditions:` block that names
another project.

| Step | Script | Writes | Time |
|---|---|---|---|
| 1 | `build_turn_rate_table.jl`, then `plot_c1_c2.jl` | the turn-rate table rows | ~12 min |
| 2 | `identify_kite_delay_scaling.jl` | `kite_dead_time_exp`, `kite_lag_exp` | ~5 min |
| 3 | `identify_pattern_law.jl` | `pattern_delay_ref`, `pattern_delay_exp`, `pattern_v_floor` | ~10 min |
| 4 | `identify_depower_factor.jl` | `pattern_depower_exp` | ~6 min |
| 5 | `identify_kite_correction.jl` | `kite_corr_*` and the kite-correction table | ~4 min |

After each step:

- Read what the script printed: the new value, the old value (`was ...`) and the standard
  error. Report any large jump to the user. Don't silently accept it.
- Check that the step's `kite_id` has been written: `stale_identification_steps(project;
  through = k)` must be empty.
- If a step fails a gate or a fit, find the root cause. Never relax a gate, a tolerance or
  a margin to get past it.

The run scripts in steps 3 – 5 keep their logs (`output/pattern_law/`,
`output/depower_factor/`, `output/kite_correction/`), and each log has a `.kite_id`
record. `fly = false` refits saved logs only when they were flown on the current kite. If
the record is missing, it warns: fly again after a change of the kite.

## Step 6: Validate, then rate and retune

Only when `check_model_provenance(project)` passes for every project you will rate:

1. `measure_course_link.jl` (~15 min) and `plot_frf_validation.jl`: the model should stay
   below every measured margin. If it doesn't, the model is wrong. Go back; don't retune.
2. `stability_global.jl` (~94 s): the worst disk margins of the live settings over every
   archived scenario.
3. `retune_guided.jl` only if the margins after step 2 call for it. The result is a
   proposal: write it to `fc_settings_reelout.yaml` only after the user agrees, then fly
   the regression runs.

## Committing

Commit the identification as one commit: the turn-rate table, `course_loop_model.yaml`,
`kite_correction_measured.csv`, and in the body each step's old → new values. Commit a
retune separately, after it. Commit only when the user asks.
