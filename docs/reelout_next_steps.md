# Reel-out: where this stands and what is next (2026-08-18, end of session)

Working notes for picking the elevation-correction work back up on another machine.
The dated measurements behind all of it are in
[fig8_tuning_log.md](fig8_tuning_log.md); this file is only the state and the plan.

## What is in the working tree

Uncommitted on top of `79ffed4`:

- `src/fc_settings.jl` — five new `FC_Settings` fields, all defaulting to `0` (off),
  so nothing outside `examples/simple_opt_reelout.jl` changes behaviour:
  `el_bias_gain`, `el_bias_gain_final`, `el_bias_max`, `el_offset_final`,
  `el_offset_lead`.
- `examples/simple_opt_reelout.jl` — the elevation-bias learner (per-lap update, the
  `el_target`/`el_applied` bookkeeping, the in-air blend, the skip of the lap the
  lift lands in) and the `traj_opt.el_bias` block in the run summary.
- `data/fc_settings_reelout.yaml` — `el_bias_gain: 0.5`, `el_bias_gain_final: 1.0`,
  `el_bias_max: 4.0`, `el_offset_final: 1.5`, `el_offset_lead: 0.0`.
- `docs/fig8_tuning_log.md` — the session's findings.
- `data/settings_reelout_150m.yaml` — NOT from this work; it was already modified.

Tests were last run green (236/236) before the `el_offset_final`/`el_offset_lead`
edits. Re-run `include("test/runtests.jl")` first thing.

## Where the run stands

150 -> 380 m, 6 m/s, no turbulence, `attractor_dist: 8.0`: RMS d ~1.5 deg, min
elevation 8.3 deg, ~8508 W at 0.98-0.99 x predicted, all 8 criteria passing, no
lap-counter ringing, no branch flips, no steering saturation events at the
crossing. The elevation correction converges on the sag (~1.3 deg at 380 m) and
the 1.5 deg lift lands at the phase 4 -> 5 transition.

## Open, in the order I would take them

1. **Why does `el_offset_lead > 0` make the kite circle in phase 5?** Three runs
   with an 8 s lead accumulated +1.39, -0.63 and +1.39 turns with a one-sided
   steering bias; four runs without it sat at -0.07. It is not turn authority —
   the circling runs saturate LESS (5.4 % of phase 5 against 14.6 %). Next probe: a
   SHORT lead, 2-3 s, so the step lands inside the deceleration rather than 19 m
   before it. Circling there too means the lift during reel-out is the problem
   itself; clean means it is about how much reel-out remains under the step. Needs
   2-3 runs per setting because of the scatter below.
2. **The dip at the start of phase 5** is unfixed with the lead off: the run's
   lowest point is 3.8 s into phase 5, inside the 4 s blend that is still ramping
   the lift in (min elevation 8.3 deg against 8.7 with the lead). Alternative to a
   lead: give the lift its own, longer ramp instead of `path_blend_time`, or enable
   `reelout_softstop` (currently 0.0, marked "being tuned") — the code already
   latches the lift on `stop_start` when it exists.
3. **The sag is not uniform**, so the rigid shift is now the limiting assumption:
   at 380 m the bottom of the pattern is ~2.3 deg low where the top is ~0.4 deg.
   A per-point correction is only worth trying where the steering is NOT saturated;
   at the tight shoulder it is bounded by turn authority, not by the reference.
4. **Turn authority drops 22 % at `depower_final`** (`c1` 0.2752 -> 0.2133 between
   depower 0.275 and 0.328), and the feasibility gate is computed with the phase-4
   `c1`. Phase 5 is therefore flown with a margin nobody checked. Worth making the
   gate depower-aware, or at least printing the phase-5 margin at startup.

## Two traps that cost time in this session

- **The runs do not repeat.** Byte-identical settings and code gave RMS d 1.25 vs
  1.61 deg and 8 vs 9 laps, because the optimizer server answers the same request
  with slightly different paths (margin 1.00 vs 0.99 at the same length). Judge any
  change over 2-3 runs, never one, and read `traj_opt.reopt.events` when two runs
  disagree.
- **`output/reelout_150m_opt.{arrow,yaml}` is overwritten by every run**, and a run
  started from the REPL while you are reading them will swap them under you. Every
  run is archived under `output/archives/<timestamp>/` with its settings — compare
  archives, not `output/`.
- Adding a field to `FC_Settings` in a live session orphans the methods that
  dispatch on it: `Base.include` `fc_settings.jl`, then `course_controller.jl`,
  `traj_opt_settings.jl` and `optimization.jl`, or the run dies at
  `CourseControllerSettings(fcs; dt = s.dt)`.
