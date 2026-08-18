# Reel-out: where this stands and what is next (2026-08-18, end of session)

Working notes for picking the elevation-correction work back up on another machine.
The dated measurements behind all of it are in
[fig8_tuning_log.md](fig8_tuning_log.md); this file is only the state and the plan.

## What is in the working tree

The elevation-correction work described here is COMMITTED (`3dae201`, `1886f80` on
top of `79ffed4`): the five `FC_Settings` fields (`el_bias_gain`,
`el_bias_gain_final`, `el_bias_max`, `el_offset_final`, `el_offset_lead`, all
defaulting to `0` so nothing outside `examples/simple_opt_reelout.jl` changes
behaviour), the elevation-bias learner and its `traj_opt.el_bias` summary block,
the `data/fc_settings_reelout.yaml` values (`0.5` / `1.0` / `4.0` / `1.5` / `0.0`)
and the tuning-log entries.

Uncommitted, from the session of 2026-08-18 (evening), items 4, 1 and 2 below:

- `examples/simple_opt_reelout.jl` —
  1. the phase-5 turn-authority diagnostics (`c1_final`, `c1_at`,
     `phase5_margin`, the startup print and warning, the per-install
     `(phase 5: x.xx at 380 m)` note, `traj_opt.feasibility.c1_final` /
     `margin_final` / `margin_final_flown`);
  2. **the in-air shift gate fix** (`chk_points`) — the one change that moves the
     run's numbers, see the tuning-log entry "The in-air elevation-shift gate
     measured the SAMPLING, not the curve";
  3. the lift diagnostics (`lift_t_s`, `lift_remaining_m`, `lift_deg`,
     `lift_lead_s` and the `shift_delivery` block in the summary), plus the fix
     that re-arms `el_shift_warned` at an install so a held-back shift is not
     silent for the rest of the run;
  4. `output/last_run_done.txt`, a finished-run marker written last and removed at
     startup, so a watcher can block on the run instead of guessing a duration
     (see CLAUDE.md, "Waiting for a run to finish").
- `docs/fig8_tuning_log.md` — the entries that measure all of it.
- `CLAUDE.md` — how to wait for a run.

Tests were last run green (236/236) before the `el_offset_final`/`el_offset_lead`
edits. Re-run `include("test/runtests.jl")` first thing — nothing since then
touched `src/`, but it costs ~7 s.

## Where the run stands

150 -> 380 m, 6 m/s, no turbulence, `attractor_dist: 8.0`: RMS d ~1.5 deg, min
elevation 8.3 deg, ~8508 W at 0.98-0.99 x predicted, all 8 criteria passing, no
lap-counter ringing, no branch flips, no steering saturation events at the
crossing. The elevation correction converges on the sag (~1.3 deg at 380 m) and
the 1.5 deg lift lands at the phase 4 -> 5 transition.

## Open, in the order I would take them

Every item this list started with is answered, and item 5 — added below, and the
parent of 1 and 2 — is fixed. Read item 5 first: it invalidates elevation
measurements taken before 2026-08-18 22:47. What is left is in "Where to go next"
at the end.

1. **Why does `el_offset_lead > 0` make the kite circle in phase 5?** — ANSWERED,
   and the premise was wrong on both halves. The lead was **inert**: a 2.5 s lead
   latched the lift 2.5 s early (t = 122.2 s, 6.1 m of reel-out left, recorded as
   `el_bias.lift_t_s`) and produced a run BIT-IDENTICAL to lead 0 in every logged
   channel, because the lift could only ever reach the kite at a re-optimization
   install and the same install carried it either way. That is the in-air gate bug
   (item 5). With the gate fixed the lead is no longer inert but buys nothing:
   lead 2.5 vs lead 0 differ by less than the criteria margins, and the lead run
   FAILS the pattern-size criterion where lead 0 passes. **Keep `el_offset_lead`
   at 0.** Nor was there circling: net course over phase 5 drifts -0.5 turns, but
   the kite crosses the pattern centre 3 times in those 25 s and flies a wider
   pattern than the old runs did — "turns" is a drift metric, not a loop detector.
   The earlier session's +1.39-turn runs were an 8 s lead under the BROKEN gate,
   where the lift got in via a mid-reel-out install; re-measure them before
   trusting that number.
2. **The dip at the start of phase 5** — FIXED as a side effect of item 5. The
   lowest point was 4.9 s into phase 5 at 8.45 deg, inside the blend still ramping
   the lift in; it is now 10.57 deg and 23.5 s in, because the correction arrives
   continuously through the run instead of in one step at the last install. Whole-run
   min elevation 8.4 -> 8.8 deg. `reelout_softstop` was never needed for this.
3. **The sag is not uniform**, so the rigid shift is now the limiting assumption:
   at 380 m the bottom of the pattern is ~2.3 deg low where the top is ~0.4 deg.
   — DONE via `el_offset_wing`, a smoothstep lift in azimuth (zero in the centre,
   +1.5 deg past |azimuth| = 10, ramped over 8 deg). Centre-to-lobe droop past
   300 m falls 1.36 -> 0.53 deg, the run's lowest point rises 8.8 -> 10.0 deg and
   steering saturation drops 6 -> 4 %, for 0.26 % of the power and no curvature
   margin at all (0.94 -> 0.94 at 150 m). The worry about saturated shoulders did
   not materialise — saturation went DOWN, so the reference, not turn authority,
   was the binding constraint there. See the tuning-log entry "Lifting the LOBES
   flattens the sag a rigid shift cannot reach"; the ramp width is the parameter
   that matters and it is not monotone.
4. **Turn authority drops 22 % at `depower_final`** (`c1` 0.2752 -> 0.2133 between
   depower 0.275 and 0.328), and the feasibility gate is computed with the phase-4
   `c1`. Phase 5 is therefore flown with a margin nobody checked. — DONE, as
   diagnostics rather than a refusal (see the tuning-log entry "Phase 5 is flown
   with 22 % less turn authority than any gate checked"): the margin is exactly
   linear in `c1`, so phase 5 flies at 0.775x every margin the run prints. In
   2026-08-18_213634 the last installed path scored 1.00 -> **0.78** in phase 5,
   only 5 % above `min_feasibility_margin = 0.74`; earlier installs at 0.83 would
   have been 0.64. Run and confirmed (2026-08-18_215919, all 8 criteria,
   8507 W at 0.98 x, RMS d 1.58 deg, min elevation 8.4 deg — unchanged): the
   re-optimizer hands phase 5 a **tighter path every lap**, so its phase-5 margin
   falls 1.58 -> 0.78 over the run's seven installs while the gate saw 0.98 ->
   1.02. Read `traj_opt.feasibility.margin_final_flown` FIRST when diagnosing
   items 1 and 2: a phase 5 that circles or dips on a path scored near the gate is
   not a control problem at all. The last install also landed inside phase 5, so
   the depower-aware gate scored it 0.78 where the old one said 1.00 — a slightly
   tighter reply would now be rejected where it used to be installed, which is the
   one behaviour change to watch across runs.
5. **NEW, and the parent of 1 and 2: the in-air shift gate scored the flown path
   at its own point count**, and a curvature check on an upsampled polyline
   measures the sampling — 0.25 … 0.70 as flown against 0.92 … 1.10 at the reply's
   own resolution. Every in-air shift of every run before 2026-08-18 22:47 was
   refused, so `el_bias` only ever reached the kite at an install and the phase-5
   learner never did. Fixed (`chk_points`); RMS d 1.58 -> 1.22 deg, 7 -> 8 laps,
   8507 -> 8524 W, all 8 criteria still passing. Anything measured about `el_bias`,
   `el_offset_lead` or the phase-5 elevation BEFORE that fix was measuring the
   install schedule, not the setting under test — re-run it before trusting it.

## Where to go next

Nothing on the list above is blocking any more. What the evening's measurements
suggest, in the order I would take it:

1. **Shape the lobe lift on ELEVATION, not azimuth.** The sag is deepest at the
   BOTTOM of the pattern, which is close to but not the same as the lobes; the
   azimuth bins in the tuning-log entry are the data to fit. A residual 0.53 deg
   of droop is left after the azimuth version.
2. **Learn the profile instead of fixing it.** `el_bias` already learns a scalar
   per lap from a whole-lap mean; the same update per azimuth bin, with the bins
   coupled by a smoothness constraint so the reference cannot grow a corner (see
   what a corner costs in the same entry), is the obvious generalisation.
3. **Re-measure the 8 s `el_offset_lead` runs.** Their +1.39 turns were recorded
   under the broken in-air gate and mean nothing now.
4. **`min_span_frac` interacts with every lift.** A path flown higher is flown
   narrower, and `el_offset_lead = 2.5` already failed the criterion by
   hundredths. Whatever raises the pattern next should report the reach margin
   alongside the elevation it buys.

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
