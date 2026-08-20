# Reel-out: where this stands and what is next (2026-08-18, end of session)

Working notes for picking the elevation-correction work back up on another machine.
The dated measurements behind all of it are in
[fig8_tuning_log.md](fig8_tuning_log.md); this file is only the state and the plan.

## What is in the working tree

**Update, 2026-08-19.** Everything below is committed (`6ce185c` and earlier); the
per-band elevation learner of item 2 and the `lift_budget` block of item 4 are the
only things that may still be uncommitted at the end of a session. Flown settings
as of that date: `el_bias_bins: 5`, `el_bias_smooth: 0.25`, `el_offset_lead: 8.0`,
`el_offset_wing: 1.5` azimuth 10/8. The rest of this section describes the state on
2026-08-18 and is kept for the file references.

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
   at 0** — superseded on 2026-08-19 for the 8 s lead only, which does deliver
   where 2.5 s cannot; see item 3 below. Nor was there circling: net course over phase 5 drifts -0.5 turns, but
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

Nothing on the list above is blocking any more, and as of 2026-08-19 all four items
below are answered too. What is left, in the order I would take it:

- **Bound the SHAPE of the learnt profile** (from item 2). The shoulder band
  integrates against a tracking error no reference can fix, and it cost two
  installs. Cap a band's departure from the profile's mean, give the bands a lower
  gain than the mean, or put `el_bias_smooth` back to 0.5. **Still open, and the
  wind scan of 2026-08-19 says it is not one condition's artefact**: band 4 is the
  highest band and its error is still open at the end at 5.0, 5.5, 6.0 and 7.0 m/s.
  Tune it at 6.0, not at 6.5 — 6.5 is the one run where every band converges,
  because the pattern collapsed to 65 % of its commanded height and left the kite
  room to track it.
- **`el_offset_wing` does not transfer across wind speed** (NEW, from the wind
  scan). At 6.5 m/s the flown configuration FAILS the elevation-span criterion
  (6.92 deg against 7.46 required, budget -0.55 deg), and a control run with
  `el_offset_wing = 0` and nothing else changed passes: the 1.5 deg lobe lift costs
  ~1.6 deg of flown pattern height, and buys 1.4 deg of minimum elevation, 0.14 deg
  of RMS d and the whole centre-to-lobe droop for 0.3 % of the power. So the answer
  to "the next lift has to report what it does to the per-lap span" is measured —
  raising the pattern narrows it, roughly degree for degree at the lobes — and the
  open question is whether 1.5 deg is still the right number, or whether the lift
  should scale with the span the commanded pattern leaves. 7.0 m/s passes at
  +2.21 deg, so this is not a monotone wind effect: it is which pattern the
  optimizer happens to return.
- **Independent repeats.** — DONE, and the premise was wrong: repeats are EXACT,
  and the trap below is withdrawn. A repeat after `bin/run_server restart` with the
  failure cache deleted reproduced 2026-08-19_092659 in every number, so the wind is
  the only lever that gets a different answer out of the optimizer. Five speeds are
  now flown (`docs/wind_scan_results.yaml`, tuning-log entry "The runs DO repeat,
  and the wind is what the settings were never tested against"): the flown window is
  **5.5 to 7.0 m/s**, with 5.0 failing on tracking (RMS d 2.05 deg, max 11.21, 10 %
  saturation) and on never reaching phase 5 within `sim_time = 150 s`, and 6.5
  failing on the span as above. What is still untested is a repeat that differs for
  a reason OTHER than the conditions.
- **Two things the scan turned up that nobody was looking for.** (a) The tether
  force crest factor over the reeling window rises 1.07 -> 1.26 -> 1.43 from 6.0 to
  7.0 m/s, so the ground station's worst peak-to-mean load is at the TOP of the
  window, where every flight criterion is comfortable. (b) `SystemError: write:
  Broken pipe` on `POST /step` hit every run of the scan (2-4 times each, none in
  the 6.0 repeat) and `output/awetrim_server.log` never sees them, so it is a stale
  pooled socket client-side that `retry_non_idempotent = true` does not cover; the
  run then treats it as a solver failure and `reopt_retry_el_offset` reinstalls from
  guess el 24 deg, i.e. **a run can fly a path from a seed it never asked for**.
  **FIXED 2026-08-20** in `examples/awetrim_client.jl`, after two runs lost 3 and 4
  of their 7 solves and produced numbers that read as a tuning win. HTTP.jl v2.6.5
  does auto-retry a request whose pooled connection died, but only for
  `GET/HEAD/OPTIONS/TRACE/QUERY` (`_retryable_method`), so every POST here was
  exposed; `post` now empties the pool before each request and retries up to 3 times
  on `stale_conn_error`. Check `traj_opt.reopt.requests` against `installed` before
  quoting a run — a run that lost requests is not a run at different settings.

The four items as answered:

1. **Shape the lobe lift on ELEVATION, not azimuth.** — ANSWERED, and the answer
   is no: the bottom of the pattern IS the lobe, so an elevation-keyed lift ramps
   through the tightest turn of the path and squashes it where the azimuth key
   translates it. Built anyway (`lobe_lift`, `el_offset_wing_mode: "elevation"`,
   `el_offset_wing_depth`, off by default) and swept OFFLINE on the real 150 m and
   380 m optimizer paths: 1.5 deg of lift takes the 380 m curvature margin from
   1.02 to 0.05, and even 0.2 deg costs as much margin as the whole azimuth
   profile does. See the tuning-log entry "The lobe lift cannot be keyed on
   ELEVATION".
   The same sweep suggested normalizing the azimuth key by each path's own
   amplitude (`el_offset_wing_mode: "azimuth_frac"`) would recover a lift that
   walks out of a shrinking pattern. FLOWN and it does not: 2026-08-19_072132 vs
   _072606 differ in nothing that matters (min elevation 10.1 deg both, power
   8625 vs 8629 W, `centre_to_lobe_deg` 0.70 vs 0.81). The premise was wrong —
   the sweep used a 380 m reply left on the server with A = 8.7 deg, while the
   paths these runs install never go below A = 12.1 deg, so `az_full = 10 deg`
   is inside the pattern throughout. `azimuth` 10/8 stays the flown setting; the
   fractional mode is kept for the day a pattern does shrink that far.
   `traj_opt.droop_profile` in the run summary is new, and is how the next
   attempt should be scored.
2. **Learn the profile instead of fixing it.** — DONE and FLOWN (2026-08-19_080350
   and _080918): all 8 criteria against the baseline's FAILED azimuth reach, RMS d
   1.43 -> 1.25 deg, min elevation 10.1 -> 10.3 deg, 8625 -> 8642 W,
   `centre_to_lobe_deg` 0.70 -> 0.22, `margin_final_flown` 0.83 -> 0.87. The two runs
   are identical in every flight number (same optimizer session and request
   sequence), so read that as determinism, not robustness. `el_bias_bins: 5` is the
   flown default now. **What is open**, and it is in the tuning-log entry: the
   SHOULDER band (60-80 % of the amplitude) never converges — its correction climbs
   0.58 -> 2.16 deg while its error stays near -0.8 deg, because the kite is bounded
   by turn authority there and not by where the reference is — and the bump that
   leaves in the reference cost two installs (replies refused at margin 0.68 and
   0.73 against 0.74 required) and held the phase-5 lift back at 0.67. Bounding the
   shape is the next move: cap a band's departure from the profile's mean, give the
   bands a lower gain than the mean, or put `el_bias_smooth` back to 0.5.
   The mechanics: `fcs.el_bias_bins`
   runs the per-lap update per azimuth band (keyed on |azimuth| over the pattern's
   own amplitude, so the bands follow a shrinking pattern) and `fcs.el_bias_smooth`
   couples the bands; `bias_lift` applies the bands as a smoothstep between their
   CENTRES rather than a staircase, so the reference cannot grow a corner whatever
   the bands do; `el_bias_bins: 1` gets the old scalar learner back exactly.
   Offline against a noiseless
   quadratic sag the converged profile leaves 0.21 deg RMS at 5 bands and
   `el_bias_smooth: 0.25`, against 0.67 deg for the rigid shift (tuning log,
   2026-08-19, "The elevation bias LEARNS a profile now"). What to read on a run:
   `traj_opt.el_bias.lap_profiles` band by band — a band whose error does not close
   while its correction climbs is chasing a tracking error no reference can
   fix — then `centre_to_lobe_deg` and the hold-backs in `shift_delivery`.
3. **Re-measure the 8 s `el_offset_lead` runs.** — DONE, and the +1.39 turns are
   withdrawn: they do not reproduce. Two runs each on today's code
   (2026-08-19_083042/_083528 against _080350/_080918), phase-5 net rotation -0.638
   turns at lead 8 against -0.642 at lead 0, mean u_s -2.1 % in both, no phase-5
   saturation in either. The lead is now ON (`el_offset_lead: 8.0`), because it is
   what gets the 1.5 deg `el_offset_final` past the gate at all: at lead 0 the shift
   is refused at margin 0.67 and every retry over the remaining 26 s is refused too,
   so phase 5 flies without it; at lead 8 it blends in at 0.83 six seconds earlier,
   while `c1_at(phase)` is still the pattern's and not `depower_final`'s 0.775x.
   Phase-5 minimum elevation 10.31 -> 11.75 deg for -0.13 % power. See the
   tuning-log entry "The 8 s `el_offset_lead` does not circle".
4. **`min_span_frac` interacts with every lift.** — DONE: `traj_opt.lift_budget`
   in the run summary (and a line at the end of the run) reports the correction the
   path in the air actually carries and the elevation it bought, against the room
   each size criterion has left, with `tightest` naming the binding one. Measured on
   the flown configuration: the budget is ONE criterion — azimuth reach on the
   POSITIVE side, +0.36 deg (+3 %) — where the negative side has +2.04 and the
   elevation span +9.75 (+111 %). The elevation learner bought ~0.4 deg of that
   margin rather than spending it (it is why the criterion flipped from FAILED to
   passed). Those numbers were scored against the STARTUP pattern, which is the
   widest and tallest the run ever flies, and rescoring against the pattern actually
   COMMANDED — `fig8_metrics` now takes the geometry per log sample, and the run
   records what it installs — changes the answer on both axes (tuning log, "Scored
   against the pattern it was COMMANDED"): the azimuth reach has +36/+39 % rather
   than +3, because the kite flies 95-98 % of the width it is given, and the binding
   criterion is the ELEVATION span with +1.5 deg (+23 %). The old +111 % on that
   axis was the pattern's descent over the run, not its height. `el_fill` is now
   measured per lobe-to-lobe excursion (`el_span_lap` beside `el_span`), and the lap
   band follows the commanded amplitude too — the real run counts the same 8 laps
   either way, but a pattern shrinking further would have lost crossings.

## Two traps that cost time in this session
   
- ~~**The runs do not repeat.**~~ **WITHDRAWN, 2026-08-19.** It was measured on
  2026-08-18 (RMS d 1.25 vs 1.61 deg, 8 vs 9 laps on byte-identical settings, the
  server answering margin 1.00 vs 0.99 at the same length), but it does not hold
  for the current code: a repeat with the server RESTARTED and the failure cache
  DELETED reproduced its baseline in every logged number. Quote a single run's
  numbers — but still read `traj_opt.reopt.events` when two runs disagree, because
  a lost request (see the broken-pipe item above) does make them diverge.
- **`output/reelout_150m_opt.{arrow,yaml}` is overwritten by every run**, and a run
  started from the REPL while you are reading them will swap them under you. Every
  run is archived under `output/archives/<timestamp>/` with its settings — compare
  archives, not `output/`.
- Adding a field to `FC_Settings` in a live session orphans the methods that
  dispatch on it: `Base.include` `fc_settings.jl`, then `course_controller.jl`,
  `traj_opt_settings.jl` and `optimization.jl`, or the run dies at
  `CourseControllerSettings(fcs; dt = s.dt)`.

## ToDo:
- ~~make a run at 5 m/s and one at 7 m/s wind speed (current: 6 m/s) and create a
  wind_scan_results.yaml file with the key results~~ — **DONE, 2026-08-19**:
  `docs/wind_scan_results.yaml` carries 5.0, 5.5, 6.0, 6.5 and 7.0 m/s plus the
  `el_offset_wing = 0` control at 6.5, each with laps, RMS and max cross-track
  error, and the mean / peak / crest-factor triple for power, tether force and
  reel-out speed — all three scored over the same reeling window
  (`SimpleKiteControllers.reelout_power`), plus energy, the elevation span, the
  learnt bias profile and the re-optimization tally. The verdicts are in
  "Where to go next" above: the flown window is 5.5 to 7.0 m/s.
- Regenerate it with
  `examples/wind_scan.jl` if more speeds are added; each run needs
  `environment.v_wind` AND `environment.wind_vec` of
  `data/settings_reelout_150m.yaml` changed together, since `use_wind_vec` is true.