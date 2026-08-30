# 3.5 m/s — fixed, pending regression

Both failures at 3.5 m/s are closed. What remains is the regression sweep, because
`data/fc_settings_reelout.yaml` is shared by every wind speed.

## What was wrong, and what fixed it

1. **`RMS d < 3.0°` (3.09°)** — not the pattern. Phase 4 alone read 2.43°; phase 5, the
   post-reel-out hold, read 4.26° from 29 % of the settled window. `rms_d` is now scored
   over the reel-out window (phases <= 4).
2. **`heading range < 400.0°` (624°)** — Q jumped to the reverse branch at the centre
   crossing at t = 359.56 s, because `reacquire_margin` (3.0°) is the same size as the
   phase-5 sag. Raised to 6.0 and exposed as an `FC_Settings` field.

Both are written up in `docs/fig8_tuning_log.md` (entries of 2026-08-30). The confirming
run is `output/archives/2026-08-30_125149`: **all 10 passed**, `rms_d` 2.51°,
`min_whole_run` 7.3°, `av_power_ro` 887 W, 8.0 laps.

## Steps

1. Archive the confirming run with `examples/copy_scenario.jl` (it is still only in
   `output/archives/`, not in `output/scenarios/v03.5`).
2. Re-fly 4, 5, 6, 7, 8, 9, 10 and 11 m/s. `reacquire_margin: 6.0` reaches all of them and
   has never been flown anywhere; `el_offset_final: 3.0` has never been flown at 4 m/s.
   Confirm `all 10 passed` and no power regression at each.
3. Regenerate `output/scenarios/overview.md` (`examples/create_overview.jl`) once the sweep
   is green.

## What to watch in the sweep

A larger `reacquire_margin` makes Q stickier — the opposite trade where a kite genuinely off
path needs to re-acquire. The tell is a run whose cross-track stays high for seconds after
an excursion instead of snapping back, or a `max d` that grows. If one appears, the fix is
not to back the margin down globally but to reduce the phase-5 sag that made 3.0° too small
in the first place (`oldplans/PlanPhase5Sink.md`).

`rel_turbulence: 0.0` makes the plant deterministic — an unchanged re-run reproduces a log
bit for bit, so every screening run must change exactly one lever to say anything.

## Never touch

`max_rms_d: 3.0`, `max_d` (kept on the whole settled window on purpose), `el_offset_final`,
`max_steering`, `data/traj_opt.yaml`.
