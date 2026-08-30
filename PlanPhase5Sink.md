# The post-reel-out sink sets the elevation floor

Opened 2026-08-30 out of the 3.5 m/s entry tuning (see `docs/fig8_tuning_log.md` and
`.claude/skills/tune-entry/`); investigated the same day against `v03.5`, `v04`, `v05`,
`v06`. At 4 m/s and above the run's lowest elevation is not in the entry at all but in
**phase 5**, the hold after reel-out has finished, and `v05` failed there.

**Status: closed for 4, 5 and 6 m/s** by `el_offset_final: 1.5 -> 3.0` (committed) —
5 m/s 6.4 -> 7.8°, 6 m/s 7.2 -> 8.3°, 4 m/s 7.5 -> 9.4°, all `all 10 passed`, power
unmoved. 3.5 m/s is not yet re-flown; the two deeper levers below (`el_bias_max`,
`depower_final`) stay open, since the sag itself is unchanged — it is now compensated,
not removed.

## What is measured

Whole-run minimum elevation and the phase it falls in (`v04` still at `chi_dive: -125`, the
rest at -135; no turbulence, full length):

| wind | `min_whole_run` | phase | last reel-out lap | phase-5 laps | verdict |
| --- | --- | --- | --- | --- | --- |
| 3.5 m/s | 7.3° | 3 (entry) | 10.1° | 7.4° | fails RMS d only |
| 4 m/s | 7.5° | **5** | 10.0° | 8.7, 7.5, 7.5° | all 10 passed |
| 5 m/s | **6.4°** | **5** | 8.6° | 6.6, 6.4° | FAILED: min elevation > 6.5° |
| 6 m/s | 7.2° | **5** | 8.5° | 7.2° | all 10 passed |

`el_min_run_deg == el_min_final_deg` in every one of them: the whole-run minimum IS the
phase-5 minimum. The summary's own annotation on that field — "set in phase 4, so a phase-5
lift does not move it" (`examples/reelout_results.jl:695`) — is a fixed string and is now
wrong; `el_offset_final` is precisely the lever that moves the failing criterion.

## The mechanism

At the phase 4 -> 5 step the winch stops and `depower_final` takes depower 0.27 -> 0.35. Both
lift and turn authority drop with it (`c1_final: 0.1655`), and the kite falls away from the
path it was tracking. From the per-band learner in the run summaries, the mean elevation
error against the optimizer's curve, crossing band first:

| run | last reel-out lap | first phase-5 lap | learnt correction, phase 5 |
| --- | --- | --- | --- |
| `v03.5` | -0.03 -0.22 -0.48 -0.92 +0.18 | -1.46 -1.54 -2.25 -3.44 -1.69 | [2.84 3.17 **4.00 4.00 4.00**] |
| `v04` | -0.03 -0.16 -0.36 -0.75 +0.30 | -2.21 -2.19 -2.25 -2.91 -1.95 | [2.71 3.04 3.85 **4.00** 3.77] |
| `v05` | +0.03 +0.01 -0.35 -0.72 +0.30 | -2.68 -2.92 -2.69 -2.89 -1.54 | [3.08 3.50 **4.00 4.00** 3.27] |
| `v06` | +0.17 +0.26 +0.29 +0.58 +0.97 | -1.80 -1.96 -2.06 -2.65 -0.65 | [2.11 2.47 3.13 3.42 1.94] |

Tracking error under a degree through reel-out; 1.5-3.4° of sag in every band the moment the
winch stops. Cross-track at the low point roughly doubles with it (`v05`: 3.9° -> 6.5°).
`el_bias_max: 4.0` is **binding** in three of the four runs — two bands of `v05` sit at 4.00
while still measuring -2.7°, so the learner is asking for more than it is allowed and could
not close the gap even with laps to spare. `v06`, the one run that does not clamp, is also
the one with the shallowest sag.

`el_offset_final: 1.5` is the fixed lift meant to cover this. It latched correctly and early
(`lift_t_s: 145.7`, 9.3 s before phase 5, via `el_offset_lead`) — it is simply less than half
of the 2-3.4° that is actually lost.

## The criterion is an angle, and that matters here

`v05`'s numbers, from the log:

- phase 4 low point: **8.6°**, tether ~345 m, kite **36.7 m** above ground — passes.
- phase 5 low point: **6.4°**, tether 380 m, kite **42.2 m** above ground — fails.
- whole-run lowest point in metres: **31.8 m** at t = 19.6 s, in the **entry** (phase 3), at
  13.8° elevation and 134 m of tether — under `min_height: 40.0` and invisible to the
  elevation criterion.

So the run fails at its physically highest low point and passes at its lowest one. This is
not an argument for relaxing `min_elevation` — it stays closed — but it is the reason the
failure is so sensitive to where reel-out happens to end, and it says the entry is where the
real clearance is spent.

## Levers, ranked by the evidence

| lever | now | why | risk |
| --- | --- | --- | --- |
| `el_offset_final` | 1.5 | Direct compensation for the measured 2-3.4° sag, and the `lift_budget` says the next lift has **+1.98° (+29 %)** at 5 m/s before the elevation-span size criterion binds (+1.93° at 4, +2.40° at 6, +2.12° at 3.5). The mean of a shift goes in whole even where its spread is withheld. | A raised path is a tighter path; `margin_final_flown` is already 0.69 against `min_required: 0.82`. |
| `el_bias_max` | 4.0 | Clamped in three of four runs exactly where the sag is deepest. | Only helps where a lap remains to learn in and an install can carry it; phase 5 usually has neither (`max_reopt: 7`, `v05` spent 5). |
| `depower_final` | 0.35 | Attacks the cause instead of compensating it: powering up restores lift AND `c1`, which is what the doubled cross-track says is missing. Tuned for ~3458 N at 6 m/s; `v05` pulls 1399 N in phase 5, far from any limit. | Global, so it moves 9-11 m/s where force is not slack. Check those before keeping it. |

`el_offset_final: 1.5 -> 3.0` was flown first and closed the failure: the lift is delivered
about 1:1 into the phase-5 minimum (+1.4° at 5 m/s, +1.1° at 6, +1.9° at 4), costs no power,
and each run still reports ~+2° of lift budget left. `margin_final_flown` did not move (0.69
at 5 m/s) and RMS d was flat, so the tighter path did not cost tracking.

## Method

**Screening runs do not apply.** A truncated run never reaches `reelout_l_max`, so it has no
phase 5 — every run here is full length (~5 min at 5 m/s). Otherwise the discipline of
`.claude/skills/tune-entry/` holds: one lever per run, numbers from the archive named in
`output/last_run_done.txt`, `copy_scenario.jl` for a confirmation, an entry per result in
`docs/fig8_tuning_log.md`. Read the phase the minimum falls in, not just its value:

```julia
sl = load_log("reelout_150m_opt"; path = "output/scenarios/v05").syslog
el = rad2deg.(sl.elevation)
(round(minimum(el); digits = 1), sl.sys_state[argmin(el)])
```

## Done when

- [x] 5 m/s over the gate with margin: **7.8°**, `all 10 passed`, 3836 W against 3840 W.
- [x] 6 m/s **8.3°** and 4 m/s **9.4°**, both `all 10 passed`, power unmoved.
- [x] The stale annotation on `el_min_run_deg` in `examples/reelout_results.jl` corrected.
- [ ] 3.5 m/s re-flown at `el_offset_final: 3.0` — its minimum is in the entry, so no change
  is expected, but it is unconfirmed.
- [ ] Regenerate `output/scenarios/overview.md` (`examples/create_overview.jl`) once it is.
- [ ] Optional, if more margin is ever wanted: `el_bias_max` (clamped at 4.0 in three of four
  runs) and `depower_final` (the step that causes the sag).
