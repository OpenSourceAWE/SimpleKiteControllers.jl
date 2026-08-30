# The post-reel-out sink sets the elevation floor

Opened 2026-08-30 out of the 3.5 m/s entry tuning (see `docs/fig8_tuning_log.md`, that
day's entry, and `.claude/skills/tune-entry/`). The entry fault it started from is fixed;
what it uncovered is that at 4 m/s and above the run's lowest point is not in the entry at
all but in **phase 5**, the hold after reel-out has finished.

## What is measured

Whole-run minimum elevation and the phase it falls in, all at `chi_dive: -135`,
`dive_el_margin: 17`, no turbulence, full length:

| wind | `min_whole_run` | phase | phase-4 min | verdict |
| --- | --- | --- | --- | --- |
| 3.5 m/s | 7.3° | 3 (entry) | 9.5° | `v03.5_2`, elevation passes; fails RMS d only |
| 4 m/s | 7.9° | **5** | 10.0° | all 10 passed |
| 5 m/s | **6.4°** | **5** | 8.6° | `v05_2`, FAILED: min elevation > 6.5° |
| 6 m/s | **6.5°** | **5** | 8.3° | `v06` (at -125), passes by exactly 0.0° |

The gate is 6.5° (`min_elevation`). Three of the four runs are decided in a window that
carries no reel-out power at all, and 6 m/s passes it to the tenth of a degree.

## Why 5 m/s went red

`chi_dive: -125 -> -135` did not touch the entry at 5 m/s (phase 3 min 13.9 -> 13.8°). It
ended the dive 5° earlier in azimuth and 0.7 s sooner; reel-out then finished sooner and
phase 5 ran 50.2 -> 53.1 s. `v05` recovered from its minimum and ended phase 5 climbing, at
16.4°; `v05_2` ends it still sinking, at 7.2°. The 2.5° of lost margin is a downstream
timing effect, not a control fault in the entry — which is why it cannot be tuned back out
from the entry parameters.

## Where to look

`examples/simple_opt_reelout.jl`'s docstring already describes the mechanism and its levers;
the sag in phase 5 is known and partly compensated. Current values in
`data/fc_settings_reelout.yaml`:

| parameter | value | role |
| --- | --- | --- |
| `el_offset_final` | 1.5 | Fixed lift of the path once reel-out ends — a setpoint move, not an error; there is no reel-out power left to trade for height. |
| `el_offset_lead` | 8.0 | How early that lift latches, in seconds of remaining reel-out. Matters when `reelout_softstop` gives no latch of its own. |
| `el_bias_gain_final` | 1.0 | Per-lap learning gain from state 5 on, where the sag deepens. A correction only reaches the kite through an install, and phase 5 usually has none left. |
| `el_bias_bins` / `el_bias_smooth` | 5 | The correction as a profile; its spread is rationed against the curvature margin, its mean is not. |
| `reelout_softstop` | 1.5 | The deceleration trigger that creates the stop latch the lift hangs on. |
| `depower_final` | 0.35 | Depower flown from phase 5. |

The tuning log's "The whole-run minimum is the LAST in-air shift missing the gate by 0.04"
is the same failure mode seen from the other side: two runs at identical settings split by a
degree of ground clearance on whether the final shift cleared `min_feasibility_margin`. Check
first whether `v05_2`'s last shift went in at all before reaching for a new lever.

## Method

**Screening runs do not apply here.** A truncated run never reaches `reelout_l_max`, so it
has no phase 5 — every run in this task has to be full length (~5 min at 5 m/s, ~5 min at
3.5 m/s). Everything else follows the same discipline as `.claude/skills/tune-entry/`: one
lever per run, read the numbers from the archive named in `output/last_run_done.txt`, archive
a confirmation with `copy_scenario.jl`, record each result in `docs/fig8_tuning_log.md`.

Read the phase where the minimum falls, not just its value — that is what distinguishes this
fault from the entry one:

```julia
sl = load_log("reelout_150m_opt"; path = "output/scenarios/v05_2").syslog
el = rad2deg.(sl.elevation)
(round(minimum(el); digits = 1), sl.sys_state[argmin(el)])
```

## Done when

- 5 m/s back over the 6.5° gate with real margin (target: the whole-run minimum no lower
  than the phase-4 minimum, as at 3.5 m/s), `all 10 passed`, no power regression against
  `v05`'s 3827 W.
- 6 m/s off the knife edge — it currently passes at exactly 6.5°.
- 4 m/s and 3.5 m/s unregressed, the settings file being shared by every wind speed.
- Then regenerate `output/scenarios/overview.md` (`examples/create_overview.jl`), which is
  deliberately NOT regenerated while 5 m/s is red.
