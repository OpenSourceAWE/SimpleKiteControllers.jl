# Worked example: 4 m/s after the mass correction (2026-08-29)

The kite mass was corrected 6.2 -> 10.9926 kg. `output/scenarios/v04` is the passing run at
the old mass; the same settings at the corrected mass failed on the elevation floor alone.

## What was measured

| run | mass | `chi_dive` | `dive_el_margin` | min elevation, whole run | verdict |
| --- | --- | --- | --- | --- | --- |
| `v04` | 6.2 kg | -95 | 13 | **9.2°** | all 10 passed |
| baseline | 10.9926 kg | -95 | 13 | **-3.4°** | FAILED: min elevation > 6.5° (whole run) |
| screening | 10.9926 kg | -125 | 13 | **5.9°** | 0.6° short |
| screening | 10.9926 kg | -125 | 17 | **10.1°** | pass (screening length) |
| `v04_3` | 10.9926 kg | **-125** | **17** | **7.9°** | **all 10 passed** (full length) |

`min elevation > 6.5° (whole run)` was the ONLY criterion the baseline failed. Phase 4 was
unaffected throughout — laps and settled cross-track never regressed.

## The phase table that showed it was the dive

From the failing baseline (`chi_dive = -95`):

| phase | azimuth | elevation | duration |
| --- | --- | --- | --- |
| 0 park | -0.3 -> -1.1° | 73.4° | 2.0 s |
| 1 dive | **-1.1 -> -108.9°** | 73.9 -> 39.2° | 18.3 s |
| 2 hold | -108.9 -> -111.0° | 35.9° | 0.8 s |
| 3 transition | **-111 -> -12.1°** | 35.8 -> 14.7°, dipping to **-3.4°** | 19.7 s |

5° past flat, the dive barely descended and instead traversed 110° of azimuth, dumping the
kite at the edge of the window; phase 3 was a 19.7 s recovery back across it, and that is
where the altitude went. Survivable at 6.2 kg, not with 4.8 kg more wing.

After both levers (`-125` / `17`), the same table read: dive -1.1 -> -80.7°, 73.9 -> 43.1°,
13.5 s; hold 0.8 s; transition -82.4 -> -17.8°, 39.6 -> 15.4°, dipping to 10.1°, 13.6 s.

## Ruled out

- The **entry descent limiter** was active for 0.0 % of phase 3: it engages only above
  `entry_chi_max` (95°) and the guidance was commanding 62.7° — a CLIMB — at the lowest
  point. `entry_d_gate` was never reached either (`dmin` 48.6° against a 12° gate). The kite
  was told to climb and sank anyway.
- `entry_depower` (0.34) and `entry_f_min` (350.0) were left alone. Powering up buys lift,
  which means a SLOWER descent and a LONGER traverse — the wrong direction.

## Attribution of the fix

1. `chi_dive: -95 -> -125` recovered 9.3° of the 10.9° gap on its own: dive azimuth travel
   110° -> 81°, dive duration 18.3 -> 14.4 s, transition 19.7 -> 14.6 s, min elevation
   -3.4° -> +5.9°.
2. `dive_el_margin: 13 -> 17` closed the remaining 0.6°: min elevation 5.9° -> 10.1°
   (screening), 7.9° at full length — equal to `min_settled`, so the dip was gone entirely.

At the winning tuning the optimizer's `el_c_path` was 26.2°, so dive -> hold fired at
26.2 + 17 = 43.2° (confirmed against the log's `var_04`); `fcs.el_center: 18.0` was inert.

`av_power_ro` 1667 W at full length — above the old 6.2 kg baseline's 1551 W.
