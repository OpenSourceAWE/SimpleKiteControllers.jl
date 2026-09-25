# Symmetric figure-eight paths (`pattern_symmetric`)

2026-09-25. Results of forcing the optimized reel-out figure-eight to be
mirror-symmetric about azimuth 0, flown with `simple_opt_reelout.jl` at Cabauw and
Maasvlakte.

**Status: on (`pattern_symmetric: true` in `data/traj_opt.yaml`). Tested at 4, 7
and 10 m/s at both sites; all 12 runs passed all 10 success criteria.**

- **Cabauw:** at 4 m/s the free optimum is already nearly symmetric, and forcing it
  costs nothing. At 7 and 10 m/s it costs 2 – 3 % energy. In exchange the optimizer
  is 2 – 4× faster, no re-optimization failed, every installed path had a curvature
  margin of at least 1.0, and tracking improved.
- **Maasvlakte:** the free optimum is much closer to symmetric at every wind speed.
  Symmetry costs 0.5 – 1.3 % energy and gains 1.2 – 2.4 m of clearance. Tracking is
  unchanged at 4 and 7 m/s and slightly worse at 10 m/s. The optimizer only gets
  faster at 10 m/s.

## Why

Nothing physical prefers one side: the kite, the wind (no veer) and gravity are all
symmetric. But the power objective hardly depends on the lobe balance, and the
solve has several local optima, so AWETrim lands on lopsided figures. The 7 m/s
startup path of 2026-09-25 spanned −23.7 … +19.1° azimuth. Its centre was at −2.3°,
228 of 360 points were on the left side, and the lobes differed by up to 11°
(mirror mismatch). The re-optimized paths swung from side to side:
centres −2.3°, −1.4°, +0.1° at 7 m/s, and −1.3°, +2.4°, −0.3°, −3.0° at 10 m/s.
At 4 m/s the free optimum is close to symmetric: centres +0.2 … +0.8° and a
mirror mismatch of 1.2 – 1.9°.

## How

The pattern is a uniform periodic cubic B-spline with M control points (M = 10,
the server default). A symmetric figure-eight is at the mirrored point half a
period later:

    az(u + 1/2) = −az(u),   el(u + 1/2) = el(u)

For even M, shifting u by one half is exactly a shift of the control-point index by
M/2. The condition is therefore M linear equality rows:

    C_phi[k + M/2] = −C_phi[k],   C_beta[k + M/2] = C_beta[k],   k = 0 … M/2 − 1

- **AWETrim**
  - `PatternLimits.symmetric` in `server/schemas.py`; `session.py` maps it to
    `sim_parameters["symmetric_pattern"]`.
  - `phase_parametrized.py` adds the rows.
  - The warm start is projected onto the symmetric shape
    (`symmetrize_periodic_coefficients`), so IPOPT starts feasible, even from a
    lopsided previous optimum on `/step`.
  - Odd M is rejected with an error.
- **SimpleKiteControllers**
  - `TrajOptSettings.pattern_symmetric` (default `false`), sent through
    `pattern_limits_from` in `examples/awetrim_client.jl`.
  - `with_elevation_max`, `with_azimuth_amplitude_min` and `with_size_box` in
    `simple_opt_reelout.jl` carry the flag through when they rebuild the box.
  - While the flag is off, the solution-cache keys are unchanged.

Bench check on the server (200 m, 5.2 m/s, the `optimize_path.jl` setup):

| Case | Centre | Mirror mismatch | Power |
|---|---|---|---|
| Free | +0.29° | 1.33° | 7477 W |
| Symmetric | 0.00° | 0.07° (resampling) | 7439 W |
| Symmetric, guess shifted 4° | 0.00° | 0.07° | 7463 W |

## Results: Cabauw

Each pair differs only in `pattern_symmetric`; every other archived YAML is
identical.

| Wind | Not symmetric | Symmetric |
|---|---|---|
| 4 m/s | `output/scenarios/cabauw/v04` | `output/archives/2026-09-25_222205` |
| 7 m/s | `output/scenarios/cabauw/v07` | `output/archives/2026-09-25_215215` |
| 10 m/s | `output/scenarios/cabauw/v10_2` | `output/archives/2026-09-25_215950` |

### Energy and power

| | 4 m/s not sym. | 4 m/s sym. | 7 m/s not sym. | 7 m/s sym. | 10 m/s not sym. | 10 m/s sym. |
|---|---|---|---|---|---|---|
| Energy [kJ] | 696.0 | 696.1 (+0.0 %) | 1499.7 | 1467.7 (−2.1 %) | 1646.8 | 1596.3 (−3.1 %) |
| Mean reel-out power [W] | 6526 | 6527 (+0.0 %) | 21761 | 21123 (−2.9 %) | 24438 | 23623 (−3.3 %) |
| Predicted power [W] | 6602 | 6589 (−0.2 %) | 24552 | 24067 (−2.0 %) | 23308 | 22737 (−2.4 %) |
| Measured / predicted | 0.98 | 0.98 | 0.88 | 0.87 | 1.04 | 1.03 |
| Mean / max force [N] | 2995 / 3514 | 2996 / 3507 | 6531 / 7273 | 6391 / 6907 | 7170 / 8120 | 6953 / 7892 |

Most of the energy loss is already in the optimizer's prediction. The symmetric
optimum really is a slightly lower-power figure. The loss does not come from the
controller: the measured/predicted ratio barely changes.

### Tracking and control

| | 4 m/s not sym. | 4 m/s sym. | 7 m/s not sym. | 7 m/s sym. | 10 m/s not sym. | 10 m/s sym. |
|---|---|---|---|---|---|---|
| Cross-track RMS [deg] | 0.71 | **0.66** | 1.18 | **0.76** | 1.28 | **1.14** |
| Worst lobe vs centre elevation gap [deg] | 0.80 | 0.67 | 5.35 | 0.0 | 1.52 | 2.14 |
| Peak steering [-] | 0.320 | 0.320 | 0.293 | 0.265 | 0.270 | 0.284 |
| Time at steering-rate limit [%] | 4 | 3 | 5 | 0 | – | 0 |
| High-frequency turn-rate noise [deg/s] | 0.34 | 0.32 | 0.53 | 0.34 | 0.40 | 0.34 |

### Clearance and margins

| | 4 m/s not sym. | 4 m/s sym. | 7 m/s not sym. | 7 m/s sym. | 10 m/s not sym. | 10 m/s sym. |
|---|---|---|---|---|---|---|
| Lowest point flown [m] | 40.1 | 39.9 | 44.4 | 42.3 | 36.8 | **41.1** |
| Min phase-5 elevation [deg] | 17.8 | 18.5 | 16.5 | 15.1 | 16.2 | **18.5** |
| Curvature margins at install | 1.02 – 1.06 (5 paths) | 1.04 – 1.14 (5 paths) | 1.02, 1.09 | 1.20, 1.28, 1.99 | 0.87, 1.02, 1.02 | 1.02, 1.12, 1.07 |
| Phase-5 margin of the last path | 0.69 | 0.70 | 1.00 | 1.94 | 1.32 | 1.38 |

### Optimizer

| | 4 m/s not sym. | 4 m/s sym. | 7 m/s not sym. | 7 m/s sym. | 10 m/s not sym. | 10 m/s sym. |
|---|---|---|---|---|---|---|
| Paths installed / failed | 5 / 0 | 5 / 0 | 2 / 1 | 3 / 0 | 3 / 0 | 3 / 0 |
| Longest re-optimization [s] | 11.8 | 14.3 | 87.7 (failed) | 33.3 | 60.4 | **12.6** |
| Time frozen waiting for the optimizer [s] | 34 | 30 | 111 | 49 | 127 | **32** |
| Total wall time [s] | 110 | 166¹ | 216 | 145 | 241 | 136 |
| Startup seed retry [deg] | 0 | 0 | 0 | +1 | 0 | 0 |

¹ The 4 m/s symmetric run was the first in a freshly started REPL, so its wall
time includes package loading and compilation. It is not comparable.

## Results: Maasvlakte

Flown 2026-09-25, 22:36 – 22:50. Again each pair differs only in
`pattern_symmetric`.

| Wind | Not symmetric | Symmetric |
|---|---|---|
| 4 m/s | `output/scenarios/maasvlakte/v04` | `output/archives/2026-09-25_223648` |
| 7 m/s | `output/scenarios/maasvlakte/v07` | `output/archives/2026-09-25_224252` |
| 10 m/s | `output/scenarios/maasvlakte/v10` | `output/archives/2026-09-25_224638` |

The free optimum is much closer to symmetric than at Cabauw: centre offsets of
at most 2.1° and a mirror mismatch of 0.6 – 4.5°. The largest mismatch is always
on the startup path; the re-optimized paths are within 2.6°.

| | 4 m/s not sym. | 4 m/s sym. | 7 m/s not sym. | 7 m/s sym. | 10 m/s not sym. | 10 m/s sym. |
|---|---|---|---|---|---|---|
| Mirror mismatch of the paths [deg] | 1.2 – 4.5 | 0 | 0.6 – 2.6 | 0 | 1.2 – 3.8 | 0 |
| Energy [kJ] | 308.8 | 307.2 (−0.5 %) | 1044.8 | 1031.2 (−1.3 %) | 1429.2 | 1413.9 (−1.1 %) |
| Mean reel-out power [W] | 1548 | 1533 | 12157 | 11916 | 20128 | 19771 |
| Predicted power [W] | 1465 | 1442 | 12079 | 11915 | 24093 | 23693 |
| Measured / predicted | 1.04 | 1.04 | 1.00 | 1.00 | 0.83 | 0.83 |
| Mean / max force [N] | 1191 / 1701 | 1186 / 1709 | 4516 / 5325 | 4457 / 5286 | 6210 / 6843 | 6144 / 6864 |
| Cross-track RMS [deg] | 0.67 | 0.67 | 0.70 | 0.71 | **0.95** | 1.07 |
| Worst lobe vs centre elevation gap [deg] | 0.0 | 0.0 | 0.99 | 0.75 | 6.16 | 5.84 |
| Time at steering-rate limit [%] | 4 | 4 | 6 | 6 | 1 | 0 |
| Lowest point flown [m] | 27.0 | **28.2** | 38.1 | **39.8** | 39.4 | **41.8** |
| Min phase-5 elevation [deg] | 8.6 | 9.3 | 8.5 | **10.7** | 12.8 | 12.4 |
| Curvature margins at install | 1.00 – 1.07 | 1.07 – 1.13 | 1.02 – 1.06 | 1.07 – 1.11 | 1.12, 1.50, 1.87 | 1.16, 1.41, 1.79 |
| Phase-5 margin of the last path | 0.83 | 0.85 | 0.75 | 0.78 | 1.59 | 1.51 |
| Paths installed / failed | 5 / 0 | 5 / 0 | 5 / 0 | 5 / 0 | 3 / 0 | 3 / 0 |
| Longest re-optimization [s] | 12.8 | 12.9 | 11.7 | 11.5 | 58.3 | 54.0 |
| Time frozen waiting for the optimizer [s] | 30 | 32 | 32 | 30 | 124 | **33** |

## Conclusions

### Cabauw

- At 4 m/s the free optimum is already nearly symmetric (mirror mismatch
  1.2 – 1.9°). Forcing symmetry changes nothing measurable: the energy is the
  same to 0.1 kJ, tracking is slightly better (0.71 → 0.66°) and the curvature
  margins at install are slightly higher (1.02 – 1.06 → 1.04 – 1.14). The lopsided
  optima appear at the higher wind speeds, where the figure is larger.

- At 7 and 10 m/s symmetry costs 2 – 3 % energy: 2.1 % at 7 m/s and 3.1 % at
  10 m/s.
- The optimizer benefits most: 2 – 4× less time frozen waiting for it, no failed
  re-optimization, and no installed path with a curvature margin below 1.0.
- Tracking improves at both wind speeds, by 36 % at 7 m/s and 11 % at 10 m/s.
- The clearance trend is mixed. It improved at 10 m/s (+4.3 m, +2.3°) but was
  slightly lower at 7 m/s (−2.1 m, −1.4°). At 7 m/s the symmetric startup path is
  smaller (14.3° tall instead of 17.5°).
- At 10 m/s, 2.1° of lobe-vs-centre elevation gap remains, which symmetry does not
  remove.

### Maasvlakte

- The lopsided optima are a Cabauw problem. At Maasvlakte the free optimizer
  already lands close to symmetric, so forcing symmetry changes less in both
  directions.
- Energy: −0.5 % at 4 m/s, −1.3 % at 7 m/s, −1.1 % at 10 m/s. Again almost all of
  it is in the prediction; measured/predicted is identical in every pair.
- Clearance improves at every wind speed: +1.2, +1.7 and +2.4 m at the lowest point.
- Curvature margins at install are 0.05 – 0.07 higher at 4 and 7 m/s.
- The optimizer is only faster at 10 m/s (124 → 33 s frozen); at 4 and 7 m/s the
  solves were already fast.
- Tracking is unchanged at 4 and 7 m/s and 0.12° worse at 10 m/s.

### Both sites

- Symmetry never cost more than 3.1 % energy (Cabauw 10 m/s) and never failed a
  run. It always made the optimizer at least as fast and the margins at install at
  least as high. Keeping it on is the safe default; the price is highest at
  Cabauw's higher wind speeds.

## Open

- Other wind speeds (3, 5 – 6, 8 – 9, 11 m/s) are untested.
- Phase-5 curvature margins below 1.0, in both runs of each pair, so not a symmetry
  effect: Cabauw 4 m/s (0.69 – 0.70), Maasvlakte 4 m/s (0.83 – 0.85) and 7 m/s
  (0.75 – 0.78).
- Maasvlakte 4 m/s flies as low as 27 – 28 m and 8.6 – 9.3° elevation in phase 5,
  in both runs, while passing all criteria. Worth a look independent of symmetry.
- Maasvlakte 10 m/s: measured/predicted is only 0.83, and the lobes fly 6° below
  the centre crossing, in both runs.
- A middle option: symmetric startup solve, free re-optimizations. That would keep
  the robust start and recover some of the energy on later paths. It needs a
  separate setting.
