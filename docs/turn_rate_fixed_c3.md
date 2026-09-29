# Turn-rate identification with the gravity coefficient fixed (c3)

2026-09-29. The relay sweeps of `examples/build_turn_rate_table.jl`, re-fitted
with the turn-rate law of Eq. (9) of the paper,

    ψ̇ = c1·v_a·u_s(t − τ) + c3·sin(ψ)·cos(β)

with the gravity coefficient held at `c3` = 0.23 1/s. That value was identified
separately on the 25 flown reel-out figures of eight (`examples/identify_c3.jl`,
2026-09-28). Only `c1` and the delay are fitted. The free-`c2` fit in
`data/turn_rate_coeffs.yaml` is the reference.

**Status: identified, not used.** The rows are in
`data/turn_rate_coeffs_c3.yaml`. No project points at that table, so the
controllers and the stability analysis still read `turn_rate_coeffs.yaml`.

## Why

The relay sweep cannot identify the gravity term. The relay switches the
steering whenever the heading leaves the ±10° band, so the steps of the input,
which carry the information on the delay, always fall at the same headings. A
shorter delay can then be traded against a larger free `c2`. Fixing the gravity
term at the value from the figures of eight removes that trade.

## How to run

    include("examples/build_turn_rate_table.jl")
    build_turn_rate_table(c3 = SWEEP_C3)          # whole grid, ~15 min
    add_delay_lag_split!(c3 = SWEEP_C3)           # re-split an existing c3 table

With `c3` given, the output defaults to `turn_rate_coeffs_c3.yaml`. On first
use, that file is created from the `conditions` of `turn_rate_coeffs.yaml` plus
`c3`, and `_check_conditions` refuses to mix fixed-`c3` and free-`c2` tables.
Each row stores `c3` together with the table-form `c2 = c3·mean(v_app)`
(`c2_at` of `course_loop_model.jl`), because the table parser requires a `c2`.
The fit functions are `estimate_delay_fit_c3` and `fit_c1_c3` in
`examples/delay_lag_fit.jl`. `fit_delay_lag` takes a `c3` keyword, so the dead
time + lag split, the blockwise delay scatter and the per-half delays all use
the same fixed-`c3` fit.

Conditions are those of the table: `system_reelout_maasvlakte.yaml`, 9.51 m/s
of wind, 150 m tether, 73° elevation, body damping [0, 0, 40] settling to
[0, 0, 32].

## Results

All 7 cells finished the sweep (`sweep_done`, steering up to 0.175). Depower
0.40 was flown with its 40° elevation floor and sank to 48.2°. The free-`c2`
values are from `turn_rate_coeffs.yaml` (2026-09-27).

| depower | c1 c3 fixed | c1 free c2 | delay [s] c3 / free | dead time [s] c3 / free | lag [s] c3 / free | c2 c3·v_a / free |
|---|---|---|---|---|---|---|
| 0.25  | 0.2732 | 0.2691 | 0.316 / 0.354 | 0.068 / 0.106 | 0.267 / 0.283 | 3.09 / 0.67 |
| 0.275 | 0.2480 | 0.2449 | 0.343 / 0.379 | 0.110 / 0.143 | 0.250 / 0.267 | 3.06 / 1.44 |
| 0.30  | 0.2212 | 0.2189 | 0.364 / 0.397 | 0.147 / 0.175 | 0.233 / 0.250 | 3.03 / 1.85 |
| 0.325 | 0.1937 | 0.1919 | 0.386 / 0.413 | 0.168 / 0.205 | 0.233 / 0.233 | 3.00 / 2.20 |
| 0.35  | 0.1689 | 0.1676 | 0.406 / 0.429 | 0.205 / 0.222 | 0.217 / 0.233 | 3.00 / 2.43 |
| 0.375 | 0.1481 | 0.1469 | 0.420 / 0.446 | 0.265 / 0.281 | 0.167 / 0.183 | 2.98 / 2.43 |
| 0.40  | 0.1322 | 0.1311 | 0.433 / 0.459 | 0.310 / 0.324 | 0.133 / 0.150 | 2.95 / 2.45 |

Fit quality with `c3` fixed:

| depower | residual increase, pure delay | residual increase, dead time + lag | lag's rms gain | G rel. std | delay std [s] | v_app [m/s] |
|---|---|---|---|---|---|---|
| 0.25  | +5.2 % | +12.4 % | 19.7 % | 0.172 | 0.021 | 13.45 |
| 0.275 | +3.9 % |  +9.0 % | 17.4 % | 0.178 | 0.021 | 13.31 |
| 0.30  | +3.0 % |  +6.3 % | 13.7 % | 0.195 | 0.021 | 13.16 |
| 0.325 | +1.8 % |  +4.0 % | 10.6 % | 0.194 | 0.014 | 13.04 |
| 0.35  | +1.2 % |  +2.6 % |  9.3 % | 0.216 | 0.016 | 13.04 |
| 0.375 | +2.0 % |  +3.4 % |  5.9 % | 0.232 | 0.016 | 12.97 |
| 0.40  | +1.3 % |  +1.7 % |  3.5 % | 0.285 | 0.024 | 12.84 |

"Residual increase" is the RMS residual with `c3` fixed relative to the
free-`c2` fit. "Lag's rms gain" is `1 − rms_lag/rms_delay`, which says how much
the lag improves the fit over a pure delay.

## Findings

- **c1 is robust to the gravity term.** It is 0.8 – 1.5 % higher with `c3`
  fixed, with a relative standard error of about 0.1 %, and still decays by a
  factor of 2.1 over the grid. The gain schedule would barely change.
- **The delay is 0.02 – 0.04 s shorter at every depower.** The fixed gravity term
  `c3·v_a` ≈ 3.0 is larger than the free `c2` (0.7 – 2.5), and the fit buys it
  with a shorter delay, the trade described above. Most of the shortening is in
  the dead time (−0.014 to −0.038 s); the lag drops by 0.017 s or less.
  The shift is largest at low depower, where the free `c2` was furthest from
  `c3·v_a`.
- **The fit is slightly worse**, as expected with one parameter fewer: +1 – 5 %
  residual for the pure delay and +2 – 12 % for dead time + lag, again largest at
  low depower. The residual alone would favour the free `c2`. But the free `c2`
  is not identifiable from this experiment (0.67 at depower 0.25 against 3.1
  from the flown figures), so a lower residual does not show that it is more
  correct.
- **Every row passes the table's quality bar**: `c1_rel_std` ≤ 0.01 and
  `g_rel_std` ≤ 0.35.
- **The per-half delay exponent (`delay_exp`) is unreliable**, as the docstring
  of `_delay_over_v_app` warns: 1.3 – 1.7 for most cells, but 0.08 at 0.375 and
  −3.8 at 0.40.

## Effect on the stability margins

Checked 2026-09-29 on the 22 archived scenarios of both sites with
`examples/retune_guided.jl`. `collect_scenarios()` collects the operating
points once, and `scenario_margins` rates them with the live
`fc_settings_reelout.yaml`, once with each table loaded in memory.
`C1_SETPOINT` was re-read from the loaded table. That raises the loop gain by
+1.2 % (c1 at depower 0.274: 0.2489 against 0.2459), as if the controller were
scheduled from this table with the same `heading_p`.

| | free-c2 table | c3 table | change per scenario |
|---|---|---|---|
| α inner, range | 0.635 – 1.375 | 0.697 – 1.396 | 0 to +0.070 |
| α guided, worst (Cabauw 10 m/s) | 0.288 | 0.291 | −0.009 to +0.011 |
| delay margin guided, change | | | −0.009 to +0.005 s |

With `C1_SETPOINT` kept at the free-`c2` value, so the delay change acts
alone, the changes are almost the same: α inner +0.007 to +0.076, α guided
−0.012 to +0.008 (worst 0.289), delay margin −0.008 to +0.006 s. The shorter
delay is the cause, and the 1 % gain change hardly matters.

- **The inner loop gains most**, up to +0.07, largest in the low-wind
  scenarios, where α inner is lowest (Maasvlakte 3.5 m/s: 0.635 → 0.697,
  Cabauw 3 m/s: 0.721 → 0.780). It uses the table's dead time and lag
  directly, and their sum is 0.02 – 0.04 s shorter.
- **The guided loop, the rated one, does not change.** Its kite response time
  comes from the pattern law (`pattern_dead_time_lag`), which rescales the
  table's dead time + lag to the pattern's measured sum. Only the split moves,
  a little towards lag. The worst scenario and bin stay the same (Cabauw
  10 m/s, 175 m). A few scenarios move their worst bin by one bin.
- The guided margins stand with either table. Adopting this table would only make the inner loop, which is shown for
  comparison, look more robust.

## Open

- Switching a project to this table means changing its `turn_rate_coeffs` entry;
  `reload_turn_rate_table!` then picks it up.
