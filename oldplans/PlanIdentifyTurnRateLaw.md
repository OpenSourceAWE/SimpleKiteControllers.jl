# Improve the identification of the turn rate law coefficients

## Motivation

The current turn-rate law, fitted by `identify_turn_rate_law` and `fit_delay_lag`,

$$
\dot{\psi} = c_1 v_\mathrm{a} u_\mathrm{s} + \frac{c_2}{v_\mathrm{a}} \sin\psi \cos\beta,
$$

over-predicts the turn rate at low apparent wind speed: 1.2 – 2.5 × too large
below $v_\mathrm{a} = 20\,$m/s (V3 of `oldplans/Plan_model_validation.md`). Above
about 15 – 20 m/s it is good: turn-rate VAF 0.94 – 0.98, and `c1` confirmed within
±5 % in pattern flight (V4).

A more accurate law adds a term with the kite's mass $m$ to the denominator,
$k_4 m v_\tau$ ($v_\tau$ the kite speed perpendicular to the tether). Below, this
is called the **mass term**, and the law with it the **extended law**:

$$
\dot{\chi}_\mathrm{turn} \approx
-\frac{k_1 v_\mathrm{a}^2 u_\mathrm{s} - k_3 m g \sin\chi \cos\beta}
      {k_4 m v_\tau + k_2 v_\mathrm{a}}
$$

For $k_4 m v_\tau \ll k_2 v_\mathrm{a}$ it reduces to the current law, with
$c_1 = k_1/k_2$ and $c_2 = k_3 m g / k_2$ (up to the sign convention of
$u_\mathrm{s}$); in V3Kite's sign convention and with $k_2 = 1$ the extended law is
$\dot\psi = (c_1 v_\mathrm{a}^2 u_\mathrm{s} + c_2 \sin\psi \cos\beta)/(v_\mathrm{a} + e\, v_\tau)$,
$e = k_4 m / k_2$, and $e = 0$ is the current law. So the assumption being dropped is that the mass term is
negligible against the aerodynamic damping. That term weighs more at low
$v_\mathrm{a}$, which is where the current law fails, and it lowers the predicted
turn rate there, in the direction V3 measured.

## First step: identify at a lower elevation

Fit the CURRENT law on sweep data flown at a lower elevation, where the kite flies
faster crosswind and $v_\tau/v_\mathrm{a}$ is larger. Do not apply the new formula
yet. The result shows whether $c_1$, $c_2$, the dead time and the lag change with
the elevation. If they do, the mass term is a candidate cause.

### What limits the elevation today

Lowering the floor alone gives no low-elevation data. The sweep starts at
`ELEVATION` = 73° at a constant tether length, and its relay oscillates the heading
in a ±10° band about 0°, i.e. about flying straight up. So the kite hovers near its
start. The lowest elevation each sweep reached (`min_elevation` in
`turn_rate_coeffs.yaml`):

| depower | 0.25 | 0.275 | 0.30 | 0.325 | 0.35 | 0.375 | 0.40 |
|---|---|---|---|---|---|---|---|
| min elevation [°] | 73.0 | 72.1 | 70.6 | 67.1 | 61.4 | 54.5 | 48.2 |

The elevation floor (50°, 40° at depower 0.40) only stops a sweep that sinks; it
never pulls one down.

### Changes, all in `examples/plot_turn_rate_identification.jl`

`build_turn_rate_table.jl` keeps its constants: its `conditions` block fixes
the table rows to the 73° sweep. New sweep parameters are passed as keyword
arguments of `_run_turn_rate_sweep`, with today's values as defaults, so the table
build is unchanged.

1. **Floor.** Fly with `elevation_floor` = 10°. At 150 m of tether that is still
   26 m of height.
2. **Get the kite low.** Add a start elevation `elevation` (e.g. 30°) to
   `_run_turn_rate_sweep` and pass it to `init`. Check in the plot that the kite
   stays low: parked at a constant length it may climb back towards the zenith.
   If it does, centre the relay band on a crosswind heading instead of 0°, e.g.
   ±90°, as a second keyword argument.
3. **Fit window.** Add `max_elevation`: the fit uses only data below it. The rate
   is a backward difference and the delay search shifts the steering, so the
   samples must stay contiguous. So set the fit start to
   `max(T_START, first time the elevation falls below max_elevation)`, and warn
   if the kite later rises above `max_elevation` again, instead of masking single
   samples.
4. **Report.** Print the number of samples in the window, the spread of
   $v_\mathrm{a}$ and of $v_\tau/v_\mathrm{a}$ in it, the fit ($c_1$, $c_2$,
   dead time, lag, residual), and the table row at the same depower next to it.
5. **Plot.** The four panels of today, plus a line at `max_elevation` in the
   elevation panel and the fit window marked on the time axis.

### Done when

The sweep at depower 0.275 spends at least 30 s below `max_elevation` = 45°, and
the fit there can be compared with the 73° table row. The comparison is recorded
here: whether $c_1$, $c_2$, dead time and lag differ beyond the scatter the table
gives (`g_rel_std`, `delay_std`).

### Result (2026-09-29, depower 0.275, start 30°, floor 10°, `max_elevation` 45°)

- **Relay about heading 0°:** the kite climbs from 30° back to 70° in the 10 s
  before the relay starts and then hovers at 70 – 75°. No data below 45°.
- **Relay about heading 90° (crosswind):** the kite sinks below 45° at t = 34 s
  and on to the 10° floor at 66 s (`:low_elevation`, steering amplitude 0.10).
  Fit window 32 s, 1926 samples, all below 45°; $v_\mathrm{a}$ 11.7 – 15.3 m/s,
  $v_\tau/v_\mathrm{a}$ 0.08 – 0.55. So the window is a descent, not a steady
  operating point, and $v_\mathrm{a}$ is no higher than at 73°.

| | $c_1$ [1/m] | $c_2$ [-] | delay [s] | dead time [s] | lag [s] | residual [°/s] |
|---|---|---|---|---|---|---|
| below 45°, crosswind | 0.2195 | 2.42 | 0.407 | 0.089 | 0.367 | 0.77 |
| table row, 73° | 0.2449 | 1.44 | 0.379 | 0.143 | 0.267 | 1.20 |

- $c_1$ is 10 % lower and $c_2$ 68 % higher. The table's scatter is
  `g_rel_std` 0.145 and `delay_std` 0.016 s, so the delay (+0.028 s) and the
  split into dead time and lag differ by more than it too.
- The gain scatter `G_rel_std` of the new window is 2.4. $G = \dot\psi/(v_\mathrm{a} u_\mathrm{s})$
  ignores the gravity term, which is large when flying crosswind at a low
  elevation, so this is expected and not a failed fit; the residual of the full
  law is lower than at 73°.
- The heading swung from −64° to +115°, far outside the ±10° band: at this
  steering amplitude the relay does not hold the kite crosswind.

### A steady low-elevation pattern (2026-09-29)

A fixed crosswind band flies the kite out of the wind window: at heading 90° the
azimuth ran to 76°, where the kite sank. `_run_turn_rate_sweep` therefore got
`az_reverse` (the band's centre flips between ±90° past ±`az_reverse` of azimuth,
the turn always through heading 0, upwards) and `el_hold` (the centre is tilted
up below 30° and down above it, 3°/°, within ±`el_hold_tilt` of 90°). The kite
then flies a lazy-eight-like pattern low in the wind window. One flight per fixed
steering amplitude, 200 s each, fit window from the first sample below 55°, and a
joint fit of all flights at one depower (`joint_delay_lag_fit` in
`plot_turn_rate_identification.jl`).

**Which amplitudes fly.** At depower 0.275:

- 0.05 turns too weakly: it drifts to the edge of the wind window (azimuth 67 –
  78°, $v_\mathrm{a}$ ≈ 12 m/s) and sinks, even reversing at ±10°.
- 0.15 turns at ~90 °/s while the tape needs ~1 s to swing from +0.15 to −0.15:
  the relay overshoots its band by ~50°, past heading 180° (straight down), and
  loops into the ground within 28 s, also with the tilt limited to ±25°.
- 0.075 (reversing at ±20°), 0.10 (±30°) and 0.125 (±30°, tilt ±25°) fly the full
  200 s. These are `flight_settings` of the script.

**Results.** Dead time and lag are from each fit; $c_1$, $c_2$ of the table row are
from its pure-delay fit.

| depower | amplitude | flight | samples | elevation mean [°] | $v_\mathrm{a}$ [m/s] | $v_\tau/v_\mathrm{a}$ median | $c_1$ | $c_2$ | dead [s] | lag [s] |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.275 | 0.075 | 200 s | 10 691 | 32 | 14 – 51 | 0.94 | 0.268 | 4.02 | 0.062 | 0.067 |
| | 0.100 | 200 s | 10 849 | 34 | 21 – 51 | 0.88 | 0.273 | 3.12 | 0.069 | 0.067 |
| | 0.125 | 200 s | 10 928 | 23 | 21 – 50 | 0.94 | 0.261 | 3.66 | 0.000 | 0.150 |
| | **joint** | | 32 324 | | | | **0.265** | **3.68** | **0.042** | **0.083** |
| | table row, 73° | | | 70 – 75 | ≈ 13 | | 0.245 | 1.44 | 0.143 | 0.267 |
| 0.35 | 0.075 | floor at 46 s | 1 219 | 35 | 14 – 20 | 0.46 | (0.051) | (1.26) | (1.373) | (0.000) |
| | 0.100 | floor at 43 s | 1 294 | 24 | 13 – 29 | 0.36 | 0.178 | 4.34 | 0.164 | 0.000 |
| | 0.125 | 200 s | 10 859 | 26 | 15 – 39 | 0.90 | 0.166 | 3.72 | 0.105 | 0.067 |
| | **joint** | | 13 228 | | | | **0.166** | **3.84** | **0.108** | **0.067** |
| | table row, 73° | | | ≈ 61 – 75 | ≈ 13 | | 0.168 | 2.43 | 0.222 | 0.233 |

The single fit of the short 0.35/0.075 flight (in brackets) is not meaningful;
the joint fit is carried by the steady 0.125 flight there.

Turn-rate VAF on each flight's window:

| depower | amplitude | joint fit | table $c_1$, $c_2$, 73° delay | table $c_1$, $c_2$, joint delay |
|---|---|---|---|---|
| 0.275 | 0.075 | 0.986 | 0.926 | 0.978 |
| | 0.100 | 0.997 | 0.883 | 0.987 |
| | 0.125 | 0.987 | 0.880 | 0.984 |
| 0.35 | 0.075 | 0.930 | 0.825 | 0.825 |
| | 0.100 | 0.980 | 0.773 | 0.928 |
| | 0.125 | 0.998 | 0.959 | 0.997 |

- **The delay is the main difference, and it is expected.** Dead time plus lag is
  0.125 s (0.275) and 0.175 s (0.35) here, against 0.41 and 0.46 s at 73°, where
  $v_\mathrm{a}$ is only ≈ 13 m/s. The pattern law of V4, a response time of
  0.14 s · (34/$v_\mathrm{a}$)^0.74, already models this; at $v_\mathrm{a}$ ≈ 30 – 35
  m/s it gives ≈ 0.14 – 0.15 s. With the joint fit's delay, the table's own $c_1$
  and $c_2$ explain the steady flights almost as well as the joint fit (VAF 0.978 –
  0.997 against 0.986 – 0.998).
  **Correction:** the first run of this section (one steady flight) reported that
  "the 73° coefficients do not carry over" (VAF 0.70 – 0.88). That comparison
  used the unscaled 73° delay; most of the gap was the delay, not $c_1$, $c_2$.
- **$c_1$:** 8 % above the table at depower 0.275 (0.265 against 0.245), equal to
  it at 0.35 (0.166 against 0.168). V4 found the table's $c_1$ within ±5 % in
  pattern flight.
- **$c_2$ is 1.6 – 2.5 × the table's** at both depowers (3.7 – 3.8 against 1.4
  and 2.4). At 73° $\cos\beta$ is small, so the gravity term is poorly identified
  there. Its effect on the VAF is small (last column), but it is the term the
  stability model's gravity pole comes from.
- $v_\tau/v_\mathrm{a}$ now spans ≈ 0 – 1.3, median ≈ 0.9: the variation the new
  law's mass term needs (Identification, "Identifiability of $e$"). But the
  steady flights are at $v_\mathrm{a}$ ≥ 14 m/s, mostly above 20 m/s, where the
  current law already fits with VAF ≥ 0.98. V3's failure was below 20 m/s.

### Low $v_\mathrm{a}$ and the extended law (2026-09-29, depower 0.275)

**Low $v_\mathrm{a}$ by lower wind.** `v_wind = 6.5` (m/s, against the table's
9.51): 0.10 and 0.125 fly the full 200 s at $v_\mathrm{a}$ 12.7 – 36 m/s, with 38 and
15 % of the samples below 20 m/s. 0.075 sinks to the floor after 70 s, and its
single fit is meaningless (negative $c_1$, dead time at the search limit). At
5.0 m/s 0.10 drifts to 84° of azimuth and sinks after 69 s.

**The extended law against the current one.** Fitted on the five steady flights at
depower 0.275 (0.075, 0.10, 0.125 at 9.51 m/s; 0.10, 0.125 at 6.5 m/s), each law
with its own dead time and lag, by `extended_law_fit` in
`plot_turn_rate_identification.jl`:

$$
\text{rate} = \frac{c_1 v_\mathrm{a}^2 u_\mathrm{s} + c_2 \sin(\text{angle}) \cos\beta}{v_\mathrm{a} + e\, v_\tau},
$$

$e = k_4 m / k_2$, $e = 0$ the current law, searched over 0 – 3.

| rate | law | $e$ | $c_1$ | $c_2$ | dead [s] | lag [s] | rms [°/s] |
|---|---|---|---|---|---|---|---|
| heading | current = best | 0 | 0.267 | 3.18 | 0.042 | 0.100 | 4.83 |
| course | current | 0 | 0.274 | 1.27 | 0.000 | 0.167 | 12.79 |
| course | extended | 3.0 (grid end) | 1.027 | −3.45 | 0.042 | 0.100 | 11.01 |

VAF per $v_\mathrm{a}$ bin:

| $v_\mathrm{a}$ [m/s] | samples | heading, current | course, current | course, extended |
|---|---|---|---|---|
| 10 – 15 | 2 349 | 0.983 | 0.961 | 0.873 |
| 15 – 20 | 3 631 | 0.997 | 0.958 | 0.982 |
| 20 – 25 | 6 166 | 0.999 | 0.970 | 0.990 |
| 25 – 30 | 7 081 | 0.997 | 0.957 | 0.984 |
| 30 – 40 | 18 486 | 0.995 | 0.954 | 0.972 |
| 40 – 60 | 15 810 | 0.986 | 0.913 | 0.925 |

- **Heading: the mass term is not supported.** The best $e$ is 0, and the
  current law explains the heading rate with VAF 0.983 – 0.999 in every bin,
  10 – 15 m/s included. The success criterion (VAF ≥ 0.90 in every bin) is met
  by the current law.
- **So V3's failure below 20 m/s does not reproduce in these flights.** They fly
  at a constant tether length, with the steering at a moderate fixed amplitude.
  V3's low-$v_\mathrm{a}$ samples were the fig8 transitions (steering near the
  clamp) and the reel-out (radial speed, a faster kite than the table), which is
  where `865b3fd` placed the cause.
- **Course:** the current law fits it worse than the heading. The extended law
  improves it above 15 m/s, but runs to the end of the $e$ grid with a negative
  $c_2$, and is worse at 10 – 15 m/s: not a physical fit. The course rate is not
  a better quantity to model the steering response on.

### Reeling out (2026-09-29, depower 0.275)

`_run_turn_rate_sweep(...; v_reelout)` reels out at a constant speed from `T_START`
(ramped in over 2 s, fed forward to the length loop) up to `REELOUT_L_MAX` = 380 m;
`v_reelout` in the script. At 1.0 m/s the tether grows from 150 to 339 m in
the 200 s. The steady flights (0.10 and 0.125; at 9.51 m/s also 0.075; 0.075 sinks
at 6.5 m/s):

| wind [m/s] | $v_\mathrm{a}$ [m/s] | below 20 m/s | best $e$ (heading) | $c_1$ | $c_2$ | dead [s] | lag [s] | rms [°/s] |
|---|---|---|---|---|---|---|---|---|
| 9.51 | 15 – 47 | 0 – 3 % | 0.05 | 0.269 | 3.46 | 0.042 | 0.100 | 2.52 |
| 6.5 | 5.7 – 31 | 23 – 36 % | 0 | 0.260 | 2.51 | 0.092 | 0.133 | 2.98 |

Heading-rate VAF of the current law per $v_\mathrm{a}$ bin:

| $v_\mathrm{a}$ [m/s] | 5 – 10 | 10 – 15 | 15 – 20 | 20 – 30 | 30 – 60 |
|---|---|---|---|---|---|
| 6.5 m/s, reeling out | **−6.95** (1 157) | 0.952 (927) | 0.999 | 0.998 | 0.999 |
| 9.51 m/s, reeling out | | | 0.70 (298) | 0.998 – 0.999 | 0.999 |

- **Reeling out reproduces V3's low-$v_\mathrm{a}$ error.** Below 10 m/s the current
  law has the right shape (correlation 0.97 with the measurement) but predicts 3 ×
  the turn rate (least-squares slope of the measured on the predicted 0.31); at
  10 – 15 m/s the slope is 0.88. At constant length there was no such error down
  to 10 m/s (previous section), so it comes with the reel-out.
- **The mass term does not explain it.** Refitted on $v_\mathrm{a}$ < 15 m/s only,
  its best $e$ is still 0. It lowers the turn rate where $v_\tau/v_\mathrm{a}$ is
  large, but the worst samples have the SMALLER ratio: median 0.41 below 10 m/s,
  0.82 at 10 – 15 m/s.
- **A constant term helps only in part.** With the denominator $v_\mathrm{a} + E$
  (turn rate ∝ $v_\mathrm{a}^2$ at low $v_\mathrm{a}$) the best $E$ is 6 m/s: VAF
  −2.99 below 10 m/s and 0.968 at 10 – 15 m/s, with $c_1$ 0.327, $c_2$ 3.77. A
  diagnostic, not a law.
- **The operating range reaches into it at the lowest wind speeds.** Reel-out
  samples ($v_\mathrm{ro}$ > 0.1 m/s, phase ≥ 3) below $v_\mathrm{a}$ = 12 m/s in the 22
  scenarios of 2026-09-29: Maasvlakte 3.5 m/s 82 % (lowest 7.7 m/s), Maasvlakte
  4 m/s 43 % (10.0), Cabauw 3 m/s 16 % (9.9); none in the other 19, whose lowest
  $v_\mathrm{a}$ is 13 – 33 m/s. (The 12.8 m/s of V4 was the range of its logs, not
  of the scenarios.) At 10 – 15 m/s the current law meets the success criterion
  (VAF 0.952 ≥ 0.90); below 10 m/s, reached only at Maasvlakte 3.5 m/s, it does
  not.

### Conclusion so far

The extended law with $v_\tau$ is not supported by any of the flights: at constant
length and reeling out, at $v_\mathrm{a}$ 6 – 51 m/s, its best $e$ is 0 or 0.05. The
current heading law with a delay shortened for high $v_\mathrm{a}$ (the pattern law)
explains the heading rate with VAF ≥ 0.95 from $v_\mathrm{a}$ = 10 m/s up. The one
consistent difference from the table is $c_2$, 1.6 – 2.6 × the 73° rows, which the
low-elevation flights identify far better. The low-$v_\mathrm{a}$ gain drop while
reeling out is real and not what the mass term describes; below 10 m/s it
concerns only the lowest wind speed flown (Maasvlakte 3.5 m/s).

Open, if it is worth pursuing:

1. Whether $c_2$ from the low flights should replace the table's, and what that
   does to the gravity pole in the stability analysis (Table 6 of the paper).
2. What makes the turn rate drop below $v_\mathrm{a}$ ≈ 12 m/s while reeling out.
   Dropped (2026-09-29): only the lowest wind speeds reel out there (see above).

### The low pattern over depower (2026-09-29)

`plot_turn_rate_vs_depower.jl` flies the low pattern (the three amplitudes of
`flight_settings`, 9.51 m/s, constant length) at every depower of the table and
plots the joint fit of the steady flights over the depower next to the 73° rows
(results in `output/turn_rate_low_flights.csv`):

| depower | steady flights | $c_1$ low / table | $c_2$ low / table | dead + lag [s] low / table | $v_\mathrm{a}$ [m/s] |
|---|---|---|---|---|---|
| 0.25 | 2 of 3 | 0.306 / 0.269 | 3.15 / 0.67 | 0.11 / 0.39 | 15 – 55 |
| 0.275 | 3 | 0.265 / 0.245 | 3.68 / 1.44 | 0.13 / 0.41 | 14 – 51 |
| 0.30 | 3 | 0.233 / 0.219 | 3.69 / 1.85 | 0.14 / 0.42 | 12 – 47 |
| 0.325 | 2 | 0.195 / 0.192 | 3.70 / 2.20 | 0.16 / 0.44 | 16 – 43 |
| 0.35 | 1 | 0.166 / 0.168 | 3.71 / 2.43 | 0.18 / 0.46 | 15 – 39 |
| 0.375 | 1 | 0.144 / 0.147 | 3.86 / 2.43 | 0.19 / 0.46 | 12 – 35 |
| 0.40 | 0 | 0.135 / 0.131 | 4.17 / 2.45 | 0.24 / 0.47 | 14 – 25 |

- **$c_1$ agrees with the table from depower 0.325 up** (−2 to +3 %) and is higher
  below it: +6 % at 0.30, +8 % at 0.275, +14 % at 0.25.
- **$c_2$ rises with the depower, much less than the table's:** 3.15 ± 0.23 at
  0.25, 3.68 – 3.71 from 0.275 to 0.35, 3.86 ± 0.25 at 0.375 and 4.17 at 0.40
  (±2 standard errors; no bars at 0.40, see Reliability). The step from 0.25 to
  0.275 is larger than the bars, so $c_2$ is not constant; between 0.275 and 0.35
  it is flat within them. The table's $c_2$ rises from 0.67 to 2.45; at 73°
  $\cos\beta$ is small, so it is poorly identified there.
- **The delay grows with the depower in both,** 0.11 → 0.24 s low and 0.39 →
  0.47 s in the table, the low flights shorter because of their higher $v_\mathrm{a}$.
- **Reliability:** at 0.35 and 0.375 only 0.125 flew steadily, and at 0.40 none
  did: the row at 0.40 is from flights that sank, $v_\mathrm{a}$ ≤ 25 m/s.
- **For question 1:** the current law's gravity term is $c_2/v_\mathrm{a}$, so with
  $c_2$ ≈ 3.7 (depower 0.275 – 0.35) the coefficient of Eq. (9) of the paper, $c_3 = c_2/v_\mathrm{a}$, is
  0.23 1/s at $v_\mathrm{a}$ ≈ 16 m/s and 0.11 1/s at 35 m/s, while the paper
  uses a constant 0.23 1/s from `identify_c3.jl` (removed on 2026-10-01). Which form holds is the next
  thing to check.

### The low-flight coefficients in the stability analysis (2026-09-29)

**The form of the gravity term** (`examples/gravity_term_form.jl`). The heading
rate fitted as $c_1 v_\mathrm{a} u_\mathrm{s} + c_g \sin\psi\cos\beta\, v_\mathrm{a}^{-n}$,
$n$ = 0 – 2, each with its own dead time and lag, on the steady flights at
depower 0.275 ($n = 0$: the constant $c_3$ of Eq. (9); $n = 1$: $c_2/v_\mathrm{a}$):

| flights | rms $n = 0$ [°/s] | rms $n = 1$ [°/s] | best $n$ | $c_3$ ($n = 0$) [1/s] | $c_2$ ($n = 1$) |
|---|---|---|---|---|---|
| 9.51 m/s, constant length | 5.674 | 5.696 | 0.3 | 0.098 | 3.45 |
| 6.5 m/s, constant length | 2.164 | 2.167 | 0.5 | 0.112 | 2.81 |
| reeling out, 9.51 + 6.5 m/s | 3.173 | 3.251 | 0.5 | 0.106 | 2.90 |
| all constant length | 4.818 | 4.833 | 0.5 | 0.103 | 3.18 |

- **The data do not decide the form.** The two differ by 0.1 – 2.5 % in rms, and
  the best exponent lies between them, over $v_\mathrm{a}$ 6 – 51 m/s.
- **They do decide the size:** about 0.10 1/s in the operating range, in either
  form, less than half the 0.23 1/s of Eq. (9) (`identify_c3.jl`, removed on 2026-10-01, fitted on the
  flown figures of eight). Why that fit gives twice the value is not yet known.
- The $1/v_\mathrm{a}$ form follows from the force balance (gravity against the
  aerodynamic damping $\propto v_\mathrm{a}$); a constant $c_3$ is empirical. So
  $c_2/v_\mathrm{a}$ is justified by the physics, and the data are consistent with it.

**Is the model still conservative?** (`gravity_term_form.jl`, part 2.) The pattern
model of `validate_margins.jl` at the six points where margins were measured,
with the table's $c_1$ and $C_3$ (A) or the low-flight $c_1$ and $c_2/v_\mathrm{a}$
(B), the stable sign of the gravity pole as in the original comparison:

| point | measured DM / GM | A DM / GM | B DM / GM | B against measured | $c_3$ A → B [1/s] |
|---|---|---|---|---|---|
| 150 m, 33.6 m/s | 0.274 s / 3.51 | 0.222 s / 2.59 | 0.182 s / 2.33 | −34 / −34 % | 0.23 → 0.106 |
| 200 m, 22.4 m/s | 0.480 s / 3.35 | 0.305 s / 2.59 | 0.251 s / 2.34 | −48 / −30 % | 0.23 → 0.160 |
| 200 m, 34.7 m/s | 0.324 s / 4.11 | 0.271 s / 2.89 | 0.224 s / 2.60 | −31 / −37 % | 0.23 → 0.103 |
| 300 m, 23.7 m/s | 0.301 s / 4.56 | 0.326 s / 2.62 | 0.266 s / 2.37 | −12 / −48 % | 0.23 → 0.151 |
| 300 m, 33.6 m/s | 0.363 s / 5.0 | 0.316 s / 2.81 | 0.258 s / 2.53 | −29 / −49 % | 0.23 → 0.106 |
| 300 m, 40.1 m/s | 0.344 s / 3.80 | 0.312 s / 3.06 | 0.257 s / 2.75 | −25 / −28 % | 0.23 → 0.089 |

B is below every measured margin, lower than A: for the stable sign a smaller
gravity term removes stabilization, and at the fig8 depower 0.27 B's $c_1$ is
~10 % above the table. The operating points are rebuilt from $L$ and
$v_\mathrm{a}$ ($v_\mathrm{k} = 0.96\,v_\mathrm{a}$), not from the run records, so A does
not reproduce the earlier model column exactly (delay margin 8 – 12 % higher,
above the measurement at 300 m, 23.7 m/s); A against B is like for like.

**The scenarios** (`examples/stability_new_coeffs.jl`, removed on 2026-10-01). The worst bin of each
reel-out scenario, 19 of 22: Cabauw 3 m/s and Maasvlakte 3.5 and 4 m/s are left
out, their phase-4 $v_\mathrm{a}$ drops to 10.1, 8.3 and 10.6 m/s. A as Table 6;
B the plant's $c_1$, $c_2/v_\mathrm{a}$ from the low flights, the pattern-law delay;
C as B with the low flights' dead time and lag, independent of $v_\mathrm{a}$. The
controller keeps the table's $c_1$ (as flown); the worse sign of the gravity pole.

| | $\alpha$ inner | $\alpha$ guided | delay margin guided [s] |
|---|---|---|---|
| A (Table 6) | 0.89 – 1.33 | 0.308 – 0.447 | 0.203 – 0.304 |
| B | 0.83 – 1.34 | 0.385 – 0.536 | 0.252 – 0.329 |
| C | 1.21 – 1.39 | 0.456 – 0.592 | 0.298 – 0.370 |

- B raises $\alpha$ guided by 0.04 – 0.09 in every scenario (worst: Cabauw 10 m/s,
  0.308 → 0.385), mostly because the destabilizing gravity pole is smaller
  ($c_2/v_\mathrm{a}$ ≈ 0.10 – 0.18 1/s at the worst bins' $v_\mathrm{a}$ 20 – 37 m/s).
- C is optimistic: one delay per depower, fitted at $v_\mathrm{a}$ 14 – 51 m/s, at
  every $v_\mathrm{a}$. B keeps the $v_\mathrm{a}$-dependent pattern law validated in V4.
- The three left out, A → B: Cabauw 3 m/s $\alpha$ guided 0.319 → 0.333, inner
  0.683 → 0.596; Maasvlakte 3.5 m/s 0.317 → 0.307, inner 0.599 → **0.449**;
  Maasvlakte 4 m/s 0.330 → 0.341, inner 0.675 → 0.585. Their worst bins are at
  $v_\mathrm{a}$ 15 – 17 m/s, where $c_2/v_\mathrm{a}$ ≈ 0.22 – 0.25 1/s is no smaller
  than 0.23, and their depower ~0.29 has $c_1$ 6 – 8 % above the table.

**Assessment.** Switching the stability analysis to B is justified: the
coefficients are well identified in the low pattern, B keeps the model
conservative against every measured margin, and the $c_2/v_\mathrm{a}$ form follows
from the physics. The data do not favour it over a constant $c_3$; what they show
is that the gravity coefficient is ≈ 0.10 1/s, not 0.23. Limits to state: the
coefficients come from relay flights at 150 m and constant length, the worst bins
are at 155 – 205 m while reeling out; few steady flights at depower ≥ 0.35; no
held-out validation; at the three lowest wind speeds B does not help, and below
$v_\mathrm{a}$ = 10 m/s while reeling out neither law describes the kite.

## Decisions (settled 2026-09-29)

### Course or heading: the heading

Both were fitted on the same low-pattern flights (see "Low $v_\mathrm{a}$ and the
extended law"). The current law explains the HEADING rate with VAF 0.983 – 0.999 in
every $v_\mathrm{a}$ bin; the COURSE rate fits worse with the current law (0.91 –
0.97), and the extended law on it runs to the end of its $e$ grid with a negative
$c_2$, not a physical fit. The heading stays the fitted quantity, as in
`turn_rate_coeffs.yaml`.

### Conventions, as implemented

- $v_\tau = \sqrt{v_\mathrm{k}^2 - v_\mathrm{ro}^2}$, the kite speed perpendicular to
  the tether (`law_data` in `plot_turn_rate_identification.jl`).
- The law in V3Kite's sign convention, with $k_2 = 1$:
  $\dot\psi = (c_1 v_\mathrm{a}^2 u_\mathrm{s} + c_2 \sin\psi \cos\beta)/(v_\mathrm{a} + e\, v_\tau)$,
  so $c_1 = k_1/k_2$, $c_2 = k_3 m g/k_2$, $e = k_4 m/k_2$, and $e = 0$ is the current
  law with the table's meaning of $c_1$ and $c_2$.
- The gravity term keeps V3Kite's sign and angle (the heading). Whether its
  coefficient is $c_2/v_\mathrm{a}$ (the table's form) or a constant $c_3$ (as Eq. (9)
  of the paper uses) is still open, see "Open".

## Identification, as done

- **Fit.** For a fixed $e$ the law is linear in $c_1$, $c_2$. $e$ (0 – 3), the dead
  time (whole samples) and the first-order lag are searched on grids, each
  flight's steering filtered and shifted on its own (`extended_law_fit`). No
  nonlinear fit or errors-in-variables start value was needed.
- **Identifiability of $e$.** The low flights span $v_\tau/v_\mathrm{a}$ ≈ 0 – 1.3,
  so $e$ is identifiable; it comes out 0 (0.05 at most).
- **Delay and lag.** Fitted with every law (`joint_delay_lag_fit`, now in
  `delay_lag_fit.jl`).
- **Error bars.** Standard errors from 20 s blocks, each refitted on its own
  (`block_standard_errors` in `plot_turn_rate_vs_depower.jl`); the linear fit's
  own standard errors are far too small with autocorrelated residuals.

## Data, as used

- **Relay flights in a low crosswind pattern**, not the scenario logs: fixed
  amplitudes 0.075, 0.10, 0.125, reversing in azimuth and holding ~30° of
  elevation (`_run_turn_rate_sweep`, `plot_turn_rate_identification.jl`). At
  9.51 m/s and 6.5 m/s of wind, at constant length and reeling out at 1 m/s, and
  at every depower of the table at 9.51 m/s (`plot_turn_rate_vs_depower.jl`).
  Their results are archived in `data/turn_rate_low_flights.tar.gz`.
- **The 22 scenario logs** only for the operating range: which $v_\mathrm{a}$ the
  reel-out actually flies.
- **Not done:** validation on held-out logs as in V3. The fits were compared on the
  flights they were fitted on, bin by bin.

## Success criterion: met by the current law

Turn-rate VAF ≥ 0.90 in every $v_\mathrm{a}$ bin, including 10 – 20 m/s. The
current law meets it from $v_\mathrm{a}$ = 10 m/s up, at constant length and
reeling out; the extended law adds nothing. Below 10 m/s while reeling out the
current law fails (VAF −6.95), which of the scenarios only Maasvlakte 3.5 m/s
reaches.

## Steps

1. ~~Extend `plot_turn_rate_identification.jl`: both laws, both angles, VAF per
   $v_\mathrm{a}$ bin.~~ Done.
2. ~~Fly and fit in the low pattern, checking the spread of
   $v_\tau/v_\mathrm{a}$.~~ Done, with the relay flights instead of the scenario
   logs.
3. Validate on held-out logs. Not done; only needed if a new law or new
   coefficients are to be adopted.

## Open

1. **Switch the stability analysis to B.** Done in the code on 2026-09-29:
   `PLANT_COEFFS` and `plant_coeffs(depower)` in `course_loop_model.jl` give the
   plant's $c_1$ and $c_2$ (depower 0.25 – 0.375, held at the ends), used by
   `stability_opt_reelout.jl` (hence `stability_global.jl`, `retune_guided.jl`),
   `stability_fig8.jl`, `validate_margins.jl`, `plot_frf_validation.jl` and
   `xtrack_step_analysis.jl`; the gain schedule keeps the turn-rate table.
   `C3_OVERRIDE` became `gravity_scale` (a factor on the gravity term, 0 = none).
   `C3` and `c2_at` stay for the comparisons. The stability overviews of both sites
   are regenerated with it. The paper was switched too (LearningControl
   `2c6f67c`, `9c7172a`: the low-flight $c_1$, $c_2$, the gravity pole
   $(c_2/v_\mathrm{a})\cos\beta$, no constant $c_3$), and the model section of
   `docs/course_loop_stability_reelout.md` describes the new plant (2026-10-01).

## Closed (2026-10-01), with known limitations

The plan is done. Left as known limitations, not pursued:

1. **Why `identify_c3.jl` gives 0.23 1/s** where the low flights give ≈ 0.10
   (likely its table delays, too long in pattern flight; not verified). The paper
   no longer uses $c_3$.
2. **No held-out validation** of the plant coefficients (Step 3); they are
   compared only on the flights they were fitted on.
3. **$c_1$ below depower 0.325** is 6 – 14 % above the table in the low pattern;
   the gain schedule keeps the table's $c_1$. The paper states this.
4. **Below $v_\mathrm{a}$ = 10 m/s while reeling out** neither law describes the
   kite; only Maasvlakte 3.5 m/s reaches it.

## Out of scope

The extended law is not adopted: none of the flights supports it. Adopting new
coefficients would touch everything that uses $c_1$ and $c_2$:
`turn_rate_coeffs.yaml`, the controller's gain schedule, the curvature
feedforward $u_\mathrm{ff} = \dot\psi_\mathrm{path}/(c_1 v_\mathrm{a})$, the
stability model in `course_loop_model.jl`, and the turn-rate law of the paper.
That is a separate decision.
