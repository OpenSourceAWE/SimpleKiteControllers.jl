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

A more accurate law keeps the kite's inertia term:

$$
\dot{\chi}_\mathrm{turn} \approx
-\frac{k_1 v_\mathrm{a}^2 u_\mathrm{s} - k_3 m g \sin\chi \cos\beta}
      {k_4 m v_\tau + k_2 v_\mathrm{a}}
$$

For $k_4 m v_\tau \ll k_2 v_\mathrm{a}$ it reduces to the current law, with
$c_1 = k_1/k_2$ and $c_2 = k_3 m g / k_2$ (up to the sign convention of
$u_\mathrm{s}$). So the assumption being dropped is that the inertia term is
negligible against the aerodynamic damping. That term weighs more at low
$v_\mathrm{a}$, which is where the current law fails, and it lowers the predicted
turn rate there, in the direction V3 measured.

## First step: identify at a lower elevation

Fit the CURRENT law on sweep data flown at a lower elevation, where the kite flies
faster crosswind and $v_\tau/v_\mathrm{a}$ is larger. Do not apply the new formula
yet. The result shows whether $c_1$, $c_2$, the dead time and the lag change with
the elevation. If they do, the inertia term is a candidate cause.

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
  law's inertia term needs (Identification, "Identifiability of $e$"). But the
  steady flights are at $v_\mathrm{a}$ ≥ 14 m/s, mostly above 20 m/s, where the
  current law already fits with VAF ≥ 0.98. V3's failure was below 20 m/s.

### Low $v_\mathrm{a}$ and the inertia law (2026-09-29, depower 0.275)

**Low $v_\mathrm{a}$ by lower wind.** `TR_V_WIND = 6.5` (m/s, against the table's
9.51): 0.10 and 0.125 fly the full 200 s at $v_\mathrm{a}$ 12.7 – 36 m/s, with 38 and
15 % of the samples below 20 m/s. 0.075 sinks to the floor after 70 s, and its
single fit is meaningless (negative $c_1$, dead time at the search limit). At
5.0 m/s 0.10 drifts to 84° of azimuth and sinks after 69 s.

**The inertia law against the current one.** Fitted on the five steady flights at
depower 0.275 (0.075, 0.10, 0.125 at 9.51 m/s; 0.10, 0.125 at 6.5 m/s), each law
with its own dead time and lag, by `inertia_law_fit` in
`plot_turn_rate_identification.jl`:

$$
\text{rate} = \frac{c_1 v_\mathrm{a}^2 u_\mathrm{s} + c_2 \sin(\text{angle}) \cos\beta}{v_\mathrm{a} + e\, v_\tau},
$$

$e = k_4 m / k_2$, $e = 0$ the current law, searched over 0 – 3.

| rate | law | $e$ | $c_1$ | $c_2$ | dead [s] | lag [s] | rms [°/s] |
|---|---|---|---|---|---|---|---|
| heading | current = best | 0 | 0.267 | 3.18 | 0.042 | 0.100 | 4.83 |
| course | current | 0 | 0.274 | 1.27 | 0.000 | 0.167 | 12.79 |
| course | inertia | 3.0 (grid end) | 1.027 | −3.45 | 0.042 | 0.100 | 11.01 |

VAF per $v_\mathrm{a}$ bin:

| $v_\mathrm{a}$ [m/s] | samples | heading, current | course, current | course, inertia |
|---|---|---|---|---|
| 10 – 15 | 2 349 | 0.983 | 0.961 | 0.873 |
| 15 – 20 | 3 631 | 0.997 | 0.958 | 0.982 |
| 20 – 25 | 6 166 | 0.999 | 0.970 | 0.990 |
| 25 – 30 | 7 081 | 0.997 | 0.957 | 0.984 |
| 30 – 40 | 18 486 | 0.995 | 0.954 | 0.972 |
| 40 – 60 | 15 810 | 0.986 | 0.913 | 0.925 |

- **Heading: the inertia term is not supported.** The best $e$ is 0, and the
  current law explains the heading rate with VAF 0.983 – 0.999 in every bin,
  10 – 15 m/s included. The success criterion (VAF ≥ 0.90 in every bin) is met
  by the current law.
- **So V3's failure below 20 m/s does not reproduce in these flights.** They fly
  at a constant tether length, with the steering at a moderate fixed amplitude.
  V3's low-$v_\mathrm{a}$ samples were the fig8 transitions (steering near the
  clamp) and the reel-out (radial speed, a faster kite than the table), which is
  where `865b3fd` placed the cause.
- **Course:** the current law fits it worse than the heading. The inertia law
  improves it above 15 m/s, but runs to the end of the $e$ grid with a negative
  $c_2$, and is worse at 10 – 15 m/s: not a physical fit. The course rate is not
  a better quantity to model the steering response on.

Next, if the plan continues: test the reel-out, the one effect these flights
leave out, by flying the same pattern with the winch reeling out (constant
$v_\mathrm{ro}$), and fit both laws on it. If the heading law still holds there,
the low-$v_\mathrm{a}$ error of V3 is the steering clamp, and the turn-rate law
needs no change.

## Decisions to make first

### Course or heading

The new law is for the COURSE rate $\dot\chi$. The script and
`turn_rate_coeffs.yaml` fit the HEADING rate $\dot\psi$.

- In the relay sweep the kite nearly hovers at 73° elevation. There the course,
  the direction of $v_\mathrm{k}$, is poorly defined and noisy.
- If $\dot\chi$ is fitted, the pattern-flight logs are the data source (see Data).
- If $\dot\psi$ is fitted, the use of the formula for the heading needs a
  justification: the course/heading difference is what V1 found the model got
  wrong.

### Conventions

- $v_\tau$: tangential kite speed, perpendicular to the tether,
  $\sqrt{v_\mathrm{k}^2 - v_\mathrm{ro}^2}$.
- Sign of $u_\mathrm{s}$: the formula has a leading minus, while V3Kite's law has
  $+c_1 v_\mathrm{a} u_\mathrm{s}$. Match the formula to V3Kite's steering sign.
- Zero of $\chi$: it decides the sign of the gravity term. Check it against
  $c_3 = 0.23\,$1/s of Eq. (9) of the paper.

## Identification

- **Scale.** The $k_i$ are fixed only up to a common factor. Set $k_2 = 1$ and
  fit three parameters: $a = k_1/k_2$, $b = k_3 m g/k_2$, $e = k_4 m/k_2$.
- **Start value by linear least squares.** Multiplied by the denominator, the law
  is linear in $(a, b, e)$:
  $\dot\chi\, v_\mathrm{a} = -a v_\mathrm{a}^2 u_\mathrm{s} + b \sin\chi\cos\beta - e\, v_\tau \dot\chi$.
  The measured $\dot\chi$ appears on the right-hand side, so this estimate is
  biased (errors in variables). Use it only as the start of a nonlinear fit of
  the original form.
- **Identifiability of $e$.** It separates from the rest only if $v_\tau/v_\mathrm{a}$
  varies enough in the data. Check its spread before fitting.
- **Delay and lag.** Keep the dead time $\tau$ and the kite lag $T$: fit them
  together with the new law, as `fit_delay_lag` does now with $c_1$, $c_2$.
  Otherwise the phase lag goes into the new coefficients.

## Data

- The relay sweeps of `build_turn_rate_table.jl`: one elevation, one tether
  length, $v_\mathrm{a} \approx 11$ – 16 m/s.
- The archived pattern-flight scenarios (22 runs, both sites): $v_\mathrm{a}$ 12.8 –
  40.6 m/s, large variation of $v_\tau$, and the flight the law is used for.
- Fit on part of them and validate on held-out logs, as V3 did
  (`examples/replay_prediction.jl`).

## Success criterion

Turn-rate VAF ≥ 0.90 in every $v_\mathrm{a}$ bin, including 10 – 20 m/s, where the
current law fails, with no loss against the current law above 20 m/s.

## Steps

1. Extend `examples/plot_turn_rate_identification.jl`: fit the new law next to
   the current one, plot both against the measured rate, and print the error of
   each per $v_\mathrm{a}$ bin.
2. Fit on the pattern-flight logs, and check the spread of $v_\tau/v_\mathrm{a}$ first.
3. Validate on held-out logs against the success criterion.

## Out of scope for now

If the new law is adopted, everything that uses $c_1$ has to follow:
`turn_rate_coeffs.yaml`, the controller's gain schedule, the curvature
feedforward $u_\mathrm{ff} = \dot\psi_\mathrm{path}/(c_1 v_\mathrm{a})$, the
stability model in `course_loop_model.jl`, and the turn-rate law of the paper
(Eq. (9) and the equations it builds on).
This plan is only the identification study; adopting the law is a separate
decision.
