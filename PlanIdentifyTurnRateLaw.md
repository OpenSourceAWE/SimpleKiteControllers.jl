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
