# Course-loop stability analysis (disk margins)

2026-09-25. Findings from `examples/stability_course_controller.jl`, a
disk-margin analysis of the linearized inner loop of `simple_fig8.jl`. The
method follows WinchControllers.jl's `examples/stability_lfc.jl`.
**Status: model validated against `simple_fig8.jl` runs and relay sweeps; the
gain schedule is fixed.** The steering actuator is modelled as a lag, and the
kite's dead time falls with the apparent wind speed. With the original schedule
the pattern loop was robust in the pattern itself (α ≈ 0.8 at 35 m/s) but not
at low `v_a`: α < 0.5 below about 20 m/s, unstable below about 12 m/s. Since
`v_app_min_pattern = 23` it has α ≥ 0.55 at every `v_a`
([Transition fix](#transition-fix-v_app_min_pattern)). The rate-limited tape
causes overshoot on large turns but no limit cycle
([Large errors](#large-errors-the-tapes-rate-limit)). The first version of the
model, with the turn-rate table's delay as pure dead time, predicted α = 0.06;
that result is kept below for reference.

## Model

- **Controller:** the exact discrete transfer function of the `DiscretePID` in
  `CourseController`, from the regulated error to `rel_steering`:
  `C(z) = K + bd·(z-1)/(z-ad)`, with `ad = Td/(Td+N·Ts)` and `bd = K·N·ad`
  (DiscretePIDs' backward-Euler filtered derivative). The integral term
  `K·Ts/Ti/(z-1)` is included when `heading_i` is a number.
  - Gain schedule: `K = heading_p · v_app_ref / max(v_a, v_app_min)`, times
    `entry_gain` below phase 3; from phase 3 on `v_a` is also floored at
    `v_app_min_pattern`.
- **Actuator:** a first-order lag from the commanded (`set_steering`) to the
  applied steering (`steering`), `T_act = ACTUATOR_LAG = 0.43 s`. It stands in
  for the KCU tape, which KitePodModels steps as
  `u̇ = clamp(steering_gain·(u_cmd − u), ±v_steering)` with `steering_gain = 3`
  and `v_steering = 0.2 s⁻¹`: a 0.33 s lag for small commands, rate-limited
  for larger ones. 0.43 s is its equivalent at the amplitudes of the pattern;
  see [Log validation](#log-validation-2026-09-25).
- **Kite:** the identified turn-rate law linearized about heading `ψ0`:

      ψ̇ = c1·v_a·u_s(t - τ_kite) + c2/v_a·cos(ψ0)·cos(β)·δψ

  - `c1` and `c2` come from `turn_rate_coeffs(fcs.body_damping, depower)`
    (`data/turn_rate_coeffs.yaml`).
  - The dead time scales with the apparent wind speed,
    `τ_kite = delay · (v_app_sweep / v_a)^1.24` (`kite_delay`), where `delay`
    and `v_app_sweep` are the table's `delay` and `v_app` for the depower: the
    row's dead time and the mean `v_a` of the relay sweep it was identified at.
    See [Kite dead time over v_a](#kite-dead-time-over-v_a).
  - The gravity term is a slow real pole. Both signs are checked (β = `el_center`)
    and the worse result is reported. In practice it matters only at very low `v_a`.
  - The plant is ZOH-discretized at `1/sample_freq`, and the dead time is an
    exact `round(τ_kite/Ts)`-sample shift register.
- **Margins:**
  - `RobustAndOptimalControl.diskmargin` gives the balanced disk margin α with its
    gain and phase ranges.
  - The delay margin is computed by the script's own `delay_margin`, because
    `ControlSystemsBase.delaymargin` uses the unwrapped phase margin (e.g. 374°
    instead of 14°) and so reported 1.5 s instead of 57 ms. It is 0 when the
    closed loop is already unstable.

## Log validation (2026-09-25)

Log `output/fig8_200m.arrow` of `simple_fig8.jl` (project
`system_fig8_200m.yaml`, 90 s). Window: phase 4 from 20 s after its start,
t = 55 – 90 s. Depower 0.270, `v_app` 34 – 38 m/s, pure course feedback
(`w_course` = 1).

1. **No oscillation near 0.8 Hz.** About 99 % of the power of `set_steering`
   is below 0.5 Hz. The peaks are at 0.08 Hz (the lap, ≈ 12.5 s) and its odd
   harmonics 0.23 and 0.39 Hz. There is no measurable power between 0.6 and
   1.0 Hz, where a loop with α = 0.06 (|S| ≈ 17) would ring.
2. **The actuator is the lag.** The applied steering never moves faster than
   0.2 s⁻¹ and sits on that rate limit 25 % of the time. It trails the command
   by 0.042 on average, 0.154 at most. A first-order lag with T = 0.43 s
   reproduces `set_steering` → `steering` with 1 % unexplained variance.
3. **The kite itself is fast.** `V3Kite.identify_turn_rate_law` on the window
   (applied steering → turn rate) gives `c1 = 0.2503` (the table: 0.2494 at
   depower 0.27) and a delay of **0.12 s** (correlation 0.995). From the command
   instead, the cross-correlation delay is 0.47 s. That this is close to the
   table's 0.42 s is a coincidence: the table's value is the kite alone, at
   lower `v_a`, see [Kite dead time over v_a](#kite-dead-time-over-v_a).
4. Fits of the heading rate against the command, `ψ̇ = c1·v_a·u`:

   | Form | Dead time | Lag | c1 | Unexplained |
   |---|---|---|---|---|
   | Pure dead time (the first model) | 0.44 s | – | 0.224 | 5.9 % |
   | Pure first-order lag | 0 | 0.54 s | 0.269 | 2.6 % |

### Kite dead time over v_a

The table's `delay` is identified from the *applied* steering too, yet it was
0.42 s against 0.12 s in the pattern. The cause is the apparent wind speed.
Two relay sweeps at depower 0.275 (the `build_turn_rate_table.jl` loop,
re-run with the wind as a parameter), and the fig8 log, all fitted with
`identify_turn_rate_law`:

| Run | v_a | c1 | Dead time (xcorr) | τ·v_a | LS fit: dead + lag |
|---|---|---|---|---|---|
| Relay sweep, 9.51 m/s wind (the table's condition) | 13.3 m/s | 0.2445 | 0.417 s | 5.5 m | 0.15 + 0.28 s |
| Relay sweep, 15 m/s wind | 22.5 m/s | 0.2573 | 0.217 s | 4.9 m | 0.10 + 0.12 s |
| `simple_fig8.jl`, phase 4 | 36.3 m/s | 0.2503 | 0.12 s | 4.3 m | – |

- The 9.51 m/s sweep reproduces the table's row exactly (0.417 s, `c1` 0.2445).
- The three dead times lie on `τ ∝ v_a^-1.24` to within 1 ms: the kite's
  response takes roughly a fixed distance flown, 4 – 6 m, not a fixed time.
- Split into 10 s segments, the fig8 log agrees: 0.29 s at 16 m/s (entry and
  transition), 0.14 s at 30 m/s, 0.10 – 0.13 s at 35 – 37 m/s.
- The kite's response is itself partly a lag (the LS column), which a pure
  dead time overstates; the model keeps the pure dead time, the conservative
  choice.

**The table now records `v_app`.** `build_turn_rate_table.jl` writes each
sweep's mean `v_a` to its row, and `turn_rate_coeffs` returns it (`NaN` for a
row without it). The whole `[0, 0, 40]` grid was re-run on 2026-09-25 (the 0.40
cell at its original `elevation_floor = 40`):

| Depower | 0.25 | 0.275 | 0.30 | 0.325 | 0.35 | 0.375 | 0.40 |
|---|---|---|---|---|---|---|---|
| `v_app` [m/s] | 13.47 | 13.32 | 13.17 | 13.05 | 13.05 | 12.98 | 12.84 |
| `delay` [s] | 0.383 | 0.417 | 0.450 | 0.483 | 0.517 | 0.567 | 0.600 |

The sweeps fly within 5 % of the same airspeed, so the rise of `delay` with
depower (+57 %) is a real depower effect, not a hidden `v_a` effect. `c1` and
`delay` reproduce the 2026-09-22 rows (`c1` to 1e-4, relative); `c2` moved by
up to 0.02 in places, within about one of its standard errors.

## Results (project `system_fig8_200m.yaml`, dt = 0.01 s)

Settings: `heading_p = 0.35`, `heading_d = 0.30 s`, `heading_d_n = 2`, no
integral, `depower_setpoint = 0.27`, `entry_depower = 0.37`, `entry_gain = 0.25`,
`v_app_min = 8`, `v_app_min_pattern = 23` (both changed, see below),
`body_damping = [0, 0, 40]`, actuator lag 0.43 s, kite dead time
`kite_delay(tc, v_a)`.

Pattern (phase ≥ 3, full gain), depower 0.27. α with the schedule as it was
(`v_app_min` only) and with the floor at 23 m/s from phase 3 on:

| v_a | Kite dead time | α, floor 10 m/s | α, floor 23 m/s | K, floor 23 m/s | Critical frequency |
|---|---|---|---|---|---|
| 5 m/s | 1.41 s | 0 (unstable) | 0.55 | 0.41 | 0.13 Hz |
| 10 m/s | 0.60 s | 0 (unstable) | 0.59 | 0.41 | 0.26 Hz |
| 15 m/s | 0.36 s | 0.28 | 0.60 | 0.41 | 0.38 Hz |
| 20 m/s | 0.25 s | 0.49 | 0.59 | 0.41 | 0.48 Hz |
| 27 m/s | 0.17 s | 0.69 | 0.69 | 0.35 | 0.56 Hz |
| 35 m/s | 0.13 s | 0.80 | 0.80 | 0.27 | 0.59 Hz |
| 45 m/s | 0.09 s | 0.92 | 0.92 | 0.21 | 0.63 Hz |

The rows below 13 m/s extrapolate the dead-time law.

Entry (phases 1 – 2, `entry_gain` 0.25), depower 0.37: α ≥ 1.10 from 10 m/s
up. At 5 m/s the gravity pole, at ~0 Hz, sets the margin: α = 0.29 with
`v_app_min` 10, α = 0.50 with 8 ([Entry at low v_a](#entry-at-low-v_a)).

Full gain at `v_a = v_app_ref = 27 m/s`, over depower:

| Depower | c1 [1/m] | Kite dead time | α | Critical frequency | Delay margin |
|---|---|---|---|---|---|
| 0.250 | 0.270 | 0.162 s | 0.661 | 0.59 Hz | 0.34 s |
| 0.275 | 0.245 | 0.173 s | 0.699 | 0.55 Hz | 0.39 s |
| 0.300 | 0.217 | 0.185 s | 0.748 | 0.52 Hz | 0.46 s |
| 0.325 | 0.190 | 0.196 s | 0.786 | 0.48 Hz | 0.54 s |
| 0.350 | 0.166 | 0.210 s | 0.851 | 0.44 Hz | 0.65 s |
| 0.375 | 0.144 | 0.228 s | 0.898 | 0.41 Hz | 0.78 s |
| 0.400 | 0.128 | 0.239 s | 0.951 | 0.38 Hz | 0.90 s |

For comparison, the pattern loop at depower 0.27 and `v_a` = 27 m/s with other
plant models:

| Plant | α | Critical frequency | Phase margin |
|---|---|---|---|
| Table delay 0.417 s as pure dead time, no actuator (first model) | 0.061 | 0.77 Hz | 3.5° |
| Actuator lag 0.43 s + table delay 0.417 s | 0.179 | 0.42 Hz | 10.3° |
| Actuator lag 0.43 s + 0.12 s (dead time at 36 m/s) | 0.828 | 0.61 Hz | 45.0° |
| Actuator lag 0.43 s + 0.174 s (current model at 27 m/s) | 0.686 | 0.56 Hz | 37.9° |
| Pure lag 0.54 s (log fit) | 1.195 | 0.51 Hz | 61.7° |

### Against the fig8 log

- **Phase 4 settling** (t = 35 – 39 s, `v_a` 24 – 29 m/s): the regulated error
  rings −13.5°, +4°, −5.5°, +2.9°, with a period of about 2 s (0.5 Hz) and
  decays by roughly half per half-cycle. The model predicts the critical
  frequency at 0.51 – 0.56 Hz with α ≈ 0.5 – 0.7 there, so this matches.
- **Phase 3** starts at `v_a` = 13.2 m/s, where the old schedule was at the
  edge of stability. In the run it did not matter: the error at the switch is
  136°, so the command sits on `max_steering` for 3.7 s and the loop is open.
  By the time it comes off the clamp, `v_a` is 19 m/s (α ≈ 0.45), and the
  error settles with one undershoot of 2.3°.

### Transition fix: `v_app_min_pattern`

The `1/v_a` schedule raises the full gain exactly where the kite's dead time is
longest. Holding `K` constant instead makes the loop's crossover scale with
`v_a` and its dead-time phase roughly constant, so α stays flat. Floors on the
schedule, α at depower 0.27 over `v_a` [m/s]:

| Floor \ v_a | 10 | 12 | 13 | 15 | 17.5 | 20 | 23 | 27 | 35 |
|---|---|---|---|---|---|---|---|---|---|
| 10 m/s (old) | 0.00 | 0.09 | 0.16 | 0.28 | 0.39 | 0.49 | 0.59 | 0.69 | 0.80 |
| 20 m/s | 0.49 | 0.49 | 0.50 | 0.50 | 0.49 | 0.49 | 0.59 | 0.69 | 0.80 |
| 23 m/s | 0.59 | 0.59 | 0.60 | 0.60 | 0.59 | 0.59 | 0.59 | 0.69 | 0.80 |
| 27 m/s | 0.71 | 0.71 | 0.72 | 0.71 | 0.70 | 0.70 | 0.69 | 0.69 | 0.80 |

Raising `v_app_min` itself was tried and rejected. It lowers the entry's gain
too, and the entry needs the boost against gravity: its α at 5 m/s fell to 0,
at 10 m/s from 1.10 to 0.80. The changed entry also shifted the whole run
(phase 3 began 2.3 s earlier). So the floor is a new setting,
`v_app_min_pattern` (`FC_Settings`, `CourseControllerSettings`, default 0 =
off), which `calc_steering` applies from phase 3 on only.
`data/fc_settings.yaml` sets it to 23 m/s: phase 4 never flies below 23.2 m/s
in the fig8 run, so the pattern's gain is untouched, and α ≥ 0.59 from 10 m/s
up.

`simple_fig8.jl`, `system_fig8_200m.yaml`, 7 m/s wind, 90 s:

| | Baseline | `v_app_min_pattern` 23 |
|---|---|---|
| Success criteria | 8/8 | 8/8 |
| RMS d (script, settled from 35 s) | 0.87° | 0.88° |
| RMS d / max d, t = 40 – 90 s | 0.80° / 2.16° | 0.81° / 2.19° |
| Min elevation, whole run | 16.0° | 15.8° |
| Phase 3 | 28.09 – 34.95 s | 28.09 – 35.00 s |
| Phase-3 approach after the clamp | undershoot to −2.3° | no undershoot |

The two runs agree until the command leaves the `max_steering` clamp at
t ≈ 31.5 s, the first moment the new floor changes anything.

### Entry at low v_a

Below `v_app_min` the entry's gain stops rising, so its loop gain falls with
`v_a` while the unstable gravity pole grows as `c2/v_a`. At 5 m/s they nearly
cancel. Entry α over `v_a` [m/s]:

| `v_app_min` \ v_a | 5 | 6 | 7 | 8 | 9 | 10 | 12 |
|---|---|---|---|---|---|---|---|
| 10 m/s (old) | 0.29 | 0.63 | 0.89 | 1.03 | 1.08 | 1.10 | 1.21 |
| 8 m/s | 0.50 | 0.79 | 0.89 | 0.93 | 1.02 | 1.10 | 1.21 |
| 6 m/s | 0.58 | 0.67 | 0.82 | 0.93 | 1.02 | 1.10 | 1.21 |

For the fig8 project `v_app_min` now acts on the entry only: the pattern
floor is 23 m/s, the attractor lead is off, and the feed-forward's `v_app`
floor matters only in phase 4, which flies above 16 m/s. `data/fc_settings.yaml`
sets 8 m/s.

`simple_fig8.jl`, `v_app_min_pattern` 23 in all four runs:

| Wind | `v_app_min` | Criteria | Laps | Min elevation (run) | Entry min v_a | Phase 3 |
|---|---|---|---|---|---|---|
| 7 m/s | 10 | 8/8 | 4.0 | 15.8° | 8.1 m/s | 28.1 – 35.0 s |
| 7 m/s | 8 | 8/8 | 4.0 | 15.8° | 8.1 m/s | 27.9 – 34.9 s |
| 5 m/s | 10 | **fail** | 2.0 | 0.1° | 6.3 m/s | 31.1 – 52.6 s |
| 5 m/s | 8 | **fail** | 2.0 | 1.4° | 6.3 m/s | 31.0 – 50.9 s |

At 7 m/s wind the change is invisible; at 5 m/s it helps a little. **The fig8
project fails at 5 m/s wind in any case**, also with the old schedule
(`v_app_min_pattern` off: 1.0 laps, min elevation 0.0°): the transition takes
20 s or more at `v_a` 6 – 10 m/s and the kite sinks to the ground. That is not
a loop-stability problem, see next steps.

### Large errors: the tape's rate limit

`step_response` in the script simulates the loop from a course error with the
real tape: the PD as `DiscretePID` computes it, the `max_steering` clamp, the
KitePodModels tape update, the kite's dead time and the linear turn-rate law,
at constant `v_a`. Overshoot [deg] with the pattern gain (floor 23 m/s):

| Error \ v_a | 13 | 15 | 20 | 27 | 35 m/s |
|---|---|---|---|---|---|
| 5° | 0.5 | 0.4 | 0.3 | 0.0 | 0.0 |
| 20° | 1.9 | 1.5 | 1.1 | 0.0 | 0.0 |
| 45° | 5.0 | 4.6 | 3.5 | 0.0 | 0.0 |
| 90° | 15.7 | 17.8 | 19.0 | 9.7 | 0.0 |
| 135° | 16.1 | 19.5 | 27.3 | 26.0 | 11.5 |
| 170° | 16.1 | 19.6 | 28.5 | 32.0 | 24.0 |

- **No limit cycle.** Every case settles to below 1° within 30 s. The rate
  limit does not destabilize the loop at any error or `v_a`.
- **Large turns overshoot.** The tape needs 3.2 s from `+max_steering` to
  `−max_steering`, and the kite keeps turning meanwhile: up to about 30° after
  a turn of 135° or more.
- **The model overstates it.** The turn-rate law is identified up to
  `|u| = 0.175`. In the fig8 log, at the clamp (0.32), the heading rate was
  only 0.6 – 0.75 of what the law predicts, and the course rate about 0.5. The
  real transition from 136° shows no overshoot at all (table above).

## Observations

1. **The margin depends on `v_a`.** The schedule `K ∝ 1/v_a` cancels the
   `v_a` in the plant gain, but the kite's dead time grows as `v_a` falls. With
   the old schedule, full gain was fragile below about 20 m/s and unstable
   below about 12 m/s. `v_app_min_pattern = 23` keeps α ≥ 0.55 everywhere.
2. **Depower matters less than `v_a`.** Lower depower raises `c1` (more gain),
   and the table's dead time shrinks with it; at 27 m/s α goes from 0.66 at
   depower 0.25 to 0.95 at 0.40.
3. **The entry phases are robust** thanks to `entry_gain = 0.25`, except at very
   low `v_a`, where the gravity pole competes with the reduced loop gain. That
   is a low-frequency limit, not an oscillation, and the reason the entry keeps
   the `1/v_a` boost, now down to `v_app_min = 8`.
4. **`heading_d` works mostly as gain.** With `heading_d_n = 2` the derivative
   filter corner, `N/Td` ≈ 6.7 rad/s, lies just above the crossover, so the D
   path nearly doubles |C| there and adds only about 30° of lead.
5. **The rate limit costs overshoot on large turns, not stability.**

## Caveats

- **The rate limit is nonlinear.** Small corrections pass the tape with a
  0.33 s lag, large ones lag more the larger they are. `ACTUATOR_LAG` is the
  equivalent at the amplitudes of the pattern. `step_response` covers large
  errors, but at constant `v_a`, with the linear turn-rate law and without
  gravity.
- The actuator lag comes from one run at depower 0.27 and 34 – 38 m/s. The
  dead-time exponent comes from three points at depower 0.275, spanning
  13 – 36 m/s; below 13 m/s it is extrapolated, and it is assumed to hold at
  every depower.
- The transition fix was checked in one wind speed (7 m/s) on the 200 m
  project. `simple_opt_fig8.jl` reads the same `fc_settings.yaml` when a fig8
  project is selected, and so flies with it too, unchecked.
- The relay sweeps fly at 73° elevation and 150 m tether, the pattern at 26°
  and 200 m. The fig8 point sits on the same law, so these seem not to matter
  much, but that is one point.
- The fits are closed-loop, over 35 s with few distinct frequencies. The
  time-domain fits are solid; a frequency-by-frequency comparison above 0.3 Hz
  was too noisy to use.
- The heading/course blend (`v_kite_heading`/`v_kite_course`) is not modelled.
  The loop treats the feedback angle as the turn-rate law's ψ. In the window
  the course was fed back, and its fit differs little from the heading's.
- The model contains neither the feed-forward (`u_ff`, `chi_ff`) nor the
  guidance (the attractor outer loop). Neither changes the inner-loop margins,
  but the outer loop adds its own dynamics.

## Delay vs. lag

A **delay** (dead time) shifts the signal later in time without changing its
shape. A **lag** (first-order low-pass) smears it out: the output starts
responding immediately but only follows gradually.

**Delay (dead time) τ**
- Transfer function: `e^(−sτ)`
- Step response: nothing happens for τ seconds, then the full step appears
  unchanged.
- Gain `|G| = 1` at all frequencies, so it doesn't attenuate anything.
- Phase `−ωτ`, which **grows without limit** as frequency rises.

**Lag (first-order, time constant T)**
- Transfer function: `1/(1 + sT)`
- Step response: starts moving at once and follows an exponential, reaching 63 %
  after T.
- Gain falls above `ω = 1/T`, so high frequencies are damped.
- Phase `−atan(ωT)`, which **never exceeds −90°**.

**How this affects the course loop.** At the critical frequency, about 0.8 Hz
(ω ≈ 5 rad/s), with 0.42 s of apparent delay:

| Model of the 0.42 s | Phase loss at 5 rad/s | Gain at 5 rad/s |
|---|---|---|
| Pure dead time | −ωτ = −120° | 1.0 |
| Pure lag, T = 0.42 s | −atan(2.1) = −65° | 0.43 |

So a lag costs about half as much phase and also cuts the loop gain. Both effects
move the Nyquist curve away from −1. That is why the delay model is the dominant
uncertainty in this analysis: the same 0.42 s read as dead time gives a disk
margin of 0.06, but as a lag the loop would be comfortably stable.

**Which one the kite has.** The fig8 log answers it (see
[Log validation](#log-validation-2026-09-25)): almost all of it is lag, from the
steering tape's rate limit (T ≈ 0.43 s), and in the pattern only about 0.12 s
is dead time between the applied steering and the turn rate; at lower `v_a` it
is longer. The script's plant now has the
`1/(1 + sT)` factor, with that shorter dead time.

A fit that knows only "c1 plus a delay" cannot tell the two apart. It absorbs
everything into one number, much like reading a step response by eye as
"starts responding after ≈ 0.4 s". To separate them per table cell, fit

    ψ(s)/u_s(s) = c1·v_a·e^(−sτ) / (s·(1 + sT))

to a step or chirp in `rel_steering`.

## Next steps

1. **Low-wind transition.** At 5 m/s wind the fig8 run fails (laps, min
   elevation), with or without the changes here: phase 3 lasts 20 s or more
   at `v_a` 6 – 10 m/s and the kite sinks to the ground.
2. **Other wind speeds and projects.** Re-run `simple_fig8.jl` with
   `v_app_min_pattern = 23` at other wind speeds and on the 150 m and 300 m
   projects, where phase 4 may fly below 23 m/s.
3. **Dead-time exponent at other depowers.** Fly the 0.40 cell at a second wind
   speed, to check that 1.24 holds away from 0.275.
4. **Turn rate at large steering.** Identify the turn-rate law beyond
   `|u| = 0.175` (raise `max_steering_cap`), so `step_response` can use the real
   large-signal turn rate.
5. **Reel-out.** `fc_settings_reelout.yaml` has `v_app_min_pattern` off;
   analyse its loop the same way (it has lower gains, `heading_p` 0.189).
   Started: see [course_loop_stability_reelout.md](course_loop_stability_reelout.md).

## Usage

    include("examples/stability_course_controller.jl")   # from the package root, examples env
    diskmargin(L)                                        # nominal pattern loop

Set `SHOW_PLOTS = false` before the include to skip the Bode plot and the
margin-vs-depower plot. The script needs `ControlSystemsBase` and
`RobustAndOptimalControl`, which were added to `examples/Project.toml` for it.
