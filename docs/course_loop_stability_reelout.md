# Course-loop stability during reel-out (disk margins)

2026-09-25. Findings from `examples/stability_opt_reelout.jl`, a disk-margin
analysis of the course loop flown by `simple_opt_reelout.jl` over the full
range of tether length. It extends the fig8 analysis in
[course_loop_stability.md](course_loop_stability.md); the plant model is
shared with it (`examples/course_loop_model.jl`).

**Status: the loop's margin has been measured in the simulation (see
[Measuring the margin](#measuring-the-margin-in-the-simulation-2026-09-25)). It
is 0.43 – 0.6 near 0.2 Hz, where the model predicts 0.21 – 0.5, so the model is
right in direction but pessimistic. A fix based on the model ("C", see
[Attempted fix](#attempted-fix-2026-09-25-not-adopted)) did not raise the
measured margin, made tracking worse at 5 – 10 m/s on both sites, and was not
adopted.**

- The inner course loop is robust at every tether length (α ≈ 1.0).
- The loop with the attractor guidance closed around it is fragile: α 0.14 – 0.50
  over 150 – 380 m. The margin is lowest from 150 to 270 m.
- The run the numbers come from passed all 10 success criteria, so the
  predicted margin is either pessimistic or its effect is hidden in the lap's
  own motion. Validating it is the next step.

## Model

### Inner loop (as for fig8)

- Actuator: first-order lag of 0.43 s (`ACTUATOR_LAG`), standing in for the
  rate-limited steering tape.
- Kite: the turn-rate law of `data/turn_rate_coeffs.yaml`. The kite's dead time
  scales with the apparent wind speed (`kite_delay`, exponent 1.24).
- Gravity: both signs of the gravity pole are checked. β is the pattern's
  centre elevation, read from the log (`var_04`).
- Controller: the exact discrete PD of `CourseController`, at the project's
  `1/sample_freq`.

### What differs from fig8

- **Gain schedule.** `simple_opt_reelout.jl` scales the gain by
  `gain_scale = c1(depower_setpoint)/c1(depower)` in every phase. The loop gain
  `K·c1·v_a` is therefore that of `depower_setpoint` whatever depower is flown:

      K = gain_scale · heading_p · v_app_ref / max(v_a, v_app_min, v_app_min_pattern)

  `fc_settings_reelout.yaml` has `heading_p = 0.1891`, `heading_d = 0.12 s`,
  `heading_d_n = 2`, no integral, `v_app_min = 10`, and `v_app_min_pattern` off.
- **Guidance.** The inner loop does not see the tether length, because the
  turn-rate law is physical. The guidance does. The attractor sits
  `D = attractor_distance(fcs, v_a, L)` of arc ahead of the closest point:
  `attractor_lead_time · v_a / L`, clamped to
  `[attractor_dist, 2·attractor_dist]` = [6°, 12°]. Linearized about the path:
  - A cross-track error `d` (angular) commands `δχ_set = −d/D`, the small-angle
    form of `atan(d/D)`.
  - The kite closes it at `ḋ = v_k/L · δψ`.
  - Broken at the plant input, the loop becomes

        L_g = C · (1 + ω_g/s) · P,   ω_g = v_k / (L · D)

    The guidance acts like an integral path with corner `ω_g`. While the lead
    time sets `D`, `L·D ≈ 0.8 s · v_a` and `ω_g ≈ v_k / (0.8 s · v_a)`, which
    does not depend on the length. Once `D` sits on its 6° floor (above about
    250 m here), `ω_g` falls as `1/L`.
- **Feed-forward.** The curvature feed-forward (`u_ff`, `chi_ff`) acts outside
  the loop and does not change its margins. Its fades on cross-track and course
  error are not modelled.

### Operating points

All operating points come from the last log of `simple_opt_reelout.jl` for the
selected project (`output/<log_file>_opt.arrow`):

- **Samples used:** phases 3 – 5, and only while the kite is within
  `attractor_dist` (6°) of the path. Further out the approach is not linear.
- **Bins:** 10 m of tether length, from `l_tether` to `reelout_l_max`.
- **Per sample:** `L`, `v_a`, the tangential kite speed
  `v_k = sqrt(|v|² − v_ro²)`, the depower and `ω_g`. A bin covers less than one
  lap, and `v_k/v_a` varies along the lap, so `ω_g` is computed for each sample.
- **Worst case per bin:** over `v_a` (lowest, median, highest), depower (lowest,
  highest) and both gravity signs, all at the bin's highest `ω_g`.
- **Coverage:** a bin with no samples is reported as an error, because the
  whole range must be flown before it can be checked.

## Results

Log `output/reelout_cabauw_opt.arrow`, project `system_reelout_cabauw.yaml`,
7 m/s wind, 2026-09-25 19:21, git 073c688 (dirty). dt = 1/90 s,
`body_damping = [0, 0, 40]`, `depower_setpoint = 0.274` (c1 = 0.2454). The log
covers the full range 150 – 380 m.

| L [m] | v_a [m/s] | Depower | D [°] | ω_g [rad/s] | α inner | α guided | f₀ guided | Delay margin, guided |
|---|---|---|---|---|---|---|---|---|
| 150 – 160 | 28.7 – 30.3 | 0.296 | 8.75 | 0.97 – 1.43 | 0.998 | 0.143 | 0.24 Hz | 0.096 s |
| 160 – 170 | 29.4 – 31.7 | 0.296 | 8.43 | 0.89 – 1.30 | 0.998 | 0.194 | 0.23 Hz | 0.132 s |
| 170 – 180 | 31.0 – 33.5 | 0.296 | 8.58 | 1.29 – 1.40 | 1.018 | 0.169 | 0.24 Hz | 0.113 s |
| 180 – 190 | 30.5 – 31.9 | 0.296 | 7.66 | 0.95 – 1.29 | 1.018 | 0.215 | 0.23 Hz | 0.147 s |
| 190 – 200 | 31.0 – 32.2 | 0.296 | 7.38 | 0.95 – 1.33 | 1.018 | 0.196 | 0.23 Hz | 0.133 s |
| 200 – 210 | 29.7 – 32.5 | 0.296 – 0.298 | 6.91 | 1.00 – 1.44 | 0.998 | 0.140 | 0.24 Hz | 0.093 s |
| 210 – 220 | 30.7 – 32.6 | 0.298 – 0.300 | 6.71 | 0.95 – 1.18 | 0.998 | 0.241 | 0.23 Hz | 0.168 s |
| 220 – 230 | 32.2 – 33.5 | 0.300 | 6.65 | 1.10 – 1.42 | 1.018 | 0.165 | 0.24 Hz | 0.110 s |
| 230 – 240 | 31.6 – 33.7 | 0.300 | 6.25 | 1.03 – 1.44 | 1.018 | 0.157 | 0.24 Hz | 0.105 s |
| 240 – 250 | 32.0 – 32.6 | 0.300 | 6.04 | 0.96 – 1.03 | 1.018 | 0.326 | 0.22 Hz | 0.234 s |
| 250 – 260 | 31.6 – 33.6 | 0.300 | 6.00 | 0.98 – 1.39 | 1.018 | 0.175 | 0.24 Hz | 0.117 s |
| 260 – 270 | 30.7 – 33.9 | 0.300 – 0.303 | 6.00 | 0.88 – 1.43 | 0.998 | 0.142 | 0.24 Hz | 0.095 s |
| 270 – 280 | 31.1 – 32.1 | 0.303 – 0.305 | 6.00 | 0.82 – 0.88 | 1.017 | 0.404 | 0.21 Hz | 0.299 s |
| 280 – 290 | 31.4 – 31.9 | 0.305 – 0.306 | 6.00 | 0.86 – 0.95 | 1.017 | 0.366 | 0.22 Hz | 0.267 s |
| 290 – 300 | 31.8 – 34.4 | 0.306 | 6.00 | 0.95 – 1.27 | 1.017 | 0.220 | 0.23 Hz | 0.151 s |
| 300 – 310 | 32.2 – 34.2 | 0.306 | 6.00 | 0.87 – 1.25 | 1.017 | 0.228 | 0.23 Hz | 0.157 s |
| 310 – 320 | 32.1 – 32.7 | 0.306 | 6.00 | 0.71 – 0.87 | 1.017 | 0.410 | 0.21 Hz | 0.304 s |
| 320 – 330 | 31.2 – 32.5 | 0.306 | 6.00 | 0.67 – 0.71 | 1.017 | 0.503 | 0.21 Hz | 0.385 s |
| 330 – 340 | 31.2 – 34.1 | 0.306 | 6.00 | 0.71 – 1.10 | 1.017 | 0.292 | 0.22 Hz | 0.207 s |
| 340 – 350 | 31.2 – 34.3 | 0.306 – 0.308 | 6.00 | 0.79 – 1.11 | 1.017 | 0.286 | 0.22 Hz | 0.203 s |
| 350 – 360 | 31.2 – 33.7 | 0.308 – 0.311 | 6.00 | 0.76 – 0.84 | 1.016 | 0.425 | 0.21 Hz | 0.317 s |
| 360 – 370 | 31.9 – 33.7 | 0.311 | 6.00 | 0.60 – 0.78 | 1.016 | 0.461 | 0.21 Hz | 0.348 s |
| 370 – 380 | 31.9 – 37.4 | 0.311 – 0.350 | 6.00 | 0.63 – 1.03 | 0.990 | 0.300 | 0.22 Hz | 0.216 s |

The inner loop's critical frequency is 0.28 Hz in every bin. The last bin
includes phase 5 (depower up to 0.35, 1356 samples).

## Observations

1. **The inner loop is not the problem.** With `heading_p = 0.1891` (about
   half of fig8's 0.35) and `v_a` ≥ 29 m/s on the path, α ≈ 1.0 at every
   length. The fig8 concern about low `v_a` does not arise here: on the path
   the reel-out never flies below 28.7 m/s.
2. **The guidance is nearly as fast as the course loop.** At 27 m/s,
   `K·c1·v_a = heading_p · c1 · v_app_ref ≈ 1.25 rad/s`, and the D path nearly
   doubles it near crossover, to about 1.75 rad/s. The guidance's corner `ω_g`
   is 0.6 – 1.44 rad/s. The two loops are poorly separated, and the guidance's
   integral-like factor costs `atan(ω_g/ω_c)` ≈ 35 – 40° of phase at
   crossover. That is enough to take α from 1.0 to 0.14 – 0.3.
3. **The spread between bins is `v_k/v_a` along the lap.** A 10 m bin is
   about 3 s of reel-out, less than one lap. Bins that contain the fast part
   of the lap reach `ω_g` ≈ 1.4 rad/s and α ≈ 0.14; bins without it stay near
   0.4.
4. **Longer tether helps, once `D` hits its floor.** Above about 250 m,
   `D` = 6° and `ω_g` falls as `1/L`. The best bins (α 0.4 – 0.5) are at
   310 – 370 m. From 150 to 250 m the lead time keeps `ω_g` about constant and
   the margin at its lowest.

## Attempted fix (2026-09-25): not adopted

The analysis points to two levers: slow the guidance (`attractor_lead_time`,
`attractor_dist`) and add phase lead in the PD (`heading_d`). Each candidate
was flown with `simple_opt_reelout.jl` through `FCS_OVERRIDES`, with no
turbulence. The runs are deterministic: the baseline re-flown gave the same
20 937 W to the watt. "Band" is the RMS of the regulated course error
(`var_06`) in 0.18 – 0.30 Hz, the band of the predicted resonance, in settled
phase 4. The model α is the minimum over the full tether-length range.

Cabauw 7 m/s:

| Candidate | Model α guided | Band | Course-error std | RMS d (run) | Min el. | Power | Criteria |
|---|--:|--:|--:|--:|--:|--:|---|
| baseline | 0.14 | 14.7° | 20.5° | 1.25° | 13.7° | 21 155 W | all 10 |
| `heading_d` 0.3 | 0.34 | 9.5° | 16.2° | 1.45° | 15.2° | 20 991 W | all 10 |
| lead 1.6 s, `attractor_dist` 8° | 0.46 | 11.8° | 20.0° | 0.98° | 15.2° | 21 596 W | all 10 |
| **lead 1.6 s, `attractor_dist` 8°, `heading_d` 0.3 ("C")** | **0.65** | **8.3°** | 16.4° | **0.91°** | 15.1° | 21 395 W | all 10 |
| lead 2.0 s, `attractor_dist` 10°, `heading_d` 0.3 | 0.76 | 9.0° | 20.4° | 1.03° | 14.0° | 21 637 W | all 10 |

At 7 m/s the model's ranking holds: the band shrinks as α grows, and "C"
also cuts RMS d by 27 %. The regression of "C" at other conditions did not
hold up:

| Condition | Model α, baseline → C | Band | Course-error std | RMS d (run) | Min el. | Power |
|---|--:|--:|--:|--:|--:|--:|
| Cabauw 5 m/s | 0.07 → 0.61 | 14.1 → 14.7° | 23.3 → 30.1° | 0.79 → **1.21°** | 16.2 → 15.9° | 14 068 → 14 096 W |
| Cabauw 10 m/s | 0.13 → 0.55 | 6.9 → **3.6°** | 14.6 → 15.0° | 1.19 → 1.27° | 11.3 → 11.8° | 24 643 → 24 487 W |
| Maasvlakte 7 m/s | 0.07 → 0.61 | 14.5 → 14.5° | 23.9 → 30.9° | 0.70 → **0.92°** | 8.5 → 7.2° | 12 156 → 12 218 W |
| Maasvlakte 10 m/s | 0.12 → 0.63 | 10.6 → 10.1° | 17.0 → 20.9° | 0.98 → **1.22°** | 12.0 → 12.4° | 19 944 → 20 468 W |

All runs passed all 10 criteria. "C" was **not adopted**, and
`fc_settings_reelout.yaml` is unchanged:

- **Tracking got worse.** RMS d rose in all four regression conditions, by
  up to 53 % at Cabauw 5 m/s. The minimum elevation fell by 1.3° at Maasvlakte
  7 m/s.
- **The model is not predictive at low wind.** At 5 – 7 m/s (v_a ≈ 24 – 26
  m/s) the model puts the baseline at α ≈ 0.07, close to instability. Yet the
  baseline tracks best there (RMS d 0.70 – 0.79°), and "C", with α ≈ 0.6, does
  not reduce the band at all. Only at Cabauw 7 and 10 m/s does the band follow
  α.
- **A likely reason is the chord.** A longer lead puts the attractor further
  ahead, and the chord cuts the curve by more. `chi_ff` removes only
  `ff_gain` = 70 % of the chord offset, and the remaining 30 % is a steady
  inside cut that grows with `D`. That would also explain why RMS d rises
  while the band falls. This has not been checked.
- **It matches the earlier finding.** In
  [fig8_tuning_log.md](fig8_tuning_log.md) (2026-09-21), the 0.6 – 4 s ring
  was found to be mostly the forced response to the path's own curvature
  content, not a lightly damped mode. Lead 1.2 s left it unchanged, and
  `heading_d` 0.16 did nothing. The model sees only the loop's damping, not
  what drives it.

**Status after the attempt:** the guided-loop margin as modelled is not a
reliable predictor of tracking quality. Low α is not, on its own, a defect
to fix. Before any setting is changed on its strength, the model has to
explain why the baseline tracks well at α ≈ 0.07.

## The tape's lag in the reel-out

`ACTUATOR_LAG` = 0.43 s was identified on a fig8 log, where the tape sits on
its 0.2/s rate limit 20 % of the time. The reel-out steers less hard. A
least-squares fit of `ẏ = (u − y)/T`, from `set_steering` to `steering`, on
the baseline runs gives, in phase 4:

| Run | T | Rate-limited | Unexplained |
|---|--:|--:|--:|
| Cabauw 5 m/s | 0.327 s | 1.6 % | 1.8 % |
| Cabauw 10 m/s | 0.327 s | 0.3 % | 2.8 % |
| Maasvlakte 7 m/s | 0.352 s | 4.8 % | 3.9 % |
| Maasvlakte 10 m/s | 0.326 s | 0.5 % | 1.9 % |
| Cabauw 7 m/s | 0.498 s | 5.5 % | 20 % |

- **Phase 4 at 5 and 10 m/s:** the tape is almost linear, and its lag is the
  small-signal `1/steering_gain` = 0.333 s.
- **Phase 3:** the entry turns the kite round, and the tape is rate-limited
  28 % of the time (T = 1.3 s at Cabauw 5 m/s).
- **Phase 5:** it steers harder, T = 0.45 s.
- **Cabauw 7 m/s:** the fit is poor (20 % unexplained) and needs a closer
  look.

`stability_opt_reelout.jl` now fits `T` on the log per tether-length bin,
from the same on-path samples it analyses (`fit_actuator_lag`), prints it as
a column and uses it in the plant. `LOG_DIR` points the script at an archived
run. Worst guided α over the length, with `ACTUATOR_LAG` → with the fitted
lag:

| Run | α guided, min | Median over the bins | α inner, min | Fitted lag per bin |
|---|--:|--:|--:|---|
| Cabauw 5 m/s | 0.07 → **0.19** (175 m) | 0.42 | 0.94 | 0.32 – 0.33 s |
| Cabauw 7 m/s | 0.14 → 0.00 (165 m) | 0.33 | 0.77 | 0.32 – **0.89** s |
| Cabauw 10 m/s | 0.13 → 0.25 (195 m) | 0.32 | 1.08 | 0.32 – 0.35 s |
| Maasvlakte 7 m/s | 0.07 → **0.19** (185 m) | 0.42 | 0.83 | 0.32 – 0.41 s |
| Maasvlakte 10 m/s | 0.12 → 0.24 (195 m) | 0.37 | 1.05 | 0.32 – 0.35 s |

- **The low-wind anomaly shrinks.** The model's 0.07 at 5 – 7 m/s was largely
  the fig8 lag, applied to a tape that is linear in the reel-out.
- **Cabauw 7 m/s is the exception.** One bin there has a rate-limited tape
  (0.89 s) and α = 0: this condition steers harder than the others.
  - **It is the entry, not the loop.** The two bins at 150 – 170 m cover
    t = 18.3 – 23.5 s: the handover from phase 3 to 4 and the first seconds of
    lap 1. Steering reaches 0.25 – 0.29, and the tape sits on its rate limit
    54 % and 37 % of the time. This is a large-signal transient. An
    equivalent lag no longer models the tape there, and a linear margin means
    nothing. Every later bin fits 0.32 – 0.41 s, with rate limiting of 16 %
    or less.
  - **The script now separates such bins.** Where the tape is rate-limited
    more than `MAX_RATE_LIMITED` = 20 % of the time, a bin is printed with a
    "large signal" note, left out of the rating and listed in a warning.
    Large turns are covered by `step_response` in
    `stability_course_controller.jl`.

Rated on the linear bins, the five baseline runs agree:

| Run | α inner, min | α guided, min (at L) | α guided, median | Large-signal bins |
|---|--:|--:|--:|---|
| Cabauw 5 m/s | 0.94 | 0.19 (175 m) | 0.42 | – |
| Cabauw 7 m/s | 1.03 | 0.24 (205 m) | 0.34 | 156, 165 m |
| Cabauw 10 m/s | 1.08 | 0.25 (195 m) | 0.32 | – |
| Maasvlakte 7 m/s | 0.83 | 0.19 (185 m) | 0.42 | – |
| Maasvlakte 10 m/s | 1.05 | 0.24 (195 m) | 0.37 | – |

- **The model's worst point is the same everywhere:** 0.19 – 0.25 at
  175 – 205 m. There the lead time sets `D` and `ω_g` is at its highest
  (1.0 – 1.4 rad/s).
- **The low-wind anomaly is gone.** Once the tape's lag is fitted, low wind is
  no worse than high wind.
- **The model is still pessimistic.** Even with the fitted lag, it stays below
  the 0.43 – 0.6 measured below, which was itself taken with a partly
  saturated tape.

## The guidance term and the kite's dead time

**The guidance law is as modelled.** The flown path of a run was probed with
`navigate_fig8`: the kite was offset ±0.3° normal to the path at each of the
360 points, and `dχ_set/dd · D` was read off. It comes out at 0.95
(10 – 90 %: 0.93 – 0.98) at D = 6°, and 0.92 (0.87 – 0.95) at D = 8.4°. The
pure-pursuit gain `1/D` is right to within 5 – 8 %.

**The kite's dead time is not.** `identify_turn_rate_law` on settled phase 4
of the baseline runs:

| Run | Depower | v_a | c1 fitted / table | Dead time fitted | `kite_delay` |
|---|--:|--:|--:|--:|--:|
| Cabauw 5 m/s | 0.266 | 25.6 m/s | 0.2486 / 0.2530 | **0.067 s** | 0.186 s |
| Cabauw 7 m/s | 0.306 | 32.1 m/s | 0.2146 / 0.2110 | 0.156 s | 0.154 s |
| Cabauw 10 m/s | 0.360 | 40.0 m/s | 0.1514 / 0.1565 | 0.100 s | 0.137 s |
| Maasvlakte 7 m/s | 0.265 | 23.8 m/s | 0.2468 / 0.2545 | **0.067 s** | 0.204 s |
| Maasvlakte 10 m/s | 0.294 | 30.8 m/s | 0.2249 / 0.2239 | 0.156 s | 0.157 s |

- **`c1` is confirmed** to within 3 %, and every fit has a correlation of
  0.99 or more.
- **At low wind the dead time is a third of the extrapolation.** The
  `v_a^-1.24` law was fitted at depower 0.275, over 13 – 36 m/s. At
  24 – 26 m/s in the reel-out it overstates the dead time by 0.12 – 0.14 s.
  Across the runs it follows depower more than `v_a`.
- **The script now identifies the dead time on the log** (settled phase 4)
  and scales it over `v_a` within the run with the same exponent. Together
  with the fitted tape lag, the model now takes both of its time constants
  from the log it analyses.

Guided α with both time constants from the log (linear bins):

| Run | α inner, min | α guided, min (at L) | α guided, median |
|---|--:|--:|--:|
| Cabauw 5 m/s | 1.27 | **0.41** (185 m) | 0.60 |
| Cabauw 7 m/s | 1.01 | 0.24 (205 m) | 0.33 |
| Cabauw 10 m/s | 1.17 | 0.32 (305 m) | 0.38 |
| Maasvlakte 7 m/s | 1.17 | **0.42** (185 m) | 0.63 |
| Maasvlakte 10 m/s | 1.06 | 0.26 (175 m) | 0.38 |

- **The low-wind pessimism is gone.** At 5 – 7 m/s the model gives 0.41 –
  0.42, the same range as the 0.43 measured in the simulation.
- **The weakest conditions are now Cabauw 7 and Maasvlakte 10 m/s**
  (0.24 – 0.26). The optimizer flies them at depower 0.29 – 0.31, and the kite's
  dead time is 0.156 s there, more than twice the low-wind value.
- **Those two conditions have not been measured in the simulation yet.** The
  measurement above was at 5.3 m/s.

## Feed-forward gain 1.0 (2026-09-25)

`ff_gain` 0.7 → 1.0 through `FCS_OVERRIDES`, against the archived baselines
(the runs are deterministic):

| Condition | Criteria | RMS d | Course-error std | Band | Min el. | Power | Rate-limited |
|---|---|--:|--:|--:|--:|--:|--:|
| Cabauw 5 m/s | pass → pass | 0.79 → **0.62°** | 23.3 → 13.1° | 14.1 → 11.1° | 16.2 → 16.4° | 14 068 → 13 969 W | 3 → 7 % |
| Cabauw 7 m/s | pass → **FAIL** | 1.25 → 1.42° | 20.5 → 21.4° | 14.7 → 15.0° | 13.7 → 14.3° | 21 155 → 20 463 W | 4 → 9 % |
| Cabauw 10 m/s | pass → pass | 1.19 → 1.11° | 14.6 → 12.5° | 6.9 → 8.9° | 11.3 → 12.6° | 24 643 → 23 529 W | 0 → 2 % |
| Maasvlakte 7 m/s | pass → pass | 0.70 → **0.61°** | 23.9 → 11.9° | 14.5 → 10.2° | 8.5 → 8.3° | 12 156 → 12 124 W | 6 → 9 % |
| Maasvlakte 10 m/s | pass → pass | 0.98 → 1.07° | 17.0 → 16.6° | 10.6 → 13.2° | 12.0 → 10.9° | 19 944 → 20 079 W | 1 → 7 % |

- **Low wind gains.** At 5 – 7 m/s at low depower, RMS d falls by 13 – 22 %
  and the course-error std halves.
- **At depower ~0.3 it does not help.** Cabauw 7 m/s fails "heading range
  < 400°" (415°): the lobe turns overshoot, and the unwrapped heading runs
  from −22° to 363°. Its max d rises from 5.11 to 6.18°, with 6.5° spikes at
  the crossings at t = 22 and 36 s. Maasvlakte 10 m/s loses 0.09° of RMS d and
  1.1° of minimum elevation.
- **This is the overturn the tuning log recorded at 1.0 on 2026-09-21**, and
  in the same place: the first laps at short tether.
- **The tape works harder everywhere,** with rate-limiting up by 2 – 6 points.
- **Not adopted.** `ff_gain` stays at 0.7. The conditions where 1.0 fails
  are the ones with the longest dead time and the weakest guided margin.

## Measuring the margin in the simulation (2026-09-25)

To test the model directly, `simple_opt_reelout.jl` now accepts a test input.
`STEER_DISTURBANCE` is a function `t -> Δu`, read and cleared like
`SHOW_PLOTS`, and added to `rel_steering` after the controller. When it is
set, the run keeps the globals `dist_t`, `dist_d` (the disturbance) and
`dist_u` (the steering sent to the model).

With `u = d + u_c`, the loop broken at the plant input is `L = −U_c/U`. This
is measured from the simulation without any model. Its disk margin
`2/|(1−L)/(1+L)|` can then be compared with the model's at each frequency.

**Setup.** For a long stationary window, the run holds its length for 150 s
in phase 5 (`final_time = 150`, `sim_time = 260`). The wind is the project's
default of 5.3 m/s: with a wind-speed override the script sets its own run
length and ignores `sim_time`. The length is held at 380 m, or at 200 m with
`reelout_l_max = 200`.

**Random disturbance: loses the key band.** A random binary signal (±0.02 or
±0.05, held 0.25 or 0.8 s) measured the loop well above 0.35 Hz. From 0.4 to
0.75 Hz, `|S_i|` matches the model to within 5 – 15 %, which validates the
inner loop (plant, lag, dead time, PD). Below 0.3 Hz, the steering the path
itself demands, including `u_ff` of about ±0.2, drowns the disturbance
(coherence < 0.4). That is exactly where the guided model has its worst point.

**Sine disturbance: gets it.** The sum of four sines
`0.03·Σ sin(2π f t + k)`, at `f` = 0.135, 0.175, 0.215 and 0.26 Hz (between
the lap harmonics). `L` is evaluated at each sine frequency over the whole
window, and again over each half to check consistency. Disk margin at the
worst measured frequency:

| Case | Worst f | L measured | α measured (halves) | α model guided (at f / overall) | α model inner |
|---|--:|---|---|---|--:|
| 380 m, depower 0.35, v_a 28 m/s | 0.175 Hz | 0.62 ∠ −160° | ≈ 0.6 | 0.45 / 0.50 | 0.94 |
| 200 m, depower 0.35, v_a 28 m/s | 0.215 Hz | 0.65 ∠ −178° | 0.43 (0.49, 0.37) | 0.22 / 0.21 | 0.93 |
| 200 m, candidate "C" | 0.215 Hz | 0.78 ∠ −158° | 0.47 (0.53, 0.37) | 0.72 / 0.72 | 1.01 |

The 380 m row is from the first sine run. Its halves disagree at 0.135 Hz, so
only the 0.175 – 0.26 Hz points are used.

What this shows:

1. **The low-frequency problem is real.** Near 0.2 Hz the measured phase of
   `L` is −160 to −180°, as the guided model predicts and the inner-only model
   (−120 to −130°) does not. The measured margin there is 0.43 – 0.6, below
   the inner loop's ≈ 0.95.
2. **The guided model is pessimistic by about a factor of 2.** At 200 m it
   predicts α = 0.21, and 0.43 is measured. The measured `|L|` falls faster
   between 0.135 and 0.26 Hz than the model's.
3. **"C" does not move the measured margin.** The model predicts 0.21 → 0.72,
   and 0.43 → 0.47 is measured, within the scatter between the halves. The
   worst point stays near 0.2 Hz at about −160°. Whatever sets the margin there
   is not what `attractor_lead_time`, `attractor_dist` and `heading_d` change
   in the model. That also explains the regression above: "C" bought no
   measured robustness and cost tracking.
4. **The loop is time-varying.** v_a, the gravity term and the path curvature
   all change around the lap, and the halves of one window scatter by
   ±0.08 in α. A linear time-invariant model can rank settings only roughly.

**Correction: the disturbance saturated the tape.** A fit of the tape over
those windows (see [The tape's lag](#the-tapes-lag-in-the-reel-out)) shows the
four sines at 0.03 each (peak 0.12) put it on its rate limit 19 – 24 % of the
time, with an equivalent lag of 0.48 – 0.69 s. In undisturbed flight it is
0.33 s. The margins in the table therefore describe a harder-driven loop than
the one normally flown. A repeat at 0.012 per sine (200 m, lag 0.37 s,
rate-limited 9 %, the path's own steering) was too noisy to use: the two
halves gave α = 0.29 and 0.17 at 0.215 Hz, and the whole window 0.76. With a
large disturbance the tape saturates, with a small one the path's own
steering drowns it, so this method has reached its limit on this
time-varying loop.

**Status:** the measured worst-case margin of the reel-out course loop is
0.43 – 0.6 near 0.2 Hz (5.3 m/s, 200 – 380 m), measured with a partly
saturated tape. That is marginal but not fragile, and all regression runs pass
their criteria. No setting tested improves it. The next step is to find what sets the loop near 0.2 Hz in the
simulation and is missing from the model, before tuning again. Candidates:
- the course response to heading (side slip), since the fed-back course is
  not the heading of the turn-rate law;
- the tether and roll dynamics;
- the variation over the lap.

## Wind speed and site (2026-09-25, evening)

The script run on six logs: Cabauw and Maasvlakte at 4, 7 and 10 m/s, all
flown with `pattern_symmetric: true` (see [symmetry.md](symmetry.md)), git
1c74833, uncompressed logs from `output/archives`.

| | Cabauw 4 | Cabauw 7 | Cabauw 10 | Maasvlakte 4 | Maasvlakte 7 | Maasvlakte 10 |
|---|---|---|---|---|---|---|
| Archive `2026-09-25_…` | `222205` | `215215` | `215950` | `223648` | `224252` | `224638` |
| v_a on the path [m/s] | 17 – 23 | 29 – 33 | 35 – 40 | 10 – 17 | 22 – 24 | 28 – 31 |
| Identified kite dead time [s] | 0.056 | 0.100 | 0.067 | 0.000 | 0.067 | 0.156 |
| α inner, minimum | 1.20 | 1.06 | 1.25 | 1.20 | 1.11 | 1.05 |
| **α guided, minimum** | 0.44 | 0.36 | 0.37 | **0.58** | 0.41 | **0.23** |
| … at L [m] | 165 | 195 | 195 | 155 | 185 | 195 |
| α guided, mean 150 – 230 m | 0.56 | 0.39 | 0.44 | 0.69 | 0.47 | 0.31 |
| α guided, mean 300 – 380 m | 0.78 | 0.51 | 0.51 | 0.92 | 0.69 | 0.50 |
| ω_g, mean 150 – 230 m → 300 – 380 m [1/s] | 1.09 → 0.65 | 1.34 → 1.00 | 1.34 → 1.18 | 0.93 → 0.46 | 1.30 → 0.73 | 1.33 → 0.90 |
| Rating of the script | warning | warning | warning | ok | warning | error |

All six runs passed all 10 success criteria. The non-symmetric Cabauw 4 m/s
run (22:24, `scenarios/cabauw/v04` before compression) gave α = 0.42 against
0.44 for the symmetric one, so the path symmetry does not change the picture.

1. **The margin falls with wind speed at both sites.** Only Maasvlakte 4 m/s
   is rated robust. Maasvlakte 10 m/s is in the error band (α < 0.3), with a
   mean of 0.31 over the first 80 m of reel-out. It is the same run in which
   the measured power is only 0.83 of the prediction and the lobes fly 6°
   below the centre crossing.
2. **The cause is the guidance's corner `ω_g`, as in the 7 m/s analysis
   above.** At 4 m/s, `D = 0.8 s · v_a / L` is about 6° already at 150 m, so it
   sits on the floor from the start, `ω_g` falls as `1/L`, and α recovers to
   0.8 – 0.9 by the end of the reel-out. At 7 and 10 m/s the faster kite keeps
   `D` at 9 – 11° at 150 m, the floor is only reached at 250 – 300 m, and
   `ω_g` stays near 1.3 rad/s for most of the reel-out.
3. **The inner loop stays robust** (α ≥ 1.05) at every wind speed and site.
4. **The identified dead time scatters** from 0.000 s (Maasvlakte 4 m/s,
   v_a 10 – 17 m/s) to 0.156 s (Maasvlakte 10 m/s, fit correlation 0.99). The
   large value is probably real and is what pulls Maasvlakte 10 m/s down; the
   zero at the lowest airspeed is suspicious and makes that 0.58 optimistic.

## Cross-track step test (2026-09-26)

Next step 1 below, dynamic part. The static guidance gain was already confirmed
(`dχ_set/dd · D` = 0.92 – 0.95, see
[The guidance term](#the-guidance-term-and-the-kites-dead-time)), so this tests
only the ringing of d.

**Test input.** `XTRACK_OFFSET = τ -> δ` [deg] in `simple_opt_reelout.jl`, τ
the time since phase 5 began, read and cleared like `SHOW_PLOTS`. The attractor
is moved δ along the path's right-hand normal, so the pursuit aims at the
parallel curve δ to the right: a reference step for the guided loop alone. The
run keeps `xt_t`, `xt_delta` and `xt_d`, the signed cross-track error to the
unshifted path (right of travel > 0), the closest-point index `xt_q`, and the
operating point `xt_phase`, `xt_L`, `xt_va`, `xt_vk`, `xt_dp`. `XTRACK_PHASE`
(default 5) picks the phase τ counts from; `TOS_OVERRIDES`, like
`FCS_OVERRIDES`, overrides `TrajOptSettings` fields, e.g. `reopt_enabled`.

**Setup.** Cabauw at the default wind (5.324 m/s, no override so `sim_time`
holds), `sim_time` 260 s, `FCS_OVERRIDES` `final_time = 150`,
`reelout_l_max = 200`; δ a ±1° square wave with 11 s half-periods from
τ = 10 s (12 steps). Held window: 200 m, v_a 27.8 m/s, v_k 24.5 m/s, depower
0.35, D = 6.4°, tape lag 0.39 s and rate-limited 9 % of the time, kite dead
time 0.056 s identified on the held window (correlation 0.997). Lap period
13.8 s.

**The lap forcing hides the step.** With δ = 0 the signed d has a standard
deviation of 3.1° in the held window. So the test is flown twice, with the
offset and with `XTRACK_OFFSET = τ -> 0` (the runs are deterministic: the two
are identical to the sample before the first step), and the difference
Δd = d − d_ref is analysed. Data: `output/xtrack_step_test_200m.csv` (step run
archive `2026-09-25_235333`, twin `2026-09-25_235723`); analysis:
`examples/xtrack_step_analysis.jl`.

| τ after the step [s] | 0.5 | 1.0 | 1.5 | 2.0 | 2.5 | 3.0 | 4.0 | 5.0 | 6.0 | 7.0 | 8.0 | 9.0 | 10.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Measured Δd/Δδ, mean of 12 | 0.28 | 0.51 | 0.76 | **0.88** | 0.78 | 0.59 | 0.29 | 0.31 | **0.44** | 0.30 | 0.22 | 0.38 | **0.65** |
| Model T (both gravity signs) | 0.06 | 0.32 | 0.69 | 1.03 | 1.23 | **1.27** | 1.08 | 0.91 | 0.94 | 1.02 | 1.02 | 1.00 | 0.99 |

The measured mean has a standard error of 0.1 – 0.4. The model's dominant
poles: 0.21 Hz, ζ = 0.29 – 0.32.

1. **The first response is faster than modelled:** half the step at 1.0 s
   (model 1.3 s), first peak at 2.0 s (model 2.9 s).
2. **d does not settle at δ.** After the peak it falls back to about 0.38 δ
   on average over 3 – 10 s; the model settles at 1. Hypothesis, untested: the
   curvature feed-forward (`ff_gain` 0.7) is computed for the unshifted path.
   In a turn an offset to the inside needs more turn rate and one to the
   outside less, and in both lobes the feed-forward pulls the kite back to the
   original path, while the PD has no integral action.
3. **Frequency and damping are not determinable yet.** The mean ripples with
   peaks at 2, 6 and 10 s (0.25 Hz, near the model's 0.21 Hz) and barely
   decays, but a turn or crossing passes every 3.4 s (0.29 Hz), and 12 steps
   cannot separate the two. The per-step fits scatter from 0 to 0.29 Hz.

**Feed-forward off (2026-09-26).** The same twin pair with `ff_gain = 1e-6`
(the script asserts `ff_gain > 0`, since phase 4 needs it; 1e-6 removes both
the steering feed-forward and the chord correction). Both runs pass all 10
criteria. Archives `2026-09-26_003225` (step) and `003345` (twin), data
`output/xtrack_step_test_200m_ff0.csv`.

| τ after the step [s] | 0.5 | 1.0 | 1.5 | 2.0 | 2.5 | 3.0 | 4.0 | 5.0 | 6.0 | 7.0 | 8.0 | 9.0 | 10.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Measured, `ff_gain` 0.7 | 0.28 | 0.51 | 0.76 | 0.88 | 0.78 | 0.59 | 0.29 | 0.31 | 0.44 | 0.30 | 0.22 | 0.38 | 0.65 |
| Measured, feed-forward off | 0.05 | 0.15 | 0.37 | 0.59 | 0.74 | **0.75** | 0.43 | 0.37 | 0.62 | 0.85 | 0.87 | 0.89 | 1.02 |
| Model T | 0.06 | 0.32 | 0.69 | 1.03 | 1.23 | **1.27** | 1.08 | 0.91 | 0.94 | 1.02 | 1.02 | 1.00 | 0.99 |

- The mean over 3 – 10 s rises from 0.36 to 0.70, and d reaches 0.9 – 1.0 δ
  after 7 s. That supports hypothesis 2, but the standard error is 0.5 – 1.0
  here (0.2 – 0.4 with the feed-forward), so it is not conclusive.
- The first peak now comes at 3.0 s, as in the model, but at 0.75 instead of
  1.27, and it is followed by a dip to 0.35 at 4.5 s that the model does not
  have. Without the feed-forward the initial rise is slower than modelled.

**600 s hold (2026-09-26): the twin method only holds for about 100 s.** Both
twin pairs flown again with `final_time = 600` (`sim_time` 660 s), 53 steps
each, all four runs passing all 10 criteria; data
`output/xtrack_step_test_200m_600s_ff07.csv` and `..._ff0.csv`. The standard
error did not shrink: 0.3 – 0.5 with the feed-forward, 0.5 – 0.7 without,
because the twins drift apart. The offset changes the kite's timing along the
path, and the lap forcing stops cancelling:

| Window of the hold | 0–100 s | 100–200 s | 200–300 s | 300–400 s | 400–500 s | 500–600 s |
|---|---|---|---|---|---|---|
| Std of d_step − d_ref, `ff_gain` 0.7 | 1.0° | 2.3° | 3.4° | 4.2° | 4.4° | 4.6° |
| Std of d_step − d_ref, feed-forward off | 2.0° | 4.0° | 5.0° | 5.7° | 5.0° | 4.0° |

From about 200 s on the difference is that of two unrelated lap motions of 3°
each. The averages over all 53 steps therefore say little more than the
150 s tests (mean over 3 – 10 s: 0.27 with the feed-forward, 0.74 without;
first peak 0.69 at 2.2 s and 0.97 at 3.1 s).

**Subtracting by path position (2026-09-26): the test works.** The same two
600 s pairs, flown again with the closest-point index Q recorded (`xt_q`);
data in `data/steptest/xtrack_step_test_200m_600s_ff07.csv` and `..._ff0.csv`
(rounded to 1e-4; the step run and twin archives are named in their headers). With the length
held and the path fixed (last install at 32 – 33.5 s, before phase 5 at
36.5 s), the reference run is periodic in Q to within 0.03 – 0.04°.
`subtract_by_position` removes its mean d at each Q from the step run. The
difference then stays at 0.83 – 0.89° in every 100 s window (by time it grew
to 4 – 6°), and all 53 steps are usable. All four runs pass all 10 criteria.

| τ after the step [s] | 0.5 | 1.0 | 1.5 | 2.0 | 2.5 | 3.0 | 4.0 | 5.0 | 5.5 | 6.0 | 7.0 | 8.0 | 10.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Measured, `ff_gain` 0.7 (53 steps) | 0.01 | 0.12 | 0.33 | 0.56 | 0.70 | 0.69 | 0.46 | 0.25 | **0.24** | 0.27 | 0.38 | 0.47 | 0.44 |
| Measured, feed-forward off (53 steps) | −0.02 | 0.10 | 0.35 | 0.63 | 0.81 | **0.89** | 0.74 | 0.52 | **0.50** | 0.55 | 0.71 | **0.78** | 0.66 |
| Model T | 0.06 | 0.32 | 0.69 | 1.03 | 1.23 | **1.27** | 1.08 | 0.91 | 0.91 | 0.94 | 1.02 | 1.02 | 0.99 |

Standard error 0.01 – 0.11. Fitted with a delayed second-order step response
with a free gain K (rms 0.03 – 0.04, the two halves of the steps agree), and
the model's own response fitted the same way:

| | K | Ringing | ζ | Delay |
|---|---|---|---|---|
| Model | 1.00 | 0.20 Hz | 0.34 – 0.37 | 0.35 s |
| Measured, feed-forward off | **0.66** | **0.208 Hz** | 0.25 (0.22 / 0.28) | 0.6 s |
| Measured, `ff_gain` 0.7 | **0.40** | 0.183 Hz | **0.14** (0.13 / 0.15) | 0.15 s |

1. **The model's ringing frequency is right.** Without the feed-forward the
   cross-track mode rings at 0.208 Hz against 0.20 Hz modelled. With the
   feed-forward it is 0.183 Hz.
2. **The damping is lower than modelled:** 0.25 against 0.35, and only 0.14
   as flown, with the feed-forward. The feed-forward speeds up the first
   response (delay 0.15 s instead of 0.6 s) but takes damping out of the
   cross-track mode. This is the lightly damped ring near 0.2 Hz that the
   margin analysis pointed to.
3. **The steady gain is well below 1:** 0.66 without the feed-forward, 0.40
   with it. The model has a straight path and no feed-forward, so it cannot
   show this. Hypothesis, untested: in the turns, flying δ inside or outside a
   lobe changes the turn rate needed. The PD, having no integral action, needs
   a course error for that turn rate, and the pursuit geometry turns that error
   into a pull back toward the original path, on both lobes. The feed-forward,
   computed for the unshifted path, adds to the pull. On the straight parts the
   kite should reach δ; the gain would then vary along the lap.
4. **The model's first response is faster than measured without the
   feed-forward** (delay 0.35 s against 0.6 s).

**During the reel-out (2026-09-26): the model fits much better.** The held
length differs from the reel-out in the way that matters here: reel-out speed
≈ 0 instead of 2.7 m/s, tether force ≈ 4.0 instead of 5.3 kN, depower 0.35
instead of 0.27, and the tape rate-limited 9 % of the time instead of 0 %. So
the test was repeated in phase 4 (`XTRACK_PHASE = 4`): Cabauw at the default
wind, reel-out 150 → 380 m in 75 s, `ff_gain` 0.7 as flown, re-optimization
off (`TOS_OVERRIDES`, so every run flies the same path). One δ = 0 run and six
step runs, the ±1° square wave started 10 – 21.5 s into phase 4 in steps of
2.3 s (a sixth of a lap). Subtracted by time, which holds through the reel-out:
the runs are identical before their first step, and the difference stays at
0.8 – 1.0° (the response itself) without growing. All seven runs pass all 10
criteria. 29 steps lie entirely in phase 4, at 182 – 347 m, v_a 26 – 28 m/s,
depower 0.27. Data: `data/steptest/xtrack_step_test_phase4.csv` (reference
archive `2026-09-26_074504`); `phase4_step_responses` and `model_step_average`
in the analysis script. The model is evaluated at each step's own operating
point, with the dead time (0.022 s) and tape lag (0.33 s) identified on phase 4
of the reference run, and averaged like the measurement.

| τ after the step [s] | 0.5 | 1.0 | 1.5 | 2.0 | 2.5 | 3.0 | 4.0 | 5.0 | 6.0 | 8.0 | 10.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Measured, reel-out (29 steps) | 0.04 | 0.19 | 0.43 | 0.67 | 0.81 | **0.85** | 0.77 | 0.75 | 0.73 | 0.75 | 0.80 |
| Model, same 29 steps | 0.07 | 0.31 | 0.62 | 0.90 | 1.08 | **1.15** | 1.09 | 1.00 | 0.97 | 1.00 | 1.00 |

Standard error 0.01 – 0.08.

| | K | Ringing | ζ | Delay |
|---|---|---|---|---|
| **Reel-out, measured** | **0.75** | 0.191 Hz | **0.56** | 0.45 s |
| **Reel-out, model** | 1.00 | 0.171 Hz | **0.52** | 0.25 s |
| Held at 200 m, measured (`ff_gain` 0.7) | 0.40 | 0.183 Hz | 0.14 | 0.15 s |
| Held at 200 m, model | 1.00 | 0.20 Hz | 0.34 – 0.37 | 0.35 s |

1. **During the reel-out the model's damping and frequency hold:** ζ 0.56
   against 0.52, 0.19 against 0.17 Hz. The model's own poles move from 0.20 Hz,
   ζ 0.37 at 180 m to 0.15 Hz, ζ 0.59 at 350 m.
2. **The lightly damped response belongs to the held length.** There ζ is
   0.14, a quarter of the reel-out's. Force, reel-out speed and depower all
   differ between the two, and so does the tape: rate-limited 9 % of the time at
   the held length and never in the reel-out. A rate limit takes phase out of
   the loop, which fits the lost damping; this has not been separated from the
   other three.
3. **The steady gain is below 1 in both, but less so in the reel-out:** 0.75
   against 0.40. The model cannot show this (hypothesis 3 above).
4. **The measured response is 0.2 s slower** than the model's in both cases.

**What makes the held length lightly damped? Not the tape, depower or force
(2026-09-26).** The held test (200 m, `ff_gain` 0.7) flown as a twin pair with
one condition changed at a time, 300 s holds (`final_time = 300`, `sim_time`
360 s), 26 steps each, subtracted by path position. All four runs pass all 10
criteria. Data: `data/steptest/xtrack_step_test_200m_300s_{tape08,dp027}.csv`.
The fast tape is set through a new hook, `SET_OVERRIDES`, which changes fields
of the model's `Settings` after `init`; the KCU reads `v_steering` from them
every step. It does not persist: the next run's `init` builds fresh settings.

| Held at 200 m | Tape rate-limited | Depower | Force | v_a | K | Ringing | ζ |
|---|---|---|---|---|---|---|---|
| Baseline (600 s, 53 steps) | 10.5 % | 0.35 | 3.96 kN | 27.8 m/s | 0.40 | 0.188 Hz | **0.14** |
| Tape 0.8/s instead of 0.2/s | **0 %** | 0.35 | 3.97 kN | 27.8 m/s | 0.45 | 0.192 Hz | **0.16** |
| `depower_final` 0.27 | 6.5 % | 0.30 | **7.48 kN** | 34.2 m/s | 0.33 | 0.197 Hz | **0.16** (fit rms 0.12) |
| Reel-out, phase 4 (above) | 0 % | 0.27 | 5.3 kN | 26 – 28 m/s | 0.75 | 0.191 Hz | **0.56** |

(Rate-limited: share of phase-5 samples with the tape at 97.5 % of its own
rate limit. With `depower_final` 0.27 the force rose to the phase-5 limiter's
7500 N target, and the limiter held depower at 0.30.)

1. **The tape rate limit is not the cause:** with the tape never saturating,
   ζ goes from 0.14 to 0.16.
2. **Nor are depower or force:** at depower 0.30 and 7.5 kN, more force than
   in the reel-out, ζ is still 0.16.
3. **The winch does not couple in:** it holds the length rigidly (reel-out
   speed std 0.001 m/s), and the force oscillates only at lap harmonics (0.144
   and 0.288 Hz for a 13.8 s lap, about ±100 N), not near 0.19 Hz.
4. **The controller is the same in phases 4 and 5** apart from the depower
   target and the force limiter, both covered by 2.
5. What remains is the reel-out itself: reel-out speed and the growing length.
   The geometric effect of a growing tether on the angular cross-track error,
   a decay at v_ro/L ≈ 0.01 1/s, is far too small against a mode at about
   1.2 rad/s. No mechanism is identified yet.

**Reel-out speed (2026-09-26): the damping barely depends on it.** The
reel-out step test repeated at two more reel-out speeds, with the winch
changed through a new hook, `WC_OVERRIDES`. It applies just before the
simulation loop, after the startup solve, so the optimizer plans the same
path as the baseline and only the flown winch differs. (A first attempt
applied it before the solve: AWETrim returned 422 for a winch capped at
1.9 m/s.) When `kv` changes, the upper force controller's switching speed,
derived from `kv` when the controller is built, is refreshed. Slow: `v_sat`
1.9 m/s (one δ = 0 run and four step runs; all five fail only "max force ≤
8400 N", peaking at 8.6 – 8.7 kN, because the capped winch cannot pay out:
a diagnostic, not an operating point). Fast: `kv` × 1.4 (all five pass all 10
criteria). Data: `data/steptest/xtrack_step_test_phase4_{slow,fast}.csv`.

| Reel-out speed | Force | v_a | Steps | K | Ringing | **ζ measured** | ζ model | Ringing model |
|---|---|---|---|---|---|---|---|---|
| 0 (held, 200 m) | 4.0 kN | 27.8 m/s | 53 | 0.40 | 0.188 Hz | **0.14** | 0.35 | 0.20 Hz |
| 1.9 m/s | 7.9 kN | 32.2 m/s | 37 | 0.75 | 0.208 Hz | **0.50** | 0.44 | 0.189 Hz |
| 2.7 m/s (baseline) | 5.3 kN | 26 – 28 m/s | 29 | 0.75 | 0.191 Hz | **0.56** | 0.52 | 0.171 Hz |
| 3.5 m/s | 5.0 kN | 25.4 m/s | 18 | 0.75 | 0.22 Hz | **0.62** | 0.55 | 0.159 Hz |

1. **During the reel-out the damping rises only mildly with speed** (0.50 →
   0.62), and the model follows it within 0.04 – 0.07 through v_a and v_k. It
   underestimates the frequency at 3.5 m/s.
2. **The steady gain is 0.75 at every reel-out speed,** against 0.40 held.
3. **So the held length is the outlier, not the slow end of a trend.** With
   tape, depower, force and reel-out speed ruled out, the remaining difference
   is how the winch works. Reeling out, it is speed-controlled: a change in
   force changes the reel-out speed, so the tether is compliant and the winch
   takes energy out of the kite's motion. Held, the winch keeps the length
   rigidly.

**Next:**
- Test a compliant winch at the held length. Not a simple switch here:
  `fc_settings_reelout.yaml` requires `compliance = 0` (position mode) because
  the reel-out controller and V3Kite's force mode cannot both drive the drum.
  Options: hold the length through the speed controller with a soft position
  loop, or fly phase 5 in V3Kite's force mode.
- If that restores the damping, the model needs the winch's force-speed
  coupling, and phase 5 (and parking at a fixed length) may deserve a
  compliant winch.
- Check hypothesis 3 (steady gain) by splitting the steps into turn and
  straight (from Q).

## Caveats

- **The guidance model is unvalidated.** It is the small-angle pure-pursuit
  law on a straight path. The real path is curved, the closest point `Q` moves
  with the kite, and the curvature feed-forward and its chord correction change
  the course the PD sees.
- **The log does not show the resonance clearly.** A loop with α ≈ 0.15
  should ring at 0.21 – 0.24 Hz. In phase 4 (from 10 s after its start), the
  regulated error `var_06` has about 95 % of its power below 0.3 Hz. Its
  largest components are at 0.04 Hz and at 0.12 – 0.26 Hz, which overlaps the
  lap harmonics. The check was inconclusive.
- The worst case per bin is deliberately conservative: the bin's highest `ω_g`
  is combined with every `v_a` and depower corner.
- The actuator lag and the dead-time exponent were identified on fig8 data at
  depower 0.27 – 0.275. The reel-out flies 0.296 – 0.35.
- **Compressed logs** (every folder under `output/scenarios/`, see
  `docs/log_size.md`) keep every 3rd row. Until 2026-09-25 the dead-time
  identification assumed the simulation's sample time of 1/90 s and so gave a
  third of the dead time on them: 0.022 s instead of 0.067 s, α = 0.51 instead
  of 0.42 on the Cabauw 4 m/s run of 22:24. It now uses the log's own sample
  time and gives 0.067 s and 0.42 on both. The dead time is then resolved to
  1/30 s instead of 1/90 s.

## Next steps

1. **Validate the guidance model** (dynamic part done at 200 m, see
   [Cross-track step test](#cross-track-step-test-2026-09-26)). Excite or isolate the cross-track loop in
   `simple_opt_reelout.jl`, for example with a step in `el_offset` or a short
   course disturbance in phase 4 at a fixed length. Then compare the ringing of
   `d` with the predicted 0.21 – 0.24 Hz and damping. Alternatively, fit
   `δχ_set` against the signed `d` in the log to check `1/D`; the path tangent
   would need to be logged for that.
2. **If it holds, slow the guidance.** Raising `attractor_lead_time` (0.8 s)
   or `attractor_dist` (6°) lowers `ω_g`. Check α and tracking
   (the size and elevation criteria) together, since a longer lead cuts
   corners.
3. **Maasvlakte and other wind speeds:** done at 4, 7 and 10 m/s, see
   [Wind speed and site](#wind-speed-and-site-2026-09-25-evening). Slowing the
   guidance (step 2) matters most at 7 – 10 m/s and at Maasvlakte 10 m/s.
4. **Fix the script for compressed logs:** done, it takes the sample time
   from the log's own time column.
5. **Add the guidance to the fig8 analysis.** `stability_course_controller.jl`
   models the inner loop only. The same guidance factor applies there too.

## Usage

    include("examples/stability_opt_reelout.jl")   # from the package root, examples env
    diskmargin(L)                                  # the worst guided loop

The script is also in the example menu (`menu.jl`). It needs a finished
`simple_opt_reelout.jl` run for the selected project (cabauw or maasvlakte).
Set `SHOW_PLOTS = false` before the include to skip the Bode plot of the worst
guided loop and the plot of the margins over tether length.
