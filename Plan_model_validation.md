# Plan: validating the course-loop plant model

2026-09-27. A plan to check the linear course-loop model of
`examples/course_loop_model.jl` against `simple_fig8.jl` and
`simple_opt_reelout.jl` simulations. The model is used by `stability_fig8.jl` and
`stability_opt_reelout.jl`, see [course_loop_stability.md](course_loop_stability.md).
**Status: proposed, nothing run yet.**

## What the model claims

`rel_steering` → heading, discretized at `Ts = 1/sample_freq`:

    ψ(s)/u(s) = c1·v_a · e^(−s·τ_kite) / ((1 + s·T_act)(1 + s·T_kite)(s − c2/v_a·cos ψ0·cos β))

- `T_act = ACTUATOR_LAG = 0.43 s`, the rate-limited KCU tape as an equivalent lag.
- `τ_kite = dead_time·(v_app_row/v_a)^1.03`, `T_kite = kite_lag·(v_app_row/v_a)^1.32`
  (`KITE_DEAD_TIME_EXP`, `KITE_LAG_EXP`).
- `c1`, `c2`, `dead_time`, `kite_lag`, `v_app_row` from `data/turn_rate_coeffs.yaml`.
- Controller: `course_pid(K, Ti, Td, N, Ts)` with the gain schedule of `CourseController`.

The outputs that matter are the **disk margin α, the gain and delay margins, and
the critical frequency** (about 0.5 – 0.8 Hz). The plan tests these directly
wherever it can, not just the fitted parameters.

## What is validated already

| Part | Evidence | Gap |
|---|---|---|
| Actuator lag 0.43 s | fig8 log, phase 4, 1 % unexplained variance | one run, depower 0.27, `v_a` 34 – 38 m/s |
| `c1` | relay sweeps and fig8 log agree to within 3 % | – |
| Dead time + lag over `v_a` | two relay sweeps at depower 0.275 (13.3 and 22.5 m/s) | two points, one depower, 73° elevation |
| Critical frequency | phase-4 ringing at ≈ 0.5 Hz in the fig8 log | only qualitative |
| Frequency response above 0.3 Hz | – | closed-loop data too noisy, few distinct frequencies |

## Tests

The first four tests are simulation runs; the fifth is code only. Unless a test
says otherwise, run the 200 m fig8 project, without turbulence, with `ff_gain = 0`
so that the feed-forward does not mask the feedback loop. Use pure course
feedback (`w_course = 1`), which is what the model assumes.

All hooks go between `calc_steering` and `step!` in the simulation loop of
`simple_fig8.jl` (around line 384), and default to off. V1 needs the first three
in `simple_opt_reelout.jl` too, see [V1](#v1-stability-limits-highest-priority).

    STEER_GAIN_FACTOR = 1.0       # multiplies rel_steering (V1)
    EXTRA_STEER_DELAY = 0         # samples, FIFO on rel_steering (V1)
    HOOK_SETTLE       = 15.0      # s after the start of phase 4 before a V1 hook acts
    STEER_INJECTION   = nothing   # t -> Δu, added to rel_steering and logged (V2)

### V1: stability limits (highest priority)

Test the margins at their source: push the simulated loop until it goes
unstable, then compare with the model.

#### Operating points

The operating point is set by the wind speed (`select_windspeed()`, at the 6 m
reference height, no turbulence). The fig8 pattern gives `v_a` ≈ 5 × wind at
constant length, so it cannot reach 15 m/s without flying at about 3 m/s wind,
which has never been tried. The low point therefore uses the reel-out project
in its phase 4, not its phase 3 as proposed first: phase 3 lasts about 7 s while
`v_a` rises from 13 to 27 m/s, which is too short and too unsteady for a
bisection.

| Point | Project, script | Wind | Expected `v_a`, phase 4 | Source of the expectation |
|---|---|---|---|---|
| A | `system_fig8_200m.yaml`, `simple_fig8.jl` | 7.0 m/s | 34 – 38 m/s | measured, log validation 2026-09-25 |
| B | `system_fig8_200m.yaml`, `simple_fig8.jl` | 4.5 m/s | ≈ 23 m/s | scaled from A, not flown yet |
| C | `system_reelout_maasvlakte.yaml`, `simple_opt_reelout.jl` | 4.0 m/s | ≈ 14 – 16 m/s early in phase 4 | measured 2026-08-20 on the then `system_reelout_150m.yaml` (same `settings_reelout_150m.yaml`) |

B and C are estimates. The first (baseline) run at each point checks the mean
`v_a` of the analysis window, and the wind is adjusted by the ratio before the
sweeps start. Point C reels out, so `v_a` drifts. Its window is only the part of
phase 4 where `v_a` stays within ±10 % of its mean.

#### Settings

For all points (restore them afterwards):

| Setting | File | Value | Why |
|---|---|---|---|
| `ff_gain` | `fc_settings.yaml` (A, B), `fc_settings_reelout.yaml` (C) | 0 | the feed-forward would mask the feedback loop |
| `fig8_pure_course` | both | `true` | pure course feedback from phase 3 on, as the model assumes |
| turbulence | `select_turbulence()` | 0 | deterministic runs, no masking noise |
| `sim_time` | `select_sim_time()` | A: 90 s (default), B: 120 s, C: 120 s | B enters slower; C only needs early phase 4, so it will not reach phase 5 and fails that success criterion, which is expected |

Hooks: `STEER_GAIN_FACTOR`, `EXTRA_STEER_DELAY` and `HOOK_SETTLE` (see
[Tests](#tests)), in `simple_fig8.jl` and, for point C, in `simple_opt_reelout.jl`
next to its existing `STEER_DISTURBANCE` hook (around line 2224). The hooks act only from `t_phase4 + HOOK_SETTLE` on, so the entry and phase 3 fly
unchanged and every run of a sweep enters the window in the same state. With
`ff_gain = 0`, multiplying `rel_steering` is the same as multiplying `heading_p`,
except at the `max_steering` clamp. Log the factor and the delay with the run.

Analysis window: from `t_phase4 + HOOK_SETTLE` to the end of the run, about 40 s
at A and B (3 – 2 laps). One step per run, no step changes within a run.

#### Predictions

Values from `course_loop_model.jl` at the nominal `v_a`, pattern depower and
gain schedule of each project (git `5dfb536`, `margin(L)`, worst gravity sign).
Compute them again at the mean `v_a` and depower of each baseline window before
comparing.

| Point | `v_a` | Depower | `K` floor | Gain margin | Phase crossover | Delay margin | Gain crossover | α |
|---|---|---|---|---|---|---|---|---|
| A | 35 m/s | 0.27 | 23 m/s | 4.08 | 1.03 Hz | 0.43 s (43 samples at 100 Hz) | 0.36 Hz | 0.84 |
| B | 23 m/s | 0.27 | 23 m/s | 2.81 | 0.76 Hz | 0.365 s (37 samples) | 0.35 Hz | 0.66 |
| C | 15 m/s | 0.274 | 10 m/s | 4.66 | 0.48 Hz | 0.91 s (82 samples at 90 Hz) | 0.16 Hz | 0.83 |

C is the inner loop only (`heading_d` 0.126 s, no pattern floor). The
reel-out project also flies the attractor guidance, which lowers the margins.
For C, compare with the guided margins of `stability_opt_reelout.jl` on the
baseline log (`LOG_DIR`), not with the inner-loop values of this table.

#### Procedure

At each point:

1. **Baseline** (`STEER_GAIN_FACTOR = 1`, `EXTRA_STEER_DELAY = 0`). Record the
   window's mean `v_a`, depower and the fraction of time on the tape's rate
   limit. Adjust the wind if `v_a` misses the target by more than 10 %, and
   recompute the predictions.
2. **Gain margin.** Coarse grid at 0.6, 0.8, 1.0, 1.2 and 1.4 × the predicted gain
   margin, e.g. at A: 2.4, 3.3, 4.1, 4.9, 5.7. Then bisect between the last stable
   and the first unstable factor to within 5 % of the prediction (two or three more
   runs). Result: `k_crit` and the ringing frequency `f_crit,k`.
3. **Delay margin.** At `STEER_GAIN_FACTOR = 1`, the same grid in samples, e.g. at
   A: 26, 34, 43, 52, 60. Bisect to within 2 samples. Result: `τ_crit` and its
   ringing frequency `f_crit,τ`.

The two tests ring at different frequencies: the gain test at the phase
crossover (0.5 – 1 Hz), the delay test at the gain crossover (0.16 – 0.36 Hz).
The second band overlaps the lap and its odd harmonics (at A 0.08, 0.23 and
0.39 Hz), which are there in every run. So the onset is judged against the
baseline run, not in absolute terms.

Onset criterion, evaluated in `validate_margins.jl`:

1. Band-pass the regulated error `err`: 0.4 – 2 Hz for the gain test, 0.1 – 1 Hz
   for the delay test.
2. Split the window into 5 s segments and fit the growth rate of the segments'
   RMS, after subtracting the baseline's RMS in the same band.
3. **Unstable:** positive growth rate while the band-passed RMS is still below
   5°. **Stable:** decaying, or no excess over the baseline.
4. Ringing frequency: the peak of the spectrum of `err` minus the baseline
   spectrum.
5. The applied steering rate: the fig8 log sat on the tape's 0.2 s⁻¹ rate limit
   25 % of the time with the feed-forward on. If a run adds more than 10
   percentage points to the baseline's fraction, mark it as **rate-limited**, not
   unstable: the rate limit turns growth into a limit cycle and adds phase lag
   of its own. Bisect only between runs that are not rate-limited.

**Pass:** `k_crit` within ±20 % of the gain margin, `τ_crit` within ±25 % of
`delay_margin(L)`, `f_crit,k` within ±0.1 Hz of the phase crossover and
`f_crit,τ` within ±0.1 Hz of the gain crossover.

Effort: 1 baseline + about 8 gain runs + about 8 delay runs per point, about 50
runs in total.

Open risks: the fig8 pattern has not been flown with `ff_gain = 0` and
`fig8_pure_course = true` together, nor at 4.5 m/s. If the baseline at A or B
fails, find and fix the cause before any sweep.

### V2: frequency response with injected excitation

Measure the plant with the loop closed, but with an excitation the controller
does not generate.

1. Set `STEER_INJECTION` to a periodic multisine: 15 – 20 log-spaced lines from
   0.1 to 2 Hz, random phases, 10 – 20 s period, at least 6 periods.
   Choose the amplitude so the tape stays mostly off its 0.2 s⁻¹ rate limit;
   check that the fraction of time on the limit is below about 10 %.
2. Keep the operating point steady: a reel-in-free fig8 at constant tether length.
   Accept windows only where `v_a` stays within ±10 %.
3. Estimate the plant with the indirect method, at the excited lines only,
   averaged over periods:

       P̂(jω) = G_{r→ψ}(jω) / G_{r→u}(jω)

   `r` is the injection, `u` the total `rel_steering`, `ψ` the fed-back angle.
   Also estimate `Ŝ = G_{r→u}`, the input sensitivity.
4. Overlay `P̂` on `bode(turn_rate_plant(...))` and `|Ŝ|` on the model's
   `1/|1 + L|`, with the ±2σ spread over the periods.
5. Split out the actuator: `P̂_act = G_{r→steering}/G_{r→u}` against
   `1/(1 + s·0.43)`. This tests the equivalent-lag assumption at small amplitude,
   where the tape should look like the 0.33 s linear lag instead.

**Pass:** the model inside the ±2σ band of the magnitude and phase from 0.2 to
1 Hz. The phase at 0.5 – 0.8 Hz matters most. A phase error of 10° or more
there moves α noticeably.

This also checks the dead-time/lag split in the frequency domain, independently
of the time-domain fit of `fit_delay_lag`.

### V3: k-step-ahead prediction on held-out logs

Test the model on data it was not fitted to.

1. Logs: the `simple_opt_reelout.jl` phase-3/4 logs, fig8 runs at other wind
   speeds, and the 150 m and 300 m projects. Do **not** use the runs the model was
   identified on.
2. Pass the logged `set_steering` through the plant, with `v_a`, `c1`, `c2`,
   `τ_kite`, `T_kite` and the gravity term updated each sample from the log.
   Re-initialize the state from the log every `H` seconds, `H` = 1, 2 and 3 s.
   Because of the integrator, a single free run over the whole log would drift.
3. Metrics per `v_a` bin (5 m/s wide) and per depower:
   - normalized RMS error of the heading and of the turn rate over the horizon
   - variance accounted for (VAF) of the turn rate
4. Residual checks on the one-step turn-rate residual:
   - autocorrelation: it should be white
   - cross-correlation with `set_steering`: a peak at lag `k` means the dead time
     is off by about `k·Ts`

**Pass:** turn-rate VAF ≥ 90 % at `H = 1 s` in every bin with enough data, no
residual/input cross-correlation outside the 95 % band, and no trend of the
error with `v_a`. A trend would mean the exponents are wrong.

### V4: operating-point coverage of the scaling laws

The caveats in [course_loop_stability.md](course_loop_stability.md#caveats) name
the extrapolations. Close them with relay sweeps of `build_turn_rate_table.jl`
(re-run with the wind as a parameter) and `fit_delay_lag`:

| Question | Runs |
|---|---|
| Exponents away from depower 0.275 | depower 0.40 at 9.5 and 15 m/s wind |
| Exponents above 22.5 m/s | depower 0.275 at a wind giving `v_a` ≈ 30 m/s |
| Elevation and tether length | one sweep at 26° elevation and 200 m (pattern conditions) |
| Actuator lag at other amplitudes | V2 step 5, plus the fig8 log at depower 0.35 |

**Pass:** each new point lies within ±15 % of `dead_time`·(…)^1.03 and
`kite_lag`·(…)^1.32 as predicted from its table row. If not, refit the exponents
on all points and re-run V1 at one operating point.

### V5: code consistency (unit tests)

These need no simulation and belong in `test/`:

1. **Controller:** feed a recorded error sequence through `course_pid(K, Ti, Td, N, Ts)`
   (`lsim`) and through a `DiscretePID` built as in `CourseController`. The outputs
   must agree to about 1e-10. Cover the `Ti = false` case and a finite `Ti`.
2. **Plant:** for `turn_rate_plant`, check the DC gain of `s·P` (`c1·v_a` when
   `gravity = 0`), the number of shift-register states (`round(delay/Ts)`), and
   the step response of the actuator part (63 % after `lag`).
3. **Scaling:** `kite_dead_time(tc, tc.v_app) == tc.dead_time`, and the error
   for a row without `v_app`.
4. **Margins:** `delay_margin` of a known loop, e.g. `L = e^(−sτ)·k/s`, against
   the closed form `π/(2k) − τ`.

`course_loop_model.jl` lives in `examples/`, so the test file would `include`
it and needs `ControlSystemsBase` in `test/Project.toml`.

## Order and effort

| Step | Test | Needs | Wall time (estimate) |
|---|---|---|---|
| 1 | V5 | code only | short |
| 2 | V1 at point A (`v_a` ≈ 35 m/s) | three hooks in `simple_fig8.jl` | ≈ 17 runs |
| 3 | V1 at points B (23 m/s) and C (15 m/s) | same hooks, also in `simple_opt_reelout.jl` for C | ≈ 34 runs |
| 4 | V3 | existing logs, a replay script | no new runs |
| 5 | V2 | injection hook, FRF script | 3 long runs |
| 6 | V4 | relay sweeps | 4 – 5 sweeps |

V1 comes first because it tests what the analysis is used for. If V1 passes at
all three points, V2 – V4 mainly narrow the uncertainty. If it fails, V2 shows
at which frequency the model is wrong, and V3/V4 show which parameter causes it.

## Deliverables

- `examples/validate_margins.jl`: runs V1 and prints a table of simulated vs.
  predicted limits.
- `examples/frf_injection.jl`: runs V2, estimates the FRF and plots it over the
  model's Bode plot.
- `examples/replay_prediction.jl`: V3, the metrics per `v_a` bin.
- `test/test_course_loop_model.jl`: V5.
- A "Model validation" section in [course_loop_stability.md](course_loop_stability.md)
  with the results. That section should also fix its Model section, which
  still describes a single dead-time exponent of 1.24 (`kite_delay`) instead of
  the dead-time/lag split with 1.03 and 1.32.

## Open questions

- Is the feed-forward (`u_ff`, `chi_ff`) part of what should be validated, or does
  the model stay feedback-only? With `ff_gain = 0` the tests are clean but do not
  cover the flown configuration.
- Does the heading/course blend (`w_course < 1`) need its own model? V1 with the
  default blend, compared against `w_course = 1`, would show whether it matters.
- With turbulence, V2 needs more periods. Is one turbulent V1 run at 23 m/s enough
  to confirm that the margins hold there?
