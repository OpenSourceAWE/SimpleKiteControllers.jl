# Plan: validating the course-loop plant model

2026-09-27. A plan to check the linear course-loop model of
`examples/course_loop_model.jl` against `simple_fig8.jl` and
`simple_opt_reelout.jl` simulations. The model is used by `stability_fig8.jl` and
`stability_opt_reelout.jl`, see [course_loop_stability.md](course_loop_stability.md).
**Status: V5 done and green (`test/test_course_loop_model.jl`). V1's hooks and
`examples/validate_margins.jl` are in place; every settings YAML uses
`steering_gain` 10. Point A (200 m) could not bracket a linear onset (baseline
already rate-limited); point D (300 m) replaced it. At D without the tape's
rate limit: delay margin 0.315 s against the model's 0.357 s (PASS), gain
margin 4.44 against 2.83 (FAIL). A frequency breakdown at the two onsets shows
why: tape and kite models are close, but the model closes the loop on the
heading, while the controller acts on course − commanded course, which follows
the heading only half as strongly at 1.1 Hz. Next: add that link to the model.
See V1's Results sections.**

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
feedback (`w_course = 1`). Note (found in V1, point D): the model's plant ends
at the HEADING, while this loop feeds back the course minus the guidance's
commanded course; the two differ above ~0.5 Hz.

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

#### Results: point A (2026-09-27)

`ff_gain = 0` and `fig8_pure_course = true` set in `data/fc_settings.yaml`.
Baseline: `v_a` = 36.6 ± 0.7 m/s (target 35, within 10 %), depower 0.270, all 8
success criteria pass, peak commanded steering 0.307 of `max_steering` 0.32
(96 %), 17.3 % of the window rate-limited.

**Neither sweep could bracket a linear onset — the actuator saturates first.**
The gain sweep (2.50 – 5.83×, i.e. 0.6 – 1.4× the model's predicted 4.08×) was
84 – 90 % rate-limited at every point, including the lowest. A finer delay
sweep (below) shows the same is true of the delay margin: even the smallest
grid point is already well into saturation, so the coarse grids from
`sweep_gain`/`sweep_delay`'s default 0.6 – 1.4× spacing never contain a
genuinely small-signal `:unstable` point, only `:stable` running into
`:rate_limited` directly.

Delay sweep, rate-limited fraction against extra delay (`v_steering`
threshold check, `Ts` = 0.01 s):

| extra delay | 0 (baseline) | 0.05 s | 0.10 s | 0.11 s | 0.12 s | 0.13 s | 0.14 s | 0.15 s | 0.20 s | 0.26 s |
|---|---|---|---|---|---|---|---|---|---|---|
| rate-limited | 17.3 % | 19.6 % | 28.5 % | 37.1 % | 48.8 % | 73.7 % | 63.4 % | 84.9 % | 84.9 % | 80.8 % |
| verdict | – | `:stable` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` |

Gain sweep, same layout, against the gain factor:

| gain factor | 1.0 (baseline) | 1.05 | 1.10 | 1.15 | 1.20 | 1.30 | 1.50 | 2.00 |
|---|---|---|---|---|---|---|---|---|
| rate-limited | 17.3 % | 23.4 % | 29.3 % | 38.5 % | 49.3 % | 63.5 % | 77.3 % | 90.3 % |
| verdict | – | `:stable` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` | `:rate_limited` |

Both are a **smooth, progressive** rise, not a discrete bifurcation: there is
no window where a small oscillation grows cleanly before the tape saturates.
Interpolating where each crosses the onset criterion's "+10 percentage points
over baseline" line gives:

- **practical delay margin ≈ 0.09 – 0.10 s** (between 0.05 s at 19.6 % and
  0.10 s at 28.5 %), a **factor of 4 – 5 smaller** than the model's predicted
  0.43 s (43 samples).
- **practical `k_crit` ≈ 1.08** (between 1.05 at 23.4 % and 1.10 at 29.3 %), a
  **factor of ≈ 3.8 smaller** than the model's predicted 4.08×.

**Reading:** at point A the pattern already commands steering at 96 % of
`max_steering` in ordinary tracking, so V1's method — uniformly scaling or
delaying the WHOLE tracking command — has essentially no room between normal
operation and actuator saturation. It cannot show the model's assumed
small-signal linear onset; it shows instead that **actuator saturation, not
the linear loop's ringing, is what actually limits point A**, and it binds far
below what the model's margins would suggest. The model's 4.08×/0.43 s numbers
describe a loop that has room to ring before it saturates; this one does not.
This does not confirm or refute `course_loop_model.jl`'s *plant* (it says
nothing about `c1`, `c2`, the dead time or the kite lag) — it is a finding
about how much of that margin is actually usable at this operating point, once
`max_steering` is taken into account. `stability_fig8.jl`'s own α numbers
already come from the linear model alone, so this saturation effect is not
currently reflected in `docs/course_loop_stability.md`'s reported margins
either.

**Two bugs found and fixed while running this** (both in `simple_fig8.jl` /
`simple_opt_reelout.jl`'s V1 hooks, not in `course_loop_model.jl` itself):
1. `STEER_GAIN_FACTOR` multiplied `rel_steering` *after* `calc_steering`'s own
   `max_steering` clamp, with no re-clamp — a factor > 1 could command tape
   angles far outside the calibrated range (observed: 1.33 against a 0.32
   limit) instead of just saturating earlier. Fixed by re-clamping to
   `±fcs.max_steering` after applying the factor.
2. `analyze`'s ±10 % `v_a`-excursion window cut (meant for point C's reel-out
   drift) was applied to points A/B too, where `v_a` swings ±20 % or more
   WITHIN A SINGLE LAP as ordinary pattern geometry, not drift — it was
   shredding otherwise-valid 40 s windows down to a fraction of a second.
   Fixed by scoping the cut to point `:C` only.

#### Redesign: a baseline that is not rate-limited (point D)

Point A failed because its baseline already sat on the tape's limits. The
steering a pattern needs is about u ≈ 1/(c1·L·ρ), with ρ the pattern's angular
turn radius, so its amplitude does not depend on the wind; the tape RATE it
needs scales with amplitude × lap frequency, ∝ v_a/(L·ρ)². A longer tether
therefore helps both, a lower wind only the rate (and the gravity term c2/v_a
raises the amplitude at low v_a). Screening baselines, same `fc_settings.yaml`:

| Candidate | `v_a` | rate-limited | peak cmd / `max_steering` | window |
|---|---|---|---|---|
| A: 200 m, 7 m/s | 36.6 m/s | 17.3 % | 96 % | 40 s |
| **D: 300 m, 7 m/s** | 34.2 m/s | **0.5 %** | 80 % (median 6 %, 95th pct. 66 %) | 60 s |
| 300 m, 5 m/s | 24.7 m/s | 2.3 % | 93 % | 20 s, phase 4 too late |

Point D (`system_fig8_300m.yaml`, 7 m/s, `sim_time` 120 s) is the new baseline.
Its tape is (nearly) off the rate limit, so `predict` uses the tape's
small-signal lag, `1/steering_gain` of the project's settings (`v1_lag`),
instead of the rate-limited equivalent `ACTUATOR_LAG` (0.43 s). The screening
above used the KiteUtils default `steering_gain` 3 (0.33 s).

The GAIN test still cannot work by scaling the whole command: to reach the
predicted gain margin of ~3.6 the command must stay below the clamp, so the
baseline peak would have to be under ~25 % of `max_steering`, which no pattern
flown on feedback alone reaches. The hooks now have `STEER_GAIN_FEEDBACK_ONLY`
(`run_v1`/`sweep_gain`: `feedback_only = true`): with `ff_gain > 0` it scales
only `rel_steering - u_ff`. The feed-forward lies outside the loop `L = C·P`, so
this scales the loop gain alone while the feed-forward carries the turns. Not
run yet.

#### Results: point D, delay margin, steering gain 3 (2026-09-27, superseded)

Kept for the record: flown with the KiteUtils default `steering_gain` 3 before
every settings YAML was set to 10 (next section).

Baseline: `v_a` 34.25 m/s, depower 0.270, 0.5 % rate-limited, peak command 80 %.
Prediction (lag 0.33 s): delay margin 0.408 s (41 samples) at the gain
crossover 0.40 Hz, gain margin 3.62 at 1.09 Hz, α 0.82.

| extra delay | 0.08 s | 0.16 s | 0.20 s | 0.21 s | **0.22 s** | 0.25 s | 0.33 s | 0.41 s | 0.49 s |
|---|---|---|---|---|---|---|---|---|---|
| rate-limited | 2.2 % | 6.6 % | 7.0 % | 9.9 % | **50.9 %** | 73 % | 72 % | 72 % | 74 % |
| peak cmd | 81 % | 81 % | 80 % | 81 % | **100 %** | 100 % | 100 % | 100 % | 100 % |

Unlike point A, the transition is SHARP: from 21 to 22 samples the loop goes
from near-baseline flight into a saturated limit cycle, which is what an
unstable loop becomes once the tape limits it. `onset` labels such a run
`:rate_limited`, not `:unstable` (it never sees small-amplitude growth inside
the window), so the bracket was bisected by hand between the last `:stable`
and the first `:rate_limited` run: **τ_crit = 0.215 s ± 0.005 s.**

Frequency: the zero-crossing estimate is useless here (the lap forcing
dominates the regulated error, 11° RMS in 0.1 – 1 Hz). Instead, the peak of the
error's amplitude spectrum in EXCESS of the baseline's (Hann window, whole
window) marks the lightly damped mode that forms near the boundary:

| extra delay | 0.16 s | 0.20 s | 0.21 s | 0.22 s | 0.25 s |
|---|---|---|---|---|---|
| excess peak | 0.47 Hz, 2.7° | 0.47 Hz, 3.7° | 0.36 Hz, 4.2° | 0.30 Hz, 10.7° | 0.19 Hz, 53° |

Below the onset the mode sits at **0.36 – 0.47 Hz**, around the predicted 0.40
Hz; in the limit cycle the frequency falls as the amplitude grows (the rate
limit's phase lag grows with amplitude).

| | measured | model | criterion | |
|---|---|---|---|---|
| `τ_crit` | 0.215 s | 0.408 s | ±25 % | **FAIL** (−47 %) |
| `f_crit,τ` | 0.36 – 0.47 Hz | 0.40 Hz | ±0.1 Hz | PASS |

**Reading:** the crossover frequency is right, so the loop gain `|L|` near
0.4 Hz (`c1`, the gain schedule) is right; the PHASE is not. The model has
about 360° · 0.40 Hz · 0.19 s ≈ **28° too much phase margin** at the crossover:
the simulated plant has roughly 0.19 s more effective delay than
`course_loop_model.jl` assumes at this operating point. The actuator lag does
not explain it: with 0.43 s instead of 0.33 s the predicted margin is 0.43 s,
not smaller. Candidates to check: the kite's dead time and lag (identified at
73° elevation and depower 0.275, flown here at 26° and 300 m), tether dynamics
at 300 m, and the VSM update interval (`vsm_interval` 10, i.e. 0.1 s). V2's
frequency response would show at which frequency the phase departs; V3/V4
would show which parameter causes it.

#### Results: point D, delay margin, steering gain 10 (2026-09-27)

`kcu.steering_gain: 10.0` is now set in every settings YAML with a `kcu`
section (fig8 150/200/300 m, reel-out 150/180 m, Cabauw), so the tape's
small-signal lag is 0.1 s. A faster tape asks for higher tape speeds (10 ×
the position error instead of 3 ×), so it hits the 0.2 s⁻¹ rate limit more
often: re-screened baselines, A 17 % → 28 %, D 0.5 % → 7.7 % rate-limited.
D is still far from saturated and stays the test point.

Baseline: `v_a` 34.32 ± 0.85 m/s, depower 0.270, 7.7 % rate-limited, peak
command 76 %. Prediction (lag 0.1 s): delay margin 0.357 s (36 samples) at the
gain crossover 0.57 Hz, gain margin 2.83 at 1.54 Hz, α 0.78.

`sweep_delay` now bisects on its own: from a baseline under `CLEAN_BASELINE`
(10 %) rate-limited, a jump into `:rate_limited` counts as the unstable side
(`is_unstable`, `bracket`).

| extra delay | 0.07 s | 0.14 s | 0.22 s | 0.25 s | 0.26 s | **0.27 s** | 0.29 s | 0.36 s | 0.43 s |
|---|---|---|---|---|---|---|---|---|---|
| rate-limited | 8.9 % | 11 % | 14 % | 13 % | 13 % | **18.5 %** | 79 % | 85 % | 86 % |
| peak cmd | 77 % | 78 % | 81 % | 76 % | 76 % | 89 % | 100 % | 100 % | 100 % |
| excess peak | 0.26 Hz, 0.3° | 0.37 Hz, 0.7° | 0.37 Hz, 1.5° | 0.47 Hz, 1.6° | 0.47 Hz, 1.8° | 0.47 Hz, 3.1° | 0.18 Hz, 53° | 0.16 Hz, 66° | 0.16 Hz, 94° |

The automatic bracket is 26 – 27 samples. The 27-sample run is only just past
the onset criterion (+10.8 points over the baseline, the command not yet at the
clamp), so 28 samples was flown as a check: a full limit cycle (77 %
rate-limited, command at the clamp). 26 is stable, 27 marginal (the mode grows
but has not saturated within the window), 28 unstable: **τ_crit = 0.27 ±
0.01 s**, 0.265 s if 27 counts as unstable, 0.275 s if not. `validate_margins.jl`
now uses one criterion for both tests, a saturated limit cycle (`limit_cycle`:
command at the clamp and more than half the window rate-limited), which puts 27
on the stable side.

| | measured | model | criterion | |
|---|---|---|---|---|
| `τ_crit` | 0.27 ± 0.01 s | 0.357 s | ±25 % | **borderline**, −23 to −26 % |
| `f_crit,τ` | 0.47 Hz | 0.57 Hz | ±0.1 Hz | PASS (at the limit) |

**Reading:** with the faster tape the gap is about half of what it was:
0.08 – 0.09 s of missing delay instead of 0.19 s, 360° · 0.57 Hz · 0.09 s ≈ **19°**
too much phase margin instead of 28°. About 0.1 s of the earlier gap was
therefore the gain-3 tape, which lags more near 0.4 Hz than a 0.33 s first-order
lag, most likely because the oscillation at the onset drives it onto its rate
limit. The remaining ~0.09 s is in the plant or the simulation; the 0.1 s
aerodynamic update (`vsm_interval` 10, on average about half of it as delay)
would account for about half of that. The measured critical frequency lies
0.1 Hz below the model's, consistent with a loop that has more phase lag than
modelled.

#### Results: point D, gain margin, feedback only (2026-09-27)

`ff_gain` back to 1.0 in `data/fc_settings.yaml` (`fig8_pure_course` stays
`true`), `STEER_GAIN_FEEDBACK_ONLY`: the feed-forward carries the turns and only
`rel_steering - u_ff` is scaled. In phase 4 `calc_steering` adds `u_ff`
unchanged, so the subtraction is exact. Baseline: `v_a` 34.21 m/s, **2.7 %**
rate-limited, peak command **60 %** (the feed-forward lowers both). Prediction:
gain margin 2.83 at the phase crossover 1.53 Hz.

| k (feedback only) | 1.13 | 1.70 | 2.27 | 2.83 | 3.40 | 4.00 | **4.125** | **4.25** | 4.50 | 5.0 | 6.0 | 8.0 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rate-limited | 3.4 % | 7.6 % | 12.5 % | 17.3 % | 21.6 % | 30 % | **36 %** | **53 %** | 66 % | 82 % | 93 % | 95 % |
| peak cmd | 61 % | 65 % | 68 % | 72 % | 76 % | 87 % | 99 % | 100 % | 100 % | 100 % | 100 % | 100 % |
| cmd content 0.8 – 3 Hz | 0.0001 | 0.0008 | 0.0018 | 0.0028 | 0.0055 | 0.0104 | 0.011 | 0.028 | 0.030 | 0.029 | 0.055 | 0.040 |
| at | 0.86 Hz | 0.86 | 0.86 | 0.86 | 1.06 | 1.04 | 0.90 | 0.94 | 0.80 | 0.80 | 0.84 | 0.92 |

The onset criterion of the delay test does not work here: the rate-limited
fraction rises SMOOTHLY with k (the scaled feedback carries more high-frequency
content) and crosses "+10 points over baseline" at k ≈ 2.3 with no oscillation
at all — `sweep_gain` reported "k_crit 2.28, PASS", which is an artifact of the
threshold, not an onset. The onset is where a mode near 1 Hz grows in the
steering command (amplitude spectrum of `set_steering`, 0.8 – 3 Hz, in excess
of the baseline's) and turns into a saturated limit cycle: it grows from 3.4 to
4.125 and jumps between 4.125 (0.011, command just below the clamp) and 4.25
(0.028, at the clamp, 53 % rate-limited). Bisected on "command at the clamp and
more than half the window rate-limited": **k_crit = 4.19 ± 0.06**, the mode at
**0.9 – 1.04 Hz** just below it.

| | measured | model | criterion | |
|---|---|---|---|---|
| `k_crit` | 4.19 | 2.83 | ±20 % | **FAIL**, +48 % |
| `f_crit,k` | 0.9 – 1.04 Hz | 1.53 Hz | ±0.1 Hz | **FAIL** |

#### What point D says about the model

The two tests miss in OPPOSITE directions: about 25 % less delay margin than
predicted, 48 % more gain margin. No single change to the linear model
reproduces both (fitted against the two measured margins, 2026-09-27):

- less loop gain (`c1`, K) raises both margins; extra dead time lowers both;
- an extra first-order lag of 0.3 – 0.4 s reproduces the gain side (GM 3.9 –
  4.3 at 0.98 – 1.04 Hz) but RAISES the delay margin to 0.42 s;
- the best fit of loop gain × extra lag misses both margins by 10 – 30 % and
  puts the crossovers at 0.3 / 0.6 Hz.

The first guess — the tape's rate limit, active 13 – 36 % of the window near
the onsets — was tested by repeating both sweeps without it (next section). It
explains the delay side only.

#### Results: point D without the rate limit (2026-09-27)

`v_steering` 1.0 s⁻¹ instead of 0.2 in `data/settings_fig8_300m.yaml` for these
runs only (restored afterwards); everything else as above. Both baselines are
0.0 % rate-limited. Onset judged by the mode in the steering command
(`set_steering` amplitude spectrum in excess of the baseline's) growing until
the command reaches the clamp.

Gain, feedback only (`ff_gain` 1):

| k | 2.0 | 2.4 | 2.83 | 3.4 | 4.0 | 4.25 | **4.375** | **4.5** | 5.0 |
|---|---|---|---|---|---|---|---|---|---|
| peak cmd | 67 % | 70 % | 72 % | 78 % | 79 % | 89 % | 95 % | 100 % | 100 % |
| mode in cmd | 0.0014 | 0.0016 | 0.0016 | 0.0027 @ 1.16 Hz | 0.0038 @ 1.06 | 0.0079 @ 1.14 | 0.017 @ 1.14 | 0.026 @ 1.12 | 0.127 @ 0.94 |

Delay (`ff_gain` 0):

| extra delay | 0.14 s | 0.22 s | 0.29 s | 0.30 s | **0.31 s** | **0.32 s** | 0.36 s | 0.43 s |
|---|---|---|---|---|---|---|---|---|
| peak cmd | 77 % | 80 % | 84 % | 88 % | 100 % | 100 % | 100 % | 100 % |
| mode in cmd | 0.0035 | 0.0085 @ 0.58 Hz | 0.019 @ 0.48 | 0.027 @ 0.54 | 0.048 @ 0.52 | 0.065 @ 0.52 | 0.127 @ 0.48 | 0.240 @ 0.42 |

Both onsets are clean now: a single mode grows steadily with k or τ.

| | measured | model | criterion | |
|---|---|---|---|---|
| `τ_crit` | 0.315 ± 0.005 s | 0.357 s | ±25 % | **PASS**, −12 % |
| `f_crit,τ` | 0.52 Hz | 0.57 Hz | ±0.1 Hz | PASS |
| `k_crit` | 4.44 ± 0.06 | 2.83 | ±20 % | **FAIL**, +57 % |
| `f_crit,k` | 1.13 Hz | 1.53 Hz | ±0.1 Hz | **FAIL** |

With the flown rate limit the delay margin was 0.27 s; the limit costs about
0.045 s there. The gain margin barely moves (4.19 → 4.44): the rate limit is
not why it is larger than predicted.

**Where the loop differs from the model.** At each onset the loop is on its
stability boundary, which gives one point of the real loop gain `L`: at the
delay onset |L(0.52 Hz)| = 1 and its phase is −180° + 360° · 0.52 · 0.315 s; at
the gain onset its phase at 1.13 Hz is −180° and |L| = 1/4.44. Against the
model's `L` there: 0.94 and 18° more lag at 0.52 Hz, but **0.42** and 29° more
lag at 1.13 Hz. The oscillating runs split this up, because at the onset
frequency the mode dominates every logged signal (DFT at that frequency, Hann
window, runs 0.31/0.32 s and k 4.375/4.5):

| real / model, gain and phase | 0.52 Hz | 1.13 Hz |
|---|---|---|
| command → tape (`set_steering` → `steering`) | 1.00, +2° | 1.00, +2° |
| tape → heading (`steering` → `heading`) | 1.00, −7° | 0.77, −11° |
| heading → regulated error (`var_06`), NOT in the model | 0.96, −15° | **0.55, −25°** |
| of which heading → course | 0.89, −1° | 0.53, −12° |
| **product** | **0.96, −20°** | **0.42, −34°** |
| loop, from the margins | 0.94, −18° | 0.42, −29° |

The product reproduces the measured loop gain at both frequencies. So:

1. **The tape model is right**: gain 10 as a 0.1 s first-order lag matches to
   1 % and 3° up to 1.1 Hz, as long as it stays off the rate limit.
2. **The kite model is close**: exact at 0.5 Hz up to ~7° (≈ 0.04 s) of extra
   lag; at 1.1 Hz it is 23 % too strong and 11° short on lag.
3. **The model closes the loop on the wrong signal.** Its plant ends at the
   HEADING, but with `fig8_pure_course` the controller acts on the COURSE minus
   the guidance's commanded course. Course follows heading only partly at
   higher frequency (0.53 and −12° at 1.1 Hz: the velocity vector turns more
   slowly than the wing), and `chi_cmd` itself moves with the kite, because
   the attractor guidance is an outer loop. Together they cut the loop gain at
   1.1 Hz almost in half — the main reason for the larger gain margin — and add
   15 – 25° of lag, which with the kite's extra lag is the delay side's 12 %.
4. **The rate limit** is a separate, flown effect: at 0.2 s⁻¹ it costs about
   0.045 s of delay margin at this point.

**What to change in `course_loop_model.jl`:** add the heading → course
dynamics (a first fit: a lag or a second-order low-pass between 0.5 and
1.1 Hz, from these two points or better from V2) and, for pattern flight, the
guidance loop as in `stability_opt_reelout.jl`'s guided loop; then re-predict D.
V2's injected multisine would measure all three links (tape, kite, course)
over 0.1 – 2 Hz at once, instead of at two onset frequencies.

**Not yet done:** the model change above, points B and C, and a note in
`docs/course_loop_stability.md`.

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
