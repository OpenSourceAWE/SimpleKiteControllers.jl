# Fix oscillations at 9 m/s wind

At wind speeds above 8 m/s strong force oscillations appear because the upper force controller
is getting activated.

## Measured: what the 9 m/s run actually does

Source: `output/archives/2026-08-20_150749` (9.0 m/s, git `3ea53e1` clean, "all 9 passed",
28464 W). Settled window 27.6–100 s, from `reelout_150m_opt.arrow`.

| quantity | value | reference |
| --- | --- | --- |
| time in `wcsUpperForceLimit` (`var_12` = 2) | **87.6 %** | target is 0 %, see below |
| time in speed control (`var_12` = 1) | 12.4 % | |
| state transitions | 35 in 72 s = **0.48 /s** | |
| tether force, mean / std | **8176 N** / 2998 N (cv 36.7 %) | `f_high` = 8000 N |
| tether force, min / max | 3146 N / **13974 N** | plant `max_force` = 8400 N |
| samples above `f_high` | 55.9 % | |
| samples above plant `max_force` | **45.5 %**, peak 66.4 % over | |
| reel-out speed, mean / std | 3.12 m/s / **3.13 m/s** | `v_sat` = 8.0 m/s |
| reel-out speed, min / max | **−1.26 m/s** / 8.25 m/s | negative = reeling IN |
| force oscillation period | 3.63 s (0.276 Hz) | half-lap is 5.17 s |
| corr(force, elevation) | **−0.06** | |

## Diagnosis

**It is a winch limit cycle, not state-chatter and not the pattern's load rhythm.**

- Not hysteresis chatter: 0.48 transitions/s is slow. The 5 % deadband
  ([winchcontroller.jl:99-100](../WinchControllers/src/winchcontroller.jl#L99-L100)) is not the
  problem — widening it would change nothing.
- Not the figure-eight's own aero load cycle: force is essentially **uncorrelated with lobe
  position** (r = −0.06 against elevation), and the 3.63 s period does not match the 5.17 s
  half-lap. A load cycle driven by the pattern would track both.
- It is the winch loop cycling: the drum swings ±3 m/s about a 3.12 m/s mean and reaches
  **−1.26 m/s (reeling in) to 8.25 m/s (at `v_sat`)**. Speed std equals speed mean.

**Why the limiter has no authority.** The mean force (8176 N) sits **above** `f_high` (8000 N),
so `UpperForceController` is pinned 87.6 % of the window with nothing left to give. The speed law
only asks for `kv*sqrt(f_high)` = 0.0408·√8000 = **3.65 m/s** at the limit, but holding 8000 N at
9 m/s needs more reel-out than that — so the limiter saturates against `v_sat` and cycles.

**This run should not have passed.** The repo's own side condition is
`max_pct_time_upper_force: 0.0` ([data/optimization.yaml:36](data/optimization.yaml#L36)) — the
upper force controller is meant to be active **0 %** of the time. `simple_opt_reelout.jl`'s 9
success criteria do not include it, which is why 87.6 % reported as "all 9 passed". The tether is
also over the plant's `max_force` 45.5 % of the window, peaking 66 % over it — nothing flags that
either.

## The depower question, answered

The plan asked whether the optimizer returns a higher depower at 9 m/s. Both halves matter:

- The optimizer's `input_depower` **is** ramped with wind: `1.6 m` at/below the 7.0 m/s reference,
  plus `0.25 m` per m/s above it, so 9 m/s sent `depower_seed_m: 2.1`
  ([data/traj_opt.yaml:87-112](data/traj_opt.yaml#L87-L112), confirmed in the archive's summary).
- But that is **only a seed for the path**. The run flies the fixed `depower_setpoint: 0.274`
  from [data/fc_settings_reelout.yaml:39](data/fc_settings_reelout.yaml#L39) regardless of wind
  (measured: 0.274 on the pattern, rising to `depower_final` 0.328 only in phase 5).

So the flown depower does **not** follow the wind, and it is already at its documented ceiling:
0.275 is a curvature ceiling, not a tuned optimum — 0.278 drops the startup margin to
`min_feasibility_margin` = 0.9 and the script refuses to fly
([fc_settings_reelout.yaml:28-38](data/fc_settings_reelout.yaml#L28-L38)).

**What the optimizer itself wants, correctly converted.** The 9 m/s archive's live AWETrim
session (still running, confirmed by matching timestamps) reports the LAST leg's optimized
`input_depower` as `l_dp = 1.4171 m` — a genuine post-solve value (`solution.value(mx)` in
`phase.py:842`, exactly matching `output/awetrim_server.log:281085`), not the seed. Converting it
naively with only the two scales' 0.4 m zero difference (`0.2 + 5*u_p` vs `0.6 + 5*u_p`,
[docs/steering_depower.md:147-157](docs/steering_depower.md#L147-L157)) gives `u_p ≈ 0.243` — LESS
depower than flown (0.274), which looked like it argued against raising depower.

That conversion was incomplete. `docs/steering_depower.md:181-196` already measured the FULL offset
between the two models at equal power — 0.12 of `rel_depower` (0.61 m of tape), of which the 0.4 m
scale difference is only two thirds; the rest is genuine aerodynamic disagreement. Applying the
full 0.12 (added in `examples/awetrim_client.jl`'s new `awetrim_depower_to_v3kite`, used by
`simple_opt_reelout.jl`): `u_p_equiv = (1.4171 - 0.6)/5 + 0.12 ≈ 0.283` — close to, and slightly
**above**, the flown 0.274. So once correctly converted, the optimizer is not arguing against more
depower at 9 m/s — it is roughly confirming the flown setting, maybe wanting a touch more. This
restores option A1 below rather than undercutting it, and the two are compatible: raising depower
with wind still needs the curvature-ceiling problem solved, since 0.274 is already near the 0.275
ceiling.

**Implemented (2026-08-20):** `simple_opt_reelout.jl` now logs every optimizer-reported depower
(startup solve + each reopt) to `opt_depower_log`, persists it into the run summary YAML under
`traj_opt.guess.depower_optimized`, and `simple_reelout_plots.jl`'s power figure adds a 5th panel
(flown `rel_depower` vs the optimizer's `u_p_equiv`, step-held between reopts) when that log is
present. Not yet run at any wind speed with this logging in place — the 1.417 m figure above came
from the live server session and the raw log, not from the new machinery. A fresh sweep (3-9 m/s)
would give a clean, comparable series and is the natural next check.

## Options, ranked

This is an **authority/headroom problem first, a gain problem second**. While the mean force is
above `f_high`, no retuning of the limiter can fix it — it has no headroom to regulate into. So
lower the force before touching gains.

### A. Reduce the aero force so the mean drops below `f_high` (the real fix)

1. **Make depower follow the wind.** 
1a. Find out if the optimizer provides a depower setpoint that we can use
First finding: Not directly, there is an offset to our notation that we need to determine.

1b. Today `depower_setpoint` is wind-independent while the
   optimizer's seed already ramps. The obstacle is the curvature ceiling, which is a function of the *path*, not the wind — so this needs the ceiling lifted alongside:
   - at 180 m the ceiling is 0.282 (vs 0.278 at 150 m);
 
### B. Give the winch more authority

Raise `kv` (0.0408) so `v_set` responds harder to force. Note `v_sat` = 8.0 m/s is already being
touched (8.25 m/s peak), so speed headroom is nearly gone — raising `v_sat` trades against drum
and tether limits and should be checked against the plant, not just the controller.

### C. Retune `UpperForceController` for *sustained* engagement

Only after A. These gains are explicit package defaults, never fit to this plant
([data/wc_settings.yaml:38-43](data/wc_settings.yaml#L38-L43)): `pf_high` 0.00023040,
`if_high` 0.012, `df_high` 0.000034, `kbf_high` 1.0, `ktf_high` 10.0. For a limit cycle under
saturation the suspects are `if_high` (too much integral action with nowhere to go) and
`kbf_high` = 1.0 (weak back-calculation anti-windup against `ktf_high` = 10). They were only ever
exercised in brief touches, never at 87.6 % duty.

### D. Fix the `WinchPosController` anti-windup mismatch

Its inner speed PID has anti-windup `Tt` ≈ 10 s against `winch_speed_ti` = 2 s — 5× slower than
the integrator it protects. [docs/control_algorithm.md:249-253](docs/control_algorithm.md#L249-L253)
already calls this "the most likely source of overshoot after a long saturation", untested.
Sustained upper-force engagement is exactly that condition, so this is now testable for the first
time.

### E. Add the missing guards (independent of the fix)

Add `pct_time_upper_force` and a tether-overload check to `simple_opt_reelout.jl`'s success
criteria, so a run like this fails loudly instead of reporting "all 9 passed". The metric is
already computed — `upper_force_pct` in [src/fig8_metrics.jl:366](src/fig8_metrics.jl#L366).

## Next step

Try A1 first (wind-ramped depower on the fixed lemniscate, which is the only variant whose ceiling
clears 0.30), and re-measure the same table. The target is mean force below 8000 N and
`upper_force_pct` near 0 — at which point C and D become tunable rather than moot.
