# Improve power_ratio

`power_ratio` (measured / predicted mean reel-out power, against the weighted
prediction) is 1.00 up to 8 m/s and then falls away: **0.91 at 9, 0.82 at 10,
0.83 at 11 m/s** (August 2026; for today's values see "Status 2026-10-01"). This is not a 10 m/s problem; the pattern across 9/10/11 is
the evidence.

**Acceptance:** `power_ratio` in 0.98 .. 1.02 at 9, 10 and 11 m/s, with all
success criteria still passing and measured power not lower than today's. The
band is the one used in `oldplans/PlanTunePowerRatio.md`.

Measured power must not be traded away to close the ratio. If the ratio closes
because the PREDICTION comes down to what the plant can fly, that is the
intended outcome — see the hypothesis below.

## Status 2026-10-01

**Not met at 9, 10 and 11 m/s.** Maasvlakte scenarios of 2026-10-01, 07:23 – 07:39
(`output/scenarios/maasvlakte/overview.md`), all 10 success
criteria passed in every run:

| v_wind [m/s] | 6 | 7 | 8 | 8.25 | 8.5 | 9 | 10 | 11 |
|---|---|---|---|---|---|---|---|---|
| power_ratio | 1.00 | 1.00 | 0.99 | 0.98 | 0.96 | **0.94** | **0.83** | **0.84** |
| av_power [W] | 7213 | 12004 | 18008 | 19836 | 21109 | 22503 | 19840 | 19121 |
| av_force [N] | 3193 | 4477 | 5780 | 6134 | 6386 | 6694 | 6135 | 6005 |
| max_force [N] | 4083 | 5268 | 6715 | 6784 | 7136 | 7902 | 6914 | 6838 |
| av_depower [-] | 0.27 | 0.267 | 0.266 | 0.266 | 0.268 | 0.271 | 0.297 | 0.316 |

(Whole-run means and extremes as printed by the summary, not the phase-4 window
of the table below; 3.5 – 5 m/s: 1.05, 1.07, 1.02.)

- Against August: 9 m/s improved (0.91 -> 0.94), 10 and 11 m/s did not (0.82 ->
  0.83, 0.83 -> 0.84). The ratio now leaves the band already at 8.5 m/s (0.96).
- The pattern is unchanged: measured power peaks at 9 m/s and falls above it,
  with the mean force, while the flown depower climbs (0.271 -> 0.297 -> 0.316).
- At 10 and 11 m/s the peak force (6914, 6838 N) stays 4 – 5 % below the 7200 N
  ceiling, while at 9 m/s it exceeds it (7902 N).

**Force ceilings since the plan was last updated (2026-08-30).** None of these
changes was swept against `power_ratio`:

| commit | date | `f_high` [N] | `f_high_awe_trim` [N] |
|---|---|---|---|
| `3486b1e` | 2026-08-30 | 8000 | 8000 (= no de-rating) |
| `cc2f04e` | 2026-09-18 | 8000 | 7700 |
| `f01143c` | 2026-09-21 | 8000 | 7500 |
| `5364faa` | 2026-09-21 | 7900 | 7500 |
| `8492804` | 2026-09-23 | **7200** | **7200** |

So since 2026-09-23 the value sent with the re-optimizations equals the runtime
ceiling: there is no de-rating in effect, the ceiling itself was lowered. The
startup solve and lap 1 fly under `f_high · first_lap_force_frac` = 7200 · 0.93
= 6696 N (`6ce0abe`, 2026-09-24). The sections below were written in the
8000 N era (August 2026).

**Step 1 done (2026-10-01):** the winch laws agree, the gap is a 13 % force gap
that opens with the depower; see Step 1. **Not done:** the depower confound.

## What is already known

Measured against the archived runs in `output/scenarios/` (v06 .. v11, phase-4
window, no simulation re-run needed):

| | ratio | pred W | meas W | F mean | F needed for pred | crest | peak if F_needed flown |
|---|---|---|---|---|---|---|---|
| v08 | 0.99 | 18684 | 18407 | 5980 N | 6010 N | 1.18 | 7070 N |
| v09 | 0.91 | 23931 | 21870 | 6659 N | 7039 N | 1.26 | **8834 N** |
| v10 | 0.82 | 24561 | 20163 | 6502 N | 7375 N | 1.17 | **8658 N** |
| v11 | 0.83 | 22840 | 18892 | 6065 N | 6877 N | 1.24 | **8511 N** |

Measured power PEAKS at 9 m/s and declines above it, because mean tether force
does. Flown depower climbs 0.265 -> 0.274 -> 0.299 -> 0.312 over 8 -> 11 m/s.

### The winch force/speed curve is NOT the cause

The original suspect for this plan was that the `v_set = kv*sqrt(force)` curve
flown in Julia differs from the one AWETrim optimizes against. Three
independent lines refute it:

1. **The runtime winch tracks its own law exactly.** Fitting
   `kv = v_reelout/sqrt(winch_force)` per sample over the phase-4 window returns
   the commanded value at every wind speed (0.0398 .. 0.0405 against 0.0408;
   0.0388 against v10's 0.039). Mean speed residual on v10 is -0.02 m/s on 3.12,
   and measured power 20325 W against `mean(kv*F^1.5)` of 20496 W — 0.8 % apart.
   There is no room for an 18 % error here.
2. **The 2026-08-27 kv sweep already settled it.** Over kv 0.0408 -> 0.036 at
   10 m/s, power moved 20051 -> 22543 W while `power_ratio` stayed at 0.84-0.85.
   kv moves the power LEVEL, not the gap (`docs/fig8_tuning_log.md`, "10 m/s kv").
3. **The curve is shared by construction.** `winch_from_wc` sends `kv`, `f_low`,
   `f_high`, `v_sat`, both betas and `use_awe_trim` straight off the same
   `WCSettings` the runtime `WinchController` is built from. The one deliberate
   divergence — `softminus_beta` pinned to 1e-3 for the server against 0.03
   locally — is at the LOWER force limit, and v10 spent 0.0 % of the window in
   the `LowerForceController`.

Because the law holds, the power gap IS a force gap: 6502 N flown against the
7347 N that 24561 W requires under it, 13 % down, which cubes to the 0.82
observed.

## Working hypothesis

*(As formulated in August 2026, when `f_high` was 8000 N; see the note at the end
of this section for today's 7200 N.)*

**The plant's 8400 N rating binds on the PEAK force, while AWETrim optimizes the
mean against `f_max = f_high = 8000 N` with no knowledge of the ~1.2 crest
factor of real figure-eight flight.** Its optimum is therefore not flyable
inside the rating, and the run is detuned by depower to keep peaks legal —
which is exactly what `power_ratio` is measuring.

The gap switches on precisely where `F_needed * cf_force_ro` crosses 8400 N: at
8 m/s it does not (7070 N) and the ratio is 0.99; from 9 m/s up it does, on all
three runs.

If this holds, the fix is to send a de-rated `f_max` so the predicted optimum
is one the plant can fly. The ratio then closes and measured power barely
moves.

**Note 2026-10-01: today's runs contradict the hypothesis in this form.** With
`f_max = f_high = 7200 N`, AWETrim's optimum has 1200 N, i.e. a crest factor
of 1.17, of headroom below the 8400 N rating. That is the de-rating proposed here,
reached by lowering `f_high` instead. Yet at 10 and 11 m/s the peak force is only
6914 and 6838 N, 18 – 19 % below 8400 N, and the ratio is still 0.83 and 0.84
("Status 2026-10-01"). The peak force does not bind. Nor is the run "detuned by
depower": it flies the optimizer's own depower (`fly_opt_depower`, see "Confound
to control"), so the higher depower at 10 – 11 m/s is the optimizer's choice, not
a correction by the run to keep peaks legal. This agrees with the
Step 2a result below (de-rating did not move the ratio), and points to the
depower confound: the run flies the optimizer's depower, so an error in its
conversion, calibrated at 6 m/s only, would show up here ("Confound to control").

## Step 1 — decisive, no simulation

Ask the server for the mean force and reel-out speed its reply predicts, at 8
and at 10 m/s.

- Predicted speed consistent with `v = kv*sqrt(F)` at the shipped kv (0.0408
  at 10 m/s since 2026-09-22; 0.039 when this was written, with a predicted
  force of about 7350 N then) -> the two winch laws agree, the cause is the
  force rating. Go to Step 2a.
- Predicted speed NOT consistent with that relation -> the original suspect is
  live after all, but in the UPPER saturation (`softplus_beta`, `f_max`,
  `v_max = v_sat` 3.5 m/s), not `kv`. Go to Step 2b.

**Done 2026-10-01, on the Maasvlakte runs of that morning, without a new
solve.** `replay_entries` finds every path the 8 – 11 m/s runs installed in the
solution cache (`output/opt_chain_cache`); each entry holds the server's full
`/trajectory` table. Predicted: time-weighted means over the pattern of
`tension_tether_ground` and `speed_radial`, laps 2 on (path 1 is the startup
solve at `f_max` 6696 N, 150 m). Measured: `reelout_power` over the reel-out
window of the log. `kv` sent is 0.0408 at every wind speed, `f_max` 7200 N.

| v_wind | predicted F mean / max [N] | predicted v [m/s] | predicted P [W] | measured F mean / peak [N] | measured v [m/s] | measured P [W] | F meas / pred |
|---|---|---|---|---|---|---|---|
| 8 | 5749 – 5921 / 6395 – 6755 | 3.16 – 3.22 | 18198 – 19072 | 5777 / 6715 | 3.08 | 17850 | 0.99 |
| 9 | 6971 – 7003 / 7189 – 7198 | 3.49 | 24314 – 24455 | 6674 / 7902 | 3.32 | 22257 | 0.95 |
| 10 | 6965 – 7137 / 7174 – 7198 | 3.49 – 3.50 | 24313 – 24974 | 6128 / 6914 | 3.20 | 19663 | 0.87 |
| 11 | 6648 – 6942 / 7184 – 7199 | 3.42 – 3.48 | 22755 – 24187 | 6005 / 6837 | 3.15 | 18968 | 0.88 |

The predicted power is the table's own `mean(F·v)` and equals the reply's
`avg_power_W` to within 10 W.

- **The two winch laws agree.** The plant flies `kv·sqrt(F)` exactly (10 m/s:
  0.0408 · sqrt(6128) = 3.19 m/s, flown 3.20). The prediction runs 2 – 3 %
  faster than its own `kv` (mean v / mean sqrt(F) = 0.0414 – 0.0418) at every wind
  speed, 8 m/s included, where the ratio is 0.99: a constant offset, not the gap.
- **From 9 m/s up the prediction sits in the upper saturation corner:** force
  pinned at `f_max` (max 7174 – 7199 N against 7200; crest factor ≈ 1.02 – 1.08)
  and speed at `v_sat` 3.5 m/s. The optimized kite would pull more than 7200 N,
  the winch curve caps it. At 8 m/s it does not reach the ceiling.
- **The plant stays on the law, below the corner,** with 13 % less force at 10 and
  11 m/s and 5 % less at 9 m/s. Under the law power goes with F^1.5, so
  0.87^1.5 = 0.81 and 0.95^1.5 = 0.93: the whole `power_ratio` gap (0.83, 0.84,
  0.94) is this force gap.
- **The depower is the reply's.** Converted reply depower 0.291 – 0.305 at
  10 m/s and 0.310 – 0.324 at 11 m/s, flown 0.297 and 0.316 (0.263 – 0.267 and
  0.266 at 8 m/s).

**Result.** Not the force rating (the plant peaks at 6914 N at 10 m/s) and not the
winch law (Step 2b's case: the speed fits the law, and the plant never reaches
the upper saturation it would test). At the same converted depower the plant's
kite pulls 13 % less force than AWETrim's at 0.29 – 0.32, and the same force at
0.265. The gap opens with the depower, which is the confound below: the depower
conversion (`AWETRIM_V3KITE_DEPOWER_OFFSET`, calibrated at 6 m/s at depower ≈
0.27) or the optimizer's force model at higher depower. Next: separate those
two.

## Step 2a — de-rate `f_max`

Send a de-rated `f_max` instead of `wc.f_high`. `winch_from_wc` already takes
`f_max` as a keyword for exactly this kind of override
(`fcs.first_lap_force_frac` uses it), so this is a settings change, not new
machinery.

Re-run 9, 10 and 11 m/s. Expect the ratio to close towards 1.0 with measured
power within a few percent of today's, and peak force to stay under 8400 N.

**Tried 2026-08-30, wind-speed-dependent de-rating — REFUTES the working
hypothesis at 10 m/s.** First attempt scaled the de-rating per wind speed by
the run's own measured crest factor (`8400/cf_force_ro`). At 10 m/s this
FAILED the pre-existing turn-radius feasibility gate at the starting length
before it could even be backed off enough to clear it, because de-rating
`f_max` also lowers the speed ceiling `kv*sqrt(f_max)` the optimizer plans
against, which reshapes the pattern enough to tighten its curvature margin at
150 m — a coupling the hypothesis did not anticipate. Backed off to the
largest value that cleared the gate, full run result:

| | f_max sent | predicted W | measured W | ratio | cf_force_ro | max_force_ro |
|---|---|---|---|---|---|---|
| archived (today) | 8000 N | 24561 | 20163 | 0.82 | 1.17 | 7634 N |
| de-rated | 7400 N | 24109 | 20100 | **0.83** | 1.23 | 8009 N |

Both predicted and measured power dropped by nearly the same amount, so the
ratio **did not move** (0.82 -> 0.83, still far outside 0.98..1.02) — and the
crest factor moved the WRONG way (1.17 -> 1.23), pushing peak force back up
to 8009 N despite the lower request. The plant is still under-delivering by
the same ~17-18 % whether the optimizer is asked for 8000 N or 7400 N. That
attempt's per-wind-speed machinery (`f_max_opt` column, `winch_f_max_opt`) has
been reverted from the codebase.

**Conclusion so far:** de-rating `f_max` per wind speed via the crest factor
is not the fix. The "optimizer overshoots the flyable peak, run gets detuned
by depower" story does not hold up under direct test — closing headroom on
the SENT ceiling did not close the ratio at that one setting. The gap may
live elsewhere, most likely the wind-dependent depower-offset error already
flagged in "Confound to control" below (calibrated at 6 m/s only, and the
ratio degrades monotonically away from there).

**2026-08-30 follow-up — fixed, non-wind-dependent `f_high_awe_trim`.** Added
`WCSettings.f_high_awe_trim` (`WinchControllers.jl`'s `src/wc_settings.jl`,
sourced locally — see the project's "Sourcing V3Kite" note, same mechanism),
set through this package's own `data/wc_settings.yaml`: a single force
ceiling [N] sent to AWETrim in place of `wc.f_high`, never into `wc.f_high`
itself (the runtime plant ceiling is untouched). `0.0` (the default) disables
it and leaves the request at the plain `f_high`, so existing runs are
unaffected until it is set. `winch_from_wc` (`src/awetrim_client.jl`)
resolves it into `f_max`'s default.

**Tried 2026-08-30 at 7600 N — REFUSED at the STARTUP gate before the run
could even begin.** Applying the de-rating to the startup solve reproduced
the same coupling as the crest-factor attempt above, worse: the reply's
curvature margin at the 150 m starting length came out 0.73 (path radius
3.6° against the kite's 4.9°), below `min_feasibility_margin = 0.82`, and
`check_startup_path`'s hard gate (then `examples/reelout_feasibility.jl`) aborted the run — `feas_start`
scores the pattern flown for the whole reel-out (phases 3/4), not phase 5,
which passed fine (margin 1.15) and was never the issue. The startup retry
loop (`startup_retries_max = 4`, corrected re-solves at the same length)
could not be relied on to fix this either — it only widens the turn-radius
ASK, which cannot compensate for a lowered SPEED ceiling reshaping the reply,
and the code's own history already has a case where four retries topped out
short of the gate (2026-08-27, best 0.794 against 0.82).

**Fix — scope the de-rating to re-optimization only.** `winch`/`winch_first_lap`
(the STARTUP solve, now `optimizer_conditions` in `src/opt_conditions.jl`) pin
`f_max` to the plain runtime ceiling (times `first_lap_force_frac` for
`winch_first_lap`), never `wc.f_high_awe_trim`: the curvature margin is
tightest at the untested starting length, before the run has flown a single
lap to prove the pattern out, so this is not a value to fly blind. A new
`winch_reopt`, built off `winch_from_wc`'s plain default (so it DOES pick up
`f_high_awe_trim` when set), feeds every re-optimization request from lap 2
on — by then the run has an installed, flying pattern that already cleared
the startup gates, so a re-optimization reply landing tighter is a measured
outcome to react to, not an unflown path the run aborts on before it starts.
Not swept: it was later set to 7700 and 7500 N, and is now equal to `f_high`
(7200 N), see "Status 2026-10-01".

Unlike the reverted per-wind-speed attempt, this value does not scale with
crest factor or wind speed at all — the same number is sent at every wind
speed, sidestepping the "moved the wrong way" coupling above. It has NOT been
swept against `power_ratio`, so no conclusion on it can be drawn yet — do not tune it and the
depower offset in the same run (see "Confound to control" below).

## Step 2b — match the upper saturation

Compare `calc_vro_soft`'s inversion against the server's `tension_curve` near
`f_high` at the betas actually sent, the way the `softminus_beta` mismatch at
the lower limit was found. Only worth doing if Step 1 points here.

## Confound to control

`AWETRIM_V3KITE_DEPOWER_OFFSET = 0.1010` was calibrated at **6 m/s only**, at
~0.14 of ratio per 0.010 of offset. The ratio is 1.00 at the calibration point
and degrades monotonically away from it, which a wind-dependent offset error of
~0.013 would also produce. Separate this from the force-rating story before
attributing the whole gap to either — do not tune the offset and `f_max` in the
same run.

Not a gap (corrected 2026-10-01): v10's reply asks for `rel_depower` 0.295 ..
0.301, and the run flies it. With `fly_opt_depower: true` (`data/traj_opt.yaml`,
since 2026-08-20, phase 3 included since 2026-08-27) phases 3 and 4 fly the
reply's depower, converted with `awetrim_depower_to_v3kite`
(`depower_command!` in `src/reelout_loop.jl`), not `depower_setpoint`; the flown
depower at 10 m/s, 0.299 in August and 0.297 today, is the reply's. An earlier
version of this paragraph said the run flies `depower_setpoint = 0.274`; that was
wrong. What remains open is the conversion itself, i.e. the offset above.

## Prior art — read before starting

- `oldplans/PlanInvestigate.md` — the ratio's DENOMINATOR is strongly sensitive
  to AWETrim's tether-diameter belief; predictor `E` is inert; and
  `power_ratio` must NOT be compared across settle-cache-key eras.
- `docs/fig8_tuning_log.md`, "10 m/s kv — power against the phase-4 force
  limit" (2026-08-27) — the kv sweep above, and the 8400 N rating as the
  binding limit at 10 m/s.
- `data/winch_kv_table.yaml` header — the shipped kv rows (0.0408 flat from 3
  to 12 m/s), the 2026-08-27 kv sweep at 10 m/s, and why its 0.039 row went back
  to 0.0408 on 2026-09-22 (a wider first lap spent the force headroom).
