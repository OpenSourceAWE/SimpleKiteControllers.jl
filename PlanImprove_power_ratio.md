# Improve power_ratio

`power_ratio` (measured / predicted mean reel-out power, against the weighted
prediction) is 1.00 up to 8 m/s and then falls away: **0.91 at 9, 0.82 at 10,
0.83 at 11 m/s**. This is not a 10 m/s problem; the pattern across 9/10/11 is
the evidence.

**Acceptance:** `power_ratio` in 0.98 .. 1.02 at 9, 10 and 11 m/s, with all
success criteria still passing and measured power not lower than today's. The
band is the one used in `oldplans/PlanTunePowerRatio.md`.

Measured power must not be traded away to close the ratio. If the ratio closes
because the PREDICTION comes down to what the plant can fly, that is the
intended outcome — see the hypothesis below.

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

## Step 1 — decisive, no simulation

Ask the server for the mean force and reel-out speed its reply predicts, at 8
and at 10 m/s.

- Predicted force approximately 7350 N at 10 m/s, consistent with
  `v = 0.039*sqrt(F)` -> the two winch laws agree, the cause is the force
  rating. Go to Step 2a.
- Predicted speed NOT consistent with that relation -> the original suspect is
  live after all, but in the UPPER saturation (`softplus_beta`, `f_max`,
  `v_max = v_sat` 3.5 m/s), not `kv`. Go to Step 2b.

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
unaffected until it is set. `winch_from_wc` (`examples/awetrim_client.jl`)
resolves it into `f_max`'s default.

**Tried 2026-08-30 at 7600 N — REFUSED at the STARTUP gate before the run
could even begin.** Applying the de-rating to the startup solve reproduced
the same coupling as the crest-factor attempt above, worse: the reply's
curvature margin at the 150 m starting length came out 0.73 (path radius
3.6° against the kite's 4.9°), below `min_feasibility_margin = 0.82`, and
`examples/reelout_feasibility.jl`'s hard gate aborted the run — `feas_start`
scores the pattern flown for the whole reel-out (phases 3/4), not phase 5,
which passed fine (margin 1.15) and was never the issue. The startup retry
loop (`startup_retries_max = 4`, corrected re-solves at the same length)
could not be relied on to fix this either — it only widens the turn-radius
ASK, which cannot compensate for a lowered SPEED ceiling reshaping the reply,
and the code's own history already has a case where four retries topped out
short of the gate (2026-08-27, best 0.794 against 0.82).

**Fix — scope the de-rating to re-optimization only.** `winch`/`winch_first_lap`
(the STARTUP solve, `examples/simple_opt_reelout.jl`) now explicitly pin
`f_max = F_HIGH_NOMINAL`, never `wc.f_high_awe_trim`: the curvature margin is
tightest at the untested starting length, before the run has flown a single
lap to prove the pattern out, so this is not a value to fly blind. A new
`winch_reopt`, built off `winch_from_wc`'s plain default (so it DOES pick up
`f_high_awe_trim` when set), feeds every re-optimization request from lap 2
on — by then the run has an installed, flying pattern that already cleared
the startup gates, so a re-optimization reply landing tighter is a measured
outcome to react to, not an unflown path the run aborts on before it starts.
Still untested at any nonzero value — the 9/10/11 m/s rows remain unswept.

Unlike the reverted per-wind-speed attempt, this value does not scale with
crest factor or wind speed at all — the same number is sent at every wind
speed, sidestepping the "moved the wrong way" coupling above. It has NOT yet
been swept: the 10/9/11 m/s rows above should NOT be treated as tuned, and
this needs its own sweep before drawing a conclusion — do not tune it and the
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

Related: v10's reply asks for `rel_depower` 0.295 .. 0.301 while the run flies
`depower_setpoint = 0.274`. `DepowerReply`'s docstring is explicit — "FLY THIS,
or the reported metrics are not achievable". Establish whether that gap is
intended before reading any ratio at 10 m/s as a model-vs-plant statement.

## Prior art — read before starting

- `oldplans/PlanInvestigate.md` — the ratio's DENOMINATOR is strongly sensitive
  to AWETrim's tether-diameter belief; predictor `E` is inert; and
  `power_ratio` must NOT be compared across settle-cache-key eras.
- `docs/fig8_tuning_log.md`, "10 m/s kv — power against the phase-4 force
  limit" (2026-08-27) — the kv sweep above, and the 8400 N rating as the
  binding limit at 10 m/s.
- `data/winch_kv_table.yaml` header — the shipped kv rows and why 10 m/s is
  0.039.
