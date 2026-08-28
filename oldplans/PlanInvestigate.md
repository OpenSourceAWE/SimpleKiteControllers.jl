# Investigate why power_ratio fell to ~0.93

`power_ratio` (measured / predicted mean reel-out power, against the weighted
prediction, `examples/reelout_results.jl`) fell from ~1.00 to 0.92–0.93 at
6 m/s right after switching to an 8 mm tether. Three model changes landed in
the same window; this plan separates them.

## Observed

| run | wind | ratio | source |
|-----|------|-------|--------|
| v06 | 6 m/s | 1.00 | `SimulationResults/scenarios/v06/reelout_150m_opt.yaml` |
| v07 / v08 | 7 / 8 m/s | 0.99 / 0.98 | idem |
| latest 6 m/s run | 6 m/s | 0.93 | `output/reelout_150m_opt.yaml` |
| v09 … v12 | 9–12 m/s | 0.89 … 0.93 | `SimulationResults/scenarios/v*/…`, **dates unverified** |

At 6 m/s nothing else is suspicious: the winch `kv` change only touches
≥ 10 m/s rows. The 6 m/s drop is the clean signal; treat v09–v12 as a second,
possibly different, question until attributed.

## What changed at once (the confounders)

| # | change | file(s) | state | affects |
|---|--------|---------|-------|---------|
| C1 | tether d 4 → 8 mm (plant side) | `data/settings_reelout_{150,180}m.yaml` — reaches the plant because `struc_geometry.yaml` leaves `diameter_mm: .nan → use set.d_tether` | committed `0890cef` | measured power |
| C2 | tether d 4 → 8 mm AND E 850e6 → 55e9 (predictor side) | `/home/ufechner/repos/AWETrim/data/LEI-V3-KITE/system.yaml` | **uncommitted**, edited today | predicted power |
| C3 | `v_sat` 3.8 → 3.5 m/s (`v_ro_max` likewise in both settings yamls) | `data/wc_settings.yaml` | committed `79539dd` | measured power |
| C4 | `kv` flattened to 0.0408 at v_wind 10/11/12 (was 0.0558/0.0738/0.0812) | `data/winch_kv_table.yaml` | committed today | ≥ 10 m/s only |

Predictor tether history matters: AWETrim went 10 mm → 4 mm on 2026-08-18
(`1e27e66`). Which tether config each earlier run's PREDICTION saw must be
established, not assumed — see E0.

## Low prior, do not start here

**Stiffness mismatch (the E part of C2).** During the whole ratio ≈ 1.0 era
the plant ran E = 55 GPa (`e_tether` in the settings yamls, Ruppert) while the
predictor still had 850 MPa — a 65× gap that did NOT hurt the ratio.
Quasi-steady trim barely depends on E. The uncommitted AWETrim edit mixes two
experiments; if C2 must be bisected, revert its E part first (its own comment
says: back to 850e6 if this breaks something else).

## Hypotheses

- **H1 — Thick-tether physics diverges between the models.** Drag ∝ d,
  mass/metre ∝ d². The plant integrates distributed apparent-wind drag on
  tether particles that MOVE radially outward at up to v_sat during reel-out;
  the ROM treats the tether quasi-steadily. At 8 mm the neglected terms get
  big enough to cost ~7 %. Signature: deficit grows along the reel-out (∝ L)
  and shows up as a FORCE deficit, not a speed deficit.
- **H2 — Flown path no longer matches the optimized one.** The heavier,
  laggier system turns worse than `turn_rate_coeffs(body_damping)` /
  `min_turn_radius` assume, so the flown pattern is slower or wider than the
  reply the prediction was computed for. Signature: shrinking azimuth
  amplitude, growing phase lag, deficits near the lobe shoulders;
  `fig8_metrics` drifts.
- **H3 — Predictor served a different tether than the plant flew.** If the
  server caches its config from startup (or was started before an edit), some
  runs compared an 8 mm plant against a 4 mm or 10 mm prediction.
- **H4 — High-wind cluster is really C4/C3, not diameter.** ≥ 10 m/s runs got
  a flattened `kv` and a lower v_sat inside a soft force limit whose authority
  scales as `k_v*sqrt(T)`; that reshapes flight and prediction independently
  of the tether. Only relevant once E0 dates v09–v12.

## Findings from E0/E1 (2026-08-26 evening) — premise corrected

**E0 · provenance.** All scenario folders carry their settings; together with
file mtimes and AWETrim's git history (`a745914` 18:54 committed what had been
edited at 18:01; `1e27e66` had set the predictor to 4 mm on 08-18):

| run | finished | plant | predictor | ratio |
|-----|----------|-------|-----------|-------|
| v03…v05 | 08-24 | 4 mm | 4 mm @ 850 MPa (server predates 17:36 restart too) | 1.44 / 1.02 / 1.01 |
| **v06** | **08-26 17:34** | **8 mm @ 55 GPa** | 4 mm @ 850 MPa | **1.00** |
| v07…v12 | 08-26 morning | 4 mm | 4 mm @ 850 MPa | 0.98 … 0.93 |
| latest | 08-26 18:14 (started ~18:13, after the 18:01 edit) | 8 mm @ 55 GPa | **8 mm @ 55 GPa** | 0.93 |

The server re-reads `system.yaml` per `/init` (`session.py:init()` →
`create_system_model_from_yaml`; no cache). Plant settings byte-identical
between v06 and latest (`diff` verified). **An 8 mm PLANT alone never hurt the
ratio** — v06 scored 1.00 against an honest-but-wrong predictor.

**E1 · force/speed split** (v06 vs latest, both 6 m/s):
`v_ro_av` 2.41 vs 2.42 m/s, depower 0.272 vs 0.274 — identical flight speeds;
`av_force_ro` 3692 → 2739 N (−26 %); measured_W 8683 → 6509; weighted
predicted_W 8669 → 7032. Two stacked effects:

1. **Optimizer conservatism (dominant).** Told the truth about the tether,
   AWETrim returns structurally weaker paths (startup solve predicted 7458 →
   6282 W). Less powerful paths → less harvested power. This is most of the
   absolute −25 %.
2. **Residual flight/planner gap (~7 %).** Even against its OWN new
   prediction the plant delivers only 0.93. Plausible: solutions near binding
   constraints (clearance/turn radius/force cap) are more sensitive to
   ROM-vs-plant mismatch than v06's loose regime.

Consequence for E2: the original "revert both sides' diameter" test is moot —
the plant side is proven innocent. Replaced by:

### E2' · Reproduce the 18:14 run (R1) — DONE, reproduced

Plant untouched, predictor at HEAD (8 mm @ 55 GPa), SHOW_PLOTS=false, via
kaimon `ex` (job archive `output/archives/2026-08-26_191245`, finished 19:12):

| | v06 (17:34, old predictor) | 18:14 | **R1 19:12** |
|---|---|---|---|
| predicted_W | 8669 | 7032 | 7316 |
| measured_W | 8683 | 6509 | **6738** |
| ratio | 1.00 | 0.93 | **0.92** |
| av_force_ro | 3692 N | 2739 N | 3109 N |

The 0.92–0.93 regime reproduces → real effect, not solver noise. Verdicts:

1. **Absolute power drop (−22 %)** is attributed: telling AWETrim about the
   true tether makes it return weaker paths (~7000–7300 W vs 8700–9300 W
   predicted per installation). Both under-HEAD runs sit there; v06 with the
   oblivious predictor does not.
2. **The residual ratio 0.92–0.93 is a genuine planner/flight gap**, new with
   the honest predictor: R1's replies flew with margins down to 0.85
   (`margin_final_flown` 0.64!) where v06's solutions had slack (1.0+). The
   gap concentrates in paths solved near binding constraints.

Next runs (each one variable, 6 m/s): R2 diameter-only revert (8→4 mm,
E kept), then R3 E-only revert (55 GPa→850 MPa, d kept). Watch: which variant
restores powerful solves AND/OR closes the residual gap — they are separable.

### E3 · The 2×2 factorial (predictor diameter × E), plant always 8 mm @ 55 GPa

Factory-farmed caveat that fell out of E0: the AWETrim server was restarted at
17:36:18, two minutes AFTER v06 finished — v06 used an older server instance.
Server code last changed 25-08 23:50 (`254dd9e`, WinchParams softplus betas),
so both instances *should* run identical code; yaml is re-read per `/init`.

| run | predictor d | predictor E | predicted_W | measured_W | ratio |
|-----|-------------|-------------|-------------|------------|-------|
| v06 | 4 mm | 850 MPa | 8669 | 8683 | 1.00 |
| R1 | 8 mm | 55 GPa | 7316 | 6738 | 0.92 |
| R2 | 4 mm | 55 GPa | 8670 | 6782 | **0.78** |
| R3 | 8 mm | 850 MPa | — running | — | — |

All four flew byte-identical plant + controller settings (verified by diff of
the archives against the current working tree; the v_sat 3.5 commit at 18:36
merely recorded a value v06 already flew).

**Reading so far.** v06 vs R2 isolates the E effect at 4 mm: predictions are
indistinguishable (8669 vs 8670; startup solves 7458 vs 7457) yet delivered
power collapses 8683 → 6782 W when ONLY the predictor's stiffness goes honest.
The naive-E predictor somehow lets the plant harvest ~1.9 kW more with the
same flight speeds (v_ro_av 2.41 vs 2.2). Diameter's effect is the opposite
shape: it moves the PREDICTION strongly (R1 vs R2: 7316 vs 8670) while
measured stays flat (~6750) — the optimizer, told about 8 mm, returns paths
that match what the plant can actually deliver.

Working interpretation (R3 tests it): ratio ≈ 1.0 in v06 was a coincidence of
TWO wrong things cancelling — an optimistic tether model whose "wrongness" the
plant could cash in (E modest → solver less conservative → more powerful
installed patterns → more harvested power), not a validated predictor.

### E3 RESULT (R3 done 19:24) — the factorial, complete

| run | predictor d | predictor E | predicted_W | measured_W | ratio |
|-----|-------------|-------------|-------------|------------|-------|
| v06 | 4 mm | 850 MPa | 8669 | 8683 | 1.00 |
| R1 | 8 mm | 55 GPa | 7316 | 6738 | 0.92 |
| R2 | 4 mm | 55 GPa | 8670 | 6782 | 0.78 |
| **R3** | **8 mm** | **850 MPa** | **7316** | **6738** | **0.92** |

**R3 ≡ R1 bit-for-bit** (identical replies, identical 6738 W, identical
per-install timeline): the ROM's answers are INSENSIBLE to E — and R1 vs R2
(different d!) flew within 0.15 % of each other (matched-window: F 3130 vs
3132 N), so the optimizer's diameter claim moves the PREDICTION, not the
flight. Conclusions:

1. **E in `system.yaml` is inert for path power.** Keep HEAD's honest value;
   neither number can represent the plant anyway (see segmentation below).
2. **The measured power is pinned at ~6750 ± 45 W** across all three reruns.
   The whole visible spread in power_ratio (0.92 ↔ 0.78) is the DENOMINATOR:
   what AWETrim *claims* depends strongly on its diameter (drag) belief.
3. **v06's 1.00 stands alone as an unexplained outlier.** Log forensics
   (matched L∈(180,330) m outward samples, phase 4, constant 6 m/s wind):
   same pattern shape (az_sd 0.156 vs 0.155), same wind at kite altitude
   (7.51 vs 7.50 m/s), same heights (z̄ 71 m), same AoA (0.048 rad) — but its
   kite flew 9 % FASTER along the loop (21.78 vs 19.94 m/s → +8 % v_app →
   +18 % force → +28 % instantaneous power, exactly cubic/square consistent).
   Configs byte-identical, both repos clean, no turbulence. Remaining
   suspects are process-level state (sysimage/JIT/solver instance), NOT yaml.

## Segmented-tether stiffness (plant physics caveat)

The plant tether is a series chain (6 main-line segments + bridle lines +
winch transmission); end-to-end radial stiffness sits FAR below monolithic
EA/L at 55 GPa — and below EA/L at 850 MPa too. Neither single-E value in the
predictor can represent that chain. The experiments above render this mostly
MOOT for path-power prediction (proven E-insensitive twice), but two
consequences stand:

- Do NOT treat predictor E=55 GPa as "calibrated". It is decoration. If any
  server-side use of E ever becomes load-bearing (radial dynamics, resonance,
  settling metrics like zeta/ring_time), it must be replaced by an EFFECTIVE
  end-to-end stiffness fitted against plant ring-down data, not a material
  modulus.
- The honest diagnostic already exists: summaries log zeta (0.052–0.073) and
  ring_time. Those numbers ARE sensitive to compliance; track them when the
  tether model changes.

## Status / next actions

Attribution delivered: the regression decomposes into (a) prediction-side
diameter honesty lowering the claimed optimum (dominant, well understood),
(b) a real ~7 % planner/flight gap whenever solutions sit near binding
constraints (margins < 1.0), and (c) one unexplained process-level outlier
(v06) that must be reproduced-or-refuted before ANY further ratio claims use
it as reference. Next:

- R4 (cheap, do first): rerun once more in THIS kaimon session expecting ~6750
  W again; then run once from a fresh REPL/sysimage (`bin/run_julia`) to test
  whether v06-class speed (+9 % loop velocity) reappears. If environment
  splits results, chase sysimage/threading/precompile state — not yaml.
- Record findings in docs/fig8_tuning_log.md (dated entry) after R4 settles
  the environment question.
- Experiments left NO residue: AWETrim restored to HEAD (clean tree);
  failure-cache backups kept as output/opt_failure_cache.yaml.bak-R{1,2}.

## R4 RESULT (2026-08-26 evening) — environment hypothesis dead, real cause found

R4a (same kaimon session): 6738 W, ratio 0.92. R4b (fresh `julia --project`
process via shell, cleared failure cache): 6738 W, ratio 0.92. Five runs now
bit-identical across two processes → the plant is fully deterministic given
(loaded state + inputs). Sampling note: archived scenario logs are resampled
to 30 Hz by the archiving script; all runs flew 90 Hz (user). The earlier
"sample rate difference" lead was withdrawn accordingly; matched-window means
are unaffected by resampling.

But R2 vs v06 closes the contradiction: SAME nominal predictor config produced
near-identical reply timelines (startup 7458 vs 7457 W; installs 8240/8680/
8940/9102/9219/9335 vs 8099/8599/8890/9066/9194/9319 — same schedule, ≤1 %
off), yet v06 harvested 8683 W while R2 harvested 6782 W. Deterministic plant
+ identical yaml + identical replies ⇒ an INPUT outside the archives differed.
Found it in V3Kite's depot scratchspace
(`~/.julia/scratchspaces/4caac9c8-…/v3kite_cache/`):

- created 26-08 **17:50** (after v06!): settled state keyed
  `…vapp6.0_lt150_el73_…_syssystem_reelout_150m_kcu84_bd…dps1.5_bb…md0-0-32…`
- all v06-era entries (24-08): keyed `…vapp9.51_…syssystem_reelout_kcu84…md0-0-20…`,
  i.e. an OLDER key schema (project name without `_150m`, other mass/depower-
  seed components).

So v06 flew on a settled-wing state from the OLD cache-key era; every rerun
re-settled under the NEW schema (the 17:50 file) and starts from a different
wing geometry/state. Same code, same yaml, different cached initial condition
→ deterministically different flight (+28 % power for the old state at
6 m/s). This also reframes the "0.92 gap": part of it may be the new settled
state being worse-matched to the optimizer's assumptions than the old one —
NOT necessarily ROM-vs-plant physics.

Consequences:

1. Do NOT compare power_ratio across cache-schema eras. v06-class numbers are
   not reproducible until the settled state is re-created under the OLD key or
   a fresh benchmark protocol fixes the settle inputs explicitly.
2. Actionable fix (methodological, no code change): include the ACTIVE settle-
   cache filename in each run summary (one line), so this can never happen
   silently again. Candidate owner: V3Kite `init` / `settled_state_path`.
3. Optional experiment if v06-class power should be recovered: temporarily
   rename/copy the old-era settled `.arrow` back onto the expected old key and
   rerun once — prediction: measured power returns toward ~8700 W.
   Otherwise archive this investigation; the ratio question itself is CLOSED:
   honest-diameter prediction explains the drop; E is inert; residual gap is
   partly settle-state-conditioned.

## AoA hypothesis TESTED AND REJECTED (2026-08-26 late evening)

Hypothesis: the heavier (8 mm) tether changes wing AoA, explaining v06 vs R.
Test: matched-window (L ∈ 180–330 m, phase 4) comparison of BOTH logged AoA
fields (`AoA` = VSM centre panel [rad], `var_09` = span mean [deg], per
examples/simple_reelout_plots.jl and simple_fig8_plots.jl):

| | v06 | R1 |
|---|---|---|
| centre AoA | 2.761° | 2.749° |
| span-mean AoA | 4.581° | 4.591° |

Differences ≤ 0.012° — noise. Needed for the observed +18 % force would be
~+1.5…4° given the unchanged loop geometry. Also note `alpha3/alpha4/CL2/CD2`
are all-zero in these logs (not populated by simple_opt_reelout.jl's logger).
Conclusion: mass-induced AoA shift is NOT the mechanism; consistent with the
settle-cache finding (v06 flew on a now-deleted old-key settled state).
Note: ALL settled/model caches were deleted 26-08 ~20:40 (bin/
delete_cache_files extended to match settled_*.arrow first); v06 recovery via
old cache files is therefore impossible; treat cross-era ratio comparisons as
void. Clean-cache validation earlier (20:37 run): 6730 W, ratio 0.92 — sixth
reproduction, rebuild changes nothing.

## Depower-offset calibration (2026-08-26 night) — ratio 1.0 RESTORED

User insight confirmed: the offset is a steep, linear lever on measured power
(even though the two models' AoA agree — see above). `AWETRIM_V3KITE_DEPOWER_OFFSET`
was swept by rerunning simple_opt_reelout.jl at 6 m/s, one run per value:

| offset | flown u_p (mean) | measured W | predicted W | ratio |
|--------|------------------|------------|-------------|-------|
| 0.105 (old) | 0.274 | 6738 | 7316 | 0.92 |
| 0.100 | 0.268 | 7225 | 7293 | 0.99 |
| 0.0985 | 0.266 | 7373 | 7306 | 1.01 |
| 0.095  | 0.263 | 7746 | 7295 | 1.06 |

Linear (~0.14 ratio per 0.005 offset; ~+150 W measured per −0.001); zero
crossing at offset ≈ 0.0992. NEW CALIBRATED VALUE: **0.099**. Note this is a
single-wind, single-run calibration at 6 m/s with the 8 mm tether and
predictor at AWETrim `a745914` — re-measure after any change to either
model's depower axis or aerodynamics, and expect a wind dependence not yet
mapped. Prediction stayed ~7300 W across all four runs while measured swept
~1000 W — exactly the behaviour of a pure FLIGHT-SIDE depower trim.

This reframes the residual "planner/flight gap" of ~7 % recorded above:
most of it was simply the stale post-8 mm offset. The 18:14-era conclusion
("honest predictor returns weaker paths") stands for predicted power
(8669 -> 7316), but the flight now meets the prediction to within 1 %.

- R2: predictor DIAMETER-only revert (8 → 4 mm, keep 55 GPa): isolates the
  drag/mass term's effect on path quality vs prediction accuracy.
- R3: predictor E-only revert (55 GPa → 850 MPa, keep 8 mm): isolates the
  stiffness term (EA/L up ×259 total between HEAD and HEAD~1!).

## Guardrails

- One variable per run; 6 m/s stays the reference wind while untouched by C4.
  A moved wind point rides the depower-seed ramp (`input_depower_per_wind`) —
  re-check seeds after changing winds.
- Restart the AWETrim server after ANY of its `data/**.yaml` edits, and run
  `clear_opt_failures()` after model changes: failures are cached EXACT-keyed
  while feasibility moves with the model.
- Changing tether parameters legitimately invalidates settled-wing caches
  (`examples/cache/settled_*.bin` and V3Kite's model binaries) — expect one
  slow rebuild, and confirm which cache key fired before trusting a "fast" run.

## Outcome

Findings land in `docs/fig8_tuning_log.md` as a dated entry once attributed;
update this file meanwhile. If H1 survives: quantify the missing term and pick
between correcting the ROM's tether treatment and calibrating an effective
`cd_tether` for the 8 mm line — whichever keeps one source of truth for the
tether math on each side (AWETrim AGENTS.md's profile_laws rule, applied here).