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

## Experiments — cheapest first

### E0 · Provenance audit (no compute, do this first)

Establish per run what the plant and the predictor knew:

```bash
grep -rH "d_tether\|e_tether" ~/repos/SimulationResults/scenarios/*/settings_reelout_*.yaml
stat -c '%y %n' ~/repos/AWETrim/data/LEI-V3-KITE/system.yaml   # vs run dates:
grep -H "date:\|time:\|power_ratio" ~/repos/SimulationResults/scenarios/*/reelout_150m_opt.yaml
```

Record one line per run: plant d · predictor d/E (reconstructed from AWETrim
git log + file mtime + server start time) · ratio. Whether "ratio was 1.0
before" means 4-vs-4 or 4-vs-10 changes every prior below. Also determine HOW
the server picks up `system.yaml` (startup cache vs per-request); if it caches,
every model edit needs a server restart and some past comparisons are void.

### E1 · Split the deficit into force vs speed (existing logs)

Power ratio = force ratio × speed ratio; summaries carry `av_force`,
`av_power_ro`, and the per-installation predicted-W timeline:

```bash
grep -E "av_force|av_power_ro|predicted_|installed" output/reelout_150m_opt.yaml
```

Force-dominant deficit → H1 (or capped-force artifact, H4).
Speed/tracking deficit → H2. Flat offset → winch side (C3/H4).

### E2 · Diameter-only revert, one run at 6 m/s (decisive)

Set BOTH sides' diameter back to 4 mm — plant `data/settings_reelout_150m.yaml`
and `-180m`; AWETrim `system.yaml` DIAMETER ONLY, keeping E at 55e9 — restart
the server, `clear_opt_failures()`, rerun 6 m/s.

- Ratio back ≈ 1.0 → C1/C2 confirmed; continue with E1-style localization to
  name the mechanism (H1 vs H2).
- Ratio stays ≈ 0.93 → diameter innocent; bisect C3, C4 and whatever E0 found
  about the server, one at a time, still at 6 m/s.

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