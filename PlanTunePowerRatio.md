# Tune the power ratio

At 6 m/s the power computed by the dynamic simulation and by AWETrim must agree.
Tune `AWETRIM_V3KITE_DEPOWER_OFFSET` in `examples/awetrim_client.jl` until
`power_ratio` lands in **0.995 .. 1.004**, i.e. the summary prints `1.00`.

## Why the current value is wrong

`0.099` was calibrated on 2026-08-26 while the wing weighed **6.2 kg**. Commit
`6443fbe` (2026-08-29) raised `kite.mass` in `data/settings_reelout_150m.yaml` to
**10.9926 kg**, matching AWETrim's LEI-V3 wing. Mass is never sent to the server —
there is no mass field anywhere in `examples/awetrim_client.jl` — so AWETrim was
already predicting an 11 kg wing, and the offset had silently absorbed the 4.8 kg
mismatch. Removing the mismatch invalidates the calibration, which is exactly the
re-measure condition the constant's own docstring states.

## Current state, measured

`output/archives/2026-08-29_220825` (6 m/s, no turbulence, mass 10.9926):

| quantity | value |
|---|---|
| measured reel-out power | 7506 W |
| predicted reel-out power | 7301 W |
| `power_ratio` | 1.0281 |
| residual (measured − predicted) | **+205 W** |

`power_ratio_free_speed` is 1.03 in the same run. That reference is an upper bound
over any winch in `[f_min, f_max]`, so a value above 1 cannot be explained by the
`k_v` law — the disagreement is real, and the depower offset is the right lever.

## Sensitivity, and what it demands of the constant

From the calibration table in the docstring:

| offset | meas − pred [W] |
|---|---|
| 0.105  | −578 |
| 0.100  | −68  |
| 0.0985 | +67  |
| 0.095  | +451 |

That is 1029 W over Δoffset 0.010, so **~100 W per 0.001**, and 0.14 of ratio per
**0.010**. The prose beside that table ("~0.14 ratio per 0.005", "~150 W per 0.001")
is ~1.5x too steep and must be corrected; its zero crossing of 0.0992 is consistent
with the table and is fine.

Consequences:

- Closing +205 W needs **Δoffset ≈ +0.0020**, so start at **0.1010**.
- The 0.995 .. 1.004 band is ±37 W, i.e. **±0.00036 in offset**. A 0.001 step moves
  the ratio by 1.4 %, nearly three times the whole band, so the constant must be
  written with **four decimals** — `0.1010`, not `0.101`.

## Repeatability, and what "passed" means

With `default_turbulence: 0.0` the run is essentially deterministic. Archived 6 m/s
runs at fixed settings: 8615 W seven times; 7445, 7445, 7445, 7445, **7419**; 8582,
8582, 8582, 8582, **8613**. Power reproduces to the watt, with occasional discrete
~0.35 % jumps when the optimizer answers with a different path.

The band is therefore achievable, but one such jump eats 70 % of it. **Accept only on
two consecutive passing runs**, and treat a single near-miss as a re-run, not as a
reason to move the offset again.

## Procedure

1. Set `AWETRIM_V3KITE_DEPOWER_OFFSET = 0.1010`.
2. Set the wind speed to 6.0 in `data/gui.yaml`.
3. `include("examples/simple_opt_reelout.jl")` in the live REPL; block on
   `output/last_run_done.txt` (~4-6 min per run).
4. Read `power` (`measured_W`, `predicted_W`, `ratio`) from the archive the marker
   names, not from `output/`.
5. Outside the band? One Newton step from the measured residual at 100 W per 0.001,
   keeping four decimals. Back to 3.
6. Inside the band? Repeat the run once. Two consecutive passes accept the value.

## On success

- Update the calibration table and the two wrong slope figures in the
  `AWETRIM_V3KITE_DEPOWER_OFFSET` docstring, noting the 11 kg wing.
- Fix `docs/steering_depower.md:253`, which still quotes the retired `0.107`.
- Add a dated entry to `docs/fig8_tuning_log.md`.
- The offset is one scalar shared by every wind speed, so 4, 5, 7 and 8 m/s move with
  it. Re-run the sweep and refresh `notebooks/overview.md`, which is stale for every
  wind speed since the mass change.
