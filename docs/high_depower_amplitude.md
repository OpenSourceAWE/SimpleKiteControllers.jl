# Relay flights at high depower: elevation floor and steering amplitude

Findings of 2026-10-06: how to get more steady relay flights for the turn-rate
table at depower 0.325 – 0.40, where today 2, 2, 1 and 0 of the three flights of
`examples/build_turn_rate_table.jl` stay airborne. Nothing was written to
`data/turn_rate_coeffs.yaml`; the flights were driven from scratch scripts that call
`_fly_relay`, `_fit_window`, `joint_delay_lag_fit` and `block_standard_errors`.

## Summary

- Lowering the elevation floor from 10° to 5° gains nothing: every flight that
  sank below 10° also sank below 5°, and every steady flight bottomed out at
  12 – 13.5°. The fits are identical to the table (which also confirms that the
  table reproduces).
- The limit is turn authority, not the floor: the flight that fails is the
  smallest amplitude, 0.075 (and 0.10 at 0.375 and above).
- Replacing 0.075 by 0.15 from depower 0.325 on gives one more steady flight in every
  cell and a first cell at 0.40. 0.15 loops into the ground at 0.275, but `c1`
  is 40 % lower at 0.375, so there it turns like 0.09 does at 0.275.
- `c1` moves by less than 1 %; `c2` stays within its standard error, which shrinks.
- The near-zero lag of the table at 0.35 and 0.375 (0.011, 0.022 s) becomes
  0.056 s, as at low depower: it was an artefact of fitting one or two flights.
- A lower elevation hold (25° instead of 30°) was not flown; with the same weak
  turn at small amplitude it would likely make the failures worse.

## Floor 5° instead of 10°

Flights 0.075 / 0.10 / 0.125 as in `FLIGHT_SETTINGS`, `elevation_floor = 5.0`.
Minimum elevation of each flight; "sank" = below 5°.

| depower | 0.075 | 0.100 | 0.125 | steady, floor 10° | steady, floor 5° |
|---|---|---|---|---|---|
| 0.325 | sank | 13.5° | 12.3° | 2 | 2 |
| 0.350 | sank | 12.0° | 12.6° | 2 | 2 |
| 0.375 | sank | sank | 12.1° | 1 | 1 |
| 0.400 | sank | sank | sank | 0 | 0 |

## Amplitude 0.15

Floor 10°. Flights 0.10 (`az_reverse` 30°, `el_hold_tilt` 45°), 0.125 (30°, 25°),
and 0.15 (30°) at `el_hold_tilt` 25° and at 15°. All 0.15 flights flew the full 200 s.

Minimum elevation of the 0.15 flights, 25° / 15° tilt: 10.2° / 15.4° (0.325),
10.2° / 10.6° (0.35), 11.1° / 11.0° (0.375), 11.1° / 11.0° (0.40). The 0.10 flight
sank at 0.375 (41 s) and 0.40 (35 s), the 0.125 flight at 0.40 (45 s).

Joint fit, current flights → with the 0.15 flight at 15° tilt:

| depower | steady | `c1` | `c2` ± se | dead time [s] | lag [s] |
|---|---|---|---|---|---|
| 0.325 | 2 → 3 | 0.1875 → 0.1871 | 2.83 ± 0.12 → 2.75 ± 0.09 | 0.072 → 0.072 | 0.044 → 0.056 |
| 0.350 | 2 → 3 | 0.1626 → 0.1632 | 3.04 ± 0.07 → 3.00 ± 0.06 | 0.117 → 0.083 | 0.011 → 0.056 |
| 0.375 | 1 → 2 | 0.1424 → 0.1437 | 3.09 ± 0.07 → 3.12 ± 0.05 | 0.128 → 0.094 | 0.022 → 0.056 |
| 0.400 | 0 → 1 | — → 0.1292 | — → 3.23 ± 0.03 | — → 0.139 | — → 0.033 |

With 25° tilt the fits are nearly the same (`c2` 2.90, 2.98, 3.14, 3.26); 15° is
preferred for its margin at 0.325. Elsewhere both bottom out only 0.6 – 1.1° above the floor.

## Next steps

Steps 1 – 3 are done (2026-10-07: `flight_settings(depower)`, `FLIGHT_SETTINGS_HIGH`,
`HIGH_DEPOWER` in `examples/build_turn_rate_table.jl`; the cells 0.325 – 0.40 rebuilt in
11.5 min, all four `time_limit` with 3 / 3 / 2 / 1 steady flights and the coefficients of
the table above to the last digit). Step 4 is open.

1. Make `FLIGHT_SETTINGS` depend on the depower: 0.075 / 0.10 / 0.125 up to 0.30;
   0.10 / 0.125 / 0.15 (`az_reverse` 30°, `el_hold_tilt` 15°) from 0.325 on.
2. Add 0.40 to `TABLE_DEPOWERS`.
3. Rebuild the cells 0.325 – 0.40 (`run_example("build_turn_rate_table.jl";
   depowers = [0.325, 0.35, 0.375, 0.40])`).
4. Update the paper (LearningControl `main.tex`, Sect. `sec:turn_rate_id`): the
   coefficient table and its runs column, the amplitude sentence, the remark that
   the lag falls to 0.01 – 0.02 s above 0.325, and "from u_d = 0.40 on no run
   stays airborne" (also in the figure caption and the docstring of the script).
