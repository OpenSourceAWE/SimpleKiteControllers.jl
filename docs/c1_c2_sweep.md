# Turn-rate law versus parasitic wing drag

Findings of 2026-10-06: the turn-rate coefficients `c1` and `c2` identified at a
fixed, low depower for four values of the parasitic wing drag coefficient
(`wing_drag_coeff`, the extra CD spread over the 20 wing nodes, see
`apply_wing_drag!` in `examples/model_setup.jl`).

## Summary

- More wing drag lowers both coefficients: from CD 0 to 0.06, `c1` falls by 31 %
  (0.314 → 0.216) and `c2` by 31 % (2.54 → 1.75). The decrease flattens with
  drag: −0.043, −0.031 and −0.024 in `c1` per step of 0.02.
- The apparent wind speed of the fit windows falls from 44.6 to 37.6 m/s.
- Interpolated at CD 0.03, the drag the runs fly, the sweep gives `c1` ≈ 0.255
  and `c2` ≈ 2.06, against 0.254 and 1.97 in the table row at depower 0.25
  (`data/turn_rate_coeffs.yaml`, 2026-10-05): the identification reproduces.
- The total delay (dead time + lag) shortens with drag, 0.117 → 0.072 s. Its split
  is not identifiable: the lag improves the residual RMS by only 0–2 %, and at
  CD 0.06 the fit puts all of it into the dead time.
- At 5 m/s wind (see [At 5 m/s wind](#at-5-ms-wind)), `c1` is 4–9 % lower and
  falls with drag at the same rate; `c2` agrees above CD 0. The total delay is
  about 0.07 s longer. The kite does not hold the 30° elevation with much drag:
  the CD 0.04 row rests on one flight, and the CD 0.06 row on none.

## Conditions

The relay flights of `examples/build_turn_rate_table.jl` (`_fly_low_flights`), at
the table's conditions:

| | |
|---|---|
| Project | `system_reelout_maasvlakte.yaml` (kite `kite_settings_psm_kernel.yaml`) |
| Depower | 0.25 |
| Wind speed | 9.51 m/s |
| Tether length | 150 m |
| Start elevation, elevation hold | 30°, 30° |
| Time step, VSM interval | 1/90 s, 5 |
| Flights | steering amplitudes 0.075, 0.100, 0.125, 200 s each |

Per drag value, the joint fit (`joint_delay_lag_fit`) of the flights that flew the
full time; standard errors from 20 s blocks (`block_standard_errors`). The drag
was set by overriding `apply_wing_drag!` in the session; nothing was written to
the turn-rate table.

## Results

| Extra CD | steady flights | `c1` ± se | `c2` ± se | dead time [s] | lag [s] | total delay [s] | v_a [m/s] | RMS gain of lag |
|---|---|---|---|---|---|---|---|---|
| 0.00 | 2 of 3 | 0.3138 ± 0.0007 | 2.54 ± 0.07 | 0.039 | 0.078 | 0.117 | 44.6 | 2.0 % |
| 0.02 | 3 of 3 | 0.2708 ± 0.0006 | 2.19 ± 0.09 | 0.039 | 0.056 | 0.095 | 42.2 | 2.1 % |
| 0.04 | 3 of 3 | 0.2399 ± 0.0008 | 1.93 ± 0.10 | 0.039 | 0.044 | 0.083 | 39.5 | 0.6 % |
| 0.06 | 3 of 3 | 0.2161 ± 0.0005 | 1.75 ± 0.09 | 0.072 | 0.000 | 0.072 | 37.6 | 0.0 % |

At CD 0 the flight with the largest amplitude (0.125) sank below the 10° floor;
the row is the joint fit of the other two.

On the Cabauw project (`system_reelout_cabauw.yaml`) at CD 0, none of the three
flights stayed airborne (v_a ≈ 61 m/s, all below the floor within 21 s), so
`build_turn_rate_table.jl` now asserts that the Maasvlakte project is selected.

## At 5 m/s wind

The same sweep at a wind speed of 5 m/s (`_fly_low_flights(0.25; v_wind = 5.0)`),
all other conditions as above.

| Extra CD | steady flights | `c1` ± se | `c2` ± se | dead time [s] | lag [s] | total delay [s] | v_a [m/s] | RMS gain of lag |
|---|---|---|---|---|---|---|---|---|
| 0.00 | 2 of 3 | 0.3011 ± 0.0011 | 2.11 ± 0.08 | 0.117 | 0.078 | 0.195 | 22.4 | 2.4 % |
| 0.02 | 2 of 3 | 0.2600 ± 0.0056 | 2.15 ± 0.17 | 0.128 | 0.044 | 0.172 | 18.4 | 0.2 % |
| 0.04 | 1 of 3 | 0.2288 ± 0.0032 | 1.91 ± 0.13 | 0.139 | 0.011 | 0.150 | 17.3 | 0.0 % |
| 0.06 | 0 of 3 | (0.1965 ± 0.0165) | (1.80 ± 0.27) | 0.139 | 0.000 | 0.139 | 10.8 | 0.0 % |

- The flight with the smallest amplitude (0.075) sank below the 10° floor at
  every drag value, after 60–92 s; at 9.51 m/s it was the largest amplitude that
  sank, and only at CD 0.
- At CD 0.04 the 0.100 flight sank too, so the row is the fit of one flight.
- At CD 0.06 both shorter flights sank and the 0.125 flight ended with an error
  after 127 s. With no steady flight, the joint fit used all three descending
  windows (v_a 10.8 m/s); the row is not a valid identification.
- Against 9.51 m/s, `c1` is lower by 4 % (CD 0), 4 % (0.02) and 5 % (0.04),
  9 % at 0.06. `c2` is lower at CD 0 (2.11 against 2.54), and the same within one
  standard error at 0.02 and 0.04.
- The total delay is 0.07–0.08 s longer than at 9.51 m/s for CD 0 to 0.04, and
  the split into dead time and lag is again not identifiable (RMS gain ≤ 2.4 %).
