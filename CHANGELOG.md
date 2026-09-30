# Changelog

## Unreleased

### Added

- Moved from `examples/` into the package (V3Kite is still not a dependency):
  the AWETrim client, optimizer request/conditions, pattern limits, re-optimization
  gate, loop decisions, run setup/startup, `RunState` and the reel-out loop step,
  run summaries, GUI state, `run_example`/`script_inputs`.
- `FigureEightController(fcs; dt, A, B)`, `apply_overrides!`, `turn_rate_coeffs(...; table)`.
- Turn-rate identification in a low crosswind pattern, while reeling out and over
  depower.
- Zenodo DOI badge.

### Removed

- `ParkingController`, `linearize` and `navigate`.
- Example helpers now in the package (`awetrim_client.jl`, `gui_state.jl`, `script_inputs.jl`, ...).

### Fixed

- `DelayedInjection` in `validate_margins.jl` never switched on in `simple_opt_reelout.jl`.

### Changed

- Reel-out guidance retuned for guided disk margin >= 0.3 (`fc_settings_reelout.yaml`):
  `attractor_lead_time` 0.96 s, `heading_p` 0.183805, `heading_d` 0.141 s,
  `ff_gain` 0.76, `entry_gain` 0.23. RMS cross-track error falls 11 - 22 %, power unchanged.
- `examples/simple_opt_reelout.jl` refactored into functions (globals 101 → 8); replays identical.
- `examples/simple_opt_reelout.jl`: the tether/bridle structural damping
  `damping_per_stiffness` (the keyword default of `init_model` in
  `examples/opt_reelout_lib.jl`) is now 0.002 s instead of 0.001 s. The flight
  barely changes: at Maasvlakte 8.25 and 3.5 m/s all 10 success criteria pass,
  cross-track RMS and mean reel-out power change by less than 2 %; the
  standard deviation of the tether force at 3.5 m/s rises by about 20 %. The
  simulation is faster: 2.77 → 2.19 ms per step at 8.25 m/s (1.26 x) and
  2.87 → 1.75 ms per step at 3.5 m/s (1.64 x), from 3 interleaved replays of
  each setting, spread within 3 %.
