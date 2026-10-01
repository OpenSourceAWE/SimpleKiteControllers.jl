# Changelog

## Unreleased

### Added

- Moved from `examples/` into the package (V3Kite is still not a dependency):
  the AWETrim client, optimizer request/conditions, pattern limits, re-optimization
  gate, loop decisions, run setup/startup, `RunState` and the reel-out loop step,
  run summaries, GUI state, `run_example`/`script_inputs`.
- Helpers the examples each defined for themselves: `try_turn_rate_coeffs`,
  `run_input_files`/`archive_run_files`, `guidance_rate`, `muted`/`latest_global`,
  `unwrap_angle`/`unwrap_onto`, `first_error_line` and `stack_fits`.
  `simple_reelout.jl` now uses `build_controllers`.
- The linear course-loop model of the stability analysis, from `examples/course_loop_model.jl`:
  the pattern law, plant coefficients and margin helpers in `src/course_loop_model.jl`; the
  transfer functions (`course_pid`, `turn_rate_plant`, `delay_margin`, `guidance_tf`,
  `kite_correction`) in a package extension loaded with `using ControlSystemsBase`.
- `FigureEightController(fcs; dt, A, B)`, `apply_overrides!`, `turn_rate_coeffs(...; table)`.
- Turn-rate identification in a low crosswind pattern, while reeling out and over
  depower.
- Zenodo DOI badge.
- `data/fc_settings_fig8_150m.yaml`: the 150 m fig8 project's own controller settings,
  a copy of `fc_settings.yaml` with a taller pattern (`f8_b` 16°). The 200 m lemniscate
  is curvature-limited on the shorter tether; RMS cross-track error 4.36° → 0.82°.
- `wind_schedule` / `apply_wind_schedule!` and the `FC_Settings` fields `depower_high`,
  `f8_a_high`, `f8_b_high`, `wind_ramp_low`, `wind_ramp_high`: depower and pattern size
  ramped with the wind speed (off by default). `simple_fig8.jl` applies it; the fig8
  settings ramp from 7 to 10 m/s to depower 0.33 and a taller (150 m: also wider)
  pattern, so 10 m/s no longer stops on the overspeed guard. All three projects pass
  all 8 criteria at 8.5 and 10 m/s.

### Removed

- `ParkingController`, `linearize` and `navigate`.
- Example helpers now in the package (`awetrim_client.jl`, `gui_state.jl`, `script_inputs.jl`, ...).
- `examples/fig8_log_meta.jl`, which nothing in the package used.

### Fixed

- `DelayedInjection` in `validate_margins.jl` never switched on in `simple_opt_reelout.jl`.
- `simple_fig8.jl` flew whatever turn-rate table the session held, so a run after a
  reel-out script silently used the reel-out table; it now reloads its project's.
- `settings_fig8_150m.yaml`: `sample_freq` 90 → 100 Hz; at 90 Hz the run developed a
  growing 8 Hz oscillation in AoA, bridle pulley and tether force.
- `settings_fig8_300m.yaml`: `sim_time` 90 → 130 s; the run flew only 2.0 laps and
  failed `laps >= 2.5`.
- `simple_fig8.jl` crashed at 4 m/s: the kite flew into the ground during the entry
  and the solver failed later. `chi_dive` -85 → -145° (both fig8 settings files) lets
  4 m/s reach the pattern at 200 and 300 m, and the script now stops with an error on
  ground contact instead of a solver error. At 150 m the entry needs `chi_dive` -160°
  and `dive_el_margin` 25° (`fc_settings_fig8_150m.yaml`) to stay above 10°.

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
- `DAMPING_PER_STIFFNESS` raised from 0.001 s to 0.002 s in `simple_fig8.jl`,
  `simple_fig8_live.jl`, `simple_reelout.jl`, `simple_auto_parking.jl` and
  `simple_opt_fig8.jl`, matching `simple_opt_reelout.jl`.
- The fig8 projects (`system_fig8_{150,200,300}m.yaml`) use `turn_rate_coeffs_low.yaml`,
  like the reel-out projects; it is now also the table loaded at package start.
- Fig8 guidance retuned (`fc_settings.yaml`, 7 m/s): `attractor_dist` 8 → 5.5°, RMS
  cross-track error at 200 m 0.85° → 0.68°, at 300 m 0.84° → 0.71°.
- Fig8 entry: `dive_el_margin` 7 → 15° (also in `fc_settings_fig8_150m.yaml`). The dip
  below the pattern during the entry is gone; whole-run minimum elevation 15.8° → 18.7°
  at 200 m, 17.9° → 19.2° at 300 m, 11.0° → 16.5° at 150 m.
