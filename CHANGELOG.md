# Changelog

## Unreleased

### Changed

- The plant flies with wing drag: `wing_drag_coeff` 0.03 in both kite settings files.
  V3Kite's `init` reads the value but does not apply it; `apply_wing_drag!`
  (`examples/model_setup.jl`) spreads it over the wing nodes after settling, in
  `init_model` and in the relay flights of `build_turn_rate_table.jl`. 0.07 (the value
  of V3Kite's flight replays) and 0.05 needed more depower at high wind than the kite
  can fly; 0.03 flies all 22 scenarios, and the low-wind power ratio is now about 1.
- Re-identified with the wing drag, in order: the turn-rate table; the depower
  conversion (offset 0.1076, slope 0.1392, curvature 0.2427); the course-loop model
  (`kite_dead_time_exp` 1.459 → 1.225, `kite_lag_exp` 0.371 → 0.437, pattern delay
  0.144 s (34/v_a)^0.641 → 0.108 s (34/v_a)^0.932, `pattern_depower_exp` 5.98 → 4.47,
  kite correction re-measured at v_a 32 m/s); and the course link, re-measured by
  multisine injection at 150, 200 and 300 m. The kite responds about 25 % faster at
  high apparent wind, so `attractor_lead_time` stays at 1.05 s (a retune to 1.21 s on
  the old model was undone). All 22 scenarios pass all criteria; worst guided disk
  margin 0.506 (Maasvlakte 3.5 m/s).
- The turn-rate table flies relay amplitudes 0.10-0.15 from depower 0.325 on (0.075,
  which sank at every high depower, is dropped) and has a cell at 0.40:
  3/3/2/1 steady flights at 0.325-0.40, was 2/2/1/0. `build_turn_rate_table.jl`
  asserts that the Maasvlakte project is selected.
- The kite parks at the end of the reel-out: with `park_final` (on) phase 5 steers
  course 0, straight up, and `park_lead` (6 s) starts that climb before the reel-out
  ends. Without it the force jumped by about 35 % when the winch stopped, beyond what
  the depower limiter could hold at Cabauw 10 m/s. The pattern metrics, the reel-out
  power (`reelout_power`, `reelout_block` with `t_end`), the power ratio and the
  stability rating of `stability_opt_reelout.jl` end where the climb starts; the
  elevation floor, the force limit and `energy_run` still cover the whole run.
- The turn-radius request is sized at `request_depower_estimate`, a depower ramped on
  the wind at 100 m, instead of the depower of the solve's seed, which seeded
  Maasvlakte 8 m/s at 1.85 m of tape against 1.42-1.46 m flown.
- High-wind pattern box: `pattern_azimuth_max_high` 32 → 34° and
  `pattern_elevation_amplitude_max_high` 11 → 13°, which Cabauw 10 m/s needs with drag.
  At 36° the re-optimizations grew to ±32° and tracked worse.
- Free-speed reference in `reelout_results.jl` also for power ratios ≥ 1.05 (was 1.1)
  and minimum forces below `FREE_SPEED_FORCE_MAX` = 2000 N (was 1000 N).
- Font sizes of `plot_c1_c2.jl` and `plot_relay_low_elevation.jl` for the paper; the
  attractor-distance plot of `plots_extra.jl` runs to 150 s.
- The "pattern law" is called the pattern delay approximation, since it is an empirical
  fit, not a physical law: `identify_pattern_law.jl` is `identify_pattern_delay.jl`, the
  key `pattern_law_depower` of the course-loop model file is `pattern_delay_depower`, its
  provenance key `pattern_law` is `pattern_delay`, and the logs are kept in
  `output/pattern_delay/`.
- The low-wind schedule sets only the starting length and `guess_el_center`: the
  `low_wind_v_app_min` and `low_wind_el_offset_final` columns are gone, so Maasvlakte
  3.5 m/s flies `v_app_min` 10 m/s and `el_offset_final` 1.0° like every other run. With
  the gravity feed-forward it passes all criteria either way (2026-10-05): minimum
  elevation 9.2° (set in the entry; phase 5 9.47° with the 1.0° lift, 9.97° with 1.5°),
  guided disk margin 0.51, 962 W.
- `attractor_lead_time` of `fc_settings_reelout.yaml` is 1.05 s (was 0.96 s). At high
  wind the lead time sets the attractor arc, and with it the guidance corner `ω_g`; the
  longer lead lifts the guided disk margin of Cabauw 10 m/s from 0.48 to 0.53 and of
  7 m/s from 0.58 to 0.62, with a slightly lower RMS cross-track error and 1-1.5 % less
  mean reel-out power. Below about 5 m/s the arc stays at `attractor_dist` and nothing
  changes.
- The turn-rate table is identified at low elevation only: `build_turn_rate_table.jl`
  flies, per depower, the three crosswind relay flights at fixed amplitudes that
  `plot_turn_rate_identification.jl` flew (`_fly_low_flights`, elevation held near
  30°) and writes the joint fit of the steady ones, with block standard errors
  (`*_se`). The stepped relay sweep at 73°, the `c3` keyword and
  `add_delay_lag_split!` are gone. `_run_turn_rate_sweep` is now `_fly_relay`.
- `data/turn_rate_coeffs_low.yaml` is now `data/turn_rate_coeffs.yaml`, and every
  project names it. The 73° table and `data/turn_rate_coeffs_c3.yaml` are deleted;
  `system_reelout_180m.yaml`, the last project on the 73° table, now flies the
  low-elevation one.
- `plot_c1_c2.jl` draws the `c1_se`, `dead_time_se` and `kite_lag_se` bars.

### Added

- A low-wind schedule for `simple_opt_reelout.jl`: section `low_wind:` of
  `fc_settings_reelout.yaml` ([`FC_LowWind`], `low_wind_schedule`,
  `apply_low_wind_schedule!`) sets the starting tether length, the startup guess's
  `guess_el_center` per wind speed at 100 m height,
  linear in between. At 150 m the guided disk margin was marginal at Cabauw 3 m/s and
  Maasvlakte 3.5 and 4 m/s (0.41-0.44); starting at 195 m lifts it to 0.57-0.61 at
  equal or higher power. `stability_opt_reelout.jl` applies the schedule at the log's
  wind speed. An override of the same setting wins over the schedule.

- Moved from `examples/` into the package (V3Kite is still not a dependency):
  the AWETrim client, optimizer request/conditions, pattern limits, re-optimization
  gate, loop decisions, run setup/startup, `RunState` and the reel-out loop step,
  run summaries, GUI state, `run_example`/`script_inputs`.
- Helpers the examples each defined for themselves: `try_turn_rate_coeffs`,
  `run_input_files`/`archive_run_files`, `guidance_rate`, `muted`/`latest_global`,
  `unwrap_angle`/`unwrap_onto`, `first_error_line` and `stack_fits`.
  `simple_reelout.jl` now uses `build_controllers`.
- `examples/opt_reelout_lib.jl` is now `examples/model_setup.jl`; its `init_model`
  (new keywords `warmup_torque`, `pad_final_time`, `set_overrides` optional) replaces
  the copied `init(...)` call in `simple_fig8.jl`, `simple_fig8_live.jl`,
  `simple_opt_fig8.jl` and `simple_reelout.jl`.
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
- `examples/plot_path3d_paper.jl`: Fig. 14 of the LearningControl paper, the 3D view of
  the Cabauw 5.75 m/s run (`v05.75`), saved as `pattern_cabauw_5.8ms.png` in
  `../LearningControl/figures`. The path is cut at a height of 320 m, before the final
  climb towards the zenith.
- Model provenance: every identification step records the `kite_id` (a hash of
  `kite_fingerprint`: masses, body damping, geometry files, wing drag, bridle) of the
  kite it flew, step 1 per row of the turn-rate table and steps 2-5 in a `provenance:`
  section of `course_loop_model.yaml`. `check_model_provenance` compares them with the
  kite a project flies; `stability_opt_reelout.jl`, `stability_fig8.jl`,
  `stability_global.jl` and `retune_guided.jl` refuse a model of another kite, and
  step k refuses to start before steps 1..k-1 are done on the same kite.
- `reidentify-kite` skill (`.claude/skills/`): the re-identification after a change of
  the kite, in order, then validation, rating and retuning.
- `examples/measure_course_link.jl`: flies the multisine injection at 300, 200 and
  150 m, writes `data/course_link_measured.csv` and prints the delay and gain margins
  on the measured plant next to the model's.
- `examples/kite_model.jl`: the paper's figure of the structural discretisation of the
  kite (`kite_model.pdf`).
- `examples/plot_worst_margin_bode.jl`: re-analyses the scenario with the lowest guided
  disk margin and writes `worst_loop_bode.pdf`, with the phase and gain margins marked,
  and `worst_loop_disk_margin.pdf`, the disk margin over frequency, to
  `LearningControl/figures`. Needs MakieControlPlots 0.1.20.
- Notes in `docs/`: the wing-drag test and the low-wind tension with drag in
  `power_ratio_findings.md`, `c1_c2_sweep.md` and `high_depower_amplitude.md`.
- `presentation/Robust_kite_control_Fechner.odp` and its PDF.

### Removed

- `ParkingController`, `linearize` and `navigate`.
- Example helpers now in the package (`awetrim_client.jl`, `gui_state.jl`, `script_inputs.jl`, ...).
- `examples/fig8_log_meta.jl`, which nothing in the package used.

### Fixed

- Replays failed with `KeyError "reply"` when a re-identified plant put the archived
  startup path below `min_feasibility_margin`: the retry ladder asked for answers the
  archive does not hold. A replay now flies the archived path as installed.
- `plot_relay_low_elevation.jl` defined a global `time`, which hid `Base.time()` in the
  REPL and broke later `simple_fig8.jl` runs; it is `t_log` now.
- Broken `@ref` links on the internals page of the documentation.
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
