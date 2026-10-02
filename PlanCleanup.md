# Plan: remaining duplicate code in examples/

Done on 2026-10-01: `build_controllers` in `simple_reelout.jl`, `try_turn_rate_coeffs`,
`run_input_files`/`archive_run_files`, `guidance_rate`, `muted`/`latest_global`,
`unwrap_angle`/`unwrap_onto`, `first_error_line` and `stack_fits` moved into `src/`.
The four copies of the `init(...)` call now use `init_model` in `examples/model_setup.jl` (formerly `opt_reelout_lib.jl`).

This file lists what is left. The package does not depend on V3Kite or GLMakie
(see `examples/model_setup.jl`), so anything that calls `init`, the viewer or
Makie can only move into a shared file in `examples/`, not into `src/`.

## 2. The simulation-loop tail (examples-side)

Repeated in `simple_fig8.jl`, `simple_opt_fig8.jl` and `simple_reelout.jl`
(and in the live twin):

- the `catch exc; @error "Simulation stopped early at t≈…"` block,
- `t_wall`/`t_sim`, `save_log(…; colmeta = timestamp_colmeta())`, `load_log`,
- the `v_wind_200m .= calc_wind_factor(s.am, 200.0) .* …` fill inside the loop.

A `finish_run(s, log_name, output_path, t_wall_start) -> (; sl, t_wall, t_sim)`
plus a `fill_v_wind_200m!(s)` would cover it. `calc_wind_factor` is from
AtmosphericModels, which `src/` already uses, so `fill_v_wind_200m!` could go into
the package if it takes `s.am` and `s.sys_state`.

## 3. Winch setup in `simple_reelout.jl` vs `build_winch`

`simple_reelout.jl` builds `dt0`, `wc`, `rcs` and `wpc` by hand, and does the
compliance checks too. `build_winch` (`src/winch_setup.jl`) does the same, **but it
also sets `kv`, `f_low` and `force_limit` from the wind-dependent tables**, while
`simple_reelout.jl` flies the file's flat values. Swapping it in therefore changes
the flight. Decide first whether `simple_reelout.jl` should use the tables;
only then replace the block.

## 4. Replay and video recording (examples-side)

`replay_run`/`record_video` in `simple_fig8_live.jl` and `replay`/`record_video` in
`simple_reelout_play.jl` are about 110 lines of near-identical code. Differences:

- frame selection: `VIEWER_INTERVAL*REPLAY_TIME_LAPSE` vs `VIEWER_INTERVAL`,
- status text: `maybe_update_status_text!` vs a local `TEXT_UPDATE_HZ` throttle,
- `log_dt` computed inside vs outside, and a `GC.gc()` in one `finally`.

Move both into `examples/viewer_replay.jl` with these as keywords; the
`on(viewer.btn_PLAY.clicks)` hook can go there too.

## 5. Time-series preparation for the plots (partly package)

`plot_time_series` in `plot_pattern_utils.jl` and the top level of
`simple_reelout_plots.jl` (≈60 identical 6-line windows) and
`simple_fig8_plots.jl` (≈19) derive the same series from a log: `psi`, `chi`,
`chiset`, `chi_u`, `err_course`, `err_heading`, `l_tether`, `v_reelout`, `v_set`,
`u_d`, `wc_state`, `state`, `fig8`.

- `simple_reelout_plots.jl` already includes `plot_pattern_utils.jl`; check
  whether its own copy of the time-series panel can simply call `plot_time_series`.
- The pure-array part (no Makie) could become `log_series(sl, rng)` in `src/`.

## 6. Smaller leftovers

- The `trim(x) = x[dmax + 1:end]` closure is still in V3Kite's `joint_delay_lag_fit`
  and `plot_turn_rate_identification.jl` for the shifted
  inputs `us = reduce(vcat, [trim(shift_delay(u, d)) for u in ufs])`.
- `col` is defined in `plot_relay_low_elevation.jl` and `plot_turn_rate_vs_depower.jl`
  (check whether they really do the same thing).
- `verdict` is in `stability_global.jl` and `validate_margins.jl` (likely different
  semantics; check before merging).

## Leave as is

- `simple_fig8_live.jl` vs `simple_fig8.jl`: a deliberate twin ("a diff against
  `simple_fig8.jl` should show nothing else"). Merging them needs hooks inside the
  loop, which is a bigger design change.
- The `Pkg.activate`/`using` header at the top of every script.

## Note for live sessions

A REPL that ran the old scripts still has `Main.unwrap_angle`, `Main.onto`,
`Main.muted`, `Main.latest`, `Main.first_line` and `Main.guidance_rate` defined.
These shadow the package's exports, and `Main.guidance_rate` has the old argument
order. Restart the session before flying the changed scripts.
