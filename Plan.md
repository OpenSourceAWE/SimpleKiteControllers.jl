# Implement reel-out example

Write an example `simple_reelout.jl` that extends `simple_fig8.jl` by adding reel-out of
the tether. The run starts exactly like the current example (park -> dive -> hold ->
fig8), begins reeling out once the pattern is established, and stops reeling out when a
maximum tether length is reached — then flies on at constant length until `sim_time`.

**No pumping cycle.** Reel-in, depowered return and cycle-averaged power are out of scope; what is built here must not preclude them, but nothing in this plan implements them.

## Decisions taken

- **The REEL_OUT law goes into WinchControllers.jl**, not into `src/` here and not into
  V3Kite. This package keeps only the run's tuning (`FC_Settings`) and the example.
- **The run starts at 150 m and lasts 150-300 s.** The simulation is about twice as fast
  as realtime, so wall time is about half the sim time — a 300 s run costs ~150 s. The
  run is therefore sized by what the pay-out needs, not by cost.
- **REEL_OUT is not a fourth `WinchControllerState`.** The three existing states
  (`wcsLowerForceLimit`, `wcsSpeedControl`, `wcsUpperForceLimit`) name which
  sub-controller the 3-channel mixer has selected; they are an output, not a user
  choice. REEL_OUT selects the *law that produces `v_set_in`* for the speed controller,
  i.e. which branch `calc_vro` takes, and the two force limiters keep working around it.
- **The law already exists**, gated behind `wcs.test`:

  ```julia
  function calc_vro(wcs::WCSettings, force)
      if wcs.test
          return sqrt(force) * wcs.kv          # <- the REEL_OUT law, k_v = wcs.kv
      else
          ... wcs.vf_max * sqrt((force - f_low)/(f_high - f_low))  # signed, reels in below f_low
      end
  end
  ```

  So step one is mostly *promoting a test flag to a named mode*, not writing new control
  code. `wcs.kv` (default 0.06) is `k_v`; do not add a second parameter for it.

## Step 1 — REEL_OUT mode in WinchControllers.jl

In `src/wc_settings.jl` and `src/wc_components.jl`:

1. Add a mode field to `WCSettings` selecting the `v_set_in` law:
   `vroReelOut` = `kv * sqrt(force)`, `vroPiecewise` = today's `vf_max`/`f_low`/`f_high`
   law. Default `vroPiecewise` — the operational behaviour must not change.
2. `calc_vro` dispatches on the mode. Keep `wcs.test == true` selecting `vroReelOut`, so
   the existing tests and `examples/test_winchcontroller.jl` keep passing unchanged;
   note in the docstring that `test` is now a synonym and the mode field is preferred.
3. `WCSettings` is loaded from YAML through `StructTypes.constructfrom!`, which will not
   turn a YAML string into a Julia `@enum` by itself. Either add the `StructTypes`
   mapping for the enum or make the field a `Symbol`; whichever is chosen, cover a YAML
   round-trip in `test/test_wc_settings.jl`.
4. Docstring of `calc_vro`: state that the square-root law is unsigned — it never
   commands reel-in, and it is the `LowerForceController` that takes over below `f_low`
   and pulls in at `v_sw`. That hand-over is the mode's reel-in behaviour, not a gap.

`calc_v_set`, the mixer, the speed controller and both force controllers are untouched.

## Step 2 — `examples/test_reelout.jl` in WinchControllers.jl

Modelled on the existing `examples/test_winchcontroller.jl`, which already has every
piece: `get_triangle_wind`, `get_startup`, the `Winch` plant model, `WCLogger`,
`calc_v_set` + `on_timer`.

- `wcs = WCSettings(dt = 0.02)`, mode `vroReelOut`, `DURATION = 60.0`.
- Wind: `get_triangle_wind(wcs, 0.0, 5.0, 0.1, length(lg))` — its `freq` argument is
  1/period, so `0.1` is the requested 10 s period; 60 s gives six periods.
- Force: `F_t = k_kite * v_wind^2` with `k_kite = 240.0` N·s²/m², so 5 m/s -> 6000 N.
- **This force is open-loop**: it does not depend on `v_ro`, so the winch cannot
  influence its own input and the test can only check the law, the saturations and the
  hand-over to the force limiters — not closed-loop stability. Add the closed-loop
  variant `k_kite * (v_wind - v_ro)^2` (what `test_winchcontroller.jl` uses) behind a
  `CLOSED_LOOP` flag in the same script; it is one line and it is the case that can
  actually oscillate.
- **`f_high` defaults to 3800 N, below the 6000 N peak**, so with defaults the
  `UpperForceController` takes over at every peak and `v_ro` will *not* follow
  `kv*sqrt(F)` there. Run two legs: `f_high = 8000` for the clean law check, then the
  default for the limiter hand-over. Both are worth a plot.
- `kv = 0.06` gives 4.65 m/s at 6000 N, inside `v_sat = 8.0`; `f_low = 350` N is reached
  at `v_wind = 1.21` m/s, so the lower-force controller engages in every trough. Expect
  and plot that, do not treat it as a failure.
- Plot `v_wind`, `F_t`, `v_ro`, plus `v_set_in` and `get_state(wc)` so the hand-overs are
  visible.
- Add a headless `test/test_reelout.jl` (no plots) to `test/runtests.jl` asserting, on
  the clean leg and away from the limiters, `|v_ro - kv*sqrt(F)| < tol` in steady state.

File name: `test_reelout.jl`, not `test_reel-out.jl` — a hyphen is out of step with every
other file in that repo.

## Step 3 — `examples/simple_reelout.jl` here

A copy of `simple_fig8.jl` with the winch replaced; the entry state machine, guidance,
PID and blending are unchanged.

**Dependency.** Add `WinchControllers` to `examples/Project.toml` (compat `0.5.5`). The
root Manifest already pins the ecosystem, so use `Pkg.add("WinchControllers@0.5.5")` —
`Pkg.resolve()` alone will not pull it in. **As implemented:** the registered 0.5.5 does
not have the `mode` field from step 1 yet (unpublished), so `[sources]` points at a local
`path = "../../WinchControllers.jl"` for now, with a comment saying to switch to a
`rev`/`url` pin once a release ships it — same pattern as V3Kite's `bin/dev`, just not
wired to a script since this is the only package that needs it today.

**Settings collision — do NOT call `update(wcs)` / `WCSettings(true; dt)`.**
WinchControllers' `update` reads `joinpath(get_data_path(), wc_settings())`, which under
this run resolves to *this* package's `data/wc_settings.yaml` — a file in **V3Kite's**
`WC_Settings` schema (`winch_pos_kp`, `winch_speed_k`, ...). The key names do not
overlap, so the load would silently leave every WinchControllers field at its default.
Build `WCSettings(dt = s.dt)` in code instead and set its fields from `FC_Settings`,
which is where this repo's convention puts a run's tuning anyway.

**New `FC_Settings` fields** (`src/fc_settings.jl`):
`reelout_kv` -> `wcs.kv`, `reelout_f_low` -> `wcs.f_low`, `reelout_f_high` -> `wcs.f_high`,
`reelout_v_max` -> `wcs.v_sat`, and `reelout_l_max` [m], the stop length. **As
implemented:** these live in a NEW `data/fc_settings_reelout.yaml` (a copy of
`fc_settings.yaml` with `compliance: 0`), not the shared `fc_settings.yaml` —
that file's `compliance: 0.5` default is fig8's FORCE-mode tuning, and REEL_OUT
needs `compliance = 0` (see the exclusivity check below). Two files avoids
either a silent default mismatch or making every `simple_reelout.jl` run start
with a manual `fcs.compliance = 0` override; `system_reelout_150m.yaml`'s
`fc_settings:` key names the new file, exactly as `wc_settings:`/`sim_settings:`
already point at their own files per project.

**Coupling to V3Kite.** `step!` accepts `set_length` or `set_torque`, never a speed, so
the controller's `v_set` is integrated into the length setpoint:

```julia
v_set = calc_v_set(wc, reel_out_speed(s), winch_force(s), wcs.f_low)
l_set = min(l_set + v_set * s.dt, fcs.reelout_l_max)
step!(s; rel_depower, rel_steering, set_length = l_set,
      speed_limit = fcs.reelout_v_max, acceleration_limit = ..., vsm_interval = fcs.vsm_interval)
on_timer(wc)
```

`wcs.dt` must equal `s.dt`, and `on_timer(wc)` must be called exactly once per step. This
cascades WinchControllers' speed controller onto V3Kite's own inner speed PI inside
`winch_position_torque!`; that is acceptable while the outer loop is the slower of the
two, and it is the reason `speed_limit`/`acceleration_limit` are passed rather than left
at `Inf`. The alternative — a `v_set` -> torque loop — is a change to V3Kite and is not
taken here.

**Force mode is mutually exclusive with REEL_OUT.** `compliance > 0` holds a *force*;
error out at startup if `fcs.compliance > 0`, rather than silently ignoring one of the
two winches.

**Phases.** Hold `l0` until the pattern is established — start reeling out when the run
first reaches phase 4 (cross-track error below `settled_d_gate`), not at phase 3. When
`l_set` reaches `reelout_l_max`, freeze it; the rest of the run is the current
constant-length example. Log which of the three it is in.

**Feasibility — the binding constraint at 150 m, and the first thing to check.**
`check_pattern_feasible(fec, l_tether, ...)` is evaluated at one length today. A longer
tether *shrinks* the minimum angular turn radius (`1/(L*c1*u_s)`), so reel-out only ever
helps the margin — the **worst case is the start**, at 150 m, before a single metre has
been paid out. That is exactly the length `docs/fig8_tuning_log.md` (TETHER_LENGTH,
2026-07-26) records the pattern being moved *away* from: 150 -> 200 m was made because the
angular turn radius at 150 m was too large for this pattern, and it is called the single
most effective lever after `c1`. So before writing any winch code, run
`check_pattern_feasible` at 150 m with today's `fc_settings.yaml` and see what the margin
is. If it is below ~1, the pattern has to be adapted for this run — enlarge and lower it
(`f8_a`..`f8_d`, `el_center`), lower `body_damping` for a larger `c1`, or raise
`max_steering` — and the entry has to survive at 150 m too, since reel-out does not begin
until phase 4. Print the margin at both `l_tether` and `reelout_l_max` at startup; here it
is the FIRST number that decides whether the run is possible at all.

**Log slots.** `var_01`..`var_09` are taken, `step!` fills `var_14`..`var_16`;
`var_10`..`var_13` are free. Use `var_10` = `l_set`, `var_11` = `v_set`, `var_12` =
`Int(get_state(wc))`, `var_13` = force error. Per CLAUDE.md a slot change touches three
files — here the script docstring, the plots script and the metrics.

**Project files.** The run **starts at 150 m**: add `data/system_reelout_150m.yaml` +
`data/settings_reelout_150m.yaml` as copies of the 150 m pair (`l_tethers: [150.0]`), with
`log_file: 'reelout_150m'` and **`sim_time` between 150 s and 300 s** — the 150 m file's
90 s is sized for the entry plus two laps, not for a pay-out, and the simulation runs at
about twice realtime (a 300 s run costs ~150 s of wall time), so run length is not a
constraint here. Start
at 150 s and lengthen if the pay-out is still running at the end. `reelout_l_max = 250.0`
m is the starting value, which ends the run at the length the constant-length pattern was
tuned around and then some; at ~2 m/s the 100 m pay-out is ~50 s on top of the ~40 s
entry, so 150 s leaves room for laps at the final length. `select_project.jl` picks up
any `system*.yaml` in `data/`, so nothing else needs to change. The `wc_settings:` key in
the new system file still names the V3Kite-format file — see the collision above.

**Plots and metrics.** A `simple_reelout_plots.jl` beside `simple_fig8_plots.jl`, adding a
winch panel (tether length, `v_ro` vs `v_set`, winch force, controller state). In
`src/fig8_metrics.jl` add the mean mechanical reel-out power `mean(F * v_ro)` over the
reel-out window — headless, so sweeps keep working.

Finally add the script to `examples/menu.jl`.

## Step 4 — Acceptance

1. **VERIFIED.** WinchControllers' full test suite: 126/126 pass (`Pkg.test()`), the mode
   defaults to `"piecewise"`, and `examples/test_winchcontroller.jl` is unchanged.
2. **VERIFIED.** `test/test_reelout.jl`'s clean leg (`f_high = 8000`, above the dummy
   force's peak): `v_ro` tracks `kv*sqrt(F_t)` to within 0.25 m/s once the speed
   controller has been continuously active for >= 1 s. The excluded transient is a real
   one, not a law error: every trough drives the force below `f_low` and the
   `LowerForceController` reels all the way in (down to ~-5 m/s in the bench test) before
   handing back, and recovering from that excursion takes a settling window.
3. **VERIFIED.** A 200 s run of `simple_reelout.jl` (`system_reelout_150m.yaml`,
   `fc_settings_reelout.yaml`) grew the tether from 150.0 m to exactly 250.0 m
   (`reelout_l_max`) and held there, printing feasibility margins 1.22 (start, 150 m) and
   2.04 (end, 250 m). No overspeed abort, no solver failure, 200 s sim in 99.5 s wall
   (2.01x realtime, matching the "~2x realtime" documented elsewhere). All 3 plots
   (`simple_reelout_plots.jl`) rendered without error.
4. **VERIFIED.** Fig8 metrics over the reel-out run's settled window (RMS d = 1.23°, mean
   0.91°, max 4.94°, 7 laps, all 7 success criteria passed) are AS GOOD AS or better than
   a 90 s constant-length baseline flown on `system_fig8_150m.yaml` at the same 150 m
   start (RMS d = 1.51°, mean 1.08°, max 4.98°, 4 laps, all 7 criteria passed) — reeling
   out did not degrade tracking. Mean reel-out power: 3502 W over 2589 samples (the active
   pay-out window, ≈29 s of the 200 s run).

## Open questions

- **Is the pattern feasible at 150 m at all? — RESOLVED, yes.** Measured with today's
  `fc_settings.yaml` (`body_damping = [0,0,40]`, `max_steering = 0.32`, `c1 = 0.2819`):
  margin 1.22 at 150 m, 1.63 at 200 m, 2.04 at 250 m — all feasible, growing with length
  as expected. The tuning log's 150 -> 200 move predates the current `body_damping`/
  pattern values; no pattern change is needed to start this run at 150 m.
- Does the pattern centre need to descend as the tether grows, or is holding `el_center`
  good enough for a 100 m pay-out? Decide from the first run, not up front.
- Naming of the mode field and its enum values is not fixed; `vroReelOut`/`vroPiecewise`
  above is a placeholder consistent with the `wcs*` state names.
