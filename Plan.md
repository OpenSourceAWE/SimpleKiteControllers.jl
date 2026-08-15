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

## Finding (2026-08-14) — `v_ro` lags `v_set` by ~1.2 s and halves its amplitude

Visible in the winch panel of `simple_reelout_plots.jl`: `v_ro` follows the sawtooth of
`v_set` at roughly half the amplitude and about a quarter period late. **MEASURED** on
`output/reelout_150m.arrow` over phase 4 (47.2 -> 124.8 s, `v_set > 0`):

| quantity                        | value                                    |
|:--------------------------------|:-----------------------------------------|
| dominant period of `v_set`      | 5.69 s (omega = 1.10 rad/s), 13 cycles    |
| mean `v_set` / mean `v_ro`      | 2.58 / 2.49 m/s — no steady-state error  |
| std `v_set` / std `v_ro`        | 3.01 / 1.48 m/s — amplitude ratio 0.49   |
| best cross-correlation lag      | 1.16 s (corr 0.986)                      |

**Cause: the length integration, not the winch.** The means match, so this is a
first-order phase lag, not a transport delay or a slow drum. `step!` takes a length or a
torque, never a speed, so the loop integrates `v_set` into `l_set`; V3Kite's
`winch_position_torque!` then differentiates it back out through a *pure P* outer loop,
`v_sp = kp_pos * (l_set - l)` with `winch_pos_kp = 0.5` 1/s. Integrator + P loop is
exactly a first-order lag of `tau = 1/kp_pos = 2 s`. The inner PI speed loop adds a
second, much smaller pole: `winch_speed_k = 30` N*m*s/m is 30*G/r ~ 1150 N per m/s at the
tether against a drum inertia of 0.204*G^2/r^2 ~ 300 kg equivalent linear mass, so
`tau_inner ~ 0.25 s`. Predicted at omega = 1.10: gain 0.40, lag 1.28 s — versus 0.49 and
1.16 s measured. The outer P loop accounts for essentially all of it.

Same root cause, same fix, one more symptom: a P loop tracking a *ramp* must run a
proportional error, so the actual tether length trails `l_set` by `v_ro/kp_pos` = 2*2.5 ~
**5 m** for the whole pay-out.

Neither limiter is binding: `v_set` slews at ~4.7 m/s^2 peak, against `speed_limit` = 8
m/s and `acceleration_limit` = `rcs.max_acc` = 8 m/s^2 — note that is the `WCSettings`
default, NOT the settings YAML's `max_acc: 4.0`, which `simple_reelout.jl` never reads.

Options, best first. **Resolved: option 1 was implemented and shipped, and it made
options 2 and 3 unnecessary — see "Improve reel-out winch control" below for the
before/after measurements.**

1. **Speed feed-forward** (removes the lag structurally). Give `step!` /
   `winch_position_torque!` a `v_ff` kwarg fed the current `v_set`, and use
   `v_sp = v_ff + kp_pos * (l_set - l)`. Leaves only `tau_inner ~ 0.25 s` and kills the 5 m
   length offset too. Needs no retuning; the change is in V3Kite, not here.
2. **Raise `winch_pos_kp`** in `data/wc_settings.yaml`, 0.5 -> 1.0...1.3, for `tau ~ 0.8 s`.
   One number, no API change. Do not go far past 1.3 without also raising
   `winch_speed_k`: the cascade wants the outer pole well inside 1/(3*tau_inner) ~ 1.3.
3. **Question the setpoint instead.** `v_set = kv*sqrt(F)` uses the *instantaneous* force,
   which swings over each figure-eight. Chasing +/-3 m/s at a 5.7 s period costs +/-900 N of
   tether force just to accelerate the drum's 300 kg equivalent inertia — force taken out
   of the kite. The attenuation may be the plant correctly filtering a setpoint that should
   not have been that lively; low-passing the force into `calc_v_set` is the alternative.

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

## Improve reel-out winch control
1. implement speed feed-forward control as explained above in V3Kite repo —
   **DONE and VERIFIED.** `step!` and
   `winch_position_torque!` gained a `v_ff` keyword [m/s]:
   `v_sp = v_ff + kp_pos * (set_length - l)`, with `speed_limit` and
   `acceleration_limit` still acting on the total setpoint. Default `v_ff = 0.0`
   is the old pure-feedback behaviour, so `simple_fig8.jl` and every
   constant-length caller are untouched. Covered by a new
   `test/test-interface.jl` testset asserting the default, the no-length-error
   case (`v_sp == v_ff`), that the P term ADDS to the feed-forward, and that
   `speed_limit` clamps the sum.

   **WIRED UP here too.** `bin/dev` re-pointed `examples/Project.toml` at the local
   `path = "../../V3Kite"` (the `rev = "v1.1.1"` pin, which predates the kwarg, is
   commented out above it and must come back — pointing at a new tag, not v1.1.1 —
   before this is released), and the loop now passes
   `step!(s; ..., set_length = l_set, v_ff = v_set, ...)`. `v_set` is already `0.0`
   outside the pay-out window, where `l_set` is constant and there is nothing to
   feed forward, so no extra gating is needed.

   **MEASURED**, same run configuration, on the pay-out window of
   `output/reelout_150m.arrow` (phase 4 with `v_set > 0`; 150 -> 350 m, 82.5 s):

   | quantity                       | `v_ff = 0` | `v_ff = v_set` |
   |:-------------------------------|-----------:|---------------:|
   | lag `v_set` -> `v_ro`          |     1.16 s |     **0.20 s** |
   | `l_set` - actual tether length | ~5 m standing | **-0.05 m** mean (-0.15 .. +0.59) |
   | std of `v_set`                 | 3.01 m/s   |   **0.14 m/s** |
   | std of `v_ro`                  | 1.48 m/s   |       0.17 m/s |
   | mean `v_set` / mean `v_ro`     | 2.58 / 2.49 |    2.42 / 2.42 |
   | time in speed control          | cycled 0<->1<->2 | **99.4 %**, state 0 never |
   | mean reel-out power            |     3502 W |     **4015 W** |
   | fig8 RMS cross-track           |     1.23 deg |     unchanged |

   The residual 0.20 s is the inner speed loop, right at the predicted ~0.25 s, and
   the standing length error is gone. Both expected.

   **The unpredicted result is row 3: the setpoint itself stopped oscillating.**
   `v_set = kv*sqrt(F)` is closed-loop — reel-out speed feeds back into apparent
   wind and so into force. At 1.16 s of lag that feedback arrived a fifth of a
   period late and sustained the 0 .. 7.5 m/s sawtooth; at 0.20 s it damps it. The
   sawtooth was never a property of the flight, it was the winch lag beating
   against its own force law. Two consequences:
   - Option 3 above (low-pass the force into `calc_v_set`) is **moot** — there is
     no longer an oscillation to filter. Do not implement it.
   - The +14.6 % power is not from reeling out faster; mean `v_ro` is slightly
     LOWER. It is from no longer falling into the force troughs: the
     `LowerForceController`, which used to reel IN at every trough, is now never
     invoked (state 0: 0.0 % of the window).

   **`acceleration_limit`: leave it at `rcs.max_acc` = 8.0 m/s^2, the `WCSettings`
   default, NOT the plant's `winch: max_acc: 4.0`.** With `v_ff` the pay-out speed
   is asked for the instant reel-out engages instead of being ramped in by a
   lagging P loop, so the drum's own inertia (`inertia_total * gear_ratio^2 /
   drum_radius^2` = 0.408*6.2^2/0.1615^2 = 602 kg of equivalent linear mass) is a
   plausible source of an engagement force spike, and the plant's own 4 m/s^2 the
   obvious cure. **TRIED AND REVERTED — it does not work.** Two runs, identical but
   for the kwarg (`output/archives/2026-08-15_214448` = `rcs.max_acc` 8.0,
   `2026-08-15_220138` = `project_set.max_acc` 4.0):

   | | a = 8 | a = 4 |
   |:--|--:|--:|
   | peak force, reeling window | 3758 N | 3783 N |
   | peak force, first 5 s bin  | 3641 N | 3732 N |
   | ring overshoot             | 0.88 m/s | 0.98 m/s |
   | ring duration              | 8.5 s | 9.6 s |
   | ring zeta                  | 0.120 | 0.113 |
   | mean reel-out power        | 7763 W | 7761 W |

   Three findings, all of which outlive the reverted change:

   - **The limiter is almost never the binding constraint.** Reconstructing the raw
     setpoint `v_sp = v_ff + kp*(l_set - l)` (kp = 0.5) over the pay-out window,
     `|dv_sp/dt|` exceeds 4 m/s^2 in ~1 % of steps and 8 m/s^2 in 0.03 %. Mean
     `v_set` is 2.33 m/s, max 2.93. There is nothing there to clip.
   - **There is no engagement spike left to cure.** The one this fix was aimed at
     (4582 N for 0.48 s at the instant of engagement) was measured back when
     `reelout_t_startup` was 2.0 s and `reelout_f_high` 3800 N; the tuning has since
     moved to `t_startup: 10.0` and `f_high: 7600`, and the plant's `max_force` from
     4000 to 8000 N. The 10 s soft start spreads engagement out on its own. Today's
     peak is 3783 N at t = 37.2 s, 6.6 s AFTER pay-out begins, and per-5-s-bin
     maxima sit at 3.5-3.8 kN every ~10 s across the whole 86 s window — the
     figure-eight turn load, less than half of `max_force`, not a drum-inertia
     transient.
   - **The engagement ring is not inertia-limited.** Slowing the setpoint made it
     worse, not better (rows 3-5): the tighter limiter makes the drum lag the
     `t_startup` ramp, the length error grows, and the P term hands it back later.
     Past t ~ 40 s the two runs agree to three digits. So look at `t_startup` and at
     the inner speed loop for the ring, not at `acceleration_limit`.

## Implement parallel execution harness
To optimize the shape of the figure of eight, many simulations are needed. Write the code that enables parallel simulations in separate Julia processes. 

There should be a file optimization.yaml with the settings:
max_processes = 6
min_f8_a = 20
max_f8_a = 30
min_f8_b = 8
max_f8_b = 12
step_size = 1 # step size in degrees

The results shall be stored in the file
optimization_results.yaml. The results table must be protected against concurrent access.

Key parameter to optimize is the power.

Side conditions:
- the upper force controller shall never become engaged
- pct_time_within_2pct_of_peak must not exceed 5%

**IMPLEMENTED**, not yet run over the full grid.

- `data/optimization.yaml` — the settings above, plus the results file name, a
  `max_runs_per_process` restart bound, and the two side conditions as explicit
  limits (`max_pct_time_upper_force: 0.0`,
  `max_pct_time_within_2pct_of_peak: 5.0`). Read into `OptSettings`
  (`src/optimization.jl`) the same way `fc_settings.yaml` is read into
  `FC_Settings`: unknown key is an error, missing key falls back to the default.
- `examples/optimize_fig8.jl` — the driver. Prints the plan (grid, cores, free
  RAM, project, wind conditions, both limits), starts the workers, reports
  progress and an ETA every 30 s, restarts a worker that dies while work is left,
  and prints/writes the ranked report at the end
  (`output/optimization_report.txt`).
- `examples/optimize_fig8_worker.jl` — one worker. Claims a grid point, flies it
  by `include`ing `simple_reelout.jl` with `FCS_OVERRIDES = Dict(:f8_a => …,
  :f8_b => …)`, records the run's metrics, repeats. Long-lived and task-pulling
  rather than one process per run: package load plus model build is about a
  minute against a few minutes of simulation.
- `simple_reelout.jl` gained three optional globals, all read-and-cleared on the
  `SHOW_PLOTS` convention so a sweep can never leak into an interactive run:
  `FCS_OVERRIDES` (the swept parameters), `OUTPUT_PATH` (each worker logs into
  its own `output/optimization/w<id>/`, since the output names come from the
  project and would otherwise be one shared file) and `RUN_ARCHIVE = false` (55
  archive folders each holding a 40 MB copy of the arrow log, for information the
  results table already carries).
- `winch_state_pct` (`src/fig8_metrics.jl`) is the new metric the first side
  condition needs: the share of the reel-out window spent in each of the
  `WinchController`'s three states, scored over the same window as
  `reelout_power`. It also goes into every run summary as
  `reelout: winch_state:`, so the interactive script reports it too. Measured on
  the two runs above: 100 % speed control, upper force never engaged.

Two things worth knowing before reading the first sweep:

- **Concurrency.** Claiming a grid point and appending a result share ONE lock,
  an atomic `mkdir` (`with_file_lock`), so a combination is never flown twice and
  entries never interleave. Verified with six real processes racing over the
  55-point grid: 55 entries, 55 unique, whole grid covered, no malformed entry.
  Appending (rather than rewriting the table) keeps the critical section O(1) and
  means a killed process can at worst leave one truncated entry.
- **The first run is flown alone.** On a cold checkout the settled-geometry and
  model caches do not exist, and six processes building them into the same files
  would corrupt them. The driver therefore starts one worker, waits for its first
  result, and only then starts the rest — and aborts the sweep if that first run
  did not complete, rather than repeating the same failure 55 times.

The results table is also the resume state: a combination already in it is never
re-flown, so an interrupted sweep continues where it stopped and starting over
means deleting `output/optimization_results.yaml`.

## Verify harness
Before committing hours, fly two points: set `max_f8_a: 21.0` and
`max_f8_b: 8.0` in `data/optimization.yaml` (a 2 x 1 grid) with
`max_processes: 2`, run this script, and check that
`output/optimization_results.yaml` has two entries whose `mean_power_W` and
`pct_time_upper_force` are filled in.