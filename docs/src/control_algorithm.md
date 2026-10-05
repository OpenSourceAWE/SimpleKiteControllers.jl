# How the controller works

Flying the optimal trajectory for a given inflow condition is achieved in four steps:
- create the optimal trajectory
- use a path following algorithm to determine the course needed to follow it
- determine the steering set point derived from the actual position and desired course
- determine the optimal reel-out speed

The four steps are three cascaded loops plus the winch. Steps 2 and 3 run once per
timestep (`1/sample_freq`) and form the flight controller proper: geometry in, `rel_steering`
out. Step 1 runs before the flight, and again a few times during a reel-out. Step 4 is
independent of the steering path and acts on the drum.

The settings of steps 1–3 are in `data/fc_settings.yaml` (`simple_fig8.jl`) and
`data/fc_settings_reelout.yaml` (`simple_reelout.jl`, `simple_opt_reelout.jl`), loaded into
[`FC_Settings`](@ref) ([`src/fc_settings.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/fc_settings.jl)), with one YAML section per part:
`course`, `feedforward`, `pattern`, `wind_ramp`, `winch`, `reelout`, `low_wind`, `run`. Below,
`fcs.pattern.attractor_dist` and similar names refer to those parts. The winch settings of step 4
are in `data/wc_settings.yaml`.

## 1. Create the optimal trajectory

The reference path is a closed curve in **(azimuth, elevation), both in degrees**, discretized
into `num_points` points and treated as cyclic. The default is a parameterized lemniscate,
[`figure_eight_path`](@ref) in [`src/figure_eight_controller.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/figure_eight_controller.jl):

- `f8_a` — width, the azimuth half-span [deg]
- `f8_b` — height, the elevation span [deg]
- `az_center` / `el_center` — where the pattern sits on the sphere
- `up_loops` — traversal direction; the path is reversed if the direction at the right lobe
  does not match

[`set_path_center!`](@ref) moves the centre during the run and rebuilds the discretization in place.
[`set_path!`](@ref) installs any other closed curve (an optimized one, see below). It can resample the
curve and keeps Q on the same branch at the crossing.

**A trajectory is only optimal if it is flyable.** The V3's identified turn-rate law gives a
minimum angular turn radius `ρ = 1/(L·c1·u_s)` ([`min_turn_radius`](@ref)). The apparent wind speed
cancels, so `ρ` depends only on tether length, steering authority and `c1`.
[`check_pattern_feasible`](@ref) compares it against the tightest geodesic radius of the path
([`path_min_radius`](@ref), computed in true spherical geometry) and reports the ratio as `margin`.
Below 1.0 the path asks for a turn the kite cannot fly at `max_steering`, whatever the PID
tuning. In this metric the tightest point of a lemniscate is the **upper shoulder** of each
lobe, not the lobe tip. It tightens as the pattern is raised, because `cos(elevation)`
compresses the azimuth axis, which is why the pattern must be flown low and wide. The margin is
worst at the START of a reel-out run, because a longer tether only ever shrinks `ρ`.

In `simple_opt_reelout.jl` the shape comes from the **external optimizer** AWETrim, through its
REST client ([`src/awetrim_client.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/awetrim_client.jl)):

- **Startup solve** ([`src/startup_path.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/startup_path.jl)): the inflow, the
  first-lap winch, the minimum turn radius and a guess lemniscate fitted into the startup box
  go to `POST /init`. The server returns a closed trajectory. Failed solves are retried
  ([`src/startup_retry.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/startup_retry.jl)).
- **Re-optimizations during the reel-out**, as the tether grows. Before a reply is installed,
  [`src/reopt_gate.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/reopt_gate.jl) checks its curvature margin at the current
  length, its lowest height and elevation, whether blending into it folds the path, and its
  predicted power. A path that grew is cross-checked against a cold solve started inside the
  size box.

The server maps the soft winch law of step 4 onto its own radial force model, so the optimized
path already assumes that law. The depower it is given comes from the identified depower
conversion (`data/depower_conversion.yaml`). See
[TrajectoryOptimization.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/TrajectoryOptimization.md) for the interface. The path following
below does not care where the points came from: it can fly any closed (azimuth, elevation)
curve.

## 2. Path following: from position to a commanded course

The guidance is the **attractor-point ("L0") law** of Fernandes et al., Energies 2022
(https://www.mdpi.com/1996-1073/15/4/1390), implemented in [`calc_attractor`](@ref) and
[`navigate_fig8`](@ref) ([`src/figure_eight_controller.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/figure_eight_controller.jl)):

1. Find the closest point **Q** on the reference path. Its distance is the cross-track error
   `dmin` [deg].
2. Walk [`attractor_distance`](@ref) degrees of arc **forward along the path** from Q to the attractor
   point **R**.
3. Return the great-circle course from the kite to R as `chi_set` [rad].

Unlike L1 logic, the attractor is well defined at any distance from the path, so there is no
approach mode and no controller switching.

**The lead.** `attractor_distance` buys phase lead against the steering dead time, but too much
lead cuts the corners of the pattern. In the reel-out runs it is not a fixed number:
`attractor_distance(fcs, v_app, L)` ([`src/fc_settings.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/fc_settings.jl)) sets it each
step to the arc flown in `attractor_lead_time` (1.05 s), `rad2deg(t·v_app/L)`. The arc is
clamped to `[floor, 2·floor]`. The floor ([`attractor_floor`](@ref SimpleKiteControllers.attractor_floor)) is `attractor_dist` scaled by
`attractor_dist_ref_length/L`, a fixed arc *length* of 25.3 m (8.35° at 175 m), so the
guidance rate it sets no longer drops as the tether grows. With `attractor_lead_time = 0` the
floor alone is flown, which is what `simple_fig8.jl` does (`attractor_dist: 5.5`).

**The crossing.** The lemniscate crosses itself, and at the crossing two branches whose
tangents point ~180° apart are almost equally close. Picking the wrong one flips the commanded
course. These guards prevent it:

- **Continuity** (`search_window`, `search_window_max_frac`): Q is searched only within a
  window of arc around the previous Q. The window's half-width is capped at a fraction of the
  path length, so a small pattern still has a guarding window. Beyond `reacquire_dist` of
  cross-track error the search goes global again, so a kite that really is off the path is not
  trapped on a stale branch. A global candidate that beats the in-window one by
  `reacquire_margin` also wins.
- **Flight direction** (`branch_tol`, `branch_hysteresis`, `min_speed`): candidates whose
  tangent points against the kite's motion are dropped. Among the remaining near-equal local
  minima, the branch whose tangent needs the smaller heading change wins, but only by
  `branch_hysteresis`. Inside that band, the candidate nearest the previous Q is kept.
  Without the hysteresis, Q flips back and forth wherever the kite runs nearly parallel to the
  path. The direction estimate is a low-passed (`course_tau`) increment of the kite's own
  position. Its two components are filtered separately, so ±180° never wraps. The direction is
  trusted only above `min_speed`.
- **Rate limit** (`q_rate_gain`): Q may not move faster than `q_rate_gain` times the arc the
  kite flew this step (at least one point). When two minima of a flat distance profile swap
  order, the new Q is walked to over a few steps rather than jumped to. The rate limit is off
  only while the kite is off the path (the search is global).

Bearing convention throughout: **`0` = towards zenith, positive towards larger azimuth**, which
makes `chi_set` directly comparable to `SysState.heading`. [`path_tangent`](@ref) returns the
traversal direction at Q and is used as the entry reference. With the kite almost directly
above the pattern, the great-circle course to any attractor is "straight down" and its sign is
numerical noise, while the tangent is always well defined.

The **entry** to the pattern is not part of the guidance. It is the phase ladder of the course
controller (§3), see [reelout\_state\_machine.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/reelout_state_machine.md).

## 3. Steering set point from position and desired course

The inner loop turns the commanded course into `rel_steering ∈ [-max_steering, max_steering]`,
and sets `rel_depower` alongside it. It lives in [`CourseController`](@ref)/[`calc_steering`](@ref) in
[`src/course_controller.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/course_controller.jl), called once per step by
`simple_fig8.jl`, `simple_fig8_live.jl`, `simple_opt_fig8.jl` and `simple_reelout.jl`. The
reel-out runs call it through [`steering_command!`](@ref SimpleKiteControllers.steering_command!) in
[`src/reelout_loop.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/reelout_loop.jl), which also supplies the gain scale and the
feed-forward.

**The phase ladder.** Phase 0 park (zero steering for `park_time`) → 1 dive (open-loop
`chi_dive` until `el_center + dive_el_margin`) → 2 hold (`chi_hold` for `hold_time`) → 3
transition (closed loop on the guidance) → 4 figure of eight (once `dmin < fig8_d_gate`) → 5
final (after the reel-out ends). [`set_phase!`](@ref) never moves backwards. Until the kite is close
to the path, a **descent limiter** caps the commanded steepness at `entry_chi_max`, blended in
over `entry_d_blend` above `entry_d_gate`.

**The feedback angle** is not simply the heading. At low kite speed the heading is the meaningful
quantity, at high speed the course is, and the two differ by the V3's ~13° drift angle. The
controller blends them by kite speed between `v_kite_heading` and `v_kite_course`
(`fig8_pure_course` forces pure course from phase 3 on). `SysState.course` needs
`course_offset` (π by default) added first. Its raw zero points away from zenith, and feeding it
unshifted gives positive feedback that makes the run diverge. The error is then
`wrap2pi(psi_prime - chi_cmd)`, formed here because `DiscretePIDs` does not wrap, and handed to
the PID against a zero reference.

**The PID** (built directly from `DiscretePID`; no kite-model dependency) is a PD controller —
`heading_i` is `false` in both settings files — with a filtered derivative (`heading_d_n = 2`;
the `DiscretePIDs` default of 10 rings) and output limits at `±max_steering`. Its gain is
`K = gain_scale · K_phase · v_app_ref / v_app_eff`, scheduled three ways:

- by **apparent wind**. The plant's turn rate is `ψ̇ ≈ c1·v_a·u_s`, so the loop gain is
  constant only if `K ∝ 1/v_app`. Only the product `heading_p · v_app_ref` is physical.
  `v_app_eff` is floored at `v_app_min` (and at `v_app_min_pattern` from phase 3 on, when set),
  because the dead time grows at low `v_app` and an unbounded gain boost would cost margin.
- by **phase**: `K_phase = entry_gain · heading_p` below phase 3, full `heading_p` from phase 3.
  During park the PID is stepped with a zero error and its output discarded, so the controller
  engages without a jump.
- by **depower** (`gain_scale`, reel-out runs only): `c1` changes with the depower actually
  flown, so `loop_gain_scale` ([`src/loop_decisions.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/loop_decisions.jl)) scales the
  gain by `c1(depower_setpoint)/c1(depower)`. This keeps the loop at the gain it was tuned at.

**The curvature feed-forward** (from phase 4 on; `feedforward:` section). `u_ff` is the path's
own course rate at `ff_lead_time` ahead of Q ([`path_turn_rate`](@ref)) through the turn-rate law,
`u_ff = ff_gain · ψ̇_path/(c1·v_app)`. It is added to the PID's output and clamped with it, and
the PID never sees it. It fades out with large cross-track (`ff_d_fade`) or course
(`ff_err_fade`) errors. The attractor is a chord ahead of the kite, so even a kite exactly on
the path reads a steady course error, and the PD would steer the curvature a second time.
`chi_ff` ([`path_chord_offset`](@ref)) is therefore subtracted from the commanded course. Without that
correction the two add up and the kite overturns. Both signals are low-passed over `ff_tau`.
With `ff_gravity_rate > 0`, `u_ff` also cancels the law's gravity turn in its fixed-`c3` form,
`−ff_gravity_rate·sin χ·cos β/(c1·v_app)`, with χ and β read off the path at the same
lead point ([`path_gravity_shape`](@ref)). Without it the PD holds the gravity turn off with a steady
course error, and the kite flies below the path on every horizontal leg.

The output is fed to `rel_steering` **unnegated**: positive `rel_steering` produces a positive
heading rate on this plant (measured, r = +0.998). Other kites in the ecosystem negate; that does
not transfer.

**Depower.** The target is `entry_depower` during dive and hold, `depower_final` in phase 5
(with a force limiter up to `depower_final_max`), and `depower_setpoint` otherwise. In
`simple_fig8.jl` the setpoint follows the wind ramp. A change of target ramps `rel_depower`
over `depower_blend_time`.

The plant constants behind all of this — `c1`, `c2` and the steering `delay` of
`ψ̇ = c1·v_a·u_s + c2/v_a·sin(ψ)·cos(β)` — are not tuning parameters. They are identified per
`(body_damping, depower)` and looked up from `data/turn_rate_coeffs.yaml` with
[`turn_rate_coeffs`](@ref). A lookup outside the identified range throws
rather than extrapolating.

## 4. Optimal reel-out speed

The winch runs its own loop on the measured tether force, independent of the steering above. In
`reelout` mode, WinchControllers.jl commands the quasi-steady optimum
**`v_set = kv·sqrt(force)`**, with one `kv` (0.0408) at every wind speed. How the usable force
window `[f_low, f_high]` is enforced depends on `force_limit`:

- **`"soft"`** (the default, `calc_vro_soft`): the speed law itself bends into the limits
  through smooth corners — softplus at `f_high` (`softplus_beta`), softminus at `f_low`
  (`softminus_beta`), and a smooth clamp at `v_sat` (`v_sat_beta`). With `soft_lfc`, a reel-in
  line from `v_reel_in` at zero force replaces the lower force controller. The force is
  low-passed over `force_limit_tau` before the law is inverted. This is the same tension curve
  AWETrim optimizes against.
- **`"hard"`**: the original three-state machine.

  | state (`var_12`) | active when | effect |
  |:--|:--|:--|
  | 0 lower force | force below `f_low` | reels **in** to restore tension |
  | 1 speed control | normal regime | `v_set = kv·sqrt(force)` |
  | 2 upper force | force at/above `f_high` | reels out faster to cap the force |

`data/winch_table.yaml` overrides `f_low` and `force_limit` per wind speed. In the first lap,
and in the startup solve, `f_high` is de-rated by `first_lap_force_frac`.

`step!` takes a torque, never a length or a speed, so the run integrates `v_set` into a length
setpoint `l_set`, converts it to torque with WinchControllers.jl's cascaded length loop
([`examples/winch_adapter.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/winch_adapter.jl)), and **also** passes `v_set` to that
loop as `v_ff`. Without the feed-forward, the outer P loop (`winch_pos_kp`) sits between
integrating here and differentiating there and acts as a 2 s first-order lag. That lag was
measured at 1.16 s of delay and 0.49 of the commanded amplitude on the 5.7 s reel-out
oscillation, plus a standing ≈5 m length error.

Three refinements around the ends of the reel-out window, in
[`src/reelout_loop.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/reelout_loop.jl):

- **Soft-start** ramps the *command* over `reelout_softstart`. The controller's own integrators
  and force limiters still see the unramped value, so the start-up transient is shaped without
  slowing down the force regulation.
- **Soft-stop** latches once EITHER the remaining distance would be covered within
  `reelout_softstop` seconds at the current rate, OR `n_fig_eight` complete figures of eight have
  been flown. The lap count is a second, independent stop criterion (`n_fig_eight = 0`, the
  current setting, disables it and leaves `reelout_l_max` as the only criterion). Whichever
  fires first decelerates linearly to exactly 0, landing at `reelout_l_max` for the length
  criterion or below it for the lap criterion. A hard cut would leave the drum's momentum to the
  position loop, which brakes it with a reel-in transient and a power undershoot. Loosening the
  acceleration limit instead makes it worse, not better.
- **Force floor guard** before reel-out begins: a standalone `LowerForceController`
  (`guard_lfc`, floor `entry_f_min`), clamped to reel-in only, catches the force sag the dive
  causes (measured ~50 N). It is deliberately not the main controller, whose speed integrator
  would wind up while its output is ignored and then dump a saturated command the moment it is
  first used.

Reel-out starts `reelout_delay` after phase 3, or earlier once the force reaches
`reelout_f_trigger`. Below 6.4 m/s of wind at 100 m height, the `low_wind` schedule
([`src/low_wind_schedule.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/low_wind_schedule.jl)) sets a longer start length and a lower
startup elevation guess. The full phase/winch
gating, including which timer starts what, is in
[reelout\_state\_machine.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/reelout_state_machine.md).

## Stability and robustness

The **course loop** has a margin analysis; the **winch** loop does not.

### Course loop: disk margins

[course\_loop\_stability.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/course_loop_stability.md) (figure of eight) and
[course\_loop\_stability\_reelout.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/course_loop_stability_reelout.md) (reel-out, over the full
tether length) linearize the course loop, with the steering tape modelled as a lag and the
kite's dead time and lag depending on `v_a`. They compute disk margins for the inner loop alone
and for the loop closed through the attractor guidance ("guided"). The plant model is
[`src/course_loop_model.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/src/course_loop_model.jl); the transfer functions are in the
ControlSystemsBase extension.

- **Validated** against the nonlinear simulation: by pushing the loop to instability, by injected
  multisines, and by cross-track step tests during the reel-out (damping and ringing frequency
  match while the steering stays off its clamp). The model under-predicts the simulation's
  margins, so it is conservative.
- **Used for tuning.** `examples/stability_global.jl` rates the *live* settings at every
  operating point of the 22 archived scenarios of both sites (tether-length bins, `v_a`,
  depower, dead time from the logs). `examples/retune_guided.jl` proposes small changes
  against it. `heading_p`, `heading_d`, `attractor_lead_time` and the 1/L attractor floor were
  all set this way. The floor was chosen to keep the worst guided disk margin ≥ 0.51.
- **Not covered:** steering saturation. The lightly damped ringing seen at a held length comes
  from the clamp, which the linear model cannot see.

### What keeps the other loops stable by construction

- **Nothing fast is in the winch loop.** The outer P loop is `winch_pos_kp = 0.5`, a 2 s time
  constant. The inner speed loop reaches its setpoint proportionally (`winch_speed_k = 30`),
  with the integral acting only as a slow trim (`winch_speed_ti = 2 s`).
- **Feed-forward carries the steady state, feedback only the deviation.** `force_to_torque`
  supplies the holding torque, so at equilibrium with `winch_ff_scale = 1` the PI correction is
  exactly zero. The same idea buys reel-out tracking: `v_ff` (§4) removes the 2 s lag without
  raising `winch_pos_kp`, i.e. without spending margin. On the steering side, `u_ff` (§3) flies
  the path's curvature, so the PD only corrects the deviation.
- **Both winch limits act on the total setpoint**, feed-forward included — `speed_limit` first,
  then the `acceleration_limit·dt` rate limiter. Together, feed-forward and P term can never
  ask the drum for more speed or acceleration than it has.
- **The steering PID cannot wind up.** `heading_i` is `false`, so the ±`max_steering` clamp
  bounds a PD law that has no memory.
- **Admissibility is checked before flight, not by the controller.** `check_pattern_feasible`
  (§1) rejects a path that no tuning can fly, the re-optimization gate rejects replies below
  the curvature margin, and `turn_rate_coeffs` throws outside the identified range.
- **Every loop engages without a jump**: the heading PID is stepped with a zero error during
  park (§3). `WinchForceController.f_lpf` starts at the measured force. `warmup_torque` relaxes
  the model against the same callback the run will use. Soft-start/soft-stop shape both ends
  of the reel-out window (§4), and depower changes are ramped.

### What is verified by tests

- **Winch, unit level** — WinchControllers.jl's `test/test_torque_controllers.jl`: zero length
  error gives exactly the force feed-forward; speed saturation; the `acceleration_limit·dt`
  ramp; the `winch_force_min` floor; and that `f_lpf` starts at the measured force. V3Kite's
  `test/test-interface.jl` adds that `v_ff` *adds to* the P term rather than replacing it, and
  that `speed_limit` clamps the sum.
- **Guidance** — [`test/test_fig8_controller.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/test/test_fig8_controller.jl) covers the
  crossing guards (`branch_disambiguation`, `search_window_continuity`,
  `set_path_keeps_branch_at_crossing`, `reacquire_respects_rate_limit`), the feasibility check
  (`turn_radius_feasibility`), the curvature feed-forward helpers (`path_turn_rate_and_chord`)
  and the turn-rate table lookup.
- **Inner loop** — [`test/test_course_controller.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/test/test_course_controller.jl): bumpless
  park engagement, the `1/v_app` and `entry_gain` gain schedule, the `±max_steering` clamp, the
  wrapped error at the ±180° cut, the ψ' blend's endpoints and `course_offset`, the descent
  limiter's gate/blend/latch, the ladder's phase transitions (including `set_phase!`'s "never
  backwards"), and the unnegated `rel_steering` sign convention, plus a hand-checked numeric
  fixture. [`test/test_steering_blocks.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/test/test_steering_blocks.jl) and
  [`test/test_reelout_loop.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/test/test_reelout_loop.jl) cover the reel-out wiring around it.
- **Stability model** — [`test/test_course_loop_model.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/test/test_course_loop_model.jl).
- **Closed loop against the nonlinear model** — V3Kite's `test/test_parking_ripple.jl` flies 600
  steps and fails if the detrended AoA ripple RMS exceeds 1.5× a measured 0.0064° baseline. It
  also reports the spectral peak, so a lightly damped mode that comes back shows up as a
  frequency and not just as a larger number. Solver cost is tracked alongside as a second, less
  noisy signal. The regression scenarios themselves (`build_all_scenarios.jl`) are flown by
  hand, not in CI.

### What is not established

1. **No winch margins.** Nothing in the winch loop is linearized; its stability rests on the
   structural properties above and on the parking-ripple test at one operating point (9.51 m/s,
   150 m tether, `dt = 0.05/3`). Tether stiffness, and with it the winch loop gain, varies
   strongly with length and load.
2. **The adapter is a hand-kept copy** of V3Kite's. It has drifted once already (a
   `winch_acc_limit(s.set)` default that raised a `MethodError` for every caller relying on it,
   since fixed), and V3Kite's copy still does not pass the drum inertia to the acceleration
   feed-forward (harmless there, since V3Kite runs with `winch_acc_ff = 0`). The file's header
   says to keep the two in step; nothing enforces it. A single copy would need a
   WinchControllers extension in V3Kite.

### Next step

Run the parking-ripple metric as a small grid over wind speed and tether length to see how far
the effective winch loop gain actually moves. A winch margin analysis along the lines of the
course-loop one is more work, and only pays off if that grid shows enough variation to need
gain scheduling.
