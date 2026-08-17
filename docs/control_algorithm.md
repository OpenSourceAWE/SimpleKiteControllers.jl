# How the controller works

Flying the optimal trajectory for a given inflow condition is achieved in four steps:
- create the optimal trajectory
- use a path following algorithm to determine the course needed to follow it
- determine the steering set point derived from the actual position and desired course
- determine the optimal reel-out speed

The four steps are three cascaded loops plus the winch. Steps 2 and 3 run once per
timestep (`1/sample_freq`) and form the flight controller proper: geometry in, `rel_steering`
out. Step 1 runs before the flight, or at most slowly during it. Step 4 is independent of the
steering path and acts on the drum.

## 1. Create the optimal trajectory

The reference path is a closed curve in **(azimuth, elevation), both in degrees**, discretized
into `num_points` points and treated as cyclic. Today it is a parameterized lemniscate,
[`figure_eight_path`](../src/figure_eight_controller.jl#L50):

- `f8_a` — width, the azimuth half-span [deg]
- `f8_b` — height, the elevation span [deg]
- `f8_c` / `f8_d` — size of the right lobe and asymmetry, both `0` for a symmetric eight
- `az_center` / `el_center` — where the pattern sits on the sphere
- `up_loops` — traversal direction; the path is reversed if the direction at the right lobe
  does not match

The centre can be moved during the run with
[`set_path_center!`](../src/figure_eight_controller.jl#L168), which rebuilds the discretization
in place — meant for walking `el_center` gradually from the capture elevation down to the
force-optimal one, never for a step change.

**A trajectory is only optimal if it is flyable.** The V3's identified turn-rate law gives a
minimum angular turn radius `ρ = 1/(L·c1·u_s)` — the apparent wind speed cancels, so it depends
only on tether length, steering authority and `c1`.
[`check_pattern_feasible`](../src/figure_eight_controller.jl#L408) compares it against the
tightest geodesic radius of the path ([`path_min_radius`](../src/figure_eight_controller.jl#L395),
computed in true spherical geometry) and reports the ratio as `margin`. Below 1.0 the path asks
for a turn the kite cannot fly at `max_steering`, at any PID tuning. The tightest point of a
lemniscate in this metric is the **upper shoulder** of each lobe, not the lobe tip, and it
collapses as the pattern is raised, because `cos(elevation)` compresses the azimuth axis — which
is why the pattern must be flown low and wide. The margin is worst at the START of a reel-out
run: a longer tether only ever shrinks `ρ`.

Two mechanisms pick the shape:

- **Shape sweep** (`examples/optimize_fig8.jl`, [`src/optimization.jl`](../src/optimization.jl)).
  A grid over `f8_a` × `f8_b` is flown, one simulation per grid point across a pool of worker
  processes, and ranked by **mean reel-out power**. Shapes below `min_feasibility_margin` are
  dropped pre-flight rather than flown; runs that hit the winch's UpperForceController or that
  sat near the steering clamp are rejected as side-condition failures, since their power measures
  the limiter and not the shape. Results are appended to a YAML table under a file lock, which
  doubles as the sweep's resume state.
- **External optimizer** (`examples/optimize_path.jl`, the AWETrim REST client). Inflow
  conditions and an initial guess go to `POST /init`, and the server returns a closed trajectory
  of the same number of points. It maps `v_set = kv·sqrt(force)` onto its own radial force model,
  so the optimized path already assumes the winch law of step 4. See
  [TrajectoryOptimization.md](TrajectoryOptimization.md) for the interface and its open
  questions. The path following below does not care where the points came from: any closed
  (azimuth, elevation) curve is flyable by it.

## 2. Path following: from position to a commanded course

The guidance is the **attractor-point ("L0") law** of Fernandes et al., Energies 2022
(doi:10.3390/en15041390), implemented in
[`calc_attractor`](../src/figure_eight_controller.jl#L237) and
[`navigate_fig8`](../src/figure_eight_controller.jl#L305):

1. Find the closest point **Q** on the reference path; its distance is the cross-track error
   `dmin` [deg].
2. Walk `attractor_distance` degrees of arc **forward along the path** from Q to the attractor
   point **R**.
3. Return the great-circle course from the kite to R as `chi_set` [rad].

Unlike L1 logic the attractor is well defined at any distance from the path, so there is no
approach mode and no controller switching. `attractor_distance` is the single tuning knob: it
buys phase lead against the steering dead time — the run prints the lead arc as a flight time and
compares it with the identified `delay` — but too much lead cuts the corners of the pattern.

The lemniscate crosses itself, and at the crossing two branches whose tangents are ~180° opposed
are almost equally close. Picking the wrong one flips the commanded course. Two guards prevent it:

- **Continuity** (`search_window`): Q is searched only within a window of arc around the previous
  Q, so it advances along the path instead of teleporting. Beyond `reacquire_dist` of cross-track
  error the search goes global again, so a kite genuinely off-path is not trapped on a stale
  branch.
- **Flight direction** (`branch_tol`, `min_speed`): among the remaining near-equal local minima,
  the branch whose tangent needs the smaller heading change wins. The direction estimate is a
  low-passed (`course_tau`) increment of the kite's own position, with the two components filtered
  separately so ±180° never wraps.

Bearing convention throughout: **`0` = towards zenith, positive towards larger azimuth**, which
makes `chi_set` directly comparable to `SysState.heading`.
[`path_tangent`](../src/figure_eight_controller.jl#L332) returns the traversal direction at Q and
is used as the entry reference: with the kite almost directly above the pattern the great-circle
course to any attractor is "straight down" and its sign is numerical noise, while the tangent is
never degenerate.

The **entry** to the pattern is not part of the guidance. `examples/simple_fig8.jl` and
`examples/simple_reelout.jl` own a phase state machine (park → dive → hold → transition → fig8)
that overrides `chi_set` with open-loop courses during the dive and hold, and a descent limiter
that caps the commanded steepness while the kite is far off the path, blended in over
`entry_d_blend`. See [reelout_state_machine.md](reelout_state_machine.md).

## 3. Steering set point from position and desired course

The inner loop turns the commanded course into `rel_steering ∈ [-max_steering, max_steering]`.

**The feedback angle** is not simply the heading. At low kite speed the heading is the meaningful
quantity, at high speed the course is, and the two differ by the V3's ~13° drift angle. The
controller blends them by kite speed between `v_kite_heading` and `v_kite_course`
(`fig8_pure_course` forces pure course from phase 3 on). `SysState.course` needs
`wrap_to_pi(course + pi)` first — its raw zero points away from zenith, and feeding it unshifted
is positive feedback that diverges the run. The error is then `wrap_to_pi(fb - chi_cmd)`, formed
here because `DiscretePIDs` does not wrap, and handed to the PID against a zero reference.

**The PID** (`create_heading_pid`, V3Kite) is a PD controller by default — `heading_i` is `false`
— with a filtered derivative (`heading_d_n`; the `DiscretePIDs` default of 10 rings) and output
limits at `±max_steering`. Its gain is scheduled twice:

- by **apparent wind**, `K = heading_p · v_app_ref/v_app`. The plant's turn rate is
  `ψ̇ ≈ c1·v_a·u_s`, so the loop gain is constant only if `K ∝ 1/v_app`. Only the product
  `heading_p · v_app_ref` is physical.
- by **phase**, `entry_gain · heading_p` (25 %) during the entry, full gain from phase 3. During
  park the PID is stepped with a zero error and its output discarded, so engagement is bumpless.

The output is fed to `rel_steering` **unnegated**: positive `rel_steering` produces a positive
heading rate on this plant (measured, r = +0.998). Other kites in the ecosystem negate; that does
not transfer.

The plant constants behind all of this — `c1`, `c2` and the steering `delay` of
`ψ̇ = c1·v_a·u_s + c2/v_a·sin(ψ)·cos(β)` — are not tuning parameters but identified per
`(body_damping, depower)` and looked up from a table with
[`turn_rate_coeffs`](../src/turn_rate_table.jl); a lookup outside the identified range throws
rather than extrapolating.

## 4. Optimal reel-out speed

The winch runs its own loop on the measured tether force, independent of the steering above. In
`reelout` mode WinchControllers.jl commands the quasi-steady optimum
**`v_set = kv·sqrt(force)`** (`calc_vro`), bracketed by two force limiters that take over when
the speed law would leave the usable force window:

| state (`var_12`) | active when | effect |
|:--|:--|:--|
| 0 lower force | force below `f_low` | reels **in** to restore tension |
| 1 speed control | normal regime | `v_set = kv·sqrt(force)` |
| 2 upper force | force at/above `f_high` | reels out faster to cap the force |

`step!` takes a torque, never a length or a speed, so the example integrates `v_set` into a length
setpoint `l_set`, converts it with WinchControllers.jl's cascaded length loop
([`examples/winch_adapter.jl`](../examples/winch_adapter.jl)), and **also** passes `v_set` to that
loop as `v_ff`. Without the feed-forward, the integrate-here/differentiate-there pair through the
outer P loop (`winch_pos_kp`) is a 2 s first-order lag — measured at 1.16 s of delay and 0.49 of
the commanded amplitude on the 5.7 s reel-out oscillation, plus a standing ≈5 m length error.

Three refinements around the ends of the reel-out window, all in `examples/simple_reelout.jl`:

- **Soft-start** ramps the *command* over `reelout_softstart`; the controller's own integrators
  and force limiters still see the unramped value, so the engagement transient is shaped without
  lagging the force regulation.
- **Soft-stop** latches once the remaining distance would be covered within `reelout_softstop`
  seconds at the current rate, then decelerates linearly to exactly 0 at `reelout_l_max`. A hard
  cut leaves the drum's momentum to the position loop, which brakes it with a reel-in transient
  and a power undershoot. Loosening the acceleration limit instead makes it worse, not better.
- **Force floor guard** before reel-out begins: a standalone `LowerForceController`, clamped to
  reel-in only, catches the force sag the dive causes (measured ~50 N). It is deliberately not the
  main controller, whose speed integrator would wind up while its output is ignored and then dump
  a saturated command the moment it is first used.

The full phase/winch gating, including which timer starts what, is in
[reelout_state_machine.md](reelout_state_machine.md); the winch's own tuning lives in
`data/wc_settings.yaml`.

## Stability and robustness

**No loop in this stack is backed by a margin analysis.** Nothing is linearized about a trim
point and no gain or phase margin is computed anywhere; the `linearize` of
[`parking_controller.jl`](../src/parking_controller.jl#L48) is an NDI *inversion* of the plant,
not a stability analysis. Stability is an empirical claim, resting on the structural properties
below plus regression runs against the full nonlinear model.

### What keeps the loops stable by construction

- **Nothing fast is in the loop.** The winch's outer P loop is `winch_pos_kp = 0.5`, a 2 s time
  constant, and the inner speed loop reaches its setpoint proportionally (`winch_speed_k = 30`)
  with the integral acting only as a slow trim (`winch_speed_ti = 2 s`). The fast path is
  proportional in both, and the only integrator in the whole winch is that 2 s trim.
- **Feed-forward carries the steady state, feedback only the deviation.** `force_to_torque`
  supplies the holding torque, so at equilibrium with `winch_ff_scale = 1` the PI correction is
  exactly zero — the loop never has to hold a large output. The same idea buys reel-out tracking:
  `v_ff` (§4) removes the 2 s lag without raising `winch_pos_kp`, i.e. without spending margin.
- **Both winch limits act on the total setpoint**, feed-forward included — `speed_limit` first,
  then the `acceleration_limit·dt` rate limiter — so the sum of feed-forward and P term can never
  ask the drum for a speed or a slew it does not have.
- **The steering PID cannot wind up.** `heading_i` is `false` in both `data/fc_settings.yaml` and
  `data/fc_settings_reelout.yaml`, so the ±`max_steering` clamp bounds a memoryless PD law, whose
  D path is filtered at `heading_d_n = 2.0`.
- **Gain scheduling holds the loop gain constant** instead of tuning for the worst case:
  `K = heading_p·v_app_ref/v_app` cancels the `v_a` of `ψ̇ ≈ c1·v_a·u_s` (§3), and `entry_gain`
  runs the entry at 25 %.
- **Admissibility is checked before flight, not by the controller.** `check_pattern_feasible`
  (§1) rejects a path that no tuning can fly, and `turn_rate_coeffs` throws outside the identified
  range rather than extrapolating.
- **Engagement is bumpless everywhere**: the heading PID is stepped with a zero error and its
  output discarded during park (§3), `WinchForceController.f_lpf` initializes to the measured
  force, `warmup_torque` relaxes the model against the same callback the run will use, and
  soft-start/soft-stop shape both ends of the reel-out window (§4).

### What is actually verified by running it

- **Winch, unit level** — WinchControllers.jl's `test/test_torque_controllers.jl`: zero length
  error gives exactly the force feed-forward, speed saturation, the `acceleration_limit·dt` ramp,
  the `winch_force_min` floor, and the bumpless `f_lpf` init. V3Kite's `test/test-interface.jl`
  adds that `v_ff` *adds to* the P term rather than replacing it, and that `speed_limit` clamps
  the sum.
- **Guidance** — [`test/test_fig8_controller.jl`](../test/test_fig8_controller.jl) covers both
  crossing guards directly (`branch_disambiguation`, `search_window_continuity`), which are the
  failure modes that flip the commanded course, plus `turn_radius_feasibility`.
- **Closed loop against the nonlinear model** — V3Kite's `test/test_parking_ripple.jl` flies 600
  steps and gates the detrended AoA ripple RMS at 1.5× a measured 0.0064° baseline, reporting the
  spectral peak as well, so a lightly damped mode coming back is visible as a frequency and not
  just as a larger number. Solver cost is tracked alongside as a second, less noisy signal.

### What is not established, plus one bug this turned up

1. **No margins**, as above. Everything below follows from that.
2. **One operating point per metric.** The ripple baseline is fixed at 9.51 m/s, 150 m tether,
   `dt = 0.05/3`. There is no sweep over wind speed, tether length or `dt`, so how far the
   effective loop gain travels across the envelope is unknown — tether stiffness, and with it the
   winch loop gain, varies strongly with length and load. The shape sweep of §1 varies the
   *path*, not the *tuning*.
3. **Anti-windup is 5× slower than the integrator it protects.** `WinchPosController` builds its
   `DiscretePID` without `Tt` and with `Td` unset, so the `DiscretePIDs` fallback `Tt = 10 s`
   applies against `winch_speed_ti = 2 s`. On a sustained `winch_torque_limit = 500 N·m`
   saturation the integrator unwinds five times slower than it wound up. Untested, and the most
   likely source of overshoot after a long saturation.
4. **The adapter is a hand-kept copy, and it had drifted** (fixed).
   [`winch_adapter.jl`](../examples/winch_adapter.jl#L71) defaulted `acceleration_limit` to
   `winch_acc_limit(s.set)`, passing the whole `Settings` struct where
   `winch_acc_limit(max_acc) = max_acc > 0 ? … : Inf` wants a number, so every caller relying on
   the default raised `MethodError: no method matching isless(::Int64, ::Settings)` —
   `simple_fig8.jl:231` and `:386`, `simple_fig8_live.jl:317` and `:513`, and
   `simple_reelout.jl:234`; only `simple_reelout.jl:517`, which passes the limit explicitly,
   was unaffected. Now `s.set.max_acc`, matching V3Kite's copy. The file's own header says to keep
   the two in step; nothing enforces it, and a default-valued keyword is exactly where a drift
   hides from the call sites that pass the argument.
5. **Two `max_acc` values disagree.** `data/wc_settings.yaml` carries `max_acc: 8.0`, commented as
   "step!'s acceleration_limit"; the plant settings carry `winch: max_acc: 4.0`, and the adapter's
   default is meant to read the latter. `simple_reelout.jl` deliberately overrides with
   `rcs.max_acc = 8`, documented and measured — but the `wc_settings.yaml` comment is stale
   (`step!` no longer takes an acceleration limit at all) and a duplicated constant with two
   values invites reading the wrong one.
6. **`speed_limit` defaults to `Inf`** in the adapter, because the settings give the signed pair
   `v_ro_max`/`v_ro_min` rather than one number. `simple_reelout.jl` passes `rcs.v_sat = 8` m/s
   explicitly; the fig8 and parking hold paths run on the default, where a large length error
   would command an unbounded speed setpoint.

### Cheapest way to close them, in order

Reconcile the two `max_acc` values; set `Tt ≈ winch_speed_ti`
and add a saturation-recovery test; then run the parking-ripple metric as a small grid over wind
speed and tether length to see how far the effective loop gain actually travels. A margin plot is
considerably more work and only pays off if that grid shows enough variation to need scheduling.
