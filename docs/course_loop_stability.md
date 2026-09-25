# Course-loop stability analysis (disk margins)

2026-09-25. Findings from `examples/stability_course_controller.jl`, a
disk-margin analysis of the linearized inner loop of `simple_fig8.jl`. The
method follows WinchControllers.jl's `examples/stability_lfc.jl`.
**Status: open.** The model says the pattern loop is barely stable. That has not
yet been checked against the simulation.

## Model

- **Controller:** the exact discrete transfer function of the `DiscretePID` in
  `CourseController`, from the regulated error to `rel_steering`:
  `C(z) = K + bd·(z-1)/(z-ad)`, with `ad = Td/(Td+N·Ts)` and `bd = K·N·ad`
  (DiscretePIDs' backward-Euler filtered derivative). The integral term
  `K·Ts/Ti/(z-1)` is included when `heading_i` is a number.
  - Gain schedule: `K = heading_p · v_app_ref / max(v_a, v_app_min)`, times
    `entry_gain` below phase 3.
- **Plant:** the identified turn-rate law linearized about heading `ψ0`:

      ψ̇ = c1·v_a·u_s(t - delay) + c2/v_a·cos(ψ0)·cos(β)·δψ

  - `c1`, `c2` and `delay` come from `turn_rate_coeffs(fcs.body_damping, depower)`
    (`data/turn_rate_coeffs.yaml`).
  - The gravity term is a slow real pole. Both signs are checked (β = `el_center`)
    and the worse result is reported. In practice it matters only at very low `v_a`.
  - The plant is ZOH-discretized at `1/sample_freq`, and the dead time is an
    exact `round(delay/Ts)`-sample shift register.
- **Margins:**
  - `RobustAndOptimalControl.diskmargin` gives the balanced disk margin α with its
    gain and phase ranges.
  - The delay margin is computed by the script's own `delay_margin`, because
    `ControlSystemsBase.delaymargin` uses the unwrapped phase margin (e.g. 374°
    instead of 14°) and so reported 1.5 s instead of 57 ms.

## Results (project `system_fig8_200m.yaml`, dt = 0.01 s)

Settings: `heading_p = 0.35`, `heading_d = 0.30 s`, `heading_d_n = 2`, no
integral, `depower_setpoint = 0.27`, `entry_depower = 0.37`, `entry_gain = 0.25`,
`body_damping = [0, 0, 40]`.

| Case | α | Critical frequency | Gain margin range | Phase margin | Delay margin |
|---|---|---|---|---|---|
| Pattern, depower 0.27, v_a ≥ 10 m/s | **0.061** | 0.77 Hz | 0.94 – 1.06 | 3.5° | 57 ms |
| Pattern, v_a = 5 m/s (gain schedule clamped) | 0.705 | 0.72 Hz | 0.48 – 2.09 | 38.8° | 0.99 s |
| Entry, depower 0.37, v_a ≥ 15 m/s | 1.42 | 0.47 Hz | 0.17 – 5.9 | 70.9° | ~3.9 s |
| Entry, v_a = 5 m/s | 0.28 | ~0 Hz (gravity pole) | 0.75 – 1.33 | 16.1° | 5.9 s |

Full gain at `v_a = v_app_ref = 27 m/s`, over depower:

| Depower | c1 [1/m] | Delay [s] | α | Critical frequency | Delay margin |
|---|---|---|---|---|---|
| 0.250 | 0.270 | 0.383 | 0.034 | 0.85 Hz | 27 ms |
| 0.275 | 0.245 | 0.417 | 0.080 | 0.77 Hz | 76 ms |
| 0.300 | 0.217 | 0.450 | 0.161 | 0.72 Hz | 175 ms |
| 0.325 | 0.190 | 0.483 | 0.259 | 0.66 Hz | 314 ms |
| 0.350 | 0.166 | 0.517 | 0.351 | 0.60 Hz | 459 ms |
| 0.375 | 0.144 | 0.567 | 0.436 | 0.54 Hz | 605 ms |
| 0.400 | 0.128 | 0.600 | 0.518 | 0.51 Hz | 748 ms |

The disk-margin phase margin is the phase margin the loop keeps when gain and
phase vary together, so it is smaller than the classic one. The classic phase
margin of the nominal pattern loop is about 14°, at a crossover of 4.3 rad/s.
The classic gain margin is also about 1.06, because the Nyquist curve passes
within 0.06 of −1 at 4.84 rad/s. The sensitivity peak is |S| ≈ 17.

## Observations

1. **Above `v_app_min` the margin does not depend on `v_a`.**
   - The schedule `K ∝ 1/v_a` cancels the `v_a` in the plant gain, so the loop
     gain is `heading_p · v_app_ref · c1`.
   - Only below `v_app_min`, where the schedule is clamped, does the loop gain
     fall, and the margin grows.
2. **Depower drives the margin.** Lower depower raises `c1` (more loop gain) and
   also shortens the delay. On balance the gain effect wins, so the margin is
   worst at the depower flown in the pattern (0.25 – 0.28).
3. **The entry phases are robust** thanks to `entry_gain = 0.25`, except at very
   low `v_a`. There, the gravity pole competes with the reduced loop gain. That
   is a low-frequency limit, not an oscillation.
4. **What to expect in flight if the model holds:** a lightly damped
   steering/heading oscillation near 0.8 Hz in the pattern, and extreme
   sensitivity to any extra delay or gain (±6 %).

## Caveats

- The identified `delay` (0.38 – 0.60 s) is modelled as **pure dead time**. If
  it is partly a lag (e.g. first-order tape or roll dynamics), the real phase
  loss near 0.8 Hz is smaller and the margins larger. This is the dominant
  uncertainty.
- The heading/course blend (`v_kite_heading`/`v_kite_course`) is not modelled.
  The loop treats the feedback angle as the turn-rate law's ψ.
- The analysis is linear and ignores the `max_steering` clamp, which in the
  pattern limits any growing oscillation.
- The model contains neither the feed-forward (`u_ff`, `chi_ff`) nor the
  guidance (the attractor outer loop). Neither changes the inner-loop margins,
  but the outer loop adds its own dynamics.

## Delay vs. lag

A **delay** (dead time) shifts the signal later in time without changing its
shape. A **lag** (first-order low-pass) smears it out: the output starts
responding immediately but only follows gradually.

**Delay (dead time) τ**
- Transfer function: `e^(−sτ)`
- Step response: nothing happens for τ seconds, then the full step appears
  unchanged.
- Gain `|G| = 1` at all frequencies, so it doesn't attenuate anything.
- Phase `−ωτ`, which **grows without limit** as frequency rises.

**Lag (first-order, time constant T)**
- Transfer function: `1/(1 + sT)`
- Step response: starts moving at once and follows an exponential, reaching 63 %
  after T.
- Gain falls above `ω = 1/T`, so high frequencies are damped.
- Phase `−atan(ωT)`, which **never exceeds −90°**.

**How this affects the course loop.** At the critical frequency, about 0.8 Hz
(ω ≈ 5 rad/s), with 0.42 s of apparent delay:

| Model of the 0.42 s | Phase loss at 5 rad/s | Gain at 5 rad/s |
|---|---|---|
| Pure dead time | −ωτ = −120° | 1.0 |
| Pure lag, T = 0.42 s | −atan(2.1) = −65° | 0.43 |

So a lag costs about half as much phase and also cuts the loop gain. Both effects
move the Nyquist curve away from −1. That is why the delay model is the dominant
uncertainty in this analysis: the same 0.42 s read as dead time gives a disk
margin of 0.06, but as a lag the loop would be comfortably stable.

**Which one the kite has.** Physically it is probably a mix:
- **True dead time** comes from things like the sample-and-hold and the steering
  actuator's update.
- **Lag-like behaviour** comes from the tape or motor ramping and the kite
  needing time to roll and build up the turn.

A relay sweep that fits only "c1 plus a delay" can't tell the two apart. It
absorbs everything into one number, much like reading a step response by eye as
"starts responding after ≈ 0.4 s".

You can separate them with a step or chirp in `rel_steering` and a fit of

    ψ(s)/u_s(s) = c1·v_a·e^(−sτ) / (s·(1 + sT))

That fit gives τ and T separately, and it is the natural refinement for
`build_turn_rate_table.jl`. The plant in `stability_course_controller.jl` would
then gain a `1/(1 + sT)` factor, with the remaining dead time `τ < delay`.

## Next steps

1. **Validate the plant model.**
   - Compare the model's closed-loop step response (or the Bode plot of `L`)
     with a simulated heading step at depower 0.27.
   - Look in existing `simple_fig8.jl` logs for steering oscillation at ~0.8 Hz
     (FFT of `rel_steering` in the settled window).
   - If the sim is much better damped than predicted, re-identify the delay as
     dead time + lag in `build_turn_rate_table.jl`, and use that in the plant.
2. **If the model holds, add a tuning sweep to the script** over `heading_p`,
   `heading_d` and `heading_d_n`, to find settings with α ≥ 0.5 at depower
   0.25 – 0.28. Candidates:
   - lower `heading_p`;
   - more derivative phase lead near 0.8 Hz (larger `heading_d_n`, adjusted
     `heading_d`);
   - scheduling the gain on `c1` (as `gain_scale` does in
     `simple_opt_reelout.jl`), so the loop gain is constant over depower.
3. Cross-check any change against the fig-8 criteria in `fig8_tuning_log.md`:
   lower gain costs tracking.

## Usage

    include("examples/stability_course_controller.jl")   # from the package root, examples env
    diskmargin(L)                                        # nominal pattern loop

Set `SHOW_PLOTS = false` before the include to skip the Bode plot and the
margin-vs-depower plot. The script needs `ControlSystemsBase` and
`RobustAndOptimalControl`, which were added to `examples/Project.toml` for it.
