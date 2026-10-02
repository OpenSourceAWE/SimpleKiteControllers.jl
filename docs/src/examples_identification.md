# Examples - identification

The controllers on the other pages rely on a model of how the kite turns: the turn-rate law
`ψ̇ = c1·v_a·u_s + c2/v_a·sin(ψ)·cos(β)`, with the steering `u_s`, the apparent wind speed `v_a`
and the elevation `β`, and the dead time and lag between the commanded steering and the turn
rate. The scripts on this page identify these coefficients from simulated flights of the V3 kite,
and check the linear course-loop model used by the stability analyses against flown logs.
How to install and start the examples is described on
[Examples - general](examples_general.md).

## Turn-rate law

### [`build_turn_rate_table.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/build_turn_rate_table.jl)
Fills `data/turn_rate_coeffs.yaml`, the table [`turn_rate_coeffs`](@ref) interpolates in. For
each depower it flies three relay flights low in the wind window, at fixed steering amplitudes:
each flight relays about a crosswind heading, reverses at a given azimuth and holds an elevation
of about 30°, so the kite flies a lazy-eight-like pattern at 20 – 50 m/s of apparent wind, as in
its figures of eight. The turn-rate law, dead time and lag are fitted on the steady flights
together. It flies this package's project, so the identification sees the same plant the runs
do, and it rewrites the file after every depower, so a diverged run costs only one cell.

### [`plot_c1_c2.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_c1_c2.jl)
Plots `c1` and the steering delay, split into dead time and lag, against the depower with error
bars, one figure per `body_damping` in the table.

### [`plot_turn_rate_identification.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_turn_rate_identification.jl)
Flies the low-elevation flights of `build_turn_rate_table.jl` at one depower, without writing
the table, and compares the fit with the table's row and with an extended law. Each flight is
fitted on its own and all steady flights together, and turn rate, apparent wind speed, kite speed
and elevation are plotted over time.

### [`plot_turn_rate_vs_depower.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_turn_rate_vs_depower.jl)
Runs the low-elevation identification of `plot_turn_rate_identification.jl` at every depower of
the table and plots `c1`, `c2`, the dead time and the lag over the depower with error bars. The
fits are saved to `output/turn_rate_low_flights.csv` after every depower, so the plot can be
redrawn without flying again.

### [`plot_relay_low_elevation.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_relay_low_elevation.jl)
Draws the relay excitation of one low-elevation identification flight as a figure for the paper:
the heading with the edges of the relay band, the elevation, and the commanded, actual and
delay-shifted steering.

## Re-identifying after a change of the kite

The stability analyses use two data files with identified values, both named in the system
project, so a changed kite can get its own copies:

- `turn_rate_coeffs` → `data/turn_rate_coeffs.yaml`, the turn-rate law over depower (`c1`,
  `c2`, dead time and lag), read by [`turn_rate_coeffs`](@ref);
- `course_loop_model` → `data/course_loop_model.yaml`, the other parameters of the linear
  course-loop model, read into [`CourseLoopModel`](@ref).

After a change of the kite (mass, geometry, bridle, damping, aerodynamics), copy both files
under new names, enter the new names in the `system:` section of the kite's project, select
that project and re-identify in the order below; each step uses the results of the steps
before it. Update the comment above each value in `course_loop_model.yaml` with where the new
value came from. The steering tape's lag needs no identification: it is `1/steering_gain`
of the KCU's P controller, from the project's settings file. Only the first step has a script that writes its file; for the others the
scripts fly and measure, and the fit is done in the REPL.

1. **The turn-rate law.** Run `build_turn_rate_table(remake = true)` from
   `build_turn_rate_table.jl`; it rewrites the turn-rate table of the selected project. Check
   the result with `plot_c1_c2.jl`.

2. **`kite_dead_time_exp` and `kite_lag_exp`**, how the kite's dead time and lag scale with
   the apparent wind speed. Run `plot_turn_rate_identification.jl` at one depower and two
   wind speeds, for example `run_example("plot_turn_rate_identification.jl"; v_wind = 6.5)`
   and the same at the table's wind speed. It prints the joint dead time `τ` and lag `T`
   of each run and the apparent wind speed `v_a` the flights flew at. Then
   `exp = log(x₁/x₂) / log(v_a₂/v_a₁)` for `x` = `τ` and `x` = `T`.

3. **The pattern law** (`pattern_delay_ref`, `pattern_v_ref`, `pattern_delay_exp`,
   `pattern_v_floor`), the kite's response time `τ + T` in pattern flight. Fly
   `simple_fig8.jl` with the projects `system_fig8_150m`, `system_fig8_200m` and
   `system_fig8_300m` at several wind speeds, and a weak-wind `simple_opt_reelout.jl` run, so
   that `v_a` spans about 13 – 40 m/s. Keep only runs whose logged depower is
   `pattern_law_depower` (the `depower_setpoint` of the figure-of-eight projects; their
   `wind_ramp` section changes it above `wind_ramp_low`). On each
   log, identify the pure delay with `identify_turn_rate_law` (V3Kite) on phase 4 from 15 s
   after its start. Fit `delay = pattern_delay_ref · (pattern_v_ref / v_a)^pattern_delay_exp`
   with `pattern_v_ref` a typical `v_a` of the figure of eight, and set `pattern_v_floor` to
   the lowest `v_a` measured.

4. **`pattern_depower_exp`**, the growth of the response time with depower. Fly point `D`
   of `validate_margins.jl` (`system_fig8_300m`, 7 m/s) with `depower_setpoint` raised in
   steps (0.30, 0.33, 0.36 last time), identify the delay as in step 3 and divide it by the
   pattern law at the same `v_a`. Fit `exp(pattern_depower_exp · (depower −
   pattern_law_depower))` to these ratios, and restore `depower_setpoint` afterwards.

5. **`kite_corr_zero` and `kite_corr_pole`**, the lag-lead [`kite_correction`](@ref). Fly
   point `D` with a multisine on the steering command, `run_v1(:D; injection = Multisine())`
   in `validate_margins.jl`, with `v_steering` raised to 1.0 s⁻¹ in the project's settings
   file for these runs only, so the tape stays off its rate limit. `frf_injection` gives the
   command → heading response at each line. Divide it by the model's tape lag × turn-rate law
   with the pattern law's dead time and lag ([`turn_rate_plant`](@ref),
   [`pattern_dead_time_lag`](@ref)), and fit `(1 + s/ω_z)/(1 + s/ω_p)` to the ratio over
   0.5 – 2.1 Hz. The same runs give the measured responses in
   `data/course_link_measured.csv` and `data/course_correction_measured.csv`, which have no
   script that writes them either.

Then check the model against the simulation with `validate_margins.jl` and
`plot_frf_validation.jl`: the model should stay below every measured margin.

## Validation of the course-loop model

### [`validate_margins.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/validate_margins.jl)
Pushes the simulated course loop until it rings, by raising the steering gain or adding steering
delay, and compares the critical gain factor and extra delay, and their ringing frequencies, with
the gain margin, delay margin and crossover frequencies of the linear model of the course loop.

### [`plot_frf_validation.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_frf_validation.jl)
Plots the frequency response of the course loop measured by injection in `simple_fig8.jl` over
the Bode plot of the linear model at the same operating point, one column per tether length
(150, 200 and 300 m), and writes `docs/course_loop_frf.png`.

### [`xtrack_step_analysis.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/xtrack_step_analysis.jl)
Evaluates cross-track step tests of the guided course loop: the measured response of the
cross-track error to a step of the attractor offset, at constant length and during the reel-out,
fitted with a second-order model and compared with the closed-loop model of
`stability_opt_reelout.jl`. The measured part works on the saved tests in `data/steptest/`
without flying. This analysis is work in progress.
