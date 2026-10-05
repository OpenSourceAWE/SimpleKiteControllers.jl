# Examples - identification

The controllers on the other pages rely on a model of how the kite turns: the turn-rate law
`ψ̇ = c1·v_a·u_s + c2/v_a·sin(ψ)·cos(β)`, with the steering `u_s`, the apparent wind speed `v_a`
and the elevation `β`, and the dead time and lag between the commanded steering and the turn
rate. The scripts on this page identify these coefficients from simulated flights of the V3 kite,
and check the linear course-loop model used by the stability analyses against flown logs.
How to install and start the examples is described on
[Examples - general](examples_general.md). `menu2()`, in a REPL started with `bin/run_julia`,
offers the project selection, `build_turn_rate_table.jl`, `plot_c1_c2.jl`,
`identify_kite_delay_scaling.jl`, `identify_pattern_law.jl`, `identify_depower_factor.jl`,
`identify_kite_correction.jl` and, to check the result, `stability_opt_reelout.jl`.

## Turn-rate law

### [`build_turn_rate_table.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/build_turn_rate_table.jl)
Fills `data/turn_rate_coeffs.yaml`, the table [`turn_rate_coeffs`](@ref) interpolates in. For
each depower it flies three relay flights low in the wind window, at fixed steering amplitudes:
each flight relays about a crosswind heading, reverses at a given azimuth and holds an elevation
of about 30°, so the kite flies a lazy-eight-like pattern at 20 – 50 m/s of apparent wind, as in
its figures of eight. The turn-rate law, dead time and lag are fitted on the steady flights
together. It flies the selected project, so the identification sees the same plant the runs
do, and it rewrites the file after every depower, so a diverged run costs only one cell. Running
the script re-identifies every depower; `run_example("build_turn_rate_table.jl"; remake = false)`
flies only the missing or failed ones.

### [`identify_kite_delay_scaling.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/identify_kite_delay_scaling.jl)
Flies the low-elevation flights of `build_turn_rate_table.jl` at one depower and two or more
wind speeds, fits how the kite's dead time and lag scale with the apparent wind speed,
`x ∝ v_a^-exp`, and writes the two exponents, with their provenance, into the course-loop model
file of the selected project.

### [`identify_pattern_law.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/identify_pattern_law.jl)
Flies figures of eight at three tether lengths and several wind speeds and a weak-wind reel-out,
identifies the kite's response time on each log and fits the pattern law, the response time over
the apparent wind speed, which it writes into the course-loop model file of the selected project.

### [`identify_depower_factor.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/identify_depower_factor.jl)
Flies the figure of eight at 300 m and 7 m/s at several depower settings, identifies the kite's
response time on each log and fits how it grows with the depower relative to the pattern law,
which it writes into the course-loop model file of the selected project.

### [`identify_kite_correction.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/identify_kite_correction.jl)
Measures the kite's steering → heading response with a multisine injected into the steering
command of a figure of eight at 300 m, writes its ratio to the turn-rate law as a table (the
measured kite correction) and checks that the lag-lead correction in the course-loop model file
stays conservative against it.

### [`plot_c1_c2.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_c1_c2.jl)
Plots the turn-rate table: `c1`, `c2`, the dead time and the lag against the depower with error
bars, one figure per `body_damping`. It writes the paper's figure of the turn-rate law,
`turn_rate_low_pattern.pdf`.

### [`plot_turn_rate_identification.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_turn_rate_identification.jl)
Flies the low-elevation flights of `build_turn_rate_table.jl` at one depower, without writing
the table, and compares the fit with the table's row and with an extended law. Each flight is
fitted on its own and all steady flights together, and turn rate, apparent wind speed, kite speed
and elevation are plotted over time.

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
before it. The steering tape's lag needs no identification: it is `1/steering_gain` of the
KCU's P controller, from the project's settings file. Each step has a script that writes its
result and its provenance into the file.

1. **The turn-rate law.** Set `conditions: system` in the copied turn-rate table to the
   kite's project and `conditions: dt` to its time step (`1/sample_freq`), then run `build_turn_rate_table.jl`; it flies the selected project,
   re-identifies every depower and rewrites the turn-rate table the project names. Check
   the result with `plot_c1_c2.jl`.

2. **`kite_dead_time_exp` and `kite_lag_exp`**, how the kite's dead time and lag scale with
   the apparent wind speed. Run `identify_kite_delay_scaling.jl`: it flies the flights of
   step 1 at depower 0.275 and two wind speeds (6.5 and 9.51 m/s), fits `x ∝ v_a^-exp` to the
   joint dead time `τ` and lag `T` of each, and writes both exponents and their provenance
   into the course-loop model file. `run_example("identify_kite_delay_scaling.jl"; save = false)`
   only prints them.

3. **The pattern law** (`pattern_delay_ref`, `pattern_delay_exp`, `pattern_v_floor`), the
   kite's response time `τ + T` in pattern flight. Run `identify_pattern_law.jl`: it flies
   `simple_fig8.jl` at 150, 200 and 300 m and several wind speeds and a weak-wind
   `simple_reelout.jl`, so that `v_a` spans about 13 – 35 m/s, identifies the pure delay on
   phase 4 of each log with `identify_turn_rate_law` (V3Kite), fits
   `delay = pattern_delay_ref · (pattern_v_ref / v_a)^pattern_delay_exp` to the logs flown at
   `pattern_law_depower`, and writes the three values and their provenance into the
   course-loop model file. The logs are kept in `output/pattern_law/`;
   `run_example("identify_pattern_law.jl"; fly = false)` refits them.

4. **`pattern_depower_exp`**, the growth of the response time with depower. Run
   `identify_depower_factor.jl`: it flies point `D` (`system_fig8_300m`, 7 m/s) with
   `simple_fig8.jl` at depower 0.27, 0.30, 0.33 and 0.36 (the input `fcs_overrides` of
   `simple_fig8.jl`), identifies the delay as in step 3, divides it by the pattern law at the
   same `v_a`, fits `exp(pattern_depower_exp · (depower − pattern_law_depower))` to these
   ratios and writes the exponent and its provenance into the course-loop model file.

5. **The kite correction** (`kite_correction` → `kite_correction_measured.csv`, and
   `kite_corr_zero`, `kite_corr_pole`). Run `identify_kite_correction.jl`: it flies point `D`
   of `validate_margins.jl` twice with the tape's rate limit raised to 1 s⁻¹ (in the
   project's settings file, restored afterwards), a baseline for the lap period and a run
   with a multisine added to the steering command at lines halfway between the lap's
   harmonics, 0.5 – 2.1 Hz, and divides the measured tape → heading response by the
   turn-rate law with the pattern law's dead time and lag. The ratio loses gain with
   frequency without the phase lag a causal transfer function would have with it, so it is
   kept as a table: the script writes it to the project's kite-correction file, and
   `stability_opt_reelout.jl` also rates the guided loop with it. The lag-lead
   [`kite_correction`](@ref) stays as the causal stand-in in the transfer-function models;
   the script checks that it is conservative against the table (gain not lower than
   measured below 0.8 Hz, phase not less lagging from 0.8 to 1.4 Hz) and otherwise writes
   the best-fitting conservative one. `data/course_correction_measured.csv`, measured the
   same way, has no script that writes it.

Then check the model against the simulation with `measure_course_link.jl` and
`plot_frf_validation.jl`: the model should stay below every measured margin.

## Validation of the course-loop model

### [`validate_margins.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/validate_margins.jl)
Pushes the simulated course loop until it rings, by raising the steering gain or adding steering
delay, and compares the critical gain factor and extra delay, and their ringing frequencies, with
the gain margin, delay margin and crossover frequencies of the linear model of the course loop.

### [`measure_course_link.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/measure_course_link.jl)
Measures the command → course response of the course loop by multisine injection at 300, 200
and 150 m (points `D`, `A` and `F`, 7 m/s), a baseline and an injection run each with the tape's
rate limit raised, writes it to `data/course_link_measured.csv` and prints the delay and gain
margins of the loop on the measured plant next to the model's. About 15 minutes;
`run_example("measure_course_link.jl"; fly = false)` evaluates the saved runs in
`output/course_link/` again.

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
