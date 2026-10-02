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
