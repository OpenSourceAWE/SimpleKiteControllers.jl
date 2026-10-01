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
each (`body_damping`, `depower`) cell it settles the kite, holds a constant tether length,
oscillates the heading with a relay and a stepped steering amplitude, and fits the turn-rate law,
dead time and lag on the log. It flies this package's project, so the sweep sees the same
plant the runs do, and it rewrites the file after every cell, so a diverged run costs only one cell.

### [`plot_c1_c2.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_c1_c2.jl)
Plots `c1` and the steering delay, split into dead time and lag, against the depower with error
bars, one figure per `body_damping` in the table. `c2` is not plotted, because the relay sweep
cannot identify it.

### [`plot_turn_rate_identification.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_turn_rate_identification.jl)
Identifies the turn-rate law at a low elevation, where the kite flies in the pattern. Instead of
relaying about straight up near the zenith, as the table's sweeps do, each flight relays about a
crosswind heading and reverses at a given azimuth, so the kite flies a lazy-eight-like pattern low
in the wind window at 20 – 50 m/s of apparent wind. Each flight is fitted on its own and all
steady flights together, and turn rate, apparent wind speed, kite speed and elevation are plotted
over time.

### [`plot_turn_rate_vs_depower.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_turn_rate_vs_depower.jl)
Runs the low-elevation identification of `plot_turn_rate_identification.jl` at every depower of
the table and plots `c1`, `c2`, the dead time and the lag over the depower with error bars. The
fits are saved to `output/turn_rate_low_flights.csv` after every depower, so the plot can be
redrawn without flying again.

### [`plot_relay_low_elevation.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_relay_low_elevation.jl)
Draws the relay excitation of one low-elevation identification flight as a figure for the paper:
the heading with the edges of the relay band, the elevation, and the commanded, actual and
delay-shifted steering.

### [`gravity_term_form.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/gravity_term_form.jl)
Two checks on saved data, without flying. First, it fits the gravity term of the turn-rate law as
`c_g·sin(ψ)·cos(β)·v_a^(−n)` for `n` from 0 to 2 to find which power of the apparent wind speed
fits best. Second, it checks whether the linear pattern model with the identified coefficients is
still conservative at the operating points where the stability margins were measured.

### [`delay_lag_fit.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/delay_lag_fit.jl)
A helper, included by `build_turn_rate_table.jl` and `stability_opt_reelout.jl`, that splits the
kite's response to the applied steering into a dead time and a first-order lag by fitting on a grid.

## Validation of the course-loop model

### [`validate_margins.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/validate_margins.jl)
Pushes the simulated course loop until it rings, by raising the steering gain or adding steering
delay, and compares the critical gain factor and extra delay, and their ringing frequencies, with
the gain margin, delay margin and crossover frequencies of the linear model of the course loop.

### [`plot_frf_validation.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_frf_validation.jl)
Plots the frequency response of the course loop measured by injection in `simple_fig8.jl` over
the Bode plot of the linear model at the same operating point, one column per tether length
(150, 200 and 300 m), and writes `docs/course_loop_frf.png`.

### [`replay_prediction.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/replay_prediction.jl)
Passes the logged steering command of held-out logs through the plant model of the course loop,
with its parameters updated from the log every sample, and compares the predicted heading with
the flown one k steps ahead.

### [`xtrack_step_analysis.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/xtrack_step_analysis.jl)
Evaluates cross-track step tests of the guided course loop: the measured response of the
cross-track error to a step of the attractor offset, at constant length and during the reel-out,
fitted with a second-order model and compared with the closed-loop model of
`stability_opt_reelout.jl`. The measured part works on the saved tests in `data/steptest/`
without flying. This analysis is work in progress.
