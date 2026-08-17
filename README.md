# SimpleKiteControllers

[![Build Status](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)

## Introduction
This package provides:
- a path following figure of eight controller
- a client for the [AWETrim](https://github.com/awegroup/AWETrim) reelout flight-path optimizer

Planned:
- a controller for flying circles
- a parking controller, that keeps the nose of the kite pointing into the wind and thus keeps it in a steady state airborne as long as there is sufficient wind

## This package provides
- the figure-of-eight path-following guidance: the types `FigureEightController` and
  `FigureEightSettings` and the functions `figure_eight_path`, `calc_attractor`,
  `navigate_fig8`, `set_path_center!`, `path_tangent`
- the figure-of-eight inner loop: the types `CourseController` and
  `CourseControllerSettings`, driven by `calc_steering` and `set_phase!` — the
  heading/course PID, entry state machine and `rel_depower`, shared by all three
  `examples/simple_fig8*.jl` scripts
- the curvature feasibility check `check_pattern_feasible` (with `min_turn_radius`,
  `path_min_radius`, `path_radius_profile`) — a pattern tighter than the kite's minimum
  turn radius cannot be tracked at any PID tuning, so this is worth running before a
  simulation, not after
- `turn_rate_coeffs`, the identified turn-rate-law coefficients `c1`, `c2` and steering
  `delay` the feasibility check needs, interpolated in depower from
  `data/turn_rate_coeffs.yaml`
- `fig8_metrics` / `print_fig8_metrics`, headless quality metrics for a flown run
- `FC_Settings`, every tuning parameter of a figure-of-eight run, loaded from
  `data/fc_settings.yaml`
- the types `ParkingController` and `ParkingControllerSettings` and the functions
  `linearize`, `calc_steering`, `navigate` — the NDI/turn-rate building blocks for the
  planned parking controller; unit-tested, but not yet driven end-to-end in an example
  (see TODO)

## Examples

`examples/simple_fig8.jl` flies the figure-of-eight controller on the TU Delft V3 kite,
using [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) as the plant.

<p align="center">
  <img src="https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/V3_Kite_system_fig8_200m_pattern.png" width="60%" alt="V3 Kite flying a 200 m figure-of-eight pattern">
</p>

For easy use of the examples and scripts it is suggested to install the package using git:

```bash
git clone https://github.com/OpenSourceAWE/SimpleKiteControllers.jl
cd SimpleKiteControllers.jl/bin
./install
./create_sys_image
cd ..
./bin/run_julia
```
The step `create_sys_image` is not strictly needed and takes 15-60 min. Skip it if you are short of time. 

Optionally you can also install the flight path optimizer with the command:
```
./bin/install_awetrim
```
and start it in a second terminal window:
```
./bin/run_server
```

Then, from a Julia REPL in this repository:

```julia
menu()
```
This function will show the following menu:

```text
Choose example to run or `q` to quit: 
 > select_turbulence.jl    - choose the turbulence level init() applies (default or 0.0…1.0)
   select_project.jl       - choose which system project (150m/200m/300m) to fly
   select_sim_time.jl      - choose the simulation time (default or a specific value)
   select_plots.jl         - choose figures: pattern/3d path/time series/power/aerodynamics
   simple_fig8.jl          - fly the figure-of-eight pattern (minutes!)
   simple_fig8_live.jl     - the same run, shown live in the 3D viewer (minutes!)
   simple_fig8_plots.jl    - plot the last logged run of active project
   simple_reelout.jl       - fly the pattern, then reel out to reelout_l_max (minutes!)
   simple_reelout_plots.jl - plot the last logged reel-out run
   simple_reelout_play.jl  - replay the last logged reel-out run in the 3D viewer
   optimize_fig8.jl        - sweep the pattern shape in parallel processes (HOURS!)
   optimize_path.jl        - Julia client for the AWETrim reelout flight-path optimizer
   export_v3_segments.jl   - write the V3 segment table to output/v3_segments.csv
   quit
```
The menu shows ten entries at a time and scrolls; the four `select_*` entries change the
simulation settings, which are persisted to `data/gui.yaml` and read fresh by every run
rather than cached in a REPL global.

The runs themselves come in two families. `simple_fig8.jl` flies the pattern at constant
tether length and plots the results when it is done; `simple_fig8_live.jl` is the same
run shown in the 3D viewer while it flies, and `simple_fig8_plots.jl` re-plots a log that
is already on disk. `simple_reelout.jl` flies the same entry and pattern but reels the
tether out under load until `reelout_l_max`, with `simple_reelout_plots.jl` and
`simple_reelout_play.jl` for its logs. Both families write an Arrow log to `output/`,
named after the active project's `log_file` setting.

`optimize_fig8.jl` sweeps the pattern shape by driving `simple_reelout.jl` in parallel
worker processes — hours, not minutes, and resumable. `optimize_path.jl` is the separate
AWETrim client described above.

## Documentation

- [docs/control_algorithm.md](docs/control_algorithm.md) — how the controller works, from the
  optimal trajectory through path following and the steering set point to the reel-out speed,
  plus what is and is not verified by the test suite
- [docs/thesis.md](docs/thesis.md) — the heading/course fusion ψ' in detail, and how it differs
  from the reference formulation
- [docs/reelout_state_machine.md](docs/reelout_state_machine.md) — the flight phases and winch
  states of `examples/simple_reelout.jl`, with the transition conditions
- [docs/fig8_tuning_log.md](docs/fig8_tuning_log.md) — the dated record of the parameter
  experiments behind the shipped tuning, including which levers turned out to be dead ends
- [docs/sysimage_notes.md](docs/sysimage_notes.md) and
  [docs/ScratchUsage.md](docs/ScratchUsage.md) — startup cost and where the generated model and
  settling caches land
- [docs/TrajectoryOptimization.md](docs/TrajectoryOptimization.md) — notes on the trajectory
  optimization test cases

## TODO
- add examples for using the parking controller

## Related
A fully working set of flight path controllers and planners can be found here: [KiteControllers.jl](https://github.com/aenarete/KiteControllers.jl)
