# SimpleKiteControllers

[![Build Status](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)

## Introduction
This package provides:
- a path following figure of eight controller

Planned:
- a controller for flying circles
- a parking controller, that keeps the nose of the kite pointing into the wind and thus keeps it in a steady state airborne as long as their is sufficient wind

## This package provides
- the figure-of-eight path-following guidance: the types `FigureEightController` and
  `FigureEightSettings` and the functions `figure_eight_path`, `calc_attractor`,
  `navigate_fig8`, `set_path_center!`, `path_tangent`
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
using [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) as the plant. V3Kite is not
registered and has to be **writable** (it caches settled wing geometry), so clone it next to this repository before running:

```bash
git clone https://github.com/OpenSourceAWE/V3Kite.jl V3Kite
cd V3Kite/bin
./install
./create_sys_image
cd ../../SimpleKiteControllers
./bin/run_julia
```

Then, from a Julia REPL in this repository:

```julia
menu()
```

## TODO
- add examples for using the parking controller
- the figure-of-eight path follower ([Fernandes_2022](https://www.mdpi.com/1996-1073/15/4/1390))
  is implemented, but the entry state machine, the winch force mode and the heading PID
  still live in `examples/simple_fig8.jl` rather than in a controller type

## Related
A fully working set of flight path controllers and planners can be found here: [KiteControllers.jl](https://github.com/aenarete/KiteControllers.jl)
