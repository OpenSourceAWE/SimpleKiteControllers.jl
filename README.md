# SimpleKiteControllers

[![Build Status](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/actions/workflows/CI.yml?query=branch%3Amain)

## Introduction
This package shall (in the beginning) provide two controllers:
- a parking controller, that keeps the nose of the kite pointing into the wind and thus keeps it in a steady state airborne as long as their is sufficient wind
- a path following figure of eight controller for pulling ships

All controllers shall come with an auto-tuning script for tuning the controller parameters for new kites. It might be necessary to run this script for different wind speeds and to create a lookup table that adapts the controller parameters to the wind speed.

The auto-tuning script shall optimize the controller performance in the time domain, using a cost function similar to these [Performance Indicators](https://opensourceawe.github.io/WinchControllers.jl/dev/performance_indicators/). In addition a linearized model shall be used to quantify the stability using the [diskmargin](https://juliacontrol.github.io/RobustAndOptimalControl.jl/dev/api/#RobustAndOptimalControl.diskmargin).

## This package provides
- the types `ParkingController` and `ParkingControllerSettings`
- the functions `linearize`, `calc_steering`, `navigate`
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

## Examples

`examples/simple_fig8.jl` flies the figure-of-eight controller on the TU Delft V3 kite,
using [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) as the plant. V3Kite is not
registered and has to be **writable** (it caches settled wing geometry into its own
`data/`), so clone it next to this repository before running:

```bash
git clone https://github.com/OpenSourceAWE/V3Kite.jl ../V3Kite.jl
```

Then, from a Julia REPL in this repository:

```julia
include("examples/simple_fig8.jl")
```

The script activates `examples/Project.toml` itself, saves its log as `fig8_run` and
brings the plots up at the end. Every parameter of the run is a field of `FC_Settings`;
set `fcs` in the REPL before the `include` to override any of them. A 150 s run is
several tens of thousands of `step!` calls and takes many minutes of wall time.

## Installation
<details>
  <summary>Installation of Julia</summary>

If you do not have Julia installed yet, please read [Installation](https://github.com/aenarete/KiteSimulators.jl/blob/main/docs/Installation.md).

</details>

<details>
  <summary>Installation as package</summary>

### Installation of SimpleKiteControllers as package

It is suggested to use a local Julia environment. You can create it with:
```bash
mkdir myproject
cd myproject
julia --project=.
```
(don't forget typing the dot at the end), and then, on the Julia prompt enter:
```julia
using Pkg
pkg"add https://github.com/OpenSourceAWE/SimpleKiteControllers.jl"
```
You can run the tests with:
```julia
using Pkg
pkg"test SimpleKiteControllers"
```
</details>

## TODO
- add examples for using the parking controller
- add the auto-tuning script described above
- the figure-of-eight path follower ([Fernandes_2022](https://www.mdpi.com/1996-1073/15/4/1390))
  is implemented, but the entry state machine, the winch force mode and the heading PID
  still live in `examples/simple_fig8.jl` rather than in a controller type

## Related
A fully working set of flight path controllers and planners can be found here: [KiteControllers.jl](https://github.com/aenarete/KiteControllers.jl)
