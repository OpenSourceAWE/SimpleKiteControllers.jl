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

## TODO
- add examples for using the parking controller
- implement a new path following controller for flying figures of eight, see [Fernandes_2022](https://www.mdpi.com/1996-1073/15/4/1390)
