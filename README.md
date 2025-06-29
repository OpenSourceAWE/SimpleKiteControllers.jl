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

## TODO
- add examples for using the parking controller
- implement a new path following controller for flying figures of eight, see [Fernandes_2022](https://www.mdpi.com/1996-1073/15/4/1390)
