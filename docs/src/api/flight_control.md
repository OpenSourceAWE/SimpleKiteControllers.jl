```@meta
CurrentModule = SimpleKiteControllers
```

# Flight control

The two loops that steer the kite: the figure-of-eight guidance turns the kite's position
into a commanded course, the course controller turns that course into steering and depower.

```@docs
FC_Settings
FC_Course
FC_FeedForward
FC_Pattern
FC_WindRamp
FC_Winch
FC_Reelout
FC_Run
fc_settings
wind_schedule
apply_wind_schedule!
```

## Course controller

The inner loop, with the entry state machine (park, dive, hold, transition, figure-of-eight).

```@docs
CourseController
CourseControllerSettings
calc_steering
set_phase!
```

## Figure-of-eight guidance

The outer loop: an attractor point runs ahead of the kite along the reference path.

```@docs
FigureEightSettings
FigureEightController
navigate_fig8
calc_attractor
attractor_index
signed_cross_track
path_chord_offset
set_path_center!
set_path!
attractor_distance
guidance_rate
```

## Turn-rate law

The identified turn-rate coefficients `c1`, `c2` and their lookup table.

```@docs
V3_TURN_RATE_COEFFS
V3_TURN_RATE_C1
V3_TURN_RATE_C2
turn_rate_coeffs
try_turn_rate_coeffs
turn_rate_depower_range
reload_turn_rate_table!
stack_fits
```
