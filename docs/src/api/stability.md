```@meta
CurrentModule = SimpleKiteControllers
```

# Course-loop stability

The linear model behind `examples/stability_*.jl`. The transfer functions need
`using ControlSystemsBase`, which loads the package extension.

## Plant

```@docs
kite_dead_time
pattern_dead_time_lag
plant_coeffs
dead_time_fraction
c2_at
turn_rate_plant
```

## Controller and guidance

```@docs
course_pid
guidance_tf
kite_correction
load_course_correction
course_correction
```

## Margins

```@docs
delay_margin
frd_margins
frd_diskmargin
rate_disk_margin
```
