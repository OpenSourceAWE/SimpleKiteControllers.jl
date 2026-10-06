```@meta
CurrentModule = SimpleKiteControllers
```

# Course-loop stability

The linear model behind `examples/stability_*.jl`. The transfer functions need
`using ControlSystemsBase`, which loads the package extension.

## Plant

```@docs
CourseLoopModel
course_loop_model
reload_course_loop_model!
kite_dead_time
pattern_dead_time_lag
turn_rate_plant
```

## Provenance

Which kite the turn-rate table and the course-loop model were identified on. The rating
and retuning scripts refuse a model of another kite.

```@docs
kite_fingerprint
kite_id
stale_identification_steps
check_model_provenance
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
