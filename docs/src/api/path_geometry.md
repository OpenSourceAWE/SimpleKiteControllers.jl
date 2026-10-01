```@meta
CurrentModule = SimpleKiteControllers
```

# Flight-path geometry

Reference paths are closed curves in azimuth and elevation [deg].

## Building a path

```@docs
figure_eight_path
resample_path
prepare_path
blend_paths
lobe_lift
```

## Querying a path

```@docs
azimuth_frac
azimuth_bin
path_tangent
path_normal
path_distance
path_turn_rate
```

## Checking a pattern

Whether a path can be flown: its turn radius, its height above ground and its size
relative to the path it replaces.

```@docs
min_turn_radius
path_min_radius
path_radius_profile
check_pattern_feasible
path_min_height
check_pattern_height
pattern_size_growth
```
