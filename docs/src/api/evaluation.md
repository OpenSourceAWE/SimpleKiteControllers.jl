```@meta
CurrentModule = SimpleKiteControllers
```

# Run evaluation

## Metrics of a logged run

```@docs
fig8_metrics
print_fig8_metrics
reelout_power
reelout_ringing
winch_state_pct
lap_durations
weighted_prediction
```

### Helpers

```@docs
on_log
unwrap_angle
unwrap_onto
```

## Commented run summary

The YAML file written next to the log of a run, and the archive of its input files.

```@docs
write_yaml_commented
time_keyed
package_git_state
success_verdict
simulation_block
fig8_metrics_block
reelout_block
performance_block
opt_cycle_max
run_input_files
archive_run_files
```
