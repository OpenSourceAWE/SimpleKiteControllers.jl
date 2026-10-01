```@meta
CurrentModule = SimpleKiteControllers
```

# API

Docstrings of all exported types, functions and constants, grouped as in the module's export list.

## Figure-of-eight inner-loop (course) controller

```@docs
CourseController
CourseControllerSettings
calc_steering
set_phase!
```

## Figure-of-eight guidance

```@docs
FigureEightSettings
FigureEightController
figure_eight_path
calc_attractor
navigate_fig8
set_path_center!
set_path!
resample_path
prepare_path
blend_paths
lobe_lift
azimuth_frac
azimuth_bin
path_tangent
path_distance
path_turn_rate
path_chord_offset
attractor_index
path_normal
signed_cross_track
min_turn_radius
path_min_radius
path_radius_profile
check_pattern_feasible
path_min_height
check_pattern_height
pattern_size_growth
```

## Turn-rate-law lookup table

```@docs
V3_TURN_RATE_COEFFS
turn_rate_coeffs
turn_rate_depower_range
V3_TURN_RATE_C1
V3_TURN_RATE_C2
reload_turn_rate_table!
try_turn_rate_coeffs
stack_fits
```

## Wind-speed-dependent winch law lookup table

```@docs
winch_f_low
winch_force_limit
winch_table_lookup
winch_table_select
```

## Figure-of-eight run metrics

```@docs
fig8_metrics
print_fig8_metrics
reelout_power
reelout_ringing
unwrap_angle
unwrap_onto
winch_state_pct
lap_durations
on_log
weighted_prediction
```

## Commented run summaries

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

## Flight-controller settings

```@docs
FC_Settings
winch_force_gains
project_file
fc_settings
load_yaml_fields!
apply_overrides!
attractor_distance
guidance_rate
wind_schedule
apply_wind_schedule!
turn_rate_coeffs_file
winch_table_file
traj_opt_settings_file
```

## Externally optimized flight path

```@docs
TrajOptSettings
turn_radius_lap_reelout
```

## Reel-out feasibility gates

```@docs
ReeloutFeasibility
Phase5MarginState
c1_at
phase5_margin
check_reelout_feasibility
check_startup_path
```

## Simulated-time budget of a reel-out run

```@docs
reelout_budget
sim_budget
BUDGET_HEIGHT
```

## Run inputs and their defaults

```@docs
run_input_defaults
```

## Winch settings, length loop and controllers

```@docs
load_wc_settings
build_winch
build_controllers
```

## Parallel shape optimization

```@docs
OptSettings
opt_grid
task_key
pattern_margin
filter_grid
with_file_lock
init_results_file
record_result!
load_results
claim_task!
release_claims!
reset_claims!
n_unclaimed
run_metrics
side_conditions
rank_results
unique_results
format_results_table
```

## Linear course-loop model

The transfer functions of the stability analysis (`examples/stability_*.jl`) need `using ControlSystemsBase`, which loads the package extension.

```@docs
kite_dead_time
pattern_dead_time_lag
plant_coeffs
dead_time_fraction
c2_at
course_pid
turn_rate_plant
delay_margin
guidance_tf
kite_correction
frd_margins
frd_diskmargin
rate_disk_margin
load_course_correction
course_correction
```

## Data

```@docs
skc_data_path
```

## State of the example menu (`data/gui.yaml`)

```@docs
gui_state_file
ensure_gui_state_file
read_gui_field
write_gui_field
default_project
default_reelout_project
default_plots
selected_project
selected_reelout_project
selected_fig8_project
selected_sim_time
selected_plots
selected_windspeed
selected_turbulence
set_selected_project
set_selected_sim_time
set_selected_plots
set_selected_windspeed
scenario_site
selected_scenarios_dir
apply_windspeed_override!
```

## Caller inputs of the example scripts

```@docs
run_example
script_inputs
muted
latest_global
first_error_line
```

## A reel-out run: state and loop

```@docs
RunState
step_commands!
record_step!
check_overspeed
apply_optimized_kv!
RunSetup
```

## Run log and startup retries

```@docs
with_run_log
startup_ladder_report
ladder_line
startup_log_lines
```
## Internals

Non-exported names that the docstrings above refer to.

```@docs
SimpleKiteControllers.BUDGET_KNOT
SimpleKiteControllers.C3
SimpleKiteControllers.KITE_DEAD_TIME_EXP
SimpleKiteControllers.PATTERN_DELAY_REF
SimpleKiteControllers.PATTERN_DEPOWER_EXP
SimpleKiteControllers.PATTERN_V_FLOOR
SimpleKiteControllers.PLANT_COEFFS
SimpleKiteControllers.PLANT_SPLIT
SimpleKiteControllers.RETIRED_YAML_KEYS
SimpleKiteControllers.merge_into!
SimpleKiteControllers.setup_run
SimpleKiteControllers.solve_startup_path!
SimpleKiteControllers.startup_feasibility
SimpleKiteControllers.steering_command!
SimpleKiteControllers._dist
SimpleKiteControllers._path_geometry
```
