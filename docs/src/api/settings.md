```@meta
CurrentModule = SimpleKiteControllers
```

# Settings and data files

## Externally optimized flight path

```@docs
TrajOptSettings
turn_radius_lap_reelout
```

## Loading settings

```@docs
load_yaml_fields!
apply_overrides!
set_fc_field!
get_fc_field
```

## Project and data files

```@docs
skc_data_path
project_file
turn_rate_coeffs_file
course_loop_model_file
kite_correction_file
winch_table_file
traj_opt_settings_file
```

## State of the example menu

The choices of the `examples/select_*.jl` menus, persisted in `data/gui.yaml`.

### Reading and writing the file

```@docs
gui_state_file
ensure_gui_state_file
read_gui_field
write_gui_field
```

### Defaults

```@docs
default_project
default_reelout_project
default_plots
```

### Current selection

```@docs
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
apply_windspeed_override!
```

### Scenario archive

```@docs
scenario_site
selected_scenarios_dir
```
