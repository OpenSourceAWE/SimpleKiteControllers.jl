# Plan next steps

## Step 1 - DONE -

The log files in arrow format must be written into the output folder. Create an output folder relative to the repo root if it does not exist. Add this folder to .gitignore

As file name for the log files the field log_file defined in settings_xxx.yaml shall be used.

## Step 2 - DONE -

Configure Revise such that it revises structs.

Enabling struct revision

On Julia 1.12+, Revise can automatically revise struct definitions in a running session. This feature requires scanning the global method table and type hierarchy at startup, which can be slow so it's disabled by default. If you would like to enable it you can set the revise_structs preference to true via Preferences.jl.

Add the following to the LocalPreferences.toml file in your active project:

[Revise]
revise_structs = true

## Step 3 - DONE -

Write a script bin/copy_model that copies the file model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin  
from ../V3Kite/data/model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin to 
examples/cache
and in addition copies the system image ../V3Kite/bin/kps-image-1.12.so to
the folder bin of this package.
Check first if both files exist, and copy them only if both exist.

## Step 4 - DONE -

- Add a script menu.jl to the examples folders that allows to run simple_fig8.jl and 
simple_fig8_plots.jl
- Add a function menu() to run_julia

## Step 5 - DONE -

In KiteControllers there is the first menu entry select_project. Can you add this to SimpleKiteControllers? Currently, there are three projects (system.yaml files) in the data folder.

## Step 6 Refactoring - DONE -

Never use @isdefined in example scripts for model parameters that are set in the script.
Reason: If I manually include an example script, it shall always read the latest changes
from the yaml files.

## Step 7 Add menu entry select_sim_time - DONE -

Add a menu entry select_sim_time. It should allow the options:

- default
- a specific value in seconds

The specific value should be entered numerically. The option default means to use
the project specific default time.

Store this value in menu_state.yaml

## Step 8 Add menu entry select_plots - DONE -

Add a menu entry select_plots, that displays a checkbox menu with the following check boxes:

- pattern
- time series
- aerodynamics

Store these values in menu_state.yaml

simple_fi8_plots.jl shall then show only the selected plots.
