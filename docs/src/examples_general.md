# Examples - general

All examples fly the TU Delft V3 kite, using [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl)
as the plant. This page explains how to install and start them, and describes the scripts that
choose the settings of a run and the helper files the runs share. The runs themselves are on the
pages [Examples - figure-of-eight](examples_fig8.md) and [Examples - reel-out](examples_reelout.md),
the identification of the plant and the validation of its linear model on
[Examples - identification](examples_identification.md).

## Installation and the example menu

For easy use of the examples and scripts it is suggested to install the package using git:

```bash
git clone https://github.com/OpenSourceAWE/SimpleKiteControllers.jl
cd SimpleKiteControllers.jl/bin
./install
./create_sys_image
cd ..
./bin/run_julia
```
The step `create_sys_image` is not strictly needed and takes 15-60 min. Skip it if you are short of time. 

Optionally you can also install the flight path optimizer with the command:
```
./bin/install_awetrim
```
and start it in the background:
```
./bin/run_server start     # stop, restart, status and log are the other subcommands
```
`start` returns once the server answers, and it survives the terminal it was
started from. Without a subcommand `./bin/run_server` runs it in the foreground
in a second terminal window, as before.

Then, from a Julia REPL in this repository:

```julia
menu()
```
This function will show the following menu:

```text
Choose example to run or `q` to quit: 
 > select_turbulence.jl         - choose the turbulence level init() applies (default or 0.0…1.0)
   select_windspeed.jl          - choose the wind speed init() applies (default or a specific m/s)
   select_project.jl            - choose which system project (150m/200m/300m) to fly
   select_sim_time.jl           - choose the simulation time (default or a specific value)
   select_plots.jl              - choose figures: pattern/3d path(+webgl)/time series/power/aero
   plot_scenario.jl             - replot an archived run from output/scenarios/<site>/
   move_scenario.jl             - move the last reel-out run into output/scenarios/<site>/vNN
   copy_scenario.jl             - same, but keeps vNN_2/vNN_3/... instead of overwriting
   build_all_scenarios.jl       - re-fly and replace every scenario of both sites (30 min!)
   simple_opt_reelout.jl        - reel out along an externally optimized path (minutes!)
   simple_reelout_plots.jl      - plot the last logged reel-out run
   stability_opt_reelout.jl     - disk margins of the reel-out course loop over tether length
   stability_global.jl          - worst reel-out disk margin of every archived scenario (minutes!)
   simple_fig8.jl               - fly the figure-of-eight pattern (minutes!)
   simple_fig8_live.jl          - the same run, shown live in the 3D viewer (minutes!)
   simple_fig8_plots.jl         - plot the last logged run of active project
   stability_fig8.jl            - disk margins of the course-control loop (fig8 project)
   simple_opt_fig8.jl           - fly an externally optimized path at constant length (minutes!)
   simple_reelout.jl            - fly the pattern, then reel out to reelout_l_max (minutes!)
   simple_reelout_play.jl       - replay the last logged reel-out run in the 3D viewer
   simple_auto_parking.jl       - fly heading-stabilized parking of the V3 kite
   simple_auto_parking_plots.jl - plot the last logged parking run
   optimize_path.jl             - Julia client for the AWETrim reelout flight-path optimizer
   export_v3_segments.jl        - write the V3 segment table to output/v3_segments.csv
   create_overview.jl           - write scenario overview.md across wind speeds
   create_plots.jl              - batch-generate plots for notebooks/images/<site>
   publish.jl                   - export and push the results notebook
   plot_powercurve.jl           - plot mean reel-out power vs wind speed across archived scenarios
   quit
```
The menu shows twelve entries at a time and scrolls; the five `select_*` entries change the
simulation settings, which are persisted to `data/gui.yaml` and read fresh by every run
rather than cached in a REPL global.

## Settings of a run

Each of the five `select_*` scripts defines a function of the same name, such as `select_project()`,
which can be called again once the script has been included. The choices are written to
`data/gui.yaml`, which every run reads fresh.

### [`select_project.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/select_project.jl)
Chooses which system project the examples fly and writes the choice to `data/gui.yaml`.

To fly a figure of eight at a fixed tether length, select one of the three projects
- `system_fig8_150m.yaml`
- `system_fig8_200m.yaml`
- `system_fig8_300m.yaml`

To produce power by reeling out, select one of the projects
- `system_reelout_cabauw.yaml`: onshore, with a strong wind shear
- `system_reelout_maasvlakte.yaml`: nearshore, with little wind shear

The reel-out scripts read the selection through [`selected_reelout_project`](@ref), which ignores
a figure-of-eight project and flies `system_reelout_maasvlakte.yaml` instead.

### [`select_sim_time.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/select_sim_time.jl)
Chooses how long a run lasts: the project's own `sim_time`, or a value in seconds entered at the
prompt.

### [`select_windspeed.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/select_windspeed.jl)
Chooses the mean wind speed `init` applies: the project's own `v_wind`, or a value in m/s at the
project's reference height. A wind speed without a cached turbulent wind field makes the next run
generate one, which takes a while and about 1.2 GB of disk.

### [`select_turbulence.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/select_turbulence.jl)
Chooses the turbulence level: the `use_turbulence` of the project's settings, or a value between
`0.0` (no turbulence) and `1.0` (the Cabauw reference level).

### [`select_plots.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/select_plots.jl)
A checkbox menu that chooses which figures the plot scripts draw: the pattern, the time series,
the aerodynamics, and, for the reel-out runs only, the 3D path (GLMakie or WGLMakie) and the
power of the winch.

## Tools

### [`create_sys_image.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/create_sys_image.jl)
Builds a PackageCompiler system image with MakieControlPlots and most of V3Kite's dependencies,
which cuts the time to load the packages before every session. This is what `bin/create_sys_image`
runs; it takes 15 – 60 minutes and is optional.

### [`export_v3_segments.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/export_v3_segments.jl)
Writes the segment table of `v3_segments.jl` to `output/v3_segments.csv`. It builds the structure
from V3Kite's geometry file without building the model, so it runs in about a second.

## Shared helper files

These files are not run on their own; the examples on the other pages `include` them.

### [`menu.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/menu.jl)
Defines `menu()`, the interactive menu shown above. The chosen script is `include`d, so it runs
exactly as it would by hand.

### [`menu2.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/menu2.jl)
Defines `menu2()`, the menu for the model identification: the project selection,
`build_turn_rate_table.jl`, `plot_c1_c2.jl`, the other identification scripts and
`stability_opt_reelout.jl`, see [Examples - identification](examples_identification.md).

### [`identification_utils.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/identification_utils.jl)
Rewrites single values of a YAML file, and the comment above them, without losing the other
comments, and flies a run and identifies the kite's response time on its log; used by the
identification scripts that write `course_loop_model.yaml`.

### [`model_setup.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/model_setup.jl)
The model side of the runs, which the package cannot hold because it does not depend on V3Kite:
`init_model` initializes the V3 model at the project's wind and tether length and settles it,
holding the tether length during the warm-up.

### [`winch_adapter.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/winch_adapter.jl)
The glue between the plant and the winch controller: it reads the plant's scalars off the V3
model and hands them to the scalar-only functions of WinchControllers.jl, which turn a length or
force set point into the winch torque `step!` takes.

### [`v3_segments.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/v3_segments.jl)
The segment topology of the V3 kite: which two points each segment connects, and whether it is a
tether, wing or bridle segment. It is shared by `export_v3_segments.jl` and the 3D viewer of
`simple_fig8_live.jl`, so the exported table and the drawn one cannot disagree.
