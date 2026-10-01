# Examples

`examples/simple_fig8.jl` flies the figure-of-eight controller on the TU Delft V3 kite,
using [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) as the plant.

![V3 Kite flying a 200 m figure-of-eight pattern](V3_Kite_system_fig8_200m_pattern.png)

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
   optimize_fig8.jl             - sweep the pattern shape in parallel processes (HOURS!)
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

The runs themselves come in two families. `simple_fig8.jl` flies the pattern at constant
tether length and plots the results when it is done; `simple_fig8_live.jl` is the same
run shown in the 3D viewer while it flies, and `simple_fig8_plots.jl` re-plots a log that
is already on disk. `simple_reelout.jl` flies the same entry and pattern but reels the
tether out under load until `reelout_l_max`, with `simple_reelout_plots.jl` and
`simple_reelout_play.jl` for its logs. Both families write an Arrow log to `output/`,
named after the active project's `log_file` setting.

`simple_opt_fig8.jl` is a third run in the first family: same plant, entry and inner
loop as `simple_fig8.jl`, but the reference path comes from the AWETrim optimizer instead
of from the `f8_*` lemniscate parameters — it asks for the power-optimal path under the
run's own wind and winch, installs it and flies it at constant tether length. It starts
the server itself if none is running. `simple_opt_reelout.jl` is the same idea with the
reel-out winch, which is what the path was optimized for: its run summary reports the
power the optimizer predicted next to the power the run harvested. Both read their
optimizer settings — server, initial guess, solver knobs — from `data/traj_opt.yaml`, and
the reel-out one logs to `<log_file>_opt` so the lemniscate run stays as its baseline.

`optimize_fig8.jl` sweeps the pattern shape by driving `simple_reelout.jl` in parallel
worker processes — hours, not minutes, and resumable. `optimize_path.jl` is the separate
AWETrim client described above.

`examples/simple_auto_parking.jl` flies the attitude-stabilized parking maneuver: the wing
is settled at a fixed depower setting and held at a constant tether length while a
gain-scheduled heading PID regulates the heading to zero, so the kite does not drift away
from straight-up parking. It logs to `output/tmp_auto_parking.arrow` and prints the heading
regulation RMS error and the AoA ripple metrics; `examples/simple_auto_parking_plots.jl`
re-plots that log without re-simulating.

