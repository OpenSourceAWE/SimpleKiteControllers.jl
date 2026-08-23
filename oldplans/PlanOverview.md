# Add overview.md
Status: **DONE**

Create a table overview.md as file in the root directory.

The table shall be generated from the following files:
```
SimulationResults/scenarios/{v03,v04,v05,v06,v07,v08,v08_2,v09,v10}/reelout_150m_opt.yaml
```
The file shall be generated in the folder SimulationResults/scenarios

It shall have the following columns:
  date: "2026-08-22"                # wall-clock date the run finished
  time: "17:57:35"                  # wall-clock time the run finished
  wind_speed_m_s: 6.0               # wind speed at 6 m sent to the optimizer [m/s]
  mean_power_W: 8591                # mean reel-out power the run harvested [W]
  power_ratio: 0.99                 # measured / predicted mean reel-out power, against the weighted prediction
  total_wall_time_s: 190.1          # the whole script, start to this summary [s]
  realtime_factor: 1.81             # sim_time / wall_time, excluding time frozen for re-optimization
  optimization_requests: 7          # solves that completed, accepted or rejected
  optimizations_installed: 7        # new paths actually flown

One row for each wind speed. The wind speed is also part of the folder name, v03 means 3 m/s ground wind speed at 6 m height.

The script `create_overview.jl` in the examples folder of the SimpleKiteControllers project shall generate the overview. It shall be added to the menu.jl file.

## Corrections
In the script create_overview.jl, change the column names of the generated table:

- replace the column name wind_speed_m_s with v_wind
- replace mean_power_W with power
- replace total_wall_time_s with total_time
- replace realtime_factor with rt_factor
- replace optimization_requests with opt_requests
- replace optimizations_installed with opts_installed

## Additions
Can you add the following fields to the summary of each simulation
- av_power
- min_power
- max_power
- min_force
- av_force
- max_force
- v_ro_min
- v_ro_av
- v_ro_max
- av_depower
The averaging shall apply to phase four.

Remove the existing mean_N, peak_N (reelout.force block) and mean_W, peak_W
(reelout.power block) fields, since they are superseded by the phase-four
fields above. Leave the rest of those two blocks (cf_force_ro, cf_power_ro,
duration_s, n_samples, energy_kJ, energy_run_kJ) untouched.

a. add this to reelout_results.jl
b. write a one-time script to extend the existing reelout_150m_opt.yaml files in the subfolders of output/scenarios
c. update create_overview.jl
d. remove mean_N/peak_N/mean_W/peak_W from reelout_results.jl and from the archived reelout_150m_opt.yaml files
