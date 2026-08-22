# Add overview.md
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