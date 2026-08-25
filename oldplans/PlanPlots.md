# More and better plots

## Step 1
Rename the keys in the summary:
wind_speed_m_s -> wind_speed_gnd
mean_power_W -> av_power_ro
av_power_W -> av_power_ro
min_power_W -> min_power_ro
max_power_W -> max_power_ro
min_force_N -> min_force_ro
av_force_N -> av_force_ro
max_force_N -> max_force_ro
v_ro_min_m_s -> v_ro_min
v_ro_av_m_s -> v_ro_av
v_ro_max_m_s -> v_ro_max
av_depower -> av_depower_ro

in reelout_results.jl

and remove mean_power_W = opt_power_meas, power averaged over the whole reeling window (samples where the length setpoint is increasing) from the yaml file.

This is a deliberate redefinition: the power curve moves from "mean power over
the whole reeling window" (the old mean_power_W) to "mean power over phase
four only" (av_power_ro, the renamed av_power_W). Everything downstream reads
av_power_ro from now on.

Also update create_overview.jl, which independently hard-indexes the old key
names (`row[key]`, no fallback) in two places: the `COLUMNS` mapping (used for
the Markdown table) and `powercurve_png` (its own embedded power-curve plot,
built the same way as plot_powercurve.jl). Update `COLUMNS` to the new key
names, drop `"mean_power_W" => "power"`, point `powercurve_png` at
`wind_speed_gnd`/`av_power_ro`, and update the module docstring's key list and
its "distinct from power/mean_power_W" note accordingly.

## Step 2
Update the existing yaml log files in the subfolders of output/scenarios

## Step 3
Fix plot_powercurve.jl to use only existing keys.

## Step 4
- add average force (field av_force_ro and speed (field v_ro_av) to the power curve plot  script plot_powercurve.jl
- use plotx to plot power, force and speed stacked

## Step 5
- add max_force_ro, max_power_ro and max_speed_ro to the plot

## Step 6
- add min_force_ro, min_power_ro and min_speed_ro to the plot

## Step 7
- update create_overview.jl to use the new plot

## Step 8 ✓
"The pattern plot" here means the azimuth/elevation flight-pattern plot
(flown vs. attractor, or flown vs. optimizer-raw when available) currently
built inline in `simple_reelout_plots.jl`'s `"pattern" in plots` block — NOT
the powercurve plot from steps 1-7, which stays a single aggregate figure
across all scenarios.

✓ Extracted block's logic into standalone `plot_pattern_scenario()` function in
  `plot_pattern_utils.jl`, which loads scenario directory's own settings/log,
  computes az/el plot without needing script-local state, and accepts optional
  `opt_raw` parameter for optimizer paths.
✓ Updated `simple_reelout_plots.jl` to include `plot_pattern_utils.jl` and call
  the extracted function for its interactive "pattern" plot, preserving
  behavior with `OPT_PATHS_RAW` support.
✓ Created `create_plots.jl`: loops non-empty `output/scenarios/` folders, calls
  `plot_pattern_scenario()` per scenario with `disp=false`, saves PNG files to
  `notebooks/pattern_<scenario>.png` (e.g. `pattern_v08.png`).
