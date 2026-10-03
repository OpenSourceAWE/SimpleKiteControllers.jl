# Examples - reel-out

The runs on this page fly the same entry and figure-of-eight pattern as the
[figure-of-eight examples](examples_fig8.md), but reel the tether out under load, which is how
the kite produces power. Installation, the example menu and the `select_*` scripts are described
on [Examples - general](examples_general.md).

## Reel-out runs

### [`simple_reelout.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/simple_reelout.jl)
Flies the figure-of-eight pattern with a reel-out winch. The tether starts at `l_tether` (150 m by
default); from the moment the guidance engages, the winch reels out with the speed law
`v_set = kv * sqrt(force)` of WinchControllers.jl, until the tether reaches `reelout_l_max` or
the kite has flown `n_fig_eight` figures of eight. Then the length is held, and in a fifth phase
the depower switches to `depower_final` while the pattern keeps flying. There is no reel-in
phase. The run writes an Arrow log to `output/`, named after the project's `log_file` setting.

### [`simple_reelout_plots.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/simple_reelout_plots.jl)
Re-plots the last log of `simple_reelout.jl` or `simple_opt_reelout.jl` without simulating again.
The figures are those of `simple_fig8_plots.jl`, plus the flown tether length against the lap
count, the reel-out speed against its set point, the commanded depower with the state of the
winch controller, and optionally the 3D path and the power of the winch.

### [`simple_reelout_play.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/simple_reelout_play.jl)
Replays the last log of `simple_reelout.jl` in the KiteViewers 3D window. No simulation runs; the
project and the log are found exactly as `simple_reelout.jl` finds them.

### [`simple_opt_reelout.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/simple_opt_reelout.jl)
`simple_reelout.jl` along a path the [AWETrim](https://github.com/awegroup/AWETrim) optimizer
produced instead of along the lemniscate: the optimizer is asked for the power-optimal reel-out
path for the run's own wind and winch, and the path is re-optimized while the tether gets longer.
Feasibility gates ([`check_reelout_feasibility`](@ref), [`check_startup_path`](@ref)) reject paths the
kite cannot fly before they are installed. The run summary reports the power the optimizer
predicted next to the power the run harvested. The settings of the optimizer come from
`data/traj_opt.yaml`, and the run logs to `<log_file>_opt`, so the lemniscate run stays as its baseline.

### [`plot_trajectory.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_trajectory.jl)
Plots the newest startup path that the feasibility gate of `simple_opt_reelout.jl` rejected: the
reference curve in the azimuth–elevation plane as the optimizer returned it, with its curvature
margin and predicted power. The gate saves each rejected path in `trajectories/` before the run
aborts.

## Stability and tuning

### [`stability_opt_reelout.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/stability_opt_reelout.jl)
A disk-margin stability analysis of the course-control loop of `simple_opt_reelout.jl` over the
full range of tether length, from `l_tether` to `reelout_l_max`. Plant and controller are those
of `stability_fig8.jl`; the operating points (apparent wind speed, kite speed, depower, elevation)
are taken from a flown log. It needs `ControlSystemsBase`.

### [`stability_global.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/stability_global.jl)
Runs `stability_opt_reelout.jl` on every archived scenario of both sites, one site after the
other as `build_all_scenarios.jl` does, prints the worst disk margin of each one and writes each
site's `stability_overview.md`. The operating points come from the scenario's log and the
controller from the current settings, so the table answers whether the current tuning is stable
at every operating point flown so far.

### [`retune_guided.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/retune_guided.jl)
Retunes the reel-out course loop in small steps until the worst guided disk margin over all
archived scenarios of both sites reaches a target. It starts from the current
`fc_settings_reelout.yaml` and evaluates each trial setting on the linear model of
`stability_opt_reelout.jl`, without flying.

## Scenarios

A scenario is the archived record of one `simple_opt_reelout.jl` run: its log, its run summary
and every settings file that produced it, in `output/scenarios/<site>/vNN`, with `site` either
`maasvlakte` or `cabauw` and `NN` the wind speed.

### [`move_scenario.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/move_scenario.jl)
Moves the last finished run of `simple_opt_reelout.jl` into `output/scenarios/<site>/`, in a
folder named after the wind speed it was flown at (6.0 m/s becomes `v06`). Wind speed and site
are read from the run's own summary. The log is compressed on the way.

### [`copy_scenario.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/copy_scenario.jl)
The same as `move_scenario.jl`, but a run at a wind speed that already has a scenario is stored
as `vNN_2`, `vNN_3`, … instead of replacing it.

### [`build_all_scenarios.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/build_all_scenarios.jl)
Re-flies every archived scenario of both sites with the current code and settings, and replaces
each scenario with the new run if it finished cleanly and passed all success criteria; otherwise
the old scenario is kept. Afterwards it rewrites the overview of each site. This takes about 30 minutes.

### [`plot_scenario.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_scenario.jl)
An interactive menu to re-plot an archived scenario with `simple_reelout_plots.jl`, using the
log and settings stored in the scenario's own folder.

### [`create_overview.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/create_overview.jl)
Writes the `overview.md` of the active site for the
[SimulationResults](https://opensourceawe.github.io/SimulationResults/) notebooks: one row per
wind speed with power, force, reel-out speed, power ratio and the success criteria, read from the
run summaries of the scenarios.

### [`create_plots.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/create_plots.jl)
Generates the pattern, time-series, power and aerodynamics plots and an interactive WGLMakie 3D
path for every scenario of the active site, and saves them to `notebooks/images/<site>/`.

### [`plot_powercurve.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_powercurve.jl)
Plots the mean, maximum and minimum reel-out power, tether force and reel-out speed against the
wind speed, one point per scenario of the active site.

### [`plot_patterns_paper.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_patterns_paper.jl)
Generates the pattern plots of the paper: the Maasvlakte and Cabauw scenarios at a given wind
speed, and a pair of runs with and without feedforward on a shared axis scale. The PDFs are saved
in the `figures/` folder of the paper's repository.

### [`compress.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/compress.jl)
Shrinks a saved Arrow log by dropping the corner points of the aerodynamic panels, which are only
used for visualization and make up about 78 % of a reel-out log. Optionally it also decimates
the rows. A compressed log loads and replays unchanged.

## Regression checks

### [`regression_baseline.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/regression_baseline.jl)
Flies `simple_opt_reelout.jl` at a site and wind speed with the optimizer's answers replayed from
an archived scenario, so no optimizer server is needed, and writes the log and summary to a
folder of its own. Run it before and after a refactor, and compare the two results with
`compare_runs.jl`.

### [`compare_runs.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/compare_runs.jl)
Compares two runs of `simple_opt_reelout.jl`, the Arrow logs column by column and the run
summaries key by key, and prints every difference. Fields that differ between any two runs, such
as wall-clock times and the git state, are ignored.

## Helper files

### [`reelout_results.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/reelout_results.jl)
Scoring, summary, archive and plots of a run of `simple_opt_reelout.jl`, included by it: it
reloads the log, prints the results, writes the run summary, copies log and settings into a
timestamped archive folder and draws the plots. It does not touch the plant, so a change to it
can be checked by re-scoring an existing log.

### [`plot_pattern_utils.jl`](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/examples/plot_pattern_utils.jl)
The plotting functions for pattern, time series, power and aerodynamics, shared by
`simple_reelout_plots.jl` and `create_plots.jl`.
