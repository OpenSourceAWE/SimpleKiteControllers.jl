# Add summary to yaml simulation report

`examples/reelout_results.jl` (`include`d by `simple_opt_reelout.jl` as its last step)
already writes a rich, nested YAML report next to the run's `.arrow` log — every
number below already exists somewhere in it, scattered across `simulation`,
`reelout`, `traj_opt` and `performance`. This plan adds one new top-level
`summary:` block, written as the LAST key of the file, that collects the eight
numbers a reader actually wants at a glance without duplicating or moving
anything that is already there — a recap of what came before, not a substitute
for `success_criteria` staying the first key.

## The eight fields and where they come from today

| new key                    | source (in the existing YAML)              | Julia variable at the point of use     |
|-----------------------------|---------------------------------------------|-----------------------------------------|
| `date`                      | `simulation.date`                            | `Dates.format(run_time, "yyyy-mm-dd")`  |
| `time`                       | `simulation.time`                            | `Dates.format(run_time, "HH:MM:SS")`    |
| `wind_speed_m_s`             | `traj_opt.inflow.wind_speed_m_s`             | `inflow.wind_speed`                     |
| `mean_power_W`               | `reelout.power.mean_W`                        | `opt_power_meas` (== `rp.mean_power`)   |
| `power_ratio`                | `traj_opt.power.ratio`                        | `opt_power_meas / opt_power_pred_eff`   |
| `total_wall_time_s`          | `performance.total_wall_time_s`               | `t_total`                                |
| `realtime_factor`            | `performance.realtime_factor`                 | `t_sim / max(t_wall - reopt_blocked_s, eps())` |
| `optimization_requests`      | `traj_opt.reopt.requests`                     | `reopt_n`                                |
| `optimizations_installed`    | `traj_opt.reopt.installed`                    | `count(e -> e.status == "installed", reopt_events)` |

Note `wind_speed` also exists, unqualified, as `simulation.wind_speed` — that is
`project_set.v_wind`, the mean wind speed **passed to `init`**. The field this plan
wants is the one literally suffixed `_m_s` in the report, `traj_opt.inflow.wind_speed_m_s`
— the wind speed **sent to the optimizer** (`inflow.wind_speed`). The two agree in
every run seen so far, but they are not the same number by definition; pull from
`inflow`, not from `project_set`.

Two of the nine numbers the old plan named don't have a bare `_W`/`ratio` key of their
own to copy — `reelout.power.mean_W` and `traj_opt.power.measured_W` are both just
`rp.mean_power` rounded the same way; use `reelout.power.mean_W`'s source
(`opt_power_meas`) since it is the plainer of the two paths.

## Missing-data guards

Three of the eight fields are only defined when something happened during the run,
exactly mirroring the guards already in place for their source blocks:

- `mean_power_W` / `power_ratio` — omitted when `rp === nothing` (tether never
  reeled out), same as `reelout.power` and `traj_opt.power.measured_W`/`ratio` are
  omitted today.
- `total_wall_time_s` / `realtime_factor` — omitted when `t_sim <= 0`, same as
  `performance` is omitted today (the `@warn "No simulated time elapsed"` branch).

Follow the existing convention exactly: a key is **left out**, never written as
`null` — `examples/wind_scan.jl` and `examples/plot_powercurve.jl` both `get(...,
default)` off these dicts, so an absent key is already the expected "not
measured" signal.

`optimization_requests` / `optimizations_installed` have no such gap — `reopt_n`
and `reopt_events` are `0`/`empty` rather than `nothing` when re-optimization
never ran, matching `traj_opt.reopt.requests`/`installed` today.

## Step 1 — `examples/reelout_results.jl`

All the source variables above only exist in final form after the `performance`
block (near the end of the file, right before `open(joinpath(output_path,
log_name * ".yaml"), "w") do io ... end`), since `total_wall_time_s` is by
definition the last thing computed — which is convenient, since the block is
meant to be the last key of the file anyway. No reordering of the existing
`OrderedDict` is needed: just assign `summary["summary"] = summary_block` right
before the `open(...) do io` call, after everything else has already been
assigned.

```julia
summary_block = OrderedDict{String, Any}(
    "date" => (Dates.format(run_time, "yyyy-mm-dd"), "wall-clock date the run finished"),
    "time" => (Dates.format(run_time, "HH:MM:SS"), "wall-clock time the run finished"),
    "wind_speed_m_s" => (inflow.wind_speed, "wind speed at 6 m sent to the optimizer [m/s]"))
if !isnothing(opt_power_meas)
    summary_block["mean_power_W"] = (round(Int, opt_power_meas), "mean reel-out power the run harvested [W]")
    summary_block["power_ratio"] = (round(opt_power_meas / opt_power_pred_eff; digits = 2),
        "measured / predicted mean reel-out power, against the weighted prediction")
end
if t_sim > 0
    summary_block["total_wall_time_s"] = (round(t_total; digits = 1), "the whole script, start to this summary [s]")
    summary_block["realtime_factor"] = (round(t_sim / max(t_wall - reopt_blocked_s, eps()); digits = 2),
        "sim_time / wall_time, excluding time frozen for re-optimization")
end
summary_block["optimization_requests"] = (reopt_n, "solves that completed, accepted or rejected")
summary_block["optimizations_installed"] = (count(e -> e.status == "installed", reopt_events),
    "new paths actually flown")
summary["summary"] = summary_block
```

`examples/simple_reelout.jl` has its own, near-identical copy of this file
(`write_yaml_commented` is duplicated there too) — check whether the same
`summary`-block construction is needed there before treating Step 1 as done;
if `simple_reelout.jl` doesn't build `reelout_summary`/`traj_opt` at all, only
the fields it does have apply.

## Step 2 — backfill the existing report files

The report files needing this all already have every source value the new
block needs (this is a *derived* section, not new data) — they just don't have
the `summary:` key yet:

```
SimulationResults/scenarios/{v03,v04,v05,v06,v07,v08,v08_2,v09,v10}/reelout_150m_opt.yaml
```

Do **not** re-run any simulation to regenerate them (minutes each, and the
whole point is that nothing measured has changed). Instead, write a short,
one-off Julia script (run once via a kaimon `ex` call or a scratch file, not
committed to `examples/`) that, for each folder:

1. `d = YAML.load_file(path)` to read the existing values back out (comments
   are not part of the YAML, so parsing loses nothing this plan needs).
2. Build `summary_block` the same way as Step 1, pulling from `d["simulation"]`,
   `d["traj_opt"]["inflow"]`, `d["reelout"]["power"]` (guard with `haskey`),
   `d["traj_opt"]["power"]` (guard with `haskey`), `d["performance"]` (guard
   with `haskey`), `d["traj_opt"]["reopt"]`.
3. Render just that block's text with the existing `write_yaml_commented(io, 1,
   summary_block)` (indent 1, since it nests under `summary:`), to a buffer.
4. Open the file in append mode and write `"summary:\n"` plus the rendered
   lines — a pure append, not a full YAML round-trip, so every existing line's
   formatting and comments survive untouched.
5. No line-splicing or read-back of the original text needed, since the new
   block only ever goes at the very end.

`SimulationResults` is its own git repository (see the workspace root list) —
confirm before running the backfill that it's fine to modify committed
reference data there, and commit the change separately from the
`SimpleKiteControllers.jl` code change.
