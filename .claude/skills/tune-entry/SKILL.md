---
name: tune-entry
description: Tune the reel-out entry phases (park, dive, hold, transition) at a given wind speed when a `simple_opt_reelout.jl` run fails the whole-run minimum-elevation criterion — the kite sinks too deep before the figure-eight starts. Use when a wind speed regresses on "min elevation > 6.5° (whole run)", when re-checking a wind speed after `chi_dive`/`dive_el_margin` changed, or after a plant change (mass, damping, turn-rate table) that moves the entry.
---

# Tune the reel-out entry at a wind speed

Procedure for the 150 m -> 380 m reel-out run (`examples/simple_opt_reelout.jl`) when its
entry drops the kite below the elevation floor. Derived from the 4 m/s fix after the kite
mass was corrected 6.2 -> 10.9926 kg; the worked example with all measured numbers is in
[references/case-4ms.md](references/case-4ms.md).

## Invocation

The argument is the **mean wind speed in m/s** under test — `/tune-entry 3.5` means "fix the
entry at 3.5 m/s". It is the wind speed passed to `set_selected_windspeed` in step 3 and the
one whose `output/scenarios/vNN` folder is the baseline in step 0. With no argument, take the
wind speed from the failing run in the conversation, or read
`output/scenarios/overview.md` and ask which of the failing speeds to work on — never guess.

## Scope

In scope: `FAILED: min elevation > 6.5° (whole run)`, with phase 4 (the pattern itself)
unaffected — laps and settled cross-track unchanged, the dip sitting entirely in phases 1-3,
before the scoring window opens.

Out of scope, diagnose separately: `phase 5 (final) reached` (the run never finished
reel-out — a winch/budget or run-length fault), `laps >= 2.5`, the azimuth-reach criteria,
`RMS d < 3.0°` (pattern shape / path tracking). A wind speed can fail both — fix the
elevation part here and treat the rest on its own.

## The constraint that shapes everything

`data/fc_settings_reelout.yaml` is **one file for every wind speed**; there is no
per-wind-speed entry override. Anything tuned here is flown at 3.5 m/s and at 11 m/s too, so
the job is not done until the neighbouring wind speeds that already passed have been
re-checked (step 5). `data/fc_settings.yaml` belongs to `simple_fig8.jl` — never edit it for
this.

## Step 0 — Baseline

1. `output/scenarios/overview.md` — which wind speeds pass, and with what verdict.
2. The wind speed's own folder, e.g. `output/scenarios/v04/reelout_150m_opt.yaml`:
   `success_criteria`, `fig8_metrics.elevation_deg.min_whole_run` / `min_settled`,
   `laps`, `cross_track_deg.rms`, and `reelout` power for the regression check later.
3. That folder's `fc_settings_reelout.yaml` is the copy that **produced** it — diff it
   against `data/fc_settings_reelout.yaml` to know what actually changed since.

## Step 1 — Diagnose: it is almost always the dive

Get the phase table out of the log (kaimon `ex`, `q=false`; `println` is stripped, so the
snippet returns a string). Point `path` at the archive named in `output/last_run_done.txt`
for a fresh run, or at `output/scenarios/vNN` for an archived one:

```julia
using KiteUtils
sl = load_log("reelout_150m_opt"; path = "output").syslog
el = rad2deg.(sl.elevation); az = rad2deg.(sl.azimuth)
rows = String[]
for p in 0:4
    i = findall(==(p), sl.sys_state)
    isempty(i) && continue
    push!(rows, string("phase ", p, ": az ", round(az[first(i)]; digits = 1), " -> ",
        round(az[last(i)]; digits = 1), "°, el ", round(el[first(i)]; digits = 1), " -> ",
        round(el[last(i)]; digits = 1), "° (min ", round(minimum(el[i]); digits = 1), "°), ",
        round(sl.time[last(i)] - sl.time[first(i)]; digits = 1), " s"))
end
push!(rows, string("whole run min elevation: ", round(minimum(el); digits = 1), "°"))
join(rows, "\n")
```

The failure signature: **phase 1 is too flat**. It trades little elevation for a lot of
azimuth (110° of traverse at `chi_dive = -95`), dumps the kite at the edge of the window,
and phase 3 becomes a long recovery back across it — and the sag is in that recovery, not in
the transition logic. Read the ratio of the dive's elevation drop to its azimuth travel and
the phase-3 duration; both shrink when the fix works.

Before reaching for a phase-3 lever, check it is even active — at 4 m/s all three were inert:
the descent limiter only engages above `entry_chi_max` and the guidance was commanding a
climb at the lowest point, and `dmin` never reached `entry_d_gate`. A limiter that never fired
did not cause the dip.

## Step 2 — The levers, in this order

One lever per screening run, so each is attributable.

| order | parameter | direction | what it does |
| --- | --- | --- | --- |
| 1 | `chi_dive` | more negative (e.g. -95 -> -125) | Dive course: >90 descends, <90 climbs, 90 flat. The biggest lever — sets the descent:traverse ratio of phase 1. |
| 2 | `dive_el_margin` | up (e.g. 13 -> 17) | Dive -> hold threshold is `el_center + margin`; raising it ends the dive higher, so phase 3 has more altitude to spend. |

**Non-obvious:** in `simple_opt_reelout.jl` the ladder's centre is the OPTIMIZED path's, not
`fcs.el_center` (`ccs.el_center = el_c_path`, around `examples/simple_opt_reelout.jl:1076`),
so `dive_el_margin` moves a threshold that itself moves with the optimizer's reply. Confirm
the actual threshold against the log's `var_04`, and expect a different `el_c_path` at a
different wind speed.

`chi_hold`, `hold_time` and `park_time` have little leverage; `entry_depower` works **against**
the fix (more lift = slower descent and a longer traverse) and `entry_f_min` is cosmetic here
(its guard PID winds up when the entry pulls under the floor, without moving the elevation
criterion). None of them were needed at 4 m/s.

## Step 3 — Screening runs

A screening run flies a shortened run to score `min_whole_run` only, at ~1/3 the wall time of
a confirmation.

**Requesting `sim_time: 60` does not give a 60 s run below 6 m/s.** The script scales it by a
wind-dependent budget (`examples/simple_opt_reelout.jl:331-352`): below
`V_BUDGET_KNOT = 6.0` m/s the effective time is `sim_time * (default_v_wind/wind_speed)^1.795`
— at 4 m/s against the project's 6.0 m/s that turned a requested 60 s into 124.2 s. At or
above the knot the budget is computed from reel-out physics instead and a request is ignored
entirely. Read the effective length the script logs at startup before concluding a screening
run was too short to say anything.

Set up and fly one:

```julia
include("examples/gui_state.jl")
set_selected_windspeed(4.0)      # the wind speed under test
set_selected_sim_time(60.0)      # screening budget; see the scaling above
include("examples/simple_opt_reelout.jl")
```

Then block on the marker rather than guessing a sleep:

```bash
until [ -f output/last_run_done.txt ]; do sleep 5; done; cat output/last_run_done.txt
```

Reading a screening run:

- Score **only** `elevation_deg.min_whole_run` (computed over the whole run, so truncation
  does not touch it) plus the phase table. `success_criteria` is unusable —
  `phase 5 (final) reached`, `laps >= 2.5` and the azimuth-reach criteria fail by
  construction until reel-out actually finishes.
- Only the guess path is flown; the first re-optimized path installs after the window closes.
  Good for attributability (every screening run is compared against the same fixed path),
  useless for judging phase-4 power or laps.
- Do **not** archive with `copy_scenario.jl` — a truncated log is not a scenario. Record the
  result in `docs/fig8_tuning_log.md` as a screening result, naming the lever and its value.
- Lowering `entry_time` (default 62.0) for screening is optional; it only matters when the
  effective length is short enough that the metrics window would not open.

## Step 4 — Full-length confirmation

Once a tuning passes on `min_whole_run`, restore `set_selected_sim_time(nothing)` and
`entry_time: 62.0`, fly ONE full 150 m -> 380 m run, and only then:

- read `success_criteria` (target `all 10 passed`) and `min_whole_run` — ideally equal to
  `min_settled`, meaning the entry dip is gone rather than merely under the gate;
- check `av_power_ro` did not regress against the wind speed's previous passing scenario;
- read the numbers from the **archive** named in `output/last_run_done.txt`, not from
  `output/` — the next run overwrites `output/reelout_150m_opt.{arrow,yaml}`;
- archive with `include("examples/copy_scenario.jl")`;
- write the entry in `docs/fig8_tuning_log.md`: phase table, the levers and their values, the
  before/after `min_whole_run`, power, and the archive folder.

## Step 5 — Regression at the other wind speeds

Because the settings file is shared, re-fly the wind speeds that already passed — at minimum
the immediate neighbours, and any speed whose scenario is older than the plant change that
started this. Confirm `all 10 passed` and no power regression. Then regenerate
`output/scenarios/overview.md` (`examples/create_overview.jl`).

If a neighbour now needs a *different* `chi_dive`/`dive_el_margin` than the one just tuned,
that is a real conflict, not a tuning choice: report it and ask, rather than trading one wind
speed against another. There is no wind-speed table for these fields today.

## Never touch for this

- `min_elevation: 6.5` — the failing criterion itself; relaxing it hides the fault.
- `depower_setpoint` — a curvature ceiling, not a free knob.
- `f8_a` / `f8_b` / `el_center` — they describe the guess only; the flown path is the
  optimizer's.
- `max_steering` — raising it is closed, see the tuning log.
- `data/traj_opt.yaml`, in particular `reopt_enabled` — the user's values; ask first.
