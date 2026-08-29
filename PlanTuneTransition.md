# Tune dive and transition phase

The kite mass was corrected and increased from 6.2 to about 11 kg. This results in the kite diving too
deep during the transition phase. This must be fixed, first for 4 m/s wind. The baseline
simulation can be found in `output/scenarios/v04_2` (4 m/s, `mass: 10.9926`, everything else at
`e272dd2`); `output/scenarios/v04` is the same settings at the old 6.2 kg and passes.

**Status: FIXED for 4 m/s** (`chi_dive: -125.0`, `dive_el_margin: 17.0`, both committed in
`data/fc_settings_reelout.yaml`; confirmed at full length, archived as `output/scenarios/v04_3`,
`all 10 passed`). Remaining: re-check 3 m/s and 5-6 m/s with the same entry tuning (see
"Next steps" at the end).

## What is measured

| | mass | chi_dive | dive_el_margin | min elevation, WHOLE RUN | verdict |
| --- | --- | --- | --- | --- | --- |
| `v04` | 6.2 kg | -95 | 13 | **9.2°** | all 10 passed |
| `v04_2` | 10.9926 kg | -95 | 13 | **-3.4°** | FAILED: min elevation > 6.5° (whole run) |
| (screening) | 10.9926 kg | -125 | 13 | **5.9°** | FAILED: min elevation > 6.5° (whole run), 0.6° short |
| (screening) | 10.9926 kg | -125 | 17 | **10.1°** | pass (screening length only, see below) |
| `v04_3` | 10.9926 kg | -125 | 17 | **7.9°** | **all 10 passed** (full 150 m -> 380 m confirmation) |

`min elevation > 6.5° (whole run)` was the ONLY criterion `v04_2` failed. Phase 4 was unaffected
throughout (laps and settled cross-track never regressed) — the dip is entirely in the entry,
before the scoring window opens.

Per phase, from the `v04_2` log (`chi_dive = -95`, the pre-fix baseline):

| phase | azimuth | elevation | duration |
| --- | --- | --- | --- |
| 0 park | -0.3 -> -1.1° | 73.4° | 2.0 s |
| 1 dive | **-1.1 -> -108.9°** | 73.9 -> 39.2° | 18.3 s |
| 2 hold | -108.9 -> -111.0° | 35.9° | 0.8 s |
| 3 transition | **-111 -> -12.1°** | 35.8 -> 14.7°, dipping to **-3.4°** | 19.7 s |

**The transition was not the fault; the dive was.** At `chi_dive = -95` the dive is 5° past flat,
so it barely descends and instead traverses 110° of azimuth, dumping the kite at the edge of the
window. The transition is then a 19.7 s recovery back across it, and that is where the altitude
goes. At 6.2 kg this was survivable; 4.8 kg more wing was not.

Ruled out early (see the 2026-08-29 entries in `docs/fig8_tuning_log.md`):

- The entry **descent limiter** played no part — active for 0.0 % of phase 3, because it only
  engages when `|chi_set| > entry_chi_max` (95°) and the guidance was commanding 62.7° at the
  lowest point, a CLIMB. `entry_d_gate` was never reached either (`dmin` 48.6° at the low point,
  gate 12°). The kite was told to climb and sank anyway: an ENERGY problem, not a guidance one —
  except the fix confirmed it WAS geometry after all (see below), just not a guidance-limiter one.

## What fixed it

Two levers, applied in sequence, each confirmed with a screening run before moving to the next
(see "Method" below for how the screening runs were read):

1. **`chi_dive: -95 -> -125`** (already staged in commit `6443fbe`, flown for the first time here).
   Recovered 9.3° of the 10.9° gap on its own: dive azimuth travel 110° -> 81°, dive duration
   18.3 -> 14.4 s, transition duration 19.7 -> 14.6 s, whole-run min elevation -3.4° -> +5.9°.
   Confirms the fault was the dive's descent:traverse ratio, not an energy deficit.
2. **`dive_el_margin: 13 -> 17`**, once `chi_dive` alone left a 0.6° shortfall. Raises the dive ->
   hold threshold (`el_c_path + margin`, see "Non-obvious" below) so phase 3 starts with more
   altitude to spend: whole-run min elevation 5.9° -> 10.1° (screening), 7.9° at full length —
   equal to `min_settled`, so the dip is gone entirely, not just under the gate.

Neither `entry_depower` nor `entry_f_min` needed changing. `av_power_ro` at the winning tuning
(1667 W, full length) is ABOVE the old 6.2 kg baseline's 1551 W.

## Parameters considered

All of these are `FC_Settings` fields in `data/fc_settings_reelout.yaml` (the reel-out run's file —
`data/fc_settings.yaml` belongs to `simple_fig8.jl` and must not be edited for this).

### Directly shapes the dive (phase 1) — where the fix came from

| parameter | value used | effect |
| --- | --- | --- |
| `chi_dive` | **-125.0** (was -95) | Dive course. >90 descends, <90 climbs, 90 flat. The single biggest lever: sets the ratio of descent to lateral travel (1:3.5 at -95, much tighter at -125). |
| `dive_el_margin` | **17.0** (was 13) | Margin above the pattern centre where the dive ends. Sets how much altitude phase 3 has to spend. |
| `chi_hold` | -90.0 (unchanged) | Course during the 0.8 s hold. Flat; little leverage — confirmed unnecessary. |
| `hold_time` | 0.8 (unchanged) | Duration of the hold. |
| `park_time` | 2.0 (unchanged) | Zero-steering settle before the dive. |

**Non-obvious:** in `simple_opt_reelout.jl` the ladder's centre is the OPTIMIZED path's, not
`fcs.el_center` — `examples/simple_opt_reelout.jl:1076` does `ccs.el_center = el_c_path`. In the
screening runs that was 26.2°, so dive -> hold fired at 26.2 + 17 = 43.2° at the winning tuning,
confirmed against the log's `var_04`. `fcs.el_center: 18.0` is inert here. Tuning
`dive_el_margin` therefore moves a threshold that itself moves with the optimizer's reply.

### Energy budget of the entry — considered, NOT needed

| parameter | value | effect | why not used |
| --- | --- | --- | --- |
| `entry_depower` | 0.34 (unchanged) | Depower during dive and hold; powering up buys `v_app` and lift. | More lift = SLOWER descent and a LONGER traverse — works against the fix, and turned out unnecessary once `chi_dive`/`dive_el_margin` closed the gap. |
| `entry_f_min` | 350.0 (unchanged) | Force floor of the entry guard, phases 0-2; the entry pulls 327-329 N, under the floor, so the guard's PID winds up (cosmetic, does not move the elevation criterion). | Not touched — the elevation fault was already fixed without it. |
| `depower_blend_time` | 2.5 (unchanged) | Ramp time between phase-ladder depower targets. | — |
| `entry_gain` | 0.25 (unchanged) | Factor on `heading_p` below phase 3. | — |

### Governs the transition itself (phase 3) — inert in every run measured here

| parameter | value | note |
| --- | --- | --- |
| `fig8_d_gate` | 4.0 (unchanged) | Cross-track error at which phase 3 -> 4. |
| `entry_chi_max` | 95.0 (unchanged) | Steepest commanded course while off-path. INERT (never engaged, guidance stayed a climb). |
| `entry_d_gate` / `entry_d_blend` | 12.0 / 4.0 (unchanged) | Also inert (`dmin` never reached the gate). |

### Winch side of the entry — not touched

| parameter | value | effect |
| --- | --- | --- |
| `reelout_delay` | 4.2 (unchanged) | Wait after phase 3 before reel-out starts; winch commanded to zero until it expires. |
| `reelout_f_trigger` | 6400.0 (unchanged) | Force that releases reel-out early. Inert at 4-6 m/s. |
| `reelout_softstart` | 12.0 (unchanged) | Ramps commanded speed 0 -> `v_set`. |

### Explicitly NOT levers here

- `min_elevation: 6.5` — the failing criterion. Relaxing it hides the problem instead of fixing it.
- `depower_setpoint: 0.274` — a curvature CEILING (0.278 at 150 m), not a free knob.
- `f8_a`/`f8_b`/`el_center` — describe the GUESS only; the flown path comes from the optimizer.
- `max_steering: 0.32` — raising it is closed, see the tuning log.
- Anything in `data/traj_opt.yaml` — leave the user's values alone (in particular `reopt_enabled`).

## Method: screening runs, then one full-length confirmation

### Requesting `sim_time: 60` does NOT give a 60 s run below 6 m/s

`simple_opt_reelout.jl` scales the requested `sim_time` by a wind-dependent budget
(`examples/simple_opt_reelout.jl:331-352`). Below `V_BUDGET_KNOT = 6.0` m/s, `EFFECTIVE_SIM_TIME
= something(SIM_TIME, project_set.sim_time) * wind_ratio^1.795`, `wind_ratio =
default_v_wind / WIND_SPEED`. At 4 m/s against the project's own 6.0 m/s `v_wind`, a requested
60 s became an EFFECTIVE 124.2 s — long enough that the default `entry_time: 62.0` window would
have opened anyway. Lowering `entry_time` to ~45 for screening (as done here) is harmless but
was not actually necessary at 4 m/s; it may matter more for a wind speed whose scale factor is
smaller (i.e. closer to 6 m/s, where the knot itself takes over and the budget is computed from
the reel-out physics instead — see the same lines). Confirm the effective length in the log
before assuming a screening run was too short to say anything.

Consequences that DO hold regardless of the effective length, because reel-out at 4 m/s never
gets far in ~2 minutes:

- **The pass/fail verdict is unusable at screening length.** `phase 5 (final) reached`,
  `laps >= 2.5` and the azimuth-reach criteria fail by construction until reel-out actually
  finishes. Score on **`elevation_deg.min_whole_run`** (computed over the whole run, unaffected by
  truncation) plus the phase table; ignore `success_criteria` until the confirmation run.
- **Only the guess path is flown.** The first re-optimized path installs well after a screening
  run's window closes, so screening says nothing about phase 4 power/laps — good for
  attributability (every screening run is compared against the same fixed path), useless for
  scoring the pattern itself.

Each screening run: `include("examples/simple_opt_reelout.jl")` with `wind_speed: 4.0` and
`sim_time: 60` in `data/gui.yaml` (`set_selected_sim_time`/`set_selected_windspeed` in
`examples/gui_state.jl`, or the interactive `select_sim_time.jl`/`select_windspeed.jl`), block on
`output/last_run_done.txt`, and read `elevation_deg.min_whole_run` and the phase table
(`sys_state`/`elevation`/`azimuth` in the arrow log) out of `output/reelout_150m_opt.yaml`/`.arrow`.
Do NOT archive these with `copy_scenario.jl` — a truncated log is not a scenario. Record each in
`docs/fig8_tuning_log.md` as a screening result, with the lever and its value.

Once a tuning passes on `elevation_deg.min_whole_run`, restore `sim_time` to `nothing` (project
default) and `entry_time` to 62.0, fly ONE full 150 m -> 380 m run, and only THEN read
`success_criteria` and archive with `copy_scenario.jl`.

## Next steps

Re-check the neighbouring wind speeds with `chi_dive: -125.0`, `dive_el_margin: 17.0` (both now
committed) before regenerating the overview table:

- **3 m/s**: `v03_2` already failed differently at the corrected mass (`phase 5 (final) reached`,
  not an elevation criterion) — likely needs its own diagnosis, not assumed to inherit this fix.
- **5-6 m/s**: not yet re-flown at the corrected mass; check whether the same dive/margin values
  hold or need their own screening pass.
- Once both are green (or separately fixed), regenerate `output/scenarios/overview.md`.
