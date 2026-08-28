# Reduce the peak force for 9 m/s scenario
The force must ALWAYS stay below 8400 N. That is not the case for 9 m/s wind speed.

Try to reduce the peak force, currently about 8590 N.

## RESULT (2026-08-27)

SOLVED with knob 1 alone: `reelout_f_trigger` 7000 -> 6400 N in
`data/fc_settings_reelout.yaml`.

| reelout_f_trigger [N] | peak force [N] | t_peak [s] |
|----------------------:|---------------:|-----------:|
| 7000 (baseline)       | 8592           | 25.3       |
| 6800                  | 8437           | 25.2       |
| 6600                  | 8235           | 25.2       |
| 6400                  | 8122           | 27.9       |

The full-length run (150 -> 380 m, 113 s) at 6400 N confirms the same peak,
8122 N, 278 N of margin under the 8400 N rating. Knobs 2-4 were not needed.
The 6200 N step was started but cancelled once 6400 was already under the
limit; the file was left at 6400.

## Knobs, in the order to try them

The peak comes from the entry swoop loading the tether while the winch is still
held by the `reelout_delay` timer. The knobs that release the drum EARLIER act
directly on that; the depower knobs act on how hard the swoop loads it.

1. `reelout_f_trigger` (now 7000, in fc_settings_reelout.yaml) — LOWER it in
   steps of 200. It releases reel-out early once the force crosses it; a lower
   value opens the drum sooner in the swoop. Watch that it does not fire before
   the swoop has loaded the tether: the gate LATCHES (never re-closes), so a too
   early release changes the entry itself. Check the run still settles.
2. `reelout_delay` (now 4.2, same file) — LOWER it in steps of 0.2. Same
   mechanism as the trigger, via the timer instead of the force.
3. `entry_depower` (now 0.34) — RAISE it in steps of 0.005. Unloads the wing
   during dive/hold, so the swoop loads the tether less. Changes the entry
   trajectory; check settling and the elevation floor.
4. `depower_blend_time` (now 2.5) — LOWER it in steps of 0.2. A shorter ramp
   reaches `entry_depower` sooner, unloading the wing earlier.
5. `depower_setpoint` (now 0.274) — RAISE it in steps of 0.001, LAST: the file
   itself documents a ceiling at 0.278 for the optimized path at 150 m
   (min_feasibility_margin = 0.9, simple_opt_reelout.jl refuses to fly above
   it), so only 0.004 of headroom is left. Raising it also costs turn authority
   (c1 drops).

Not in fc_settings_reelout.yaml but the most direct lever of all: `f_high`
(now 8000, in wc_settings.yaml) — LOWER it (e.g. 7600) so the winch's upper
force controller starts shedding earlier. It is a WinchControllers knob, not an
FC one; keep it below the plant's 8400 N rating with margin.

## TODO
- record the baseline: current settings and the 8590 N peak (the archive in
  output/ already carries the settings of the last full run)
- change ONE knob per run, in the order above, +- one step, and compare peaks
- if a knob has no effect, revert it and continue with the next
- after each run, check the run still settles onto the pattern (the peak is not
  the only thing that matters)

## Success criteria
Peak force <= 8400 N

## Simulations
When you run the simulations to determine the peak force:
- reduce the simulation time to 40 s, which is sufficient to determine the entry peak
- run the script simple_opt_reelout.jl using kaimon in the REPL started by the user
- disable plotting (SHOW_PLOTS = false before the include)
- the success check in print_fig8_metrics uses require_final = true, which a
  40 s run can never pass (phase 5 is never reached); read the peak directly
  from the log instead:
  maximum(Float64.(getindex.(sl.winch_force, 1)))

