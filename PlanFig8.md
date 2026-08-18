# Stop reel-out after a fixed number of figures of eight

Today `examples/simple_reelout.jl` has exactly ONE stop criterion for the winch:
the length setpoint reaching `fcs.reelout_l_max`. That is what freezes `l_set`
and what moves the flight phase from 3/4 to 5 ("final"). Add a SECOND,
independent criterion — `n_fig_eight`, the number of figures of eight flown —
so a run can be sized in laps instead of in metres. Whichever criterion fires
first ends reel-out; the rest of the run is flown at constant length exactly as
now.

(The original one-liner named `simple_fig6.jl`, which does not exist. The script
with the length stop is `examples/simple_reelout.jl`; `simple_fig8.jl` flies at
constant length and has no winch to stop. `data/fc_settings_reelout.yaml`, the
file named for the new key, is that script's settings file, which confirms it.)

**Scope: the WINCH stops, not the run.** The simulation still runs to
`sim_time`, in phase 5, after the criterion fires. Ending the run itself on a
lap count is a separate change and is not implemented here.

## Decisions taken

- **Reuse the existing lap counter, do not add a second one.** `fig8_n` /
  `fig8_idx_progress` ([simple_reelout.jl:334-337](examples/simple_reelout.jl#L334-L337))
  already count laps off the guidance's own path index (`fec.last_idx`
  unwrapped over `n_path`), and are already logged as `SysState.fig_8`. The
  criterion reads them; nothing new is counted.
- **The criterion is `fig8_idx_progress >= n_fig_eight * n_path`**, i.e. N
  COMPLETED traversals of the reference path counted from the moment the counter
  starts. Not `fig8_n >= n_fig_eight`: `fig8_n` is `1` while the FIRST lap is
  being flown (it is a "which lap am I on" display value), so comparing it
  directly would stop one lap early. The equivalent in `fig8_n` terms is
  `fig8_n >= n_fig_eight + 1`; use the progress variable and avoid the off-by-one
  entirely.
- **Laps are counted from `phase >= 4`, not from reel-out engagement.** That is
  where `fig8_n` starts, and it is what `fig_8` in the log already means, so the
  criterion and the logged lap count can never disagree. Consequence, worth
  stating in the docstring: reel-out engages at phase 3 + `reelout_delay`, which
  can precede phase 4, so the first fraction of a lap is not counted; and a run
  that never settles into phase 4 never fires this criterion at all —
  `reelout_l_max` remains the backstop in that case.
- **`n_fig_eight = 0` disables it**, the same convention `reelout_softstart` and
  `reelout_softstop` already use. Default `0` in `FC_Settings`, so every existing
  run and every archived config keeps its current behaviour bit for bit.
- **`reelout_l_max` stays**, unconditionally, as the second criterion and as the
  clamp on `l_set`. Two independent limits, `min` of the two in effect. It is
  also what `check_pattern_feasible` is evaluated at, and with a lap stop the run
  can only end SHORTER, so that check stays conservative.
- **Soft-stop keeps working, with time as the free variable instead of
  distance.** The length criterion knows the remaining distance and solves
  `stop_T = 2*remaining/v_cmd` so that `v_set` hits 0 exactly AT `reelout_l_max`.
  A lap-count stop is an event with no distance attached, so invert the relation:
  latch at `stop_T = 2 * fcs.reelout_softstop` (the same nominal duration, the
  same linear ramp from the current `v_cmd`) and let the final length land where
  it lands, below `reelout_l_max`. With `reelout_softstop = 0` (the current YAML
  value) both paths hard-stop, unchanged.
- **Name.** `n_fig_eight` as requested, kept in the `# --- REEL_OUT winch` block
  of the YAML and of the struct next to `reelout_l_max`, because that is what it
  is a sibling of. (`reelout_n_fig8` would match the prefix convention of the
  other four keys; not worth the churn if `n_fig_eight` is preferred.)

## Step 1 — the setting

`src/fc_settings.jl`, in the `# ---- REEL_OUT winch` block, right after
`reelout_l_max` (they are the two stop criteria and belong together):

```julia
"""
Number of complete figures of eight after which reel-out stops [-], the second
stop criterion beside [`reelout_l_max`](@ref); whichever is reached first ends
reel-out and enters phase 5. `0` disables it, leaving `reelout_l_max` the only
criterion (the pre-change behaviour). Counted from the same lap counter the log
records as `SysState.fig_8`, which starts at phase 4, so the partial lap flown
between reel-out engaging (phase 3 + `reelout_delay`) and the pattern settling
is not counted, and a run that never reaches phase 4 never fires this criterion.
NOTE that `depower_final` is tuned for the force at a full 150 -> 350 m reel-out;
stopping short leaves a different length and a different force, so a lap-limited
run may want its own `depower_final`.
"""
n_fig_eight::Int64 = 0
```

`data/fc_settings_reelout.yaml`, in the same block, with the aligned trailing
comment the file uses:

```yaml
  n_fig_eight:        0        # Laps after which reel-out stops; 0 = length only [-]
```

No loader change: `FC_Settings` is `@with_kw`, a missing key falls back to the
default and an unknown key errors, and `FCS_OVERRIDES` accepts any field by
`hasfield`, so `examples/optimize_fig8.jl` can sweep it for free.

## Step 2 — the criterion in `examples/simple_reelout.jl`

The lap counter is updated at [simple_reelout.jl:381-393](examples/simple_reelout.jl#L381-L393),
BEFORE the winch block, so the criterion can be evaluated in the same step the
lap completes.

1. **Latch.** Next to `stop_start`/`stop_v_entry`/`stop_T`
   ([:326-328](examples/simple_reelout.jl#L326-L328)) add
   `reelout_done = false` and `stop_reason = ""` (`"length"` or `"laps"`, for the
   RESULTS block and for the log's own record of why it stopped).

2. **Reel-out gate.** Replace `l_set < fcs.reelout_l_max` in the branch condition
   ([:400-401](examples/simple_reelout.jl#L400-L401)) with `!reelout_done`.
   `l_set` reaching `reelout_l_max` sets `reelout_done` (below), so the length
   path is unchanged in behaviour; the flag simply becomes the single thing both
   criteria write to and everything else reads.

3. **Lap latch**, inside the reel-out branch, next to the existing soft-stop
   latch ([:432-437](examples/simple_reelout.jl#L432-L437)):

   ```julia
   # Second stop criterion: N COMPLETE laps since the counter started at phase 4.
   # `fig8_idx_progress`, not `fig8_n`, which reads 1 during the first lap.
   if isnan(stop_start) && fcs.n_fig_eight > 0 &&
      fig8_idx_progress >= fcs.n_fig_eight * n_path
       global stop_reason = "laps"
       if fcs.reelout_softstop > 0 && v_cmd > 0
           global stop_start = t
           global stop_v_entry = v_cmd
           # No remaining distance to solve T from — unlike the reelout_l_max
           # latch, the length is the free variable here. Same nominal duration.
           global stop_T = 2 * fcs.reelout_softstop
       else
           global reelout_done = true   # hard stop, as reelout_l_max does today
       end
   end
   ```

   Order matters: put this AFTER the `reelout_l_max` soft-stop latch, so that if
   both would fire on the same step the length criterion (which lands exactly on
   `reelout_l_max`) wins and `stop_reason` says so.

4. **Completion.** After `l_set` is integrated
   ([:438-440](examples/simple_reelout.jl#L438-L440)):

   ```julia
   if l_set >= fcs.reelout_l_max
       global reelout_done = true
       isempty(stop_reason) && (global stop_reason = "length")
   elseif !isnan(stop_start) && stop_reason == "laps" && t - stop_start >= stop_T
       global reelout_done = true   # the soft-stop ramp has run out
   end
   ```

5. **Phase 5.** Replace the trigger at
   [:367](examples/simple_reelout.jl#L367) — `phase in (3, 4) && l_set >= fcs.reelout_l_max`
   — with `phase in (3, 4) && reelout_done`. It is evaluated from the PREVIOUS
   step's flag, exactly as it reads the previous step's `l_set` today, so the
   one-step offset is unchanged.

6. **Startup banner** ([:248-249](examples/simple_reelout.jl#L248-L249)): print
   both criteria, e.g. `... stopping at 350 m or after 6 figures of eight.`, and
   the length only when `n_fig_eight == 0`.

## Step 3 — results and log

- The unconditional tether line ([:662-667](examples/simple_reelout.jl#L662-L667))
  currently prints `target 350.0 m` and calls it a day. Print the reason too, and
  add `"stop_reason" => (stop_reason, "criterion that ended reel-out: length, laps, or none")`
  and `"laps_reeled" => ...` to `reelout_summary["tether"]`, so a lap-limited run
  is not mistaken for a run that failed to reach its target length. `stop_reason`
  is `""` when the run ended with reel-out still going — report that as `"none"`.
- No new `var_XX` slot. `fig_8` already carries the lap count and `var_10`
  already carries `l_set`; the stop is fully reconstructible from those two.

## Step 4 — documentation

Every one of these describes the length stop as the only one:

- [`examples/simple_reelout.jl`](examples/simple_reelout.jl) docstring: the
  opening paragraph ("until `fcs.reelout_l_max`"), the phase-5 paragraph
  ([:89-95](examples/simple_reelout.jl#L89-L95)), and the `# Parameters` list of
  REEL_OUT fields ([:108-118](examples/simple_reelout.jl#L108-L118)).
- [`docs/reelout_state_machine.md`](docs/reelout_state_machine.md): the `3 or 4 -> 5 final`
  row (line 52), the **reel-out** branch condition (line 99), the soft-stop
  bullet (lines 122-128), and the latched-variables table (lines 173-180, add
  `reelout_done`/`stop_reason`). The two SVGs are generated from
  `reelout_state_machine.drawio` — update the .drawio conditions and regenerate
  with the two `drawio -x` commands the file's header already gives.
- [`docs/control_algorithm.md`](docs/control_algorithm.md) lines 167-168, the
  soft-stop bullet.
- [`src/fig8_metrics.jl`](src/fig8_metrics.jl) line 407: `require_final`'s
  docstring says the phase-5 check confirms "`l_set` reached `reelout_l_max`" —
  it now confirms reel-out FINISHED, by either criterion. The check itself is
  unchanged.
- [`data/fc_settings_reelout.yaml`](data/fc_settings_reelout.yaml) header
  comment, which lists what the `reelout_*` keys govern.
- `docs/fig8_tuning_log.md` gets an entry only if the change is used to retune
  something; the mechanism itself belongs in the docs above.

## Verification

No unit test: the criterion is script-level logic in an example, like
`reelout_softstart`/`reelout_softstop` before it, and `test/runtests.jl` covers
`src/` only. Verify by running, in this order:

1. `n_fig_eight: 0` — a full run must be bit-identical to the current one
   (same `output/*.yaml` summary apart from the new keys). This is the
   regression check that matters.
2. `n_fig_eight: 3`, `reelout_l_max` left at 350 — expect `stop_reason: laps`,
   the final `var_10` short of 350 m, `fig_8` reaching 4 (3 complete laps, the
   4th in progress) at the moment `sys_state` steps to 5, and the phase-5 tail
   flown at that shorter length.
3. `n_fig_eight: 99` — the lap criterion must never fire and the run must match
   case 1 exactly, confirming the `min`-of-two-criteria wiring has no side effect
   when the length wins.

Each is a several-minute run, so run them yourself:
`include("examples/simple_reelout.jl")`.
