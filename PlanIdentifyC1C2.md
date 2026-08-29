# Re-identification of the turn-rate law needed
Because the mass of the kite changed, C1 and C2 need to be identified again.

The table turn_rate_coeffs.yaml need to be updated.

Probably all old values should be replaced.

For depower, using the range that is used by simple_opt_reelout.jl is sufficient.

For body_damping, 0,0,40 can always be used.
## The sweep must be flown against THIS repo's project

The tool is V3Kite's `examples/steering_test_v3.jl`, one row per run. (There is no
`build_turn_rate_table.jl` in V3Kite any more, although three error messages in
`src/turn_rate_table.jl` still name it.)

Its `PROJECT` defaults to `"system_reelout.yaml"`, which is **V3Kite's own** project,
and that is not the plant this repo flies:

| | V3Kite `sim_settings_reelout.yaml` | this repo's `settings_reelout_150m.yaml` |
|---|---|---|
| `kite.mass` | 6.2 kg | **10.9926 kg** |
| `d_tether` | 4.0 mm | **8.0 mm** |
| `sample_freq` | 20 Hz | 90 Hz |
| `rel_tol` | 0.01 | 0.002 |
| `winch.max_force` | 4000 N | 8400 N |
| `kite_settings` | `kite_settings_psm.yaml` | `kite_settings_psm_kernel.yaml` |

Run as shipped, the sweep would re-identify the light kite on a 4 mm tether — which is
exactly what the current table already is (`conditions.system: "system_reelout.yaml"`),
so the mass would not have changed at all. Set instead:

```julia
PROJECT = "/home/ufechner/repos/SimpleKiteControllers.jl/data/system_reelout_150m.yaml"
```

An absolute path is all that is needed, and nothing has to be copied between the
repos. `init` derives `data_path` from the project file's own directory
(`project_data_path`), and `project_file` looks for each `system:` entry **beside the
project first, under `v3_data_path()` after**. So `settings_reelout_150m.yaml` and
`kite_settings_psm_kernel.yaml` are taken from this repo, while `struc_geometry.yaml`,
`cfd_aero_geometry.yaml`, `vsm_settings.yaml`, `settle_settings_default.yaml` and
`heading_settings_psm.yaml` still resolve to V3Kite's `data/`, the only place they
exist.

Do **not** fix the mass by editing V3Kite's `sim_settings_reelout.yaml` instead: the
settled-state cache key (`settled_state_path`) carries the project's basename but not
the wing mass, so an in-place mass edit would silently reuse the 6.2 kg settled state.
Switching the project changes the `_sys` tag and forces a re-settle.

Keep unchanged in the script:

- `V_WIND = 9.51`, `TETHER_LENGTH = 150.0`, `DT = 0.05/3`. These are the script's own
  arguments, not project values. Keeping them makes the new rows comparable with the
  old ones, and `DT` must stay equal to the table's `conditions.dt` because an
  interpolated `delay` is rounded up to a multiple of it.
- `set_data_path(v3_data_path())`, which only decides where `save_log` writes
  `tmp_steering`.
- `elevation` is not an argument at all; it comes from the project (73° either way).

Verify before trusting a row:

- `Settings(PROJECT).mass` prints `10.9926`.
- the run re-settles (a new `settled_*_syssystem_reelout_150m_*.arrow`) instead of
  loading a cached state.

Then set `system: "system_reelout_150m.yaml"` in the table's `conditions:` block.

## Damping: fly start [0,0,40] / sim [0,0,32], key the row [0,0,40]

Fly the sweep the way the runs are flown: `BODY_START_DAMPING = [0.0, 0.0, 40.0]`,
`BODY_SIM_DAMPING = [0.0, 0.0, 32.0]` in `steering_test_v3.jl`, which is already its
default. Settling starts at 40 and decays to the 32 the sweep is then flown at, exactly
as every example here does — all six call `init` with
`body_start_damping = fcs.body_damping` and `body_sim_damping = 0.8 .* fcs.body_damping`,
against `body_damping: [0, 0, 40]` in both `fc_settings.yaml` and
`fc_settings_reelout.yaml`.

Key the row `[0.0, 0.0, 40.0]`, the START value. **No code changes.** That is what
`fcs.body_damping` holds and what all eight lookups pass
(`simple_fig8.jl:265`, `simple_fig8_live.jl:342`, `simple_opt_fig8.jl:354`,
`simple_reelout.jl:312`, `simple_opt_reelout.jl:780`, `optimize_fig8.jl:115`,
`awetrim_client.jl:939`, `reelout_feasibility.jl:126` and `:170`), it is what
`reload_turn_rate_table!` anchors `V3_TURN_RATE_C1`/`C2` on, and it is the convention
`FC_Settings.body_damping`'s docstring already states: one value fixes both the
settling transient and the 0.8x floor the run flies, "so it is the key
`turn_rate_coeffs` is looked up with".

So `[0,0,40]` in the table means "started at 40, flown at 32" from now on. Record the
pair explicitly in each new row so the label cannot be misread:

```yaml
    body_damping:                    # lookup key: what settling STARTED from
      - 0.0
      - 0.0
      - 40.0
    body_sim_damping: [0.0, 0.0, 32.0]   # what the sweep was FLOWN at
```

`_parse_turn_rate_entry` reads named keys only, so the extra key is ignored by the
loader and is pure provenance.

This is a third reason to replace every old row. The 2026-08-10 `[0,0,40]`/0.23 row was
deliberately flown at 40 THROUGHOUT (floor equal to the start value), so it carries the
same key as the new rows while describing a damping no run has ever flown; the older
rows predate the floor entirely and do not state what they decayed to. Mixing them with
rows that mean "40 → 32" under one key is exactly the confusion to avoid.

## Depower: identify at 0.25, 0.30, 0.35, 0.40

### What the script flies today

`fly_opt_depower: true` in `data/traj_opt.yaml`, so the ladder is:

| phase | `rel_depower` | source |
|---|---|---|
| 0 park | 0.274 | `fcs.depower_setpoint` |
| 1-2 dive / hold | 0.34 | `fcs.entry_depower` |
| 3-4 transition / fig8 | the optimizer's own, `(l_dp - 0.6)/5 + 0.1010` | `depower_flown_opt`, updated at every re-optimization |
| 5 final | 0.35 | `fcs.depower_final` |

`depower_blend_time` and `path_blend_time` ramp between the rungs, so everything in
between is traversed as well.

Measured across every run in `output/scenarios/`, all flown with
`fly_opt_depower: true`:

| scenario | rel_depower range | mean over phase 4 |
|---|---|---|
| v03 / v03_2 | 0.290-0.295 | 0.291 / 0.293 |
| v04 (x3) | 0.275-0.287 | 0.280 |
| v05 | 0.268-0.276 | 0.272 |
| v06 | 0.264-0.269 | 0.267 |
| v07 | 0.262-0.265 | 0.264 |
| v08 | **0.260**-0.264 | 0.263 |
| v09 | 0.271-0.281 | 0.274 |
| v10 | 0.294-0.297 | 0.296 |
| v11 | 0.311-**0.319** | 0.314 |

The optimizer's depower is **U-shaped in wind**, not monotone: a minimum near 0.26 at
8 m/s, rising to 0.29 at 3 m/s and 0.32 at 11 m/s. Together with the entry and final
rungs the flown span over 3-11 m/s is **0.260 .. 0.35**.

Only two values are ever LOOKED UP, though: `fcs.depower_setpoint` = 0.274
(`simple_opt_reelout.jl:780`, `awetrim_client.jl:939`, `reelout_feasibility.jl:126`)
and `fcs.depower_final` = 0.35 (`reelout_feasibility.jl:170`). The curvature gate
therefore already scores at 0.274 while phases 3-4 fly 0.26-0.32 — a pre-existing gap,
not one this re-identification introduces.

### The grid

**0.25, 0.30, 0.35, 0.40**, four sweeps.

- Brackets the measured 0.260-0.35 with margin at both ends. The margin is not
  optional: every number above was flown at 6.2 kg, and at 10.9926 kg the optimizer
  needs more lift and will ask for LESS depower, so the range moves — and
  `depower_setpoint` itself is due for a retune at the new mass
  (`oldplans/PlanFix3m_per_s.md`, step 3).
- Keeps 0.25 a real row, so `reload_turn_rate_table!`'s hardcoded
  `turn_rate_coeffs([0.0, 0.0, 40.0], 0.25)` anchor for `V3_TURN_RATE_C1`/`C2` resolves
  to an exact hit instead of an interpolation — or, if the row were missing entirely,
  to an `@error` that leaves both constants at `NaN`.
- 0.05 spacing is dense enough: `c1` is interpolated log-linearly and decays close to
  exponentially with depower.
- Two rows would technically serve the two lookups, but the table refuses to
  extrapolate, so any retune of `depower_setpoint` or `depower_final` past the end of a
  tight grid throws at startup.

Note when comparing against the archived numbers: their rel-equivalents were computed
with each run's own `AWETRIM_V3KITE_DEPOWER_OFFSET`, which was 0.099 before
2026-08-29 and is 0.1010 now, so older rows read up to 0.002 low.

## The fig8 project moves to the new mass too

`data/settings_fig8_200m.yaml` is now `mass: 10.9926`, raised from 6.2 alongside
`settings_reelout_150m.yaml`. This is what makes "replace all old rows" safe: the table
is loaded at package load through `system_fig8_200m.yaml`, so once every row describes
the 10.9926 kg wing the fig8 runs read a table that matches their own plant instead of
one identified for a kite 4.8 kg lighter.

Consequence to expect rather than debug: `fc_settings.yaml`'s figure-eight tuning
(`depower_setpoint` 0.27, `entry_depower` 0.37, the steering gains, the pattern shape)
was tuned at 6.2 kg and is now as stale as the reel-out side's. Treat a changed fig8
run as the mass change showing, not as a regression.

Every settings file in `data/` now carries 10.9926 kg — `settings_fig8_150m.yaml`,
`settings_fig8_200m.yaml`, `settings_fig8_300m.yaml`, `settings_reelout_150m.yaml` and
`settings_reelout_180m.yaml` — so no project can be flown against the new table at the
old mass by accident. `settings_reelout_150m.yaml` holds the full explanation of the
value; the other four point at it in one line.

The archived copies under `output/scenarios/*/` keep 6.2 kg and must stay that way:
they are the record of what each run actually flew, and every scenario there predates
the change.

## Rewriting the tests

Replacing every row breaks `test/test_fig8_controller.jl` in three places. Only one of
them is really about the table; the other two only borrowed it for a number.

### What already survives untouched

The two synthetic-table testsets — `"turn_rate_coeffs interpolation (conditions
block)"` (521-575) and `"turn_rate_coeffs own-conditions rows are ordinary data"`
(577-600) — swap `_TURN_RATE_TABLE[]` for a hand-built table and restore it in a
`finally`. They already cover log-linear `c1`, linear `c2`/`delay`, the `delay`
round-up to a `conditions.dt` multiple, exact hits not being interpolated, refusal to
extrapolate, `interpolate = false`, an unusable row throwing on an exact hit, and an
unknown `body_damping` throwing. None of that depends on which rows ship, so none of it
changes. Add one thing: a SECOND `body_damping` group to the synthetic table, so
"damping is never interpolated across" keeps a positive case once the shipped file has
only `[0, 0, 40]` rows.

### The one testset that must be rewritten

`@testset "turn_rate_coeffs"` (482-519) pins twenty assertions to values that are all
about to be deleted: `c1` at `[0,0,40]`/0.25, 0.23, 0.40 and 0.55, and the
`[10,10,40]` / `[20,20,40]` rows. Do not re-pin it to the new numbers — that only moves
the same maintenance burden to the next re-identification. Test the SHIPPED FILE's
contract instead, which is what this testset is uniquely able to do:

1. Every row of the loaded table either passes `_is_usable_turn_rate_entry` or carries a
   non-passing `outcome`, and no row is silently half-valid.
2. **Every depower the shipped settings actually look up resolves.** For both
   `fc_settings.yaml` and `fc_settings_reelout.yaml`:
   `turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)` and, for the reel-out
   file, `..., fcs.depower_final)`. This is the assertion worth having: it fails the
   moment someone retunes a depower past the end of the identified range, which is the
   actual bug this file can produce, and it never needs editing when the coefficients
   change.
3. `V3_TURN_RATE_C1` and `V3_TURN_RATE_C2` are finite and equal
   `turn_rate_coeffs([0.0, 0.0, 40.0], 0.25)` — this is what catches
   `reload_turn_rate_table!`'s `@error` path, which leaves both at `NaN` rather than
   failing loudly.
4. `c1` is strictly decreasing in depower across the rows, computed FROM the table
   rather than written out. A physical invariant that holds for any valid
   identification.
5. Integer input still converts (`turn_rate_coeffs([0, 0, 40], 0.25)`), and an
   unidentified `body_damping` still throws.

Keep exactly ONE pinned number as a canary against an accidental edit of the YAML: `c1`
at the `[0,0,40]`/0.25 anchor, with a comment saying it is expected to change on a
deliberate re-identification and nowhere else.

The cross-damping assertions (`[20,20,40] < [10,10,40] < [0,0,40]`) are DELETED, not
ported: the grid above identifies one damping level, so there is no second group in the
shipped file to compare against, and "more damping is less agile" is a claim about the
plant rather than about this code.

### The two borrowed lookups

Neither is about the table, so both get literals and stop depending on it:

- `min_turn_radius` (609-610) uses `turn_rate_coeffs([10,10,40], 0.25).c1` against
  `[0,0,40]`'s only to show the radius grows as `c1` falls. Pass two `c1` literals.
- `reelout_feasibility` (1115) takes `coeffs = turn_rate_coeffs([10,10,40], 0.25)`
  purely to have some `c2`/`delay` for a `ReeloutFeasibility`. Use literals.

### Order of work

Rewrite the testset FIRST, against the table as it stands. Points 1-5 all pass on the
current file, so a green suite before the sweeps proves the new tests actually test
something; a red one after them then means the data, not the assertions.

## The table builder is back

`examples/build_turn_rate_table.jl` is a port of V3Kite.jl's script of the same
name, which was never deleted — it sits on that repo's unmerged `fig8` branch
(commit `9f1be96`, 2026-08-01) and so was never reachable from its main, which is
why the error messages pointing at it had gone stale. Those pointers in
`src/turn_rate_table.jl` now name the local copy.

It lives here because it writes THIS package's data file and flies THIS package's
project, which is what the first section of this plan requires.

What the port changed:

- `init(...; body_damping)` -> `body_start_damping` / `body_sim_damping`, carrying
  the [0,0,40] -> [0,0,32] decision above.
- `step!(...; set_length)` -> a caller-owned `WinchPosController` and
  `set_torque = winch_torque!(wpc, s, l0)`; V3Kite has been torque-only since.
- Added `aero_mode = ContinuousAero()`, `vsm_interval = 5`, `remake_model = false`,
  matching the current `steering_test_v3.jl`.
- `PROJECT` is `project_file("system_reelout_150m.yaml")` at 150 m, not V3Kite's
  own `system_reelout.yaml` at 200 m, and the file is written under
  `skc_data_path()`.
- Grid default is the depower grid above at one damping level.
- Each row now records `body_sim_damping` and `date` as provenance.
- New `_check_conditions`: the script refuses to write a row unless the file's
  `conditions:` block matches the constants it sweeps at. So the conditions update
  this plan calls for is now enforced rather than remembered — the FIRST run will
  stop and say so, because the block still reads `system_reelout.yaml` / 200 m.
- Dropped the old 200 m-vs-150 m `c1` sensitivity check: it compared against a
  constant identified at 6.2 kg.

Kept as it was: incremental write after every cell, the refusal to let a failed
re-run demote a passing row, and the skip-if-already-passing resume.

Run it as `include("examples/build_turn_rate_table.jl")` (definitions only), then
`build_turn_rate_table()`.
