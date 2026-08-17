# Improve and unify the winch controllers

Two winch controllers are in play for `examples/simple_reelout.jl`: V3Kite's OWN
`WC_Settings`/POSITION torque loop (`data/wc_settings.yaml`, unchanged by this
plan) and WinchControllers.jl's `WinchController`/`WCSettings` (`rc`/`rcs`),
layered on top of it to drive the REEL_OUT law. This plan covers making the
second one's settings visible and its force floor actually protective.

## WinchControllers.jl's own settings were invisible — now `data/wc_settings_reelout.yaml`

**Problem.** `rcs` (`WCSettings`) was built with `WCSettings(dt = s.dt)` and only
five of its ~35 fields (`kv`, `f_low`, `f_high`, `v_sat`, `t_startup`) were ever
set, by hand, from `FC_Settings.reelout_*`. Everything else — every
LowerForceController/UpperForceController gain (`pf_low`, `if_low`, `df_low`,
`nf_low`, `kbf_low`, `ktf_low`, `fac`, and the `_high` counterparts), `t_blend`,
`max_acc`, `winch_iter`, `mode` — sat at WinchControllers.jl's own package
defaults, unset and invisible anywhere in this repo.

`WinchControllers.update(rcs)` (its own YAML loader) was deliberately NOT the
fix: it discovers its file via `KiteUtils.get_data_path()`/`wc_settings()`, the
SAME project key (`wc_settings:`) that already names V3Kite's `WC_Settings`
file. The two schemas share no field names, so calling it on the current
project would silently leave every WinchControllers field at its default —
exactly the state this was meant to fix, just less obviously.

**DONE.**

- `data/wc_settings_reelout.yaml` — WinchControllers.jl's full native
  `wc_settings:` schema, one file, fully commented. Seeded from
  WinchControllers.jl's plain struct defaults plus the five values that were
  already being set by hand (`mode: "reelout"`, `kv: 0.0408`, `f_low: 350.0`,
  `f_high: 7600.0`, `v_sat: 8.0`, `t_startup: 0.25`), so loading it changes
  NOTHING that was already running — verified directly (loaded values matched
  the prior hard-coded ones exactly).
- `load_yaml_fields!(obj, filename, section; path)` (`src/fc_settings.jl`,
  exported) — a small generic YAML-into-mutable-struct loader (reflective:
  `hasfield`/`setfield!`/`fieldtype` on `typeof(obj)`), so `src/` does not need
  to depend on WinchControllers.jl just to load its settings type.
  [`FC_Settings`](@ref)'s own constructor is now built on it too, rather than
  duplicating the same loop.
- `examples/simple_reelout.jl`: `rcs = load_yaml_fields!(WCSettings(dt = s.dt),
  "wc_settings_reelout.yaml", "wc_settings")`, then `rcs.dt = s.dt` again (the
  file's `dt` is a placeholder, not a tuning choice).
- Removed the now-redundant `FC_Settings.reelout_kv`/`reelout_f_low`/
  `reelout_f_high`/`reelout_v_max`/`reelout_t_startup` fields — keeping both
  would have meant two competing sources of truth for the same five values.
  `data/fc_settings_reelout.yaml` and every docstring referencing them updated
  to point at `data/wc_settings_reelout.yaml` instead.

**VERIFIED** (standalone, no V3Kite needed): `FC_Settings("fc_settings_reelout.yaml")`
and `load_yaml_fields!(WCSettings(dt=0.02), "wc_settings_reelout.yaml",
"wc_settings")` both load without error and reproduce the exact prior values
(`mode=reelout kv=0.0408 f_low=350.0 f_high=7600.0 v_sat=8.0 t_startup=0.25
max_acc=8.0 t_blend=0.25 fac=0.25`). No test file referenced the removed
fields. Full test suite (`test/runtests.jl`) and a real closed-loop run were
NOT re-checked after this specific change — do that before trusting it fully.

## Force floor was dormant before phase 3 — now active from t = 0

**Problem, reported by the user and then confirmed from a plot they attached**
(`output/archives/2026-08-16_104935/reelout_150m.yaml`): tether force sags to
about 50 N around t ~ 5.2 s, during the dive (phase 1). WinchControllers.jl's
LowerForceController exists specifically to catch this — its `active`
flag is monitored every step (`get_v_set_out` -> `solve` -> checks `force <
f_low`) regardless of which state currently has the mixer's output, confirmed
by tracing the code (two independent passes, one by hand and one by an agent,
agreed) — so there is no bug in the controller's OWN activation logic.

The actual gap: `rc`/`calc_v_set` was only ever stepped inside the
`phase >= 3` REEL_OUT block. Before that (park/dive/hold, phases 0-2), `l_set`
is held flat at the settled length and NOTHING is watching the force at all —
the LowerForceController never got a chance to run, let alone activate.

### Attempt 1 — step `rc` throughout the entry — CLOSED, made it worse

Stepping `rc`/`calc_v_set` every iteration (not just from phase 3 on), gating
`l_set` to move only while `get_state(rc) == Int(wcsLowerForceLimit)` (state
0), looked right and passed an isolated component test. **Live it was a
regression**, confirmed from a plot: `v_reelout` spiked to `+8 m/s` (`v_sat`,
saturated) during the dive — REELING OUT, not in — and the dive stretched to
~18 s instead of a few.

**Root cause:** `rc`'s own `SpeedController` sub-component is ACTIVE
(`set_inactive(wc.sc, (lfc.active || ufc.active) && ...)`) whenever the force
limiters are off, i.e. essentially the whole entry, since nothing is meant to
move the drum yet. Gating only what *I* did with the mixer's output did
nothing to stop the SpeedController's OWN integrator from accumulating
`v_set_in - v_act = kv*sqrt(force) - v_act` continuously in the background,
with `v_act` pinned near 0 the entire time (nothing was actually driving the
drum). Over ~20 s that integrator winds up hugely and, once anything about the
state changes, dumps a large stale command — landing exactly on `v_sat`,
consistent with a saturated wound-up integrator. The isolated test that
"passed" only stepped the FULL `rc`, so it exercised this exact interaction
but the test was too short to show the windup — a longer isolated run would
have caught it; a REAL closed-loop run did.

### Attempt 2 — standalone `LowerForceController`, decoupled from `rc` — DONE and VERIFIED

Use a bare `guard_lfc = LowerForceController(rcs)` for phases 0-2 instead of
`rc` — no `SpeedController`, no mixer, so there is nothing to wind up while
its output is unused. Stepped by hand through the same setters `calc_v_set`
uses internally (`set_reset`, `set_f_set`, `set_v_sw`, `set_v_act`,
`set_tracking(0.0)`, `set_force`, `get_v_set_out`, `on_timer`); `l_set` moves
only while `guard_lfc.active`. `rc` itself is now untouched until phase 3,
exactly as before EITHER attempt — its soft-start clock, integrators etc. see
nothing before reel-out is meant to begin.

**VERIFIED in isolation**: at 500 N (above `f_low`), `guard_lfc` stays
inactive with `v_guard` EXACTLY `0.0` for the whole 2 s tested — no drift, no
windup. At 50 N, it activates on the very first step and reels in smoothly and
monotonically (0 -> -0.16 -> -0.5 -> -1.0 -> -1.7 -> -3.4 -> -5.1 m/s over 6 s)
— no positive excursion at any point.

**VERIFIED in closed loop** (real V3Kite run, `output/archives/2026-08-16_113800`):
- Minimum force during the entry: **163 N** (t = 3.8 s, guard actively reeling
  in at `-0.41 m/s` at that moment) — sagging, but nowhere near the ~50 N (or
  the near-zero the very first, unguarded plot showed) from before.
- `v_reelout` over the WHOLE entry (phases 0-2): max `+0.0 m/s`, min
  `-1.59 m/s` — never once reels out, no trace of the `v_sat` spike.
- Whole-run tether-force CV back to **3.2 %** (was 41 % with the Attempt-1 bug).
- All 7 fig8 success criteria pass.
- Full test suite (`test/runtests.jl`): **120/120 pass.**

**Still open, lower priority:**
- Whether the guard's transient reel-in measurably perturbs the entry's
  timing or the margin `check_pattern_feasible` reports (evaluated at the
  fixed `l_tether`, which the guard can now transiently shrink) — the dive in
  the verified run took ~23.5 s, longer than a typical unguarded dive; not
  established whether that is the guard's effect or ordinary run-to-run
  variation.

## Move winch controller from V3Kite to WinchControllers.jl
There are two structs WCSettings and two yaml files. Merge them such that there is only
one struct with all the fields.

Then move all winch control related code from V3Kite to the WinchControllers package.

V3Kite shall use the WinchControllers package and continue to work as before.

The tests in WinchControllers.jl and in V3Kite must pass.

I already dev'd both packages in SimpleKiteControllers.jl.

### DONE and VERIFIED (2026-08-16)

**The constraint that shaped the design.** V3Kite's winch functions took
`s::V3KITE` and reached into `s.sys.winches[1]` (SymbolicAWEModels) and `s.dt`.
WinchControllers.jl must NOT gain a dependency on any kite model, so the moved
code takes PLAIN SCALARS — force, speed, length, `dt`, and the drum's
radius/gear ratio/friction. V3Kite then keeps one-line `V3KITE`-typed methods
that pull those scalars out of the model and delegate, which is what makes
"continue to work as before" true: **V3Kite's public API is byte-for-byte the
same**, so SimpleKiteControllers' three example scripts needed no edit at all.

**One struct.** WinchControllers' `WCSettings` absorbed all 9 of V3Kite's
former `WC_Settings` fields (`winch_pos_kp`, `winch_speed_k`, `winch_speed_ti`,
`winch_torque_limit`, `winch_ff_scale`, `winch_force_tau`, `winch_len_kp`,
`winch_damp`, `winch_force_min`) — 45 fields total, no name collisions (V3Kite's
are all `winch_*`; the only pre-existing `winch_` field was `winch_iter`). `dt`
gained a default (`0.02`) so `WCSettings()` constructs, which the torque
controllers need for their own field defaults.

**One file per project, not per controller.** Both packages' `data/wc_settings.yaml`
now carry the union. A key either package omits still falls back to the struct
default, so KiteControllers' three `wc_settings*.yaml` files keep loading
unchanged.

**Moved to `WinchControllers/src/torque_controllers.jl`:** `WinchPosController`,
`WinchForceController`, `winch_position_torque!`, `winch_force_torque!`,
`force_to_torque`, `winch_acc_limit`. WinchControllers gained `DiscretePIDs`
(for the inner speed PI) and exports all six.

**Left in V3Kite:** `const WC_Settings = WCSettings` (an alias, not a second
type), the `V3KITE`/`Settings`/`sys` delegating methods, and a new
`drum_params(s)` helper. `WC_Settings(filename)`'s loader became TYPE-AWARE —
the merged struct is no longer all-`Float64` (it has `mode::String`,
`test::Bool`, `max_iter::Int64`), so blanket `Float64(v)` coercion had to go —
and it now ERRORS on an unknown key instead of silently ignoring it.

**VERIFIED — all four suites green:**

| suite | result |
|:--|--:|
| WinchControllers.jl `Pkg.test()` (incl. Aqua) | **126/126** |
| V3Kite `Pkg.test()` | **405/405** |
| SimpleKiteControllers `test/runtests.jl` | **120/120** |
| end-to-end `simple_reelout.jl` | all 8 criteria pass |

The end-to-end run is the strongest evidence: every reported number is
**bit-identical to the pre-refactor run** — RMS d 1.24°, force mean 3467 N
(CV 3.9 %), reel-out force mean 3348 N, power mean 7943 W, ring period 2.62 s,
E 682.1 kJ. That run drives the POSITION winch through `set_length`, so the
moved `winch_position_torque!` is genuinely exercised and reproduces exactly.

**Export ambiguity — checked, absent.** `simple_reelout.jl` imports BOTH
`using V3Kite` and `using WinchControllers: WCSettings, …`, which would normally
be an ambiguity error for a name exported by two modules. It is safe here only
because V3Kite `import`s those names from WinchControllers and EXTENDS them
rather than defining its own, so both modules export the identical binding —
confirmed at runtime, all five shared names `===` each other. Keep it that way:
redefining any of them in V3Kite instead of importing would break every script
that loads both.

**KiteControllers.jl is the other dependent** (compat `"0.5.4"`, and it does
`@reexport using WinchControllers`, so the six new exports flow through it to
its users). Checked for collisions against all six names plus `drum_params`:
**zero**. Its compat range `>=0.5.4, <0.6` also admits the release carrying this
merge, so no downstream edit is needed. (KiteModels does NOT depend on
WinchControllers — it depends on WinchModels, the drum/generator plant model,
which is a different package.)

### One struct, one file — this repo's two YAMLs collapsed too

Merging the structs was pointless while the caller still needed two files, so
`data/wc_settings_reelout.yaml` is **deleted** and its 36 keys are folded into
`data/wc_settings.yaml`. There was nothing to reconcile: the two key sets were
PERFECTLY DISJOINT (9 `winch_*` + 36 = 45 = the whole merged struct, zero
overlap), and all four `system_*.yaml` projects — the reel-out one included —
already named `wc_settings.yaml`.

`simple_reelout.jl` now loads ONE object where it used to load two:

```julia
wc  = WC_Settings(wc_settings(project))   # the whole merged struct
rcs = wc                                  # same object, two controllers read it
```

`init` copies the `winch_*` gains into V3Kite's own `WinchPosController` rather
than holding a reference, so sharing one mutable object between the two winches
is safe; `rcs.dt = s.dt` after `init` is the only mutation.

**This also fixed a latent wart.** `wc_settings_reelout.yaml` had to be loaded by
HARDCODED FILENAME, because the project's `wc_settings:` key was already claimed
by V3Kite's file — so switching projects could not switch the reel-out winch
tuning. It routes through the project key like everything else now.

**VERIFIED:** SimpleKiteControllers `test/runtests.jl` clean, and a reel-out run
on the single file reproduces the two-file run EXACTLY — RMS d 1.24°, force mean
3467 N (CV 3.9 %), reel-out force 3348 N, power 7943 W, ring period 2.62 s,
E 682.1 kJ, 175 → 350 m, all 8 criteria pass. Not one digit moved, which is the
check that matters: a merged settings file that silently changed a value would
show up here.

**Still open:**
- **Two `[sources]` path entries must be dropped once a WinchControllers release
  ships the merge**: `V3Kite/Project.toml` → `../WinchControllers.jl`, and
  `SimpleKiteControllers/examples/Project.toml` → `../../WinchControllers.jl`.
  Both are machine-specific and not committable as-is; each carries a comment
  saying so. The second was needed because this repo's manifest was pulling
  WinchControllers from the REGISTRY even though V3Kite was dev'd — so V3Kite's
  new dependency resolved to a copy without `torque_controllers.jl`.
- WinchControllers' version is still `0.5.6`; the merge is additive (new fields
  with defaults, new exports, nothing removed) so `0.5.7` is the right bump, not
  done here.
- V3Kite's own `data/wc_settings.yaml` still spells out only the `winch_*` half.
  That is deliberate — V3Kite drives only the torque controllers, and the
  omitted keys fall back to struct defaults — but if a V3Kite run ever needs
  WinchControllers' speed controller, they have to be added there.

## Investigate why V3Kite depends on WinchControllers
KiteModels.jl does not depend on WinchControllers. Why is V3Kite.jl depending on
WinchControllers?

### Answer: it did not, until the move above added it — and that inverted the layering

The two `step!` signatures are the whole story:

| package | winch inputs |
|:--|:--|
| KiteModels `next_step!` | `set_speed`, `set_torque`, `set_force` |
| V3Kite `step!` (before) | `set_torque`, **`set_length`**, `v_ff`, `speed_limit`, `acceleration_limit` |

KiteModels takes only PLANT-level inputs — the winch model consumes speed,
torque or force directly — so it needs WinchModels and no controller. V3Kite's
`step!` took `set_length`, which is a CONTROL concept: turning a length setpoint
into a drum torque is a cascaded loop with its own gains, saturations and state.
V3Kite embedded that loop, so once it moved to WinchControllers.jl the
dependency followed: a PLANT package depending on a CONTROLLER package.

The cost was concrete. `torque_controllers.jl` references only `WCSettings` and
`DiscretePID` from its new home — zero coupling to the rest of WinchControllers
— yet living there made every downstream V3Kite install pull NLsolve,
LineSearches, NLSolversBase, WinchModels, Timers, StructTypes and
DocStringExtensions. (An earlier count of 10 packages was wrong: FiniteDiff,
Distances and StatsAPI arrive via the SymbolicAWEModels/NonlinearSolve chain
regardless.)

### Option C — `step!` is torque-only, like KiteModels — DONE and VERIFIED

- **`step!` takes a torque and nothing else.** `set_length`, `v_ff`,
  `speed_limit` and `acceleration_limit` are gone; `set_torque` is applied as
  given, and omitting it holds the measured force.
- **`warmup!` takes a callback**, `winch_torque = (s, l_hold) -> torque`, so
  `init` can still relax against the winch mode the run will actually command
  without V3Kite knowing what a controller is. `nothing` holds the measured
  force. This was the design crux: a callback is just a function, so it costs no
  dependency.
- **`init` dropped `wc`** (no controller to build) and renamed `warmup_wfc` to
  `warmup_torque`.
- **Deleted from V3Kite:** the `winch_ctrl` field, `src/wc_settings.jl`, and both
  torque-controller wrappers. **Kept**, because they are PLANT properties:
  `force_to_torque` (self-contained again), `drum_params`, `winch_acc_limit` —
  the drum's acceleration limit belongs to the drum even though a controller
  enforces it.
- **New `examples/winch_adapter.jl`** — `winch_torque!`, `winch_force_hold!`,
  `load_wc_settings`. This is the plant/controller glue, and it belongs to the
  APPLICATION: V3Kite must not depend on a controller, WinchControllers must not
  depend on a kite model. It is duplicated in `V3Kite/examples/` and
  `SimpleKiteControllers/examples/` — keep the two in step.
- **Call sites converted:** V3Kite's 5 examples + 3 test files, and
  SimpleKiteControllers' 3 scripts. `simple_reelout.jl` is the delicate one: its
  `v_ff` and its non-default limits (`rcs.v_sat`, `rcs.max_acc` = 8 rather than
  the plant's 4 m/s²) are passed explicitly through the adapter.

**V3Kite no longer depends on WinchControllers at runtime.** It is a test/
examples dependency only, so its own CI still pulls it but downstream users do
not. `NLsolve` and the rest pruned out of V3Kite's manifest.

**VERIFIED — four suites, and the numbers:**

| suite | result |
|:--|--:|
| WinchControllers.jl | 126/126 |
| V3Kite | **395/395** (was 405; the drop is test rewrites — the `WC_Settings` testset consolidated and the `winch_ctrl` assertions removed — not failures) |
| KiteControllers.jl | 232/232 |
| SimpleKiteControllers | 120/120 |

The end-to-end `simple_reelout.jl` run reproduces the pre-refactor result
**bit-identically**: RMS d 1.24°, force mean 3467 N (CV 3.9 %), reel-out force
3348 N, power 7943 W, ring period 2.62 s / zeta 0.11, E 682.1 kJ, 175 -> 350 m,
all 8 criteria pass. That is the check that matters — it drives the winch
through the moved length loop AND through `warmup_torque`, the two places a
behaviour change would have hidden.

**Still open:**
- **Three `[sources]` path entries to drop once a WinchControllers release ships
  the merge**: `V3Kite/test/Project.toml`, `V3Kite/examples/Project.toml` and
  `SimpleKiteControllers/examples/Project.toml`, all pointing at the local
  `WinchControllers.jl` checkout. V3Kite's own `Project.toml` no longer needs one.
- `winch_adapter.jl` is duplicated in two repos. Fine while it is ~90 lines of
  glue; if a third application appears, it wants a home of its own.
- WinchControllers is still version `0.5.6`; `0.5.7` is the right bump.
