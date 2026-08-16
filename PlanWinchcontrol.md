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
nothing before pay-out is meant to begin.

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
