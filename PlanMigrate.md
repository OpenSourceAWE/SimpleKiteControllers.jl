# Migrate simple_fig8.jl from V3Kite.jl to SimpleKiteControllers

## Where this stands

The migration itself is **done** (uncommitted in this repository): the guidance,
the metrics, the turn-rate table and `FC_Settings` live in `src/`, the run lives
in `examples/simple_fig8.jl`, and the three pieces that reach into a `V3KITE`
live in `examples/v3kite_support.jl`. All of it is written against **V3Kite's
main branch**, with local reimplementations standing in for what main does not
have. `examples/Project.toml` pins V3Kite to a local *path* checkout for the
same reason.

What is left is to push the genuinely-missing pieces upstream into V3Kite main
and then delete the local stand-ins. That is two steps, and **they must land
together** — see the collision warning in Step 2.

Everything upstreamed already exists, tested, on V3Kite's `fig8` branch. That
branch also carries the controller half (`fig8_controller.jl`,
`turn_rate_table.jl`, `fig8_metrics.jl`, `fc_settings.jl`), which has moved
*here* and must not go back — see "Explicitly not moving".

---

## Step 1: V3Kite PR — branch `fix_missing` off `main`

Cherry-pick / port from `fig8`, in this order. Each item is independent, so a
reviewer can drop one without unpicking the rest.

### 1a. Elevation in the settled-geometry cache key (`src/stabilization.jl`)

**A wrong-results bug on main, unrelated to fig8, and the reason this item is
first.** The settling elevation arrives through `init_row`'s position, not
through `V3SettleConfig`, so it is absent from the cache suffix. Two runs at
different elevations share one `data/settled_*.bin`, and the second silently
gets the first one's geometry.

Fix is fig8's: derive `el_deg` from `KiteUtils.calc_elevation([x, y, z])` and
append `_el<tag>` to the suffix.

The same commit generalises `damping_tag` to numbers other than damping.
**Rename it `num_tag`, not `_num_tag`** — fig8's name breaks the repo's
no-underscore-prefix rule (`CLAUDE.md`, Conventions).

*Fallout:* every existing `settled_*.bin` cache key changes, so the first run
after this re-settles. Expected, not a regression.

### 1b. `data_path` passthrough in `init` (`src/interface.jl`)

`settle_wing` already takes `data_path` (`stabilization.jl:117`). `init` does
not: it hardwires `set_data_path(v3_data_path())` and calls `settle_wing`
without one, so the cache always lands in V3Kite's own `data/`.

Add `data_path = v3_data_path()` as an `init` kwarg and forward it. This is
**not** on the `fig8` branch — it is new work.

*Acceptance:* `examples/Project.toml` here can drop the `[sources]` path
checkout of V3Kite and use a `url`/`rev` entry. That is the whole point: Pkg
makes url-installed packages read-only, and today's first run into a read-only
V3Kite `data/` is a permission error.

### 1c. `vsm_interval` keyword on `step!` (`src/interface.jl`)

Main hardwires `vsm_interval = 1` in its `sim_step!` call. Add the keyword with
default `1` and forward it, plus fig8's docstring paragraph (raising it only
adds aero lag; it is for sweeps and speed trade-offs, not for stabilising the
explicit coupling).

*Acceptance:* the `fcs.vsm_interval == 1 || error(...)` guard at
`examples/simple_fig8.jl:133` can be deleted.

### 1d. `drag_floor` and NaN-gated L/D (`src/sim_helpers.jl`, `src/interface.jl`)

Add `LD_CD_MIN` + `drag_floor(sam)` (`drag_coeff_ref` already exists on main),
export `drag_floor`, and switch `step!`'s `var_15`/`var_16` from
`> 1e-6 ? ratio : 0.0` to `> drag_floor(s.sam) ? ratio : NaN`.

**This is a breaking change for log consumers** and needs a changelog line:

- `examples/parking.jl:117-118` computes the ratios itself, ungated — leave it
  or align it, but decide.
- `examples/simple_parking_plots.jl`, `examples/simple_auto_parking_plots.jl`
  plot the slots; NaN reads as a gap, which is the intent, but check the axis
  limits do not come out NaN.
- `examples/rest_server.jl` already maps non-finite floats to JSON `null`
  (`sanitize`, line 112) — no server change needed. But
  `examples/matlab/simple_parking_client.m:98` and
  `simple_sinus_client.m:151` assign `resp.var_15` straight into a preallocated
  double; a `null` arrives as `[]` and the assignment errors. Guard both.

### 1e. `N` (derivative-filter gain) on `create_heading_pid` (`src/sim_helpers.jl`)

One kwarg, default `10.0`, forwarded to `DiscretePID`, plus fig8's docstring
paragraph on the noise-gain trade.

*Acceptance:* `create_fig8_pid` disappears from `examples/v3kite_support.jl`.
It exists only because two exported functions of one name cannot both be
`using`-ed.

### 1f. Force-mode winch (`src/interface.jl`, `src/wc_settings.jl`)

Upstream `winch_force_torque!` and its four settings (`winch_force_tau`,
`winch_len_kp`, `winch_damp`, `winch_force_min`), plus `winch_ff_scale` on the
position controller.

**Take this repository's shape, not fig8's.** fig8 hangs the force-mode state
(`f_lpf`, `force_tau`, `len_kp`, `damp`, `force_min`) off `WinchPosController`,
so a struct named "position controller" carries five fields no position
controller reads. `examples/v3kite_support.jl` already has the cleaner split: a
separate `WinchForceController` and `winch_force_torque!(wfc, s, set_length)`.
Port that, keeping `WinchPosController` to `ff_scale` alone.

**Decision required — shipped defaults.** Adding the *fields* with neutral
defaults (`winch_ff_scale = 1.0`) is safe. fig8 also retunes V3Kite's shipped
`data/wc_settings.yaml` to `winch_ff_scale = 0.7` / `winch_speed_ti = 20`,
which changes the winch behaviour of `simple_parking.jl`, `flight_replay.jl`
and `rest_server.jl` for everyone. Recommendation: upstream the code with
`1.0`/`2.0` defaults, and keep the compliant tuning in a V3Kite wc-settings
file selected per project. Note `simple_fig8.jl` reads
`WC_Settings(wc_settings(fcs.project))` from V3Kite's data directory, so
wherever the tuned values land, that lookup has to keep resolving.

If the shipped YAML *is* retuned, fig8's `test/test-interface.jl` changes come
along with it — including the relaxed position-mode bounds
(`abs(reel_out_speed) < 2.0`, `abs(l - l0) < 0.5`), which only make sense under
`ff_scale = 0.7`.

### 1g. `warmup!` and the settling→dynamics handover (`src/interface.jl`)

Upstream `warmup!`, `init`'s `warmup_time` / `warmup_force_mode` kwargs
(default `0.0` = current behaviour), and the `init_winch_torque!(sys)` call
after the brake is released.

**`init_winch_torque!` is a behaviour change for every existing caller** — it
removes a torque step at t=0 that `simple_parking.jl`, `flight_replay.jl` and
`rest_server.jl` currently integrate through. It is the right fix, but call it
out in the PR description and re-run those three.

Adapt `warmup!`'s signature to whatever 1f decides (`wfc` argument vs
`force_mode` flag) so the two are consistent.

### 1h. `span_mean_aoa` (`src/sim_helpers.jl`)

Straight lift from `examples/v3kite_support.jl:42`, plus the export. Pure
addition, no behaviour change.

### Explicitly not moving to V3Kite

These are on the `fig8` branch and stay off `fix_missing` — they are the
controller half, and they now live here:

- `src/fig8_controller.jl`, `src/fig8_metrics.jl`, `src/turn_rate_table.jl`,
  `src/fc_settings.jl` and their includes/exports in `src/V3Kite.jl`
- the `__init__` that loads `data/turn_rate_coeffs.yaml`, and the file itself
- `_stash_turn_rate_conditions!` and its call in `init` (`interface.jl:553`) —
  a model-agnostic controller reaching into model init is exactly the coupling
  this migration removes; it is already gone from this repository
- `examples/build_turn_rate_table.jl`, `examples/fig_eight_plots.jl`,
  `examples/plot_rate_coeffs.jl`, `examples/simple_fig8*.jl`
- `test/test_fig8_controller.jl` (ported here), `docs/fig8_tuning_log.md`,
  `PlanFig8.md`, `PlanC1C2.md`, `SmallPlan.md`

### Step 1 verification

1. `include("test/runtests.jl")` in V3Kite passes (via the kaimon `ex` tool).
2. `simple_parking.jl` and `flight_replay.jl` still run, and the 1g/1f
   behaviour changes to their first seconds are understood, not just tolerated.
3. Bump V3Kite's version (it is `0.1.0` and unregistered) so this repository has
   something to pin.

---

## Step 2: SimpleKiteControllers cleanup — same PR cycle, immediately after

**Do not land Step 1 alone.** The moment `warmup!`, `winch_force_torque!` and
`span_mean_aoa` are exported from V3Kite, `using V3Kite` plus
`include("v3kite_support.jl")` defines each name twice and the run breaks. The
support file's own docstring records this: `create_fig8_pid` is renamed
precisely because two exported functions of one name cannot both be `using`-ed.

1. Delete from `examples/v3kite_support.jl`: `span_mean_aoa`,
   `WinchForceController`, `winch_force_torque!`, `warmup!`, `create_fig8_pid`.
   That empties the file — delete it, drop the `include` and the explanatory
   comment block at `examples/simple_fig8.jl:115-118`, and move
   `WinchForceController(fcs::FC_Settings)` (the `compliance` scaling
   constructor, `v3kite_support.jl:94`) into `src/fc_settings.jl`, which is
   where an `FC_Settings`-derived object belongs and needs no kite model.
2. `examples/simple_fig8.jl`: delete the `vsm_interval == 1` guard (line 133)
   and pass `vsm_interval = fcs.vsm_interval` to `step!`; rename
   `create_fig8_pid` → `create_heading_pid` at line 235.
3. `examples/simple_fig8_plots.jl`: drop the docstring note about `var_15`/
   `var_16` spiking on unload — with 1d they are NaN gaps.
4. `examples/Project.toml`: replace the `[sources]` path entry with a
   `url`/`rev` entry (enabled by 1b) and rewrite the comment block above it,
   which currently explains both the writable-checkout workaround and the
   "everything main does not have lives in this repository" split. Both are then
   false.
5. Pass V3Kite's `data_path` explicitly at `init` if the run should stop writing
   settled caches into the V3Kite checkout.

### Step 2 verification

The real acceptance test is reproduction, not compilation:

1. `include("test/runtests.jl")` here passes.
2. `simple_fig8.jl` runs to completion against `fix_missing` with every
   workaround removed, and `fig8_metrics` reproduces the numbers in V3Kite's
   `docs/fig8_tuning_log.md` for the same `fc_settings.yaml`. Ratios logged in
   `var_15`/`var_16` will differ where they were previously `0.0` — that is 1d
   working, not a regression.
3. `simple_fig8_plots.jl` renders.

---

## Step 3: housekeeping

- Decide the fate of V3Kite's `fig8` branch. After `fix_missing` merges it holds
  only the controller sources that now live here, plus the tuning log and the
  three plan documents. Keep it as an archive of the tuning history or delete it
  — but do not leave it looking like live work.
- Move `docs/fig8_tuning_log.md` here if it is still the reference for
  `fc_settings.yaml`; it documents settings that no longer exist in V3Kite.
- Pin the V3Kite version in `examples/Project.toml` `[compat]`, and record in
  this repository's README which V3Kite release is the minimum.
