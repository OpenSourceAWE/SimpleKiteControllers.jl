# Migrate simple_fig8.jl from V3Kite.jl to SimpleKiteControllers

## Where this stands

**The migration is complete on both sides, and verified.** The guidance, the
metrics, the turn-rate table and `FC_Settings` live in `src/`; the run lives in
`examples/simple_fig8.jl` and reaches the kite only through V3Kite's public API.
`examples/v3kite_support.jl` and every other local stand-in are gone. The suite
passes here and `simple_fig8.jl` flies to completion against `fix_missing` with
every workaround removed.

What is left is **not code**:

1. V3Kite's [PR #41](https://github.com/OpenSourceAWE/V3Kite.jl/pull/41)
   (branch `fix_missing`, "Add the interface pieces external controllers need")
   is **open, not merged**. `examples/Project.toml` pins V3Kite to that branch
   and cannot move to `main` until it lands.
2. The Step 3 housekeeping below is untouched.

Steps 1 and 2 are kept as a record of what shipped and of the decisions taken
along the way — several differ from what this document originally proposed.

---

## Step 1: V3Kite PR — branch `fix_missing` off `main` — DONE, awaiting merge

All eight items are implemented on `fix_missing`. What actually landed:

| Item | Landed as |
| :--- | :--- |
| 1a. Elevation in the settled-geometry cache key | `stabilization.jl` derives `el_deg` from `KiteUtils.calc_elevation` and appends `_el<tag>`; `damping_tag` generalised to `num_tag` (no underscore prefix) |
| 1b. `data_path` passthrough in `init` | Grew a **second** kwarg: `init` takes both `data_path` (where bundled data is read) and `cache_path` (where everything generated is written, defaulting to `default_cache_path(data_path)`). The split is what lets a read-only url-installed V3Kite work at all |
| 1c. `vsm_interval` on `step!` | Keyword with default `1`, forwarded to `sim_step!` |
| 1d. `drag_floor` and NaN-gated L/D | `LD_CD_MIN`, `drag_floor(sam)`, exported; `step!`'s `var_15`/`var_16` gate on it. The Matlab clients and `examples/parking.jl` were guarded in the same PR |
| 1e. `N` on `create_heading_pid` | One kwarg, default `10.0` |
| 1f. Force-mode winch | Took **this repository's shape**: a separate `WinchForceController` + `winch_force_torque!(wfc, s, set_length)`, with `WinchPosController` keeping `ff_scale` alone |
| 1g. `warmup!` and the settling→dynamics handover | `init` gained `warmup_time` / **`warmup_wfc`** (a winch controller, not the proposed `warmup_force_mode` flag — that is what keeps it consistent with 1f), plus the `init_winch_torque!(sys)` call after the brake is released |
| 1h. `span_mean_aoa` | Exported from `sim_helpers.jl` |

**The 1f defaults question was decided as recommended.** V3Kite's shipped
`data/wc_settings.yaml` keeps its position-mode tuning (`winch_ff_scale: 1.0`,
`winch_speed_ti: 2.0`) and merely *gains* the four force-mode keys, so no
existing example changes behaviour through the YAML. The relaxed position-mode
test bounds from the `fig8` branch were therefore not needed.

Note this leaves the force-mode gains defined in two places: V3Kite's
`wc_settings.yaml` (as model defaults) and `FC_Settings`'s `winch_*` fields
(what a fig8 run actually flies, via `winch_force_gains`). `simple_fig8.jl`
uses the latter. See Step 3.

### Explicitly not moved to V3Kite

These are still on V3Kite's `fig8` branch and are the controller half — they
live here now and must not go back:

- `src/fig8_controller.jl`, `src/fig8_metrics.jl`, `src/turn_rate_table.jl`,
  `src/fc_settings.jl` and their includes/exports in `src/V3Kite.jl`
- the `__init__` that loads `data/turn_rate_coeffs.yaml`, and the file itself
- `_stash_turn_rate_conditions!` and its call in `init` — a model-agnostic
  controller reaching into model init is exactly the coupling this migration
  removed. Its replacement is `set_turn_rate_conditions!`, called by the *run*
- `examples/build_turn_rate_table.jl`, `examples/fig_eight_plots.jl`,
  `examples/plot_rate_coeffs.jl`, `examples/simple_fig8*.jl`
- `test/test_fig8_controller.jl` and `docs/fig8_tuning_log.md` (both copied
  here), `PlanFig8.md`, `PlanC1C2.md`, `SmallPlan.md`

### Step 1 verification — outstanding

1. V3Kite's own `test/runtests.jl` passes on `fix_missing` (it gained cases for
   the new interface pieces).
2. `simple_parking.jl` and `flight_replay.jl` still run, and the `init_winch_torque!`
   behaviour change to their first seconds is understood, not just tolerated.
3. **Bump V3Kite's version** — still `0.1.0` and unregistered, so this repository
   has nothing to pin in `[compat]`.

---

## Step 2: SimpleKiteControllers cleanup — DONE

1. `examples/v3kite_support.jl` is deleted. `span_mean_aoa`,
   `WinchForceController`, `winch_force_torque!`, `warmup!` and
   `create_heading_pid` all come from `using V3Kite`.
2. **Deviation from the original plan:** the `compliance` scaling did *not*
   arrive here as a `WinchForceController(fcs::FC_Settings)` constructor. It is
   [`winch_force_gains`](src/fc_settings.jl), which returns the scaled gains as
   plain numbers for the caller to splat:
   `WinchForceController(; winch_force_gains(fcs)...)`. A constructor would have
   put a V3Kite type in `src/`, which is the dependency this migration exists to
   avoid.
3. `simple_fig8.jl` passes `vsm_interval = fcs.vsm_interval` (guard deleted) and
   calls `create_heading_pid`.
4. `simple_fig8_plots.jl` no longer warns about `var_15`/`var_16` spiking on
   unload — with 1d they are NaN gaps.
5. `examples/Project.toml` uses a `url`/`rev` source pinned to `fix_missing`,
   with the comment block rewritten. Its `[compat]` also carries an exact
   `RuntimeGeneratedFunctions = "=0.5.22"` pin (unrelated to this migration:
   0.5.23 breaks deserialization of the cached compiled model).
6. `simple_fig8.jl` passes `cache_path = joinpath(@__DIR__, "cache")` to `init`,
   so everything V3Kite generates lands in this repository (gitignored) rather
   than in the read-only installed V3Kite.

### Step 2 verification — all confirmed

1. `include("test/runtests.jl")` passes here — 108/108, ~7 s.
2. `simple_fig8.jl` runs to completion against `fix_missing` with every
   workaround removed, and `fig8_metrics` reproduces the numbers in
   `docs/fig8_tuning_log.md` for the same `fc_settings.yaml`. Ratios logged in
   `var_15`/`var_16` differ where they were previously `0.0` — that is 1d
   working, not a regression.
3. `simple_fig8_plots.jl` renders.

---

## Step 3: housekeeping — not started

- **Merge PR #41**, then switch `examples/Project.toml`'s V3Kite `rev` from
  `fix_missing` to `main` and delete the branch note above it.
- **Pin V3Kite in `[compat]`** once it has a version worth pinning (Step 1
  verification 3), and record the minimum in this repository's README.
- **Decide the fate of V3Kite's `fig8` branch.** It still carries the controller
  sources that now live here (`src/fig8_controller.jl`, `src/fc_settings.jl`,
  `src/turn_rate_table.jl`, `src/fig8_metrics.jl`) and its own copy of the tuning
  log, plus the three plan documents — which are still live, since the tuning log
  cites them. Keep it as an archive or delete it, but do not leave it looking
  like work in progress. If it stays, delete `docs/fig8_tuning_log.md` from it so
  there is one copy, here.
- ~~Move `docs/fig8_tuning_log.md` here.~~ **Done** (2026-08-03). It is
  `docs/fig8_tuning_log.md` in this repository, and `src/fc_settings.jl`,
  `data/fc_settings.yaml`, `examples/simple_fig8.jl` and `CLAUDE.md` point at the
  local copy. The entries themselves were left verbatim; a header block maps the
  ALL-CAPS globals onto today's `FC_Settings` fields and notes that the plans and
  `data/` files they cite are V3Kite's.
- **Resolve the duplicated force-mode gains** (see Step 1, 1f): decide whether
  a fig8 run should keep reading them from `FC_Settings` or from V3Kite's
  `WC_Settings`, and delete the losing copy or document why both exist.
- Consider deleting this document once the above is done — it describes a
  migration, not the system.
