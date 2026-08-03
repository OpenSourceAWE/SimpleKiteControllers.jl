# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Julia package with simple flight controllers for airborne wind energy kites: a **parking
controller** (nose into the wind, kite held airborne in steady state) and a
**figure-of-eight path-following controller** (L0 attractor guidance after
[Fernandes et al., Energies 2022](https://www.mdpi.com/1996-1073/15/4/1390)).

The defining constraint: **`src/` never depends on a kite model.** It is pure geometry,
settings and metrics — no simulation, no plant. Everything that touches a kite
(`init`/`step!`, both winch modes, `span_mean_aoa`, `create_heading_pid`) lives in
`examples/` and comes from [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl)
through its public API. Keep it that way: a new helper that needs a `V3KITE` belongs in
an example, not in `src/`.

## Commands

```bash
julia --project -e 'using Pkg; Pkg.test()'    # full suite, ~7 s (pure geometry)
```

In a live REPL, `include("test/runtests.jl")` for everything, or
`include("test/test_fig8_controller.jl")` for the figure-eight / metrics / turn-rate
testsets alone (it activates nothing, so `using SimpleKiteControllers` must already work
in that environment). `bin/run_julia` starts a REPL on the project, picking up a system
image at `bin/kps-image-<version>-<branch>.so` if one exists.

The example is run by `include`, not from the shell — it activates `examples/` itself:

```julia
include("examples/simple_fig8.jl")     # 150 s sim time = many minutes of wall time
```

Override any parameter by defining `fcs` (and optionally `SHOW_PLOTS = false`) in the
REPL *before* the `include`; the script only loads `fc_settings.yaml` into `fcs` if
nothing else has. Keep the same REPL across runs — the first one compiles the V3 model
into `examples/cache/` (a few minutes, ~23 MB, gitignored).

## Environments

Root `Project.toml` declares `[workspace] projects = ["examples"]`, so `examples/` keeps
its own `Project.toml` (GLMakie, V3Kite and its solver chain — deliberately *not* deps of
the library) but resolves against the **single Manifest.toml at the root**. A dependency
shared by both is therefore the same version in a script as in the package it exercises.
Needs Pkg 1.11+; older Pkg silently resolves `examples/` standalone.

Both `Project.toml` files carry comment blocks explaining *why* each compat bound and
`[sources]` pin exists (a `RuntimeGeneratedFunctions` exact pin that keeps the model cache
loadable, the V3Kite branch pin). Read them before touching a version bound.

## Architecture

`src/SimpleKiteControllers.jl` sets the include order — `turn_rate_table.jl` **before**
`figure_eight_controller.jl`, since the guidance's feasibility helpers default `c1` to
`V3_TURN_RATE_C1` defined there.

- `parking_controller.jl` — `ParkingControllerSettings`/`ParkingController`, the NDI
  block (`linearize`), great-circle `navigate`, and the cascaded turn-rate/heading
  `calc_steering`.
- `figure_eight_controller.jl` — `FigureEightSettings`/`FigureEightController`. Builds a
  discretized lemniscate in (azimuth, elevation) degrees, finds the closest point Q and
  returns an attractor point a fixed arc ahead (`calc_attractor`), then the great-circle
  course to it (`navigate_fig8`). Also the curvature feasibility check
  (`min_turn_radius`, `path_min_radius`, `path_radius_profile`,
  `check_pattern_feasible`).
- `turn_rate_table.jl` — `turn_rate_coeffs(body_damping, depower)`, interpolating `c1`
  (log-linearly), `c2` and the steering `delay` from `data/turn_rate_coeffs.yaml`. Owns
  the loaded table as mutable state (`reload_turn_rate_table!`). Non-passing and legacy
  rows are never interpolation neighbours, and a lookup outside the identified range
  **throws** rather than extrapolating.
- `fc_settings.jl` — `FC_Settings`, every tuning parameter of a figure-eight run, loaded
  from `data/fc_settings.yaml`. Plus `winch_force_gains`, which applies the `compliance`
  scaling and returns plain numbers (the winch object it feeds belongs to the kite model),
  and `project_file`, which resolves the system project against this package's `data/`.
  `FC_Settings` holds the controller's tuning ONLY: the plant, the run length
  (`sim_time`) and the timestep (`1/sample_freq`) are in `data/settings_fig8_200m.yaml`,
  and the example takes `s.steps`/`s.dt` back from the model after `init`.
- `fig8_metrics.jl` — `fig8_metrics`/`print_fig8_metrics`, headless scoring of a flown
  log. No plotting dependency, so sweeps can run headless.

`examples/simple_fig8.jl` is the integration: it owns the four-phase entry state machine
(park → dive → hold → fig8), the entry descent limiter, the heading/course feedback
blend, the winch mode choice and the simulation loop. Per the README TODO, that logic has
not yet been lifted into a controller type.

### Things that will bite

- **Two data paths.** `skc_data_path()` is this package's `data/`, always;
  `KiteUtils.get_data_path()` is whatever was last set. `examples/simple_fig8.jl`
  points it at this package's `data/` and it STAYS there for the whole run — the
  settings, `wc_settings.yaml` and `fig8_run.arrow` are all here, and
  `simple_fig8_plots.jl` loads the log from here too. That needs a V3Kite whose
  `init` does not move the global path (branch `init-keeps-data-path`); against an
  older V3Kite the path flips to `v3_data_path()` at `init` and the log lands in
  the model's `data/` instead. The run's **system project** does not depend on
  either: `project_file()` returns
  `data/system_fig8_200m.yaml` as an ABSOLUTE path, which makes KiteUtils resolve
  `sim_settings` next to it — `data/settings_fig8_200m.yaml` here, not the identically
  named file in the model's data. A project this package does not carry is passed
  through unchanged. The geometry, polars and VSM settings never go through the
  project file at all; V3Kite loads them from `v3_data_path()` directly.
- **Bearing convention:** `0` = towards zenith, positive towards larger azimuth. `chi_set`
  is directly comparable to `SysState.heading`. `SysState.course` is **not** — it needs
  `wrap_to_pi(course + pi)`; feeding it raw is positive feedback and diverges the run.
- **`rel_steering` is fed unnegated** on this plant (measured, r = +0.998). Other kites in
  the ecosystem negate; that does not transfer.
- **The pattern must be flown low and wide.** The V3's minimum angular turn radius is
  `1/(L·c1·u_s)`, and a lemniscate's tightest curvature is on the lobe's upper shoulder,
  collapsing as the pattern is raised (`cos(elevation)` compresses the azimuth axis). A
  figure-eight near zenith is geometrically impossible at any PID tuning —
  `check_pattern_feasible` prints the margin at startup.
- **Log slot mapping** (`var_01`…`var_09`, `bearing`, `sys_state` as phase 0–4) is
  documented in `examples/simple_fig8.jl`'s docstring and relied on by
  `fig8_metrics.jl` and `simple_fig8_plots.jl`. Changing a slot means changing all three.

## Conventions

- SPDX `MPL-2.0` header on new source files (the two older parking-controller files
  predate this).
- **Three places for three kinds of writing**, and they must not swap roles: a docstring
  says what a thing is and how it works; an inline comment states a single non-obvious
  fact; `docs/fig8_tuning_log.md` carries the story — what was measured, what was tried,
  which levers are closed. A measurement or a failed experiment in a docstring belongs in
  the log instead. See "Coding conventions" below for the hard rules.
- Tunables go into a YAML file loaded through a `@with_kw` struct (`FC_Settings`,
  `FigureEightSettings`), never hardcoded in a script. Unknown YAML keys are an error;
  missing ones fall back to the struct default. The YAML header comment block acts as the
  file's docstring.
- Never hardcode `c1`/`c2`/`delay` — always `turn_rate_coeffs(body_damping, depower)`.
  Both arguments move them a lot.
- The **dated tuning history** (sweeps, reverted attempts, why each lever is closed) lives
  in [docs/fig8_tuning_log.md](docs/fig8_tuning_log.md). Add findings there, not in the
  settings docstrings. Its older entries predate the migration: ALL-CAPS names are today's
  `FC_Settings` fields, and the plans and `data/` files they cite are V3Kite's (its own
  header block maps this out). The sections at the end were moved out of source comments
  on 2026-08-03 and are pointed at from the code they came from.

## Current state

- Branch `fig8`. `examples/Project.toml` pins V3Kite to its `fix_missing` branch; switch
  to `main` once [V3Kite PR #41](https://github.com/OpenSourceAWE/V3Kite.jl/pull/41) merges.
- **`PlanMigrate.md` is outdated** — it describes the (now completed) move of this
  controller out of V3Kite, and still refers to an `examples/v3kite_support.jl` that no
  longer exists. Do not treat it as a description of the current code: the helpers it
  discusses (`create_heading_pid`, `WinchForceController`, `winch_force_torque!`,
  `span_mean_aoa`, `warmup!`) are V3Kite exports now.

## Coding conventions

- Inline comments are ONLY allowed when stating a very non-obvious fact, and
  then keep them to 1 line at most. Give every type/function a docstring ("""
  not #) instead, but not too verbose, people won't reed it if your docstring is
  too long, and explain how the code works in the docstring, but not the whole
  story behind it.
- Remove or make inline comments 1 line where you see them.
- In YAML files, consider the comments at the top of the file as docstring where you can add multiline comments.
- Everything with a docstring should be added to the docs, otherwise you get an
  error when building the docs. (This repository has no Documenter setup yet — no
  `docs/make.jl`, no `docs/Project.toml` — so nothing enforces this today.)
