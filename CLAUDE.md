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
include("examples/simple_fig8.jl")     # cached: ~2x realtime, 150 s sim = ~75 s wall
```

`SHOW_PLOTS = false` before the `include` suppresses the figures at the end. It is
one-shot — the script resets it to `true` at startup, so a sweep sets it inside its
loop and a leftover `false` never silently swallows a later run's plots. `fcs` is NOT
sticky: every `include` reloads it fresh from `fc_settings.yaml` via
`fcs = FC_Settings(fc_settings(project))`, unconditionally and with no `@isdefined`
guard, so a pre-defined or hand-mutated `fcs` left in the REPL is discarded the moment
the run that was meant to use it starts — override a parameter by editing the YAML
file instead. Keep the same REPL across runs — the
first one compiles the V3 model (a few minutes, ~23 MB) into V3Kite's own cache
directory and settles the wing into `examples/cache/settled_*.bin`. Both are reused
afterwards, and that is what the ~2x realtime above assumes: with the packages
precompiled and both caches present the run is fast, the very first one is minutes
slower. A `body_damping` the settling cache has not seen pays that cost again.

## Use Kaimon

Kaimon exposes the live Julia REPL and code-intelligence tools; prefer it over a shell
subprocess or ad-hoc `grep`/`find`.

- **Executing code** — `ex` evaluates in the REPL the user is watching; `src/` edits are
  picked up automatically via Revise, so never call `Revise.revise()` yourself. Run tests
  by `include`-ing the test file into that REPL, not `Pkg.test()` in a subprocess — a fresh
  subprocess can't reuse the compiled V3 model or `examples/cache/settled_*.bin`, so a full
  run there pays the multi-minute first-run cost every time.
- **Finding code** — semantic search when exploring or when you only know what the code
  does, not its name; grep when you already have an exact symbol, call site, or string to
  match.

### Waiting for a run to finish

A reel-out run takes ~4-6 min of wall time (150 s of sim at ~1.4x realtime, plus the
~100 s the blocking re-optimizer holds it), and the `include` returns nothing until it
is over. Do not wait on it with a guessed `sleep`: `examples/simple_opt_reelout.jl`
**removes `output/last_run_done.txt` at startup and writes it as the very last thing it
does**, so its presence means "this run is over" and never "a previous run was". Block
on the file instead:

```bash
until [ -f output/last_run_done.txt ]; do sleep 5; done; cat output/last_run_done.txt
```

It carries the timestamp, the archive directory, the summary path, the pass/fail verdict
and the measured power — enough to know whether to bother reading further. Read the
run's numbers from the **archive** it names, not from `output/`: every run overwrites
`output/reelout_150m_opt.{arrow,yaml}`, and a run started while you are reading them
will swap them under you.

### Latest results per wind speed

`output/scenarios/` is a symlink into the
[SimulationResults](https://github.com/OpenSourceAWE/SimulationResults) repo, which
keeps the latest 150 m reel-out optimizer run for each wind speed: `v03`, `v06`, `v08`,
`v09`, `v10` — the number is the **mean wind speed in m/s**, not a version counter.
Measured mean reel-out power ranges from 394 W at 3 m/s to 35 kW at 10 m/s.

Each folder is a self-contained record of one run: the flight log
(`reelout_150m_opt.arrow`), the run summary (`reelout_150m_opt.yaml` — git hash and
dirty status, wind and turbulence, wall-clock time, `fig8_metrics`, and measured vs.
predicted power; read this first instead of loading the Arrow), and copies of every
settings file that fed it (`fc_settings_reelout.yaml`, `settings_reelout_150m.yaml`,
`system_reelout_150m.yaml`, `traj_opt.yaml`, `wc_settings.yaml`, `gui.yaml`), so the run
is reproducible independently of whatever `data/` currently holds. Nothing here is
overwritten by a new run, so this is the place to compare wind speeds or an older
tuning — `output/` itself is not.

### The optimizer's failed-request cache

A failing solve runs to IPOPT's iteration cap (~81 s, against ~1.5 s for one that
converges) and in `reopt_blocking` mode the simulation is frozen for all of it, so
`examples/awetrim_client.jl` records every failed request in
`output/opt_failure_cache.yaml` and does not send it again. Worth ~88 s a run on the
150 -> 380 m reel-out, whose first guess seed fails at two lengths and is silently
recovered by `reopt_retry_el_offset`.

```julia
clear_opt_failures()          # forget them all; or delete the YAML, or drop one entry
```

Set `opt_failure_cache: false` in `data/traj_opt.yaml` to bypass it — after an
AWETrim or server upgrade, when what used to fail may not any more. The key covers
everything the server sees and the tether length is EXACT, because the failures are
isolated pockets in length (180.0 m converges, 180.00027 m does not), so a request
only hits the cache when a run repeats it exactly.

Only `simple_opt_reelout.jl` writes the marker; copy its two lines (the `rm` next to
`mkpath(output_path)` in the script itself and the `open(RUN_DONE_FILE, "w")` block at
the end of `examples/reelout_results.jl`) into another example that needs to be waited
on.

`init` is called **without `cache_path`** on purpose. V3Kite's default puts the model
where its `@compile_workload` compiled against it — always a depot scratchspace now,
for a local checkout as much as an installed copy — and deserializing any *other* model
binary costs ~40 s of re-JIT per process, because the RuntimeGeneratedFunction id is part
of the type the cached SciML specializations are keyed to.

## Environments

Root `Project.toml` declares `[workspace] projects = ["examples"]`, so `examples/` keeps
its own `Project.toml` (GLMakie, V3Kite and its solver chain — deliberately *not* deps of
the library) but resolves against the **single Manifest.toml at the root**. A dependency
shared by both is therefore the same version in a script as in the package it exercises.
Needs Pkg 1.11+; older Pkg silently resolves `examples/` standalone.

Both `Project.toml` files carry comment blocks explaining *why* each compat bound and
`[sources]` pin exists (a `RuntimeGeneratedFunctions` exact pin that keeps the model cache
loadable, the V3Kite source). Read them before touching a version bound.

### Sourcing V3Kite

`examples/Project.toml` can source V3Kite from a git rev or from a **local path**; a
`path` entry is machine-specific and not committable. Both give `init()` in ~2.9 s. The
choice is about Revise on V3Kite's source, not about speed — that was true until
2026-08-05, when `init` cost ~34 s under `url`/`rev`, and the real cause turned out to be
that the run deserialized a model binary V3Kite's precompile workload had never compiled
against. Dropping `cache_path` fixed it for both sources (a PackageCompiler sysimage is
*not* the lever — it is worth ~4 s either way).

Switch it with `bin/dev` (local checkout, default `../V3Kite`) and `bin/free` (back to
`url`/`rev`); never edit it by hand. `bin/dev` comments the `url`/`rev` line out and adds
a `path` line beside it, so `bin/free` restores the pinned rev verbatim — pass
`bin/free <rev>` only to deliberately move the pin. `bin/dev --status` reports which
source is active without changing anything. A `path` entry is machine-specific, so run
`bin/free` before pushing. Either direction ends in `Pkg.resolve()` + `Pkg.instantiate()`,
which triggers a ~3 min V3Kite precompile.

`LocalPreferences.toml` (root and `examples/`) sets `[Revise] revise_structs = true`, so a
live REPL session picks up edits to `struct` definitions (`FC_Settings`,
`FigureEightSettings`, ...) without a restart. Julia 1.12+ only.

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
  the loaded table as mutable state (`reload_turn_rate_table!`). Non-passing
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
- `course_controller.jl` — `CourseControllerSettings`/`CourseController`, the
  figure-of-eight inner loop: commanded course in, `rel_steering`/`rel_depower` out
  (`calc_steering`). Owns the four-phase entry state machine (0 park, 1 dive, 2 hold,
  3 transition, 4 fig8; 5 "final" is winch-triggered from outside via `set_phase!`), the
  entry descent limiter, and the heading/course feedback blend.
- `winch_kv_table.jl` — `winch_kv(v_wind)`, wind-speed-dependent `kv` for
  WinchControllers.jl's reel-out law, interpolated from the system project's
  `winch_kv_table` file.
- `optimization.jl` — `OptSettings` and the shared machinery (`opt_grid`,
  `claim_task!`/`release_claims!`, `record_result!`/`load_results`, `rank_results`, ...)
  behind the parallel figure-eight shape sweep driven by `examples/optimize_fig8.jl`.
- `traj_opt_settings.jl` — `TrajOptSettings`, the settings of a run flown along an
  externally optimized path (`examples/simple_opt_fig8.jl`), loaded from
  `data/traj_opt.yaml`.

`examples/simple_fig8.jl` is the integration: it drives `CourseController` from
`SysState`, plus the winch mode choice and the simulation loop.

### Things that will bite

- **Two data paths.** `skc_data_path()` is this package's `data/`, always;
  `KiteUtils.get_data_path()` is whatever was last set. `examples/simple_fig8.jl`
  points it at this package's `data/` and it STAYS there for the whole run — the
  settings and `wc_settings.yaml` are here, while the arrow log goes to `output/`
  (named after the project's `log_file` setting) and `simple_fig8_plots.jl` loads
  it from there too. That needs a V3Kite whose
  `init` does not move the global path (merged for `v1.0.1`); against an
  older V3Kite the path flips to `v3_data_path()` at `init` and the log lands in
  the model's `data/` instead. The run's **system project** does not depend on
  either: `project_file()` returns
  `data/system_fig8_200m.yaml` as an ABSOLUTE path, which makes KiteUtils resolve
  `sim_settings` next to it — `data/settings_fig8_200m.yaml` here, not the identically
  named file in the model's data. A project this package does not carry is passed
  through unchanged. The geometry, polars and VSM settings never go through the
  project file at all; V3Kite loads them from `v3_data_path()` directly.
- **One state file, `data/gui.yaml`** (gitignored; `gui.yaml.default` is the template).
  Project, `sim_time`, plots and `default_turbulence` all live in its `gui:` section.
  `examples/gui_state.jl` reads it with a line regex and writes it with KiteUtils'
  `update_yaml_scalar`, and V3Kite's `set_default_turbulence(; data_path = skc_data_path())`
  writes the turbulence key into the same file — line-based on both sides, so the keys and
  the comments survive each other. `simple_fig8.jl` passes the level to `init` as
  `use_turbulence`: the model's own `data/gui.yaml` is never consulted, and is read-only
  for a Pkg-installed V3Kite anyway.
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
- **Log slot mapping** (`var_01`…`var_09`, `bearing`, `sys_state` as `CourseController`'s
  phase 0–4) is documented in `examples/simple_fig8.jl`'s docstring and relied on by
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
- Never hardcode `c1`/`c2`/`delay` — always `turn_rate_coeffs(fcs.body_damping, depower)`.
  Both arguments move them a lot. `body_damping` is the key: V3Kite's `init` decays it to a
  floor of 0.8x it, so the one value fixes both the settling transient and the damping the
  pattern is flown with.
- **Terminology: "reel-out", never "pay-out".** The tether is reeled out, the winch reels
  out, the phase is the reel-out window — in code, comments, docstrings, YAML, docs and
  diagrams alike. "pay-out"/"pays out"/"paying out" were renamed out of the repo on
  2026-08-16; do not reintroduce them.
- The **dated tuning history** (sweeps, reverted attempts, why each lever is closed) lives
  in [docs/fig8_tuning_log.md](docs/fig8_tuning_log.md). Add findings there, not in the
  settings docstrings. Its older entries predate the migration: ALL-CAPS names are today's
  `FC_Settings` fields, and the plans and `data/` files they cite are V3Kite's (its own
  header block maps this out). The sections at the end were moved out of source comments
  on 2026-08-03 and are pointed at from the code they came from.

## Current state

- Branch `main`. V3Kite is at `v1.0.2`, but sourced from a **local checkout** — see
  "Sourcing V3Kite" above; `examples/Project.toml` must be put back on `url`/`rev`
  before pushing.

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
