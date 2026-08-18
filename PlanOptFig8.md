# Fly an externally optimized flight path

Add an example that asks the AWETrim optimizer for the power-optimal reel-out
path under the inflow conditions of the run, injects that path into the existing
L0 attractor guidance, and flies it — instead of the lemniscate
[`figure_eight_path`](src/figure_eight_controller.jl#L50) builds from `f8_a`/`f8_b`.

The REST contract itself is already solved: [examples/optimize_path.jl](examples/optimize_path.jl)
talks to the server, and [docs/TrajectoryOptimization.md](docs/TrajectoryOptimization.md)
records the interface discussion. What is missing is everything between "a
trajectory arrived" and "the kite flew it".

## What the original one-liner did not say

Five gaps, none of them in the HTTP call:

1. **The guidance cannot hold a path it did not build.** `az_path`/`el_path` are
   written only by [`_build_path`](src/figure_eight_controller.jl#L126), which
   always evaluates the lemniscate; [`set_path_center!`](src/figure_eight_controller.jl#L168)
   only moves that lemniscate. There is no way in to an arbitrary closed curve.
2. **The conditions must be derived, not typed in.** `optimize_path.jl` hardcodes
   `k_v = 0.11`, `f_min = 1000`, `f_max = 8400`, `wind_speed = 5.2`. The plant's
   actual values are `kv: 0.0408`, `f_low: 350`, `f_high: 7600`
   ([data/wc_settings.yaml](data/wc_settings.yaml)) and `v_wind: 6.0`,
   `upwind_dir: -90.0`, `profile_law: 3`, `z0: 0.0002`, `h_ref: 6.0`
   ([data/settings_reelout_150m.yaml](data/settings_reelout_150m.yaml)). A path
   optimized for a winch three times softer than the real one is not the path for
   this run.
3. **One path, two tether lengths.** The optimizer anchors the pattern to a single
   radius (`length`), and a reel-out run walks 150 m -> `reelout_l_max` = 350 m.
   The path that is optimal at 150 m is not optimal at 350 m, and its curvature
   margin only ever gets worse as the tether grows.
4. **Entry and metrics are parameterized on the lemniscate.** `CourseController`
   dives to `fcs.el_center` and `fig8_metrics` scores against `f8_a`/`f8_b`.
   Neither exists for an optimized curve; both have to be derived from the path
   that actually arrived.
5. **The server has to be up.** That is the original Step 1, and it is the
   smallest of the five.

## Decisions taken

- **Name: `examples/simple_opt_fig8.jl`.** NOT `opt_fig8.jl` — the repo would then
  carry `opt_fig8.jl` (flies one optimized path), `optimize_fig8.jl` (parallel
  shape sweep, hours) and `optimize_path.jl` (REST client demo) side by side in
  the menu, three names within one edit distance of each other for three
  unrelated things. `simple_*` is this repo's prefix for "a script that flies the
  kite" (`simple_fig8`, `simple_reelout`, `simple_auto_parking`), which is what
  this is.
- **Built on `simple_reelout.jl`, not `simple_fig8.jl`.** The optimizer's only
  supported mode is `"reelout"`, it maximizes reel-out power, and it maps
  `v_set = k_v*sqrt(force)` onto its radial force model — which is exactly the
  law `simple_reelout.jl` already flies through `WinchControllers.jl`
  (`rcs.kv`). At constant length the optimized path answers a question nobody
  asked.
- **Stage 1 flies at constant length anyway**, as the cheap validation of the
  path injection (see Stage 4). Constant-length is a test rig here, not the
  product.
- **The REST client lives in `examples/`, the path plumbing in `src/`.**
  `src/` has no HTTP/JSON3 dependency and should not grow one for a client that
  only an example uses; `examples/awetrim_client.jl` is an `include`, exactly like
  [examples/winch_adapter.jl](examples/winch_adapter.jl). The pure geometry —
  accepting, resampling and orienting a closed curve — is `src/`, where the
  existing path code and its tests are. This keeps the CLAUDE.md rule intact
  (`src/` still touches no kite model, and now no network either).
- **`optimize_path.jl` stays** as the standalone contract demo and gets its struct
  definitions taken over by `awetrim_client.jl` rather than duplicated.
- **No coupling with `optimize_fig8.jl`.** The server serves ONE session per
  process (`scripts/server/parallel.md` in the AWETrim checkout: a second `/init`
  gets 409), so the parallel shape sweep cannot give each worker its own
  optimization. Out of scope, and worth one line in the docstring so it is not
  attempted.
- **Loop direction is asserted, not assumed.** The reel-out settings fly
  `up_loops: false` ([data/fc_settings_reelout.yaml:88](data/fc_settings_reelout.yaml#L88))
  and the server's default guess is `downloops: true` — they agree today, and the
  code must fail loudly on the day they do not, because a path traversed the wrong
  way is flyable, plausible-looking and wrong.

## Step 1 — start, stop and check the server without a spare terminal

**Status: implemented 2026-08-18** (`bin/run_server`, `examples/awetrim_client.jl`).

Today [bin/run_server](bin/run_server) `exec`s AWETrim's script in the foreground
and the README says "start it in a second terminal window". Turn the local
wrapper into a small process manager; AWETrim itself is an upstream checkout and
is not touched.

```
bin/run_server                 # unchanged: foreground, Ctrl-C to stop
bin/run_server start           # detached; writes output/awetrim_server.pid
bin/run_server stop            # TERM, then KILL after a grace period
bin/run_server restart
bin/run_server status          # PID alive AND GET /health answers
bin/run_server log             # tail -f output/awetrim_server.log
```

- Detach with `setsid nohup … >> output/awetrim_server.log 2>&1 &`; the PID
  written is the **python** process, because AWETrim's script ends in `exec`.
- `status` must probe `/health`, not just the PID: a live process that is still
  importing CasADi is not a server yet.
- `stop` is idempotent, and clears a stale PID file whose process is gone.
- `--port`/`--host` pass through, and the port is recorded next to the PID so
  `stop`/`status` do not assume 8000.
- Both new files go under `output/` (already gitignored), not into the AWETrim
  checkout.

**Not systemd.** A `systemd --user` unit works and survives a reboot, but it needs
a unit file with absolute paths baked in, `loginctl enable-linger` on a headless
box, and a `daemon-reload` after every edit — three machine-specific things for a
dev tool tied to one checkout. Note it in the script's header as the option for
anyone who wants the server up at boot, and leave it at that.

**Julia side:** `ensure_server(url)` in `examples/awetrim_client.jl` probes
`/health` with a ~1 s timeout, and on a refused connection runs
`bin/run_server start` and polls until healthy or ~60 s have passed. A run should
not fail because a terminal was closed, and it should say plainly when it starts
a server that was not there.

## Step 2 — `examples/awetrim_client.jl`

**Status: implemented 2026-08-18.**

Lift the structs and transport out of `optimize_path.jl` unchanged
(`InflowConditions`, `WinchParams`, `Trajectory`, `InitParams`, `StepParams`, the
`StructTypes` declarations, `post`/`get_json`) and add:

- **`opt_init` / `opt_step` / `opt_status`, not `init`/`step`/`status`.** The
  endpoint functions had to be renamed after all: a script that flies an
  optimized path also does `using V3Kite`, whose exported `init` builds the kite
  model, and a second `init` in `Main` is not a shadow but an error ("function
  V3Kite.init must be explicitly imported to be extended"). `optimize_path.jl`
  only got away with it by never loading V3Kite.
- `ensure_server(url)`, `server_running(url)`, `stop_server(url)` — Step 1.
- `opt_trajectory(; resimulate)` — `GET /trajectory`. Worth having: it returns far more
  than the closed curve, namely the dense per-node table `t, s, azimuth,
  elevation, azimuth_dot, elevation_dot, distance_radial, speed_radial, s_dot,
  tension_tether_ground, input_steering, input_depower`, plus `spline.downloops`
  and `metrics.avg_power_W`. **That table is in RADIANS**, unlike the degrees of
  the `/init` and `/step` replies — the single most likely unit bug in this whole
  plan.
- `inflow_from_settings(set::Settings)` and `winch_from_wc(wc::WCSettings)` —
  Step 5's mapping, as functions so the example cannot drift from the YAML.

`optimize_path.jl` then becomes `include("awetrim_client.jl")` plus its demo,
and keeps its current behaviour — with `ensure_server()` in front of it, so the
demo no longer fails with a connection error when the server is down.

## Step 3 — let the guidance hold an arbitrary closed path (`src/`)

**Status: implemented 2026-08-18.** 190 tests pass, the 170 that predate this unchanged — which is what makes the `_path_geometry` extraction behaviour-preserving rather than merely intended to be.

In [src/figure_eight_controller.jl](src/figure_eight_controller.jl):

1. **Extract** the geometry half of `_build_path` into
   `_path_geometry(az, el, up_loops)` -> `(az, el, seg_len, tangent)`: drop a
   duplicated closing point, reverse the traversal if the direction at the
   azimuth extreme does not match `up_loops`, then compute `seg_len`/`tangent`.
   `_build_path` keeps only "evaluate the lemniscate, call `_path_geometry`", so
   the existing behaviour is unchanged by construction.

2. **Add** (exported):

   ```julia
   set_path!(fec, az, el; resample = 0, up_loops = fec.fes.up_loops)
   resample_path(az, el, n) -> (az, el)
   ```

   `set_path!` installs an externally supplied closed curve. `resample_path`
   redistributes `n` points equidistant in **the guidance's own metric** — the
   `cos(elevation)`-compressed flat-sky distance of
   [`_dist`](src/figure_eight_controller.jl#L185), not the geodesic one — because
   that is the metric `attractor_distance` and `search_window` are measured in.

3. **Why resampling is not optional.** The server returns up to 1000 points fitted
   from a B-spline, spaced in the path parameter `s`, not in arc length. Two
   things break on that: `calc_attractor` converts `search_window` to an index
   half-width as `n * search_window / total_len`
   ([:250](src/figure_eight_controller.jl#L250)), which is only a window in
   degrees if the spacing is uniform; and the branch disambiguation looks for
   **local** minima of the distance array, which a dense, unevenly spaced curve
   can manufacture from numerical noise. Resample to `fes.num_points` (361 today)
   and both assumptions hold again. Keep `resample = 0` as "take the points as
   given" for tests.

4. **State on re-installation.** Do NOT reset the course filter (`has_prev`,
   `fx`/`fy`) — it is what disambiguates the two branches at the crossing, and
   dropping it mid-flight loses branch lock for `course_tau` seconds. Do remap
   `last_idx` to the point of the new path nearest the previous Q, so the local
   search window stays anchored where the kite actually is.

5. **Tests** in `test/test_fig8_controller.jl` (pure geometry, no kite, still
   ~7 s): a lemniscate pushed through `set_path!` yields the same attractor as
   the built-in path within a tolerance; `resample_path` keeps the curve closed
   and makes `seg_len` uniform to within a few percent; a reversed input curve
   comes back with the requested `up_loops`; `path_min_radius` is unchanged by
   resampling.

## Step 4 — Stage 1: fly an optimized path at constant length

`examples/simple_opt_fig8.jl`, first version, built from `simple_fig8.jl`:
request one path, install it before the run, fly it, plot it. No winch coupling,
no re-optimization. This exists to prove Step 3 against the real plant for the
price of one 90 s run, before the reel-out machinery is added on top.

Acceptance: the run completes, `dmin` settles to the same order as a lemniscate
run, and the pattern plot shows the flown track on the optimizer's curve.

## Step 5 — Stage 2: the conditions come from the run, not from the script

The request is built from the same files the plant is built from.

| optimizer field | source | note |
|:---|:---|:---|
| `wind_speed` | `set.v_wind` | both are at `h_ref`/`REFERENCE_HEIGHT` = 6 m — assert it |
| `wind_direction` | `set.upwind_dir` mod 360 | both are "direction the wind comes FROM", 0 = N, clockwise; `-90` -> `270`, wind from the west |
| `profile_law`, `alpha`, `z0` | same-named keys of the settings file | the enum matches 0..3; the server also has 4..6 (fitted CUSTOM_*), unused here |
| `turbulence` | — | leave at 0: the server **accepts and echoes it but does not use it**, the optimizer is deterministic on the mean profile. Do not send the run's `use_turbulence` and imply otherwise |
| `k_v`, `f_min`, `f_max` | `wc.kv`, `wc.f_low`, `wc.f_high` | the same `WCSettings` object `WinchController` is built from |
| `length` | `s.sys_state.l_tether[1]` after settling | the tether length actually flown, not the nominal `l_tethers` |
| `input_depower` | `fcs.depower_setpoint`? | **open, see below** |
| `trajectory` (initial guess) | the current lemniscate from `fcs.f8_a/f8_b/...` | a guess the kite is known to be able to fly, and directly comparable in the pattern plot |

Two checks before anything is flown, both cheap and both failure modes that cost a
run otherwise:

- **Feasibility.** Run [`check_pattern_feasible`](src/figure_eight_controller.jl#L408)
  on the returned path at the tether length it will be flown at — the LONGEST
  one, `fcs.reelout_l_max`, not the starting one, since the margin shrinks as the
  tether grows. The optimizer knows nothing about the V3's turn-rate law, so a
  path tighter than `1/(L*c1*u_s)` is a perfectly plausible thing for it to
  return. Refuse to fly below margin 1.0 and say what would have to change.
- **Winch stiffness.** `kv = 0.0408` gives `0.0408*sqrt(7600)` = 3.6 m/s at
  maximum force. `optimize_path.jl`'s own header warns that a too-stiff winch
  (its example: 1.8 m/s at max force) makes the problem infeasible and the server
  replies **422**. This may well be the first thing that happens. Catch 422
  specifically, print the solver message, and — since the reel-out speed the
  optimizer wants is roughly a third of the wind speed — say which of `kv`,
  `f_high` or the wind is the binding one rather than showing a bare HTTP error.

Also: assert `spline.downloops == !fcs.up_loops` from `GET /trajectory` and abort
on a mismatch (Decisions above).

## Step 6 — Stage 3: reel out along the optimized path

Now on `simple_reelout.jl`. The winch, the entry state machine, the four-phase
descent and the log slots are untouched; only the path source changes. Two things
have to be derived from the path instead of read from `FC_Settings`:

- **`el_center`** — `CourseController`'s dive target and `var_04`. Use the mean
  elevation of the installed path; add it to the startup banner so it is visible
  when the dive ends somewhere unexpected.
- **`az_amplitude` / `el_height`** for [`fig8_metrics`](src/fig8_metrics.jl#L54),
  which already takes both as keyword arguments (`az_center`, `az_amplitude`,
  `el_height`) — feed them from the path's own extent, so the pattern-size
  criteria score against the curve that was flown and not against `f8_a`/`f8_b`.

The run summary gains one line worth more than the rest of it: the optimizer's
predicted `metrics.avg_power_W` next to the measured
[`reelout_power`](src/fig8_metrics.jl) of the log. That number is the whole point
of the exercise, and a large gap is the finding.

## Step 7 — Stage 4: re-optimize during reel-out

The pattern is anchored to `length`, and the run doubles it. Re-optimize on a lap
boundary (the run already counts laps via `fig8_idx_progress`), which is where a
path swap is cheapest.

- **Non-blocking.** `step(params; wait = false)` returns immediately with a step
  index; poll `status()` from the simulation loop and pick the result up from
  `GET /trajectory` when the state leaves `"solving"`. A solve is 10-20 s of wall
  time and the loop must not stall in it. While a solve runs the server keeps
  serving the previous path, and a failed solve keeps it too — so "no new path
  yet" needs no special case.
- **Blend, do not jump.** Resample old and new to the same `num_points`, align
  the start index at the azimuth extreme, then interpolate between the two point
  arrays over `path_blend_time`. Installing a new curve in a single step is a
  step in the cross-track error, and the guidance responds to it with steering.
- **Re-check feasibility on every new path** before installing it, at the current
  length. Keep flying the old path if the new one fails, and log that it did.
- **Guard rails:** a bounded number of re-optimizations per run, and the whole
  block behind an `enabled` flag, so a run can be repeated without the server.

## Settings

One new YAML file, `data/traj_opt.yaml`, loaded through a `@with_kw` struct
`TrajOptSettings` in `src/traj_opt_settings.jl` — same construction as
[`OptSettings`](src/optimization.jl) (which serves `optimize_fig8.jl` and equally
never touches a kite), with the file's header comment block as its docstring.
Fields: `base_url`, `name`, `n_points`, `resample_points`, `request_timeout_s`,
`autostart_server`, `input_depower`, `reg_weight`, `detect_simple_bounds`,
`reopt_enabled`, `reopt_every_n_laps`, `max_reopt`, `path_blend_time`,
`min_feasibility_margin`. No tunable is typed into the script — the repo
convention, and the reason `optimize_path.jl`'s hardcoded constants are gap #2.

## Open questions

- **`input_depower`.** The optimizer's default `optimization_params` are
  `["C_phi", "C_beta", "input_depower"]`, so it *optimizes* depower and returns
  the value in `optimized_parameters`, while `InitParams.input_depower` = 1.6 is
  its starting value. This repo commands `fcs.depower_setpoint = 0.27`. The
  conversion is the open item already recorded in
  [docs/TrajectoryOptimization.md](docs/TrajectoryOptimization.md) ("check how
  `rel_depower` is converted to depower line length in KitePodModels"). Until it
  is settled: send the default, and only **report** the optimized value in the
  summary rather than applying it. Applying an unconverted depower silently flies
  a different kite.
- **Azimuth sign.** Both sides agree that azimuth is in degrees with 0 downwind
  (`SysState.azimuth` is "azimuth angle in wind reference frame"; the server's
  schema says "0 pointing downwind (+x, the direction the wind blows towards)").
  The **handedness** is not established by either document. Settle it empirically
  before Stage 3, with the asymmetric guess from Stage 1: send a deliberately
  one-sided initial guess and check which way the returned path leans. A sign
  error here mirrors the pattern, which for a symmetric lemniscate is invisible
  and for an optimized (asymmetric) path is not.
- **Point count.** `InitRequest.n_points` defaults to 100 and is capped at 1000,
  while `optimize_path.jl` sends a 1000-point guess and its header says the reply
  has as many points as were sent. Determine which governs the reply length, once,
  and pin it in `data/traj_opt.yaml` — Step 3's resampling makes the answer
  harmless, but not knowing it makes the request non-reproducible.

## Validation

Cheap, run every time (~7 s, no kite):

```julia
include("test/runtests.jl")     # incl. the new set_path!/resample_path testsets
```

Expensive, and per CLAUDE.md not run as part of the implementation — for
verification, run:

```julia
include("examples/simple_opt_fig8.jl")
```

Acceptance, stage by stage:

1. Server: `bin/run_server start; bin/run_server status` reports healthy;
   `stop` leaves no process and no stale PID file; a second `start` is a no-op.
2. Stage 1: a constant-length run completes on the optimized path, with a
   settled cross-track error comparable to the lemniscate run and no branch flip
   at the crossing.
3. Stage 2: the request echoed by `/init` carries the run's own wind and winch
   values; a deliberately too-stiff `kv` produces the explained 422 message and
   not a stack trace.
4. Stage 3: a full reel-out run completes, and the summary shows measured mean
   reel-out power against the optimizer's `avg_power_W`.
5. Stage 4: a re-optimization at a lap boundary installs the new path without a
   visible step in `dmin`, and a run with the server killed mid-flight continues
   on the path it already has.

## Documentation to update at the end

`README.md` (menu listing and the two run families), `examples/menu.jl`,
`docs/control_algorithm.md` (its "External optimizer" bullet describes
`optimize_path.jl` as a client only — it becomes a flown path source), and a
dated entry in `docs/fig8_tuning_log.md` with the first measured
optimizer-vs-simulation power gap.
