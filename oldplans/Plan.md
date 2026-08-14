# Plan next steps

## Step 1 - DONE -

The log files in arrow format must be written into the output folder. Create an output folder relative to the repo root if it does not exist. Add this folder to .gitignore

As file name for the log files the field log_file defined in settings_xxx.yaml shall be used.

## Step 2 - DONE -

Configure Revise such that it revises structs.

Enabling struct revision

On Julia 1.12+, Revise can automatically revise struct definitions in a running session. This feature requires scanning the global method table and type hierarchy at startup, which can be slow so it's disabled by default. If you would like to enable it you can set the revise_structs preference to true via Preferences.jl.

Add the following to the LocalPreferences.toml file in your active project:

[Revise]
revise_structs = true

## Step 3 - DONE, then partly REVERTED -

Write a script bin/copy_model that copies the file model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin  
from ../V3Kite/data/model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin to 
examples/cache
and in addition copies the system image ../V3Kite/bin/kps-image-1.12.so to
the folder bin of this package.
Check first if both files exist, and copy them only if both exist.

Copying the model turned out to be the *cause* of step 10's slow `init`, not a
speed-up: a copy in `examples/cache` is a different binary from the one V3Kite's
precompile workload compiled against. The model half is gone and the script,
copying the system image only, is now `bin/copy_image`.

## Step 4 - DONE -

- Add a script menu.jl to the examples folders that allows to run simple_fig8.jl and 
simple_fig8_plots.jl
- Add a function menu() to run_julia

## Step 5 - DONE -

In KiteControllers there is the first menu entry select_project. Can you add this to SimpleKiteControllers? Currently, there are three projects (system.yaml files) in the data folder.

## Step 6 Refactoring - DONE -

Never use @isdefined in example scripts for model parameters that are set in the script.
Reason: If I manually include an example script, it shall always read the latest changes
from the yaml files.

## Step 7 Add menu entry select_sim_time - DONE -

Add a menu entry select_sim_time. It should allow the options:

- default
- a specific value in seconds

The specific value should be entered numerically. The option default means to use
the project specific default time.

Store this value in menu_state.yaml

## Step 8 Add menu entry select_plots - DONE -

Add a menu entry select_plots, that displays a checkbox menu with the following check boxes:

- pattern
- time series
- aerodynamics

Store these values in menu_state.yaml

simple_fi8_plots.jl shall then show only the selected plots.

## Step 9 Investigate problem with bin files - DONE 2026-08-04 -

`model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin`
existed at four locations at once; the copy script (then `bin/copy_model`, now
`bin/copy_image`) copied it to one of them.

Not a multiplicity problem — the four are four independent producers' caches. The
real fault was a **RuntimeGeneratedFunctions version split between the process
writing the `.bin` and the one reading it**: RGF 0.5.22 serializes a dropped-expr
RGF into a file no version can read back (`ArgumentError: cannot deserialize a
dropped RuntimeGeneratedFunction`), 0.5.24 writes a
`_SerializedRuntimeGeneratedFunction` proxy that loads. `../V3Kite`'s root project
resolved 0.5.22, its `examples/` 0.5.24 — so which of the two wrote
`../V3Kite/data/model_*.bin` decided whether the file was loadable, under an
identical filename. The failure is near-silent: `load_serialized_model!` catches
it, writes a `.bin.error.log` and rebuilds from source, so the symptom is a slow
`init`, not a crash.

Fixed by updating RGF to 0.5.24 in `../V3Kite`'s root env, pinning
`RuntimeGeneratedFunctions = "0.5.24"` in `examples/Project.toml`, and deleting the
dead binaries. A good file can be told from a bad one without Julia: it contains
the literal string `_SerializedRuntimeGeneratedFunction`. The record of the failure
is `docs/model_..._1wch.bin.error.log`.

Superseded in part by step 10: the script no longer copies a model at all, so the
marker check it grew here is gone with it.

## Step 10 Why init was 15x slower from a git-rev V3Kite - DONE 2026-08-05 -

`init` cost ~2 s with `V3Kite = {path = ...}` and ~45 s with
`{url = ..., rev = "v1.0.2"}` — same code, same system image.

**The answer: `[sources]` was never the variable.** It was *which* `model_….bin`
the run deserializes, and whether V3Kite's `@compile_workload` compiled against
that same file. The RGF `id` is a hash of the generated body and a *type
parameter*, so two independently built binaries of the same model produce
different types — and every specialization the workload cached into V3Kite's
pkgimage (`DiffEqBase.promote_f` at 22.5 s, 23 `SymbolicIndexingInterface`
observed-getters at 9.2 s, the `generated_callfunc` instances) is keyed to a type
that never comes into existence in the other run. All of it re-JITs.

Swapping only the binary crosses over cleanly:

| `cache_path` holds | `path` source | `url`/`rev` source |
| --- | --- | --- |
| the dev checkout's bin | **6.2 s** | 47.1 s |
| the scratchspace bin | 47.1 s | **5.7 s** |

The `path` source only ever looked like a 15x speed-up because under it
`examples/cache`'s copy and the workload's own file were the same bytes.

**Fix: `init` is called without `cache_path`** (`simple_fig8.jl`,
`precompile_fig8.jl`), so it defaults to `default_cache_path` — V3Kite's `data/`
under a `path` source, the depot scratchspace under `url`/`rev`, in both cases the
directory the workload compiled against. `init` is now 2.9 s on both sources, and
`bin/dev`/`bin/free` decide only whether Revise sees V3Kite's source.

### Dead ends, all measured and all closed

Do not re-attempt these; `docs/sysimage_notes.md` has the numbers.

1. System image contents — baking `V3Kite` in, leaving it out, making this repo's
   image byte-identical to V3Kite's. Worth ~4 s either way, never the 40 s.
2. Stale `.bin` caches regenerated from fresh processes under the sysimage.
3. V3Kite's `@compile_workload` not running — it runs under both sources.
4. `create_sysimage([:MakieControlPlots, :V3Kite])` + fig8 workload: 45.2 s →
   26.7 s, but only by baking what the correct binary gives for free, and it puts
   V3Kite in the image, which costs Revise on its source.
5. Feeding a `--pkgimages=no` trace as `precompile_statements_file`: 44.8 s →
   22.2 s for +1.3 MB, Revise kept. Better than (4) on every axis and still
   pointless next to using the right binary.

### Still open, upstream in V3Kite

Ship the model binary (artifact, or `*.bin.xz` in `data/`) so the workload and
every downstream run share bytes instead of each install minting its own. As it
stands, `@compile_workload` spends 30–40 s of every V3Kite precompile compiling
against a model only that depot will ever load.

Also look at docs/ScratchUsage.md and docs/sysimage_notes.md

## Step 11 Local system image without a V3Kite one - DONE 2026-08-05 -

Goal: bake enough of V3Kite's dependency chain into `examples/create_sys_image.jl`'s
own image that a copy of V3Kite's (installation-simplifying, but 2.3 GB and too
large for Windows) system image is no longer needed.

Baseline, `using V3Kite, MakieControlPlots`:

| image | load time | size |
| --- | --- | --- |
| none | 15.2 s | — |
| V3Kite's own (`bin/copy_image`) | 1.8 s | 2.19 GB |

**The suggested 8-package list (StaticArrays, Rotations, NonlinearSolve, the
three OrdinaryDiffEq\* packages, SteadyStateDiffEq, SymbolicIndexingInterface)
was not enough on its own:** 12.3 s at 1.57 GB — barely better than no image at
all. `--trace-compile` showed almost nothing left to JIT, so the remaining cost
is *loading*, not compiling. Isolated it to two packages neither the solver
list nor `include_transitive_dependencies` pulls in, because both sit *above*
the solvers in V3Kite's own dependency tree:

| load (fresh process, solver-only image) | time |
| --- | --- |
| `ModelingToolkit` alone | 5.8 s |
| `SymbolicAWEModels` alone (pulls in ModelingToolkit) | 10.9 s |
| `V3Kite` (pulls in SymbolicAWEModels) | 13.1 s |

Both cost roughly comparable amounts, and both are also most of what makes
V3Kite's own image large — there is no package set that is both small and fast.
`examples/create_sys_image.jl` now bakes in the solver list plus
`ModelingToolkit` and `SymbolicAWEModels` on every platform except Windows,
where only `ModelingToolkit` is added (smaller image, part of the speed gap
closed, exact numbers not measured — no Windows machine available here).
`examples/Project.toml` carries both as direct deps for the same reason as the
solver packages: `PackageCompiler.create_sysimage` requires it.

**Rebuilt and confirmed:** the full (non-Windows) image built locally via
`bin/create_sys_image` is **2.29 GB, 1.58 s** load time — matches V3Kite's own
image on speed, at essentially the same size, without ever copying a system
image out of a V3Kite checkout. `bin/copy_image` is no longer needed on this
platform; it stays for Windows use until that path is measured separately.

## Step 12 improve system image - DONE 2026-08-05 -

`save_log`/`load_log` (KiteUtils) cost **15.3 s** under the step-11 image, which
didn't bake in `KiteUtils` or exercise its Arrow write/read path. Added a
`save_log`/`load_log` round trip to `examples/create_sys_image.jl`'s
`precompile_execution_file` workload and `:KiteUtils` to the baked-in package
list (already a direct dep of `examples/Project.toml`, so no new entry needed
there).

Rebuilt and measured: `save_log`+`load_log` now **2.6 s**, and the
`using V3Kite, MakieControlPlots` load time (1.55 s) and image size (2.31 GB)
are unchanged from step 11 within noise.

# Step 13 Make turbulence work
- Add a call to set_default_turbulence() as in V3Kite to the menu - DONE -

# Step 14 Write v3_segments.csv table - DONE 2026-08-12 -
Add a script export_v3_segments.jl.

It shall create a csv file in the output folder with the columns:

segment point1 point2 segment_type

segment is a number from 1 to n
point1 and point2 are the numbers of the point that the segment connects
segment_type is tether or bridle

**A third type was missing: the wing structure.** `tether`/`bridle` covers only
49 of the V3's 95 segments; the other 46 (leading-edge tubes, struts,
trailing-edge wires, diagonal springs) are neither. The exported types are
therefore `wing` (46), `bridle` (43) and `tether` (6). Finer splits exist but
were not taken: the four wing sub-kinds, and `power_line`/`steering_line` within
the bridle (the KCU tapes 87–89 and the mains 63/64), which is what the
deprecated `SegmentType` enum named.

`Segment` carries no type — `SegmentType` is no longer a constructor parameter —
so `examples/export_v3_segments.jl` derives it: in a tether's `segment_idxs` →
`tether`; else both endpoints `is_wing_node` → `wing`; else `bridle`. It builds
the structure with `load_sys_struct_from_yaml` (V3Kite's committed
`data/struc_geometry.yaml`, `AeroNone` so no VSM geometry is loaded), not with
`init`: seconds instead of minutes, and the loader still runs
`expand_auto_tethers!`, without which the file would have 89 segments/39 points
and no tether rows at all instead of 95/44. The trimmed geometry `init` actually
flies differs only in positions and rest lengths, not in topology.

# Step 15 Live 3D view of a figure-eight run - IMPLEMENTED 2026-08-13, full run not yet flown -

Integrate SimpleKiteControllers.jl with KiteViewers.jl: `examples/simple_fig8_live.jl`
shows the kite, bridle and tether **live**, updated from the simulation loop — not
replayed from a log afterwards. `KiteViewers.jl/examples/park_v3.jl` was the
reference for the API, not for the structure: it drives `update_segments!` from a
finished `.arrow` log, this script drives it from `s.sys_state` right after `step!`.

`examples/simple_fig8_live.jl` started as a **byte-identical copy** of
`simple_fig8.jl` and the fork was kept minimal: outside the docstring, the diff is
the three `VIEWER_*` parameters, the selective KiteViewers import, an eight-line
viewer set-up before the loop, a twelve-line update block inside it, `for _` →
`for i`, and a final frame after it. Nothing else. The fork is the cost of this
step — the alternative, a one-shot `SHOW_VIEWER` global in `simple_fig8.jl` in the
style of `SHOW_PLOTS`, avoids 470 duplicated lines but mixes an interactive window
into the script that sweeps run headless.

## What was built

- `examples/v3_segments.jl` — the segment classification, lifted out of
  `export_v3_segments.jl` so the exported CSV and the drawn topology come from one
  place. `segment_types(sys_struct)` and `segment_matrix(sys_struct, type_code)`;
  the caller passes the integer code per type name, so the file needs neither a
  viewer nor GLMakie. `export_v3_segments.jl` now includes it.
- `examples/simple_fig8_live.jl` — the live run, added to `examples/menu.jl`
  (`pagesize` 8 → 10, the list no longer fitted).
- Defaults: `VIEWER_INTERVAL = 5`, `VIEWER_TIME_LAPSE = 1.0`, `VIEWER_SCALE = 0.08`,
  `VIEWER_KITE_SCALE = 4.0`. The kite scale is measured, not copied: at
  `park_v3.jl`'s 2.0 the V3's 5 m wing on a 200 m tether is a few pixels, 10 is
  too coarse.
- Pacing, added after the first runs came out at 1x to 5x realtime depending on
  how hard the step was: `wait_until` holds every drawn frame to
  `VIEWER_INTERVAL * dt / VIEWER_TIME_LAPSE` seconds of wall time, with the
  deadline reset from the clock after each wait rather than accumulated — a slow
  frame is never repaid by a fast burst, which is what makes catch-up jerky.
  Measured on a replay: 1.00x at lapse 1, 2.00x at lapse 2, and 0.62x when the
  per-frame work exceeds the budget, i.e. it caps but never rushes. The waiting is
  the whole cost — unpaced, the same frames take 0.1 ms each, since
  `update_segments!` only writes observables and GLMakie renders them on its own.
  Consequence: the `Performance: … x realtime` line of a live run reports the
  pacing, not the solver.

## Verified, and what is not

Verified against a real log (`output/fig8_200m.arrow`, 9001 rows) driven through
exactly the calls the loop makes: `segment_matrix` off the loaded structure gives
95 × 3 over 44 points, 6 tether / 43 bridle / 46 wing — step 14's counts;
`Viewer3D(project_set, false)` + `KiteViewers.init` + 451 frames of
`update_segments!`/`update_status_text!` render the kite, tether and status text
correctly, and `Z[1]` reads 93.96 m at the kite, confirming point 1 is the kite end.

**Not yet run: a full live simulation.** The twelve lines inside the loop and the
interaction between GLMakie's render task and V3Kite's stepping (whether `yield()`
alone keeps the window responsive under `step!`) are still untested.

## Prerequisite: KiteViewers 0.6.0 — resolved, but only in the live manifest

The point/segment API this step needs — `load_segments`, `init(kv, segments)`,
`update_segments!`, `update_status_text!`, and the `SegmentType` values
`TETHER`/`BRIDLE`/`WING` — arrived with KiteViewers 0.6.0 ("Add support for the
V3Kite model") and is in **no registered version**: the newest registered tag is
`v0.5.2`, which exports none of it. The dependency was added on 2026-08-13 and
`Manifest-v1.12.toml` now holds `version = "0.6.0"` from
`repo-url = ".../KiteViewers.jl.git"`, `repo-rev = "main"`; that copy has the API.
Version bounds are not a problem — 0.6.0 accepts the GLMakie `0.13.13` and
KiteUtils `0.11.13` the manifest already holds.

Two things were needed to keep it, and both are DONE:

1. The git source lived in the manifest alone, so a `Pkg.resolve()`/`Pkg.update()`
   could have fallen back to the registered 0.5.2 and broken the script with a
   `MethodError` on `init`. `examples/Project.toml` now carries
   `KiteViewers = {url = ".../KiteViewers.jl.git", rev = "main"}` in `[sources]` and
   `KiteViewers = "0.6"` in `[compat]`, which resolve confirmed keeps 0.6.0.
2. `Manifest-v1.12.toml.default`, which `bin/install` copies over the working
   manifest, still recorded 0.5.2 — a fresh clone would have installed the version
   without the API. Refreshed from the resolved manifest.

Still open: `rev = "main"` is a moving target, unlike the `rev = "v1.1.1"` V3Kite is
pinned to. Move it to a tag as soon as KiteViewers 0.6.0 is released.

## What the integration needs

- **`init` collides.** V3Kite exports `init` (the model) and KiteViewers exports
  `init` (the topology set-up), so a bare `using KiteViewers` beside `using V3Kite`
  makes the unqualified `init` at the head of the script ambiguous. Import
  selectively — `using KiteViewers: Viewer3D, update_segments!, update_status_text!,
  clear_viewer, stop` — and call `KiteViewers.init(viewer, segments)` qualified.
  Nothing else clashes: both packages re-export the same KiteUtils bindings
  (`Settings`, `SysState`, `save_log`, `set_data_path`, …), which is not a conflict.
- **Construct the viewer with the run's own settings:** `Viewer3D(project_set, false)`,
  not `Viewer3D(false)` as in `park_v3.jl`. The zero-argument constructor calls
  `se()`, which loads `system.yaml` from the current data path — and this package's
  `data/` deliberately has no `system.yaml`, only `system_fig8_{150,200,300}m.yaml`.
  `show_kite = false` because the wing is drawn from the segment topology, not from
  the `kite.obj` mesh.
- **Build the topology in memory, do not read the CSV.** `init(viewer, segments)`
  wants a plain `n × 3` `(point1, point2, segment_type)` integer matrix;
  `load_segments` is only a CSV reader for it. The live script already holds the
  built structure as `s.sys`, so it can classify the 95 segments directly with the
  rule step 14 established (in a tether's `segment_idxs` → `tether`; else both
  endpoints `is_wing_node` → `wing`; else `bridle`) and never depend on
  `output/v3_segments.csv` being present or current. That means lifting
  `segment_type` out of `export_v3_segments.jl` into a small shared
  `examples/v3_segments.jl` that both include, so the exported table and the drawn
  one cannot disagree.
- **Positions line up already.** `step!` calls `update_sys_state!`, so after it
  `s.sys_state.X/Y/Z` hold the current point positions, in `s.sys.points` order;
  `update_segments!` reads only slots `1:n_points` (44) and ignores the extra slots
  (VSM panel corners, wing/body origins). `height = s.sys_state.Z[1]` is the kite
  side: the V3's single tether runs point 1 → 39 and **39 is the winch point**.
- **Update every `VIEWER_INTERVAL` steps** (default 5, a constant beside
  `AERO_MODE` in the USER PARAMETERS block) with `update_segments!(viewer,
  s.sys_state; scale, kite_scale)` and `update_status_text!`. `scale = 0.08`,
  `kite_scale = 2.0` are `park_v3.jl`'s values for a comparable tether length and
  are the starting point, not a measured choice.
- **Yield to GLMakie.** The render loop is a task, and a `step!` loop that never
  yields starves it into a frozen window; a `yield()` (or a very short `sleep`) at
  each viewer update is what keeps the window drawing and its buttons alive.
  No `wait_until` pacing: the run is far slower than realtime, so there is nothing
  to slow down to.
- **Let STOP end the run:** break the loop on `viewer.stop`, and treat that break
  like the overspeed break — the log is still saved and scored. `park_v3.jl`'s
  `replaying[]` guard around the PLAY button is a replay-only concern (a second
  replay starting on top of the first) and has no counterpart here.
- The tail of the script is unchanged: the log is saved, `print_fig8_metrics` runs,
  and `simple_fig8_plots.jl` still comes up unless `SHOW_PLOTS = false`. Whether
  the viewer window is closed at the end or left open for inspection is a choice —
  leaving it open and letting the next run reuse it is friendlier in a REPL.

Finally, add the script to `examples/menu.jl` beside `simple_fig8.jl`.

