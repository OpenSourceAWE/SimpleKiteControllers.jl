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
6. Both Discourse drafts in `docs/` rest on the disproven premise that a git-rev
   source loses precompiled code. Retracted in place, not posted.

### Still open, upstream in V3Kite

Ship the model binary (artifact, or `*.bin.xz` in `data/`) so the workload and
every downstream run share bytes instead of each install minting its own. As it
stands, `@compile_workload` spends 30–40 s of every V3Kite precompile compiling
against a model only that depot will ever load.

Also look at docs/ScratchUsage.md and docs/sysimage_notes.md

