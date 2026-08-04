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

## Step 3 - DONE -

Write a script bin/copy_model that copies the file model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin  
from ../V3Kite/data/model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin to 
examples/cache
and in addition copies the system image ../V3Kite/bin/kps-image-1.12.so to
the folder bin of this package.
Check first if both files exist, and copy them only if both exist.

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

## Step 9 Investigate problem with bin files - DONE -
This file: model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin
exists at multiple locations:
/home/ufechner/.julia/scratchspaces/4caac9c8-c726-438f-ab10-3553e918eab1/v3kite_cache/model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin
/home/ufechner/repos/SimpleKiteControllers.jl/examples/cache/model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin
/home/ufechner/repos/V3Kite/data/model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin
/home/ufechner/repos/V3Kite.jl-bak/data/model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin

But the copy_model script is copying it only to one location. Perhaps that causes problems?

### Findings (2026-08-04)

The number of copies is not the problem — copying to more locations would not help.
Only `examples/cache` is ever read by this repo: `simple_fig8.jl` passes
`cache_path = joinpath(@__DIR__, "cache")` to `init`, so V3Kite reads and writes the
model there and nowhere else. The other three are other producers' caches:
`../V3Kite/data` (V3Kite runs from its dev checkout, `default_cache_path` =
`data_path`), `~/.julia/scratchspaces/<V3Kite-uuid>/v3kite_cache` (a Pkg-INSTALLED
V3Kite, i.e. the `url`/`rev` source, or its `@compile_workload`), and
`../V3Kite.jl-bak/data` (dead).

The real cause is a **RuntimeGeneratedFunctions version split between the process
that writes the `.bin` and the one that reads it**. Both committed error logs show:

```text
ArgumentError: cannot deserialize a dropped RuntimeGeneratedFunction;
serialize it before calling drop_expr
```

- RGF **0.5.22** serializes a dropped-expr RGF keeping its type parameter
  `B = Nothing`, and every version's `deserialize` throws on that — the file is
  unloadable by anything, including 0.5.22 itself.
- RGF **0.5.24** adds a `serialize` method for that case, writing a
  `_SerializedRuntimeGeneratedFunction` proxy that carries the body. Of the eight
  RGF versions installed in the depot, only 0.5.24 has it.

The environments disagree: this repo's workspace and `../V3Kite/examples` resolve
**0.5.24** (loadable), `../V3Kite`'s ROOT project resolves **0.5.22** (unloadable).
So `../V3Kite/data/model_*.bin` — the file `bin/copy_model` takes — is a coin flip:
written by a run from V3Kite's *examples* env it is fine, written by its *root* env
(tests, precompile) it is poison, under the identical filename, since the name
encodes only version and structure.

Good and bad files are told apart without Julia — a good one contains the literal
string `_SerializedRuntimeGeneratedFunction`:

```text
0  ~/.julia/scratchspaces/.../v3kite_cache/model_....bin   <- bad
1  examples/cache/model_....bin                            <- good
1  ../V3Kite/data/model_....bin                            <- good, md5-identical
0  ../V3Kite.jl-bak/data/model_....bin                     <- bad
```

The bad scratchspace file is exactly the one named in the committed
`docs/model_..._1wch.bin.error.log`, which confirms the test. The failure is nearly
silent: `load_serialized_model!` catches it, warns, writes the `.error.log` and
rebuilds the model from source — the symptom is `init` taking minutes again, not a
crash. Current state is healthy: `examples/cache` holds a good bin.

### Actions - all done 2026-08-04 -

1. `Pkg.update("RuntimeGeneratedFunctions")` in `../V3Kite`'s root environment:
   0.5.22 ⇒ 0.5.24, removing the only producer of bad files. Its
   `Manifest-v1.12.toml` is gitignored there, so the checkout stayed clean. Nothing
   pins RGF in V3Kite, so this can drift back on a future re-resolve — action 3 is
   the standing guard.
2. `RuntimeGeneratedFunctions = "0.5.24"` added to `examples/Project.toml`
   `[compat]`, the pin CLAUDE.md already claimed existed. `Pkg.resolve()` changes
   neither project nor manifest — 0.5.24 was already resolved here.
3. `bin/copy_model` now refuses a source `.bin` lacking the
   `_SerializedRuntimeGeneratedFunction` marker, before touching the destination,
   and names the `Pkg.update` that fixes the checkout. It removes a stale
   `*.bin.error.log` next to the destination when it does copy, so a log cannot
   outlive the file it accuses. Verified against a fake checkout holding the bad
   scratchspace bin: refused, `examples/cache` unchanged.
4. Deleted the two dead bins (scratchspace, `.jl-bak`) and the stale
   `examples/cache/*.bin.error.log`. The copy kept in `docs/` is the record.

Verified afterwards by a short `simple_fig8.jl` run from a fresh REPL: the model
deserialized in 2.2 s (V3Kite's own ground truth is 2.02 s) and no `.error.log`
was written, so `load_serialized_model!` never reached its `catch`.

## Step 10 Why a git-rev V3Kite makes init 15x slower

Independent of step 9, and unchanged by it: `init` costs ~2 s with
`V3Kite = {path = ...}` and ~35 s with `{url = ..., rev = "v1.0.2"}` — same code,
same model bin, both sources measured with and without a system image
(docs/sysimage_notes.md). That document's five dead ends establish what it is NOT
(system image, model cache, RGF version, workload not running); it closes on
"not yet explained: *why* a git-rev-sourced package's precompiled code goes
unused."

Every attempt so far compared artifacts. This one captures what the 33 s is
actually spent compiling: run an init-only script under `--trace-compile` against
both sources and diff the two logs. That names the methods and the concrete
types being re-specialized — whether it is the RGF-wrapped ODE right-hand side,
and if so for which signature — instead of inferring it from which stale file
might be to blame.

### Measured (2026-08-04) — table in docs/sysimage_notes.md

45.7 s vs 2.57 s init; 309 methods / 36.5 s compiled vs 74 / 3.4 s. The gap is
compilation, not I/O or JIT of the right-hand side itself: 245 signatures are
compiled under `url`/`rev` and not at all under `path`, led by ONE instance of
`DiffEqBase.promote_f` on the MTK `GeneratedFunctionWrapper` ODEFunction at
22.5 s, then 23 SymbolicIndexingInterface observed-getter closures at 9.2 s.
All are methods owned by other packages specialized on types that only exist
once V3Kite's `@compile_workload` runs `init` — external CodeInstances cached
into V3Kite's pkgimage, which is what a git-rev source fails to reuse.

Next: those 245 signatures are a list, and PackageCompiler takes one
(`precompile_statements_file` / `precompile_execution_file`). Dead end (1) in
docs/sysimage_notes.md passed a package list, which never runs `init` and so
never sees these types. Baking them in would make the COMMITTABLE `url`/`rev`
state fast rather than routing around it with `bin/dev`.

Also look at docs/ScratchUsage.md and docs/sysimage_notes.md

