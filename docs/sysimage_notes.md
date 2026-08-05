# System image notes (`bin/create_sys_image`)

Record of what was tried to speed up `examples/` startup with a PackageCompiler
system image, and why none of it touches the ~30 s cost inside `init()`. Read
this before re-attempting any of the approaches below; each one was tried and
measured, not just proposed.

**Resolved:** the `init()` cost is not a system image problem, and not the
`[sources]` line either — it is whether the model `.bin` you deserialize is the
same one V3Kite's `@compile_workload` compiled against. See
[The actual cause](#the-actual-cause-the-workloads-model-binary-2026-08-05).
Everything before that section is the trail to it; the `url`/`rev`-vs-`path`
finding in [The cause](#the-cause-how-v3kite-is-sourced-2026-08-03) is real but
measures a proxy, not the variable.

## The symptom

`include("examples/simple_fig8.jl")` logs:

```text
[ Info: Loading settled geometry
[ Info: Initializing v3 model...
[ Info: Model bin name: model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin
[ Info: v3 model initialized in ~27 seconds.
```

"Loading settled geometry" is a cache hit (fast). The ~27 s is entirely inside
`SymbolicAWEModels.init!` → `reinit!` → `OrdinaryDiffEqBDF.init(prob.prob,
FBDF(); ...)`: JIT-compiling the `RuntimeGeneratedFunctions`-wrapped,
ModelingToolkit-generated ODE right-hand side for the V3 model (44 points, 95
segments). It happens on every fresh Julia process, independent of
`bin/run_julia`'s system image.

## Which system image every number below uses

**V3Kite's, not this repo's.** `bin/kps-image-1.12.so` here is a copy made by
`bin/copy_image` — `cmp` reports it byte-identical to `../V3Kite/bin/kps-image-1.12.so`
— and `bin/run_julia` picks that file up. It is built by V3Kite's
`test/create_sys_image.jl`, which lists 29 packages including the whole SciML
stack (ModelingToolkit, OrdinaryDiffEq*, NonlinearSolve,
SymbolicIndexingInterface, SymbolicAWEModels) and, through PackageCompiler's
`include_transitive_dependencies` (default `true`, so the script never names
it), also DiffEqBase, SciMLBase, ModelingToolkitBase and
RuntimeGeneratedFunctions. V3Kite itself is deliberately absent, so Revise keeps
working on its source.

This repo's `examples/create_sys_image.jl` — `create_sysimage([:MakieControlPlots])`
— has **never** been the image under test here. Do not reason about these
measurements from its package list.

## What was tried

1. **Build `bin/create_sys_image` baking in `V3Kite` + `MakieControlPlots`.**
   Loads instantly, `SimpleKiteControllers` precompiles fine, but the 27 s
   `init()` cost was unchanged — same as a plain `julia --project`.

2. **Theory: stale `.bin` caches.** `Serialization` ties a type's identity to
   its defining module's build ID, and a custom sysimage gives `V3Kite` a
   different build ID than an ordinary Pkg-precompiled load. Deleted
   `examples/cache/*.bin` (the settled-geometry and compiled-model caches) and
   regenerated them from two **fresh** processes, both started with `-J
   bin/kps-image-<major>.so`. **Did not help** — second fresh process was still
   ~27 s. This rules out a stale-cache-vs-sysimage mismatch as the cause.

3. **Checked whether V3Kite's own `PrecompileTools` workload was actually
   running.** `V3Kite` is resolved from the registry (`~/.julia/packages/V3Kite/…`,
   not a dev checkout), so its generated artifacts land in
   `~/.julia/scratchspaces/<V3Kite-uuid>/v3kite_cache/` (see
   `default_cache_path` in V3Kite's `src/stabilization.jl`), not its package
   directory. Confirmed a `model_v0.11.1_..._44pnt_95seg_....bin` there,
   written by `src/precompile.jl`'s `@compile_workload` (which uses
   `system_cabauw.yaml`) — same structural size (44 pt / 95 seg) as
   `system_fig8_200m.yaml`, since the point/segment count comes from the kite's
   geometry, not the system yaml. So a matching, successfully-precompiled cache
   already existed before both fresh-process tests in (2). **Still did not
   help.**

4. **Theory: baking `V3Kite` into the custom sysimage defeats its own
   `@compile_workload` cache**, because once `V3Kite`'s code is linked into a
   sysimage instead of loaded from its own separate pkgimage file, that
   pkgimage's cached native code for the RGF-wrapped ODE function no longer
   applies. Removed `:V3Kite` from the `create_sysimage` package list in
   `examples/create_sys_image.jl` (kept only `:MakieControlPlots`), rebuilt,
   and added a step to `bin/create_sys_image` that runs `using V3Kite,
   MakieControlPlots` under the new image so `V3Kite` precompiles — with its
   `@compile_workload` — against it. **Still did not help.**

5. **Made this repo's system image and model `.bin` byte-identical to the ones
   `V3Kite.jl` on `main` uses locally.** Same `bin/kps-image-1.12.so`, same
   `model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin`
   in `examples/cache/`. **The ~27 s `init()` persists.** This eliminates the
   sysimage and the compiled-model cache — the two artifacts that differ most
   visibly between the repos — as the differentiator, and with them the whole
   line of attack in (1)–(4). Whatever costs the 25 s is in the *environment*
   the model is loaded into, not in the image or the cache it is loaded from.

## Reference measurement

Running `examples/simple_parking.jl` **from within `V3Kite.jl`'s own checkout**,
via its own `bin/run_julia` (so with a custom system image, same as this repo's
27 s runs):

```text
[ Info: Model bin name: model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin
[ Info: v3 model initialized in 2.020781677 seconds.
```

Same checkout, plain `julia --project` (no system image):

```text
[ Info: Model bin name: model_v0.11.1_jl1.12_v3_particle_dir_dynamic_44pnt_95seg_0grp_1wng_1wch.bin
[ Info: v3 model initialized in 1.246194078 seconds.
```

Same model bin name (same 44pt/95seg structure) as this repo's ~27 s runs — so
it is not the model, confirming (3). Both the 2 s and the ~1.25 s figures come
from `V3Kite.jl`'s own checkout, with and without a system image respectively —
so sysimage presence/absence is not the differentiator, in either repo. These
are the figures a `path` source reproduces in this repo.

## The cause: how V3Kite is sourced (2026-08-03)

> Superseded on 2026-08-05 — see
> [The actual cause](#the-actual-cause-the-workloads-model-binary-2026-08-05).
> Every measurement here reproduces, but `[sources]` is a proxy for which model
> `.bin` the precompile workload compiled against. Hold the binary fixed and the
> source stops mattering.

**It is the `[sources]` entry in `examples/Project.toml`, and nothing else.**
`{url = ..., rev = "v1.0.1"}` costs ~34 s in `init()`; `{path = ...}` pointing
at a local checkout of the *same commit* costs ~2 s.

Measured with one script that does only `init()`, run in fresh processes
against the same `examples/cache/*.bin`, the local V3Kite checkout clean at the
`v1.0.1` tag (`git describe` → `v1.0.1`), so both sources carry identical code:

| `[sources] V3Kite` | with `bin/kps-image-1.12.so` | plain `julia --project` |
| ------------------ | ---------------------------- | ----------------------- |
| `url` + `rev`      | 33.6 s                       | 38.2 s                  |
| `path`             | 2.25 s                       | 1.77 s                  |

The `path` figures reproduce `V3Kite.jl`'s own 2.02 s / 1.25 s ground truth
below. The system image is worth about 4 s either way — irrelevant next to the
32 s.

What this is *not*: the workload does run under both sources. Under `url`/`rev`
a forced `Pkg.precompile()` logs V3Kite's `@compile_workload` executing its own
`init` (38.2 s, writing to
`~/.julia/scratchspaces/<V3Kite-uuid>/v3kite_cache/`) and leaves a ~141 MB
pkgimage — same size class as the `path` build. So the workload runs, compiles,
and is cached to disk; its native code for the RGF-wrapped ODE right-hand side
is simply not *used* at runtime when the package came from a git rev. It is
re-JIT'ed on every process.

(Pkgimage size splits by build config, not by source: ~141 MB when precompiled
under plain `julia`, ~37 MB under `-J bin/kps-image-1.12.so`, since the sysimage
already carries much of the code. The two configs use separate cache slots, so
switching between `bin/run_julia` and plain `julia` never invalidates the other.)

Ruled out along the way: the system image (1, 4, 5), the compiled-model `.bin`
(2, 5), `RuntimeGeneratedFunctions` version (identical `git-tree-sha1` in both
environments' manifests), V3Kite's precompile workload not running (3, and
again above), GLMakie/MakieControlPlots being absent from V3Kite's own
environment (they are also in its `examples/Project.toml`).

Not yet explained: *why* a git-rev-sourced package's precompiled RGF code goes
unused. That is a Pkg/PrecompileTools question, not something this repo's
`src/` or `bin/` can work around — the only lever here is the `[sources]` line,
and a `path` there is machine-specific and cannot be committed.

## What the 33 s actually compiles (2026-08-04)

Everything above compares *artifacts*. This compares *work*: an init-only
reduction of `simple_fig8.jl` run under `--trace-compile --trace-compile-timing`
against both sources, same system image, same `examples/cache`.

| | `url` + `rev` | `path` |
| --- | --- | --- |
| `init` | 45.7 s | 2.57 s |
| methods compiled | 309 | 74 |
| total traced compile time | 36.5 s | 3.4 s |

So the gap is not I/O, deserialization or JIT of the ODE right-hand side
itself — it is **type inference and codegen of the SciML machinery wrapped
around it**, and it accounts for essentially the whole difference. 245
signatures are compiled under `url`/`rev` and under `path` not at all. Three
groups carry it:

| Compiled only under `url`/`rev` | Count | Time |
| --- | --- | --- |
| `DiffEqBase.promote_f(::ODEFunction{true, AutoSpecialize, ModelingToolkitBase.GeneratedFunctionWrapper{...}}, ...)` | 1 | 22.5 s |
| `SymbolicIndexingInterface.var"#46#47"{ODEIntegrator{FBDF{...}}}` (observed-variable getters) | 23 | 9.2 s |
| `Serialization.deserialize` / `RuntimeGeneratedFunctions.drop_expr`, one per distinct RGF type | 100 | 0.5 s |

A single `promote_f` instance is half the run. What all three have in common:
the *method* belongs to another package (DiffEqBase, SymbolicIndexingInterface,
Serialization) while the *type* it specializes on only ever comes into
existence when V3Kite's `@compile_workload` runs `init`. Those are external
specializations — cached into V3Kite's pkgimage, not into their owners'. It is
exactly this class of cached code that a git-rev-sourced V3Kite fails to reuse,
which sharpens the open question from "its precompiled code" to "its externally
owned CodeInstances".

Untried lever this suggests: bake those specializations into the system image.
Both `create_sys_image.jl` scripts already pass a `precompile_execution_file`
that runs a model, so the mechanism is in place. Neither the package set nor the
workload is what stops it — see the measurements in the next section but one.
Every module owning the code in the table is already in the image in use (see
"Which system image every number below uses"): DiffEqBase and SciMLBase
transitively, SymbolicIndexingInterface explicitly, Serialization as an stdlib,
and ModelingToolkitBase, which owns the RGF `cache_tag`/`context_tag` — neither
V3Kite nor SymbolicAWEModels calls `RuntimeGeneratedFunctions.init` anywhere, so
those tags are MTK's.

Dead end (4)'s reasoning does not carry over to this case: it removed V3Kite to avoid
defeating V3Kite's own pkgimage, but under a `url`/`rev` source those external
CodeInstances are exactly the ones already going unused — there is nothing left
to defeat.

### Tried it: half the gap closes, and stops (2026-08-04)

Built exactly that image — `create_sysimage([:MakieControlPlots, :V3Kite])` with
the init-only fig8 script as `precompile_execution_file` — and measured the
init-only script against it, all on the `url`/`rev` source:

| system image (`url`/`rev` source) | `init` | wall | image |
| --- | --- | --- | --- |
| current `bin/kps-image-1.12.so` | 45.2 s | 66.0 s | 2.1 GB |
| V3Kite + fig8 workload | 26.7 s | 47.8 s | 2.3 GB |
| *(`path` source, current image)* | *2.6 s* | | |

The mechanism worked as predicted and only as far as predicted: `promote_f` and
all 23 observed-getter closures are **gone** from the trace — baked in, ~18 s
recovered. What surfaces underneath is the rest of the original 33 s:

```text
36405 ms  RuntimeGeneratedFunctions.generated_callfunc   (26 instances)
```

That is the RGF-wrapped ODE right-hand side itself, and it does **not** bake:
26 instances compile at runtime even though the build workload compiled the same
ones in-process. Likely because `generated_callfunc` is a generated function
whose generator reads RGF's *runtime* body cache — populated only when the model
binary is deserialized — so its code cannot be validated at image-load time.
Not proven; it is the next thing to check if this is picked up again.

**Two variables moved at once, and were not separated.** The package set gained
`:V3Kite` *and* the workload became the fig8 `init`. The baseline already carried
the SciML stack, so this experiment cannot attribute the ~18 s to either. The
next section settles which.

Not adopted. It buys 40% for a fatter image, still leaves `url`/`rev` ten times
slower than `path`, and puts V3Kite into the image — which costs Revise on
V3Kite's source, exactly what a local checkout is for. `bin/dev` remains the
lever. Worth revisiting only if the RGF bodies can be made to bake, or if the
workload-only variant above turns out to carry the ~18 s.

### Why nothing was ever baked: the statements are never traced (2026-08-04)

Not the model, not the package set. **V3Kite's image build traces its workload
against a `path`-sourced V3Kite, so the specializations come out of V3Kite's
pkgimage and are never JIT-compiled — and `--trace-compile` records only what
JITs.** `test/Project.toml` there pins `V3Kite = {path = ".."}` unconditionally,
so this is true of every image that script has ever built.

Four measurements, all against the image in use, `examples/precompile_fig8.jl` as
the workload (an `init` on `system_fig8_200m.yaml` plus five `step!`s):

| run | traced lines | `promote_f` | SII getters |
| --- | --- | --- | --- |
| `path` source, pkgimages on (= how the image is built) | 30 | 0 | 0 |
| same, `--pkgimages=no` | 293 | 1 | 51 |

And the model theory is dead. The `promote_f` statement traced from the fig8
`init` is **byte-identical** to the one traced from V3Kite's own workload
(`settle_wing` at 250 m / 70°, `system.yaml`) — same RGF `id`
`(0xc03b7c9f, 0x507d5098, 0x4a279e5d, 0xf02b9b0b)`, same everything. Tether
length, elevation and system yaml do not enter the generated body; the 44pt/95seg
structure does, and both models share it. So V3Kite's existing workload already
produces exactly the signature the fig8 run needs, and a fig8-specific workload
buys nothing on that count. Both RGF tags read
`ModelingToolkitBase.var"#_RGF_ModTag"`, confirming V3Kite appears nowhere in the
type.

Replaying that 293-line trace through PackageCompiler's own algorithm
(`PackageCompiler.jl:436-484`) in a process where **V3Kite is not loaded**: 277
statements survive, 16 drop, all 16 by `UndefVarError: V3Kite`. `promote_f` is
among the survivors. So the statement neither needs nor mentions V3Kite, and
`:V3Kite` in the package set was not what recovered the ~18 s above.

### The lever, built and measured: `init` halves, Revise kept (2026-08-05)

Feed that `--pkgimages=no` trace to PackageCompiler as a
`precompile_statements_file`. No package-list change, no V3Kite in the image, so
Revise on V3Kite's source is untouched. Two steps:

```bash
# 1. trace the statements the build cannot see for itself
julia --project=examples -J bin/kps-image-1.12.so --pkgimages=no \
      --trace-compile=fig8.log examples/precompile_fig8.jl

# 2. build V3Kite's image as usual, with that log added
cd ../V3Kite
V3KITE_PRECOMPILE_STATEMENTS=<abs path>/fig8.log \
V3KITE_SYSIMAGE_PATH=<abs path>/bin/kps-image-1.12-stmts.so \
    julia --project=test -e 'include("test/create_sys_image.jl")'
```

(The three `V3KITE_*` env vars were added to V3Kite's `test/create_sys_image.jl`
for this; all default to the previous behaviour. `V3KITE_SYSIMAGE_PATH` keeps an
experiment from clobbering the image `bin/run_julia` picks up.)

`init` on `system_fig8_200m.yaml`, `url`/`rev` source, three runs each:

| image | `init` | size |
| --- | --- | --- |
| `bin/kps-image-1.12.so` | 44.8 / 45.6 / 45.8 s | 2194594064 B |
| same + 293 statements | 22.5 / 22.2 / 22.2 s | 2195908888 B |

**Half the cost for 1.3 MB.** Better than the V3Kite-in-the-image variant above on
every axis: 22.2 s against 26.7 s, +1.3 MB against +200 MB, and Revise survives.

The trace confirms the mechanism. Under the new image `promote_f` (21.5 s) and
all 74 `SymbolicIndexingInterface` getter statements are **gone**, 274 traced
statements fall to 38, and what is left is the same wall the earlier experiment
hit:

```text
14783.7 ms  RuntimeGeneratedFunctions.generated_callfunc
10189.4 ms  RuntimeGeneratedFunctions.generated_callfunc
 4194.5 ms  RuntimeGeneratedFunctions.generated_callfunc   (26 instances total)
```

So the earlier caveat was wrong to worry about: the isolated `precompile`
returning `true` in 0.07 s did not predict the saving, but the saving is real.
The RGF bodies remain unbakeable, and they are now the whole of what is left.

Still not adopted as the default — `bin/dev` is one command and gets `init` to
2.5 s, which no image can match while the RGF bodies re-JIT. This matters when a
`path` source is not an option, and it means the statements file, not the package
list, is the knob to reach for.

`bin/create_sys_image` is still worth keeping for `MakieControlPlots`/`GLMakie`
load time — that part works. It just does not address the `init()` cost this
investigation was chasing.

## The actual cause: the workload's model binary (2026-08-05)

**Neither the system image nor `[sources]`. The variable is which
`model_….bin` V3Kite's `@compile_workload` compiled against, and whether the run
deserializes that same file.**

Two model binaries exist on this machine under the same name (same 44pt/95seg
key), built independently:

| md5 | where | who built it |
| --- | --- | --- |
| `54c07adc` | `../V3Kite/data/`, copied to `examples/cache/` by what was then `bin/copy_model` | a dev-checkout run |
| `ac8cea9a` | `~/.julia/scratchspaces/<V3Kite-uuid>/v3kite_cache/` | the workload, under an installed V3Kite |

Cross them against both sources — same system image, same script
(`examples/precompile_fig8.jl` with `cache_path` overridden), same
`settled_*.bin` in both cache dirs, only the model binary swapped:

| `cache_path` holds | `path` source | `url`/`rev` source |
| --- | --- | --- |
| `54c07adc` (dev checkout's) | **6.2 s** | 47.1 s |
| `ac8cea9a` (scratchspace) | 47.1 s | **5.7 s** |

A clean crossover. The source contributes nothing; matching the binary is
everything. The traces agree exactly — the fast cells are 47 traced statements
with no `promote_f`, no RGF `generated_callfunc` and 4 `SymbolicIndexingInterface`
lines; the slow cells are 287, with `promote_f` at 22.9 s and 74 SII getters. The
slow `url`/`rev` cell reproduces the 2026-08-04 baseline (293 lines, 44.8 s) that
this whole document was chasing.

### Why the binaries differ, and why that costs 40 s

The RGF **id is a type parameter**, and it is a hash of the generated body. The
`promote_f` signature traced against `ac8cea9a` reads
`RuntimeGeneratedFunction{…, (0xa41fe0d9, 0x823b1c4d, …)}`, against `54c07adc`
`(0xc03b7c9f, 0x507d5098, 0x4a279e5d, 0xf02b9b0b)`. Six of the smaller RGF ids
are shared; the ODE right-hand side's is not. Argnames are identical
(`__argₛᵧₘ1249670312406203976` in both), so this is body content, not gensym
noise. Everything the workload baked into V3Kite's pkgimage —
`DiffEqBase.promote_f`, the SII observed-getters, the `generated_callfunc`
instances — is keyed to a type that never comes into existence in a run holding
the other binary. All of it re-JITs.

### Why an installed V3Kite always lands on the wrong one

1. `*.bin` is in V3Kite's `.gitignore`, so a Pkg-installed V3Kite ships **no**
   model binary.
2. `default_cache_path` (`src/stabilization.jl:140`) sends an installed package
   to `DEPOT/scratchspaces/<uuid>/v3kite_cache/` and a dev checkout to its own
   `data/` — the package directory is read-only and `Pkg.gc`-able.
3. So `src/precompile.jl`'s `@compile_workload` builds and compiles against a
   private scratchspace model, while `examples/cache/` held whatever the copy
   script took out of the dev checkout. Different bytes, guaranteed.
4. With a `path` source those two are the *same file*, which is the entire
   reason `bin/dev` looks like a 15× speedup.

Confirmed end to end: the `url`/`rev` precompile logs the workload deserializing
the scratchspace binary (`v3 model initialized in 37.4 s`) and leaves its md5
unchanged, and a run pointed at that binary then costs 5.7 s.

### Done: `cache_path` dropped from `init` (2026-08-05)

`examples/simple_fig8.jl` and `examples/precompile_fig8.jl` no longer pass
`cache_path`. `init` then defaults to `default_cache_path`, which is by
construction the directory the workload compiled against — V3Kite's `data/` under
a `path` source, the depot scratchspace under `url`/`rev`.

| source | `init`, before | `init`, after |
| --- | --- | --- |
| `path` | 2.9 s | 2.9 s |
| `url` + `rev` | 44.8 s | **2.9 s** |

The `[sources]` line is no longer a performance decision; `bin/dev` / `bin/free`
now only decide whether Revise sees V3Kite's source. One-time cost on a machine
that has not run the configuration before: the `settled_*.bin` is regenerated in
the new location (both were seeded by copying here, which is equivalent — the
filename encodes every parameter). `examples/cache/` is unused and its ~30 MB was
deleted on 2026-08-05; the `.gitignore` entry stays so a stray copy cannot be
committed.

Still worth doing upstream in V3Kite: ship the model binary (artifact, or
`*.bin.xz` in `data/`, which is gitignored today) so the workload and every
downstream run share bytes instead of each install minting its own. As it stands
`@compile_workload` spends 30–40 s of every V3Kite precompile compiling against a
model that only that depot will ever load.
