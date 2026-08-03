# System image notes (`bin/create_sys_image`)

Record of what was tried to speed up `examples/` startup with a PackageCompiler
system image, and why none of it touches the ~30 s cost inside `init()`. Read
this before re-attempting any of the approaches below; each one was tried and
measured, not just proposed.

**Resolved:** the `init()` cost is not a system image problem at all — it is
how V3Kite is sourced into `examples/Project.toml`. See
[The cause](#the-cause-how-v3kite-is-sourced-2026-08-03); (1)–(5) below are the
dead ends that led there.

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

`bin/create_sys_image` is still worth keeping for `MakieControlPlots`/`GLMakie`
load time — that part works. It just does not address the `init()` cost this
investigation was chasing.
