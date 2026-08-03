# System image notes (`bin/create_sys_image`)

Record of what was tried to speed up `examples/` startup with a PackageCompiler
system image, and — the actual finding worth keeping — why it does not touch
the ~27 s cost inside `init()`. Read this before re-attempting any of the
approaches below; each one was tried and measured, not just proposed.

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
so sysimage presence/absence is not the differentiator, in either repo. This is
the ground truth the fix needs to reproduce.

## Where this stands

Not resolved. Removing V3Kite from the sysimage (4) did not reproduce the 2 s
figure, so a sysimage-vs-pkgimage conflict was not the (or not the whole)
story. Ruled out so far as the differentiator: `RuntimeGeneratedFunctions`
version (identical `git-tree-sha1` in both environments' manifests), stale
`.bin` caches, V3Kite's own precompile workload not running, GLMakie/
MakieControlPlots being absent from V3Kite's own environment (they are also
in its `examples/Project.toml`).

**Leading lead, not yet tested:** how V3Kite is *sourced* differs between the
two environments. `V3Kite.jl/examples/Project.toml` has
`[sources] V3Kite = {path = ".."}` — a dev'd, path-sourced package. This
repo's `examples/Project.toml` has `V3Kite = {url = "...", rev = "v1.0.1"}`,
which resolves into `~/.julia/packages/V3Kite/mnfec/` — a registry-style,
git-pinned copy. Worth testing directly: point this repo's
`examples/Project.toml` at a local V3Kite checkout via `path = ...` instead of
`url`/`rev`, reinstantiate, and re-measure `init()` — if that alone drops it to
~2 s, the fix is about how a git-rev-pinned dependency's pkgimage cache
(in)validates versus a dev'd one, which would need investigating/reporting
upstream in `V3Kite.jl` or the Pkg precompilation machinery itself, not
something to work around from this repo's `src/` or `bin/`.

Also not yet checked: whether some other package present in
`examples/Project.toml` but not in V3Kite's own (`DiscretePIDs`, the exact
`RuntimeGeneratedFunctions = "=0.5.22"` pin, or `SimpleKiteControllers`
itself) invalidates part of V3Kite's/`SymbolicAWEModels`'s pkgimage cache for
the RGF-wrapped model code when loaded alongside it.

`bin/create_sys_image` is still worth keeping for `MakieControlPlots`/`GLMakie`
load time — that part works. It just does not address the `init()` cost this
investigation was chasing.
