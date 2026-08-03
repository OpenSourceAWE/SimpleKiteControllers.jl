# System image notes (`bin/create_sys_image`)

Record of what was tried to speed up `examples/` startup with a PackageCompiler
system image, and — the actual finding worth keeping — why it does not touch
the ~27 s cost inside `init()`. Read this before re-attempting any of the
approaches below; each one was tried and measured, not just proposed.

## The symptom

`include("examples/simple_fig8.jl")` logs:

```
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

## Where this stands

Not resolved. The reported difference — fast when V3Kite is used directly
(from within its own checkout/environment), slow when loaded as a dependency
of `examples/` — was not reproduced by (4), so removing V3Kite from the
sysimage was necessary but not sufficient. Ruled out so far as the
differentiator: `RuntimeGeneratedFunctions` version (identical
`git-tree-sha1` in both environments' manifests), stale `.bin` caches,
V3Kite's own precompile workload not running.

Not yet checked: whether some other package present in `examples/Project.toml`
(`GLMakie`, `DiscretePIDs`, `RuntimeGeneratedFunctions`'s exact `=0.5.22` pin,
or `SimpleKiteControllers` itself) invalidates part of V3Kite's or
`SymbolicAWEModels`'s pkgimage cache for the RGF-wrapped model code when
loaded alongside it — Julia's precompile invalidation is transitive and can be
triggered by an unrelated sibling dependency. That would explain "works alone,
breaks as a dependency" independently of the sysimage question, and would need
investigating in `V3Kite.jl`/`SymbolicAWEModels.jl` directly (`Base.watch_...`/
`SnoopCompile`-style invalidation tracing, not something fixable from this
repo's `src/` or `bin/`).

`bin/create_sys_image` is still worth keeping for `MakieControlPlots`/`GLMakie`
load time — that part works. It just does not address the `init()` cost this
investigation was chasing.
