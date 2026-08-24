# Why `examples/reel_out_v3.jl` (V3Kite) is slow to `init()` on a fresh session

## Symptom

`V3Kite/examples/reel_out_v3.jl` took ~75 s to `init()` a fresh Julia session,
while `V3Kite/examples/simple_parking.jl` took ~3 s, even after both scripts were
made to pass identical `body_sim_damping`, `damping_per_stiffness`,
`remake_model = false`, and the same `system_reelout.yaml` project. Re-running
either script in the *same* (already warm) session was fast.

## Ruled out

- **`remake_model`** — both scripts already passed `remake_model = false`; the
  model bin cache hit in both cases ("Read the model back ... from: ...").
- **Settled-state cache misses from mismatched damping / depower params** — the
  settled-state cache key (built in `V3Kite/src/stabilization.jl`, function
  around line 160-185) includes `body_start_damping`, `body_sim_damping`,
  `damping_per_stiffness` (as a `_dps` suffix tag) and the depower tape length
  (`dp...` prefix, from `depower_setpoint`, in `build_geom_suffix`,
  `V3Kite/src/calibration.jl:172`). Matching these values only changes *which*
  cache file is looked up; once the file existed, "Loading settled state" and
  "Read the model back" both logged cache hits — yet total `init()` time stayed
  at ~75 s either way.
- **The individual logged sub-steps** ("Read the model back": ~1 s, "RHS
  compiled & integrator built": 0.0 s, "Initial aero solved": 0.0 s) summed to
  ~1 s, nowhere near the ~75 s total — so the cost was not in any step that
  prints its own timing.

## Root cause: package-extension method invalidation

Wrapping the `init()` call in `@time` gave the decisive evidence:

```
73.063581 seconds (222.65 M allocations: 10.533 GiB, 2.26% gc time,
2 lock conflicts, 100.67% compilation time: 80% of which was recompilation)
```

Essentially all of the wall time was **compilation**, and 80% of *that* was
**recompilation** — i.e. Julia recompiling methods that had already been
compiled once (during `V3Kite`'s `PrecompileTools.@compile_workload` in
`src/precompile.jl`) but were subsequently **invalidated**.

`SymbolicAWEModels` (the model backend V3Kite builds on) declares:

```toml
weakdeps = ["GeometryBasics", "MakieControlPlots"]
```

`reel_out_v3.jl` does `using MakieControlPlots` (needed for `plot2d`), which
`simple_parking.jl` never does. Loading `MakieControlPlots` activates
`SymbolicAWEModels`' package extension for it, which adds new, more-specific
methods to generic functions already used internally by
V3Kite/SymbolicAWEModels/ModelingToolkit. In Julia, adding a more-specific
method to an already-compiled generic function **invalidates** every previously
compiled call site of that function — even call sites with nothing to do with
plotting. Those invalidated methods are only actually recompiled the next time
they run, which happens deep inside `init()`.

Sequence:

1. `using MakieControlPlots` loads → SymbolicAWEModels' Makie extension loads →
   a swath of already-precompiled methods (baked in by V3Kite's precompile
   workload) get invalidated.
2. `simple_parking.jl` never loads `MakieControlPlots`, so nothing is
   invalidated; its `init()` reuses the precompiled workload untouched (~3 s).
3. The recompilation cost is paid only when the invalidated code actually runs
   — inside `init()` — matching exactly where the ~75 s shows up.
4. It does not recur on a second `init()` call in the same session: the
   recompiled native code stays valid for the rest of the process.
5. No amount of matching cache-key parameters (depower, damping) could have
   fixed this, since invalidation is unrelated to either data cache.

## Practical remedies

- Build a project sysimage that already includes `MakieControlPlots`
  (`bin/create_sys_image` is referenced in `V3Kite/src/precompile.jl`'s
  comments) — this pays the recompilation cost once, at sysimage build time,
  instead of on every fresh `julia` process.
- Or address it upstream in `SymbolicAWEModels`'s Makie extension, if the
  invalidating methods can be narrowed or avoided (e.g. not overloading a
  `Base`/generic function that other, unrelated code paths also call).

## How to reproduce / verify

```julia
@time s = init(...)   # look for "N% compilation time: M% of which was recompilation"
```

A high compilation-time percentage with a high recompilation share is the
signature of this issue. `using SnoopCompileCore; @snoopr` around the
`using MakieControlPlots` line would enumerate the exact invalidated method
edges if a precise upstream fix is wanted.
