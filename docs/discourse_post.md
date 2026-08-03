Title: Precompiled native code seems unused when a package is sourced from a git rev instead of a local path

On Julia 1.12.6, the same package at the same commit runs ~20x slower when it comes from `Pkg.add(url=..., rev=...)` than from a local `path`. The hot workload is a ModelingToolkit-generated, `RuntimeGeneratedFunctions`-wrapped ODE right-hand side that the package compiles in a `PrecompileTools.@compile_workload`; with the git-rev source it appears to be re-JIT'ed in every fresh process.

Repro ([V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) v1.0.1, ~3 min precompile per environment, two fresh processes):

```julia
using Pkg; Pkg.activate(tempname())
Pkg.add(url="https://github.com/OpenSourceAWE/V3Kite.jl", rev="v1.0.1")
Pkg.precompile()
using V3Kite; @time init(9.0, 200.0)      # 38.2 s

# fresh process; local clone of the same repo, checked out at v1.0.1
using Pkg; Pkg.activate(tempname())
Pkg.develop(path="/path/to/V3Kite.jl")
Pkg.precompile()
using V3Kite; @time init(9.0, 200.0)      # 1.8 s
```

| `V3Kite` source | plain `julia --project` | with a PackageCompiler sysimage |
| --------------- | ----------------------- | ------------------------------- |
| `url` + `rev`   | 38.2 s                  | 33.6 s                          |
| `path`          | 1.77 s                  | 2.25 s                          |

Under the git-rev source the workload *does* run: `Pkg.precompile()` logs it executing `init`, and leaves a ~141 MB pkgimage — same size class as the `path` build. So it compiles and is cached to disk, the native code is simply not used at runtime. Ruled out: the system image, the package's own on-disk model cache, and the `RuntimeGeneratedFunctions` version (identical `git-tree-sha1` in both manifests). Full measurement log incl. the dead ends: [docs/sysimage_notes.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/sysimage_notes.md).

Is this expected — does a git-rev source invalidate RGF-backed precompiled code somehow — or is it a bug?
