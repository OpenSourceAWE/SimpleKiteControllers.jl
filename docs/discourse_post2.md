Title: Follow-up: a git-rev source seems to disable reuse of *externally owned* precompiled CodeInstances

Follow-up to [the post above](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/discourse_post.md), where the same package at the same commit inited ~20x slower from `Pkg.add(url=..., rev=...)` than from a local `path`. I profiled it instead of comparing artifacts: same script, same system image, both sources, `--trace-compile --trace-compile-timing` (Julia 1.12.6, V3Kite v1.0.2).

| | `url` + `rev` | `path` |
| --- | --- | --- |
| `init` | 45.7 s | 2.6 s |
| methods compiled | 309 | 74 |
| total compile time | 36.5 s | 3.4 s |

So the gap is inference and codegen, and 245 signatures compile under `url`/`rev` that never compile under `path`. Three groups carry it:

| compiled only under `url`/`rev` | n | time |
| --- | --- | --- |
| `DiffEqBase.promote_f(::ODEFunction{true, AutoSpecialize, ModelingToolkitBase.GeneratedFunctionWrapper{...}}, ...)` | 1 | 22.5 s |
| `SymbolicIndexingInterface.var"#46#47"{ODEIntegrator{FBDF{...}}}` (observed-variable getters) | 23 | 9.2 s |
| `Serialization.deserialize` / `RuntimeGeneratedFunctions.drop_expr`, one per RGF type | 100 | 0.5 s |

What these have in common: the *method* belongs to another package (DiffEqBase, SymbolicIndexingInterface, Serialization), while the *type* it specializes on only comes into existence when V3Kite's `@compile_workload` runs `init`. They are external specializations, cached into V3Kite's own pkgimage rather than their owners'. That is the class of code that goes unused — the package's *internal* methods appear to be fine.

Three things support that reading.

**Under `path`, those signatures only JIT when pkgimages are switched off.** Same script, same image, `path` source:

| | traced lines | `promote_f` | SII getters |
| --- | --- | --- | --- |
| pkgimages on | 30 | 0 | 0 |
| `--pkgimages=no` | 293 | 1 | 51 |

So the CodeInstances really are in V3Kite's pkgimage and really are used — under `path`. Under `url`/`rev` the same signatures JIT with pkgimages fully on.

**Feeding exactly those statements to PackageCompiler halves the `url`/`rev` run.** Take the 293-line `--pkgimages=no` trace as a `precompile_statements_file`, change nothing else — no V3Kite in the package set, same workload:

| system image (`url`/`rev` source) | `init` | size |
| --- | --- | --- |
| baseline | 44.8 / 45.6 / 45.8 s | 2194594064 B |
| \+ the 293 statements | 22.5 / 22.2 / 22.2 s | 2195908888 B |

Half the cost for 1.3 MB. In the new trace `promote_f` (21.5 s) and all the `SymbolicIndexingInterface` getters are gone, and 274 traced statements fall to 38. The statements don't mention V3Kite either: replaying them through PackageCompiler's own filter in a process where V3Kite is *not* loaded, 277 of 293 survive — `promote_f` among them — and the 16 that drop do so with `UndefVarError: V3Kite`.

**The remainder is the one thing that will not bake:** ~29 s of `RuntimeGeneratedFunctions.generated_callfunc` over 26 instances, the ODE right-hand side itself. Presumably because its generator reads RGF's runtime body cache, populated only when the model binary is deserialized, so its code cannot be validated at image-load time.

So: is a git-rev-sourced package expected to lose the external CodeInstances its `@compile_workload` produced, while a `path`-sourced one keeps them? If that is by design, what invalidates them — and is there a supported way to keep them, short of vendoring the dependency as a local checkout?

Measurements and the dead ends behind them: [docs/sysimage_notes.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/sysimage_notes.md).
