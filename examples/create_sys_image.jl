# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Builds a PackageCompiler system image with MakieControlPlots and most of
V3Kite's own dependency chain precompiled for the example scripts.

Measured 2026-08-05 against a `using V3Kite, MakieControlPlots` baseline of
15.2 s (no image) / 1.8 s (V3Kite's own 2.19 GB image): baking in the
solver-stack packages alone (StaticArrays, Rotations, NonlinearSolve, the
OrdinaryDiffEq* packages, SteadyStateDiffEq, SymbolicIndexingInterface) only
reached 12.3 s at 1.57 GB — `--trace-compile` showed almost nothing left to
JIT, so the remaining cost is *loading* `ModelingToolkit` (~5.8 s alone) and
`SymbolicAWEModels` (~5.1 s more on top), neither a dependency of the solver
packages and so not pulled in by `include_transitive_dependencies`. Those two
are also most of what makes V3Kite's own image large, so there is no package
set that is both small and fast — `WINDOWS_IMAGE` below trades speed for size
on that platform only, where the full image doesn't fit.

Neither V3Kite nor SimpleKiteControllers goes into the image, so Revise keeps
applying to both packages' sources. V3Kite is left to Pkg's ordinary
precompilation, where its `@compile_workload` (see its `src/precompile.jl`) covers
the model.

Run via `bin/create_sys_image`, which stacks a throwaway environment holding
PackageCompiler onto `examples/`'s (PackageCompiler is not a dependency of either)
and moves the resulting image into `bin/`.

The workload traced into the image (via `precompile_execution_file`, in an
isolated process) still runs `init`/`step!` on the V3 model, so the example
scripts' full call path is exercised and the baked-in packages' own precompile
statements get recorded.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(@__DIR__)
end

using PackageCompiler

workload = tempname() * ".jl"
write(workload, """
    using V3Kite
    using MakieControlPlots

    set_data_path(v3_data_path())
    s = init(9.51, 150.0; system_yaml = "system_reelout.yaml")
    for _ in 1:5
        step!(s; rel_depower = 0.25, set_length = s.sys_state.l_tether[1])
    end
    """)

const SOLVER_PACKAGES = [:MakieControlPlots, :StaticArrays, :Rotations, :NonlinearSolve,
    :OrdinaryDiffEqBDF, :OrdinaryDiffEqCore, :OrdinaryDiffEqNonlinearSolve,
    :SteadyStateDiffEq, :SymbolicIndexingInterface]
const WINDOWS_IMAGE = Sys.iswindows()
packages = WINDOWS_IMAGE ? [SOLVER_PACKAGES; :ModelingToolkit] :
                            [SOLVER_PACKAGES; :ModelingToolkit; :SymbolicAWEModels]

@info "Creating sysimage..." packages
try
    PackageCompiler.create_sysimage(
        packages;
        sysimage_path = "kps-image_tmp.so",
        include_transitive_dependencies = true,
        precompile_execution_file = workload,
    )
finally
    rm(workload)
end
