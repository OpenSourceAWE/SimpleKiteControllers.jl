# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Builds a PackageCompiler system image with MakieControlPlots precompiled for
the example scripts. Neither V3Kite nor SimpleKiteControllers goes into the
image: SimpleKiteControllers stays out so Revise keeps applying to `src/`, and
V3Kite stays out because baking it into a sysimage defeats its own
`PrecompileTools` workload — V3Kite's model is built from
`RuntimeGeneratedFunctions`-wrapped ModelingToolkit code, whose JIT-compiled
form lives in V3Kite's OWN pkgimage cache; once V3Kite's code is linked into a
custom sysimage instead, that cache no longer applies and `init`/`step!` pay
the full JIT cost on every run regardless of how many times the sysimage is
rebuilt. So V3Kite is left to Pkg's ordinary precompilation, where its
`@compile_workload` (see its `src/precompile.jl`) already covers this.

Run via `bin/create_sys_image`, which stacks a throwaway environment holding
PackageCompiler onto `examples/`'s (PackageCompiler is not a dependency of
either) and moves the resulting image into `bin/`.

The workload traced into the image (via `precompile_execution_file`, in an
isolated process) still runs `init`/`step!` on the V3 model, so the example
scripts' full call path is exercised and MakieControlPlots' own precompile
statements get recorded — V3Kite just doesn't end up linked into the image
itself.
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

@info "Creating sysimage..."
try
    PackageCompiler.create_sysimage(
        [:MakieControlPlots];
        sysimage_path = "kps-image_tmp.so",
        include_transitive_dependencies = true,
        precompile_execution_file = workload,
    )
finally
    rm(workload)
end
