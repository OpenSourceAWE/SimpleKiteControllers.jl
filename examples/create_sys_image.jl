# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Builds a PackageCompiler system image with V3Kite and MakieControlPlots
precompiled for the example scripts. SimpleKiteControllers itself is
deliberately left OUT of the image, so Revise keeps applying to `src/` when
developing against it.

Run via `bin/create_sys_image`, which stacks a throwaway environment holding
PackageCompiler onto `examples/`'s (PackageCompiler is not a dependency of
either) and moves the resulting image into `bin/`.

The workload that gets traced into the image is `init`/`step!` on the V3
model, run in an isolated process by `PackageCompiler.create_sysimage`'s
`precompile_execution_file` so the traced precompile statements land in the
image rather than just the packages' own precompile workloads.
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
        [:V3Kite, :MakieControlPlots];
        sysimage_path = "kps-image_tmp.so",
        include_transitive_dependencies = true,
        precompile_execution_file = workload,
    )
finally
    rm(workload)
end
