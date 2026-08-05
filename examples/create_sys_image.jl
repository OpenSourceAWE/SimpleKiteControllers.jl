# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Builds a PackageCompiler system image with MakieControlPlots precompiled for the
example scripts. That is all it buys: GLMakie/MakieControlPlots load time. It does
not shorten `init()`, which is governed by whether the model binary being
deserialized is the one V3Kite's `@compile_workload` compiled against — see
[docs/sysimage_notes.md](../docs/sysimage_notes.md).

Neither V3Kite nor SimpleKiteControllers goes into the image, so Revise keeps
applying to both packages' sources. V3Kite is left to Pkg's ordinary
precompilation, where its `@compile_workload` (see its `src/precompile.jl`) covers
the model.

Run via `bin/create_sys_image`, which stacks a throwaway environment holding
PackageCompiler onto `examples/`'s (PackageCompiler is not a dependency of either)
and moves the resulting image into `bin/`.

The workload traced into the image (via `precompile_execution_file`, in an
isolated process) still runs `init`/`step!` on the V3 model, so the example
scripts' full call path is exercised and MakieControlPlots' own precompile
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
