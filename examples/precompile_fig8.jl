# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Minimal workload that builds the model `simple_fig8.jl` runs — `init` plus five
`step!`s, no controller, no plotting — for tracing the SciML specializations keyed
to it: `DiffEqBase.promote_f` (22.5 s at runtime) and the
`SymbolicIndexingInterface` observed-variable getters (9.2 s). Context and
measurements: [docs/sysimage_notes.md](../docs/sysimage_notes.md).

It exists for the speed, not for the model: V3Kite's own workload traces a
byte-identical `promote_f` signature (same RGF `id`), so this one is not more
specific — it just runs in seconds instead of a 60 s simulation with two GLMakie
windows.

**A trace is only useful with `--pkgimages=no`.** Under the `path` source the
specializations come from V3Kite's pkgimage, nothing JITs, and `--trace-compile`
records 30 lines with none of them in it:

    julia --project=examples -J bin/kps-image-1.12.so --pkgimages=no \\
          --trace-compile=fig8.log examples/precompile_fig8.jl

Usable either as `precompile_statements_file` (that log) or as
`precompile_execution_file` when building V3Kite's system image — the one
`bin/run_julia` uses here, copied over by `bin/copy_image`, NOT this repo's:

    V3KITE_PRECOMPILE_FILE=\$PWD/examples/precompile_fig8.jl ../V3Kite/bin/create_sys_image

PackageCompiler runs it with `JULIA_LOAD_PATH` pinned to V3Kite's `test/`
environment, so it may use ONLY V3Kite and that environment's dependencies —
`SimpleKiteControllers` is not among them, which is why the four settings below are
duplicated instead of read through `FC_Settings`. They must track
`data/fc_settings.yaml`; everything else `init` takes from the system yaml itself.
"""

using V3Kite
using KiteUtils: Settings

# Mirrors data/fc_settings.yaml — FC_Settings is unavailable in this environment.
const V_WIND = 5.0
const BODY_DAMPING = [0.0, 0.0, 40.0]
const ELEVATION = 73.0
const DEPOWER_SETPOINT = 0.26

const DATA = normpath(joinpath(@__DIR__, "..", "data"))
const PROJECT = joinpath(DATA, "system_fig8_200m.yaml")

set_data_path(DATA)

# Absolute `system_yaml` wins over init's `data_path`, and makes KiteUtils resolve
# sim_settings and wc_settings next to it — the same files simple_fig8.jl gets.
s = init(V_WIND, Settings(PROJECT).l_tether;
    body_damping = BODY_DAMPING, elevation = ELEVATION,
    depower_setpoint = DEPOWER_SETPOINT, system_yaml = PROJECT,
    sim_time = 1.0, warmup_time = 0.0)

# Force mode is left out on purpose: WinchForceController needs
# SimpleKiteControllers' winch_force_gains, and the winch does not enter the
# model expression.
for _ in 1:5
    step!(s; rel_depower = DEPOWER_SETPOINT, rel_steering = 0.0,
        set_length = s.sys_state.l_tether[1])
end

nothing
