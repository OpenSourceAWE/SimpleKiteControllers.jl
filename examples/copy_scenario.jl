# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Archive the last finished `simple_opt_reelout.jl` run into `output/scenarios/`
WITHOUT overwriting an existing scenario at the same wind speed.

A thin wrapper around `move_scenario.jl`: it sets `UNIQUE_SCENARIO = true`
(see that file's docstring) and includes it, so a run at a wind speed that
already has a `vNN` folder lands in `vNN_2`, `vNN_3`, ... instead of replacing
it. Everything else — compression, the `overwrite = true` for a target that
did not already exist, the archive being removed from `output/archives/` —
is exactly `move_scenario.jl`'s own behaviour.

    include("copy_scenario.jl")
"""

UNIQUE_SCENARIO = true
include(joinpath(@__DIR__, "move_scenario.jl"))
