# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plot the turn-rate-law coefficients `c1` and `c2` against relative depower
`u_s`, from [`V3_TURN_RATE_COEFFS`](@ref) (`data/turn_rate_coeffs.yaml`, see
`turn_rate_table.jl`). One figure per `body_damping` present in the table,
since the two coefficients are only comparable across rows swept at the same
damping.

    include("plot_c1_c2.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using SimpleKiteControllers: V3_TURN_RATE_COEFFS

"""
    plot_c1_c2()

For each `body_damping` in [`V3_TURN_RATE_COEFFS`](@ref), sort its rows by
depower `u_s` and plot `c1` and `c2` against it in a stacked, two-panel
figure, one figure per damping.
"""
function plot_c1_c2()
    dampings = sort(unique(first(k) for k in keys(V3_TURN_RATE_COEFFS)); by = string)

    for bd in dampings
        rows = sort([(dp, v) for ((b, dp), v) in V3_TURN_RATE_COEFFS if b == bd]; by = first)
        u_s = first.(rows)
        c1 = [r[2].c1 for r in rows]
        c2 = [r[2].c2 for r in rows]

        fig_name = "c1_c2_" * join(round.(bd; digits = 1), "_")
        plotx(u_s, c1, c2;
              xlabel = "relative depower u_s [-]",
              ylabels = ["c1 [-]", "c2 [-]"],
              scatter = true, disp = true,
              title = "turn-rate coefficients, body_damping = $bd",
              fig = fig_name)
    end
end

plot_c1_c2()
