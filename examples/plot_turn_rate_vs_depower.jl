# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The turn-rate law identified in the LOW crosswind pattern at every depower of
`data/turn_rate_coeffs.yaml`, plotted over the depower next to the table's 73°
rows: `c1`, `c2`, the dead time and the kite's lag.

For each depower this runs `plot_turn_rate_identification.jl` without its plots
and without the inertia-law comparison: one flight per amplitude of its
`flight_settings`, at the table's wind and constant tether length, and the joint
fit of the steady ones (`joint_delay_lag_fit`). The results go to
`output/turn_rate_low_flights.csv`, one row per depower (`rms` in °/s, speeds in
m/s, times in s), so the plot can be
redrawn without flying again. The dead time and lag of the low flights are at
`v_a` ≈ 20 – 50 m/s, those of the table at ≈ 13 m/s, so they are expected to be
shorter (see `PlanIdentifyTurnRateLaw.md`).

About 5 – 10 minutes per depower. `TR_FROM_CSV` (read and cleared like
`SHOW_PLOTS`) only plots the saved results:

    include("plot_turn_rate_vs_depower.jl")
    TR_FROM_CSV = true; include("plot_turn_rate_vs_depower.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using LaTeXStrings
using Printf
using DelimitedFiles: readdlm, writedlm

from_csv = @isdefined(TR_FROM_CSV) ? TR_FROM_CSV : false
TR_FROM_CSV = false

"The depowers of the table's rows [-]"
table_depowers = [0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40]
csv_file = normpath(joinpath(@__DIR__, "..", "output", "turn_rate_low_flights.csv"))
csv_header = ["depower", "n_steady", "n_flights", "c1", "c2", "dead_time", "lag", "rms",
              "va_min", "va_max", "table_c1", "table_c2", "table_dead_time", "table_lag"]

if !from_csv
    rows = Vector{Vector{Float64}}()
    for dp in table_depowers
        global DEPOWER = dp
        global SHOW_PLOTS = false
        global TR_FIT_LAWS = false
        try
            include(joinpath(@__DIR__, "plot_turn_rate_identification.jl"))
        catch e
            @warn "Depower $dp: the identification failed; no row." exception = e
            continue
        end
        steady = count(f -> f.outcome == :time_limit, flights)
        va = reduce(vcat, [f.fit.v_app for f in joint_flights])
        tab(k) = isnothing(row) ? NaN : Float64(get(row, k, NaN))
        push!(rows, [dp, steady, length(flights), joint.c1, joint.c2, joint.dead_time, joint.lag,
                     rad2deg(joint.rms_lag), minimum(va), maximum(va),
                     tab("c1"), tab("c2"), tab("dead_time"), tab("kite_lag")])
        @info @sprintf("Depower %.3f: %d of %d flights steady, c1 = %.4f, c2 = %.2f, dead %.3f s, lag %.3f s.",
                       dp, steady, length(flights), joint.c1, joint.c2, joint.dead_time, joint.lag)
        # Written after every depower, so a failure later on costs only the rest.
        open(csv_file, "w") do io
            writedlm(io, permutedims(csv_header), ',')
            writedlm(io, permutedims(reduce(hcat, rows)), ',')
        end
    end
end

data, header = readdlm(csv_file, ','; header = true)
col(name) = Float64.(data[:, findfirst(==(name), vec(header))])
dp = col("depower")

p = plotx(
    dp,
    (col("c1"), col("table_c1")),
    (col("c2"), col("table_c2")),
    (col("dead_time"), col("table_dead_time")),
    (col("lag"), col("table_lag"));
    xlabel = L"\mathrm{depower}~[-]",
    ysize = 18,
    ylabels = [
        L"c_1~[1/\mathrm{m}]",
        L"c_2~[-]",
        L"\tau_\mathrm{dead}~[\mathrm{s}]",
        L"T_\mathrm{lag}~[\mathrm{s}]",
    ],
    labels = [
        ["low pattern", "table, 73°"],
        ["low pattern", "table, 73°"],
        ["low pattern", "table, 73°"],
        ["low pattern", "table, 73°"],
    ],
    fig = "Turn-rate law over depower",
)
display(p)
sleep(0.1)  # Allow Makie to render the plot before continuing

nothing
