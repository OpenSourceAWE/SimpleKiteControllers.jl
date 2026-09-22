# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plot the turn-rate-law coefficients `c1` and `c2` and the steering `delay`
against relative depower `u_s`, with error bars, one figure per `body_damping`
present in the table — the three quantities are only comparable across rows
swept at the same damping.

Read straight from `data/turn_rate_coeffs.yaml` rather than from
[`V3_TURN_RATE_COEFFS`](@ref): the lookup dict carries only `c1`, `c2` and
`delay`, while the `*_std` columns `examples/build_turn_rate_table.jl` writes
live in the file alone. A row from before those columns existed simply plots
without bars.

    include("plot_c1_c2.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using SimpleKiteControllers: skc_data_path, turn_rate_coeffs_file, project_file
using LaTeXStrings
using YAML

# Where the PDFs land: LearningControl's figures/ (the document that includes
# them), NOT this package's output/. An absolute path because the two repos are
# siblings only by convention -- expanded, not assumed, so a missing sibling
# fails with a clear "no such directory" instead of writing somewhere surprising.
const FIG_DIR = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))

# Axis label size, in points. `plotx`' default of 16 is sized for a full-screen
# window; these figures are included at column width, where the label shrinks
# with the figure and 16 pt reads small on paper.
const LABEL_SIZE = 22

# Fraction of the depower span added at each end of the x axis. Without it the
# stack clamps to the data range (`plotx`' default), which puts the first and
# last marker on the spines and cuts their error bars in half.
const X_MARGIN = 0.04

# Half-width of the error bars, in standard deviations. 2 reads as a ~95%
# interval where the underlying sigma is a standard error; the figure title
# states it, because an unlabelled bar is ambiguous by a factor of 3 and that
# ambiguity is worse than either choice. Set to 1 for bare standard deviations.
const K_SIGMA = 2

"""
    _std_column(entries, key; k=K_SIGMA) -> Union{Vector{Float64}, Nothing}

Column `key` of `entries` scaled by `k`, or `nothing` if any row lacks it.
All-or-nothing because `plotx` takes one error series per channel: a partially
filled column would have to be padded, and a padded zero reads as "identified
with no uncertainty" rather than "not recorded".
"""
function _std_column(entries, key; k::Real = K_SIGMA)
    all(haskey(e, key) for e in entries) || return nothing
    return [k * Float64(e[key]) for e in entries]
end

"""
    plot_c1_c2()

For each `body_damping` in `data/turn_rate_coeffs.yaml`, sort its rows by
depower `u_s` and plot `c1`, `c2` and `delay` against it in a stacked,
three-panel figure with error bars from the `c1_std`, `c2_std` and `delay_std`
columns.

The three bars do not mean the same thing. `c1_std`/`c2_std` are the linear
fit's own standard errors, and are optimistic — the residuals of a fitted
flight path are strongly autocorrelated, so the effective sample count is far
below `n`, which is why they come out visibly tighter than the delay's.
`delay_std` is a spread across blocks of one sweep, and the delay is quantised
to the identification timestep, so its panel is a staircase with bars at least
a sample tall.

All three are drawn at `±K_SIGMA` times the recorded sigma. Scaling does not
make the coefficient bars honest — their bias is the autocorrelation above, not
the multiplier — so read them as a lower bound whatever `K_SIGMA` is.

The figures carry no title: they are meant to be included in a document whose
caption says what they show. That caption is the only place `K_SIGMA` is now
stated, so it has to say `±2σ` (or whatever `K_SIGMA` is set to) itself. Each
figure is also written to `$FIG_DIR` as a PDF, one per damping.
"""
function plot_c1_c2()
    path = joinpath(skc_data_path(), turn_rate_coeffs_file(project_file()))
    # A diverged cell is written without c1/c2/delay (`build_turn_rate_table.jl`
    # records the outcome but no coefficients), so there is nothing to plot for it.
    entries = [e for e in YAML.load_file(path)["entries"] if haskey(e, "c1")]
    if isempty(entries)
        @warn "plot_c1_c2: no entries in $path -- identify some with " *
              "examples/build_turn_rate_table.jl first"
        return nothing
    end
    dampings = sort(unique(Float64.(e["body_damping"]) for e in entries); by = string)

    for bd in dampings
        rows = sort([e for e in entries if Float64.(e["body_damping"]) == bd];
                    by = e -> Float64(e["depower"]))
        u_s = [Float64(e["depower"]) for e in rows]
        c1 = [Float64(e["c1"]) for e in rows]
        c2 = [Float64(e["c2"]) for e in rows]
        delay = [Float64(e["delay"]) for e in rows]

        pad = X_MARGIN * (maximum(u_s) - minimum(u_s))

        fig_name = "c1_c2_delay_" * join(round.(bd; digits = 1), "_")
        plotx(u_s, c1, c2, delay;
              xlims = (minimum(u_s) - pad, maximum(u_s) + pad),
              # Round ticks at the sweep's own 0.05 grid: the padded range makes
              # Makie pick 0.27/0.30/0.33/... otherwise, which reads as if the
              # cells had been identified at those settings.
              xticks = 0.25:0.05:0.40,
              # All-math labels with upright units: `L"..."` wraps a string with no
              # `$` in it in math mode entirely, so a bare "delay [s]" would come
              # out italicised letter by letter.
              # u_d, not u_s: the paper (LearningControl/main.tex) writes the
              # relative steering as u_s and the relative depower as u_d, and
              # this axis is the depower.
              xlabel = L"\mathrm{relative\ depower}\ u_\mathrm{d}\ [-]",
              ylabels = [L"c_1\ [\mathrm{1/m}]", L"c_2\ [-]",
                         L"\mathrm{delay}\ [\mathrm{s}]"],
              yerr = [_std_column(rows, "c1_std"), _std_column(rows, "c2_std"),
                      _std_column(rows, "delay_std")],
              scatter = true, disp = true, labelsize = LABEL_SIZE,
              fig = fig_name)
        mkpath(FIG_DIR)
        savefig(joinpath(FIG_DIR, fig_name * ".pdf"))
    end
end

plot_c1_c2()
