# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plot the turn-rate law of the table — `c1`, `c2`, the dead time and the kite's lag —
against relative depower `u_d`, with error bars, one figure per `body_damping`
present in the table — the quantities are only comparable across rows swept at the
same damping. This is the paper's figure of the turn-rate law
(`LearningControl/figures/turn_rate_low_pattern.pdf`).

Read straight from the turn-rate table of the selected project (`select_project()`,
`data/turn_rate_coeffs.yaml` in every project so far) rather than from
[`V3_TURN_RATE_COEFFS`](@ref): the lookup dict carries only `c1`, `c2` and
`delay`, while the `*_se` columns `examples/build_turn_rate_table.jl` writes
live in the file alone. A row without them simply plots without bars.

    include("plot_c1_c2.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using GLMakie
using MakieControlPlots
using SimpleKiteControllers: skc_data_path, turn_rate_coeffs_file, project_file, selected_project
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
const LABEL_SIZE = 26

"""
    COLUMN_THEME

The `PAPER_THEME` of `plots_extra.jl` (TeX Gyre Termes, the face of the paper's
body text) with larger tick labels, as in `plot_relay_low_elevation.jl`: plotx
draws 768 px (576 pt) wide, and the figure fills one column of the two-column
layout, about 240 pt, so it is scaled to 0.42. With `LABEL_SIZE`, the text
prints at 7 to 8 pt.
"""
const COLUMN_THEME = Theme(
    fonts = (; regular = "TeX Gyre Termes", bold = "TeX Gyre Termes Bold",
               italic = "TeX Gyre Termes Italic"),
    Axis = (; xticklabelsize = 24, yticklabelsize = 24),
)

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
depower `u_d` and plot `c1`, `c2`, and the dead time `τ_d` and lag `T_k` of the
delay (`fit_delay_lag`), against it in a stacked, four-panel figure with error bars
from the `c1_se`, `c2_se`, `dead_time_se` and `kite_lag_se` columns. A row without
the split plots it as `NaN`, i.e. not at all. One panel each,
not the three times in one with a legend: dead time and lag cross, so a legend
covers data in every corner.

The standard errors are the scatter of the fit over 20 s blocks of the flights,
divided by √(number of blocks) (`block_standard_errors` of
`build_turn_rate_table.jl`), not the linear fit's own, which assume independent
residuals. Dead time and lag trade against each other within a block, so their
bars carry that too. All are drawn at `±K_SIGMA` times the recorded sigma.

The figures carry no title: they are meant to be included in a document whose
caption says what they show. That caption is the only place `K_SIGMA` is now
stated, so it has to say `±2σ` (or whatever `K_SIGMA` is set to) itself. Each
figure is also written to `$FIG_DIR` as a PDF, `turn_rate_low_pattern.pdf`, with the
damping appended when the table holds more than one.
"""
function plot_c1_c2()
    path = joinpath(skc_data_path(), turn_rate_coeffs_file(project_file(selected_project())))
    # Only the rows `turn_rate_coeffs` uses: a cell whose flights all sank keeps the fit of
    # those flights (`outcome: low_elevation`), and a diverged one no coefficients at all.
    entries = [e for e in YAML.load_file(path)["entries"]
               if haskey(e, "c1") && get(e, "outcome", "") in ("time_limit", "sweep_done")]
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
        dead_time = [Float64(get(e, "dead_time", NaN)) for e in rows]
        kite_lag = [Float64(get(e, "kite_lag", NaN)) for e in rows]

        pad = X_MARGIN * (maximum(u_s) - minimum(u_s))

        fig_name = "turn_rate_low_pattern" *
                   (length(dampings) > 1 ? "_" * join(round.(bd; digits = 1), "_") : "")
        # The save re-runs the builder, so it has to happen under the theme too.
        with_theme(COLUMN_THEME) do
            plotx(u_s, c1, c2, dead_time, kite_lag;
                  xlims = (minimum(u_s) - pad, maximum(u_s) + pad),
                  # Round ticks at the table's own 0.05 grid: the padded range makes
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
                             L"\tau_\mathrm{d}\ [\mathrm{s}]", L"T_\mathrm{k}\ [\mathrm{s}]"],
                  yerr = [_std_column(rows, "c1_se"), _std_column(rows, "c2_se"),
                          _std_column(rows, "dead_time_se"),
                          _std_column(rows, "kite_lag_se")],
                  scatter = true, disp = true, labelsize = LABEL_SIZE,
                  fig = fig_name)
            mkpath(FIG_DIR)
            savefig(joinpath(FIG_DIR, fig_name * ".pdf"))
        end
    end
end

plot_c1_c2()
