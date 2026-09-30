# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The turn-rate law identified in the LOW crosswind pattern at every depower of
`data/turn_rate_coeffs.yaml`, plotted over the depower with error bars: `c1`,
`c2`, the dead time and the kite's lag.

For each depower this runs `plot_turn_rate_identification.jl` without its plots
and without the extended-law comparison: one flight per amplitude of its
`flight_settings`, at the table's wind and constant tether length, and the joint
fit of the steady ones (`joint_delay_lag_fit`). Two things are saved after every
depower, so a crash later on costs only the rest:

- `output/turn_rate_low_flights.csv`, one row per depower: the fit, its standard
  errors, and the table's 73° row for comparison (`rms` in °/s, speeds in m/s,
  times in s). Enough to redraw or restyle the plot (`from_csv`).
- `output/turn_rate_low_flights/depower_<u_d>.csv`: the fit windows of the steady
  flights, one row per sample (`flight`, `amplitude`, `time` [s], `us`, `rate`
  [rad/s], `v_app` [m/s], `psi`, `beta` [rad]). Enough to refit and recompute the
  error bars, e.g. with another `block_length` (`from_raw`).

A fresh run first renames both, with the time of their last change appended, so
it never overwrites an earlier result. The results of 2026-09-29 are archived in
`data/turn_rate_low_flights.tar.gz`; `from_csv` and `from_raw` unpack it into
`output/` when the files are missing there.

The error bars are `±k_sigma` standard errors from blocks: every steady flight's
fit window is cut into `block_length` blocks, dead time, lag, `c1` and `c2` are fitted
on each block alone (`joint_delay_lag_fit`), and the standard error of each is
the scatter over the blocks divided by √(number of blocks). Not the linear fit's
own standard errors, which assume independent residuals: the residuals of a flown
path are strongly autocorrelated, so those come out far too small. The dead time
and lag trade against each other within a block, so their bars also carry that.

About 5 – 10 minutes per depower. The inputs `from_csv` and `from_raw` do without flying
(`src/script_inputs.jl`):

    include("plot_turn_rate_vs_depower.jl")                          # fly, save, plot
    run_example("plot_turn_rate_vs_depower.jl"; from_csv = true)     # plot the saved rows
    run_example("plot_turn_rate_vs_depower.jl"; from_raw = true)     # refit the saved windows, plot
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using LaTeXStrings
using Printf
using DelimitedFiles: readdlm, writedlm
using Statistics: std
using Base.CoreLogging: with_logger, NullLogger
import Dates
using SimpleKiteControllers: run_example, script_inputs

(; from_csv, from_raw) = script_inputs(@__FILE__, (; from_csv = false, from_raw = false))

"The depowers of the table's rows [-]"
table_depowers = [0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40]
csv_file = normpath(joinpath(@__DIR__, "..", "output", "turn_rate_low_flights.csv"))
raw_dir = normpath(joinpath(@__DIR__, "..", "output", "turn_rate_low_flights"))
raw_file(dp) = joinpath(raw_dir, @sprintf("depower_%.3f.csv", dp))
# The results of 2026-09-29, archived: the summary and the fit windows, as a flying run writes them.
archive_file = normpath(joinpath(@__DIR__, "..", "data", "turn_rate_low_flights.tar.gz"))

"""
    unpack_archive_if_missing(need_windows)

Unpack `archive_file` into `output/` (never into `data/`) when the summary, or
with `need_windows` the fit-window folder, is missing there. Files already in
`output/` are never overwritten: a missing piece is only restored when the whole
set is absent.
"""
function unpack_archive_if_missing(need_windows)
    missing_csv = !isfile(csv_file)
    missing_raw = need_windows && !isdir(raw_dir)
    (missing_csv || missing_raw) || return
    isfile(archive_file) || error("Neither $(missing_csv ? csv_file : raw_dir) nor $archive_file exists: fly first.")
    (isfile(csv_file) || isdir(raw_dir)) &&
        error("Only part of the results is in $(dirname(csv_file)); move it away to unpack $archive_file.")
    run(`tar -xzf $archive_file -C $(dirname(csv_file))`)
    @info "Unpacked $archive_file into $(dirname(csv_file))."
end
csv_header = ["depower", "n_steady", "n_flights", "c1", "c2", "dead_time", "lag", "rms",
              "va_min", "va_max", "table_c1", "table_c2", "table_dead_time", "table_lag",
              "n_blocks", "c1_se", "c2_se", "dead_time_se", "lag_se"]
block_length = 20.0  # [s] length of the blocks the standard errors are taken over
k_sigma = 2     # [-] half-width of the error bars in standard errors; 2 reads as ~95 %

"""
    block_standard_errors(fits; block_t=block_length) -> NamedTuple

Standard errors of the dead time, lag, `c1` and `c2`: each fit window of `fits`
(from `identify_turn_rate_law`) is cut into blocks of `block_t` seconds, the
four are fitted on each block alone with `joint_delay_lag_fit`, and each standard
error is the scatter over the blocks divided by √(number of blocks). Returns
`(; n, c1, c2, dead_time, lag)`, `NaN` for fewer than two blocks.
"""
function block_standard_errors(fits; block_t = block_length)
    nb = round(Int, block_t / DT)
    blocks = NamedTuple[]
    for f in fits, k in 1:nb:length(f.rate) - nb + 1
        r = k:k + nb - 1
        sub = (; us = f.us[r], rate = f.rate[r], v_app = f.v_app[r], psi = f.psi[r], beta = f.beta[r])
        # Muted: a short block's lag may hit its search limit, which is scatter, not news.
        push!(blocks, with_logger(() -> joint_delay_lag_fit([sub], DT), NullLogger()))
    end
    se(field) = length(blocks) > 1 ?
        std([getfield(b, field) for b in blocks]) / sqrt(length(blocks)) : NaN
    return (; n = length(blocks), c1 = se(:c1), c2 = se(:c2), dead_time = se(:dead_time), lag = se(:lag))
end

"""
    read_raw(dp) -> Vector{NamedTuple}

The fit windows `plot_turn_rate_identification.jl` saved for depower `dp`
(its input `windows_dir`), one per flight, with the
fields `joint_delay_lag_fit` and `block_standard_errors` read.
"""
function read_raw(dp)
    m, h = readdlm(raw_file(dp), ','; header = true)
    c(name) = Float64.(m[:, findfirst(==(name), vec(h))])
    flight = c("flight")
    return [(; time = c("time")[i], us = c("us")[i], rate = c("rate")[i], v_app = c("v_app")[i],
             psi = c("psi")[i], beta = c("beta")[i])
            for i in (findall(==(k), flight) for k in sort(unique(flight)))]
end

"Rename `path` with the time of its last change appended, if it exists"
function keep_old(path)
    ispath(path) || return
    stamp = Dates.format(Dates.unix2datetime(mtime(path)), "yyyy-mm-dd_HHMMSS")
    base, ext = splitext(path)
    mv(path, base * "_" * stamp * ext)
    @info "Kept the previous result as $(base * "_" * stamp * ext)."
end

(from_csv || from_raw) && unpack_archive_if_missing(from_raw)

if from_raw
    # DT, joint_delay_lag_fit and friends, without flying.
    include(joinpath(@__DIR__, "build_turn_rate_table.jl"))
    old, old_header = readdlm(csv_file, ','; header = true)
    oc(name) = Float64.(old[:, findfirst(==(name), vec(old_header))])
    rows = Vector{Vector{Float64}}()
    for (k, dp) in enumerate(oc("depower"))
        isfile(raw_file(dp)) || (@warn "No saved fit windows for depower $dp; row dropped."; continue)
        # Local: the identification script leaves globals of these names in Main.
        local fits, joint, se, va
        fits = read_raw(dp)
        joint = joint_delay_lag_fit(fits, DT)
        se = block_standard_errors(fits)
        va = reduce(vcat, [f.v_app for f in fits])
        push!(rows, [dp, oc("n_steady")[k], oc("n_flights")[k], joint.c1, joint.c2, joint.dead_time,
                     joint.lag, rad2deg(joint.rms_lag), minimum(va), maximum(va),
                     oc("table_c1")[k], oc("table_c2")[k], oc("table_dead_time")[k], oc("table_lag")[k],
                     se.n, se.c1, se.c2, se.dead_time, se.lag])
    end
    open(csv_file, "w") do io
        writedlm(io, permutedims(csv_header), ',')
        writedlm(io, permutedims(reduce(hcat, rows)), ',')
    end
elseif !from_csv
    keep_old(csv_file)
    keep_old(raw_dir)
    rows = Vector{Vector{Float64}}()
    for dp in table_depowers
        try
            # It saves each depower's fit windows to `raw_dir`.
            run_example("plot_turn_rate_identification.jl"; depower = dp, show_plots = false,
                        fit_laws = false, windows_dir = raw_dir)
        catch e
            @warn "Depower $dp: the identification failed; no row." exception = e
            continue
        end
        local steady, va, se
        steady = count(f -> f.outcome == :time_limit, flights)
        va = reduce(vcat, [f.fit.v_app for f in joint_flights])
        tab(k) = isnothing(row) ? NaN : Float64(get(row, k, NaN))
        se = block_standard_errors([f.fit for f in joint_flights])
        push!(rows, [dp, steady, length(flights), joint.c1, joint.c2, joint.dead_time, joint.lag,
                     rad2deg(joint.rms_lag), minimum(va), maximum(va),
                     tab("c1"), tab("c2"), tab("dead_time"), tab("kite_lag"),
                     se.n, se.c1, se.c2, se.dead_time, se.lag])
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
# A depower at which no flight stayed up has no steady fit and no error bars
# (at 0.40, 2026-09-29): its row is a fit of crashing flights, so it is not plotted.
keep = Float64.(data[:, findfirst(==("n_steady"), vec(header))]) .> 0
col(name) = Float64.(data[:, findfirst(==(name), vec(header))])[keep]
dp = col("depower")

# Where the PDF lands: LearningControl's figures/, as in `plot_c1_c2.jl`.
fig_dir = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))
fig_name = "turn_rate_low_pattern"

pad = 0.04 * (maximum(dp) - minimum(dp))
# `disp = true`: plotx shows the figure itself, as in `plot_c1_c2.jl`. No title:
# the caption in the paper states the ±k_sigma standard errors.
plotx(dp, col("c1"), col("c2"), col("dead_time"), col("lag");
      xlims = (minimum(dp) - pad, maximum(dp) + pad),
      xticks = 0.25:0.05:0.40,
      xlabel = L"\mathrm{relative\ depower}\ u_\mathrm{d}\ [-]",
      ylabels = [L"c_1\ [\mathrm{1/m}]", L"c_2\ [-]",
                 L"\tau_\mathrm{d}\ [\mathrm{s}]", L"T_\mathrm{k}\ [\mathrm{s}]"],
      yerr = [k_sigma .* col("c1_se"), k_sigma .* col("c2_se"),
              k_sigma .* col("dead_time_se"), k_sigma .* col("lag_se")],
      scatter = true, disp = true, labelsize = 22,
      fig = fig_name)
mkpath(fig_dir)
savefig(joinpath(fig_dir, fig_name * ".pdf"))

nothing
