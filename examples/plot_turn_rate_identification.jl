# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Fly ONE relay sweep of `build_turn_rate_table.jl` at a single depower and plot
the log: turn rate, apparent wind speed, kite speed and elevation over time.
The turn-rate panel also shows the identified model: the pure delay of
`identify_turn_rate_law` split into a dead time τ and a first-order kite lag T by
`fit_delay_lag`, as the table's `dead_time` and `kite_lag` are.

The sweep is `_run_turn_rate_sweep` with the cell's own `u_s_max` and elevation
floor from `data/turn_rate_coeffs.yaml` when the table has a row for this
depower (the default cap and floor otherwise), so the plot shows the run the
table's row was identified on. Nothing is written to the table; the fit is
printed with `format_turn_rate_report`, followed by τ, T and how much the lag
lowers the residual against the pure delay.

The turn rate is `calc_turn_rate(sl; source = :heading)`, the backward difference
of the heading that `identify_turn_rate_law` fits, aligned to `time[2:end]`; the
other panels use the same samples. The relay excitation, and the fit window,
start at `T_START`.

About two minutes for the sweep. `DEPOWER` is read and cleared like `SHOW_PLOTS`:

    DEPOWER = 0.3; include("plot_turn_rate_identification.jl")   # default 0.275
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using LaTeXStrings
using LinearAlgebra: norm

# `_run_turn_rate_sweep`, the fixed sweep conditions and `_elevation_floor`.
include(joinpath(@__DIR__, "build_turn_rate_table.jl"))

depower = @isdefined(DEPOWER) ? Float64(DEPOWER) : 0.275
DEPOWER = 0.275

# The table's row for this cell, if any: its amplitude cap and elevation floor.
row = let entries = YAML.load_file(joinpath(skc_data_path(), OUT_FILE))["entries"]
    k = findfirst(e -> _entry_key(e) == (BODY_START_DAMPING, depower), entries)
    isnothing(k) ? nothing : entries[k]
end
isnothing(row) && @warn "No row for depower $depower in $OUT_FILE: flying the default cap and floor."
max_steering_cap = isnothing(row) ? MAX_STEERING_CAP : Float64(row["u_s_max"])
elevation_floor = isnothing(row) ? _elevation_floor(depower) :
                  Float64(get(row, "elevation_floor", _elevation_floor(depower)))

r = _run_turn_rate_sweep(depower; max_steering_cap, elevation_floor)
@info "Sweep outcome: $(r.outcome), steering amplitude reached $(r.u_s_max)."
isnothing(r.fit) && error("The identification failed; nothing to plot.")
println(format_turn_rate_report(r.fit))
# The pure delay of the report, split into the dead time τ and the kite's first-order lag T,
# as `add_delay_lag_split!` does for the table's `dead_time` and `kite_lag`.
split = _split_delay(r.fit)
@printf("Dead time τ = %.3f s, kite lag T = %.3f s (pure delay %.3f s); c1 = %.4f 1/m, \
         c2 = %.3f; residual %.3f °/s against %.3f °/s with the pure delay (%.0f %% better).\n",
        split.dead_time, split.lag, r.fit.delay_sec, split.c1, split.c2,
        rad2deg(split.rms_lag), rad2deg(split.rms_delay),
        100 * (1 - split.rms_lag / split.rms_delay))

# The model's turn rate over the fit window, with the fitted τ and T: the steering is
# lag-filtered and shifted by the dead time as `fit_delay_lag` does, whole samples only.
us_model = shift_delay(lag_filter(r.fit.us, split.lag, DT),
                       round(Int, split.dead_time / DT + 0.5))
rate_model = rad2deg.(split.c1 .* r.fit.v_app .* us_model .+
                      split.c2 ./ r.fit.v_app .* sin.(r.fit.psi) .* cos.(r.fit.beta))

# The logger is preallocated for SWEEP_SIM_TIME; a sweep that ends early leaves the rest
# of the rows at zero (time 0 included), which folds the time axis back onto itself.
sl = r.sl[1:findlast(>(0), r.sl.time)]
# calc_turn_rate is aligned to time[2:end], so every other signal starts at index 2 too.
rng = 2:length(sl.time)
turn_rate = rad2deg.(calc_turn_rate(sl; source = :heading, dt = DT))
# The model on the same samples, NaN before the fit window.
turn_rate_model = fill(NaN, length(rng))
window = findall(>=(T_START), sl.time[rng])
length(window) == length(rate_model) || error("The fit window does not match the log.")
turn_rate_model[window] .= rate_model

p = plotx(
    sl.time[rng],
    (turn_rate, turn_rate_model),
    Float64.(sl.v_app[rng]),
    norm.(sl.vel_kite[rng]),
    rad2deg.(sl.elevation[rng]);
    xlabel = L"\mathrm{time}~[\mathrm{s}]",
    ysize = 18,
    ylabels = [
        L"\dot{\psi}~[°/\mathrm{s}]",
        L"v_{\mathrm{a}}~[\mathrm{m/s}]",
        L"v_{\mathrm{k}}~[\mathrm{m/s}]",
        L"\mathrm{elevation}~[°]",
    ],
    labels = [
        [L"\dot{\psi}", @sprintf("model, τ = %.3f s, T = %.3f s", split.dead_time, split.lag)],
        nothing,
        nothing,
        nothing,
    ],
    fig = @sprintf("Turn-rate identification, depower %.3f", depower),
)
display(p)
sleep(0.1)  # Allow Makie to render the plot before continuing

nothing
