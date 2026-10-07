# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Identify the turn-rate law at a LOW elevation, from several relay flights at one
depower, and plot each flight: turn rate, apparent wind speed, kite speed and
elevation over time.

The flights `build_turn_rate_table` identifies a row of `data/turn_rate_coeffs.yaml`
from (`_fly_low_flights` of `build_turn_rate_table.jl`): every flight relays about
a crosswind heading (`±HEADING_CENTER`), reverses at `±az_reverse` of azimuth and
tilts the band to hold `EL_HOLD`, so the kite flies a lazy-eight-like pattern low
in the wind window, at `v_a` ≈ 20 – 50 m/s (depower 0.275, 2026-09-29).

One flight per entry of `flight_settings(depower)`, each at a fixed amplitude for
`SWEEP_SIM_TIME`. Each is fitted on its own (`identify_turn_rate_law`, then
`fit_delay_lag`), and all steady ones together (`joint_delay_lag_fit`; all of them
if none flew the full time): one dead time, lag, `c1` and `c2` for every flight,
the steering of each flight filtered and shifted separately so no shift crosses
from one flight into the next. The fit window of a flight starts at the first
sample below `MAX_ELEVATION` after `T_START` and runs to its end, contiguous, as
the backward-difference turn rate and the delay search need. The result is
compared with the table's row at the same depower, and with the extended law
(`fit_laws`). Nothing is written to the table.

The turn-rate panel shows the measured rate, `calc_turn_rate(sl; source =
:heading)`, and the joint model, which starts where the flight's fit window does.
The elevation panel carries `MAX_ELEVATION`.

Under a minute per flight (the 18 flights of `build_turn_rate_table.jl` took about
12 minutes, 2026-10-02). The inputs `depower`, `v_wind` (the wind speed,
default the table's `V_WIND`) and `v_reelout` (the reel-out speed, default 0) are
passed with `run_example` (`src/script_inputs.jl`):

    run_example("plot_turn_rate_identification.jl"; depower = 0.3)    # default 0.275
    run_example("plot_turn_rate_identification.jl"; v_wind = 6.5)     # low v_a
    run_example("plot_turn_rate_identification.jl"; v_reelout = 1.0)  # reeling out
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using LaTeXStrings
using LinearAlgebra: norm
using Statistics: median, var

using SimpleKiteControllers: run_example, script_inputs
# `_fly_low_flights`, the fixed conditions of the flights, `out_file` and `_entry_key`.
run_example("build_turn_rate_table.jl"; identify = false)

# ==================== USER PARAMETERS ==================== #

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
# `v_wind` [m/s]: wind of the flights; V_WIND = 9.51 is the table's. At 6.5 the pattern flies
# v_a 13 – 36 m/s, 39 % of it below 20 m/s where V3 found the law too fast; at 5.0 it drifts out
# of the window (2026-09-29). `v_reelout` [m/s]: reel-out speed of the flights from T_START on, up
# to REELOUT_L_MAX; 0 holds the length. `show_plots`, `fit_laws`: plots, and the comparison of the
# current law with the extended law (~1 – 2 min).
(; depower, v_wind, v_reelout, show_plots, fit_laws) =
    script_inputs(@__FILE__, (; depower = 0.275, v_wind = V_WIND, v_reelout = 0.0,
                               show_plots = true, fit_laws = true))
depower, v_wind, v_reelout = Float64(depower), Float64(v_wind), Float64(v_reelout)

# ==================== JOINT FIT ========================== #

"The model's turn rate [°/s] of `fit`'s window for the delay-lag fit `dl`"
model_rate(fit, dl, d = round(Int, dl.dead_time / DT + 0.5)) =
    rad2deg.(dl.c1 .* fit.v_app .* shift_delay(lag_filter(fit.us, dl.lag, DT), d) .+
             dl.c2 ./ fit.v_app .* sin.(fit.psi) .* cos.(fit.beta))

# ==================== EXTENDED LAW ======================= #

"""
    law_data(f, source) -> NamedTuple

The samples of flight `f`'s fit window for a turn-rate fit on `source` (`:heading`
or `:course`): the rate [rad/s] (`calc_turn_rate`, aligned to `time[2:end]`), the
angle it is the rate of, the steering, `v_a`, the elevation and `v_τ` [m/s].
"""
function law_data(f, source)
    sl = f.sl
    rng = 2:length(sl.time)
    w = findall(>=(f.t_fit), sl.time[rng])
    ang = source === :course ? sl.course : sl.heading
    vk = norm.(sl.vel_kite)
    v_tau = sqrt.(max.(vk .^ 2 .- Float64.(first.(sl.v_reelout)) .^ 2, 0.0))
    return (; rate = calc_turn_rate(sl; source, dt = DT)[w],
            ang = Float64.(wrap_to_pi.(ang[rng][w])), us = Float64.(sl.steering[rng][w]),
            v_app = Float64.(sl.v_app[rng][w]), beta = Float64.(sl.elevation[rng][w]),
            v_tau = v_tau[rng][w])
end

"""
    extended_law_fit(data, dt; es=0:0.05:3, lag_max=0.5, t_max=0.5) -> NamedTuple

Fit the extended turn-rate law, the current one plus the mass term `k4·m·v_τ` in the
denominator, in V3Kite's sign convention
and with `k2 = 1` (`oldplans/PlanIdentifyTurnRateLaw.md`),

    rate = (c1·v_a²·u_s + c2·sin(angle)·cos(β)) / (v_a + e·v_τ),

on the flights `data` (from `law_data`). `e = 0` is the current law. For a fixed
`e` the law is linear in `c1`, `c2`; `e`, the dead time (whole samples up to
`t_max`) and the first-order lag (`0:2dt:lag_max`) are searched on grids, each
flight's steering filtered and shifted on its own and the first `t_max/dt`
samples of every flight dropped, as in `joint_delay_lag_fit`.

Returns `(; best, current)`, each `(; e, c1, c2, dead_time, lag, d, rms)`: the best
fit over all `e`, and the best with `e = 0`.
"""
function extended_law_fit(data, dt; es = 0:0.05:3, lag_max = 0.5, t_max = 0.5)
    dmax = round(Int, t_max / dt)
    trim(x) = x[dmax + 1:end]
    cat(field) = stack_fits(data, field; skip = dmax)
    rate, ang, v_app, beta, v_tau = cat(:rate), cat(:ang), cat(:v_app), cat(:beta), cat(:v_tau)
    grav = sin.(ang) .* cos.(beta)
    best = current = nothing
    for T in 0:2dt:lag_max
        ufs = [lag_filter(x.us, T, dt) for x in data]
        for d in 0:dmax
            us = reduce(vcat, [trim(shift_delay(u, d)) for u in ufs])
            for e in es
                den = v_app .+ e .* v_tau
                A = [v_app .^ 2 .* us ./ den grav ./ den]
                c = A \ rate
                rms = sqrt(sum(abs2, rate .- A * c) / length(rate))
                cand = (; e, c1 = c[1], c2 = c[2], dead_time = max(d - 0.5, 0.0) * dt, lag = T, d, rms)
                (isnothing(best) || rms < best.rms) && (best = cand)
                e == 0 && (isnothing(current) || rms < current.rms) && (current = cand)
            end
        end
    end
    best.e >= last(es) - step(es) / 2 && @warn "extended_law_fit: e hit the end of its grid, $(last(es))."
    return (; best, current)
end

"The turn rate [rad/s] of the law `p` (from `extended_law_fit`) on one flight's `data`"
law_rate(x, p) = let us = shift_delay(lag_filter(x.us, p.lag, DT), p.d)
    (p.c1 .* x.v_app .^ 2 .* us .+ p.c2 .* sin.(x.ang) .* cos.(x.beta)) ./ (x.v_app .+ p.e .* x.v_tau)
end

# ======================== FLIGHTS ======================== #

# The table's row for this cell, if any, to compare with.
row = let entries = YAML.load_file(joinpath(skc_data_path(), out_file()))["entries"]
    k = findfirst(e -> _entry_key(e) == (BODY_START_DAMPING, depower), entries)
    isnothing(k) ? nothing : entries[k]
end
isnothing(row) && @warn "No row for depower $depower in $(out_file()): nothing to compare with."

(; flights, joint_flights, joint) = _fly_low_flights(depower; v_wind, v_reelout)
isnothing(joint) && error("No flight came below MAX_ELEVATION; nothing to fit.")

# ======================== REPORT ========================= #

println()
@printf("%-22s %7s %9s %8s %8s %8s %9s  %s\n", "", "c1 [1/m]", "c2 [-]", "dead [s]", "lag [s]",
        "rms[°/s]", "samples", "v_a [m/s], v_τ/v_a (median), elevation [°]")
for f in flights
    iw = f.sl.time .>= f.t_fit
    @printf("amplitude %.3f          %7.4f %9.3f %8.3f %8.3f %8.3f %9d  %.1f – %.1f, %.2f – %.2f (%.2f), %.1f – %.1f\n",
            f.a, f.dl.c1, f.dl.c2, f.dl.dead_time, f.dl.lag, rad2deg(f.dl.rms_lag), length(f.fit.time),
            extrema(f.fit.v_app)..., extrema(f.v_ratio)..., median(f.v_ratio), extrema(f.el[iw])...)
end
@printf("joint, %d flights        %7.4f %9.3f %8.3f %8.3f %8.3f %9d  (pure delay: rms %.3f °/s)\n",
        length(joint_flights), joint.c1, joint.c2, joint.dead_time, joint.lag, rad2deg(joint.rms_lag),
        joint.n, rad2deg(joint.rms_delay))
isnothing(row) ||
    @printf("table row              %7.4f %9.3f %8.3f %8.3f %8s %9s  v_a %.1f m/s, %s\n",
            row["c1"], row["c2"], get(row, "dead_time", NaN), get(row, "kite_lag", NaN), "", "",
            row["v_app"], row["date"])

# Current law (e = 0) against the law with the mass term, on the heading and on the course,
# each with its own delay and lag; VAF per v_a bin on all flights, the first t_max skipped.
va_bins = [10, 15, 20, 25, 30, 40, 60]   # [m/s]
laws = fit_laws ? Dict(source => extended_law_fit([law_data(f, source) for f in joint_flights], DT)
                       for source in (:heading, :course)) : nothing
for source in (fit_laws ? (:heading, :course) : ())
    data = [law_data(f, source) for f in joint_flights]
    L = laws[source]
    println()
    @printf("%s rate: current law c1 = %.4f, c2 = %.3f, dead %.3f s, lag %.3f s, rms %.2f °/s\n",
            source, L.current.c1, L.current.c2, L.current.dead_time, L.current.lag, rad2deg(L.current.rms))
    @printf("%s rate: extended law e = %.2f, c1 = %.4f, c2 = %.3f, dead %.3f s, lag %.3f s, rms %.2f °/s\n",
            source, L.best.e, L.best.c1, L.best.c2, L.best.dead_time, L.best.lag, rad2deg(L.best.rms))
    skip = round(Int, 0.5 / DT)
    meas = reduce(vcat, [x.rate[skip + 1:end] for x in data])
    va = reduce(vcat, [x.v_app[skip + 1:end] for x in data])
    pred(p) = reduce(vcat, [law_rate(x, p)[skip + 1:end] for x in data])
    cur, new = pred(L.current), pred(L.best)
    @printf("  v_a bin [m/s]   samples   VAF current   VAF extended\n")
    for k in 1:length(va_bins) - 1
        i = findall(v -> va_bins[k] <= v < va_bins[k + 1], va)
        length(i) < 100 && continue
        vaf(p) = 1 - var(meas[i] .- p[i]) / var(meas[i])
        @printf("  %4d – %-4d     %7d   %11.3f   %11.3f\n", va_bins[k], va_bins[k + 1], length(i),
                vaf(cur), vaf(new))
    end
end

# ========================= PLOTS ========================= #

for f in (show_plots ? flights : NamedTuple[])
    sl = f.sl
    # calc_turn_rate is aligned to time[2:end], so every other signal starts at index 2 too.
    rng = 2:length(sl.time)
    turn_rate = rad2deg.(calc_turn_rate(sl; source = :heading, dt = DT))
    # The joint model on the same samples, NaN before the fit window.
    turn_rate_model = fill(NaN, length(rng))
    window = findall(>=(f.t_fit), sl.time[rng])
    length(window) == length(f.fit.time) || error("The fit window does not match the log.")
    turn_rate_model[window] .= model_rate(f.fit, joint, joint.d)
    p = plotx(
        sl.time[rng],
        (turn_rate, turn_rate_model),
        Float64.(sl.v_app[rng]),
        f.vk[rng],
        (f.el[rng], fill(MAX_ELEVATION, length(rng)));
        xlabel = L"\mathrm{time}~[\mathrm{s}]",
        ysize = 18,
        ylabels = [
            L"\dot{\psi}~[°/\mathrm{s}]",
            L"v_{\mathrm{a}}~[\mathrm{m/s}]",
            L"v_{\mathrm{k}}~[\mathrm{m/s}]",
            L"\mathrm{elevation}~[°]",
        ],
        labels = [
            [L"\dot{\psi}", @sprintf("joint model, τ = %.3f s, T = %.3f s", joint.dead_time, joint.lag)],
            nothing,
            nothing,
            ["elevation", "max_elevation"],
        ],
        fig = @sprintf("Turn-rate identification, depower %.3f, amplitude %.3f, wind %.1f m/s, reel-out %.1f m/s",
                       depower, f.a, v_wind, v_reelout),
    )
    display(p)
    sleep(0.1)  # Allow Makie to render the plot before continuing
end

nothing
