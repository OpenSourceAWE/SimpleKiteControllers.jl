# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Identify the turn-rate law at a LOW elevation, from several relay flights at one
depower, and plot each flight: turn rate, apparent wind speed, kite speed and
elevation over time.

The first step of `PlanIdentifyTurnRateLaw.md`. The table's sweeps start at 73°
and relay about heading 0 (straight up), so the kite hovers near the zenith at
`v_a` ≈ 11 – 16 m/s. Here every flight relays about a crosswind heading (`±90°`),
reverses at `±az_reverse` of azimuth and tilts the band to hold `el_hold`, see
`_run_turn_rate_sweep`: the kite flies a lazy-eight-like pattern low in the wind
window, at `v_a` ≈ 20 – 50 m/s (depower 0.275, 2026-09-29).

One flight per entry of `flight_settings`, each at a fixed amplitude for
`SWEEP_SIM_TIME`. Each is fitted on its own (`identify_turn_rate_law`, then
`fit_delay_lag`), and all steady ones together (`joint_delay_lag_fit`; all of them
if none flew the full time): one dead time, lag,
`c1` and `c2` for every flight, the steering of each flight filtered and shifted
separately so no shift crosses from one flight into the next. The fit window of
a flight starts at the first sample below `max_elevation` after `T_START` and
runs to its end, contiguous, as the backward-difference turn rate and the delay
search need. The law fitted is still the current one; the result is compared
with the table's 73° row at the same depower. Nothing is written to the table.

The turn-rate panel shows the measured rate, `calc_turn_rate(sl; source =
:heading)`, and the joint model, which starts where the flight's fit window does.
The elevation panel carries `max_elevation`.

About two to three minutes per flight. The inputs `depower`, `v_wind` (the wind speed,
default the table's `V_WIND`) and `v_reelout` (the reel-out speed, default 0) are
passed with `run_example` (`examples/script_inputs.jl`):

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
using DelimitedFiles: writedlm

# `_run_turn_rate_sweep`, the fixed sweep conditions, `_split_delay`, `lag_filter` and
# `joint_delay_lag_fit`.
include(joinpath(@__DIR__, "build_turn_rate_table.jl"))
include(joinpath(@__DIR__, "script_inputs.jl"))

# ==================== USER PARAMETERS ==================== #

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
# `v_wind` [m/s]: wind of the flights; V_WIND = 9.51 is the table's. At 6.5 the pattern flies
# v_a 13 – 36 m/s, 39 % of it below 20 m/s where V3 found the law too fast; at 5.0 it drifts out
# of the window (2026-09-29). `v_reelout` [m/s]: reel-out speed of the flights from T_START on, up
# to REELOUT_L_MAX; 0 holds the length. `show_plots`, `fit_laws`: plots, and the comparison of the
# current law with the extended law (~1 – 2 min); `plot_turn_rate_vs_depower.jl` switches both
# off. `windows_dir`: where the fit windows are saved, see below.
(; depower, v_wind, v_reelout, show_plots, fit_laws, windows_dir) =
    script_inputs(@__FILE__, (; depower = 0.275, v_wind = V_WIND, v_reelout = 0.0,
                               show_plots = true, fit_laws = true, windows_dir = nothing))
depower, v_wind, v_reelout = Float64(depower), Float64(v_wind), Float64(v_reelout)
# One flight per fixed steering amplitude `a` [-], each with its own azimuth of reversal
# `az_reverse` [°] and tilt limit of the elevation hold `el_hold_tilt` [°], see
# `_run_turn_rate_sweep`. The range that flies steadily at depower 0.275 (2026-09-29): 0.05
# turns too weakly and drifts to the edge of the wind window even reversing at ±10°; 0.15
# turns ~90 °/s against a tape that needs ~1 s to swing, so the relay overshoots its band
# past heading 180° and loops into the ground.
flight_settings = [(a = 0.075, az_reverse = 20.0, el_hold_tilt = 45.0),
                   (a = 0.100, az_reverse = 30.0, el_hold_tilt = 45.0),
                   (a = 0.125, az_reverse = 30.0, el_hold_tilt = 25.0)]
start_elevation = 30.0   # [°] elevation the flights start at; the table's sweeps start at 73°
heading_center = 90.0    # [°] centre of the relay's heading band, crosswind; 0 climbs back to ~70°
el_hold = 30.0           # [°] elevation the band's tilt holds the kite near
elevation_floor = 10.0   # [°] a flight stops below this
max_elevation = 55.0     # [°] a flight's fit window starts at its first sample below this

# ==================== JOINT FIT ========================== #

# `joint_delay_lag_fit` is in `delay_lag_fit.jl`, so `plot_turn_rate_vs_depower.jl` can refit
# saved fit windows without flying.

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
and with `k2 = 1` (`PlanIdentifyTurnRateLaw.md`),

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
    cat(field) = reduce(vcat, [trim(getfield(x, field)) for x in data])
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

# The table's row for this cell, if any: the 73° result to compare with.
row = let entries = YAML.load_file(joinpath(skc_data_path(), OUT_FILE))["entries"]
    k = findfirst(e -> _entry_key(e) == (BODY_START_DAMPING, depower), entries)
    isnothing(k) ? nothing : entries[k]
end
isnothing(row) && @warn "No row for depower $depower in $OUT_FILE: nothing to compare with."

flights = NamedTuple[]
for (; a, az_reverse, el_hold_tilt) in flight_settings
    r = _run_turn_rate_sweep(depower; max_steering_cap = 1.0, elevation_floor, v_wind, v_reelout,
                             elevation = start_elevation, heading_center,
                             start_steering = a, steering_step = 0.0, az_reverse, el_hold,
                             el_hold_tilt)
    # The logger is preallocated for SWEEP_SIM_TIME; a flight that ends early leaves the rest
    # of the rows at zero (time 0 included), which folds the time axis back onto itself.
    sl = r.sl[1:findlast(>(0), r.sl.time)]
    el = rad2deg.(sl.elevation)
    k_below = findfirst(i -> sl.time[i] >= T_START && el[i] < max_elevation, eachindex(sl.time))
    if isnothing(k_below)
        @warn @sprintf("Amplitude %.3f: never below max_elevation = %.1f° after T_START; skipped.",
                       a, max_elevation)
        continue
    end
    t_fit = sl.time[k_below]
    in_window = sl.time .>= t_fit
    frac_above = count(in_window .& (el .> max_elevation)) / count(in_window)
    frac_above > 0 &&
        @warn @sprintf("Amplitude %.3f: %.0f %% of the fit window above max_elevation = %.1f°.",
                       a, 100 * frac_above, max_elevation)
    fit = merge(identify_turn_rate_law(sl; dt = DT, t_start = t_fit, min_steering = MIN_STEERING_FIT),
                (; c3 = nothing))
    vk = norm.(sl.vel_kite)
    v_tau = sqrt.(max.(vk .^ 2 .- Float64.(first.(sl.v_reelout)) .^ 2, 0.0))
    push!(flights, (; a, outcome = r.outcome, sl, el, vk, t_fit, fit, dl = _split_delay(fit),
                    v_ratio = v_tau[in_window] ./ Float64.(sl.v_app[in_window])))
    @info @sprintf("Amplitude %.3f: %s after %.0f s, fit window from %.1f s.", a, r.outcome,
                   last(sl.time), t_fit)
end
isempty(flights) && error("No flight reached max_elevation; nothing to fit.")
# The steady flights, if any: a flight that sank to the floor is a transient with a short window.
joint_flights = let steady = filter(f -> f.outcome == :time_limit, flights)
    isempty(steady) ? flights : steady
end
joint = joint_delay_lag_fit([f.fit for f in joint_flights], DT)

# The fit windows of the steady flights, one row per sample, for refitting without flying
# (`plot_turn_rate_vs_depower.jl`, `from_raw`). Only when the caller passes `windows_dir`.
if !isnothing(windows_dir)
    mkpath(windows_dir)
    let file = joinpath(windows_dir, @sprintf("depower_%.3f.csv", depower))
        open(file, "w") do io
            writedlm(io, permutedims(["flight", "amplitude", "time", "us", "rate", "v_app", "psi", "beta"]), ',')
            for (i, f) in enumerate(joint_flights)
                n = length(f.fit.time)
                writedlm(io, hcat(fill(i, n), fill(f.a, n), f.fit.time, f.fit.us, f.fit.rate,
                                  f.fit.v_app, f.fit.psi, f.fit.beta), ',')
            end
        end
        @info "Saved the fit windows to $file."
    end
end

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
    @printf("table row, 73°         %7.4f %9.3f %8.3f %8.3f %8s %9s  v_a %.1f m/s (c1, c2 of the pure-delay fit)\n",
            row["c1"], row["c2"], get(row, "dead_time", NaN), get(row, "kite_lag", NaN), "", "",
            row["v_app"])

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
        (f.el[rng], fill(max_elevation, length(rng)));
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
