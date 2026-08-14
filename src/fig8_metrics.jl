
# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Headless quantitative metrics for figure-of-eight runs (see
`examples/simple_fig8.jl`). No plotting dependency, so this runs in headless
sweeps.

Assumes the V3 log layout: `set_steering` is dimensionless `rel_steering`, not
meters, so high-frequency content is reported in rel-steering units; the single
winch is logged in `winch_force[1]`; `var_01` carries the cross-track error
[deg] (`simple_fig8.jl` has the full slot mapping; `simple_reelout.jl` adds
`var_10`-`var_13`, used by [`reelout_power`](@ref)).
"""

"""
    fig8_metrics(sl; t_start=0.0, settle_time=10.0, settle_d_threshold=5.0, hf_window=0.5)

Compute figure-of-eight quality metrics of syslog `sl` over the settled window
(`stats_start` to the end). `settle_time` [s] is a fallback upper bound on how
long the entry transient is assumed to take; the actual `stats_start` is
`t_start` plus however long it takes the cross-track error (`var_01` [deg]) to
first drop below `settle_d_threshold` degrees, capped at `t_start + settle_time`
so a run that never converges still gets scored from a bounded window rather
than an empty one; the elapsed value is returned as `settle_time_used`.
`hf_window` [s] is the moving-average width subtracted out to isolate
high-frequency content. Returns `nothing` if the window is empty.

The elevation floor is reported both over the settled window and over the
**whole run** (`min_elevation_all`) — the latter is the success criterion,
because a floor breach during the entry transient still counts as a breach.

`az_amplitude`/`el_height` are the REFERENCE path's own `A` and `B` [deg]
(azimuth spans `±A`, elevation spans `B` peak to peak). Pass them and the metrics
also report **how much of the commanded pattern was actually flown**:
`az_reach_pos`/`az_reach_neg` (mean per-lobe azimuth extreme, one value per
completed excursion to that side) and the fill fractions
`az_fill_pos`/`az_fill_neg`/`el_fill`. Without them the flown extent is still
reported, the fractions are `NaN`, and the lap band falls back to its
self-normalized form. Only the reach distinguishes "tracking the pattern" from
"tracking a piece of it" — cross-track error is measured to the closest point of
the path and is blind to the size of the eight.

Steering is reported on BOTH sides of the actuator, and the pair is the point:
`steering_sat_frac`/`max_steering_used` describe the command,
`tape_rate_frac`/`max_steering_delivered` what the KCU tape did. `v_steering`
[1/s] is the tape's rate limit and must match the `kcu:` section of the settings
YAML in use (0.2 for every V3 file shipped here); it is a keyword so a log from a
differently configured KCU can still be scored.

Why each guard is shaped the way it is: `docs/fig8_tuning_log.md`.
"""
function fig8_metrics(sl; t_start = 0.0, settle_time = 10.0, settle_d_threshold = 5.0,
                      hf_window = 0.5,
                      lap_frac = 0.5, min_excursion = deg2rad(5.0),
                      az_center = nothing, az_amplitude = nothing,
                      el_height = nothing, v_steering = 0.2)
    t_all = Float64.(sl.time)
    # Shrink the settle window to the first convergence below settle_d_threshold,
    # rather than always waiting out the (generously guessed) fixed settle_time.
    converge_idx = findfirst(i -> t_all[i] >= t_start &&
                                  abs(sl.var_01[i]) < settle_d_threshold,
                            eachindex(t_all))
    stats_start = converge_idx === nothing ? t_start + settle_time :
                  min(t_start + settle_time, t_all[converge_idx])
    settled = findall(>=(stats_start), t_all)
    isempty(settled) && return nothing

    d = Float64.(sl.var_01[settled])
    dt = length(t_all) > 1 ? mean(diff(t_all)) : 0.01

    fp = Float64.(getindex.(sl.winch_force[settled], 1))

    # Recomputed from the logged headings, robust to a stale heading_rate.
    heading_rate_deg =
        [0.0; rad2deg.(wrap2pi.(diff(Float64.(sl.heading)))) ./ diff(t_all)]
    psi_dot_settled = heading_rate_deg[settled]

    """
        hf_std(x)

    Std of `x` after removing a centered moving average `hf_window` [s] wide,
    which isolates ringing and chatter from the commanded maneuver.
    """
    function hf_std(x)
        n = max(1, round(Int, hf_window / dt))
        half = n ÷ 2
        ma = similar(x)
        for i in eachindex(x)
            lo = max(1, i - half)
            hi = min(length(x), i + half)
            ma[i] = mean(@view x[lo:hi])
        end
        return std(x .- ma)
    end

    steering_hf = hf_std(Float64.(sl.set_steering[settled]))
    turnrate_hf = hf_std(psi_dot_settled)

    az = Float64.(sl.azimuth[settled])
    # The PATTERN's centre when known; the flown mean scores laps a circling kite never flew.
    az_c = az_center === nothing ? mean(az) : az_center
    half_range = maximum(abs.(az .- az_c))
    # Excursion band a crossing must clear; the absolute floor is what stops phantom laps.
    band = max(lap_frac * (az_amplitude === nothing ? half_range :
                           deg2rad(az_amplitude)),
               min_excursion)
    crossings = 0
    side = 0
    reach_pos = Float64[]
    reach_neg = Float64[]
    peak = 0.0                     # signed extreme of the current excursion
    for a in az
        off = a - az_c
        if off > band && side <= 0
            side < 0 && push!(reach_neg, -peak)
            side = 1
            crossings += 1
            peak = off
        elseif off < -band && side >= 0
            side > 0 && push!(reach_pos, peak)
            side = -1
            crossings += 1
            peak = off
        elseif side > 0
            peak = max(peak, off)
        elseif side < 0
            peak = min(peak, off)
        end
    end
    laps = max(0, crossings - 1) / 2   # the first arrival is not yet a crossing

    # Mean per-lobe extreme [deg]; `*_worst` is the weakest lobe flown.
    az_reach_pos = isempty(reach_pos) ? rad2deg(max(maximum(az) - az_c, 0.0)) :
                   rad2deg(mean(reach_pos))
    az_reach_neg = isempty(reach_neg) ? rad2deg(max(az_c - minimum(az), 0.0)) :
                   rad2deg(mean(reach_neg))
    az_reach_pos_worst = isempty(reach_pos) ? az_reach_pos : rad2deg(minimum(reach_pos))
    az_reach_neg_worst = isempty(reach_neg) ? az_reach_neg : rad2deg(minimum(reach_neg))
    el = Float64.(sl.elevation[settled])
    el_span = rad2deg(maximum(el) - minimum(el))
    az_fill_pos = az_amplitude === nothing ? NaN : az_reach_pos / az_amplitude
    az_fill_neg = az_amplitude === nothing ? NaN : az_reach_neg / az_amplitude
    el_fill = el_height === nothing ? NaN : el_span / el_height

    # Proxy for the clamp, which is not logged: within 2% of the largest command.
    us = abs.(Float64.(sl.set_steering[settled]))
    us_max = maximum(us)
    sat_frac = us_max > 0 ? count(>=(0.98 * us_max), us) / length(us) : 0.0

    # TAPE rate saturation: `sat_frac` alone cannot tell authority from RATE limiting.
    act = Float64.(sl.steering[settled])
    tape_rate = length(act) > 1 ? abs.(diff(act)) ./ dt : Float64[]
    tape_rate_frac = isempty(tape_rate) ? 0.0 :
        count(>=(0.95 * v_steering), tape_rate) / length(tape_rate)
    max_tape_rate = isempty(tape_rate) ? 0.0 : maximum(tape_rate)
    max_steering_delivered = maximum(abs.(act))

    return (;
        stats_start,
        settle_time_used = stats_start - t_start,
        laps,
        az_reach_pos,
        az_reach_neg,
        az_reach_pos_worst,
        az_reach_neg_worst,
        az_fill_pos,
        az_fill_neg,
        el_span,
        el_fill,
        az_amplitude,
        el_height,
        rms_d = sqrt(mean(d .^ 2)),
        mean_d = mean(d),
        max_d = maximum(d),
        min_elevation_settled = rad2deg(minimum(sl.elevation[settled])),
        min_elevation_all = rad2deg(minimum(sl.elevation)),
        mean_force = mean(fp),
        std_force = std(fp),
        cv_force = std(fp) / mean(fp),
        max_turn_rate = maximum(abs.(psi_dot_settled)),
        steering_hf_std = steering_hf,
        turnrate_hf_std = turnrate_hf,
        max_steering_used = us_max,
        steering_sat_frac = sat_frac,
        max_steering_delivered,
        tape_rate_frac,
        max_tape_rate,
        v_steering,
    )
end

"""
    reelout_power(sl) -> NamedTuple or `nothing`

Mean mechanical reel-out power `mean(winch_force * v_reelout)` [W] over the
window where the tether length setpoint (`var_10`, `examples/simple_reelout.jl`
only) was still increasing — the run's actual reel-out phase, between the
pattern settling and `reelout_l_max`. Returns `nothing` for a log that never
reeled out: a plain figure-eight run (`var_10` is unused there, so constant),
or a reel-out run that never reached phase 4.

Headless, no plotting dependency, so a sweep can call it on every log.
"""
function reelout_power(sl)
    l_set = Float64.(sl.var_10)
    length(l_set) > 1 || return nothing
    # +1: `diff` reports the change INTO sample i+1, which is the reeling one.
    reeling = findall(>(0.0), diff(l_set)) .+ 1
    isempty(reeling) && return nothing
    fp = Float64.(getindex.(sl.winch_force[reeling], 1))
    v_ro = Float64.(getindex.(sl.v_reelout[reeling], 1))
    return (; mean_power = mean(fp .* v_ro), n = length(reeling))
end

"""
    print_fig8_metrics(sl; kwargs...)

[`fig8_metrics`](@ref) plus a human-readable summary and a pass/fail line
against V3Kite.jl's success criteria. Returns the metrics NamedTuple with
the verdict merged in (`criteria`, the number checked, and `criteria_failed`,
the names of those that failed), or `nothing`, with a `@warn`, if unavailable.

Pass `az_amplitude`/`el_height` (the path's `A` and `B` [deg]) to also check the
pattern's SIZE: the tracking criteria are all relative to the closest point of
the path and are therefore blind to a kite flying a small eight, or one that
sits in half the wind window — both are close to the path at every instant. The
extra criteria require the mean per-lobe azimuth reach on BOTH sides, and the
elevation span, to be at least `min_span_frac` of what was commanded. They are
skipped (and the pass count drops accordingly) when the geometry is not given.
"""
function print_fig8_metrics(sl; t_start = 0.0, settle_time = 10.0,
                            settle_d_threshold = 5.0,
                            hf_window = 0.5, min_elevation = 10.0,
                            max_rms_d = 3.0, max_d_limit = 8.0, min_laps = 2.5,
                            lap_frac = 0.5, min_excursion = deg2rad(5.0),
                            az_center = nothing, az_amplitude = nothing,
                            el_height = nothing, min_span_frac = 0.7,
                            v_steering = 0.2)
    m = fig8_metrics(sl; t_start, settle_time, settle_d_threshold, hf_window,
                     lap_frac, min_excursion,
                     az_center, az_amplitude, el_height, v_steering)
    if m === nothing
        @warn "No settled samples — no figure-eight metrics."
        return nothing
    end
    @printf("Fig8 (settled from t=%.1fs, settle_time=%.1fs): laps=%.1f | RMS d=%.2f° mean=%.2f° max=%.2f°\n",
            m.stats_start, m.settle_time_used, m.laps, m.rms_d, m.mean_d, m.max_d)
    @printf("  elevation: min settled=%.1f° min WHOLE RUN=%.1f° | peak ψ̇=%.0f°/s\n",
            m.min_elevation_settled, m.min_elevation_all, m.max_turn_rate)
    # Separate from the tracking line: low RMS d says ON the path, this says how much of it.
    if az_amplitude === nothing && el_height === nothing
        @printf("  extent: azimuth -%.1f°..+%.1f° (worst lobe -%.1f°/+%.1f°), elevation span %.1f° — no reference geometry given, NOT checked\n",
                m.az_reach_neg, m.az_reach_pos,
                m.az_reach_neg_worst, m.az_reach_pos_worst, m.el_span)
    else
        @printf("  extent: azimuth -%.1f°..+%.1f° (%.0f%%/%.0f%% of ±A, worst lobe -%.1f°/+%.1f°), elevation span %.1f° (%.0f%% of B)\n",
                m.az_reach_neg, m.az_reach_pos,
                100 * m.az_fill_neg, 100 * m.az_fill_pos,
                m.az_reach_neg_worst, m.az_reach_pos_worst,
                m.el_span, 100 * m.el_fill)
    end
    @printf("  tether force: mean=%.0fN std=%.0fN (CV=%.1f%%)\n",
            m.mean_force, m.std_force, 100 * m.cv_force)
    @printf("  steering: peak |u_s|=%.3f, %.0f%% of time within 2%% of it | HF std: steering=%.4f turnrate=%.2f°/s\n",
            m.max_steering_used, 100 * m.steering_sat_frac,
            m.steering_hf_std, m.turnrate_hf_std)
    @printf("  tape: delivered peak |u_s|=%.3f of %.3f commanded | RATE-limited %.0f%% of the time (peak %.3f/s of %.3f/s)\n",
            m.max_steering_delivered, m.max_steering_used,
            100 * m.tape_rate_frac, m.max_tape_rate, m.v_steering)

    checks = [
        ("laps >= $(min_laps)",            m.laps >= min_laps),
        ("RMS d < $(max_rms_d)°",          m.rms_d < max_rms_d),
        ("max d < $(max_d_limit)°",        m.max_d < max_d_limit),
        ("min elevation > $(min_elevation)° (whole run)",
                                           m.min_elevation_all > min_elevation),
    ]
    # Both sides separately: one "span" check passes on a kite that never crosses over.
    if az_amplitude !== nothing
        push!(checks,
              ("azimuth reach +$(round(min_span_frac * az_amplitude, digits=1))°",
               m.az_reach_pos >= min_span_frac * az_amplitude),
              ("azimuth reach -$(round(min_span_frac * az_amplitude, digits=1))°",
               m.az_reach_neg >= min_span_frac * az_amplitude))
    end
    if el_height !== nothing
        push!(checks,
              ("elevation span > $(round(min_span_frac * el_height, digits=1))°",
               m.el_span >= min_span_frac * el_height))
    end
    failed = [name for (name, ok) in checks if !ok]
    if isempty(failed)
        @info "Success criteria: all $(length(checks)) passed."
    else
        @warn "Success criteria FAILED: " * join(failed, ", ")
    end
    # Returned as well as printed, so a headless sweep need not scrape the log line.
    return merge(m, (; criteria = length(checks), criteria_failed = failed))
end
