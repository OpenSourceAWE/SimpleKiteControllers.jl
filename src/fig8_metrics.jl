
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
(azimuth spans `±A`, elevation spans `B` peak to peak), and `az_center` its
centre [rad]. Pass them and the metrics also report **how much of the commanded
pattern was actually flown**: `az_reach_pos`/`az_reach_neg` (mean per-lobe
azimuth extreme, one value per completed excursion to that side) and the fill
fractions `az_fill_pos`/`az_fill_neg`/`el_fill`. Without them the flown extent is
still reported, the fractions are `NaN`, and the lap band falls back to its
self-normalized form.

Each of the three may be a single number or **one number per log sample**, for a
run whose reference changes under it — a re-optimized reel-out shrinks its own
pattern by a third in azimuth, so one fixed `A` scores the last laps against a
reach nothing ever asked them to fly. Per-sample geometry is used where each
quantity is measured: every lobe extreme against the `A` commanded at the sample
it was reached, every excursion's elevation span against the mean `B` commanded
across it, and the lap-detection band against the `A` in force at each sample. A
constant geometry passed per sample is the scalar case exactly.

`el_fill` is measured on the span of each excursion (`el_span_lap`), not on
`el_span`, which is the span over the whole settled window: the window takes its
top from one lap and its bottom from another, so a pattern that descends as the
tether grows reads as TALLER than anything that was commanded, and a kite that
misses the top of one lobe is covered by the other. `el_span` is still reported,
since the two together say whether the pattern moved. Only the reach distinguishes "tracking the pattern" from
"tracking a piece of it" — cross-track error is measured to the closest point of
the path and is blind to the size of the eight.

Steering is reported on BOTH sides of the actuator, and the pair is the point:
`steering_sat_frac`/`max_steering_used` describe the command,
`tape_rate_frac`/`max_steering_delivered` what the KCU tape did. `v_steering`
[1/s] is the tape's rate limit and must match the `kcu:` section of the settings
YAML in use (0.2 for every V3 file shipped here); it is a keyword so a log from a
differently configured KCU can still be scored.

`heading_range` [deg] is the span of the UNWRAPPED heading over the settled
window (`max - min`, unwrap by `wrap2pi`'d steps). An oscillating figure-eight
stays bounded here regardless of lap count, since each lobe swings back into
the same band; it grows without bound the moment the guidance's reference
loses its branch and the kite spins an extra loop instead of turning back —
invisible to `rms_d`/`max_d`, which stay small because the guidance tracks
whatever shape it is given correctly, and to the extent/lap checks, which are
blind to how many times the heading wound around to fly it.

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

    # Unwrapped heading span over the settled window: an oscillating
    # figure-eight stays bounded here (~330° measured on every clean run,
    # regardless of lap count, since each lobe swings back into the same
    # band) but grows without bound the moment Q loses its branch and the
    # kite spins an extra loop instead of turning back — invisible to every
    # other check here, since cross-track error stays small throughout (the
    # guidance tracks whatever shape it is given correctly; the shape itself
    # is what went wrong). See docs/fig8_tuning_log.md, 2026-08-20.
    psi_settled = Float64.(sl.heading[settled])
    psi_unwrapped = first(psi_settled) .+
                    cumsum(vcat(0.0, wrap2pi.(diff(psi_settled))))
    heading_range = rad2deg(maximum(psi_unwrapped) - minimum(psi_unwrapped))

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
    # The reference geometry may be one number or one number per LOG SAMPLE: with
    # re-optimization the commanded pattern shrinks by a third over a run, and a
    # fill fraction against the pattern of lap 1 scores the last laps against a
    # reach they were never asked to fly.
    ref(x, name) = x === nothing ? nothing :
        x isa AbstractVector ?
            (length(x) == length(t_all) ? Float64.(x[settled]) :
             throw(ArgumentError("$name has $(length(x)) entries but the log has \
                                  $(length(t_all)) samples; pass one per sample \
                                  or a single number"))) :
            fill(Float64(x), length(settled))
    az_c_s = ref(az_center, "az_center")
    amp_s = ref(az_amplitude, "az_amplitude")
    hgt_s = ref(el_height, "el_height")
    # The PATTERN's centre when known; the flown mean scores laps a circling kite never flew.
    az_c_s === nothing && (az_c_s = fill(mean(az), length(az)))
    half_range = maximum(abs.(az .- az_c_s))
    crossings = 0
    side = 0
    reach_pos = Tuple{Float64, Int}[]   # (extreme [rad], sample it was reached at)
    reach_neg = Tuple{Float64, Int}[]
    peak = 0.0                     # signed extreme of the current excursion
    peak_i = 1
    for (i, a) in enumerate(az)
        off = a - az_c_s[i]
        # Excursion band a crossing must clear; the absolute floor is what stops
        # phantom laps, and it follows the pattern the kite is being given.
        band = max(lap_frac * (amp_s === nothing ? half_range : deg2rad(amp_s[i])),
                   min_excursion)
        if off > band && side <= 0
            side < 0 && push!(reach_neg, (-peak, peak_i))
            side = 1
            crossings += 1
            peak, peak_i = off, i
        elseif off < -band && side >= 0
            side > 0 && push!(reach_pos, (peak, peak_i))
            side = -1
            crossings += 1
            peak, peak_i = off, i
        elseif side > 0
            off > peak && ((peak, peak_i) = (off, i))
        elseif side < 0
            off < peak && ((peak, peak_i) = (off, i))
        end
    end
    laps = max(0, crossings - 1) / 2   # the first arrival is not yet a crossing

    # Mean per-lobe extreme [deg]; `*_worst` is the weakest lobe flown.
    az_reach_pos = isempty(reach_pos) ? rad2deg(max(maximum(az .- az_c_s), 0.0)) :
                   rad2deg(mean(first, reach_pos))
    az_reach_neg = isempty(reach_neg) ? rad2deg(max(maximum(az_c_s .- az), 0.0)) :
                   rad2deg(mean(first, reach_neg))
    az_reach_pos_worst = isempty(reach_pos) ? az_reach_pos :
                         rad2deg(minimum(first, reach_pos))
    az_reach_neg_worst = isempty(reach_neg) ? az_reach_neg :
                         rad2deg(minimum(first, reach_neg))
    el = Float64.(sl.elevation[settled])
    el_span = rad2deg(maximum(el) - minimum(el))
    # Each lobe against the pattern commanded WHERE IT WAS FLOWN. Constant
    # geometry gives exactly `reach / amplitude` back, since the mean of a ratio
    # with a fixed denominator is the ratio of the mean.
    fill_frac(reach, fallback) = amp_s === nothing ? NaN :
        isempty(reach) ? fallback / mean(amp_s) :
        mean(rad2deg(p) / amp_s[i] for (p, i) in reach)
    az_fill_pos = fill_frac(reach_pos, az_reach_pos)
    az_fill_neg = fill_frac(reach_neg, az_reach_neg)
    # The elevation span is a WINDOW quantity, so a shrinking pattern inflates it:
    # the top comes from the first lap and the bottom from the last. Measured per
    # excursion instead whenever the geometry is given per sample.
    el_spans = Tuple{Float64, Float64}[]   # (span [deg], B commanded across it)
    # Lobe extreme to lobe extreme, which is one full up-and-down of the reference;
    # the partial stretches before the first and after the last are left out, since
    # a fraction of a lap spans a fraction of the pattern and would read as a kite
    # flying a smaller one.
    let bounds = sort!(vcat([i for (_, i) in reach_pos], [i for (_, i) in reach_neg]))
        for k in 1:(length(bounds) - 1)
            rng = bounds[k]:bounds[k + 1]
            # The mean over the segment, not its first sample: the reference
            # shrinks THROUGH the segment, and the span is a property of all of it.
            length(rng) > 1 &&
                push!(el_spans, (rad2deg(maximum(el[rng]) - minimum(el[rng])),
                                 hgt_s === nothing ? NaN : mean(@view hgt_s[rng])))
        end
    end
    el_span_lap = isempty(el_spans) ? el_span : mean(first, el_spans)
    # Per excursion, always: a window-wide span takes its top from one lap and its
    # bottom from another, so a pattern that DESCENDS reads as taller than the one
    # commanded (129 % of B on a synthetic run that flew every pattern it was
    # given), and a kite that misses the top of one lobe is covered by the other.
    el_fill = hgt_s === nothing ? NaN :
        isempty(el_spans) ? el_span / mean(hgt_s) :
        mean(sp / b for (sp, b) in el_spans)

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
        el_span_lap,
        el_fill,
        az_amplitude,
        el_height,
        rms_d = sqrt(mean(d .^ 2)),
        mean_d = mean(d),
        max_d = maximum(d),
        heading_range = heading_range,
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

Mechanical reel-out power `winch_force * v_reelout` scored over the window where
the tether length setpoint (`var_10`, `examples/simple_reelout.jl` only) was
still increasing — the run's actual reel-out phase, between the guidance
engaging and `reelout_l_max`. Returns `nothing` for a log that never reeled out:
a plain figure-eight run (`var_10` is unused there, so constant), or a reel-out
run that never got past the entry.

Fields: `mean_power` [W] and `energy` [J] over that window, its `duration` [s]
and sample count `n`, plus `energy_run` [J], the same integral over the WHOLE
log. The pair is the point — a change that starts reeling earlier trades mean
power against a longer window, and only `energy_run` says which way the run came
out. It is the honest total: it also counts what the drum gives back on the
reel-in transients outside the window, which `energy` cannot see.

Also `peak_power` [W], the window's maximum, and `cf_power_ro`, its crest factor
`peak_power / mean_power` — how peaky the reel-out power is, relevant for
sizing the generator and grid connection. `mean_force`/`peak_force`/`cf_force_ro`
[N] are the same triple for the tether force alone, over the same window.

`idx` is the sample indices making up that window, so a caller can score anything
else over exactly the same samples — `sl.time[rp.idx]` are their timestamps.

Headless, no plotting dependency, so a sweep can call it on every log.
"""
function reelout_power(sl)
    l_set = Float64.(sl.var_10)
    length(l_set) > 1 || return nothing
    # +1: `diff` reports the change INTO sample i+1, which is the reeling one.
    reeling = findall(>(0.0), diff(l_set)) .+ 1
    isempty(reeling) && return nothing
    dt = Float64(sl.time[2]) - Float64(sl.time[1])
    f_all = Float64.(getindex.(sl.winch_force, 1))
    p_all = f_all .* Float64.(getindex.(sl.v_reelout, 1))
    p = p_all[reeling]
    f = f_all[reeling]
    mean_power = mean(p)
    peak_power = maximum(p)
    mean_force = mean(f)
    peak_force = maximum(f)
    return (; mean_power, peak_power, cf_power_ro = peak_power / mean_power,
            mean_force, peak_force, cf_force_ro = peak_force / mean_force,
            energy = sum(p) * dt,
            duration = length(reeling) * dt, n = length(reeling),
            energy_run = sum(p_all) * dt, idx = reeling)
end

"""
    winch_state_pct(sl) -> NamedTuple or `nothing`

How the REEL_OUT winch controller spent the reel-out window, as the percentage
of its samples in each of WinchControllers.jl's three states (logged to `var_12`
by `examples/simple_reelout.jl`): `lower_force_pct` (state 0, the
`LowerForceController` reeling IN to keep the tether taut), `speed_pct` (state 1,
the `v_set = kv*sqrt(force)` law itself) and `upper_force_pct` (state 2, the
`UpperForceController` capping the force at `WCSettings.f_high`).

Scored over the same window as [`reelout_power`](@ref) — the samples where the
length setpoint was still growing — because `var_12` only means anything while
the controller is being stepped: before reel-out engages it still reads whatever
state the freshly built controller started in. Returns `nothing` for a log that
never reeled out, same guard as [`reelout_power`](@ref).

`upper_force_pct` is a side CONDITION rather than a quality metric, and is why
this function exists: the pattern sweep (`examples/optimize_fig8.jl`) rejects any
shape that engages the upper limiter, because a shape which pulls harder than the
winch is allowed to hold had its power bought against the force cap — the number
describes the cap, not the pattern. `lower_force_pct` is the mirror image and
worth watching for the same reason (see the `v_ff` entry in `Plan.md`, where it
going to zero was the whole power gain).
"""
function winch_state_pct(sl)
    l_set = Float64.(sl.var_10)
    length(l_set) > 1 || return nothing
    # +1: `diff` reports the change INTO sample i+1, exactly as in `reelout_power`.
    reeling = findall(>(0.0), diff(l_set)) .+ 1
    isempty(reeling) && return nothing
    # Logged as a Float32 through `var_12`; round before comparing to the enum values.
    state = round.(Int, Float64.(sl.var_12[reeling]))
    n = length(state)
    return (; lower_force_pct = 100 * count(==(0), state) / n,
            speed_pct = 100 * count(==(1), state) / n,
            upper_force_pct = 100 * count(==(2), state) / n,
            n)
end

"""
    _longest_run(idx) -> AbstractVector{Int}

Longest run of consecutive integers in the sorted vector `idx`. Used to drop a
one-sample glitch (`var_10` logs 0 for the first couple of steps) from the
reel-out window before it is mistaken for the start of the ring.
"""
function _longest_run(idx)
    best_start = 1
    best_len = 1
    start = 1
    for i in 2:length(idx)
        if idx[i] != idx[i - 1] + 1
            len = i - start
            len > best_len && ((best_len, best_start) = (len, start))
            start = i
        end
    end
    len = length(idx) - start + 1
    len > best_len && ((best_len, best_start) = (len, start))
    return idx[best_start:(best_start + best_len - 1)]
end

"""
    reelout_ringing(sl; detrend_window=2.0, ring_span=20.0, min_peak_gap=1.0,
                    peak_floor=0.02, settle_frac=0.05) -> NamedTuple or `nothing`

Characterizes the underdamped ring the winch's `v_set = kv*sqrt(force)` law
excites when reel-out engages (`docs/fig8_tuning_log.md`, "REEL_OUT winch"):
`v_reelout` overshoots the speed the square-root law settles to and rings for
several cycles before decaying into it. Returns `nothing` for a log that never
reeled out, same guard as [`reelout_power`](@ref).

The ring rides on the startup ramp (force, and with it `v_reelout`, is still
rising over the same seconds), so peaks are found on the RESIDUAL of
`v_reelout` after subtracting a centered moving average `detrend_window` [s]
wide — the ramp, not the ring, dominates the raw signal's own peak spacing.
Only residual maxima above `peak_floor` [m/s] and at least `min_peak_gap` [s]
apart, within `ring_span` [s] of reel-out starting, count as ring peaks; this
excludes the slower, much smaller speed variation the flown pattern itself
imposes once the ring has died out.

Fields: `n_peaks`; `period_s`, the mean peak-to-peak spacing; `zeta`, the
damping ratio from the log decrement of successive peak amplitudes; `overshoot_m_s`,
the first peak's amplitude above the local trend; `duration_s`, the time from
reel-out start until the residual last exceeds `settle_frac` of `overshoot_m_s`;
`peak_v_reelout_m_s`, the raw (not detrended) speed maximum within `ring_span`;
and `steady_v_reelout_m_s`, the mean `v_reelout` after `ring_span` for
comparison. `period_s` and `zeta` are `NaN` when fewer than 2 (respectively no
decaying pair of) peaks are found.
"""
function reelout_ringing(sl; detrend_window = 2.0, ring_span = 20.0,
                         min_peak_gap = 1.0, peak_floor = 0.02, settle_frac = 0.05)
    l_set = Float64.(sl.var_10)
    length(l_set) > 1 || return nothing
    reeling = findall(>(0.0), diff(l_set)) .+ 1
    isempty(reeling) && return nothing
    reeling = _longest_run(reeling)
    t_all = Float64.(sl.time)
    dt = t_all[2] - t_all[1]
    v_ro = Float64.(getindex.(sl.v_reelout, 1))[reeling]
    t_rel = t_all[reeling] .- t_all[reeling[1]]

    head = findall(<=(ring_span), t_rel)
    n = max(1, round(Int, detrend_window / dt))
    half = n ÷ 2
    v_head = v_ro[head]
    resid = similar(v_head)
    for i in eachindex(v_head)
        lo = max(1, i - half)
        hi = min(length(v_head), i + half)
        resid[i] = v_head[i] - mean(@view v_head[lo:hi])
    end

    peaks = Int[]
    for i in 2:(length(resid) - 1)
        resid[i] > resid[i - 1] && resid[i] >= resid[i + 1] && resid[i] > peak_floor || continue
        if isempty(peaks) || t_rel[head[i]] - t_rel[head[peaks[end]]] >= min_peak_gap
            push!(peaks, i)
        elseif resid[i] > resid[peaks[end]]
            peaks[end] = i
        end
    end

    steady_idx = findall(>=(ring_span), t_rel)
    steady_v_reelout_m_s = isempty(steady_idx) ? NaN : mean(v_ro[steady_idx])
    n_peaks = length(peaks)
    n_peaks == 0 && return (; n_peaks, period_s = NaN, zeta = NaN, overshoot_m_s = NaN,
                            duration_s = 0.0, peak_v_reelout_m_s = maximum(v_head),
                            steady_v_reelout_m_s)

    peak_amps = resid[peaks]
    peak_times = t_rel[head[peaks]]
    period_s = n_peaks >= 2 ? mean(diff(peak_times)) : NaN
    decrements = [log(peak_amps[i] / peak_amps[i + 1]) for i in 1:(n_peaks - 1)
                  if peak_amps[i] > 0 && peak_amps[i + 1] > 0]
    delta = isempty(decrements) ? NaN : mean(decrements)
    zeta = isnan(delta) ? NaN : delta / sqrt((2pi)^2 + delta^2)

    thresh = settle_frac * peak_amps[1]
    settle_idx = findlast(x -> abs(x) >= thresh, resid)
    duration_s = settle_idx === nothing ? 0.0 : t_rel[head[settle_idx]]

    return (; n_peaks, period_s, zeta, overshoot_m_s = peak_amps[1], duration_s,
            peak_v_reelout_m_s = maximum(v_head), steady_v_reelout_m_s)
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
They are scored on the fill fractions, so a geometry passed per log sample (see
[`fig8_metrics`](@ref)) is checked lap by lap against the pattern in force at the
time; the degrees in the printed criterion names are then the mean of it.

`max_heading_range` (default `400.0`°) bounds `heading_range` (see
[`fig8_metrics`](@ref)): every clean run measured stays near 330°, so 400°
leaves margin above that baseline while still catching even a single mild
extra-loop episode (measured +150-450° over baseline) well before it could be
mistaken for normal lap-to-lap variation.

Pass `require_final = true` (default `false`) to also check that `sys_state`
reaches `5` (phase "final") somewhere in the log — `examples/simple_reelout.jl`
only, whose entry state machine has that phase; a plain `simple_fig8.jl` run's
`sys_state` never goes past `4` and would fail this check every time, hence the
default off. Checks that reel-out actually FINISHED within the run — by either
of its two stop criteria, `l_set` reaching `reelout_l_max` or `n_fig_eight` laps
completed — not just that the pattern was tracked well.
"""
function print_fig8_metrics(sl; t_start = 0.0, settle_time = 10.0,
                            settle_d_threshold = 5.0,
                            hf_window = 0.5, min_elevation = 10.0,
                            max_rms_d = 3.0, max_d_limit = 8.0, min_laps = 2.5,
                            max_heading_range = 400.0,
                            lap_frac = 0.5, min_excursion = deg2rad(5.0),
                            az_center = nothing, az_amplitude = nothing,
                            el_height = nothing, min_span_frac = 0.7,
                            v_steering = 0.2, require_final::Bool = false)
    m = fig8_metrics(sl; t_start, settle_time, settle_d_threshold, hf_window,
                     lap_frac, min_excursion,
                     az_center, az_amplitude, el_height, v_steering)
    if m === nothing
        @warn "No settled samples — no figure-eight metrics."
        return nothing
    end
    @printf("Fig8 (settled from t=%.1fs, settle_time=%.1fs): laps=%.1f | RMS d=%.2f° mean=%.2f° max=%.2f°\n",
            m.stats_start, m.settle_time_used, m.laps, m.rms_d, m.mean_d, m.max_d)
    @printf("  elevation: min settled=%.1f° min WHOLE RUN=%.1f° | peak ψ̇=%.0f°/s | heading range=%.0f°\n",
            m.min_elevation_settled, m.min_elevation_all, m.max_turn_rate, m.heading_range)
    # Separate from the tracking line: low RMS d says ON the path, this says how much of it.
    if az_amplitude === nothing && el_height === nothing
        @printf("  extent: azimuth -%.1f°..+%.1f° (worst lobe -%.1f°/+%.1f°), elevation span %.1f° — no reference geometry given, NOT checked\n",
                m.az_reach_neg, m.az_reach_pos,
                m.az_reach_neg_worst, m.az_reach_pos_worst, m.el_span)
    else
        # The per-lap span, which is what the fill fraction beside it is measured on.
        el_shown, el_note = (m.el_span_lap, " per lap")
        @printf("  extent: azimuth -%.1f°..+%.1f° (%.0f%%/%.0f%% of ±A, worst lobe -%.1f°/+%.1f°), elevation span %.1f°%s (%.0f%% of B)\n",
                m.az_reach_neg, m.az_reach_pos,
                100 * m.az_fill_neg, 100 * m.az_fill_pos,
                m.az_reach_neg_worst, m.az_reach_pos_worst,
                el_shown, el_note, 100 * m.el_fill)
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
        ("heading range < $(max_heading_range)°", m.heading_range < max_heading_range),
    ]
    # Both sides separately: one "span" check passes on a kite that never crosses over.
    # Scored on the FILL fractions, which is the same test as comparing degrees
    # against `min_span_frac * A` for a fixed pattern and the only one that means
    # anything for a pattern that changes size under the run. The thresholds
    # printed are then the mean commanded geometry, for reading only.
    mean_ref(x) = x isa AbstractVector ? mean(Float64.(x)) : Float64(x)
    if az_amplitude !== nothing
        thr = round(min_span_frac * mean_ref(az_amplitude), digits = 1)
        push!(checks,
              ("azimuth reach +$(thr)°", m.az_fill_pos >= min_span_frac),
              ("azimuth reach -$(thr)°", m.az_fill_neg >= min_span_frac))
    end
    if el_height !== nothing
        push!(checks,
              ("elevation span > $(round(min_span_frac * mean_ref(el_height), digits=1))°",
               m.el_fill >= min_span_frac))
    end
    if require_final
        push!(checks, ("phase 5 (final) reached", any(==(5), Int.(sl.sys_state))))
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
