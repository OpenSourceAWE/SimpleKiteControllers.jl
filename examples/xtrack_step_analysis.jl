# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Cross-track step test of the guided course loop: the measured response of the signed
# cross-track error d to an attractor offset δ (`XTRACK_OFFSET` in simple_opt_reelout.jl),
# against the model T = (1 - 1/G)·L/(1 + L) of stability_opt_reelout.jl.
# WORK IN PROGRESS, see docs/course_loop_stability_reelout.md, "Cross-track step test".
#
# Measured part, from the saved 600 s tests at 200 m (no simulation needed):
#     include("examples/xtrack_step_analysis.jl")
#     x = load_xtrack_csv("data/steptest/xtrack_step_test_200m_600s_ff0.csv")   # or ..._ff07.csv
#     keep = x.t .- x.t[1] .>= 10                     # the reference once the offset test has started
#     sr = step_responses(x.t, x.δ, subtract_by_position(x.d, x.q, x.d_ref[keep], x.q_ref[keep]))
#     y = vec(mean(reduce(hcat, sr.resp); dims = 2))
#     fit_second_order_gain(sr.τ, y)                  # K, f_d, ζ, delay: 0.66, 0.208 Hz, 0.25, 0.6 s
#
# Model part: needs the globals of stability_opt_reelout.jl (fcs, Ts, course_pid, turn_rate_plant,
# C1_SETPOINT, DP_LO, DP_HI, V_MIN_PATTERN) plus `log_delay` and `guidance_rate`. On a run that
# reels out only to 200 m that script stops at the dead-time identification (phase 4 < 20 s);
# identify the dead time on the held phase-5 window instead and define the two helpers by hand.

"""
Columns of a saved cross-track step test: time, δ, d of the step run and of the δ = 0 twin
[s, deg], and, when saved, the closest-point indices `q`, `q_ref` of both runs (else empty).
"""
function load_xtrack_csv(file)
    rows = [parse.(Float64, split(l, ',')) for l in eachline(file) if !startswith(l, '#') && !startswith(l, "time")]
    q = length(rows[1]) >= 6 ? round.(Int, getindex.(rows, 5)) : Int[]
    q_ref = length(rows[1]) >= 6 ? round.(Int, getindex.(rows, 6)) : Int[]
    return (t = getindex.(rows, 1), δ = getindex.(rows, 2), d = getindex.(rows, 3), d_ref = getindex.(rows, 4),
            q, q_ref)
end

"""
    subtract_by_position(d, q, d_ref, q_ref; smooth = 2) -> Vector

`d` minus the reference run's mean `d_ref` at the same closest point Q (averaged over
`2·smooth + 1` neighbouring points, cyclic). With the length held and the path fixed the lap
forcing depends on Q only, so unlike a subtraction by time this still cancels it after the
offset has shifted the kite's timing along the path. Points the reference never visited are NaN.
"""
function subtract_by_position(d, q, d_ref, q_ref; smooth = 2)
    n = max(maximum(q), maximum(q_ref))
    sums, cnts = zeros(n), zeros(Int, n)
    for (x, k) in zip(d_ref, q_ref)
        sums[k] += x; cnts[k] += 1
    end
    f = map(1:n) do k
        ks = [mod1(k + j, n) for j in -smooth:smooth]
        c = sum(cnts[ks])
        c == 0 ? NaN : sum(sums[ks]) / c
    end
    return d .- f[q]
end

using Statistics, ControlSystemsBase

"Aligned, normalised step responses of d: (d(t_k + τ) - d(t_k)) / Δδ_k over `win` seconds."
function step_responses(t, δ, d; win = 10.0, pre = 1.0)
    ks = findall(i -> δ[i] != δ[i - 1], 2:length(δ)) .+ 1
    dt = median(diff(t))
    n = round(Int, win / dt)
    np = round(Int, pre / dt)
    resp = Vector{Vector{Float64}}()
    for k in ks
        k - np >= 1 && k + n <= length(t) || continue
        Δ = δ[k] - δ[k - 1]
        any(isnan, d[k - np:k + n]) && continue   # a Q the reference never visited
        d0 = mean(d[k - np:k - 1])
        push!(resp, (d[k:k + n] .- d0) ./ Δ)
    end
    return (τ = collect(0:n) .* dt, resp, steps = length(resp))
end

"Least-squares fit of a delayed 2nd-order step response to `y(τ)`: (f_d [Hz], ζ, delay [s], rms)."
function fit_second_order(τ, y)
    best = (Inf, 0.0, 0.0, 0.0)
    for fn in 0.05:0.005:0.6, ζ in 0.05:0.01:1.2, Td in 0.0:0.05:1.0
        ωn = 2π * fn
        yy = map(τ) do t
            s = t - Td
            s <= 0 && return 0.0
            if ζ < 1
                ωd = ωn * sqrt(1 - ζ^2)
                1 - exp(-ζ * ωn * s) * (cos(ωd * s) + ζ / sqrt(1 - ζ^2) * sin(ωd * s))
            else
                1 - exp(-ωn * s) * (1 + ωn * s)
            end
        end
        r = sqrt(mean(abs2, yy .- y))
        r < best[1] && (best = (r, fn * sqrt(max(1 - ζ^2, 0.0)), ζ, Td))
    end
    return (f_d = best[2], ζ = best[3], delay = best[4], rms = best[1])
end

"Like `fit_second_order`, with a free steady gain `K` (least squares per grid point): (K, f_d [Hz], ζ, delay [s], rms)."
function fit_second_order_gain(τ, y)
    best = (Inf, 0.0, 0.0, 0.0, 0.0)
    for fn in 0.05:0.005:0.5, ζ in 0.02:0.01:1.0, Td in 0.0:0.05:1.0
        ωn = 2π * fn
        r1 = sqrt(max(1 - ζ^2, 1e-9))
        ωd = ωn * r1
        s = [t <= Td ? 0.0 : 1 - exp(-ζ * ωn * (t - Td)) * (cos(ωd * (t - Td)) + ζ / r1 * sin(ωd * (t - Td)))
             for t in τ]
        K = sum(s .* y) / sum(abs2, s)
        r = sqrt(mean(abs2, K .* s .- y))
        r < best[1] && (best = (r, K, fn * sqrt(max(1 - ζ^2, 0.0)), ζ, Td))
    end
    return (K = best[2], f_d = best[3], ζ = best[4], delay = best[5], rms = best[1])
end

"Model T = (1 - 1/G)·L/(1 + L) from δ to d at one operating point, per gravity sign."
function model_T(Lt, v_app, v_kite, depower, el_c, lag)
    tc = turn_rate_coeffs(fcs.body_damping, clamp(depower, DP_LO, DP_HI))
    K = C1_SETPOINT / tc.c1 * fcs.heading_p * fcs.v_app_ref / max(v_app, V_MIN_PATTERN)
    C = course_pid(K, fcs.heading_i, fcs.heading_d, fcs.heading_d_n, Ts)
    ωg = guidance_rate(Lt, v_app, v_kite)
    G = 1 + ωg * Ts / (tf("z", Ts) - 1)
    τd = log_delay(v_app)
    map((-cosd(el_c), cosd(el_c))) do gravity
        P = turn_rate_plant(tc.c1, tc.c2, τd, v_app, gravity, Ts; lag)
        Lg = C * G * P
        T = minreal(feedback(Lg) * (1 - 1 / G); atol = 1e-8)
        ps = log.(complex(poles(T))) ./ Ts               # continuous equivalents
        osc = filter(p -> imag(p) > 1e-3 && abs(p) < 2π * 1.0, ps)
        dom = isempty(osc) ? nothing : argmax(real, osc)  # least damped below 1 Hz
        (; gravity, T, ωg, K, τd,
           f_d = isnothing(dom) ? NaN : imag(dom) / 2π,
           ζ = isnothing(dom) ? NaN : -real(dom) / abs(dom))
    end
end
