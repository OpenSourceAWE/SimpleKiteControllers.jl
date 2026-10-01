# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The kite's response to the applied steering, split into a dead time and a
# first-order lag. Shared by `build_turn_rate_table.jl` (the relay sweeps) and
# `stability_opt_reelout.jl` (the flown log). Needs V3Kite's
# `estimate_delay_fit`, `fit_c1_c2`, `shift_delay` and Printf in scope.

"""
    lag_filter(u, T, dt) -> Vector{Float64}

`u` through a first-order lag of time constant `T` [s], sampled at `dt`:
`y[k] = a·y[k-1] + (1-a)·u[k]`, `a = exp(-dt/T)`, starting at `y[1] = u[1]`.
`T = 0` returns `u` unchanged, so the pure-delay fit is the grid's first column.
"""
function lag_filter(u, T, dt)
    y = Float64.(u)
    T > 0 || return y
    a = exp(-dt / T)
    for k in 2:length(y)
        y[k] = a * y[k - 1] + (1 - a) * u[k]
    end
    return y
end

"""
    fit_delay_lag(fit, dt; lag_max=1.0, lag_step=dt, t_max=3.0, c3=nothing) -> NamedTuple

Split the kite's response to the applied steering into a dead time `τ` and a
first-order lag `T`, the model

    ψ̇ = c1·v_a·u_s(t − τ)/(1 + sT) + c2/v_a·sin(ψ)·cos(β)

`fit` is an `identify_turn_rate_law` result (or any NamedTuple with its `us`,
`rate`, `v_app`, `psi` and `beta`), sampled at `dt` [s]. For each `T` in
`0:lag_step:lag_max` the steering is lag-filtered, and `estimate_delay_fit` finds the
best dead time for it, fitting `c1` and `c2`; the pair with the smallest
residual wins. `identify_turn_rate_law`'s `delay` is the `T = 0` column of that
grid, so `rms_lag <= rms_delay` always.

With `c3` given, the gravity term is Eq. (9) of the paper, `c3·sin(ψ)·cos(β)` with
`c3` [1/s] held fixed, and only `c1` is fitted ([`estimate_delay_fit_c3`](@ref));
the returned `c2` is then its table-form equivalent `c3·mean(v_a)`.

Returns `(; dead_time, lag, c1, c2, rms_lag, rms_delay)`: the dead time [s]
with the same sub-sample and half-sample treatment as `delay_sec`, the lag [s],
the coefficients fitted with both, and the residual RMS [rad/s] of this fit and
of the pure-delay one. Warns when `T` hits `lag_max`.

Only steps in the steering separate the two: at low frequency both are a phase
of `-ω(τ + T)`, so check how much `rms_lag` gains over `rms_delay`. Measured
2026-09-26 on relay sweeps at depower 0.275: 17 % at `v_a` = 13.3 m/s, 4 % at
22.5 m/s.
"""
function fit_delay_lag(fit, dt; lag_max::Real = 1.0, lag_step::Real = dt, t_max::Real = 3.0,
                       c3::Union{Nothing, Real} = nothing)
    best = nothing
    rms_delay = NaN
    for T in 0:lag_step:lag_max
        uf = lag_filter(fit.us, T, dt)
        d, rms, d_frac = isnothing(c3) ?
            estimate_delay_fit(uf, fit.rate, fit.v_app, fit.psi, fit.beta, dt; t_max) :
            estimate_delay_fit_c3(uf, fit.rate, fit.v_app, fit.psi, fit.beta, dt; c3, t_max)
        T == 0 && (rms_delay = rms)
        (isnothing(best) || rms < best.rms) && (best = (; T, d, rms, d_frac, uf))
    end
    best.T >= lag_max - lag_step / 2 &&
        @warn @sprintf("fit_delay_lag: the kite's lag hit the search limit %.2f s; raise lag_max.", lag_max)
    us_del = shift_delay(best.uf, best.d)
    c = isnothing(c3) ? fit_c1_c2(fit.v_app, fit.psi, fit.beta, fit.rate, us_del) :
                        fit_c1_c3(fit.v_app, fit.psi, fit.beta, fit.rate, us_del; c3)
    return (; dead_time = max(best.d_frac - 0.5, 0.0) * dt, lag = best.T, c1 = c.c1, c2 = c.c2,
            rms_lag = best.rms, rms_delay)
end

"""
    estimate_delay_fit_c3(us, rate, v_app, psi, beta, dt; c3, t_max=10.0) -> (d, rms, d_frac)

V3Kite's `estimate_delay_fit` for the turn-rate law of Eq. (9) of the paper,

    ψ̇ = c1·v_a·u_s(t − τ) + c3·sin(ψ)·cos(β)

with the gravity coefficient `c3` [1/s] held fixed (`C3` = 0.23 1/s, identified on
the flown figures of eight by `identify_c3.jl`, removed on 2026-10-01). The gravity term is subtracted
from `rate` and only `c1` is fitted at each shift. Same search, parabola refinement
and return values as `estimate_delay_fit`.

Fixing `c3` removes the trade of the relay sweep, where the steps of the input
always fall at the same headings and a shorter delay is bought with a larger
free `c2`.
"""
function estimate_delay_fit_c3(us::AbstractVector, rate::AbstractVector,
                               v_app::AbstractVector, psi::AbstractVector,
                               beta::AbstractVector, dt::Real; c3::Real, t_max::Real = 10.0)
    n = length(us)
    @assert n == length(rate) == length(v_app) == length(psi) == length(beta) "estimate_delay_fit_c3: inputs must have equal length"
    d_max = min(n - 3, round(Int, t_max / dt))
    y = rate .- c3 .* sin.(psi) .* cos.(beta)
    function mse(d)
        k = (1 + d):n
        x = v_app[k] .* view(us, k .- d)
        c1 = sum(x .* view(y, k)) / sum(abs2, x)
        return sum(abs2, view(y, k) .- c1 .* x) / length(k)
    end
    m = [mse(d) for d in 0:d_max]
    i = argmin(m)
    d = i - 1
    d_frac = Float64(d)
    if 1 < i < length(m)
        curv = m[i - 1] - 2m[i] + m[i + 1]
        curv > 0 && (d_frac += clamp(0.5 * (m[i - 1] - m[i + 1]) / curv, -0.5, 0.5))
    end
    return d, sqrt(m[i]), d_frac
end

"""
    fit_c1_c3(v_app, psi, beta, psi_dot, us; c3) -> NamedTuple

V3Kite's `fit_c1_c2` with the gravity term of Eq. (9), `c3·sin(ψ)·cos(β)`, held fixed:
`c1` is the least-squares fit of `c1·v_a·u_s = ψ̇ − c3·sin(ψ)·cos(β)`.

Returns the fields of `fit_c1_c2`: `c2` is the table-form equivalent `c3·mean(v_a)`
(`c2_at` of `course_loop_model.jl`) and `se2` is 0, since it was not fitted;
`cond` is 1 for the single regressor.
"""
function fit_c1_c3(v_app::AbstractVector, psi::AbstractVector, beta::AbstractVector,
                   psi_dot::AbstractVector, us::AbstractVector; c3::Real)
    x = v_app .* us
    y = psi_dot .- c3 .* sin.(psi) .* cos.(beta)
    c1 = sum(x .* y) / sum(abs2, x)
    resid = y .- c1 .* x
    n = length(y)
    sigma2 = sum(abs2, resid) / max(n - 1, 1)
    return (c1 = c1, c2 = c3 * sum(v_app) / n, se1 = sqrt(sigma2 / sum(abs2, x)), se2 = 0.0,
            rms = sqrt(sum(abs2, resid) / n), cond = 1.0, n = n)
end

"""
    joint_delay_lag_fit(fits, dt; lag_max=1.0, t_max=0.8) -> NamedTuple

One dead time, first-order lag, `c1` and `c2` for all `fits` (each an
`identify_turn_rate_law` result, sampled at `dt`): `fit_delay_lag` over several
flights. For every lag `T` in `0:dt:lag_max` and dead time of `d` samples in
`0:t_max/dt`, each flight's steering is lag-filtered and shifted on its own, the
first `t_max/dt` samples of every flight are dropped (so every candidate is
scored on the same samples, none of them padded by the shift), and `fit_c1_c2`
is fitted on all flights stacked. The pair with the smallest residual wins.

Returns `(; dead_time, lag, d, c1, c2, rms_lag, rms_delay, n)`: the dead time [s]
as `delay_sec` counts it (whole samples less half a sample), the lag [s], the
shift in samples, the coefficients, the residual RMS [rad/s] of this fit and of
the best pure delay (`T = 0`), and the number of samples.
"""
function joint_delay_lag_fit(fits, dt; lag_max = 1.0, t_max = 0.8)
    dmax = round(Int, t_max / dt)
    trim(x) = x[dmax + 1:end]
    stack(field) = stack_fits(fits, field; skip = dmax)
    rate, v_app, psi, beta = stack(:rate), stack(:v_app), stack(:psi), stack(:beta)
    best = nothing
    rms_delay = Inf
    for T in 0:dt:lag_max
        ufs = [lag_filter(f.us, T, dt) for f in fits]
        for d in 0:dmax
            us = reduce(vcat, [trim(shift_delay(u, d)) for u in ufs])
            c = fit_c1_c2(v_app, psi, beta, rate, us)
            T == 0 && (rms_delay = min(rms_delay, c.rms))
            (isnothing(best) || c.rms < best.rms) && (best = (; T, d, c1 = c.c1, c2 = c.c2, rms = c.rms))
        end
    end
    best.T >= lag_max - dt / 2 && @warn "joint_delay_lag_fit: the lag hit lag_max = $lag_max s."
    return (; dead_time = max(best.d - 0.5, 0.0) * dt, lag = best.T, best.d, best.c1, best.c2,
            rms_lag = best.rms, rms_delay, n = length(rate))
end
