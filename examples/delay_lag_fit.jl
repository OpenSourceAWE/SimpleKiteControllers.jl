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
    fit_delay_lag(fit, dt; lag_max=1.0, lag_step=dt, t_max=3.0) -> NamedTuple

Split the kite's response to the applied steering into a dead time `τ` and a
first-order lag `T`, the model

    ψ̇ = c1·v_a·u_s(t − τ)/(1 + sT) + c2/v_a·sin(ψ)·cos(β)

`fit` is an `identify_turn_rate_law` result (or any NamedTuple with its `us`,
`rate`, `v_app`, `psi` and `beta`), sampled at `dt` [s]. For each `T` in
`0:lag_step:lag_max` the steering is lag-filtered, and `estimate_delay_fit` finds the
best dead time for it, fitting `c1` and `c2`; the pair with the smallest
residual wins. `identify_turn_rate_law`'s `delay` is the `T = 0` column of that
grid, so `rms_lag <= rms_delay` always.

Returns `(; dead_time, lag, c1, c2, rms_lag, rms_delay)`: the dead time [s]
with the same sub-sample and half-sample treatment as `delay_sec`, the lag [s],
the coefficients fitted with both, and the residual RMS [rad/s] of this fit and
of the pure-delay one. Warns when `T` hits `lag_max`.

Only steps in the steering separate the two: at low frequency both are a phase
of `-ω(τ + T)`, so check how much `rms_lag` gains over `rms_delay`. Measured
2026-09-26 on relay sweeps at depower 0.275: 17 % at `v_a` = 13.3 m/s, 4 % at
22.5 m/s.
"""
function fit_delay_lag(fit, dt; lag_max::Real = 1.0, lag_step::Real = dt, t_max::Real = 3.0)
    best = nothing
    rms_delay = NaN
    for T in 0:lag_step:lag_max
        uf = lag_filter(fit.us, T, dt)
        d, rms, d_frac = estimate_delay_fit(uf, fit.rate, fit.v_app, fit.psi, fit.beta, dt;
                                            t_max)
        T == 0 && (rms_delay = rms)
        (isnothing(best) || rms < best.rms) && (best = (; T, d, rms, d_frac, uf))
    end
    best.T >= lag_max - lag_step / 2 &&
        @warn @sprintf("fit_delay_lag: the kite's lag hit the search limit %.2f s; raise lag_max.", lag_max)
    c = fit_c1_c2(fit.v_app, fit.psi, fit.beta, fit.rate, shift_delay(best.uf, best.d))
    return (; dead_time = max(best.d_frac - 0.5, 0.0) * dt, lag = best.T, c1 = c.c1, c2 = c.c2,
            rms_lag = best.rms, rms_delay)
end
