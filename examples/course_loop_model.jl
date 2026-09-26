# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The linear course-loop model shared by `stability_fig8.jl` and
# `stability_opt_reelout.jl`: the plant (actuator lag, turn-rate law, the kite's
# dead time and lag over v_a), the discrete PD and the margin helpers. Needs
# ControlSystemsBase and LinearAlgebra.diagm in scope.

# Identified 2026-09-25, see docs/course_loop_stability.md.
"Equivalent lag [s] of the rate-limited steering tape, `set_steering` -> `steering`"
const ACTUATOR_LAG = 0.43   # simple_fig8.jl log, phase 4, depower 0.27, v_app 34-38 m/s
"""
Exponents of the kite's dead time and lag over `v_a`, `x ∝ v_a^-exp`: the
relay sweeps at depower 0.275 at 9.51 and 15 m/s of wind (`v_a` 13.3 and
22.5 m/s) split into dead time + lag (`fit_delay_lag`), 2026-09-26:
0.141 + 0.267 s and 0.082 + 0.133 s. Both are roughly a fixed distance flown,
1.9 m and 3.0 – 3.5 m.
"""
const KITE_DEAD_TIME_EXP = 1.03
const KITE_LAG_EXP = 1.32

"""
    kite_dead_time(tc, v_app) -> Float64
    kite_lag(tc, v_app) -> Float64

The kite's dead time and first-order lag [s] from the applied steering to the
turn rate at `v_app` [m/s], for the turn-rate coefficients `tc`: the table's
`tc.dead_time` and `tc.kite_lag`, scaled as `(tc.v_app / v_app)^exp` with
[`KITE_DEAD_TIME_EXP`](@ref) and [`KITE_LAG_EXP`](@ref), where `tc.v_app` is the
airspeed of the sweep they were identified at. Away from it they are extrapolated.
"""
kite_dead_time(tc, v_app) = _scaled_row(tc, :dead_time, v_app, KITE_DEAD_TIME_EXP)
kite_lag(tc, v_app) = _scaled_row(tc, :kite_lag, v_app, KITE_LAG_EXP)

function _scaled_row(tc, key, v_app, expo)
    x = getfield(tc, key)
    (isnan(tc.v_app) || isnan(x)) && error("course_loop_model: the turn-rate table row has no v_app or " *
        "$key; run add_delay_lag_split! of examples/build_turn_rate_table.jl for it.")
    return x * (tc.v_app / v_app)^expo
end

"""
    course_pid(K, Ti, Td, N, Ts) -> TransferFunction

Discrete transfer function from the regulated error to `rel_steering` of the
`DiscretePID` built in `CourseController`: `K` + `K·Ts/Ti/(z-1)` +
`bd·(z-1)/(z-ad)`, with `ad = Td/(Td+N·Ts)` and `bd = K·N·ad`. `Ti = false` means
no integral action.
"""
function course_pid(K, Ti, Td, N, Ts)
    z = tf("z", Ts)
    ad = Td / (Td + N * Ts)
    bd = K * N * ad
    C = K + bd * (z - 1) / (z - ad)
    Ti isa Bool || (C += K * Ts / Ti / (z - 1))
    return C
end

"""
    turn_rate_plant(c1, c2, delay, v_app, gravity, Ts; lag = ACTUATOR_LAG, kite_lag = 0.0) -> StateSpace

`rel_steering` -> heading, ZOH-discretized: the actuator lag `lag` [s], then the
turn-rate law with the kite's own first-order lag `kite_lag` [s] and its dead
time `delay` [s] rounded to whole samples. `gravity = cos(ψ0)·cos(β)` in
[-1, 1] selects the sign and size of the gravity pole.
"""
function turn_rate_plant(c1, c2, delay, v_app, gravity, Ts; lag = ACTUATOR_LAG, kite_lag = 0.0)
    first_order(T) = ss(-1 / T, 1 / T, 1.0, 0.0)
    kite = ss(c2 / v_app * gravity, c1 * v_app, 1.0, 0.0)
    lag > 0 && (kite = kite * first_order(lag))
    kite_lag > 0 && (kite = kite * first_order(kite_lag))
    P = c2d(kite, Ts)
    n = round(Int, delay / Ts)
    n == 0 && return P
    # Dead time as an n-sample shift register; a z^-n transfer function is ill-conditioned.
    A = diagm(-1 => ones(n - 1))
    D = ss(A, [1.0; zeros(n - 1)], [zeros(1, n - 1) 1.0], 0.0, Ts)
    return P * D
end

"""
    delay_margin(L) -> Float64

Smallest extra dead time [s] that destabilizes `L`, over all its gain
crossovers; 0 if the closed loop is already unstable.
`ControlSystemsBase.delaymargin` takes the phase margin unwrapped and so reports
e.g. 374° instead of 14° for a loop with a long dead time.
"""
function delay_margin(L)
    isstable(feedback(L)) || return 0.0
    _, _, wpm, pm = margin(L; allMargins = true)
    dms = [deg2rad(mod(p, 360)) / w for (w, p) in zip(wpm[1], pm[1]) if w > 0]
    return isempty(dms) ? Inf : minimum(dms)
end

function rate(name, αs)
    α_min = minimum(αs)
    if α_min < 0.3
        @error "$name: unstable or fragile, minimum disk margin $(round(α_min, digits=3))."
    elseif α_min < 0.5
        @warn "$name: marginally stable, minimum disk margin $(round(α_min, digits=3))."
    else
        @info "$name: stable, minimum disk margin $(round(α_min, digits=2)). A value ≥ 0.5 is considered robust."
    end
    return α_min
end
