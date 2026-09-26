# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The linear course-loop model shared by `stability_fig8.jl` and
# `stability_opt_reelout.jl`: the plant (actuator lag, turn-rate law, the kite's
# dead time over v_a), the discrete PD and the margin helpers. Needs
# ControlSystemsBase and LinearAlgebra.diagm in scope.

# Identified 2026-09-25, see docs/course_loop_stability.md.
"Equivalent lag [s] of the rate-limited steering tape, `set_steering` -> `steering`"
const ACTUATOR_LAG = 0.43   # simple_fig8.jl log, phase 4, depower 0.27, v_app 34-38 m/s
"Exponent of the kite's dead time over `v_a`, fitted at 13.3, 22.5 and 36.3 m/s"
const KITE_DELAY_EXP = 1.24

"""
    kite_delay(tc, v_app) -> Float64

Dead time [s] from the applied steering to the turn rate at `v_app` [m/s], for
the turn-rate coefficients `tc`: the table's `tc.delay` scaled as
`(tc.v_app / v_app)^KITE_DELAY_EXP`, where `tc.v_app` is the airspeed of the
sweep it was identified at. Fitted on `identify_turn_rate_law` at depower 0.275:
0.417 s at 13.3 m/s and 0.217 s at 22.5 m/s (relay sweeps), 0.12 s at 36.3 m/s
(`simple_fig8.jl`). Roughly a fixed distance flown, 4 – 6 m. Below the sweep's
`v_app` it is extrapolated.
"""
function kite_delay(tc, v_app)
    isnan(tc.v_app) && error("kite_delay: the turn-rate table row has no v_app; re-run " *
                             "examples/build_turn_rate_table.jl for it (remake = true).")
    return tc.delay * (tc.v_app / v_app)^KITE_DELAY_EXP
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
    turn_rate_plant(c1, c2, delay, v_app, gravity, Ts; lag = ACTUATOR_LAG) -> StateSpace

`rel_steering` -> heading, ZOH-discretized: the actuator lag `lag` [s], then the
turn-rate law with its dead time `delay` [s] rounded to whole samples.
`gravity = cos(ψ0)·cos(β)` in [-1, 1] selects the sign and size of the gravity
pole.
"""
function turn_rate_plant(c1, c2, delay, v_app, gravity, Ts; lag = ACTUATOR_LAG)
    kite = ss(c2 / v_app * gravity, c1 * v_app, 1.0, 0.0)
    P = c2d(lag > 0 ? kite * ss(-1 / lag, 1 / lag, 1.0, 0.0) : kite, Ts)
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
