# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The transfer-function half of src/course_loop_model.jl, loaded with `using ControlSystemsBase`.
# The docstrings are on the stubs there.

module SimpleKiteControllersControlSystemsBaseExt

using ControlSystemsBase: tf, ss, c2d, feedback, isstable, margin
using LinearAlgebra: diagm
using SimpleKiteControllers: course_loop_model
import SimpleKiteControllers: course_pid, turn_rate_plant, delay_margin, guidance_tf, kite_correction

function course_pid(K, Ti, Td, N, Ts)
    z = tf("z", Ts)
    ad = Td / (Td + N * Ts)
    bd = K * N * ad
    C = K + bd * (z - 1) / (z - ad)
    Ti isa Bool || (C += K * Ts / Ti / (z - 1))
    return C
end

function turn_rate_plant(c1, c2, delay, v_app, gravity, Ts; lag, kite_lag = 0.0)
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

function delay_margin(L)
    isstable(feedback(L)) || return 0.0
    _, _, wpm, pm = margin(L; allMargins = true)
    dms = [deg2rad(mod(p, 360)) / w for (w, p) in zip(wpm[1], pm[1]) if w > 0]
    return isempty(dms) ? Inf : minimum(dms)
end

guidance_tf(ω_g, Ts) = 1 + ω_g * Ts / (tf("z", Ts) - 1)

function kite_correction(Ts, v_app; clm = course_loop_model())
    scale = v_app / clm.kite_corr_v_ref
    fz, fp = clm.kite_corr_zero * scale, clm.kite_corr_pole * scale
    return c2d(ss(tf([1 / (2π * fz), 1], [1 / (2π * fp), 1])), Ts)
end

end
