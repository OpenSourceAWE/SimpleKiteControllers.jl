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

"""
Response time of the kite in pattern flight, `τ_kite + T_kite` [s] at `v_a`
[m/s]: `PATTERN_DELAY_REF · (PATTERN_V_REF / v_a)^PATTERN_DELAY_EXP`.
Re-identified (`identify_turn_rate_law`) on 12 pattern logs at depower 0.27,
elevation 15 – 26°, tether 150 – 380 m, `v_a` 12.8 – 40.6 m/s
(docs/Plan_model_validation.md, V4). The relay sweeps behind the table fly at
73°, where at low `v_a` the kite responds more slowly: at 12.8 m/s the table
gives 0.43 s, the pattern 0.29 s.
"""
const PATTERN_DELAY_REF = 0.14
const PATTERN_V_REF = 34.0
const PATTERN_DELAY_EXP = 0.74
"Depower [-] the pattern law was measured at; its table row is `pattern_dead_time_lag`'s `tc_ref`"
const PATTERN_LAW_DEPOWER = 0.27

"""
    pattern_dead_time_lag(tc, v_app; tc_ref = tc) -> (τ, T)

The kite's dead time and lag [s] in pattern flight: the table's
([`kite_dead_time`](@ref), [`kite_lag`](@ref)) scaled by one factor, so their
sum follows the pattern law ([`PATTERN_DELAY_REF`](@ref)). The law was
measured at depower 0.27; `tc_ref` is that depower's table row, and at another
depower (`tc`) the law is multiplied by the table's ratio of the two rows'
sums at `v_app`, so the table's depower effect and its dead-time/lag split are
kept. Use it for the pattern loop only: the entry flies high, close to the
relay sweeps' conditions, where the table itself applies.
"""
function pattern_dead_time_lag(tc, v_app; tc_ref = tc)
    τ, T = kite_dead_time(tc, v_app), kite_lag(tc, v_app)
    sum_ref = kite_dead_time(tc_ref, v_app) + kite_lag(tc_ref, v_app)
    target = PATTERN_DELAY_REF * (PATTERN_V_REF / v_app)^PATTERN_DELAY_EXP * (τ + T) / sum_ref
    f = target / (τ + T)
    return f * τ, f * T
end

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

"""
    guidance_tf(ω_g, Ts) -> TransferFunction

The attractor guidance as seen by the course loop, `1 + ω_g/s` discretized
with the pole at z = 1: the commanded course follows the cross-track error,
which integrates the course, with the corner `ω_g = v_k/(L·D)` [rad/s] (`D`
the attractor's arc distance [rad]). Multiply the inner loop `C·P` by it for
pattern flight, as `stability_opt_reelout.jl` does. Validated at the 300 m
fig8 point (docs/Plan_model_validation.md, V1 step 1): it predicts the
measured course → regulated-error link at 0.5 Hz to within 5 % and 1°.
"""
guidance_tf(ω_g, Ts) = 1 + ω_g * Ts / (tf("z", Ts) - 1)

"""
    kite_correction(Ts; fz = KITE_CORR_ZERO, fp = KITE_CORR_POLE) -> StateSpace

Lag-lead `(1 + s/ω_z)/(1 + s/ω_p)` that brings the turn-rate law's steering →
heading response to what an injected multisine measures in the simulation:
from ~0.9 Hz up the kite turns less than the relay-identified law says (0.8 at
1.1 Hz, 0.6 – 0.7 above 1.4 Hz) with ~10° more lag. Multiply the plant by it,
together with the pattern law's dead time and lag (`pattern_dead_time_lag`),
against which it is fitted.
"""
kite_correction(Ts; fz = KITE_CORR_ZERO, fp = KITE_CORR_POLE) =
    c2d(ss(tf([1 / (2π * fz), 1], [1 / (2π * fp), 1])), Ts)

"""
Zero and pole [Hz] of [`kite_correction`](@ref): fit at the 300 m fig8 point,
0.5 – 2.1 Hz, 2026-09-27, against the plant with the pattern law's dead time
and lag (`pattern_dead_time_lag`), which it goes with. Against the table's
dead time and lag it was 1.08 / 0.72 Hz: part of its lag then stood in for the
delay the table lacks at 34 m/s.
"""
const KITE_CORR_ZERO = 0.80
const KITE_CORR_POLE = 0.58

"""
    frd_margins(f, L) -> NamedTuple

Margins of a loop given only as frequency-response points `L` [complex] at the
frequencies `f` [Hz], e.g. a measured plant times the controller: the first
gain crossover (`f_gc`, phase margin `pm` [deg], delay margin `dm` [s]) and
the first phase crossover (`f_pc`, gain margin `gm`), found by interpolating
log|L| and the unwrapped phase between the points. Needed where the plant
has no good low-order model, like the fed-back course of pattern flight
(docs/Plan_model_validation.md, V1 step 1). `NaN` where there is no crossing.
"""
function frd_margins(f, L)
    idx = sortperm(f)
    f, L = f[idx], L[idx]
    lg = log.(abs.(L))
    ph = angle.(L)
    ph = first(ph) .+ cumsum(vcat(0.0, rem2pi.(diff(ph), RoundNearest)))
    cross(y, level, i) = f[i] + (level - y[i]) / (y[i+1] - y[i]) * (f[i+1] - f[i])
    igc = findfirst(i -> lg[i] >= 0 && lg[i+1] < 0, 1:(length(f) - 1))
    ipc = findfirst(i -> ph[i] > -π && ph[i+1] <= -π, 1:(length(f) - 1))
    f_gc = isnothing(igc) ? NaN : cross(lg, 0.0, igc)
    pm = isnothing(igc) ? NaN :
        rad2deg(ph[igc] + (f_gc - f[igc]) / (f[igc+1] - f[igc]) * (ph[igc+1] - ph[igc])) + 180
    f_pc = isnothing(ipc) ? NaN : cross(ph, -π, ipc)
    gm = isnothing(ipc) ? NaN :
        1 / exp(lg[ipc] + (f_pc - f[ipc]) / (f[ipc+1] - f[ipc]) * (lg[ipc+1] - lg[ipc]))
    return (; f_gc, pm, dm = deg2rad(pm) / (2π * f_gc), f_pc, gm)
end

"""
    frd_diskmargin(L) -> Float64

Balanced disk margin α (skew 0) of a loop given as frequency-response points
`L`: `2 / max |(1 − L)/(1 + L)|`, i.e. `1/‖S − 1/2‖∞` over the points. Only
as good as the points cover the frequencies where the maximum sits, the
crossover region for the course loop.
"""
frd_diskmargin(L) = 2 / maximum(abs.((1 .- L) ./ (1 .+ L)))

"Path of the measured course correction, see [`load_course_correction`](@ref)"
const COURSE_CORRECTION_FILE = normpath(joinpath(@__DIR__, "..", "data", "course_correction_measured.csv"))

"""
    load_course_correction(path = COURSE_CORRECTION_FILE) -> Vector{NamedTuple}

The measured course correction `M(f, v_a)` [-]: what the simulation's steering →
fed-back course response is, divided by this model's tape lag × turn-rate law
(without `kite_correction`, which it contains). Measured with injected
multisines on the fig8 pattern at `v_a` 23.7, 34 (200 and 300 m, pooled) and
40.1 m/s (docs/Plan_model_validation.md, V1). One table per airspeed, sorted by
`v_a`, each with the frequencies [Hz], `log|M|` and the unwrapped phase [rad],
for [`course_correction`](@ref).
"""
function load_course_correction(path = COURSE_CORRECTION_FILE)
    rows = [parse.(Float64, split(l, ",")) for l in eachline(path)
            if !startswith(l, "#") && !startswith(l, "v_a") && !isempty(strip(l))]
    vs = sort(unique(getindex.(rows, 1)))
    return map(vs) do v
        r = filter(x -> x[1] == v, rows)
        (; v_a = v, f = getindex.(r, 2), lg = log.(getindex.(r, 3)), ph = getindex.(r, 4))
    end
end

"`M` of one table at `f` [Hz]: `log|M|` and phase interpolated linearly, end values held outside."
function _course_correction(tab, f)
    i = clamp(searchsortedlast(tab.f, f), 1, length(tab.f) - 1)
    w = clamp((f - tab.f[i]) / (tab.f[i+1] - tab.f[i]), 0.0, 1.0)
    return (1 - w) * tab.lg[i] + w * tab.lg[i+1], (1 - w) * tab.ph[i] + w * tab.ph[i+1]
end

"""
    course_correction(tabs, f, v_a) -> ComplexF64

`M` at `f` [Hz] and `v_a` [m/s] from the tables of `load_course_correction`:
each table is scaled in frequency to `v_a` (its features sit at a fixed
distance flown, so they move as `f ∝ v_a`) and `log|M|` and the phase are
interpolated linearly in `v_a` between the two nearest airspeeds. Outside the
measured airspeeds the nearest table is only scaled. At the measured airspeeds
it reproduces the data; between them it predicted the 34 m/s margins from the
23.7 and 40.1 m/s tables within 23 %, on the safe side. Keep `f` inside the
measured band (0.2 – 4 Hz at 34 m/s, scaled with `v_a`).
"""
function course_correction(tabs, f, v_a)
    vs = [t.v_a for t in tabs]
    i = clamp(searchsortedlast(vs, v_a), 1, max(length(vs) - 1, 1))
    j = min(i + 1, length(vs))
    w = j == i ? 0.0 : clamp((v_a - vs[i]) / (vs[j] - vs[i]), 0.0, 1.0)
    lg1, ph1 = _course_correction(tabs[i], f * vs[i] / v_a)
    lg2, ph2 = _course_correction(tabs[j], f * vs[j] / v_a)
    return exp((1 - w) * lg1 + w * lg2) * cis((1 - w) * ph1 + w * ph2)
end
