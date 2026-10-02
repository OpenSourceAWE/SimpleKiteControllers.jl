# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The linear course-loop model of the stability analysis (`examples/stability_fig8.jl`,
# `examples/stability_opt_reelout.jl`): the plant (actuator lag, turn-rate law, the
# kite's dead time and lag over v_a), the discrete PD and the margin helpers. The
# functions that build transfer functions (`course_pid`, `turn_rate_plant`,
# `delay_margin`, `guidance_tf`, `kite_correction`) are defined by the extension
# ext/SimpleKiteControllersControlSystemsBaseExt.jl, loaded with `using ControlSystemsBase`.

# Identified 2026-09-25, see docs/course_loop_stability.md.
"Equivalent lag [s] of the rate-limited steering tape, `set_steering` -> `steering`"
const ACTUATOR_LAG = 0.43   # simple_fig8.jl log, phase 4, depower 0.27, v_app 34-38 m/s
"""
Exponents of the kite's dead time and lag over `v_a`, `x ∝ v_a^-exp`: the
relay sweeps at depower 0.275 at 9.51 and 15 m/s of wind (`v_a` 13.3 and
22.5 m/s) split into dead time + lag (V3Kite's `fit_delay_lag`), 2026-09-26:
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
[`KITE_DEAD_TIME_EXP`](@ref) and `KITE_LAG_EXP`, where `tc.v_app` is the
airspeed of the sweep they were identified at. Away from it they are extrapolated.
"""
kite_dead_time(tc, v_app) = _scaled_row(tc, :dead_time, v_app, KITE_DEAD_TIME_EXP)
kite_lag(tc, v_app) = _scaled_row(tc, :kite_lag, v_app, KITE_LAG_EXP)

"""
Response time of the kite in pattern flight, `τ_kite + T_kite` [s] at `v_a`
[m/s]: `PATTERN_DELAY_REF · (PATTERN_V_REF / v_a)^PATTERN_DELAY_EXP`.
Re-identified (`identify_turn_rate_law`) on 12 pattern logs at depower 0.27,
elevation 15 – 26°, tether 150 – 380 m, `v_a` 12.8 – 40.6 m/s
(`oldplans/Plan_model_validation.md`, V4). The relay sweeps the table came from
until 2026-10-01 flew at 73°, where at low `v_a` the kite responds more slowly: at
12.8 m/s they gave 0.43 s, the pattern 0.29 s.
"""
const PATTERN_DELAY_REF = 0.14
const PATTERN_V_REF = 34.0
const PATTERN_DELAY_EXP = 0.74
"""
Airspeed [m/s] below which the pattern law holds its value instead of growing:
the lowest `v_a` it was identified at. Reel-out logs below it measured 0.279 s
at 10.6 m/s and 0.285 s at 10.1 m/s, against 0.292 s at 12.8 m/s, where the
unfloored law would give 0.33 – 0.35 s (`oldplans/Plan_model_validation.md`).
"""
const PATTERN_V_FLOOR = 12.8
"Depower [-] the pattern law was measured at"
const PATTERN_LAW_DEPOWER = 0.27
"""
Growth of the pattern response time with depower, `exp(PATTERN_DEPOWER_EXP ·
(depower − PATTERN_LAW_DEPOWER))`: point D (300 m, 7 m/s) flown at depower
0.30 / 0.33 / 0.36 gave ×1.17 / 1.39 / 1.78 over the 0.27 law
(`oldplans/Plan_model_validation.md`). The table's rows grow only ×1.05 – 1.15 over
the same range, so their ratio is not used.
"""
const PATTERN_DEPOWER_EXP = 6.1

"""
    pattern_dead_time_lag(tc, v_app, depower) -> (τ, T)

The kite's dead time and lag [s] in pattern flight: their sum follows the pattern
law ([`PATTERN_DELAY_REF`](@ref)) times the measured depower factor
([`PATTERN_DEPOWER_EXP`](@ref)), split in the ratio of the low crosswind flights at
`depower` ([`dead_time_fraction`](@ref)). Until 2026-09-29 the split was that of the
table row `tc` (the relay sweeps at 73°); `tc` is no longer used and kept for the
callers. Use it for the pattern loop only: the entry flies high, where the pattern
law was not measured. Below [`PATTERN_V_FLOOR`](@ref)
the law holds its value there (the measured response time stops growing at about
0.28 s).
"""
function pattern_dead_time_lag(tc, v_app, depower)
    target = PATTERN_DELAY_REF * (PATTERN_V_REF / max(v_app, PATTERN_V_FLOOR))^PATTERN_DELAY_EXP *
             exp(PATTERN_DEPOWER_EXP * (depower - PATTERN_LAW_DEPOWER))
    φ = dead_time_fraction(depower)
    return φ * target, (1 - φ) * target
end

function _scaled_row(tc, key, v_app, expo)
    x = getfield(tc, key)
    (isnan(tc.v_app) || isnan(x)) && error("_scaled_row: the turn-rate table row has no v_app or " *
        "$key; re-identify it with examples/build_turn_rate_table.jl.")
    return x * (tc.v_app / v_app)^expo
end

"""
    course_pid(K, Ti, Td, N, Ts) -> TransferFunction

Discrete transfer function from the regulated error to `rel_steering` of the
`DiscretePID` built in `CourseController`: `K` + `K·Ts/Ti/(z-1)` +
`bd·(z-1)/(z-ad)`, with `ad = Td/(Td+N·Ts)` and `bd = K·N·ad`. `Ti = false` means
no integral action.
Needs `using ControlSystemsBase`, which loads the method.
"""
function course_pid end

"""
    PLANT_COEFFS

The turn-rate law of the PLANT in the stability analysis,

    ψ̇ = c1·v_a·u_s + c2/v_a·sin(ψ)·cos(β),

over depower: `(depower, c1 [1/m], c2 [-])`, identified in the low crosswind pattern
(`oldplans/PlanIdentifyTurnRateLaw.md`, 2026-09-29; `examples/plot_turn_rate_vs_depower.jl`,
the data in `data/turn_rate_low_flights.tar.gz`): relay flights at fixed steering
amplitudes, reversing in azimuth, elevation held near 30°, 150 m at constant length,
9.51 m/s of wind, `v_a` ≈ 13 – 55 m/s. Standard errors from 20 s blocks: `c1` ±0.0003
– 0.0035, `c2` ±0.07 – 0.14. Depower 0.40 is left out, none of its flights stayed up.

Replaces the table's `c1` and the constant gravity coefficient `c3` = 0.23 1/s in the
plant from 2026-09-29: at 73° the gravity term is barely observable, and the low flights
put it at ≈ 0.10 1/s in the operating range instead of 0.23. The `c2/v_a` form follows
from the force balance; the flights are consistent with it but do not rule out a
constant `c3`. The model stays below every measured margin with
it. The controller's gain schedule does NOT use this: it keeps the turn-rate table,
as flown.
"""
const PLANT_COEFFS = [(0.250, 0.30624, 3.1495), (0.275, 0.26500, 3.6836),
                      (0.300, 0.23255, 3.6867), (0.325, 0.19491, 3.6967),
                      (0.350, 0.16602, 3.7070), (0.375, 0.14439, 3.8585)]

"""
    PLANT_SPLIT

Dead time and lag [s] of the kite over depower, `(depower, dead_time, lag)`, from the
same joint fits of the low crosswind flights as [`PLANT_COEFFS`](@ref) (whole samples
of `DT` = 1/60 s for the dead time, steps of 2`DT` for the lag). Only their ratio is
used ([`dead_time_fraction`](@ref)): one pair is fitted per depower over `v_a` ≈ 12 –
55 m/s, while the response time falls with `v_a`, so the sum comes from the pattern
law. Standard errors from 20 s blocks: dead time ±0.006 – 0.022 s, lag ±0.006 – 0.017 s.
"""
const PLANT_SPLIT = [(0.250, 0.00833, 0.10000), (0.275, 0.04167, 0.08333),
                     (0.300, 0.07500, 0.06667), (0.325, 0.07500, 0.08333),
                     (0.350, 0.10833, 0.06667), (0.375, 0.17500, 0.01667)]

"""
    dead_time_fraction(depower) -> Float64

`τ_d/(τ_d + T_k)` [-] of the low crosswind flights at `depower` [-]: dead time and lag
linear in [`PLANT_SPLIT`](@ref), held at its ends outside 0.25 – 0.375.
"""
function dead_time_fraction(depower)
    dps = first.(PLANT_SPLIT)
    u = clamp(depower, first(dps), last(dps))
    k = clamp(searchsortedlast(dps, u), 1, length(dps) - 1)
    w = (u - dps[k]) / (dps[k + 1] - dps[k])
    τ = (1 - w) * PLANT_SPLIT[k][2] + w * PLANT_SPLIT[k + 1][2]
    T = (1 - w) * PLANT_SPLIT[k][3] + w * PLANT_SPLIT[k + 1][3]
    return τ / (τ + T)
end

"""
    plant_coeffs(depower) -> (; c1, c2)

`c1` [1/m] and `c2` [-] of the plant at `depower` [-], linear in [`PLANT_COEFFS`](@ref),
held at its ends outside 0.25 – 0.375.
"""
function plant_coeffs(depower)
    dps = first.(PLANT_COEFFS)
    u = clamp(depower, first(dps), last(dps))
    k = clamp(searchsortedlast(dps, u), 1, length(dps) - 1)
    w = (u - dps[k]) / (dps[k + 1] - dps[k])
    return (; c1 = (1 - w) * PLANT_COEFFS[k][2] + w * PLANT_COEFFS[k + 1][2],
            c2 = (1 - w) * PLANT_COEFFS[k][3] + w * PLANT_COEFFS[k + 1][3])
end

"""
    turn_rate_plant(c1, c2, delay, v_app, gravity, Ts; lag = ACTUATOR_LAG, kite_lag = 0.0) -> StateSpace

`rel_steering` -> heading, ZOH-discretized: the actuator lag `lag` [s], then the
turn-rate law with the kite's own first-order lag `kite_lag` [s] and its dead
time `delay` [s] rounded to whole samples. `gravity = cos(ψ0)·cos(β)` in
[-1, 1] selects the sign and size of the gravity pole.
Needs `using ControlSystemsBase`, which loads the method.
"""
function turn_rate_plant end

"""
    delay_margin(L) -> Float64

Smallest extra dead time [s] that destabilizes `L`, over all its gain
crossovers; 0 if the closed loop is already unstable.
`ControlSystemsBase.delaymargin` takes the phase margin unwrapped and so reports
e.g. 374° instead of 14° for a loop with a long dead time.
Needs `using ControlSystemsBase`, which loads the method.
"""
function delay_margin end


"""
    rate_disk_margin(name, αs) -> Float64

Log and return the minimum disk margin of `αs`, rated as robust (≥ 0.5), marginal (≥ 0.3) or fragile.
"""
function rate_disk_margin(name, αs)
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
fig8 point (`oldplans/Plan_model_validation.md`, V1 step 1): it predicts the
measured course → regulated-error link at 0.5 Hz to within 5 % and 1°.
Needs `using ControlSystemsBase`, which loads the method.
"""
function guidance_tf end

"""
    kite_correction(Ts; fz = KITE_CORR_ZERO, fp = KITE_CORR_POLE) -> StateSpace

Lag-lead `(1 + s/ω_z)/(1 + s/ω_p)` that brings the turn-rate law's steering →
heading response to what an injected multisine measures in the simulation:
from ~0.9 Hz up the kite turns less than the relay-identified law says (0.8 at
1.1 Hz, 0.6 – 0.7 above 1.4 Hz) with ~10° more lag. Multiply the plant by it,
together with the pattern law's dead time and lag (`pattern_dead_time_lag`),
against which it is fitted.
Needs `using ControlSystemsBase`, which loads the method.
"""
function kite_correction end

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
(`oldplans/Plan_model_validation.md`, V1 step 1). `NaN` where there is no crossing.
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
const COURSE_CORRECTION_FILE = joinpath(skc_data_path(), "course_correction_measured.csv")

"""
    load_course_correction(path = COURSE_CORRECTION_FILE) -> Vector{NamedTuple}

The measured course correction `M(f, v_a)` [-]: what the simulation's steering →
fed-back course response is, divided by this model's tape lag × turn-rate law
(without `kite_correction`, which it contains). Measured with injected
multisines on the fig8 pattern at `v_a` 23.7, 34 (200 and 300 m, pooled) and
40.1 m/s (`oldplans/Plan_model_validation.md`, V1). One table per airspeed, sorted by
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
