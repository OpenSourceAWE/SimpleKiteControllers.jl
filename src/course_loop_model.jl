# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The linear course-loop model of the stability analysis (`examples/stability_fig8.jl`,
# `examples/stability_opt_reelout.jl`): the plant (actuator lag, turn-rate law, the
# kite's dead time and lag over v_a), the discrete PD and the margin helpers. The
# functions that build transfer functions (`course_pid`, `turn_rate_plant`,
# `delay_margin`, `guidance_tf`, `kite_correction`) are defined by the extension
# ext/SimpleKiteControllersControlSystemsBaseExt.jl, loaded with `using ControlSystemsBase`.

"""
The identified parameters of the linear course-loop model, loaded from the file the
system project names under `course_loop_model` ([`course_loop_model_file`](@ref),
`data/course_loop_model.yaml` in every project so far), where the provenance of each
value is recorded. The
turn-rate law itself (`c1`, `c2`, dead time and kite lag over depower) is not here: it
is the turn-rate table, see [`turn_rate_coeffs`](@ref), and neither is the steering
tape's lag, which is `1/steering_gain` of the KCU's P controller. Every field must be given by the
file; the defaults are `NaN` so that a missing key cannot pass as a value.

The session's instance is [`course_loop_model`](@ref).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct CourseLoopModel @deftype Float64
    "Exponent of the kite's dead time over `v_a`, `τ ∝ v_a^-exp` [-]"
    kite_dead_time_exp = NaN
    "Exponent of the kite's lag over `v_a`, `T ∝ v_a^-exp` [-]"
    kite_lag_exp = NaN
    "Pattern law: the kite's response time `τ + T` [s] at `pattern_v_ref`"
    pattern_delay_ref = NaN
    "Pattern law: reference airspeed [m/s]"
    pattern_v_ref = NaN
    "Pattern law: exponent over `v_a`, `τ + T ∝ v_a^-exp` [-]"
    pattern_delay_exp = NaN
    "Airspeed [m/s] below which the pattern law holds its value"
    pattern_v_floor = NaN
    "Depower [-] the pattern law was measured at"
    pattern_law_depower = NaN
    "Growth of the pattern response time with depower, `exp(exp·(depower - pattern_law_depower))` [-]"
    pattern_depower_exp = NaN
    "Zero [Hz] of [`kite_correction`](@ref)"
    kite_corr_zero = NaN
    "Pole [Hz] of [`kite_correction`](@ref)"
    kite_corr_pole = NaN
end

"""
    CourseLoopModel(filename::String; path = skc_data_path()) -> CourseLoopModel

Load the `course_loop_model:` section of `filename`. Errors on an unknown key and on
a key the file does not give.
"""
function CourseLoopModel(filename::String; path = skc_data_path())
    clm = load_yaml_fields!(CourseLoopModel(), filename, "course_loop_model"; path)
    missing_keys = [f for f in fieldnames(CourseLoopModel) if isnan(getfield(clm, f))]
    isempty(missing_keys) ||
        error("$filename does not give $(join(missing_keys, ", ")) of CourseLoopModel.")
    return clm
end

const _COURSE_LOOP_MODEL = Ref{CourseLoopModel}()

"""
    reload_course_loop_model!(project = project_file()) -> CourseLoopModel

Read the system project's course-loop model file ([`course_loop_model_file`](@ref)) and
make it the session's [`course_loop_model`](@ref). It is otherwise read only at package
load, against the default `project`, like the turn-rate table
([`reload_turn_rate_table!`](@ref)).
"""
function reload_course_loop_model!(project = project_file())
    _COURSE_LOOP_MODEL[] = CourseLoopModel(course_loop_model_file(project))
end

"""
    course_loop_model() -> CourseLoopModel

The session's identified course-loop model, read at package load or by the last
[`reload_course_loop_model!`](@ref).
"""
course_loop_model() = _COURSE_LOOP_MODEL[]

"""
    kite_dead_time(tc, v_app; clm = course_loop_model()) -> Float64
    kite_lag(tc, v_app; clm = course_loop_model()) -> Float64

The kite's dead time and first-order lag [s] from the applied steering to the
turn rate at `v_app` [m/s], for the turn-rate coefficients `tc`: the table's
`tc.dead_time` and `tc.kite_lag`, scaled as `(tc.v_app / v_app)^exp` with the
exponents `kite_dead_time_exp` and `kite_lag_exp` of [`CourseLoopModel`](@ref), where
`tc.v_app` is the airspeed of the flights they were identified at. Away from it they
are extrapolated.
"""
kite_dead_time(tc, v_app; clm = course_loop_model()) =
    _scaled_row(tc, :dead_time, v_app, clm.kite_dead_time_exp)
kite_lag(tc, v_app; clm = course_loop_model()) = _scaled_row(tc, :kite_lag, v_app, clm.kite_lag_exp)

"""
    pattern_dead_time_lag(tc, v_app, depower; clm = course_loop_model()) -> (τ, T)

The kite's dead time and lag [s] in pattern flight. Their sum follows the pattern law
of [`CourseLoopModel`](@ref), `pattern_delay_ref · (pattern_v_ref / v_a)^pattern_delay_exp`,
times the measured depower factor `exp(pattern_depower_exp · (depower −
pattern_law_depower))`. It is split in the ratio `tc.dead_time : tc.kite_lag` of the
turn-rate coefficients `tc`, which must be those at `depower`.

The pattern law was identified in the low crosswind pattern (elevation 15 – 26°), so
this function holds for the pattern loop only, not for the entry, which flies at a
much higher elevation. Below `pattern_v_floor` the law holds its value (the measured
response time stops growing at about 0.28 s).
"""
function pattern_dead_time_lag(tc, v_app, depower; clm = course_loop_model())
    target = clm.pattern_delay_ref *
             (clm.pattern_v_ref / max(v_app, clm.pattern_v_floor))^clm.pattern_delay_exp *
             exp(clm.pattern_depower_exp * (depower - clm.pattern_law_depower))
    (isnan(tc.dead_time) || isnan(tc.kite_lag)) && error("pattern_dead_time_lag: the turn-rate " *
        "table row has no dead_time or kite_lag; re-identify it with examples/build_turn_rate_table.jl.")
    φ = tc.dead_time / (tc.dead_time + tc.kite_lag)
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
    turn_rate_plant(c1, c2, delay, v_app, gravity, Ts; lag, kite_lag = 0.0) -> StateSpace

`rel_steering` -> heading, ZOH-discretized: the steering tape's lag `lag` [s]
(`1/steering_gain` of the settings, the lag of the KCU's P controller), then the
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
    kite_correction(Ts; fz = course_loop_model().kite_corr_zero, fp = course_loop_model().kite_corr_pole) -> StateSpace

Lag-lead `(1 + s/ω_z)/(1 + s/ω_p)` that brings the turn-rate law's steering →
heading response to what an injected multisine measures in the simulation:
from ~0.9 Hz up the kite turns less than the relay-identified law says (0.8 at
1.1 Hz, 0.6 – 0.7 above 1.4 Hz) with ~10° more lag. Multiply the plant by it,
together with the pattern law's dead time and lag (`pattern_dead_time_lag`),
against which it is chosen. It is the causal stand-in for the measured kite correction
([`kite_correction_file`](@ref)), which loses gain without the matching phase lag and so
has no low-order causal form: the zero and pole are chosen conservative against it
(`examples/identify_kite_correction.jl`).
Needs `using ControlSystemsBase`, which loads the method.
"""
function kite_correction end

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
for [`course_correction`](@ref). The measured kite correction
([`kite_correction_file`](@ref)) has the same format and is read with this function too.
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
