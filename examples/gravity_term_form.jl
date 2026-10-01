# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Two checks of the turn-rate coefficients identified in the low crosswind pattern
(`oldplans/PlanIdentifyTurnRateLaw.md`), both on saved data, without flying.

1. **The form of the gravity term.** The heading rate is fitted as

       ψ̇ = c1·v_a·u_s + c_g·sin(ψ)·cos(β)·v_a^(−n)

   for `n` in `0:0.1:2`, each with its own dead time and lag (grid, as
   `joint_delay_lag_fit`): `n = 0` is the constant `c3` of Eq. (9) of the paper,
   `n = 1` the `c2/v_a` of the current law. On the saved fit windows of the
   steady low-pattern flights at depower 0.275: 9.51 m/s and 6.5 m/s of wind at
   constant length, and reeling out at 1 m/s.

2. **Is the model still conservative?** The pattern model of `validate_margins.jl`
   (`model_loops(...).corrected`: controller, pattern-law plant, guidance,
   `kite_correction`) at the six points where margins were measured (150 – 300 m,
   `v_a` 22 – 40 m/s), with the plant's `c1` and gravity term from the table and
   `C3` (A) or from the low flights (B). The operating point is rebuilt from
   tether length and `v_a` (`v_k = 0.96·v_a`, the fig8 project's depower and
   pattern centre), not from the original run records, so A does not reproduce the
   earlier model column exactly; the comparison A against B is like for like.
   The stable sign of the gravity pole, as in the original comparison.

The fit windows and the low-flight coefficients come from
`output/turn_rate_low_flights*`, unpacked from `data/turn_rate_low_flights.tar.gz`
when missing. Results 2026-09-29 are in `oldplans/PlanIdentifyTurnRateLaw.md`.

    include("gravity_term_form.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Printf
using DelimitedFiles: readdlm
using Statistics: mean

# DT, lag_filter, shift_delay; and V1_POINTS, course_controller_tf, v1_lag, tf_margins, model helpers.
include(joinpath(@__DIR__, "build_turn_rate_table.jl"))
include(joinpath(@__DIR__, "validate_margins.jl"))
using SimpleKiteControllers: C3

gt_output = normpath(joinpath(@__DIR__, "..", "output"))
gt_windows = joinpath(gt_output, "turn_rate_low_flights")
gt_summary = joinpath(gt_output, "turn_rate_low_flights.csv")
gt_archive = normpath(joinpath(@__DIR__, "..", "data", "turn_rate_low_flights.tar.gz"))
if !isfile(gt_summary) && !isdir(gt_windows)
    run(`tar -xzf $gt_archive -C $gt_output`)
    @info "Unpacked $gt_archive into $gt_output."
end

"The fit windows in `file` of `gt_windows`, one NamedTuple per flight"
function read_windows(file)
    m, h = readdlm(joinpath(gt_windows, file), ','; header = true)
    c(name) = Float64.(m[:, findfirst(==(name), vec(h))])
    flight = c("flight")
    return [(; us = c("us")[i], rate = c("rate")[i], v_app = c("v_app")[i], psi = c("psi")[i],
             beta = c("beta")[i]) for i in (findall(==(k), flight) for k in sort(unique(flight)))]
end

# ================= 1. FORM OF THE GRAVITY TERM ================= #

"""
    gravity_form_fit(fits; ns=0:0.1:2, lag_max=0.5, t_max=0.5) -> Vector

For each exponent `n` of `ns`, the best fit of `ψ̇ = c1·v_a·u_s + c_g·sin(ψ)·cos(β)·v_a^(−n)`
over the dead time (whole samples up to `t_max`) and lag (`0:2dt:lag_max`), each
flight's steering filtered and shifted on its own and its first `t_max` dropped:
`(; n, c1, cg, dead_time, lag, rms)`, `rms` in rad/s.
"""
function gravity_form_fit(fits; ns = 0:0.1:2, lag_max = 0.5, t_max = 0.5)
    dmax = round(Int, t_max / DT)
    trim(x) = x[dmax + 1:end]
    cat(k) = reduce(vcat, [trim(getfield(f, k)) for f in fits])
    rate, va, psi, beta = cat(:rate), cat(:v_app), cat(:psi), cat(:beta)
    g0 = sin.(psi) .* cos.(beta)
    return map(ns) do n
        grav = g0 .* va .^ (-n)
        best = nothing
        for T in 0:2DT:lag_max
            ufs = [lag_filter(f.us, T, DT) for f in fits]
            for d in 0:dmax
                us = reduce(vcat, [trim(shift_delay(u, d)) for u in ufs])
                A = [va .* us grav]
                c = A \ rate
                rms = sqrt(mean(abs2, rate .- A * c))
                (isnothing(best) || rms < best.rms) &&
                    (best = (; n, c1 = c[1], cg = c[2], dead_time = max(d - 0.5, 0.0) * DT, lag = T, rms))
            end
        end
        best
    end
end

gt_sets = ["9.51 m/s, constant length" => read_windows("depower_0.275.csv"),
           "6.5 m/s, constant length" => read_windows("wind_6.5_depower_0.275.csv"),
           "reeling out, 9.51 + 6.5 m/s" => vcat(read_windows("reelout_wind_9.51_depower_0.275.csv"),
                                                 read_windows("reelout_wind_6.5_depower_0.275.csv"))]
push!(gt_sets, "all constant length" => vcat(last(gt_sets[1]), last(gt_sets[2])))

println("\n1. Gravity term c_g·sin(ψ)·cos(β)·v_a^(−n), depower 0.275: n = 0 is Eq. (9), n = 1 the current law")
@printf("%-30s %12s %12s %7s %12s %12s\n", "flights", "rms n=0", "rms n=1", "best n", "c3 (n=0)", "c2 (n=1)")
gravity_fits = Dict{String, Any}()
for (name, fits) in gt_sets
    fs = gravity_form_fit(fits)
    gravity_fits[name] = fs
    f0, f1 = fs[findfirst(f -> f.n == 0, fs)], fs[findfirst(f -> f.n == 1, fs)]
    fb = argmin(f -> f.rms, fs)
    @printf("%-30s %9.3f °/s %9.3f °/s %7.1f %8.3f 1/s %12.2f\n", name, rad2deg(f0.rms),
            rad2deg(f1.rms), fb.n, f0.cg, f1.cg)
end

# ============ 2. MODEL AGAINST THE MEASURED MARGINS ============ #

gt_low = let mh = readdlm(gt_summary, ','; header = true)
    m, h = mh
    c(name) = Float64.(m[:, findfirst(==(name), vec(h))])
    (; dp = c("depower"), c1 = c("c1"), c2 = c("c2"))
end
"Linear interpolation of `y` over the low-flight depowers, clamped at the ends"
function low_interp(y, u)
    u = clamp(u, first(gt_low.dp), last(gt_low.dp))
    k = clamp(searchsortedlast(gt_low.dp, u), 1, length(gt_low.dp) - 1)
    w = (u - gt_low.dp[k]) / (gt_low.dp[k + 1] - gt_low.dp[k])
    return (1 - w) * y[k] + w * y[k + 1]
end

"""
    model_point(point, L, v_a; plant = :A) -> NamedTuple

Delay and gain margin of the pattern model of `validate_margins.jl` at V1 point
`point`, tether length `L` [m] and `v_a` [m/s], with `v_k = 0.96·v_a`, the
project's depower and pattern centre, and the stable sign of the gravity pole.
Plant `:A`: the table's `c1` and `C3`; `:B`: `c1` and `c2/v_a` of the low flights.
"""
function model_point(point, L, v_a; plant = :A)
    f = FC_Settings(fc_settings(project_file(V1_POINTS[point].project)))
    C, Ts = course_controller_tf(point, v_a)
    dp = f.depower_setpoint
    tc = turn_rate_coeffs(f.body_damping, dp)
    ω_g = 0.96 * v_a / (L * deg2rad(attractor_distance(f, v_a, L)))
    τp, Tp = pattern_dead_time_lag(tc, v_a, dp)
    c1, c2 = plant === :A ? (tc.c1, c2_at(v_a)) : (low_interp(gt_low.c1, dp), low_interp(gt_low.c2, dp))
    P = turn_rate_plant(c1, c2, τp, v_a, -cosd(f.el_center), Ts; lag = v1_lag(point), kite_lag = Tp)
    m = tf_margins(C * P * guidance_tf(ω_g, Ts) * kite_correction(Ts))
    return (; dm = m.dm, gm = m.gm, c3_eff = c2 / v_a)
end

# (V1 point, tether length [m], v_a [m/s], measured delay margin [s], measured gain margin)
measured_points = [(:F, 150, 33.6, 0.274, 3.51), (:B, 200, 22.4, 0.480, 3.35),
                   (:A, 200, 34.7, 0.324, 4.11), (:D, 300, 23.7, 0.301, 4.56),
                   (:D, 300, 33.6, 0.363, 5.0), (:D, 300, 40.1, 0.344, 3.80)]

println("\n2. Pattern model against the measured margins: A table c1 and C3, B low-flight c1 and c2/v_a")
@printf("%-16s %16s %16s %16s %16s %14s\n", "point", "measured DM/GM", "A DM/GM", "B DM/GM",
        "B error DM/GM", "c3 A -> B")
for (pt, L, va, dm, gm) in measured_points
    a, b = model_point(pt, L, va; plant = :A), model_point(pt, L, va; plant = :B)
    @printf("%3d m, %4.1f m/s   %6.3f s / %4.2f  %6.3f s / %4.2f  %6.3f s / %4.2f  %+5.0f %% / %+4.0f %%  %5.3f -> %5.3f\n",
            L, va, dm, gm, a.dm, a.gm, b.dm, b.gm, 100 * (b.dm / dm - 1), 100 * (b.gm / gm - 1),
            a.c3_eff, b.c3_eff)
end

nothing
