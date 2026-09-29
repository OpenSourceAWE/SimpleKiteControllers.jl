# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
V5 of oldplans/Plan_model_validation.md: code consistency of
`examples/course_loop_model.jl`, no simulation. The model lives in `examples/`,
not `src/`, so it is `include`d here rather than loaded as part of the package
— hence `ControlSystemsBase` and `DiscretePIDs` in `test/Project.toml`, both
already direct dependencies of `SimpleKiteControllers` itself.
"""

using Test
using DiscretePIDs
using ControlSystemsBase
using LinearAlgebra: diagm
using Random

include(joinpath(@__DIR__, "..", "examples", "course_loop_model.jl"))

@testset verbose=true "course_loop_model.jl (V5)" begin
    @testset "controller: course_pid vs DiscretePID" begin
        # `course_pid`'s C, applied to an error sequence via `lsim`, equals the
        # NEGATIVE of `cc.pid(0.0, err, 0.0)` (`CourseController`'s own call,
        # src/course_controller.jl) fed the same sequence as `err` — checked
        # empirically, not derived: `cc.err` and `course_pid`'s implicit `r - y`
        # error use opposite signs.
        K, Td, N, Ts = 0.35, 0.30, 2.0, 0.01
        Random.seed!(1)
        e = randn(80)
        for Ti in (false, 5.0)   # no integral action, and a finite one
            C = course_pid(K, Ti, Td, N, Ts)
            pid = DiscretePID(; K, Ti, Td, N, Ts)
            u_pid = [pid(0.0, ek, 0.0) for ek in e]
            u_lsim = vec(lsim(C, e').y)
            @test maximum(abs.(u_pid .+ u_lsim)) < 1e-10
        end
    end

    @testset "plant: plant_coeffs of the low pattern" begin
        # Exact at the identified depowers, linear between them, held at the ends.
        for (dp, c1, c2) in PLANT_COEFFS
            @test plant_coeffs(dp).c1 ≈ c1
            @test plant_coeffs(dp).c2 ≈ c2
        end
        (d1, a1, b1), (d2, a2, b2) = PLANT_COEFFS[1], PLANT_COEFFS[2]
        @test plant_coeffs((d1 + d2) / 2).c1 ≈ (a1 + a2) / 2
        @test plant_coeffs((d1 + d2) / 2).c2 ≈ (b1 + b2) / 2
        @test plant_coeffs(0.1) == plant_coeffs(first(PLANT_COEFFS)[1])
        @test plant_coeffs(0.5) == plant_coeffs(last(PLANT_COEFFS)[1])
    end

    @testset "plant: DC gain, shift-register states, actuator step" begin
        c1, c2, v_app, Ts = 0.25, 0.06, 27.0, 0.01
        P = turn_rate_plant(c1, c2, 0.0, v_app, 0.0, Ts; lag = 0.0, kite_lag = 0.0)
        @test P.nx == 1   # the turn-rate integrator alone, gravity = 0

        # DC gain of s*P is c1*v_app (gravity = 0); in discrete time that is
        # (z-1)/Ts * P evaluated at z=1.
        sP = tf([1, -1], [Ts], Ts) * P
        @test dcgain(sP)[1] ≈ c1 * v_app rtol=1e-8

        delay = 0.235
        Pd = turn_rate_plant(c1, c2, delay, v_app, 0.0, Ts; lag = 0.0, kite_lag = 0.0)
        @test Pd.nx - P.nx == round(Int, delay / Ts)

        lag = 0.43
        Gact = c2d(ss(-1 / lag, 1 / lag, 1.0, 0.0), Ts)
        res = step(Gact, 2 * lag)
        i = argmin(abs.(res.t .- lag))
        @test vec(res.y)[i] ≈ 1 - exp(-1) rtol=1e-6   # 63% after one lag
    end

    @testset "scaling: pattern_dead_time_lag" begin
        tc = (v_app = 13.3, dead_time = 0.141, kite_lag = 0.267)
        for v in (12.8, 22.4, 34.0, 40.0)
            law = PATTERN_DELAY_REF * (PATTERN_V_REF / v)^PATTERN_DELAY_EXP   # all at or above PATTERN_V_FLOOR
            τ, T = pattern_dead_time_lag(tc, v, PATTERN_LAW_DEPOWER)
            @test τ + T ≈ law                                            # the law at the reference depower
            @test τ / (τ + T) ≈ dead_time_fraction(PATTERN_LAW_DEPOWER)  # the low flights' split
            τ2, T2 = pattern_dead_time_lag(tc, v, 0.36)
            @test (τ2 + T2) / (τ + T) ≈ exp(PATTERN_DEPOWER_EXP * 0.09)  # the measured depower factor
        end
        # below PATTERN_V_FLOOR the sum holds its value there, the split still the low flights'
        τf, Tf = pattern_dead_time_lag(tc, PATTERN_V_FLOOR, PATTERN_LAW_DEPOWER)
        τ8, T8 = pattern_dead_time_lag(tc, 8.0, PATTERN_LAW_DEPOWER)
        @test τ8 + T8 ≈ τf + Tf
        @test τ8 / T8 ≈ τf / Tf
        # the factor reproduces the depower runs within 5 %: ×1.17 / 1.39 / 1.78 at 0.30 / 0.33 / 0.36
        for (dp, g) in ((0.30, 1.17), (0.33, 1.39), (0.36, 1.78))
            @test exp(PATTERN_DEPOWER_EXP * (dp - PATTERN_LAW_DEPOWER)) ≈ g rtol=0.05
        end
    end

    @testset "split: dead_time_fraction of the low pattern" begin
        # Exact at the identified depowers, held at the ends.
        for (dp, τ, T) in PLANT_SPLIT
            @test dead_time_fraction(dp) ≈ τ / (τ + T)
        end
        @test dead_time_fraction(0.1) == dead_time_fraction(first(PLANT_SPLIT)[1])
        @test dead_time_fraction(0.5) == dead_time_fraction(last(PLANT_SPLIT)[1])
        @test first.(PLANT_SPLIT) == first.(PLANT_COEFFS)   # the same fits
    end

    @testset "scaling: kite_dead_time / kite_lag" begin
        tc = (v_app = 22.5, dead_time = 0.217, kite_lag = 0.12)
        @test kite_dead_time(tc, tc.v_app) == tc.dead_time
        @test kite_lag(tc, tc.v_app) == tc.kite_lag
        tc_no_va = (v_app = NaN, dead_time = 0.217, kite_lag = 0.12)
        @test_throws ErrorException kite_dead_time(tc_no_va, 20.0)
    end

    @testset "margins: delay_margin closed form" begin
        # L = e^(-sτ)·k/s: analytically, delay_margin = π/(2k) - τ. The
        # dead time is rounded to whole samples (see turn_rate_plant), so Ts
        # must be small relative to τ for the two to agree closely.
        k, tau, Ts = 2.0, 0.05, 0.001
        n = round(Int, tau / Ts)
        Ld = c2d(tf(k, [1, 0]), Ts)
        A = diagm(-1 => ones(n - 1))
        D = ss(A, [1.0; zeros(n - 1)], [zeros(1, n - 1) 1.0], 0.0, Ts)
        L = Ld * D
        @test delay_margin(L) ≈ pi / (2k) - tau rtol=2e-2
    end

    @testset "margins: frd_margins on sampled points" begin
        # L = k·e^(-sτ)/s: |L| = 1 at f = k/2π, phase -90° - 360°·f·τ; -180° at f = 1/(4τ).
        k, tau = 2.0, 0.1
        f = collect(0.05:0.01:5.0)
        L = [k * cis(-2π * fi * tau) / (im * 2π * fi) for fi in f]
        m = frd_margins(f, L)
        f_gc = k / 2π
        @test m.f_gc ≈ f_gc rtol=1e-3
        @test m.pm ≈ 90 - 360 * f_gc * tau rtol=1e-3
        @test m.dm ≈ pi / (2k) - tau rtol=1e-2
        @test m.f_pc ≈ 1 / (4tau) rtol=1e-3
        @test m.gm ≈ 2π / (4tau) / k rtol=1e-3
    end

    @testset "margins: frd_diskmargin, closed forms" begin
        # α = 2 / max|(1 - L)/(1 + L)|: a pure integrator k/s has |.| = 1 everywhere,
        # a constant 0.5 has 1/3.
        f = 0.05:0.01:5.0
        @test frd_diskmargin([2.0 / (im * 2π * fi) for fi in f]) ≈ 2.0
        @test frd_diskmargin(fill(0.5 + 0im, 10)) ≈ 6.0
    end

    @testset "course_correction: the measured tables" begin
        tabs = load_course_correction()
        @test length(tabs) >= 2 && issorted([t.v_a for t in tabs])
        for t in tabs
            @test issorted(t.f) && length(t.f) > 20
            @test 0.2 <= first(t.f) && last(t.f) <= 4.1
            i = length(t.f) ÷ 2
            # at a measured airspeed the table is reproduced
            @test course_correction(tabs, t.f[i], t.v_a) ≈ exp(t.lg[i]) * cis(t.ph[i])
        end
        # a frequency scales with the airspeed: the feature at f, v moves to 2f at 2v
        t = tabs[1]
        @test course_correction([t], 0.5, t.v_a) ≈ course_correction([t], 1.0, 2t.v_a)
    end
end
