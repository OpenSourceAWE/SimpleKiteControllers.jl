# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
V5 of docs/Plan_model_validation.md: code consistency of
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
end
