# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
V5 of `oldplans/Plan_model_validation.md`: code consistency of
`src/course_loop_model.jl` and its ControlSystemsBase extension, no simulation.
`using ControlSystemsBase` loads the extension, hence `ControlSystemsBase` in
`test/Project.toml`.
"""

using Test
using DiscretePIDs
using ControlSystemsBase
using LinearAlgebra: diagm
using Random
using SimpleKiteControllers
using SimpleKiteControllers: project_file

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

    @testset "CourseLoopModel: loaded from the project's file" begin
        clm = course_loop_model()
        @test clm isa CourseLoopModel
        @test all(f -> isfinite(getfield(clm, f)), fieldnames(CourseLoopModel))
        project = project_file("system_reelout_maasvlakte.yaml")
        @test course_loop_model_file(project) == "course_loop_model.yaml"
        @test reload_course_loop_model!(project).kite_corr_zero == clm.kite_corr_zero
        mktempdir() do dir
            # every key is required, an unknown key is an error
            write(joinpath(dir, "m.yaml"), "course_loop_model:\n  kite_corr_zero: 0.8\n")
            @test_throws ErrorException CourseLoopModel("m.yaml"; path = dir)
            write(joinpath(dir, "m.yaml"), "course_loop_model:\n  no_such_key: 1.0\n")
            @test_throws ErrorException CourseLoopModel("m.yaml"; path = dir)
        end
    end

    @testset "provenance: the model belongs to the kite flown" begin
        projects = filter(f -> startswith(f, "system_"), readdir(skc_data_path()))
        id = kite_id("system_reelout_maasvlakte.yaml")
        @test all(project -> kite_id(project) == id, projects)
        for project in projects
            @test isempty(stale_identification_steps(project))
            @test isnothing(check_model_provenance(project))
        end
        # A copy of the project with another wing drag is another kite: every step is stale.
        mktempdir() do dir
            project = project_file("system_reelout_maasvlakte.yaml")
            system = SimpleKiteControllers.YAML.load_file(project)["system"]
            cp(project, joinpath(dir, "system_copy.yaml"))
            cp(joinpath(dirname(project), system["sim_settings"]), joinpath(dir, system["sim_settings"]))
            kite = read(joinpath(dirname(project), system["kite_settings"]), String)
            write(joinpath(dir, system["kite_settings"]),
                  replace(kite, r"wing_drag_coeff: *[0-9.]+" => "wing_drag_coeff: 0.05"))
            kite_copy = joinpath(dir, "system_copy.yaml")
            @test kite_fingerprint(kite_copy)["wing_drag_coeff"] == 0.05
            @test kite_id(kite_copy) != id
            @test [s.step for s in stale_identification_steps(kite_copy)] == 1:5
            @test [s.step for s in stale_identification_steps(kite_copy; through = 2)] == 1:2
            @test_throws ErrorException check_model_provenance(kite_copy)
            @test_throws ErrorException check_model_provenance(project; through = 0, flown = (kite_copy,))
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

    @testset "scaling: pattern_dead_time_lag" begin
        clm = course_loop_model()
        dp0 = clm.pattern_delay_depower
        tc = (v_app = 13.3, dead_time = 0.141, kite_lag = 0.267)
        for v in (12.8, 22.4, 34.0, 40.0)
            approx = clm.pattern_delay_ref * (clm.pattern_v_ref / v)^clm.pattern_delay_exp   # all at or above the floor
            τ, T = pattern_dead_time_lag(tc, v, dp0)
            @test τ + T ≈ approx                                      # the approximation at the reference depower
            @test τ / (τ + T) ≈ tc.dead_time / (tc.dead_time + tc.kite_lag)   # tc's split
            τ2, T2 = pattern_dead_time_lag(tc, v, dp0 + 0.09)
            @test (τ2 + T2) / (τ + T) ≈ exp(clm.pattern_depower_exp * 0.09)  # the measured depower factor
        end
        # below pattern_v_floor the sum holds its value there, the split still tc's
        τf, Tf = pattern_dead_time_lag(tc, clm.pattern_v_floor, dp0)
        τ8, T8 = pattern_dead_time_lag(tc, 8.0, dp0)
        @test τ8 + T8 ≈ τf + Tf
        @test τ8 / T8 ≈ τf / Tf
        # the exponent is the fit through the origin of the depower runs of identify_depower_factor.jl
        # (2026-10-05, wing drag 0.03): ×0.96 / 1.06 / 1.18 / 1.64 at 0.27 / 0.30 / 0.33 / 0.36,
        # within the rounding of the ratios to two digits
        runs = ((0.27, 0.96), (0.30, 1.06), (0.33, 1.18), (0.36, 1.64))
        fitted = sum((dp - dp0) * log(g) for (dp, g) in runs) / sum((dp - dp0)^2 for (dp, _) in runs)
        @test clm.pattern_depower_exp ≈ fitted rtol=0.01
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
nothing
