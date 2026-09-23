# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the reel-out-window metrics in `fig8_metrics.jl`:
`reelout_power`, `winch_state_pct`, `reelout_ringing`, and the `_longest_run`
helper they share. These are untouched by `test_fig8_controller.jl`'s
`fig8_metrics`/`print_fig8_metrics` tests, which never populate `var_10`.
Pure arithmetic on synthetic logs, no simulation.
"""

using Test
using SimpleKiteControllers
using Statistics: mean
import SimpleKiteControllers: _longest_run

@testset verbose = true "reelout_metrics" begin

    @testset "_longest_run" begin
        @test _longest_run([1, 2, 3, 7, 8, 20, 21, 22, 23]) == [20, 21, 22, 23]
        @test _longest_run([5]) == [5]
        @test _longest_run([1, 2, 3]) == [1, 2, 3]
        # A tie keeps the FIRST run (strict `>`, not `>=`).
        @test _longest_run([1, 2, 10, 11]) == [1, 2]
    end

    # A reel-out window: `var_10` flat for 20 samples, then rising for 60
    # (samples 21:80, the "reeling" window `diff(l_set) .+ 1` selects), then
    # flat again. `winch_force`/`v_reelout` hold a spiked value on the last
    # window sample, so mean/peak/crest-factor are not all trivially equal.
    dt = 0.1
    n = 101
    tt = collect(0.0:dt:((n - 1) * dt))
    l_set = [i <= 20 ? 100.0 : (i <= 80 ? 100.0 + (i - 20) : 160.0) for i in 1:n]
    rw = findall(>(0.0), diff(l_set)) .+ 1   # 21:80

    force = fill(100.0, n)
    vro = fill(1.0, n)
    for (k, i) in enumerate(rw)
        force[i] = k < 60 ? 1000.0 : 4000.0
        vro[i] = k < 60 ? 2.0 : 1.0
    end
    mk_sl(; var_12 = nothing) = (; time = tt,
        winch_force = [Float32[force[i], 0, 0, 0] for i in 1:n],
        v_reelout = [Float32[vro[i], 0, 0, 0] for i in 1:n],
        var_10 = Float32.(l_set),
        (var_12 === nothing ? NamedTuple() : (; var_12 = Float32.(var_12)))...)

    @testset "reelout_power" begin
        rp = reelout_power(mk_sl())
        @test rp.n == length(rw) == 60
        @test rp.idx == rw
        @test rp.duration ≈ length(rw) * dt

        p_win = force[rw] .* vro[rw]
        f_win = force[rw]
        @test rp.mean_power ≈ mean(p_win)
        @test rp.peak_power ≈ maximum(p_win)
        @test rp.cf_power_ro ≈ maximum(p_win) / mean(p_win)
        @test rp.mean_force ≈ mean(f_win)
        @test rp.peak_force ≈ maximum(f_win)
        @test rp.cf_force_ro ≈ maximum(f_win) / mean(f_win)
        @test rp.energy ≈ sum(p_win) * dt

        # `energy_run` integrates the WHOLE log, so it differs from `energy`
        # whenever anything outside the reeling window draws power.
        p_all = force .* vro
        @test rp.energy_run ≈ sum(p_all) * dt
        @test rp.energy_run > rp.energy

        # No `var_10` at all, or a single sample: too short to diff.
        @test reelout_power((; var_10 = Float32[])) === nothing
        @test reelout_power((; var_10 = Float32[1.0])) === nothing
        # `var_10` present but never increasing: no reel-out happened.
        @test reelout_power(merge(mk_sl(), (; var_10 = fill(Float32(100.0), n)))) ===
              nothing
    end

    @testset "winch_state_pct" begin
        # 20 samples state 0 (lower-force), 30 state 1 (speed law), 10 state 2
        # (upper-force), inside the 60-sample reeling window.
        state = fill(1.0, n)
        for (k, i) in enumerate(rw)
            state[i] = k <= 20 ? 0.0 : (k <= 50 ? 1.0 : 2.0)
        end
        wsp = winch_state_pct(mk_sl(; var_12 = state))
        @test wsp.n == 60
        @test wsp.lower_force_pct ≈ 100 * 20 / 60
        @test wsp.speed_pct ≈ 100 * 30 / 60
        @test wsp.upper_force_pct ≈ 100 * 10 / 60
        @test wsp.lower_force_pct + wsp.speed_pct + wsp.upper_force_pct ≈ 100.0

        @test winch_state_pct((; var_10 = Float32[1.0])) === nothing
        @test winch_state_pct(merge(mk_sl(; var_12 = state),
                                    (; var_10 = fill(Float32(100.0), n)))) === nothing
    end

    @testset "reelout_ringing" begin
        @test reelout_ringing((; var_10 = Float32[1.0])) === nothing
        @test reelout_ringing(merge(mk_sl(), (; var_10 = fill(Float32(100.0), n)))) ===
              nothing

        # No ring at all: a flat `v_reelout` through the whole reeling window
        # leaves nothing for the residual to find above `peak_floor`.
        sl_flat = merge(mk_sl(), (; v_reelout = [Float32[3.0, 0, 0, 0] for _ in 1:n]))
        rr0 = reelout_ringing(sl_flat)
        @test rr0.n_peaks == 0
        @test isnan(rr0.period_s) && isnan(rr0.zeta) && isnan(rr0.overshoot_m_s)
        @test rr0.duration_s == 0.0
        @test rr0.peak_v_reelout_m_s == 3.0
        # The reeling window here (6 s) never reaches `ring_span` (20 s default).
        @test isnan(rr0.steady_v_reelout_m_s)
    end

    @testset "reelout_ringing_decaying_oscillation" begin
        # A longer reel-out (300 samples = 30 s) carrying a genuine damped ring:
        # v_reelout = linear ramp + A0*exp(-sigma*t)*cos(omega_d*t), sigma/omega_d
        # set from a known damping ratio and period. A LINEAR ramp is exactly
        # cancelled by the centered moving average away from the array edges, so
        # the residual the function extracts is (to float noise) the oscillation
        # term alone -- this is what makes period_s/zeta checkable against the
        # ground truth below rather than only sanity-bounded.
        n2 = 400
        tt2 = collect(0.0:dt:((n2 - 1) * dt))
        l_set2 = [i <= 20 ? 100.0 : (i <= 320 ? 100.0 + (i - 20) : 400.0)
                  for i in 1:n2]
        rw2 = findall(>(0.0), diff(l_set2)) .+ 1   # 21:320, 300 samples = 30 s

        zeta_true = 0.15
        period_true = 5.0
        omega_n = 2pi / period_true
        omega_d = omega_n * sqrt(1 - zeta_true^2)
        sigma = zeta_true * omega_n
        A0 = 0.6
        m_ramp = 0.05
        ring(t) = m_ramp * t + A0 * exp(-sigma * t) * cos(omega_d * t)

        vro2 = fill(0.0, n2)
        for (k, i) in enumerate(rw2)
            vro2[i] = ring((k - 1) * dt)
        end
        vro2[1:20] .= 0.0
        sl_ring = (; time = tt2, var_10 = Float32.(l_set2),
                   v_reelout = [Float32[vro2[i], 0, 0, 0] for i in 1:n2])
        rr = reelout_ringing(sl_ring)

        @test rr.n_peaks >= 2
        # period/zeta recovered from the log decrement of detected peaks, against
        # the analytic ground truth they were built from.
        @test rr.period_s ≈ period_true rtol=0.1
        @test rr.zeta ≈ zeta_true rtol=0.1
        @test 0.0 < rr.overshoot_m_s < A0
        @test 0.0 < rr.duration_s <= 20.0   # default ring_span

        # `peak_v_reelout_m_s`/`steady_v_reelout_m_s` are plain max/mean of the
        # raw (non-detrended) signal, over `ring_span` and beyond it respectively
        # -- checked against the same closed form the log was built from.
        t_head = (0:200) .* dt              # t = 0 .. 20.0 inclusive ("<=" ring_span)
        @test rr.peak_v_reelout_m_s ≈ maximum(ring.(t_head)) rtol=1e-3
        t_steady = (200:299) .* dt          # t = 20.0 .. 29.9
        @test rr.steady_v_reelout_m_s ≈ mean(ring.(t_steady)) rtol=1e-3

        # A one-sample glitch before the real reel-out (`var_10` logs a spurious
        # blip for a step or two, per `_longest_run`'s docstring) must be dropped
        # entirely, giving back exactly the clean result -- not merely a similar
        # one, since the glitch's own bogus `v_reelout` value would otherwise
        # contaminate the detrended residual at the start of the window.
        l_set_glitch = copy(l_set2)
        l_set_glitch[5] = l_set_glitch[4] + 0.5
        vro_glitch = copy(vro2)
        vro_glitch[5] = 999.0
        sl_glitch = (; time = tt2, var_10 = Float32.(l_set_glitch),
                     v_reelout = [Float32[vro_glitch[i], 0, 0, 0] for i in 1:n2])
        @test reelout_ringing(sl_glitch) == rr
    end
end
