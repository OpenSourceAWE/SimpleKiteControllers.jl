# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the accept gate of a re-optimized path (`src/reopt_gate.jl`): each check, its
order, and whether it asks for a fresh reply or gives up. Pure numbers, no optimizer.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: gate_candidate, retried, blend_folds, opt_length, wants_challenge,
    ELEVATION_MIN_RETRY_MARGIN

@testset verbose = true "reopt_gate" begin
    tos = TrajOptSettings(; min_feasibility_margin = 1.0, min_height = 40.0,
           blend_max_retries = 2, min_power_frac = 0.5,
           min_power_frac_prev = 0.85, max_size_growth = 1.3)
    # A candidate that passes everything; each test spoils one thing.
    good = (; margin = 1.2, clearance = 80.0, l_now = 200.0, chk_el_min = 15.0, el_floor = 10.0,
            folds = false, new_pred = 20e3, opt_power_pred = 20e3, prev_install_pred = 20e3,
            power_gate_off = false, size = (; growth = 1.0, az_ratio = 1.0, el_ratio = 1.0),
            blend_attempt = 0, opt_r_reply = 8.0, r_span = (200.0, 260.0), opt_r_min = 7.0)
    gate(; kw...) = gate_candidate(tos, merge(good, NamedTuple(kw)))

    @testset "accepts_a_good_candidate" begin
        g = gate()
        @test g.verdict == :accept && isnothing(g.raise)
    end

    @testset "turn_margin_is_never_retried" begin
        g = gate(margin = 0.9, blend_attempt = 0)
        @test g.verdict == :reject
        @test occursin("curvature margin 0.90", g.detail)
        @test occursin("the optimizer measured 8.00 m at r = 200-260 m, asked for >= 7.00 m", g.detail)
        @test !occursin("optimizer measured", gate(margin = 0.9, opt_r_reply = nothing).detail)
    end

    @testset "clearance_raises_the_floor_then_gives_up" begin
        g = gate(clearance = 30.0, chk_el_min = 9.0)
        @test g.verdict == :retry && g.low
        @test g.raise ≈ asind(40.0 / 200.0) - 9.0 + ELEVATION_MIN_RETRY_MARGIN
        @test g.reason == "clearance 30.0 m"
        g2 = gate(clearance = 30.0, blend_attempt = 2)
        @test g2.verdict == :reject && g2.detail == "clearance 30.0 m" * retried(2)
        @test gate(clearance = 30.0, blend_attempt = 0).low
    end

    @testset "elevation_floor" begin
        g = gate(chk_el_min = 8.0)
        @test g.verdict == :retry && g.low && g.raise ≈ 10.0 - 8.0 + ELEVATION_MIN_RETRY_MARGIN
        @test occursin("descends to 8.0°", g.reason)
        @test gate(chk_el_min = 8.0, blend_attempt = 2).verdict == :reject
    end

    @testset "fold_and_power" begin
        g = gate(folds = true)
        @test g.verdict == :retry && !g.low && g.reason == "blend folds" && isnothing(g.raise)
        @test occursin("below 50% of the startup prediction", gate(new_pred = 9e3).reason)
        @test occursin("below 85% of the previous install's", gate(new_pred = 16e3).reason)
        g2 = gate(new_pred = 9e3, blend_attempt = 2)
        @test g2.verdict == :reject && endswith(g2.detail, ", after 2 retries")
        # The power gates are bypassed for a candidate the winch model could not predict; a fold is not.
        @test gate(new_pred = 9e3, power_gate_off = true).verdict == :accept
        @test gate(folds = true, power_gate_off = true).verdict == :retry
    end

    @testset "size_growth" begin
        big = (; growth = 1.6, az_ratio = 1.5, el_ratio = 1.7)
        g = gate(size = big)
        @test g.verdict == :retry && !g.low
        @test occursin("1.60x the previous install's size", g.reason)
        @test gate(size = big, blend_attempt = 2).verdict == :reject
        @test gate_candidate(TrajOptSettings(tos; max_size_growth = 0.0), merge(good, (; size = big))).verdict == :accept
    end

    @testset "wants_challenge" begin
        ctos = TrajOptSettings(; challenge_growth = 1.1)
        grown = (; growth = 1.23, az_ratio = 1.2, el_ratio = 1.23)
        # Cabauw 7 m/s at 236 m: grew x1.23 at 0.98 of the previous prediction.
        c = (; size = grown, new_pred = 23949.0, prev_install_pred = 24427.0)
        @test wants_challenge(ctos, c)
        # Grew with MORE power, grew too little, or no previous install: no challenge.
        @test !wants_challenge(ctos, merge(c, (; new_pred = 24700.0)))
        @test !wants_challenge(ctos, merge(c, (; size = (; growth = 1.05, az_ratio = 1.05,
                                                        el_ratio = 1.0))))
        @test !wants_challenge(ctos, merge(c, (; prev_install_pred = NaN)))
        # Shrinking with less power is the power gates' business, not this one's.
        @test !wants_challenge(ctos, merge(c, (; size = (; growth = 0.6, az_ratio = 0.6,
                                                        el_ratio = 0.5))))
        @test !wants_challenge(TrajOptSettings(; challenge_growth = 0.0), c)
    end

    @testset "checks_run_in_order" begin
        # Everything wrong at once: the turn margin wins, then clearance, elevation, power, size.
        bad = (; margin = 0.5, clearance = 10.0, chk_el_min = 1.0, folds = true, new_pred = 1e3,
               size = (; growth = 3.0, az_ratio = 3.0, el_ratio = 3.0))
        @test gate(; bad...).verdict == :reject
        @test gate(; Base.structdiff(bad, (; margin = 0))...).reason == "clearance 10.0 m"
        @test occursin("descends", gate(; Base.structdiff(bad, (; margin = 0, clearance = 0))...).reason)
    end

    @testset "blend_folds" begin
        fold_tos = TrajOptSettings(; blend_fold_margin = 0.3)
        az0, el0 = figure_eight_path(20.0, 8.0, 0.0, 30.0, 0.0, 100)
        az1, el1 = figure_eight_path(24.0, 9.0, 0.0, 32.0, 0.0, 100)
        @test !blend_folds(fold_tos, az0, el0, az0, el0)
        @test !blend_folds(fold_tos, az0, el0, az1, el1)
        # Mirrored, the path is flown the other way round: half-way the blend collapses to a line.
        @test blend_folds(fold_tos, az0, el0, -az0, el0)
    end

    @testset "opt_length" begin
        # The settled length's 5th-decimal jitter is rounded away, so the request is repeatable.
        @test opt_length(150.00282) == 150.0
        @test opt_length(150.00290) == 150.0
        @test opt_length(150.3; step = 0.5) == 150.5
        @test opt_length(150.00282; step = 0.0) == 150.00282
    end
end
nothing
