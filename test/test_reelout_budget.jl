# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the settings overrides (`apply_overrides!`, `src/fc_settings.jl`) and the reel-out
time budget under a wind-speed override (`reelout_budget`, `src/reelout_budget.jl`). Pure
numbers, no model, winch file or wind profile.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: BUDGET_KNOT, BUDGET_F_COEF, BUDGET_REEL_MARGIN, BUDGET_ENTRY,
    BUDGET_TAIL, BUDGET_BELOW_KNOT_EXPONENT

@testset verbose = true "reelout_budget" begin
    @testset "apply_overrides!" begin
        fcs = FC_Settings()
        @test apply_overrides!(fcs, Dict(:reelout_l_max => 300, :compliance => 0),
                               "FCS_OVERRIDES", "FC_Settings", "fcs") === fcs
        # Converted to the field's type, not stored as the Int that was passed.
        @test fcs.reelout_l_max === 300.0
        @test fcs.compliance == 0
        # No overrides leave the struct as it was.
        @test apply_overrides!(fcs, Dict{Symbol, Any}(), "FCS_OVERRIDES", "FC_Settings",
                               "fcs").reelout_l_max == 300.0
        @test_throws ErrorException apply_overrides!(fcs, Dict(:no_such_field => 1),
                                                     "FCS_OVERRIDES", "FC_Settings", "fcs")
    end

    # A wind factor of 1.25 puts the knot at 6.16 m/s of ground wind.
    args = (; l_reel = 100.0, kv = 0.05, v_cap = 8.0, wind_factor = 1.25)

    @testset "above_the_knot" begin
        b = reelout_budget(8.0, 6.0, 60.0; args...)
        @test !b.below_knot
        @test b.v_nominal ≈ 0.05 * sqrt(BUDGET_F_COEF) * 8.0 * 1.25
        @test b.time ≈ BUDGET_ENTRY + 100.0 / (BUDGET_REEL_MARGIN * b.v_nominal) + BUDGET_TAIL
        # The project's sim_time plays no part above the knot.
        @test reelout_budget(8.0, 6.0, 1000.0; args...).time == b.time
    end

    @testset "drum_speed_cap" begin
        b = reelout_budget(20.0, 6.0, 60.0; args..., v_cap = 1.0)
        @test b.v_nominal == 1.0
        @test b.time ≈ BUDGET_ENTRY + 100.0 / BUDGET_REEL_MARGIN + BUDGET_TAIL
    end

    @testset "below_the_knot" begin
        # Faster than the default: scaled down linearly.
        w_fast = 0.99 * BUDGET_KNOT / 1.25
        b = reelout_budget(w_fast, 4.0, 60.0; args...)
        @test b.below_knot
        @test b.time ≈ 60.0 * 4.0 / w_fast
        # Slower than the default: scaled up with the exponent.
        b = reelout_budget(4.0, 6.0, 60.0; args...)
        @test b.below_knot
        @test b.time ≈ 60.0 * 1.5^BUDGET_BELOW_KNOT_EXPONENT
        # At the default wind the project's own sim_time.
        @test reelout_budget(5.0, 5.0, 60.0; args...).time ≈ 60.0
    end
end
