# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the settings overrides (`apply_overrides!`, `src/fc_settings.jl`) and the reel-out
time budget under a wind-speed override (`reelout_budget`, `src/reelout_budget.jl`), pure
numbers, and of `sim_budget`, which reads its inputs from the Maasvlakte project's files.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: BUDGET_KNOT, BUDGET_F_COEF, BUDGET_REEL_MARGIN, BUDGET_ENTRY,
    BUDGET_TAIL, BUDGET_BELOW_KNOT_EXPONENT, AtmosphericModel, calc_wind_factor, _drum_speed_limit
import KiteUtils

@testset verbose = true "reelout_budget" begin
    @testset "apply_overrides!" begin
        fcs = FC_Settings()
        @test apply_overrides!(fcs, Dict(:reelout_l_max => 300, :compliance => 0),
                               "fcs_overrides", "FC_Settings", "fcs") === fcs
        # Converted to the field's type, not stored as the Int that was passed.
        @test fcs.reelout.reelout_l_max === 300.0
        @test fcs.winch.compliance == 0
        # No overrides leave the struct as it was.
        @test apply_overrides!(fcs, Dict{Symbol, Any}(), "fcs_overrides", "FC_Settings",
                               "fcs").reelout_l_max == 300.0
        @test_throws ErrorException apply_overrides!(fcs, Dict(:no_such_field => 1),
                                                     "fcs_overrides", "FC_Settings", "fcs")
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

    @testset "sim_budget" begin
        data_path = KiteUtils.get_data_path()
        try
            KiteUtils.set_data_path(skc_data_path())   # where the project's wc_settings file lives
            project = project_file("system_reelout_maasvlakte.yaml")
            set = KiteUtils.Settings(project)
            fcs = FC_Settings(fc_settings(project))
            # No override: the sim_time asked for, `nothing` included.
            @test sim_budget(project, set, fcs, 123.0, nothing, set.v_wind) === 123.0
            @test isnothing(sim_budget(project, set, fcs, nothing, nothing, set.v_wind))
            @test _drum_speed_limit(project) > 0
            wind_factor = calc_wind_factor(AtmosphericModel(set; nowindfield = true), BUDGET_HEIGHT)
            # An override above and one below the knot: the budget of THIS project's drum limit,
            # kv, wind factor and reel-out length, from its own sim_time when none is asked for.
            for wind in (8.25, 3.5)
                b = reelout_budget(wind, set.v_wind, set.sim_time;
                                   l_reel = fcs.reelout.reelout_l_max - set.l_tether,
                                   kv = SimpleKiteControllers._wc_settings_value(project, "kv"),
                                   v_cap = _drum_speed_limit(project), wind_factor)
                @test sim_budget(project, set, fcs, nothing, wind, set.v_wind) ≈ b.time
            end
            @test (8.25 * wind_factor >= BUDGET_KNOT, 3.5 * wind_factor < BUDGET_KNOT) == (true, true)
        finally
            KiteUtils.set_data_path(data_path)
        end
    end
end
nothing
