# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the parts of `gui_state.jl` that do not read `data/gui.yaml`,
the user's own menu state: the site of a project and the wind-speed override.
"""

using Test
using SimpleKiteControllers

"The four fields `apply_windspeed_override!` reads and writes, without a settings file"
mutable struct WindSettings
    use_wind_vec::Bool
    v_wind::Float64
    wind_vec::AbstractVector{Float64}
    upwind_dir::Float64
    upwind_elevation::Float64
end

@testset verbose = true "gui_state" begin

    @testset "scenario_site" begin
        @test scenario_site("system_reelout_cabauw.yaml") == "cabauw"
        @test scenario_site("system_reelout_maasvlakte.yaml") == "maasvlakte"
        # Every other reel-out project is filed under maasvlakte.
        @test scenario_site("system_reelout_other.yaml") == "maasvlakte"
        @test basename(dirname(selected_scenarios_dir())) == "scenarios"
    end

    @testset "defaults" begin
        @test startswith(default_project(), "system_fig8")
        @test startswith(default_reelout_project(), "system_reelout")
        @test "pattern" in default_plots()
        @test gui_state_file() == joinpath(skc_data_path(), "gui.yaml")
    end

    @testset "apply_windspeed_override!" begin
        s = WindSettings(false, 8.0, [0.0, 0.0, 0.0], -90.0, 0.0)
        apply_windspeed_override!(s, nothing)
        @test s.v_wind == 8.0
        apply_windspeed_override!(s, 5.5)
        @test s.v_wind == 5.5
        # With use_wind_vec the vector is set instead, at the project's own direction.
        s = WindSettings(true, 8.0, [0.0, 0.0, 0.0], -90.0, 0.0)
        apply_windspeed_override!(s, 4.0)
        @test sqrt(sum(abs2, s.wind_vec)) ≈ 4.0
        @test s.v_wind == 8.0
    end
end
nothing
