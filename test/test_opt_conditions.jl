# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for what the optimizer is sent about a run (`src/opt_conditions.jl`): the wind
(`inflow_from_settings`, `cap_wind_speed`) and the winches of the three kinds of solve
(`winch_from_wc`, `optimizer_conditions`), read off the run's settings.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: inflow_from_settings, cap_wind_speed, winch_from_wc,
    optimizer_conditions, WCSettings, TrajOptSettings, AWETRIM_SOFTMINUS_BETA, AtmosphericModel,
    calc_wind_factor
using KiteUtils: KiteUtils

@testset verbose = true "opt_conditions" begin
    @testset "inflow_from_settings" begin
        set = (; h_ref = 6.0, v_wind = 8.0, upwind_dir = -90.0, profile_law = 3, alpha = 0.08,
               z0 = 0.0002, heights = [10.0, 100.0], speeds = [8.0, 12.0])
        inflow = inflow_from_settings(set)
        @test inflow.wind_speed == 8.0 && inflow.wind_direction == 270.0   # FROM, in [0, 360)
        @test inflow.profile_law == 3 && inflow.alpha == 0.08 && inflow.z0 == 0.0002
        @test inflow.turbulence == 0.0                 # the optimizer does not use it
        @test inflow.heights == [6.0] && inflow.speeds == [8.0]   # samples only for the fitted laws
        custom = inflow_from_settings(merge(set, (; profile_law = 4)))
        @test custom.heights == [10.0, 100.0] && custom.speeds == [8.0, 12.0]
        @test_logs (:warn, r"h_ref") inflow_from_settings(merge(set, (; h_ref = 10.0)))
    end

    @testset "winch_from_wc" begin
        wc = WCSettings(; dt = 0.01)
        w = winch_from_wc(wc)
        @test w.mode == "reelout" && w.k_v == wc.kv && w.f_min == wc.f_low
        @test w.f_max == wc.f_high && w.v_max == wc.v_sat && isnothing(w.p_max)
        @test w.softminus_beta == AWETRIM_SOFTMINUS_BETA     # pinned, not the local winch's
        @test w.softplus_beta == wc.softplus_beta && !w.optimize_k_v
        @test isnothing(w.v_sat_beta)                        # a hard clamp: the server's plain law
        @test isnothing(winch_from_wc(wc; v_max = nothing).v_max)
        @test winch_from_wc(wc; f_max = 5000.0).f_max == 5000.0
        wc.f_high_awe_trim = 6000.0                          # a fixed de-rating wins over f_high
        @test winch_from_wc(wc).f_max == 6000.0
        # The soft clamp is sent only where the local law soft-clamps at the same speed.
        wc.force_limit = "soft"; wc.v_sat_beta = 5.0
        @test winch_from_wc(wc).v_sat_beta == 5.0
        @test isnothing(winch_from_wc(wc; v_max = wc.v_sat + 1).v_sat_beta)
    end

    @testset "cap_wind_speed" begin
        data_path = KiteUtils.get_data_path()
        try
            KiteUtils.set_data_path(skc_data_path())
            set = KiteUtils.Settings(project_file("system_reelout_maasvlakte.yaml"))
            @test cap_wind_speed((; pattern_elevation_amplitude_max_wind_height = 0.0), set, 8.0) === 8.0
            @test cap_wind_speed((; pattern_elevation_amplitude_max_wind_height = 100.0), set, 8.0) ≈
                  calc_wind_factor(AtmosphericModel(set; nowindfield = true), 100.0) * 8.0
        finally
            KiteUtils.set_data_path(data_path)
        end
    end

    @testset "optimizer_conditions" begin
        tos = TrajOptSettings()
        tos.pattern_elevation_amplitude_max_wind_height = 0.0
        set = (; h_ref = 6.0, v_wind = 8.0, upwind_dir = -90.0, profile_law = 3, alpha = 0.08,
               z0 = 0.0002)
        rcs = WCSettings(; dt = 0.01)
        rcs.f_high_awe_trim = 6000.0
        fcs = FC_Settings(; first_lap_force_frac = 0.8)
        c = @test_logs (:info, r"Optimizer conditions") optimizer_conditions(tos, fcs, set, rcs, 7200.0)
        @test c.inflow.wind_speed == 8.0 && c.inflow.wind_direction == 270.0 && c.cap_wind == 8.0
        # The startup solve at the plain ceiling, lap 1 below it, the re-optimizations at the de-rating.
        @test c.winch.f_max == 7200.0 && c.winch_first_lap.f_max == 0.8 * 7200.0
        @test c.winch_reopt.f_max == 6000.0
        @test c.opt_awe_trim == (tos.opt_awe_trim >= 0 ? tos.opt_awe_trim : rcs.use_awe_trim)
        @test c.winch.use_awe_trim == c.opt_awe_trim
        @test isnothing(c.winch.winch_mode) && !c.winch.optimize_k_v
        c = optimizer_conditions(tos, FC_Settings(; first_lap_force_frac = 1.0), set, rcs, 7200.0)
        @test c.winch_first_lap === c.winch                  # no first-lap reduction
    end
end
nothing
