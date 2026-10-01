# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for reading and writing the settings files of a run: the menu state (`gui_state.jl`,
on a temporary copy, never the user's `data/gui.yaml`), the winch settings (`load_wc_settings`,
`build_winch`), `load_yaml_fields!`, the file names a system project points to, and the
step-lookup of the winch table.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: GUI_STATE_FILE_OVERRIDE, ensure_gui_state_file, load_wc_settings,
    build_winch, load_yaml_fields!, WinchPosController
using KiteUtils: KiteUtils

# Run `f` with the menu state in a temporary file, created from `content`, or absent for `nothing`.
function with_gui_file(f, content)
    file = joinpath(mktempdir(), "gui.yaml")
    isnothing(content) || write(file, content)
    GUI_STATE_FILE_OVERRIDE[] = file
    try
        return f(file)
    finally
        GUI_STATE_FILE_OVERRIDE[] = nothing
    end
end
# A settings struct with two fields, for `load_yaml_fields!`.
mutable struct Knobs
    a::Float64
    b::Int
end
const GUI_DEFAULT = read(joinpath(skc_data_path(), "gui.yaml.default"), String)

@testset verbose = true "settings_files" begin
    @testset "gui_state_read_write" begin
        @test gui_state_file() == joinpath(skc_data_path(), "gui.yaml")   # no override
        with_gui_file(GUI_DEFAULT) do file
            @test gui_state_file() == file
            # The shipped defaults: values unquoted, trailing comments dropped.
            @test read_gui_field("project") == "system_fig8_200m.yaml"
            @test read_gui_field("sim_time") == "default"
            @test isnothing(read_gui_field("no_such_key"))
            @test selected_project() == "system_fig8_200m.yaml"
            @test isnothing(selected_sim_time()) && isnothing(selected_windspeed())
            @test selected_turbulence() == "default" && selected_plots() == default_plots()
            # Written values read back, and every other line, comments included, is kept.
            n_lines = length(readlines(file))
            set_selected_sim_time(120)
            set_selected_windspeed(8.25)
            set_selected_plots(["pattern", "power"])
            @test selected_sim_time() == 120.0 && selected_windspeed() == 8.25
            @test selected_plots() == ["pattern", "power"]
            @test length(readlines(file)) == n_lines
            @test occursin("# seconds, or \"default\"", read(file, String))
            set_selected_plots(String[])
            @test read_gui_field("plots") == "none" && selected_plots() == String[]
            set_selected_sim_time(nothing); set_selected_windspeed(nothing)
            @test isnothing(selected_sim_time()) && isnothing(selected_windspeed())
            write_gui_field("default_turbulence", 0.5)
            @test selected_turbulence() == 0.5
            write_gui_field("new_key", "x")            # a missing key goes under gui:
            @test read_gui_field("new_key") == "x" && length(readlines(file)) == n_lines + 1
        end
    end

    @testset "gui_state_project_families" begin
        with_gui_file(GUI_DEFAULT) do file
            # A fig8 selection: the reel-out scripts fall back to their default, and back.
            @test selected_fig8_project() == "system_fig8_200m.yaml"
            @test selected_reelout_project() == default_reelout_project()
            set_selected_project("system_reelout_cabauw.yaml")
            @test selected_reelout_project() == "system_reelout_cabauw.yaml"
            @test selected_fig8_project() == default_project()
            @test scenario_site() == "cabauw"
        end
        with_gui_file("gui:\n    sim_time: \"default\"\n") do file   # no project key
            @test selected_project() == default_project()
        end
    end

    @testset "ensure_gui_state_file" begin
        with_gui_file(nothing) do file
            # Created from the .default next to it, writable even from a read-only checkout.
            write(file * ".default", GUI_DEFAULT)
            chmod(file * ".default", 0o444)
            @test ensure_gui_state_file() == file && isfile(file)
            @test filemode(file) & 0o777 == 0o644
            @test read_gui_field("project") == "system_fig8_200m.yaml"
        end
        with_gui_file(nothing) do file                    # neither file: no state, no error
            @test isnothing(read_gui_field("project"))
            @test selected_project() == default_project()
        end
    end

    @testset "load_wc_settings" begin
        dir = mktempdir()
        file = joinpath(dir, "wc.yaml")
        write(file, "wc_settings:\n    kv: 0.05\n    f_low: 400\n    dt: 0.5\n")
        wc = load_wc_settings(file; dt = 0.01)
        @test wc.kv == 0.05 && wc.f_low == 400.0
        @test wc.dt == 0.01                                # the plant's step wins over the file's
        @test wc.f_high == SimpleKiteControllers.WCSettings(; dt = 0.01).f_high   # absent: default
        write(file, "wc_settings:\n    kv: 0.05\n    no_such_field: 1\n")
        @test_throws ErrorException load_wc_settings(file; dt = 0.01)
    end

    @testset "build_winch" begin
        data_path = KiteUtils.get_data_path()
        try
            KiteUtils.set_data_path(skc_data_path())
            project = project_file("system_reelout_maasvlakte.yaml")
            set = KiteUtils.Settings(project)
            (; wc, wpc, dt0) = build_winch(project, set, (; compliance = 0.0))
            @test dt0 == 1 / set.sample_freq && wc.dt == dt0
            # The wind-dependent tables override the file's flat values; kv is not one of them.
            @test wc.kv == SimpleKiteControllers._wc_settings_value(project, "kv")
            @test wc.f_low == winch_f_low(set.v_wind; project)
            @test wc.force_limit == winch_force_limit(set.v_wind; project)
            @test wpc isa WinchPosController
            # REEL_OUT holds the drum in POSITION mode: any compliance is refused.
            @test_throws ErrorException build_winch(project, set, (; compliance = 0.5))
            @test_throws ErrorException build_winch(project, set, (; compliance = -1.0))
        finally
            KiteUtils.set_data_path(data_path)
        end
    end

    @testset "load_yaml_fields" begin
        dir = mktempdir()
        write(joinpath(dir, "k.yaml"), "knobs:\n    a: 2\n    f8_c: 0\n")
        k = load_yaml_fields!(Knobs(1.0, 3), "k.yaml", "knobs"; path = dir)
        @test k.a === 2.0 && k.b == 3                      # converted; the absent field kept
        write(joinpath(dir, "k.yaml"), "knobs:\n    f8_c: 1\n")   # a retired key must be 0
        @test_throws ErrorException load_yaml_fields!(Knobs(1.0, 3), "k.yaml", "knobs"; path = dir)
        write(joinpath(dir, "k.yaml"), "knobs:\n    c: 1\n")
        @test_throws ErrorException load_yaml_fields!(Knobs(1.0, 3), "k.yaml", "knobs"; path = dir)
    end

    @testset "project_files" begin
        project = project_file("system_reelout_maasvlakte.yaml")
        @test traj_opt_settings_file(project) == "traj_opt.yaml"
        @test winch_table_file(project) == "winch_table.yaml"
        @test turn_rate_coeffs_file(project) == "turn_rate_coeffs.yaml"
        for f in (traj_opt_settings_file(project), winch_table_file(project))
            @test isfile(joinpath(skc_data_path(), f))
        end
    end

    @testset "winch_table_select" begin
        project = project_file("system_reelout_maasvlakte.yaml")
        rows = SimpleKiteControllers.YAML.load_file(
            joinpath(skc_data_path(), winch_table_file(project)))["entries"]
        winds = sort([Float64(r["v_wind"]) for r in rows])
        value(w) = only(r["force_limit"] for r in rows if Float64(r["v_wind"]) == w)
        # A step lookup: the last row at or below the wind, the first row below the range.
        @test winch_table_select(winds[2], "force_limit"; project) == value(winds[2])
        @test winch_table_select((winds[2] + winds[3]) / 2, "force_limit"; project) == value(winds[2])
        @test winch_table_select(winds[1] - 1, "force_limit"; project) == value(winds[1])
        @test winch_table_select(winds[end] + 5, "force_limit"; project) == value(winds[end])
        @test_throws ErrorException winch_table_select(8.0, "no_such_column"; project)
    end
end
nothing
