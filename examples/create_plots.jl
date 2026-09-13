# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Batch-generate pattern, time-series, power, aerodynamics and 3D-path plots
for every archived scenario in `output/scenarios/`, saving one
`pattern_<scenario>.png`, `time_series_<scenario>.png`, `power_<scenario>.png`,
`aerodynamics_<scenario>.png` and `path_webgl_<scenario>.html` per scenario
folder into `notebooks/images/`. The first four are static GLMakie PNGs; the
last is a self-contained interactive WGLMakie page (rotate/zoom in a
browser), since a 3D pattern is the one figure a flat image flattens the most.

Each subfolder of `output/scenarios/` (`v08`, `v09`, ...) is a self-contained
record of one `simple_opt_reelout.jl` run. This script loads each scenario's
flight log and settings from its own folder and generates a pattern plot showing
the flown azimuth/elevation path and the attractor reference (or optimizer's
uncorrected path if available), a time-series plot of the tracking, tether and
winch-state panels, a power plot of the winch force/speed/mechanical-power/
energy panels, and an aerodynamics plot of the angle-of-attack/L/D/speed
panels.

    include("create_plots.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using GLMakie
# Imported, not `using`d: WGLMakie exports names that clash with GLMakie's.
# `create_path_webgl_plots` switches which backend renders `Figure`/`Axis3`/...
# via `activate!`, Makie's normal multi-backend mechanism.
import WGLMakie
using MakieControlPlots
using LaTeXStrings
using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file

# Include utility function for pattern plotting
include(joinpath(@__DIR__, "plot_pattern_utils.jl"))

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))
const NOTEBOOKS_DIR = normpath(joinpath(@__DIR__, "..", "notebooks", "images"))

"""
    create_pattern_plots()

Scan `output/scenarios/` for non-empty scenario folders, generate a pattern plot
for each one using `plot_pattern_scenario`, and save as `notebooks/images/pattern_<name>.png`.
Folders with no files are skipped. Each plot is shown (`disp=true`) so
`MakieControlPlots.savefig` captures the right figure, then the window is
closed immediately via `MakieControlPlots.close`.
"""
function create_pattern_plots()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")

    # Collect non-empty scenario directories
    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && !isempty(readdir(dir))
    end
    isempty(dirs) && error("No non-empty scenario folders found in $SCENARIOS_DIR")

    # Ensure notebooks directory exists
    mkpath(NOTEBOOKS_DIR)

    @info "Generating pattern plots for $(length(dirs)) scenario(s)..."

    for scenario_dir in sort(dirs)
        scenario_name = basename(scenario_dir)
        try
            # Generate the pattern plot; disp=true is required for savefig to
            # capture this scenario's figure rather than a stale one
            p = plot_pattern_scenario(scenario_dir; disp = true)

            png_file = joinpath(NOTEBOOKS_DIR, "pattern_$(scenario_name).png")
            savefig(png_file)
            MakieControlPlots.close(p.fig)

            @info "Saved pattern plot" png_file
        catch e
            @warn "Failed to process scenario $scenario_name: $e"
        end
    end

    @info "Pattern plot generation complete"
end

"""
    create_time_series_plots()

Scan `output/scenarios/` for non-empty scenario folders, generate a
time-series plot for each one using `plot_time_series_scenario`, and save as
`notebooks/images/time_series_<name>.png`. Folders with no files are skipped. Each
plot is shown (`disp=true`) so `MakieControlPlots.savefig` captures the right
figure, then the window is closed immediately via `MakieControlPlots.close`.
"""
function create_time_series_plots()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")

    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && !isempty(readdir(dir))
    end
    isempty(dirs) && error("No non-empty scenario folders found in $SCENARIOS_DIR")

    mkpath(NOTEBOOKS_DIR)

    @info "Generating time-series plots for $(length(dirs)) scenario(s)..."

    for scenario_dir in sort(dirs)
        scenario_name = basename(scenario_dir)
        try
            p = plot_time_series_scenario(scenario_dir; disp = true)

            png_file = joinpath(NOTEBOOKS_DIR, "time_series_$(scenario_name).png")
            savefig(png_file)
            MakieControlPlots.close(p.fig)

            @info "Saved time-series plot" png_file
        catch e
            @warn "Failed to process scenario $scenario_name: $e"
        end
    end

    @info "Time-series plot generation complete"
end

"""
    create_power_plots()

Scan `output/scenarios/` for non-empty scenario folders, generate a power plot
for each one using `plot_power_scenario`, and save as
`notebooks/images/power_<name>.png`. Folders with no files are skipped. Each plot is
shown (`disp=true`) so `MakieControlPlots.savefig` captures the right figure,
then the window is closed immediately via `MakieControlPlots.close`.
"""
function create_power_plots()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")

    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && !isempty(readdir(dir))
    end
    isempty(dirs) && error("No non-empty scenario folders found in $SCENARIOS_DIR")

    mkpath(NOTEBOOKS_DIR)

    @info "Generating power plots for $(length(dirs)) scenario(s)..."

    for scenario_dir in sort(dirs)
        scenario_name = basename(scenario_dir)
        try
            p = plot_power_scenario(scenario_dir; disp = true)

            png_file = joinpath(NOTEBOOKS_DIR, "power_$(scenario_name).png")
            savefig(png_file)
            MakieControlPlots.close(p.fig)

            @info "Saved power plot" png_file
        catch e
            @warn "Failed to process scenario $scenario_name: $e"
        end
    end

    @info "Power plot generation complete"
end

"""
    create_aerodynamics_plots()

Scan `output/scenarios/` for non-empty scenario folders, generate an
aerodynamics plot for each one using `plot_aerodynamics_scenario`, and save as
`notebooks/images/aerodynamics_<name>.png`. Folders with no files are skipped. Each
plot is shown (`disp=true`) so `MakieControlPlots.savefig` captures the right
figure, then the window is closed immediately via `MakieControlPlots.close`.
"""
function create_aerodynamics_plots()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")

    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && !isempty(readdir(dir))
    end
    isempty(dirs) && error("No non-empty scenario folders found in $SCENARIOS_DIR")

    mkpath(NOTEBOOKS_DIR)

    @info "Generating aerodynamics plots for $(length(dirs)) scenario(s)..."

    for scenario_dir in sort(dirs)
        scenario_name = basename(scenario_dir)
        try
            p = plot_aerodynamics_scenario(scenario_dir; disp = true)

            png_file = joinpath(NOTEBOOKS_DIR, "aerodynamics_$(scenario_name).png")
            savefig(png_file)
            MakieControlPlots.close(p.fig)

            @info "Saved aerodynamics plot" png_file
        catch e
            @warn "Failed to process scenario $scenario_name: $e"
        end
    end

    @info "Aerodynamics plot generation complete"
end

"""
    create_path_webgl_plots()

Scan `output/scenarios/` for non-empty scenario folders, generate the 3D
flight-path figure for each one (`plot_path3d_scenario`), and save as a
self-contained interactive `notebooks/images/path_webgl_<name>.html` via
WGLMakie. Folders with no files are skipped. Switches the active Makie
backend to WGLMakie for the duration and restores GLMakie afterwards, since
the other `create_*_plots` functions render (and `savefig`) through it.
"""
function create_path_webgl_plots()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")

    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && !isempty(readdir(dir))
    end
    isempty(dirs) && error("No non-empty scenario folders found in $SCENARIOS_DIR")

    mkpath(NOTEBOOKS_DIR)

    @info "Generating 3D path (webgl) plots for $(length(dirs)) scenario(s)..."

    WGLMakie.activate!()
    for scenario_dir in sort(dirs)
        scenario_name = basename(scenario_dir)
        try
            fig = plot_path3d_scenario(scenario_dir; static_export = true)

            html_file = joinpath(NOTEBOOKS_DIR, "path_webgl_$(scenario_name).html")
            save_path3d_html(html_file, fig)

            @info "Saved 3D path (webgl) plot" html_file
        catch e
            @warn "Failed to process scenario $scenario_name: $e"
        end
    end
    GLMakie.activate!()

    @info "3D path (webgl) plot generation complete"
end

create_pattern_plots()
create_time_series_plots()
create_power_plots()
create_aerodynamics_plots()
create_path_webgl_plots()
