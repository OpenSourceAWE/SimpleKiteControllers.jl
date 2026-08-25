# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Batch-generate pattern and time-series PNG plots for every archived scenario in
`output/scenarios/`, saving one `pattern_<scenario>.png` and one
`time_series_<scenario>.png` per scenario folder into `notebooks/`.

Each subfolder of `output/scenarios/` (`v08`, `v09`, ...) is a self-contained
record of one `simple_opt_reelout.jl` run. This script loads each scenario's
flight log and settings from its own folder and generates a pattern plot showing
the flown azimuth/elevation path and the attractor reference (or optimizer's
uncorrected path if available), plus a time-series plot of the tracking, tether
and winch-state panels, then saves them to `notebooks/pattern_<scenario>.png`
and `notebooks/time_series_<scenario>.png`.

    include("create_plots.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using GLMakie
using MakieControlPlots
using LaTeXStrings
using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file

# Include utility function for pattern plotting
include(joinpath(@__DIR__, "plot_pattern_utils.jl"))

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))
const NOTEBOOKS_DIR = normpath(joinpath(@__DIR__, "..", "notebooks"))

"""
    create_pattern_plots()

Scan `output/scenarios/` for non-empty scenario folders, generate a pattern plot
for each one using `plot_pattern_scenario`, and save as `notebooks/pattern_<name>.png`.
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
`notebooks/time_series_<name>.png`. Folders with no files are skipped. Each
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

create_pattern_plots()
create_time_series_plots()
