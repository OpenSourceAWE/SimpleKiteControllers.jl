# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    plot_trajectory.jl — plot the last failed trajectory saved by simple_opt_reelout.jl

The startup feasibility gate of `simple_opt_reelout.jl` saves every rejected
startup path (and the incumbent that the gates refused) as a YAML file in
`trajectories/`, right before the run aborts. This script plots the NEWEST of
those files: the reference (azimuth, elevation) curve as the optimizer
returned it, with its measured curvature margin and predicted power in the
title.

Run from the repo root or anywhere:

    include("examples/plot_trajectory.jl")

Pass a file explicitly to plot a specific one instead of the newest:

    TRAJ_FILE = "trajectories/startup_incumbent_20260827_114530.yaml"
    include("examples/plot_trajectory.jl")
"""

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using YAML
using MakieControlPlots
using LaTeXStrings

const TRAJ_DIR = joinpath(@__DIR__, "..", "trajectories")

# TRAJ_FILE = "startup_retry3_2026-08-27_1700.yaml"

isdir(TRAJ_DIR) || error("No trajectories folder at $TRAJ_DIR — nothing has been saved yet.")

# The newest saved trajectory, unless the caller pinned one.
traj_file = if @isdefined(TRAJ_FILE)
    joinpath(TRAJ_DIR, TRAJ_FILE)
else
    files = filter(f -> endswith(f, ".yaml"), readdir(TRAJ_DIR; join = true))
    isempty(files) && error("No .yaml files in $TRAJ_DIR.")
    last(sort(files; by = f -> stat(f).mtime))
end

data = YAML.load_file(traj_file)
az = Float64.(data["azimuth_deg"])
el = Float64.(data["elevation_deg"])
margin = data["margin"]
power = data["predicted_power_W"]
l_tether = data["l_tether"]
min_margin = data["min_feasibility_margin"]

@info "Plotting $traj_file: $(length(az)) points, margin $margin (gate $min_margin), predicted power $power W at L = $l_tether m."

plotxy(
    [az],
    [el];
    xlabel = L"\mathrm{azimuth}~[°]",
    ylabel = L"\mathrm{elevation}~[°]",
    legend = [L"\mathrm{optimizer,~uncorrected}"],
    fig = "Failed startup trajectory – margin $(round(margin; digits = 3)) " *
          "(gate $min_margin), $(round(power; digits = 0)) W at L = $(round(l_tether; digits = 0)) m",
    disp = true,
)
