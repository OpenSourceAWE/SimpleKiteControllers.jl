# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Plotting for simple_auto_parking.jl results.

Loads the "tmp_auto_parking" log saved by simple_auto_parking.jl in the
`output` folder (`examples/../output`) and reproduces the same
plot as auto_parking.jl, entirely from logged data — no re-simulation needed.
v_reelout, winch_force, elevation, heading and AoA come straight from the
syslog; the L/D ratios come from the SysState spare slots that `step!` fills
during simple_auto_parking.jl's simulation loop (var_15 = L/D_wing, var_16 =
L/D_eff). The steering panel shows the actual value (`steering`, the KCU's tape-lagged
fraction) against the command (`set_steering`), the standard SysState pair.

Run from the REPL after (or instead of, if "tmp_auto_parking" already exists) running
simple_auto_parking.jl:

    include("simple_auto_parking_plots.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using LaTeXStrings
using V3Kite

@info "Loading simulation results..."
set_data_path(v3_data_path())
OUTPUT_DIR = joinpath(@__DIR__, "..", "output")
syslog = load_log("tmp_auto_parking"; path=OUTPUT_DIR)
sl = syslog.syslog

created_at = log_created_at("tmp_auto_parking"; path=OUTPUT_DIR)
fig_name = "V3 Kite Auto Parking"
if !isnothing(created_at)
    fig_name *= " – " * replace(first(split(created_at, '.')), "T" => "_")
end

# Skip the t=0 initial log entry (depower/var_14/var_15/var_16 are only
# filled from the first `step!` call onward).
rng = 2:length(sl.time)

@info "Plotting results..."
p = plotx(
    sl.time[rng],
    first.(sl.v_reelout[rng]),
    first.(sl.winch_force[rng]),
    rad2deg.(sl.elevation[rng]),
    rad2deg.(sl.heading[rng]),
    rad2deg.(sl.AoA[rng]),
    (100 .* sl.steering[rng], 100 .* sl.set_steering[rng]),
    (sl.var_15[rng], sl.var_16[rng]);
    xlabel = L"\mathrm{time}~[\mathrm{s}]",
    ysize = 18,
    legendsize = 18,
    ylabels = [
        L"v_{\mathrm{ro}}~[\mathrm{m/s}]",
        L"F_{\mathrm{t}}~[\mathrm{N}]",
        L"\mathrm{elevation}~[°]",
        L"\mathrm{heading}~[°]",
        L"\mathrm{AoA}~[°]",
        L"u_{\mathrm{s}}~[\%]",
        L"L/D~[-]",
    ],
    labels = [
        nothing,
        nothing,
        nothing,
        nothing,
        nothing,
        [L"u_{\mathrm{s}}", L"u_{\mathrm{s,set}}"],
        [L"L/D_{\mathrm{wing}}", L"L/D_{\mathrm{eff}}"],
    ],
    fig = fig_name,
)
display(p)
sleep(0.1)  # Allow Makie to render the plot before continuing

nothing
