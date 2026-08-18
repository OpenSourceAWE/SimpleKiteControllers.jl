# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Demo of the AWETrim reelout flight-path optimizer: ask for one optimized path
# and plot it against the initial guess. The client itself — the structs, the
# transport and the server process helpers — is `awetrim_client.jl`, which
# documents the contract; this file is only the walk-through.
#
#     include("examples/optimize_path.jl")
#
# The conditions below are typed in to show the request, NOT to be copied into a
# run: a flown path must be optimized for the run's own wind and winch, which is
# what `inflow_from_settings` and `winch_from_wc` are for.

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using MakieControlPlots   # for the pattern plot at the end

include(joinpath(@__DIR__, "awetrim_client.jl"))

ensure_server()

# A starting figure-eight, 1000 points, repetitive (last == first)
s = range(0.0, 2pi; length = 1000)
guess = Trajectory(20.0 .* sin.(s), 22.0 .+ 8.0 .* sin.(2 .* s))

# k_v example: v = k_v*sqrt(F) -> at 8400 N this winch reels ~10 m/s.
# Too-stiff values (e.g. 0.02 -> ~1.8 m/s at max force) make the optimization
# infeasible and the server replies 422.
winch = WinchParams("reelout", 0.04, 1000.0, 8400.0)

# 5.2 m/s at 6 m height, wind from the west, logarithmic profile
# (~8 m/s at 100 m with this roughness).
inflow = InflowConditions(wind_speed = 5.2, wind_direction = 270.0, profile_law = 2, z0 = 0.03)

# 200 m of tether; the solver knobs are left at their defaults
reply = opt_init(InitParams(name = "uwe-sim-1", length = 200.0,
                            winch_params = winch, inflow_conditions = inflow,
                            trajectory = guess))
toc("Executed init in :")
@info ("Init ok: $(reply.name), starting path $(length(reply.trajectory.azimuth)) pts")

# First optimization (blocking; reply contains the optimized path)
result = opt_step(StepParams(200.0, winch, reply.trajectory))
toc("Executed step in: ")
@info ("optimized: $(length(result.trajectory.azimuth)) pts, ",
        "elevation $(round(minimum(result.trajectory.elevation); digits=1))° – ",
        "$(round(maximum(result.trajectory.elevation); digits=1))°")

# The pattern in the azimuth/elevation plane: initial guess vs. optimized path.
p = plotxy([guess.azimuth, result.trajectory.azimuth],
           [guess.elevation, result.trajectory.elevation];
           xlabel = "azimuth [°]", ylabel = "elevation [°]",
           legend = ["initial guess", "optimized"],
           fig = "reelout pattern")
display(p)

# ... fly the kite along result.trajectory ...

# Everything the replies do not carry — the dense guidance table (in RADIANS),
# the spline's `downloops` flag, the optimized parameters:
# table = opt_trajectory()

# Later, refresh with the current conditions from your simulation. A measured
# profile is sent as a CUSTOM_* law: the heights/speeds samples are fitted
# (here: log law + Gaussian jet).
# result = opt_step(StepParams(220.0, winch, result.trajectory);   # current length
#                   inflow_conditions = InflowConditions(wind_speed = 8.4, wind_direction = 265.0,
#                                                        profile_law = 6,
#                                                        heights = [10.0, 50.0, 100.0, 200.0, 300.0],
#                                                        speeds = [5.5, 7.4, 8.0, 9.3, 8.6]))
# println("refreshed trajectory received")
