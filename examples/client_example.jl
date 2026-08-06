# Julia client for the AWETrim reelout flight-path optimizer (REST).
#
# Dependencies:  ] add HTTP JSON3 StructTypes
#
# Contract: POST /init receives and replies InitParams; POST /step receives
# and replies StepParams (both blocking — the reply contains the result).
# The reply trajectory is CLOSED (last point == first) with the same number
# of points you sent. Angles in DEGREES here; the tether length is the
# `length` field of InitParams / StepParams, the wind travels as an extra
# JSON field next to the structs.
#
# Winch coupling: the optimizer maps v_set = k_v*sqrt(force) onto its radial
# force model, so the optimized path assumes exactly your winch behavior.
#
# Infeasibility: an impossible request (too much force demanded, too little
# wind, ...) returns HTTP 422 with the solver message; the previous
# trajectory remains available under GET /trajectory.

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using HTTP, JSON3, StructTypes

# ---------------------------------------------------------------------------
# The shared structs (as agreed)
# ---------------------------------------------------------------------------
struct WinchParams
    mode::String      # "reelout" ("reelin" not supported yet)
    k_v::Float64      # v_set = k_v * sqrt(force)
    f_min::Float64    # minimum winch force [N]
    f_max::Float64    # maximum winch force [N]
end

struct Trajectory
    azimuth::Vector{Float64}    # [deg], azimuth 0 = downwind
    elevation::Vector{Float64}  # [deg], from the ground plane
end

Base.@kwdef struct InitParams
    name::String
    max_time::Float64
    length::Float64                    # initial length of the tether
    winch_params::WinchParams
    trajectory::Trajectory
    input_depower::Float64 = 1.6       # depower setting
    reg_weight::Float64 = 1.0          # regularization weight
    detect_simple_bounds::Bool = true  # solver flag
end

struct StepParams
    length::Float64                    # current length of the tether
    winch_params::WinchParams
    trajectory::Trajectory
end

StructTypes.StructType(::Type{WinchParams}) = StructTypes.Struct()
StructTypes.StructType(::Type{Trajectory}) = StructTypes.Struct()
StructTypes.StructType(::Type{InitParams}) = StructTypes.Struct()
StructTypes.StructType(::Type{StepParams}) = StructTypes.Struct()

# ---------------------------------------------------------------------------
# Transport helpers
# ---------------------------------------------------------------------------
const BASE = "http://127.0.0.1:8000"

function post(path, payload; timeout = 600)
    response = HTTP.post(BASE * path, ["Content-Type" => "application/json"];
                         body = JSON3.write(payload), read_idle_timeout = timeout)
    return response.body
end

as_dict(x) = JSON3.read(JSON3.write(x), Dict{String,Any})
as_winch(d) = WinchParams(d["mode"], d["k_v"], d["f_min"], d["f_max"])
as_traj(d) = Trajectory(Float64.(d["azimuth"]), Float64.(d["elevation"]))

function as_init_params(reply)
    return InitParams(reply["name"], reply["max_time"], reply["length"],
                      as_winch(reply["winch_params"]),
                      as_traj(reply["trajectory"]),
                      reply["input_depower"], reply["reg_weight"],
                      reply["detect_simple_bounds"])
end

"""init: send InitParams (incl. tether length and solver knobs, + wind)
-> InitParams."""
function init(params::InitParams; wind)
    payload = as_dict(params)
    payload["wind"] = wind                       # required by the optimizer
    return as_init_params(JSON3.read(post("/init", payload), Dict{String,Any}))
end

"""step: send StepParams (incl. the current tether length, + optional wind)
-> receive StepParams with the OPTIMIZED trajectory. Blocks ~10-20 s."""
function step(params::StepParams; wind = nothing)
    payload = as_dict(params)
    wind !== nothing && (payload["wind"] = wind)
    reply = JSON3.read(post("/step", payload), Dict{String,Any})
    return StepParams(reply["length"], as_winch(reply["winch_params"]),
                      as_traj(reply["trajectory"]))
end

# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------
# A starting figure-eight, 1000 points, repetitive (last == first)
s = range(0.0, 2pi; length = 1000)
guess = Trajectory(20.0 .* sin.(s), 22.0 .+ 8.0 .* sin.(2 .* s))

# k_v example: v = k_v*sqrt(F) -> at 8400 N this winch reels ~10 m/s.
# Too-stiff values (e.g. 0.02 -> ~1.8 m/s at max force) make the optimization
# infeasible and the server replies 422.
winch = WinchParams("reelout", 0.11, 1000.0, 8400.0)

# 200 m of tether; the solver knobs are left at their defaults
reply = init(InitParams(name = "uwe-sim-1", max_time = 600.0, length = 200.0,
                        winch_params = winch, trajectory = guess);
             wind = Dict("model_type" => "logarithmic", "U_ref" => 8.0,
                         "z_ref" => 100.0, "z0" => 0.03))
println("init ok: $(reply.name), starting path $(length(reply.trajectory.azimuth)) pts")

# First optimization (blocking; reply contains the optimized path)
result = step(StepParams(200.0, winch, reply.trajectory))
println("optimized: $(length(result.trajectory.azimuth)) pts, ",
        "elevation $(round(minimum(result.trajectory.elevation); digits=1))° – ",
        "$(round(maximum(result.trajectory.elevation); digits=1))°")

# ... fly the kite along result.trajectory ...

# Later, refresh with the current conditions from your simulation:
result = step(StepParams(220.0, winch, result.trajectory);   # current length
              wind = Dict("model_type" => "tabulated",
                          "heights" => [10.0, 100.0, 300.0],
                          "speeds" => [5.5, 8.0, 9.3]))
println("refreshed trajectory received")
