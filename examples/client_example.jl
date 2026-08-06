# Julia client for the AWETrim reelout flight-path optimizer (REST).
#
# Dependencies:  ] add HTTP JSON3 StructTypes
#
# Contract: POST /init receives and replies InitParams; POST /step receives
# and replies StepParams (both blocking — the reply contains the result).
# The reply trajectory is CLOSED (last point == first) with the same number
# of points you sent. Angles in DEGREES here; the server also carries wind
# and tether length as extra JSON fields next to the structs.
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
    v_max::Float64    # maximum winch speed [m/s]
    f_min::Float64    # minimum winch force [N]
    f_max::Float64    # maximum winch force [N]
end

struct Trajectory
    azimuth::Vector{Float64}    # [deg], azimuth 0 = downwind
    elevation::Vector{Float64}  # [deg], from the ground plane
end

struct InitParams
    name::String
    max_time::Float64
    winch_params::WinchParams
    trajectory::Trajectory
end

struct StepParams
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

"""init: send InitParams (+ wind, tether length, solver knobs) -> InitParams."""
function init(params::InitParams; wind, distance_radial = nothing,
              extras = Dict{String,Any}())
    payload = as_dict(params)
    payload["wind"] = wind                       # required by the optimizer
    distance_radial !== nothing && (payload["distance_radial"] = distance_radial)
    merge!(payload, extras)                      # e.g. "sim_parameters"
    return JSON3.read(post("/init", payload), InitParams)
end

"""step: send StepParams (+ optional wind / tether length) -> receive
StepParams with the OPTIMIZED trajectory. Blocks ~10-20 s."""
function step(params::StepParams; wind = nothing, distance_radial = nothing)
    payload = as_dict(params)
    wind !== nothing && (payload["wind"] = wind)
    distance_radial !== nothing && (payload["distance_radial"] = distance_radial)
    return JSON3.read(post("/step", payload), StepParams)
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
winch = WinchParams("reelout", 0.11, 8.0, 1000.0, 8400.0)

reply = init(InitParams("uwe-sim-1", 600.0, winch, guess);
             wind = Dict("model_type" => "logarithmic", "U_ref" => 8.0,
                         "z_ref" => 100.0, "z0" => 0.03),
             distance_radial = 200.0,
             extras = Dict("sim_parameters" => Dict(
                 "input_depower" => 1.6, "reg_weight" => 1.0,
                 "detect_simple_bounds" => true)))
println("init ok: $(reply.name), starting path $(length(reply.trajectory.azimuth)) pts")

# First optimization (blocking; reply contains the optimized path)
result = step(StepParams(winch, reply.trajectory))
println("optimized: $(length(result.trajectory.azimuth)) pts, ",
        "elevation $(round(minimum(result.trajectory.elevation); digits=1))° – ",
        "$(round(maximum(result.trajectory.elevation); digits=1))°")

# ... fly the kite along result.trajectory ...

# Later, refresh with the current conditions from your simulation:
result = step(StepParams(winch, result.trajectory);
              wind = Dict("model_type" => "tabulated",
                          "heights" => [10.0, 100.0, 300.0],
                          "speeds" => [5.5, 8.0, 9.3]),
              distance_radial = 220.0)   # current tether length
println("refreshed trajectory received")
