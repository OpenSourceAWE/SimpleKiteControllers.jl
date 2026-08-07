# Julia client for the AWETrim reelout flight-path optimizer (REST).
#
# Dependencies:  ] add HTTP JSON3 StructTypes
#
# Contract: POST /init receives and replies InitParams; POST /step receives
# and replies StepParams (both blocking — the reply contains the result).
# The reply trajectory is CLOSED (last point == first) with the same number
# of points you sent. Angles in DEGREES here; the tether length is the
# `length` field of InitParams / StepParams and the wind is the
# `inflow_conditions` field of InitParams — nothing travels next to the
# structs.
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
using GLMakie, MakieControlPlots   # for the pattern plot at the end

# ---------------------------------------------------------------------------
# The shared structs (as agreed)
# ---------------------------------------------------------------------------
Base.@kwdef struct InflowConditions
    wind_speed::Float64      # in m/s at 6 m height
    wind_direction::Float64  # in degrees, 0 = North, 90 = East
    profile_law::Int64       # 0=CONST, 1=EXP, 2=LOG, 3=EXPLOG, 4=CUSTOM_LOG, 5=CUSTOM_EXP, 6=CUSTOM_JET
    # the custom profiles are fitted using the heights and speeds given in the heights and speeds fields
    # CUSTOM_JET: u(z) = u_bg(z) + U_J * exp(-(z - z_c)^2 / (2*sigma^2))
    # the following fields are optional; the defaults given below are the server defaults
    alpha::Float64 = 0.08163                # exponent of the wind profile law
    z0::Float64 = 0.0002                    # surface roughness                                     [m]
    turbulence::Float64 = 0.0               # in [0, 1], 0 = no turbulence, 1 = full turbulence
    heights::Vector{Float64} = [6.0]        # heights at which the wind speed is given
    speeds::Vector{Float64} = [wind_speed]  # wind speeds at the given heights
end

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
    length::Float64                    # initial length of the tether
    winch_params::WinchParams
    inflow_conditions::InflowConditions
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

StructTypes.StructType(::Type{InflowConditions}) = StructTypes.Struct()
StructTypes.StructType(::Type{WinchParams}) = StructTypes.Struct()
StructTypes.StructType(::Type{Trajectory}) = StructTypes.Struct()
StructTypes.StructType(::Type{InitParams}) = StructTypes.Struct()
StructTypes.StructType(::Type{StepParams}) = StructTypes.Struct()

# ---------------------------------------------------------------------------
# Transport helpers
# ---------------------------------------------------------------------------
const BASE = "http://127.0.0.1:8000"

function post(path, payload; timeout = 600)
    # retry_non_idempotent: the server drops idle keep-alive connections, so a pooled
    # socket can be dead by the next call; HTTP.jl only retries POST if it asked for it
    # (safe: it retries only when no response bytes arrived, i.e. the server never saw it)
    response = HTTP.post(BASE * path, ["Content-Type" => "application/json"];
                         body = JSON3.write(payload), read_idle_timeout = timeout,
                         retry_non_idempotent = true)
    return response.body
end

as_dict(x) = JSON3.read(JSON3.write(x), Dict{String,Any})
as_winch(d) = WinchParams(d["mode"], d["k_v"], d["f_min"], d["f_max"])
as_traj(d) = Trajectory(Float64.(d["azimuth"]), Float64.(d["elevation"]))

function as_inflow(d)
    # the server echoes heights/speeds only if the request carried them
    samples = d["heights"] === nothing ? (;) :
              (; heights = Float64.(d["heights"]), speeds = Float64.(d["speeds"]))
    return InflowConditions(; wind_speed = d["wind_speed"], wind_direction = d["wind_direction"],
                            profile_law = d["profile_law"], alpha = d["alpha"], z0 = d["z0"],
                            turbulence = d["turbulence"], samples...)
end

function as_init_params(reply)
    return InitParams(reply["name"], reply["length"],
                      as_winch(reply["winch_params"]),
                      as_inflow(reply["inflow_conditions"]),
                      as_traj(reply["trajectory"]),
                      reply["input_depower"], reply["reg_weight"],
                      reply["detect_simple_bounds"])
end

"""init: send InitParams (incl. tether length, inflow conditions and solver
knobs) -> InitParams."""
function init(params::InitParams)
    return as_init_params(JSON3.read(post("/init", params), Dict{String,Any}))
end

"""step: send StepParams (incl. the current tether length, + optional new
inflow) -> receive StepParams with the OPTIMIZED trajectory. Blocks ~10-20 s."""
function step(params::StepParams; inflow_conditions = nothing)
    payload = as_dict(params)
    inflow_conditions !== nothing && (payload["inflow_conditions"] = as_dict(inflow_conditions))
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

# 5.2 m/s at 6 m height, wind from the west, logarithmic profile
# (~8 m/s at 100 m with this roughness).
inflow = InflowConditions(wind_speed = 5.2, wind_direction = 270.0, profile_law = 2, z0 = 0.03)

# 200 m of tether; the solver knobs are left at their defaults
reply = init(InitParams(name = "uwe-sim-1", length = 200.0,
                        winch_params = winch, inflow_conditions = inflow,
                        trajectory = guess))
@info ("init ok: $(reply.name), starting path $(length(reply.trajectory.azimuth)) pts")

# First optimization (blocking; reply contains the optimized path)
result = step(StepParams(200.0, winch, reply.trajectory))
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

# Later, refresh with the current conditions from your simulation. A measured
# profile is sent as a CUSTOM_* law: the heights/speeds samples are fitted
# (here: log law + Gaussian jet).
# result = step(StepParams(220.0, winch, result.trajectory);   # current length
#               inflow_conditions = InflowConditions(wind_speed = 8.4, wind_direction = 265.0,
#                                                    profile_law = 6,
#                                                    heights = [10.0, 50.0, 100.0, 200.0, 300.0],
#                                                    speeds = [5.5, 7.4, 8.0, 9.3, 8.6]))
println("refreshed trajectory received")
