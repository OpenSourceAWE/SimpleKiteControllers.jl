# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Julia client for the AWETrim reelout flight-path optimizer (REST). `include`
# it and call the functions; it leaves no state of its own behind.
#
#     include("awetrim_client.jl")
#     ensure_server()
#     reply  = opt_init(InitParams(; name = "run-1", length = 200.0, ...))
#     result = opt_step(StepParams(200.0, winch, reply.trajectory))
#
# Contract: POST /init receives and replies `InitParams`, POST /step receives and
# replies `StepParams` (blocking by default — the reply carries the optimized
# path). A reply trajectory is CLOSED (last point == first). Angles are DEGREES
# in these structs, and the physical state travels in them: the tether length is
# the `length` field, the wind the `inflow_conditions` field of `InitParams`.
# Call-mode knobs are keyword arguments instead — `opt_step(p; wait = false)`
# returns as soon as the solve is queued, so the caller keeps flying while the
# server optimizes.
#
# The endpoint functions are named `opt_*` rather than `init`/`step`/`status`
# because a script that flies an optimized path also does `using V3Kite`, whose
# exported `init` builds the kite model. A second `init` in `Main` is then not a
# shadow but an error ("function V3Kite.init must be explicitly imported to be
# extended").
#
# Winch coupling: the optimizer maps `v_set = kv*sqrt(force)` onto its radial
# force model, so the path it returns assumes exactly the winch behaviour that
# `winch_from_wc` sends it.
#
# Infeasibility: an impossible request (too much force demanded, too little
# wind, a winch too stiff to reel out at the optimum) comes back as HTTP 422
# with the solver's message; the previous trajectory stays available through
# `opt_trajectory`.

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using HTTP, JSON3, StructTypes
using Dates: now, format
using YAML
using SimpleKiteControllers: with_file_lock

const SKC_ROOT = normpath(joinpath(@__DIR__, ".."))
"Default server address; every function below takes `url` to override it."
const AWETRIM_URL = "http://127.0.0.1:8000"

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
# Failed-request cache
#
# A solve that FAILS is the expensive one: it runs to IPOPT's iteration cap
# (measured 2026-08-18 at 180 m: 2870 evaluations, 81 s, "Max Iterations") where a
# converged one takes 75 evaluations and 1.5 s — and in `reopt_blocking` mode the
# simulation is held for every second of it. Re-running a script therefore pays
# the full price again for a request that is already known not to work.
#
# Caching them is safe because the failures are NOT flaky. Measured on the same
# day: two attempts from the menu and one replay of an identical request all
# failed the same way, and the converged neighbours returned a bit-identical
# `input_depower`. What varies between runs is which optimum a CONVERGED solve
# picks, not whether it converges.
#
# The key is exact, and deliberately so. The failures are isolated pockets in
# tether length, not regions: 180.0 m converges where 180.00027 m throws, and
# 179.0 and 185.0 m converge where 181.0 throws. Bucketing the length would cache
# a failure against a length that works, which is a worse bug than the cost this
# saves. Everything else the server sees is in the key too, so a different wind,
# winch, guess or solver flag is a different request.

"""
The file recording failed optimizer requests, shared by every script and every
sweep worker on this machine. Delete it to retry them all.
"""
const OPT_FAILURE_CACHE = joinpath(SKC_ROOT, "output", "opt_failure_cache.yaml")

"""
    opt_request_key(p::InitParams) -> String

Identity of a request as the SERVER sees it: every field of `p` except `name`,
which is the caller's label for the run and not part of the problem.

`hash` is not stable across Julia versions, so an upgrade invalidates the cache.
That is a cache MISS — one wasted solve, then the entry is rewritten — never a
false hit, since a key that does not match is simply not found.
"""
function opt_request_key(p::InitParams)
    h = hash((p.length, p.input_depower, p.reg_weight, p.detect_simple_bounds,
              p.winch_params.k_v, p.winch_params.f_min, p.winch_params.f_max,
              p.inflow_conditions.wind_speed, p.inflow_conditions.wind_direction,
              p.inflow_conditions.profile_law, p.inflow_conditions.z0,
              p.trajectory.azimuth, p.trajectory.elevation))
    return string(h; base = 16)
end

"""
    opt_failures(; file = OPT_FAILURE_CACHE) -> Dict{String, Any}

Every recorded failure, keyed by [`opt_request_key`](@ref). An absent or
unreadable file is an empty cache: this must never be the reason a run stops.
"""
function opt_failures(; file::AbstractString = OPT_FAILURE_CACHE)
    isfile(file) || return Dict{String, Any}()
    try
        d = YAML.load_file(file)
        d isa Dict && haskey(d, "entries") && d["entries"] isa Dict ?
            d["entries"] : Dict{String, Any}()
    catch exc
        @warn "Ignoring an unreadable optimizer-failure cache at $file." exception = exc
        Dict{String, Any}()
    end
end

"""
    opt_failed_before(p::InitParams; file = OPT_FAILURE_CACHE) -> Nothing or Dict

The recorded failure for this exact request, or `nothing` if it has never failed.
"""
function opt_failed_before(p::InitParams; file::AbstractString = OPT_FAILURE_CACHE)
    get(opt_failures(; file), opt_request_key(p), nothing)
end

"""
    record_opt_failure!(p::InitParams, reason; file = OPT_FAILURE_CACHE)

Record that this request failed, so no later run repeats it. Under
`with_file_lock`, so parallel sweep workers cannot lose each other's entries.
The `length` and the guess elevation are stored next to the key in plain text,
because a cache nobody can read is a cache nobody trusts.
"""
function record_opt_failure!(p::InitParams, reason::AbstractString;
                             file::AbstractString = OPT_FAILURE_CACHE)
    mkpath(dirname(file))
    with_file_lock(file * ".lock") do
        entries = opt_failures(; file)
        entries[opt_request_key(p)] = Dict(
            "length_m" => p.length,
            "guess_el_center_deg" => round(sum(p.trajectory.elevation) /
                                           length(p.trajectory.elevation); digits = 2),
            "wind_speed_m_s" => p.inflow_conditions.wind_speed,
            "reason" => String(reason),
            "when" => format(now(), "yyyy-mm-dd HH:MM:SS"))
        open(file, "w") do io
            println(io, "# Optimizer requests known to fail, written by \
                         examples/awetrim_client.jl.")
            println(io, "# They are NOT retried while they are listed here — \
                         delete this file to retry them all,")
            println(io, "# or drop a single entry to retry just that one. \
                         `length_m` is exact on purpose: 180.0 m")
            println(io, "# converges where 180.00027 m does not.")
            YAML.write(io, Dict("entries" => entries))
        end
    end
    return nothing
end

"""
    clear_opt_failures(; file = OPT_FAILURE_CACHE) -> Bool

Forget every recorded failure. `true` if there was a cache to remove.
"""
function clear_opt_failures(; file::AbstractString = OPT_FAILURE_CACHE)
    had = isfile(file)
    rm(file; force = true)
    return had
end

# ---------------------------------------------------------------------------
# Transport helpers
# ---------------------------------------------------------------------------
function post(path, payload; url = AWETRIM_URL, timeout = 600)
    # retry_non_idempotent: the server drops idle keep-alive connections, so a pooled
    # socket can be dead by the next call; HTTP.jl only retries POST if it asked for it
    # (safe: it retries only when no response bytes arrived, i.e. the server never saw it)
    response = HTTP.post(url * path, ["Content-Type" => "application/json"];
                         body = JSON3.write(payload), read_idle_timeout = timeout,
                         retry_non_idempotent = true)
    return response.body
end

get_json(path; url = AWETRIM_URL) = JSON3.read(HTTP.get(url * path).body, Dict{String,Any})

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

# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------
"""
    opt_init(params::InitParams; url = AWETRIM_URL) -> InitParams

Build the optimizer's model for these conditions and fit the starting path.
Sends `InitParams` (tether length, inflow conditions, initial guess and the
solver knobs) and returns what the server accepted, with `trajectory` replaced
by the fitted starting path. No optimization happens yet — that is
[`opt_step`](@ref).
"""
function opt_init(params::InitParams; url = AWETRIM_URL)
    return as_init_params(JSON3.read(post("/init", params; url), Dict{String,Any}))
end

"""
    opt_step(params::StepParams; url = AWETRIM_URL, inflow_conditions = nothing,
             wait = true)

Optimize the path for the current tether length (and optionally a new inflow).
Blocks ~10-20 s and returns `StepParams` carrying the optimized trajectory.

With `wait = false` the server accepts the job and replies immediately; the
return value is the step index instead. Poll [`opt_status`](@ref) until its
`"state"` leaves `"solving"`, then collect the result with
[`opt_trajectory`](@ref) — which is in RADIANS, unlike the degrees of the
blocking reply. While a solve runs, and after one that failed, the server keeps
serving the PREVIOUS path, so a caller is never left without a curve to fly.
"""
function opt_step(params::StepParams; url = AWETRIM_URL,
                  inflow_conditions = nothing, wait = true)
    payload = as_dict(params)
    inflow_conditions !== nothing && (payload["inflow_conditions"] = as_dict(inflow_conditions))
    payload["wait"] = wait
    reply = JSON3.read(post("/step", payload; url), Dict{String,Any})
    wait || return reply["step_index"]::Int
    return StepParams(reply["length"], as_winch(reply["winch_params"]),
                      as_traj(reply["trajectory"]))
end

"""
    opt_status(url = AWETRIM_URL) -> Dict

Server state (`"uninitialized"`/`"ready"`/`"solving"`/`"converged"`/`"failed"`),
the step counters and the metrics of the last solve.
"""
opt_status(url::AbstractString = AWETRIM_URL) = get_json("/status"; url)

"""
    opt_trajectory(; url = AWETRIM_URL, resimulate = false) -> Dict

The last optimized trajectory in full, which the `/init` and `/step` replies do
not carry: the dense per-node table (`t`, `s`, `azimuth`, `elevation`,
`azimuth_dot`, `elevation_dot`, `distance_radial`, `speed_radial`, `s_dot`,
`tension_tether_ground`, `input_steering`, `input_depower`), the `spline` block
with its `downloops` flag, `metrics` and `optimized_parameters`.

**The table is in RADIANS**, unlike the degrees of every struct in this file.

`resimulate = true` adds a full timeseries by re-flying the pattern; it is
slower and is rejected with 409 while a solve is running.
"""
function opt_trajectory(; url = AWETRIM_URL, resimulate = false)
    return get_json("/trajectory?resimulate=$(resimulate)"; url)
end

# ---------------------------------------------------------------------------
# The conditions of a run, as the optimizer wants them
# ---------------------------------------------------------------------------
# Deliberately duck-typed rather than annotated with `KiteUtils.Settings` and
# `WinchControllers.WCSettings`: this file stays usable from a bare REPL that
# has loaded neither.

"""
    inflow_from_settings(set) -> InflowConditions

The wind of a run, from the `KiteUtils.Settings` of its system project — the
same file the plant is built from, so the path cannot be optimized for a wind
the kite does not fly in. Field for field: `v_wind`, `upwind_dir` (both are "the
direction the wind comes FROM", 0 = North, clockwise), `profile_law`, `alpha`,
`z0`, and `heights`/`speeds` for the fitted CUSTOM_* laws 4-6.

`turbulence` is left at 0 on purpose: the server accepts and echoes it but does
NOT use it — the optimizer is deterministic and works on the mean profile.
Passing a run's `use_turbulence` would imply otherwise.
"""
function inflow_from_settings(set)
    isapprox(set.h_ref, 6.0; atol = 1e-6) ||
        @warn "The optimizer reads wind_speed at 6 m, but this project's h_ref is \
               $(set.h_ref) m — the reference speed is not comparable."
    samples = set.profile_law >= 4 ?
              (; heights = Float64.(set.heights), speeds = Float64.(set.speeds)) : (;)
    return InflowConditions(; wind_speed = set.v_wind,
                            wind_direction = mod(set.upwind_dir, 360.0),
                            profile_law = set.profile_law,
                            alpha = set.alpha, z0 = set.z0,
                            turbulence = 0.0, samples...)
end

"""
    winch_from_wc(wc) -> WinchParams

The winch law of a run, from the `WinchControllers.WCSettings` its
`WinchController` is built from: `kv`, `f_low` and `f_high` of
`data/wc_settings.yaml`. The optimizer maps `v_set = kv*sqrt(force)` onto its
radial force model, so these three numbers are what makes the optimized path the
path for THIS ground station.

A winch too stiff to reach the optimal reel-out speed within `f_high` makes the
problem infeasible and the server answers 422 — `kv*sqrt(f_high)` is the speed
ceiling to compare against roughly a third of the wind speed.
"""
winch_from_wc(wc) = WinchParams("reelout", wc.kv, wc.f_low, wc.f_high)

# ---------------------------------------------------------------------------
# The server process
# ---------------------------------------------------------------------------
"""
    server_running(url; timeout = 2) -> Bool

`true` if an AWETrim server answers `GET \$url/health`. Not a liveness check on a
process: a foreground server, one started by `bin/run_server start` and one on
another machine are all equally usable, and all three answer this.
"""
function server_running(url::AbstractString = AWETRIM_URL; timeout = 2)
    try
        response = HTTP.get(url * "/health"; connect_timeout = timeout,
                            request_timeout = timeout, retry = false,
                            status_exception = false)
        return response.status == 200
    catch
        return false
    end
end

"""
    ensure_server(url = AWETRIM_URL; autostart = true, verbose = true)

Return once an AWETrim server answers at `url`, starting a detached one with
`bin/run_server start` if none does. Throws if the server cannot be started or
does not come up — `bin/run_server` blocks until `/health` answers and exits
non-zero otherwise, so the waiting and the diagnosis are its job, not this one's.

`autostart = false` turns a missing server into an error instead, which is what a
run against a server on another machine wants.
"""
function ensure_server(url::AbstractString = AWETRIM_URL;
                       autostart = true, verbose = true)
    server_running(url) && return url
    autostart || error("No AWETrim server at $url, and autostart is off. Start one \
                        with `bin/run_server start`.")
    m = match(r"^https?://([^:/]+)(?::(\d+))?", url)
    isnothing(m) && error("Cannot parse a host and port out of $url.")
    host = m[1]
    port = isnothing(m[2]) ? "8000" : m[2]
    verbose && @info "No AWETrim server at $url — starting one (bin/run_server start)."
    script = joinpath(SKC_ROOT, "bin", "run_server")
    run(Cmd(`$script start --host $host --port $port`; dir = SKC_ROOT))
    server_running(url) ||
        error("bin/run_server reported success, but $url/health still does not answer.")
    return url
end

"""
    stop_server(url = AWETRIM_URL; verbose = true) -> Bool

Stop the detached server, through `bin/run_server stop`. Returns `true` once
nothing answers at `url` any more.

Only reaches a server that `bin/run_server start` (or [`ensure_server`](@ref))
launched, since that is the one whose pid it recorded: a server started in the
foreground belongs to the terminal that runs it and is stopped there with Ctrl-C,
and one on another machine is not this script's to stop. Both cases leave a
server answering at `url`, which is what the `false` return reports.
"""
function stop_server(url::AbstractString = AWETRIM_URL; verbose = true)
    script = joinpath(SKC_ROOT, "bin", "run_server")
    run(Cmd(`$script stop`; dir = SKC_ROOT))
    stopped = !server_running(url)
    if !stopped && verbose
        @warn "Something still answers at $url — a server started in the foreground, \
               or one this machine did not start."
    end
    return stopped
end
