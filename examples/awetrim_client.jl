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
# Contract: POST /init receives `InitParams` and replies `InitReply`, POST /step
# receives `StepParams` and replies `StepReply` (blocking by default — the reply
# carries the optimized path). The replies are the request structs plus what only
# the server knows: the `depower` the path was optimized for, the
# `min_turn_radius` and `pattern_limits` it was optimized under, and the solve
# `metrics`. A reply trajectory is CLOSED (last point == first). Angles are
# DEGREES in these structs, and the physical state travels in them: the tether
# length is the `length` field, the wind the `inflow_conditions` field of
# `InitParams`. Call-mode knobs are keyword arguments instead —
# `opt_step(p; wait = false)` returns as soon as the solve is queued, so the
# caller keeps flying while the server optimizes, and `max_iter` caps the solve.
#
# Depower: the server OPTIMIZES `input_depower` by default, so the path that
# comes back is only flyable at the depower it was optimized for — read it from
# `reply.depower.value` (measured: 1.6 sent, 1.5186 returned). Send
# `depower = DepowerSpec("fixed", u_p)` to pin it to what the run actually flies,
# or `"profile"` for a per-node depower schedule.
#
# Turn radius and pattern box: `min_turn_radius` [m] makes the optimizer respect
# the kite's own turning limit (1/(c1*u_s_max) for a psi_dot = c1*v_a*u_s kite)
# instead of only its internal steering bounds, and `pattern_limits` bounds where
# the path may go in azimuth/elevation. Both replace checking-and-discarding a
# reply after the solve: the constrained solve is the one that costs nothing.
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
using Printf: @sprintf
using YAML
using SimpleKiteControllers: with_file_lock, turn_rate_coeffs

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

Base.@kwdef struct WinchParams
    mode::String      # "reelout" ("reelin" not supported yet)
    k_v::Float64      # v_set = k_v * sqrt(force)
    f_min::Float64    # minimum winch force [N]
    f_max::Float64    # maximum winch force [N]
    # Past `f_max` the controller holds the force while the reel-out speed keeps
    # rising, up to the winch's power limit — so the speed cap is a genuine input
    # and not derivable from `k_v`/`f_max`. Give either; both `nothing` leaves the
    # optimizer's own reel-speed bound (10 m/s) in force, which with this repo's
    # kv = 0.0408 and f_high = 8000 N is never reached (3.65 m/s).
    v_max::Union{Float64, Nothing} = nothing   # maximum reel-out speed [m/s]
    p_max::Union{Float64, Nothing} = nothing   # maximum winch power [W]; v_max = p_max/f_max
end

"The four-field winch of the original contract; `v_max`/`p_max` stay unset."
WinchParams(mode::AbstractString, k_v, f_min, f_max) =
    WinchParams(; mode = String(mode), k_v = Float64(k_v),
                f_min = Float64(f_min), f_max = Float64(f_max))

"""
    DepowerSpec(mode = "optimize", value = nothing)

How the server treats the depower `l_dp` while it solves. `"optimize"` (the
server default) varies it and reports what it landed on, `"fixed"` pins it to
`value` — the setting the run actually flies — and `"profile"` optimizes one
value per node, giving a depower schedule along the path.

Pinning is not free: measured 2026-08-18, the ROM's predicted power collapses as
the tape is let out (4486 W at 1.6 m, 1873 W at 1.8 m) and 2.0 m is infeasible
outright, so `"fixed"` at the flown depower buys agreement between the two models
with a much smaller feasible set. `value` is `nothing` in `"optimize"` mode to
start from `input_depower`.
"""
Base.@kwdef struct DepowerSpec
    mode::String = "optimize"                # "fixed" | "optimize" | "profile"
    value::Union{Float64, Nothing} = nothing # fixed value, or the starting one
end

"""
    DepowerReply

The depower the returned path was optimized for — FLY THIS, or the reported
metrics are not achievable. `profile` is filled in `"profile"` mode only, aligned
point for point with the reply trajectory; `value` is then its mean.
"""
struct DepowerReply
    mode::String
    value::Float64
    profile::Union{Vector{Float64}, Nothing}
end

"""
Total offset [rel_depower units] between the two models at equal power, measured
2026-08-18 (`docs/steering_depower.md`): V3Kite (VSM) needs 0.12 MORE `rel_depower`
than AWETrim's ROM for the same power (slopes -15.2 vs -17.8 per unit, matched at
6538 W: u_p = 0.2915 against u_p = 0.1699). 0.08 of that (0.4 m of tape) is the two
models' own tape-length zero (`0.6 + 5*u_p` here, `0.2 + 5*u_p` in V3Kite); the
remaining 0.04 (0.2 m) is genuine, unexplained aerodynamic disagreement — a single
measured point, not a fitted curve.
"""
const AWETRIM_V3KITE_DEPOWER_OFFSET = 0.12

"""
    awetrim_depower_to_v3kite(l_dp) -> Float64

Convert an AWETrim `l_dp` [m] (`input_depower`, `l_dp = 0.6 + 5*u_p`) into the
V3Kite `rel_depower` expected to fly at the SAME power, i.e. `(l_dp - 0.6)/5 +
`[`AWETRIM_V3KITE_DEPOWER_OFFSET`](@ref). This is the FULL correction — do not
also subtract the 0.4 m calibration offset separately, it is already inside the
0.12.
"""
awetrim_depower_to_v3kite(l_dp) = (l_dp - 0.6) / 5 + AWETRIM_V3KITE_DEPOWER_OFFSET

"""
    PatternLimits(; azimuth_max, elevation_min, elevation_max, azimuth_amplitude_min)

A box, in DEGREES, on where the optimized pattern may go. The server bounds the
B-spline's control coefficients, so by the convex-hull property the limits hold
along the whole curve and not merely at its nodes.

Every field is optional and `nothing` keeps the optimizer's own default for it
(|azimuth| <= 45.8°, 0.6° <= elevation <= 51.6°, no amplitude floor).
`azimuth_amplitude_min` is the figure's half-width and guards the degenerate
zero-width collapse; `elevation_max` guards the run-away-to-zenith basin — the
two bad basins a failed re-optimization falls into. On `/step` the struct
replaces the session's limits as a whole, so an all-`nothing` `PatternLimits()`
CLEARS them.
"""
Base.@kwdef struct PatternLimits
    azimuth_max::Union{Float64, Nothing} = nothing           # |azimuth| <= this [deg]
    elevation_min::Union{Float64, Nothing} = nothing         # elevation >= this [deg]
    elevation_max::Union{Float64, Nothing} = nothing         # elevation <= this [deg]
    azimuth_amplitude_min::Union{Float64, Nothing} = nothing # half-width >= this [deg]
end

"Metrics of one solve; `turn_radius_min_m` is the tightest PHYSICAL turn radius
of the returned path [m] and is `nothing` when it could not be evaluated."
Base.@kwdef struct SolveMetrics
    energy_J::Float64
    total_time_s::Float64
    avg_power_W::Float64
    turn_radius_min_m::Union{Float64, Nothing} = nothing
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
    input_depower::Float64 = 1.6       # depower setting; only a SEED unless depower.mode == "fixed"
    reg_weight::Float64 = 1.0          # regularization weight
    detect_simple_bounds::Bool = true  # solver flag
    # `nothing` leaves each of these to the server's default: depower is
    # OPTIMIZED from `input_depower`, the turn radius is unconstrained beyond the
    # optimizer's own steering bounds, and the pattern box is the optimizer's.
    depower::Union{DepowerSpec, Nothing} = nothing
    min_turn_radius::Union{Float64, Nothing} = nothing   # [m], 0/nothing = off
    pattern_limits::Union{PatternLimits, Nothing} = nothing
end

Base.@kwdef struct StepParams
    length::Float64                    # current length of the tether
    winch_params::WinchParams
    # `nothing` is a WARM START: the server keeps the pattern it already has, and
    # the solve starts from the previous optimum re-anchored to `length` (it moves
    # `r0` and shifts its node-wise warm start by the same delta). A trajectory
    # here REPLACES that seed — the curve is refitted into the pattern B-spline —
    # which is what a re-`/init` does too, only without dropping the session.
    trajectory::Union{Trajectory, Nothing} = nothing
    # As in `InitParams`, but here `nothing` means "keep what the session has":
    # these are set once and stay set until a later step changes them. Send
    # `min_turn_radius = 0.0` to drop the constraint and `PatternLimits()` to
    # clear the box.
    depower::Union{DepowerSpec, Nothing} = nothing
    min_turn_radius::Union{Float64, Nothing} = nothing
    pattern_limits::Union{PatternLimits, Nothing} = nothing
end

"The three-field step of the original contract; the session keeps its depower,
turn-radius and pattern limits."
StepParams(length::Real, winch_params::WinchParams, trajectory::Trajectory) =
    StepParams(; length = Float64(length), winch_params, trajectory)

"A WARM-STARTED step: no trajectory, so the solve starts from the session's own
previous optimum, re-anchored to `length`."
StepParams(length::Real, winch_params::WinchParams) =
    StepParams(; length = Float64(length), winch_params)

"""
    InitReply

What `/init` accepted: the [`InitParams`](@ref) it built the session from, with
`trajectory` replaced by the FITTED starting path, plus the depower mode and the
limits the coming solves will run under. No optimization has happened yet.
"""
struct InitReply
    name::String
    length::Union{Float64, Nothing}
    winch_params::Union{WinchParams, Nothing}
    inflow_conditions::Union{InflowConditions, Nothing}
    trajectory::Trajectory
    input_depower::Union{Float64, Nothing}
    reg_weight::Union{Float64, Nothing}
    detect_simple_bounds::Union{Bool, Nothing}
    depower::Union{DepowerReply, Nothing}
    min_turn_radius::Union{Float64, Nothing}
    pattern_limits::Union{PatternLimits, Nothing}
    state::String
    n_points::Union{Int, Nothing}
end

"""
    StepReply

What one blocking `/step` produced: the OPTIMIZED `trajectory`, the `depower` it
assumes (fly both, or `metrics` is not achievable), the limits it was optimized
under, and the solve `metrics`.
"""
struct StepReply
    length::Union{Float64, Nothing}
    winch_params::Union{WinchParams, Nothing}
    trajectory::Trajectory
    depower::Union{DepowerReply, Nothing}
    min_turn_radius::Union{Float64, Nothing}
    pattern_limits::Union{PatternLimits, Nothing}
    state::String
    step_index::Int
    metrics::SolveMetrics
end

StructTypes.StructType(::Type{InflowConditions}) = StructTypes.Struct()
StructTypes.StructType(::Type{WinchParams}) = StructTypes.Struct()
StructTypes.StructType(::Type{Trajectory}) = StructTypes.Struct()
StructTypes.StructType(::Type{InitParams}) = StructTypes.Struct()
StructTypes.StructType(::Type{StepParams}) = StructTypes.Struct()
StructTypes.StructType(::Type{DepowerSpec}) = StructTypes.Struct()
StructTypes.StructType(::Type{PatternLimits}) = StructTypes.Struct()

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

EVERY field has to be in here. A field left out is a false hit — a failure
recorded under one turn radius, depower mode or pattern box would block a
request that differs in exactly that, which is the one bug this cache must not
have. Hence the `"v2-"` prefix: it was grown on 2026-08-19 for the depower /
turn-radius / pattern-limits fields (and for `alpha` and the `heights`/`speeds`
of the fitted profiles, which the first version also missed), and the prefix
retires the entries written under the narrower key rather than reusing them.

`hash` is not stable across Julia versions either, so an upgrade invalidates the
cache. That is a cache MISS — one wasted solve, then the entry is rewritten —
never a false hit, since a key that does not match is simply not found.
"""
function opt_request_key(p::InitParams)
    w = p.winch_params
    c = p.inflow_conditions
    d = p.depower
    b = p.pattern_limits
    h = hash((p.length, p.input_depower, p.reg_weight, p.detect_simple_bounds,
              w.mode, w.k_v, w.f_min, w.f_max, w.v_max, w.p_max,
              c.wind_speed, c.wind_direction, c.profile_law, c.alpha, c.z0,
              c.turbulence, c.heights, c.speeds,
              p.trajectory.azimuth, p.trajectory.elevation,
              d === nothing ? nothing : (d.mode, d.value),
              p.min_turn_radius,
              b === nothing ? nothing : (b.azimuth_max, b.elevation_min,
                                         b.elevation_max, b.azimuth_amplitude_min)))
    return "v2-" * string(h; base = 16)
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
            "min_turn_radius_m" => something(p.min_turn_radius, 0.0),
            "depower_mode" => p.depower === nothing ? "optimize" : p.depower.mode,
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
"""
    stale_conn_error(err) -> Bool

Does `err` look like a POOLED SOCKET the server had already closed, rather than a
real transport failure? `SystemError` (EPIPE on the write), `EOFError` and
`HTTP.ParseError` ("unexpected EOF while reading HTTP/1 data") are the three faces
of it seen here; the error usually arrives wrapped, so nested `.error`/`.ex`
payloads are unwrapped and the message is matched as a backstop.
"""
function stale_conn_error(err, depth = 0)
    depth > 4 && return false
    err isa Base.SystemError && return true
    err isa EOFError && return true
    isdefined(HTTP, :ParseError) && err isa HTTP.ParseError && return true
    for f in (:error, :ex, :captured, :task)
        hasproperty(err, f) && stale_conn_error(getproperty(err, f), depth + 1) &&
            return true
    end
    msg = try sprint(showerror, err) catch; string(err) end
    return occursin("Broken pipe", msg) || occursin("ECONNRESET", msg) ||
           occursin("connection reset", msg) || occursin("unexpected EOF", msg)
end

function post(path, payload; url = AWETRIM_URL, timeout = 600, attempts = 3)
    # The server closes idle keep-alive connections long before the run's next
    # request (a re-optimization is one lap apart, ~13 s), so a pooled socket is
    # usually dead by the time it is reused — and HTTP.jl v2 does NOT cover that for
    # us here: its transparent "reused connection died" retry is gated on
    # `_retryable_method`, which is GET/HEAD/OPTIONS/TRACE/QUERY only, so every POST
    # in this file was exposed. `retry_non_idempotent` feeds a different layer (the
    # status/policy controller) and does not reach it. Measured 2026-08-20: three
    # `SystemError: write: Broken pipe` in one run, four in the next, each one
    # counted as a failed solve — a run then flies a path it never asked for, and its
    # numbers are not comparable with a clean run's.
    #
    # So the pool is emptied before each request and the retry is done here. Both are
    # safe for this API: /init and /step are re-sent only when the socket died, and
    # re-sending either recomputes and overwrites what the server holds under the
    # optimization's name — nothing accumulates.
    body = JSON3.write(payload)
    for attempt in 1:attempts
        isdefined(HTTP, :close_idle_connections!) && HTTP.close_idle_connections!()
        try
            response = HTTP.post(url * path, ["Content-Type" => "application/json"];
                                 body, read_idle_timeout = timeout,
                                 retry_non_idempotent = true)
            return response.body
        catch err
            (attempt < attempts && stale_conn_error(err)) || rethrow()
            @info "POST $path: $(first(split(sprint(showerror, err), '\n'))) on \
                   attempt $attempt of $attempts — the pooled socket was dead. \
                   Retrying on a fresh connection."
            sleep(0.2)
        end
    end
end

get_json(path; url = AWETRIM_URL) = JSON3.read(HTTP.get(url * path).body, Dict{String,Any})

as_dict(x) = JSON3.read(JSON3.write(x), Dict{String,Any})
as_traj(d) = Trajectory(Float64.(d["azimuth"]), Float64.(d["elevation"]))

# A field the server may leave out entirely (an older or newer one) reads the
# same as one it sends as null: absent means "not set", never an error.
opt_get(d, key) = d === nothing ? nothing : get(d, key, nothing)
opt_float(d, key) = (v = opt_get(d, key); v === nothing ? nothing : Float64(v))

as_winch(d) = WinchParams(; mode = d["mode"], k_v = d["k_v"],
                          f_min = d["f_min"], f_max = d["f_max"],
                          v_max = opt_float(d, "v_max"),
                          p_max = opt_float(d, "p_max"))

function as_depower(d)
    d === nothing && return nothing
    profile = get(d, "profile", nothing)
    return DepowerReply(d["mode"], Float64(d["value"]),
                        profile === nothing ? nothing : Float64.(profile))
end

function as_pattern_limits(d)
    d === nothing && return nothing
    return PatternLimits(; azimuth_max = opt_float(d, "azimuth_max"),
                         elevation_min = opt_float(d, "elevation_min"),
                         elevation_max = opt_float(d, "elevation_max"),
                         azimuth_amplitude_min = opt_float(d, "azimuth_amplitude_min"))
end

as_metrics(d) = SolveMetrics(; energy_J = d["energy_J"],
                             total_time_s = d["total_time_s"],
                             avg_power_W = d["avg_power_W"],
                             turn_radius_min_m = opt_float(d, "turn_radius_min_m"))

function as_inflow(d)
    # the server echoes heights/speeds only if the request carried them
    samples = d["heights"] === nothing ? (;) :
              (; heights = Float64.(d["heights"]), speeds = Float64.(d["speeds"]))
    return InflowConditions(; wind_speed = d["wind_speed"], wind_direction = d["wind_direction"],
                            profile_law = d["profile_law"], alpha = d["alpha"], z0 = d["z0"],
                            turbulence = d["turbulence"], samples...)
end

maybe(f, d) = d === nothing ? nothing : f(d)

function as_init_reply(reply)
    return InitReply(reply["name"], opt_float(reply, "length"),
                     maybe(as_winch, opt_get(reply, "winch_params")),
                     maybe(as_inflow, opt_get(reply, "inflow_conditions")),
                     as_traj(reply["trajectory"]),
                     opt_float(reply, "input_depower"),
                     opt_float(reply, "reg_weight"),
                     opt_get(reply, "detect_simple_bounds"),
                     as_depower(opt_get(reply, "depower")),
                     opt_float(reply, "min_turn_radius"),
                     as_pattern_limits(opt_get(reply, "pattern_limits")),
                     reply["state"], opt_get(reply, "n_points"))
end

function as_step_reply(reply)
    return StepReply(opt_float(reply, "length"),
                     maybe(as_winch, opt_get(reply, "winch_params")),
                     as_traj(reply["trajectory"]),
                     as_depower(opt_get(reply, "depower")),
                     opt_float(reply, "min_turn_radius"),
                     as_pattern_limits(opt_get(reply, "pattern_limits")),
                     reply["state"], reply["step_index"],
                     as_metrics(reply["metrics"]))
end

# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------
"""
    opt_init(params::InitParams; url = AWETRIM_URL) -> InitReply

Build the optimizer's model for these conditions and fit the starting path.
Sends `InitParams` (tether length, inflow conditions, initial guess, the solver
knobs and the depower/turn-radius/pattern limits) and returns what the server
accepted, with `trajectory` replaced by the fitted starting path. No
optimization happens yet — that is [`opt_step`](@ref).

The reply also states what the coming solves will run under: `depower.mode`,
`min_turn_radius` and `pattern_limits`, each `nothing` where the server's own
default applies.
"""
function opt_init(params::InitParams; url = AWETRIM_URL)
    return as_init_reply(JSON3.read(post("/init", params; url), Dict{String,Any}))
end

"""
    opt_step(params::StepParams; url = AWETRIM_URL, inflow_conditions = nothing,
             wait = true, max_iter = nothing)

Optimize the path for the current tether length (and optionally a new inflow).
Blocks ~10-20 s and returns a [`StepReply`](@ref): the optimized `trajectory`,
the `depower` it was optimized for, the limits it respects and the solve
`metrics` (including `turn_radius_min_m`, the tightest physical radius of the
path that came back).

`max_iter` caps the solver's iterations for this step — the one lever against a
failing solve running to IPOPT's cap, measured at 81 s against 1.5 s for a
converged one.

Every step is WARM-STARTED from the session's previous optimum; what the
`trajectory` field does is REPLACE that seed with a curve of the caller's, refitted
into the pattern B-spline. Leaving it `nothing` therefore re-anchors the last
optimum to the new `length` and solves from there — the cheap solve — while
sending one is a cold start in everything but the session (see
`TrajOptSettings.use_step`).

With `wait = false` the server accepts the job and replies immediately; the
return value is the step index instead. Poll [`opt_status`](@ref) until its
`"state"` leaves `"solving"`, then collect the result with
[`opt_trajectory`](@ref) — which is in RADIANS, unlike the degrees of the
blocking reply. While a solve runs, and after one that failed, the server keeps
serving the PREVIOUS path, so a caller is never left without a curve to fly.
"""
function opt_step(params::StepParams; url = AWETRIM_URL,
                  inflow_conditions = nothing, wait = true, max_iter = nothing)
    payload = as_dict(params)
    inflow_conditions !== nothing && (payload["inflow_conditions"] = as_dict(inflow_conditions))
    max_iter !== nothing && (payload["max_iter"] = max_iter)
    payload["wait"] = wait
    reply = JSON3.read(post("/step", payload; url), Dict{String,Any})
    wait || return reply["step_index"]::Int
    return as_step_reply(reply)
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
`tension_tether_ground`, `input_steering`, `input_depower`, `turn_radius`), the
`spline` block with its `downloops` flag, `metrics` and `optimized_parameters`.

`turn_radius` is the path's PHYSICAL turn radius [m] at each node (`r/|kappa|`,
geodesic on the tether sphere at that node's `distance_radial`) — the quantity
`InitParams.min_turn_radius` constrains, and the one to compare with the kite's
own `1/(c1*u_s_max)`. `input_depower` is constant unless the depower mode is
`"profile"`.

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
    winch_from_wc(wc; v_max = nothing, p_max = nothing) -> WinchParams

The winch law of a run, from the `WinchControllers.WCSettings` its
`WinchController` is built from: `kv`, `f_low` and `f_high` of
`data/wc_settings.yaml`. The optimizer maps `v_set = kv*sqrt(force)` onto its
radial force model, so these three numbers are what makes the optimized path the
path for THIS ground station.

A winch too stiff to reach the optimal reel-out speed within `f_high` makes the
problem infeasible and the server answers 422 — `kv*sqrt(f_high)` is the speed
ceiling to compare against roughly a third of the wind speed.

`v_max`/`p_max` are the ground station's limit PAST the force bound, where the
controller holds `f_high` and the speed keeps rising: pass the system settings'
`set.v_ro_max` (8 m/s here) to state it. Left unset, the optimizer uses its own
10 m/s bound — which with `kv = 0.0408` and `f_high = 8000 N` the square-root law
never reaches anyway (3.65 m/s), so this only starts to matter on a softer winch.
"""
winch_from_wc(wc; v_max = nothing, p_max = nothing) =
    WinchParams(; mode = "reelout", k_v = wc.kv, f_min = wc.f_low, f_max = wc.f_high,
                v_max = v_max === nothing ? nothing : Float64(v_max),
                p_max = p_max === nothing ? nothing : Float64(p_max))

"""
    min_turn_radius_request(fcs, tos; scale = 1.0, c1 = nothing) -> Union{Float64, Nothing}

`tos.min_feasibility_margin` in METRES — `margin/(c1*max_steering)`, the kite's own
physical turning limit scaled by the margin — sent WITH the request so the
optimizer cannot answer with a pattern that is about to be rejected. `nothing` —
send no constraint — when the margin is 0, which is also where the gate is off.

`scale` asks for MORE than that, and a reel-out run has to: the two numbers do not
measure the same curve at the same radius, and the difference is NOT in the run's
favour. Pass `reelout_anchor_ratio(table) * tos.turn_radius_headroom`.

The optimizer enforces `R = r/|kappa|` at each node's OWN radius, and `r` grows
through the lap — it starts at the anchor and reels out. The run installs the reply
as a fixed (azimuth, elevation) curve and flies it at the ANCHOR, where the same
angular curvature is physically tighter by `L/r`. Measured 2026-08-19 on a reply
anchored at 380 m: `distance_radial` spanned 380.0 -> 415.0 m, the tightest node sat
at r = 411 m with R = 12.32 m, and that curve at the 380 m anchor is 11.38 m — 0.92x.
The factor is `L/(L+dL)` with `dL` the lap's reel-out, so it BITES HARDEST AT THE
SHORT END: ~0.87 at 220 m against ~0.92 at 380 m. `reelout_anchor_ratio` measures
`1 + dL/L` off a reply and is the geometric half of `scale`.

On top of that the gate re-estimates the curvature from the reply's ~99 points
(`path_radius_profile`, a finite difference on the resampled polyline) and reads
~5 % tighter than the exact anchor value — 10.81 m against 11.38 m on the same
reply — and the run then adds `el_bias`/`el_offset_wing` before checking, which
compresses the azimuth axis by `cos(elevation)` a little more.
`tos.turn_radius_headroom` covers that half.

Both together are why an unscaled request came back at margin 0.63 against a
`min_feasibility_margin` of 0.74, converged and with its constraint satisfied
(2026-08-19, L = 220 m). The scaled request costs a slightly wider pattern, i.e. a
little power; it does not make the gate any weaker, which still scores the curve
that will actually be flown.

The conversion itself is exact: the gate's margin is a RATIO of two angular radii
at one tether length, and scaling both by that length turns it into a ratio of
physical ones, with the kite's physical minimum `1/(c1*u_s)` = 11.35 m at
`c1 = 0.2752`, `u_s = 0.32`. That much IS length-free — what is not is where the
optimizer measures the path's own radius.

`c1` defaults to the identified turn-rate table at `fcs.depower_setpoint`, which is
the turn authority the pattern is flown with. PASS THE ONE THE GATE WILL USE: from
phase 5 the run flies `depower_final`, where c1 is ~23 % lower (0.2133 against
0.2752), and a request made at the pattern's c1 is then ~23 % short of what the
reply will be judged against — measured 2026-08-20, a reply of 12.15 m answering a
10.37 m request and rejected at margin 0.61, which is exactly `0.74 * c1_final/c1`
of the 0.79 it would have scored in phase 4. `c1_at(phase)` in
`reelout_feasibility.jl` is that number.

The table refuses to extrapolate off its grid. A `body_damping`/`depower_setpoint`
it cannot serve therefore sends no constraint and warns, exactly as the feasibility
GATE degrades — an off-grid run loses the advice, not the run.
"""
function min_turn_radius_request(fcs, tos; scale = 1.0, c1 = nothing)
    tos.min_feasibility_margin > 0 || return nothing
    scale >= 0 || error("min_turn_radius_request: scale must be >= 0, got $scale.")
    if !isnothing(c1) && isfinite(c1) && c1 > 0
        return scale * tos.min_feasibility_margin / (c1 * fcs.max_steering)
    end
    coeffs = try
        turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
    catch exc
        exc isa ArgumentError || rethrow()
        @warn "No turn-rate coefficients for body_damping = $(fcs.body_damping), \
               depower = $(fcs.depower_setpoint) — asking the optimizer for NO \
               minimum turn radius, though min_feasibility_margin = \
               $(tos.min_feasibility_margin) will still gate the reply. Identify \
               this cell to get the constraint back."
        return nothing
    end
    return scale * tos.min_feasibility_margin / (coeffs.c1 * fcs.max_steering)
end

"AWETrim's own bounds on `input_depower`, from `src/awetrim/utils/defaults.py` [m]."
const DEPOWER_SEED_BOUNDS = (1.1, 2.3)

"""
    depower_seed(tos, wind_speed) -> Float64

The power-tape length `l_dp` [m] a request STARTS from, `tos.input_depower` plus
`tos.input_depower_per_wind` per m/s of wind above `tos.input_depower_wind_ref`,
clamped to [`DEPOWER_SEED_BOUNDS`](@ref).

Only a seed — `depower_mode` stays `"optimize"` and the server moves it — but the
seed is what decides whether the solve gets anywhere, because the AoA cap of 14°
is already binding at 6 m/s and the solve has to start on the feasible side of it.
Measured 2026-08-20 at 150 m: from 1.6 m, 6 m/s solved stage 1 of `/step` in
0.85 s and 21 iterations; 8 m/s hit the iteration cap with AoA at -25…88° and
took the constrained stage 2 down with it (422). Stage 1 carries no turn-radius
constraint, so this is not the `min_turn_radius` request going too far — the same
12.20 m ask solved at 6 m/s eighteen minutes earlier.

ONE-SIDED on purpose. Less wind would want less tape, but every run of the
5.0-7.0 m/s scan in `docs/wind_scan_results.yaml` converged from 1.6 m, and
seeding those lower would move answers that are already measured. The ramp only
adds where nothing has been flown.

The slope is anchored on two points, so treat it as provisional: 1.6 m is the
largest seed known to converge (7.0 m/s, that scan) and 1.85 m is what 8 m/s was
first solved with. A third measured wind should replace it rather than extend it.
"""
function depower_seed(tos, wind_speed)
    seed = tos.input_depower +
           tos.input_depower_per_wind * max(0.0, wind_speed - tos.input_depower_wind_ref)
    lo, hi = DEPOWER_SEED_BOUNDS
    clamped = clamp(seed, lo, hi)
    clamped == seed ||
        @warn @sprintf("Depower seed of %.3f m for %.1f m/s is outside AWETrim's \
                        bounds [%.1f, %.1f] m and was clamped to %.3f m. The ramp \
                        (input_depower %.2f + %.3f per m/s above %.1f m/s of \
                        data/traj_opt.yaml) has run out of tape.",
                       seed, wind_speed, lo, hi, clamped, tos.input_depower,
                       tos.input_depower_per_wind, tos.input_depower_wind_ref)
    return clamped
end

"""
    reelout_anchor_ratio(table) -> Float64

How much longer the tether gets over the lap a reply was solved for,
`maximum(distance_radial)/minimum(distance_radial)`, read off an
[`opt_trajectory`](@ref) table. `1.0` when the table carries no usable radial
profile — the neutral factor, so a missing column costs the correction and not the
run.

This is the factor a `min_turn_radius` request has to be scaled by, because the
optimizer's turn radius is measured along that profile while the run flies the
curve at the anchor; see [`min_turn_radius_request`](@ref). Measured off the reply,
per length, rather than assumed: the ratio is 1.19 at 180 m and 1.09 at 380 m for
the same ~35 m of reel-out per lap.

The tightest node is not necessarily the outermost one, so this over-asks slightly
— by at most the fraction of the lap between the tightest node and the end of it
(4 m of 35 on the reply measured 2026-08-19). Over-asking is the safe side: it
costs a slightly wider pattern, where under-asking costs the whole solve.
"""
function reelout_anchor_ratio(table)
    r = try
        Float64.(table["table"]["distance_radial"])
    catch
        return 1.0
    end
    (isempty(r) || !all(isfinite, r) || minimum(r) <= 0) && return 1.0
    return max(1.0, maximum(r) / minimum(r))
end

"""
    pattern_limits_from(tos) -> Union{PatternLimits, Nothing}

The box the optimized pattern must stay in, from the `pattern_*` fields of
`data/traj_opt.yaml`; each is in degrees and each is off at `0.0`. `nothing` when
all four are off, which leaves the optimizer's own defaults alone.
"""
function pattern_limits_from(tos)
    on(x) = x > 0 ? Float64(x) : nothing
    limits = PatternLimits(; azimuth_max = on(tos.pattern_azimuth_max),
                           elevation_min = on(tos.pattern_elevation_min),
                           elevation_max = on(tos.pattern_elevation_max),
                           azimuth_amplitude_min = on(tos.pattern_azimuth_amplitude_min))
    all(isnothing, (limits.azimuth_max, limits.elevation_min, limits.elevation_max,
                    limits.azimuth_amplitude_min)) && return nothing
    return limits
end

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
