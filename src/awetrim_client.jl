# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Julia client for the AWETrim reelout flight-path optimizer (REST). Part of the
# package, not exported: the example scripts reach it through
# `include("examples/awetrim_client.jl")`, which imports its names. It leaves no
# state of its own behind; loading it neither starts nor contacts a server.
#
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

"The repository root: `bin/run_server` and the `output/` caches are found from here."
const SKC_ROOT = normpath(joinpath(@__DIR__, ".."))
"Default server address; every function below takes `url` to override it."
const AWETRIM_URL = "http://127.0.0.1:8000"

# ---------------------------------------------------------------------------
# The shared structs (as agreed)
# ---------------------------------------------------------------------------
# `InflowConditions` and `WinchParams` live in the package (src/opt_conditions.jl).
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
recorded under one turn radius, depower mode or winch curve would block a
request that differs in exactly that, which is the one bug this cache must not
have. The hand-written field list of `"v2-"` missed six winch fields
(`softplus_beta`, `softminus_beta`, `use_awe_trim`, `v_reel_in`,
`reel_in_beta`, `winch_mode`), so `"v3-"` walks the structs instead
([`_key_fields`](@ref)): a field added to any request struct is in the key
without anyone having to remember it. The prefix retires the entries written
under the narrower keys rather than reusing them.

The project is NOT part of the key, and needs not be: the server sees only the
request, so two projects that send the same request pose the same problem and
share the entry. What the key cannot see is the server's own configuration
(kite, tether, solver defaults) — clear the cache after an AWETrim upgrade.

The key is a [`stable_hash`](@ref), not Base's `hash`, so it survives a Julia
upgrade. `"v4-"` retires the `hash`-based `"v3-"` keys.
"""
function opt_request_key(p::InitParams)
    return "v4-" * stable_hash(Tuple(_key_fields(getfield(p, f))
                                      for f in fieldnames(InitParams) if f !== :name))
end

"""
    stable_hash(x) -> String

64 bits of the SHA-256 of `repr(x)`, as 16 hex digits. Unlike Base's `hash`, which
may change between Julia releases, this gives the same key on every Julia version
and machine, as long as `x` holds only plain values (numbers, strings, symbols,
`nothing`, tuples and vectors of them) — which [`_key_fields`](@ref) ensures.
`repr` prints a float as the shortest string that parses back to it, so the key
is exact, and it also covers `NaN`, `Inf` and `-0.0`.
"""
stable_hash(x) = bytes2hex(sha256(repr(x)))[1:16]

# Request structs flattened to nested tuples of their field VALUES, for the key.
# Hashing the structs themselves would not do: the default `hash` of a struct
# holding a `Vector` goes by the vector's identity, not its contents, so two
# equal requests would never share a key.
_key_fields(x::Union{WinchParams, InflowConditions, Trajectory, DepowerSpec, PatternLimits,
                     StepParams}) =
    Tuple(_key_fields(getfield(x, f)) for f in fieldnames(typeof(x)))
# `symmetric` came last; left out while off, so the keys cached before it stay valid.
_key_fields(x::PatternLimits) =
    Tuple(_key_fields(getfield(x, f)) for f in fieldnames(PatternLimits)
          if !(f === :symmetric && isnothing(x.symmetric)))
_key_fields(x) = x

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

"""
    report_status_error(path, err::HTTP.StatusError)

Log the RESPONSE BODY of a failed request. `HTTP.StatusError`'s own message
carries only the status line, so a 422 otherwise arrives as
`http status error: 422 for POST .../init` with no reason — and the reason is
exactly what distinguishes the two kinds of 422 this server sends:

- a **validation** 422, whose `detail` is a list of `{loc, msg}` entries naming
  the offending field. Every request model is `extra="forbid"`, so a field this
  client sends and the server does not know lands here. That is a CLIENT BUG and
  is logged as a warning.
- an **infeasibility** 422, whose `detail` is a plain string. Routine — a solve
  that cannot satisfy its constraints, which `record_opt_failure!` exists to
  remember — so it is logged at info level.

Never throws: a diagnostic must not replace the error it is diagnosing. The
caller rethrows `err` unchanged, so `exc isa HTTP.StatusError && exc.status == 422`
downstream keeps working.
"""
function report_status_error(path, err)
    detail, validation = try
        parsed = JSON3.read(String(copy(err.response.body)), Dict{String, Any})
        d = get(parsed, "detail", parsed)
        if d isa AbstractVector
            (join(("  " * join(string.(get(e, "loc", [])), ".") * ": " *
                   string(get(e, "msg", e)) for e in d), "\n"), true)
        else
            (string(d), false)
        end
    catch
        (try first(split(String(copy(err.response.body)), '\n')) catch; "<no body>" end, false)
    end
    if validation
        @warn "POST $path -> HTTP $(err.status), the server REJECTED the request \
               (a field it does not accept, or one out of range — the models are \
               extra=\"forbid\"):\n$detail"
    else
        @info "POST $path -> HTTP $(err.status): $detail"
    end
    return nothing
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
            err isa HTTP.StatusError && report_status_error(path, err)
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
                          p_max = opt_float(d, "p_max"),
                          optimize_k_v = something(opt_get(d, "optimize_k_v"), false))

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
                         azimuth_amplitude_min = opt_float(d, "azimuth_amplitude_min"),
                         elevation_amplitude_max = opt_float(d, "elevation_amplitude_max"),
                         symmetric = opt_get(d, "symmetric"))
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

`raw = true` returns a blocking reply as the parsed JSON `Dict` instead, which is
what [`OptChain`](@ref) stores.
"""
function opt_step(params::StepParams; url = AWETRIM_URL,
                  inflow_conditions = nothing, wait = true, max_iter = nothing,
                  raw = false)
    payload = as_dict(params)
    inflow_conditions !== nothing && (payload["inflow_conditions"] = as_dict(inflow_conditions))
    max_iter !== nothing && (payload["max_iter"] = max_iter)
    payload["wait"] = wait
    reply = JSON3.read(post("/step", payload; url), Dict{String,Any})
    wait || return reply["step_index"]::Int
    return raw ? reply : as_step_reply(reply)
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
# Solution cache: a chain of requests replayed without the server
#
# The counterpart of the failed-request cache for the requests that WORKED —
# and only for those whose result was APPLIED (installed and flown). A converged
# reply that a gate rejected is not stored as a result.
#
# A result cannot be keyed by its request alone the way a failure is. A warm
# `/step` solves from the optimum the SERVER holds, and that state is made up of
# every request since the last `/init`: each `/step` also moves the session's
# length, winch and limits, even when its solve fails. So the key is a CHAIN. A
# cold request's key is `opt_request_key` of its `InitParams`, and every step's
# key hashes its parent's key with its own `StepParams`. Equal keys mean equal
# lineage.
#
# A hit is served without asking the server, so the server no longer holds the
# state the chain has reached. The next MISS therefore first REBUILDS that state:
# `/init` under the session's current config, seeded with the cached optimum,
# then one `/step` from there. The result is close to the lost state but not
# bit-identical, because IPOPT's warm start is gone. The rebuilt state therefore
# gets a key of its own (the parent's plus "rebuilt"). Whatever is solved from it
# is stored under that lineage, so it never passes for the original. A second
# rerun hits it as well.
#
# When a chain ends in an applied result, its whole lineage since the `/init`
# is stored: every reply a warm start built on, rejected ones included, because a
# replay cannot reach the result without them. A WARM step that fails is stored
# as soon as it fails (under `opt_failure_cache`), because the failure cache above
# can only key cold requests.
#
# The key cannot see the server itself. Clear this cache
# (`clear_opt_chain_cache()`) after any AWETrim change, or a replay keeps serving
# the old optimizer's answers.

"""
The directory of the solution cache, one JSON file per chain key, shared by
every script and sweep worker on this machine. Delete it to re-solve everything.
"""
const OPT_CHAIN_CACHE = joinpath(SKC_ROOT, "output", "opt_chain_cache")

"""
    OptChain(url = AWETRIM_URL; successes = true, failures = true,
             dir = OPT_CHAIN_CACHE)

One optimizer session as a chain of cached requests. [`chain_init`](@ref),
[`chain_step`](@ref), [`chain_status`](@ref) and [`chain_trajectory`](@ref) stand
in for `opt_init`, `opt_step`, `opt_status` and `opt_trajectory`: each answers
from the cache on a hit, and asks the server otherwise (rebuilding the server's
state first when needed). [`record_opt_success!`](@ref) marks the latest result
as APPLIED, which is the only point where converged replies are stored.

`successes = false` never serves or stores a converged reply, and
`failures = false` does the same for failed ones. With both off, the chain passes
every request straight through.
"""
mutable struct OptChain
    url::String
    successes::Bool
    failures::Bool
    dir::String
    state::String                        # key of the state a warm /step starts from; "" before any /init
    server::String                       # key of the state the server REALLY holds; "" = unknown
    config::Union{InitParams, Nothing}   # session config at `state`: the /init with every step since applied
    table::Union{Dict{String, Any}, Nothing}    # /trajectory at `state`, as the server would serve it
    current::Union{Dict{String, Any}, Nothing}  # entry of the latest /step, sent or served
    served::Bool                         # `current` came from the cache, not the server
    pending::Vector{Dict{String, Any}}   # steps of this lineage that were sent and are not stored yet
    hits::Int
    misses::Int
    rebuilds::Int
    # Replay mode (see `replay_entries`): the next steps' entries, served in order whatever
    # was asked; `nothing` is off. Once empty, every further step fails as a 422.
    replay::Union{Nothing, Vector{Dict{String, Any}}}
end
OptChain(url::AbstractString = AWETRIM_URL; successes::Bool = true, failures::Bool = true,
         dir::AbstractString = OPT_CHAIN_CACHE, replay = nothing) =
    OptChain(String(url), successes, failures, String(dir), "", "", nothing, nothing,
             nothing, false, Dict{String, Any}[], 0, 0, 0, replay)

"""
    replay_entries(scenario_dir, log_name; dir = OPT_CHAIN_CACHE) -> Vector{Dict}

The solution-cache entries of the paths an archived run installed, in the order
it installed them: each path of `<log_name>_opt_paths.yaml` in `scenario_dir`
matched to the converged entry whose reply (startup) or trajectory table (a
re-optimization) carries the same curve, within 0.01°. Errors if a path has no
match: the cache was cleared since that run, and it cannot be replayed.

Passed as `OptChain(...; replay)`, the run flies the archived run's optimizer
results on the current plant, without the optimizer: the way to tell a change of
the kite model from a change of the path the optimizer returns for it.
"""
function replay_entries(scenario_dir, log_name; dir = OPT_CHAIN_CACHE)
    file = joinpath(scenario_dir, log_name * "_opt_paths.yaml")
    isfile(file) || error("replay_entries: no $(basename(file)) in $scenario_dir.")
    entries = Dict{String, Any}[]
    for f in filter(endswith(".json"), readdir(dir; join = true))
        e = JSON3.read(read(f, String), Dict{String, Any})
        get(e, "status", "") == "converged" && push!(entries, e)
    end
    curves(e) = filter(!isnothing, [
        haskey(e, "reply") ? (Float64.(e["reply"]["trajectory"]["azimuth"]),
                              Float64.(e["reply"]["trajectory"]["elevation"])) : nothing,
        haskey(e, "table") ? (rad2deg.(Float64.(e["table"]["table"]["azimuth"])),
                              rad2deg.(Float64.(e["table"]["table"]["elevation"]))) : nothing])
    same(az, el, (caz, cel)) = length(caz) == length(az) &&
        maximum(abs, caz .- az) < 0.01 && maximum(abs, cel .- el) < 0.01
    return map(YAML.load_file(file)["paths"]) do p
        az, el = Float64.(p["azimuth"]), Float64.(p["elevation"])
        i = findfirst(e -> any(c -> same(az, el, c), curves(e)), entries)
        isnothing(i) && error(@sprintf("replay_entries: the path installed at t = %.1f s \
                                        in %s is not in the solution cache %s.",
                                       p["installed_t"], scenario_dir, dir))
        entries[i]
    end
end

# A step's key: its parent's, and everything of the step the server sees. `x` is
# already flattened by `_key_fields`, see there for why.
chain_key(parent::AbstractString, x) = "c2-" * stable_hash((parent, x))

_chain_file(oc::OptChain, key) = joinpath(oc.dir, key * ".json")

function _chain_entry(oc::OptChain, key)
    file = _chain_file(oc, key)
    isfile(file) || return nothing
    try
        JSON3.read(read(file, String), Dict{String, Any})
    catch exc
        @warn "Ignoring an unreadable solution-cache entry at $file." exception = exc
        nothing
    end
end

# Written to a temporary file and moved into place, so a parallel worker never
# reads half an entry.
function _write_chain_entry(oc::OptChain, entry)
    mkpath(oc.dir)
    file = _chain_file(oc, entry["key"])
    tmp = file * ".tmp-$(getpid())"
    write(tmp, JSON3.write(entry))
    mv(tmp, file; force = true)
    return nothing
end

"`p` with the fields in `kw` replaced."
_with(p::InitParams; kw...) =
    InitParams(; merge(NamedTuple{fieldnames(InitParams)}(
                           Tuple(getfield(p, f) for f in fieldnames(InitParams))),
                       values(kw))...)

# What a step leaves the session holding: its length and winch always, and the
# fields where `nothing` means "keep" only when they are sent.
function _step_config(p::InitParams, sp::StepParams, inflow_conditions)
    kw = Dict{Symbol, Any}(:length => sp.length, :winch_params => sp.winch_params)
    isnothing(sp.depower) || (kw[:depower] = sp.depower)
    isnothing(sp.min_turn_radius) || (kw[:min_turn_radius] = sp.min_turn_radius)
    isnothing(sp.pattern_limits) || (kw[:pattern_limits] = sp.pattern_limits)
    isnothing(inflow_conditions) || (kw[:inflow_conditions] = inflow_conditions)
    return _with(p; kw...)
end

# The 422 a cached failure stands for, so that callers catching the real one
# (`exc isa HTTP.StatusError && exc.status == 422`) need not know the difference.
_cached_422(entry) =
    HTTP.StatusError(HTTP.Response(422, [], Vector{UInt8}(JSON3.write(Dict("detail" =>
                         "cached failure from $(get(entry, "when", "an earlier run")): " *
                         string(get(entry, "reason", "no reason recorded")))));
                     request = HTTP.Request("POST", "/step")))

function _chain_converged!(oc::OptChain, entry)
    entry["status"] = "converged"
    entry["table"] = opt_trajectory(; url = oc.url)
    oc.table = entry["table"]
    return nothing
end

function _chain_failed!(oc::OptChain, entry, reason)
    entry["status"] = "failed"
    entry["reason"] = String(reason)
    oc.failures && _write_chain_entry(oc, entry)
    return nothing
end

"""
    chain_init(oc::OptChain, p::InitParams) -> InitReply

`opt_init` that starts a new chain. `/init` always goes to the server, because it
only fits the starting path and a following `/step` needs the fitted path it
returns.
"""
function chain_init(oc::OptChain, p::InitParams)
    oc.state = opt_request_key(p)
    oc.server = ""   # unknown until the server has accepted it
    oc.config = p
    oc.table = nothing
    oc.current = nothing
    oc.served = false
    empty!(oc.pending)
    reply = opt_init(p; url = oc.url)
    oc.server = oc.state
    return reply
end

"""
    chain_step(oc::OptChain, sp::StepParams; wait = true, inflow_conditions = nothing,
               max_iter = nothing)

`opt_step` on the chain. On a hit the reply comes from the cache and the server
is not asked: blocking, a converged entry returns its `StepReply` and a failed one
throws the same `HTTP.StatusError` 422 the server would; with `wait = false` it
returns at once and [`chain_status`](@ref) reports the outcome. On a miss the
server's state is rebuilt first if it lags behind the chain, and then the step is
sent.
"""
function chain_step(oc::OptChain, sp::StepParams; wait = true,
                    inflow_conditions = nothing, max_iter = nothing)
    isnothing(oc.config) && error("chain_step before chain_init: there is no session to step.")
    step_fields = (_key_fields(sp), _key_fields(inflow_conditions), max_iter)
    config = _step_config(oc.config, sp, inflow_conditions)
    if !isnothing(oc.replay)
        isempty(oc.replay) && throw(_cached_422(Dict{String, Any}("when" => "replay",
            "reason" => "the replayed run installed no further path")))
        entry = popfirst!(oc.replay)
        oc.hits += 1
        oc.state = entry["key"]
        oc.config = config
        oc.current = entry
        oc.served = true
        oc.table = entry["table"]
        @info @sprintf("Optimizer step at L = %.1f m replayed: the archived result at \
                        L = %.1f m (%s).", sp.length, entry["length_m"], entry["when"])
        return wait ? as_step_reply(entry["reply"]) : 0
    end
    usable(entry) = !isnothing(entry) &&
                    (entry["status"] == "converged" ?
                         oc.successes && (!wait || !isnothing(get(entry, "reply", nothing))) :
                         oc.failures)
    key = chain_key(oc.state, step_fields)
    entry = _chain_entry(oc, key)
    # A server that lags behind would be rebuilt first, and an earlier run stored what came
    # of that under the rebuilt lineage: look there too, before paying for the rebuild.
    if !usable(entry) && oc.server != oc.state
        key = chain_key(chain_key(oc.state, "rebuilt"), step_fields)
        entry = _chain_entry(oc, key)
    end
    if usable(entry)
        oc.hits += 1
        oc.state = key
        oc.config = config
        oc.current = entry
        oc.served = true
        failed = entry["status"] == "failed"
        failed || (oc.table = entry["table"])
        @info @sprintf("Optimizer step at L = %.1f m served from the solution cache (%s, \
                        recorded %s); the server was not asked.",
                       sp.length, failed ? "failed" : "converged",
                       get(entry, "when", "at an unknown time"))
        failed && wait && throw(_cached_422(entry))
        return wait ? as_step_reply(entry["reply"]) : 0
    end
    oc.misses += 1
    if oc.server != oc.state
        rebuild_session!(oc)
        key = chain_key(oc.state, step_fields)
    end
    entry = Dict{String, Any}("key" => key, "parent" => oc.state, "status" => "solving",
                              "length_m" => sp.length, "request" => as_dict(sp),
                              "when" => format(now(), "yyyy-mm-dd HH:MM:SS"))
    oc.state = oc.server = key
    oc.config = config
    oc.current = entry
    oc.served = false
    push!(oc.pending, entry)
    try
        wait || return opt_step(sp; url = oc.url, inflow_conditions, max_iter, wait = false)
        entry["reply"] = opt_step(sp; url = oc.url, inflow_conditions, max_iter, raw = true)
    catch exc
        if exc isa HTTP.StatusError && exc.status == 422
            detail = try
                string(get(JSON3.read(String(copy(exc.response.body)), Dict{String, Any}),
                           "detail", "no detail"))
            catch
                "no detail"
            end
            _chain_failed!(oc, entry, "422 from /step: " * first(detail, 300))
        else
            # Never reached the solver, or not knowably: the server's state is unknown.
            pop!(oc.pending)
            oc.current = nothing
            oc.server = ""
        end
        rethrow()
    end
    _chain_converged!(oc, entry)
    return as_step_reply(entry["reply"])
end

"""
    chain_status(oc::OptChain) -> Dict

`opt_status` on the chain. For a step that was served, or whose outcome is
already known, this is `Dict("state" => outcome)` without asking the server.
Otherwise it asks the server, and when the step has just finished it records the
outcome (fetching the trajectory of a converged one).
"""
function chain_status(oc::OptChain)
    entry = oc.current
    if !isnothing(entry) && (oc.served || entry["status"] != "solving")
        return Dict{String, Any}("state" => entry["status"])
    end
    status = opt_status(oc.url)
    if !isnothing(entry)
        status["state"] == "converged" && _chain_converged!(oc, entry)
        status["state"] == "failed" &&
            _chain_failed!(oc, entry, something(get(status, "last_error", nothing),
                                                "solver failed"))
    end
    return status
end

"""
    chain_trajectory(oc::OptChain) -> Dict

`opt_trajectory` on the chain: the last converged solve at the chain's state,
from the cache or as fetched when the solve finished. It is a copy, so a caller
that edits it cannot change what gets stored.
"""
function chain_trajectory(oc::OptChain)
    isnothing(oc.table) && return opt_trajectory(; url = oc.url)
    return deepcopy(oc.table)
end

"""
    record_opt_success!(oc::OptChain)

The latest result was APPLIED: store it, together with the unstored replies of
its lineage that a replay needs to reach it. Does nothing for a result that was
served from the cache, since that one is stored already.
"""
function record_opt_success!(oc::OptChain)
    entry = oc.current
    if isnothing(entry) || entry["status"] != "converged"
        @warn "record_opt_success! without a converged result on the chain; nothing stored."
        return nothing
    end
    entry["applied"] = true
    if oc.successes
        for e in oc.pending
            e["status"] == "converged" && _write_chain_entry(oc, e)
        end
    end
    empty!(oc.pending)
    return nothing
end

"""
    rebuild_session!(oc::OptChain)

Put the server back into the state the chain has reached after it was served
from the cache: `/init` under the chain's current config, seeded with the cached
optimum and its depower, then one `/step` from there. A failed rebuild is not
fatal. The next step is then sent to whatever the server holds, under a key that
records the failure.
"""
function rebuild_session!(oc::OptChain)
    oc.rebuilds += 1
    tab = oc.table
    p = oc.config
    if !isnothing(tab)
        seed = Trajectory(rad2deg.(Float64.(tab["table"]["azimuth"])),
                          rad2deg.(Float64.(tab["table"]["elevation"])))
        l_dp = opt_float(get(tab, "optimized_parameters", nothing), "input_depower")
        p = isnothing(l_dp) ? _with(p; trajectory = seed) :
                              _with(p; trajectory = seed, input_depower = l_dp)
    end
    @info @sprintf("Rebuilding the optimizer's session at L = %.1f m from the solution \
                    cache before sending a request it has not seen.", p.length)
    oc.server = ""
    try
        opt_init(p; url = oc.url)
        oc.table = nothing
        if !isnothing(tab)
            opt_step(StepParams(; length = p.length, winch_params = p.winch_params);
                     url = oc.url)
            oc.table = opt_trajectory(; url = oc.url)
        end
        oc.state = chain_key(oc.state, "rebuilt")
    catch exc
        exc isa HTTP.StatusError || rethrow()
        @warn "Could not rebuild the optimizer's session; the next request is solved \
               from what the server holds now." exception = exc
        oc.state = chain_key(oc.state, "rebuild failed")
    end
    oc.server = oc.state
    return nothing
end

"""
    clear_opt_chain_cache(; dir = OPT_CHAIN_CACHE) -> Bool

Forget every stored chain. `true` if there was a cache to remove.
"""
function clear_opt_chain_cache(; dir::AbstractString = OPT_CHAIN_CACHE)
    had = isdir(dir)
    rm(dir; recursive = true, force = true)
    return had
end

"""
    guess_el_center_seed(tos, wind_speed) -> Float64

The centre elevation [deg] the initial-guess lemniscate is built at:
`tos.guess_el_center_high` at and above `tos.guess_el_center_wind_ref`,
`tos.guess_el_center` below it. `tos.guess_el_center_high == 0.0` disables the
step, so this returns `tos.guess_el_center` at every wind.

A STEP, not a ramp like [`depower_seed`](@ref)'s: the guess picks a basin of a
multi-modal solve (`TrajOptSettings`' docstring), and there is no reason to
believe a basin that converges at one wind shrinks gracefully into one that
converges at another — only the tested seeds are known to work.
"""
function guess_el_center_seed(tos, wind_speed)
    tos.guess_el_center_high > 0 && wind_speed >= tos.guess_el_center_wind_ref ?
        tos.guess_el_center_high : tos.guess_el_center
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

# ---------------------------------------------------------------------------
# The optimizer session of a run (examples/simple_opt_reelout.jl)
# ---------------------------------------------------------------------------
"""
    optimizer_session(tos, inflow, replay_paths, log_name)
        -> (; el_center_seed_base, el_center_seed, startup_seed_offset, guess_az, guess_el, opt_chain)

The connection to the optimizer and the seed of the startup solve. The seed is the guess
lemniscate of `data/traj_opt.yaml`, centred at `el_center_seed` (`el_center_seed_base` until
the startup solve converges from another; `startup_seed_offset` is how far). Every request
of the run goes through `opt_chain`, which replays applied results and known failures (see
`OptChain`); `replay_paths`, a scenario folder, flies that run's optimizer results
instead of asking the optimizer, see `replay_entries`.
"""
function optimizer_session(tos, inflow, replay_paths, log_name)
    el_center_seed_base = guess_el_center_seed(tos, inflow.wind_speed)
    el_center_seed = el_center_seed_base
    startup_seed_offset = 0.0
    guess_az, guess_el = figure_eight_path(tos.guess_a, tos.guess_b,
                                           0.0, el_center_seed,
                                           0.0, tos.guess_points)
    @info @sprintf("Initial guess: %.0f° x %.0f° at %.0f°, %d points.",
                   tos.guess_a, tos.guess_b, el_center_seed, tos.guess_points)
    ensure_server(tos.base_url; autostart = tos.autostart_server)
    opt_chain = OptChain(tos.base_url; successes = tos.opt_success_cache,
                         failures = tos.opt_failure_cache,
                         replay = isnothing(replay_paths) ? nothing :
                                  replay_entries(replay_paths, log_name))
    isnothing(replay_paths) ||
        @info "Replaying the $(length(opt_chain.replay)) optimizer results of $replay_paths; the optimizer is not asked."
    return (; el_center_seed_base, el_center_seed, startup_seed_offset, guess_az, guess_el, opt_chain)
end
