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
using SHA: sha256
using Dates: now, format
using Printf: @sprintf
using YAML
using AtmosphericModels: AtmosphericModel, calc_wind_factor
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
    # Make `k_v` a DESIGN VARIABLE instead of a constant: the server brackets it a
    # factor `K_V_BRACKET_FACTOR` (2.0) either side of the value sent and solves for
    # it under its own saturating tension curve, so the path and the gain that flies
    # it are optimized together. The reply's `k_v` is then the OPTIMIZED one and the
    # path is not flyable with the value sent in — see `apply_optimized_kv!`.
    optimize_k_v::Bool = false
    # Corner sharpness of the two soft saturations the server applies to the
    # tension curve [1/N]; larger is sharper, the transition spanning a band of
    # order 1/beta. Sent because `WCSettings.force_limit = "soft"` INVERTS that
    # curve to command a reel-out speed: the controller and the optimizer must use
    # one law, and before this they agreed only because both defaulted to 1e-3.
    # `nothing` leaves the server on its own default.
    softplus_beta::Union{Float64, Nothing} = nothing
    softminus_beta::Union{Float64, Nothing} = nothing
    # Blend factor in [0, 1] towards a reel-in-capable server winch law
    # (Winch.tension_curve's use_awe_trim, AWETrim/src/awetrim/system/winch.py):
    # 0 (default) is the server's plain quadratic law above, unchanged; 1
    # replaces it below f_min with a straight line through (0, f_min) and
    # (v_reel_in, 0), handed to the quadratic law by a smooth maximum. Mirrors
    # calc_vro_soft's own use_awe_trim (WinchControllers.jl), which blends the
    # OTHER direction (force -> speed; the server computes speed -> force).
    use_awe_trim::Float64 = 0.0
    # use_awe_trim only: reel-in speed [m/s] at zero force (< 0) and handover
    # sharpness [s/m]. `nothing` keeps the server's own defaults (-2.0, 20.0).
    # Unlike softminus_beta above, reel_in_beta has no matching sharpness
    # requirement against f_min/k_v to worry about on the server side -- see
    # WinchParams.reel_in_beta's docstring in AWETrim's schemas.py.
    v_reel_in::Union{Float64, Nothing} = nothing
    reel_in_beta::Union{Float64, Nothing} = nothing
    # "force_law" (the server default) ties the tension to the reel speed through
    # the curve above, as a per-node equality; "free_speed" drops that equality and
    # bounds the tension to [f_min, f_max] instead, the reel speed becoming a direct
    # acceleration-limited control. `nothing` leaves the server on its own default.
    # The reply is then the best path for ANY winch in that force band, so its
    # predicted power is an UPPER BOUND, not a prediction of this k_v law -- see
    # TrajOptSettings.opt_winch_mode.
    winch_mode::Union{String, Nothing} = nothing
    # Sharpness [s/m] of the soft reel-speed clamp at `v_max`, the server-side
    # mirror of `calc_vro_soft`'s `_clamp_v_sat` (`WCSettings.v_sat_beta`): the
    # tension then rises to `f_max` AT `v_max`, as the controller's curve does,
    # instead of ending at the `v_max` speed bound. Needs `v_max` or `p_max`;
    # `nothing` keeps the plain law, i.e. a hard clamp.
    v_sat_beta::Union{Float64, Nothing} = nothing
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
Total offset [rel_depower units] between the two models at equal power.

Recalibrated 2026-08-29 after `kite.mass` in `data/settings_reelout_150m.yaml`
rose from 6.2 kg to 10.9926 kg (commit `6443fbe`) to match AWETrim's own LEI-V3
wing — AWETrim never receives a mass, so it was already assuming 11 kg, and the
prior 0.099 had silently absorbed the 4.8 kg mismatch. At 6 m/s (8 mm tether,
no turbulence), 0.099 measured 7506 W against 7301 W predicted (ratio 1.028).
Confirmed at **0.1010**, twice: 7286 W measured against 7262 W predicted,
ratio 1.00, both runs inside the 0.995..1.004 acceptance band.

Earlier 2026-08-26 sweep, at the 6.2 kg mass (predictor AWETrim `a745914`):

| offset | measured − predicted [W] | power_ratio |
|--------|--------------------------|-------------|
| 0.105  | −578                     | 0.92        |
| 0.100  | −68                      | 0.99        |
| 0.0985 | +67                      | 1.01        |
| 0.095  | +451                     | 1.06        |

~100 W measured per 0.001, ~0.14 of ratio per 0.010 — steep enough that the
0.995..1.004 band (±0.00036 in this variable) needs the constant written to
four decimals. Earlier history: 0.12 measured 2026-08-18
(`docs/steering_depower.md`, VSM needing 0.12 more rel_depower than the ROM for
equal power, of which 0.08 was the two tape-length zeros and 0.04 genuine aero
disagreement), lowered to 0.107 by commit `ee1ecbd` without re-measurement, then
(post-8 mm tether) to 0.099, then (post-mass) to this value. RE-MEASURE after
any change to either model's depower axis, aero, or mass.
"""
const AWETRIM_V3KITE_DEPOWER_OFFSET = 0.1010

"""
    awetrim_depower_to_v3kite(l_dp) -> Float64

Convert an AWETrim `l_dp` [m] (`input_depower`, `l_dp = 0.6 + 5*u_p`) into the
V3Kite `rel_depower` expected to fly at the SAME power, i.e. `(l_dp - 0.6)/5 +
`[`AWETRIM_V3KITE_DEPOWER_OFFSET`](@ref). This is the FULL correction — do not
also subtract the 0.4 m calibration offset separately, it is already inside
the original 0.12 split (0.08 tape zero + 0.04 aero).
"""
awetrim_depower_to_v3kite(l_dp) = (l_dp - 0.6) / 5 + AWETRIM_V3KITE_DEPOWER_OFFSET

"""
    PatternLimits(; azimuth_max, elevation_min, elevation_max,
                  azimuth_amplitude_min, elevation_amplitude_max, symmetric)

A box, in DEGREES, on where the optimized pattern may go. The server bounds the
B-spline's control coefficients, so by the convex-hull property the limits hold
along the whole curve and not merely at its nodes.

Every field is optional and `nothing` keeps the optimizer's own default for it
(|azimuth| <= 45.8°, 0.6° <= elevation <= 51.6°, no amplitude floor).
`azimuth_amplitude_min` is the figure's half-width and guards the degenerate
zero-width collapse; `elevation_max` guards the run-away-to-zenith basin — the
two bad basins a failed re-optimization falls into. `elevation_amplitude_max`
caps the figure's elevation HALF-SPAN with one smooth row (mean squared
deviation from the mean elevation <= value²/2) — where `elevation_max` only
caps where the path may sit, this caps how TALL it is. `symmetric = true`
forces a figure mirror-symmetric about azimuth 0 (half a period later the kite
is at the mirrored point). On `/step` the struct
replaces the session's limits as a whole, so an all-`nothing` `PatternLimits()`
CLEARS them.
"""
Base.@kwdef struct PatternLimits
    azimuth_max::Union{Float64, Nothing} = nothing           # |azimuth| <= this [deg]
    elevation_min::Union{Float64, Nothing} = nothing         # elevation >= this [deg]
    elevation_max::Union{Float64, Nothing} = nothing         # elevation <= this [deg]
    azimuth_amplitude_min::Union{Float64, Nothing} = nothing # half-width >= this [deg]
    elevation_amplitude_max::Union{Float64, Nothing} = nothing # half-span <= this [deg]
    symmetric::Union{Bool, Nothing} = nothing                # mirror-symmetric figure
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
The `softminus_beta` always sent to AWETrim, independent of the local winch's own
`wc.softminus_beta` — measured 2026-08-25: `2e-3`/`5e-3` both make the 3 m/s
solve FAIL (422, Max_Iterations_Exceeded), `1e-3` converges. Kept fixed here so a
local sharpening of the plain `force_limit = "soft"` law WITHOUT `soft_lfc`
(which needs `softminus_beta * f_low >= 8`, e.g. `0.03` at `f_low = 350`)
never reaches the server. With `soft_lfc = true` this value is not even read
locally any more — `calc_vro_soft` hard-clamps at `f_low` instead, see
`use_awe_trim` on `winch_from_wc` for what governs that curve's server-side
counterpart. See `data/wc_settings.yaml`.
"""
const AWETRIM_SOFTMINUS_BETA = 1e-3

"""
Whether [`winch_from_wc`](@ref) sends `v_sat_beta`. Needs an AWETrim whose
`Winch.radial_equation` blends in the speed form `v = V(F)` near `v_max`
(2026-09-23): with the plain equality `F = T(v)` the clamped curve is nearly
vertical there, and the 7 m/s Cabauw startup solve at 150 m failed in IPOPT.
With the blend it converges at 3, 4, 5, 7 and 10 m/s.
"""
const SEND_V_SAT_BETA = true

"""
    winch_from_wc(wc; v_max = wc.v_sat, p_max = nothing) -> WinchParams

The winch law of a run, from the `WinchControllers.WCSettings` its
`WinchController` is built from: `kv`, `f_low` and `f_high` of
`data/wc_settings.yaml`, plus the two `beta`s that set how softly the server
saturates its tension curve at those limits, and `use_awe_trim`/`v_reel_in`/
`reel_in_beta` for the reel-in-capable blend. The optimizer maps
`v_set = kv*sqrt(force)` onto its radial force model, so these numbers are what
makes the optimized path the path for THIS ground station.

The `beta`s matter more than they look. The server's effective force FLOOR is
`sp(beta*f_min)/beta`, not `f_min`: at the historical 1e-3 a 350 N `f_min` acts as
884 N, which is above the entire force range a 3 m/s run ever reaches. They also
have to match `calc_vro_soft`'s, which inverts this exact curve under
`force_limit = "soft"` — the two sides used to agree only by coincidence.

A winch too stiff to reach the optimal reel-out speed within `f_high` makes the
problem infeasible and the server answers 422 — `kv*sqrt(f_high)` is the speed
ceiling to compare against roughly a third of the wind speed.

`v_max`/`p_max` are the ground station's limit PAST the force bound, where the
controller holds `f_high` and the speed keeps rising. `v_max` defaults to `wc.v_sat`
— `data/wc_settings.yaml`'s own reel-out speed limit (8 m/s here), the same cap the
runtime `WinchController` enforces — so the optimizer is asked for a path THIS
ground station can actually fly. Pass `v_max = nothing` to fall back to the
optimizer's own 10 m/s bound instead, which with `kv = 0.0408` and
`f_high = 8000 N` the square-root law never reaches anyway (3.65 m/s), so this
only starts to matter on a softer winch.

`f_max` defaults to `wc.f_high_awe_trim` when that is set (a fixed de-rating,
independent of wind speed) and to `wc.f_high` otherwise, and is overridable so
a request can be solved against the ceiling that will actually be in force
while its path is flown — `fcs.first_lap_force_frac` holds the runtime limit
down for the first figure of eight, and the startup path is the one flown
there. Lowering it also lowers the speed ceiling `kv*sqrt(f_max)` referred to
above.

`softminus_beta` sent to the server is pinned to `AWETRIM_SOFTMINUS_BETA`, NOT
read off `wc`: `wc.softminus_beta` may be sharpened locally for the plain
`force_limit = "soft"` law WITHOUT `soft_lfc` (which needs `softminus_beta *
f_low >= 8`, far sharper than the server tolerates) — with `soft_lfc = true`,
`calc_vro_soft` hard-clamps at `f_low` locally instead and never reads
`softminus_beta` at all, but the pin stays unconditional so a run that flips
`soft_lfc` back off is still covered. See `AWETRIM_SOFTMINUS_BETA`'s
docstring.

`use_awe_trim` defaults to `wc`'s and is overridable so a seeding solve can be
sent at a value known to converge before the real request — see
`TrajOptSettings.opt_warm_start_awe_trim`. `v_reel_in`/`reel_in_beta` are read
straight off `wc` (unlike the `beta`s above, they need no pinning: the
server-side sharpness requirement on `reel_in_beta` is far more forgiving than
the local one, since `Winch.tension_curve` blends in the forward direction),
so a run that reels in locally optimizes against a server winch model that
can do the same. `wc.use_awe_trim` defaults to `0.0`, leaving the server's
plain law unchanged unless `data/wc_settings.yaml` opts in.

`v_sat_beta` is sent only while [`SEND_V_SAT_BETA`](@ref) is on, and then only when
the local law really soft-clamps at the same speed the
server is bounded by: `force_limit = "soft"` (only `calc_vro_soft` applies the
clamp), a finite `wc.v_sat_beta` (`Inf` is a hard clamp, the server's plain law)
and `v_max == wc.v_sat`. The server's tension curve then rises to `f_max` at
`v_max` like the controller's, instead of ending at about 6.9 kN there.
"""
winch_from_wc(wc; v_max = wc.v_sat, p_max = nothing, optimize_k_v = false,
              f_max = wc.f_high_awe_trim > 0 ? wc.f_high_awe_trim : wc.f_high,
              use_awe_trim = wc.use_awe_trim, winch_mode = nothing) =
    WinchParams(; mode = "reelout", k_v = wc.kv, f_min = wc.f_low,
                v_sat_beta = SEND_V_SAT_BETA && wc.force_limit == "soft" && isfinite(wc.v_sat_beta) &&
                             v_max !== nothing && v_max == wc.v_sat ?
                             Float64(wc.v_sat_beta) : nothing,
                f_max = Float64(f_max),
                v_max = v_max === nothing ? nothing : Float64(v_max),
                p_max = p_max === nothing ? nothing : Float64(p_max),
                optimize_k_v = Bool(optimize_k_v),
                use_awe_trim = Float64(use_awe_trim),
                v_reel_in = Float64(wc.v_reel_in),
                reel_in_beta = Float64(wc.reel_in_beta),
                # Sent unconditionally, not only under `force_limit = "soft"`: the
                # server's floor is `sp(beta*f_min)/beta`, so beta shapes the path
                # it plans whatever the runtime winch then does with it.
                softplus_beta = Float64(wc.softplus_beta),
                softminus_beta = Float64(AWETRIM_SOFTMINUS_BETA),
                winch_mode = winch_mode)

"""
    min_turn_radius_request(fcs, tos; scale = 1.0, c1 = nothing,
                            margin = tos.min_feasibility_margin) -> Union{Float64, Nothing}

`margin` in METRES — `margin/(c1*max_steering)`, the kite's own physical turning
limit scaled by the margin — sent WITH the request so the optimizer cannot answer
with a pattern that is about to be rejected. Defaults to `tos.min_feasibility_margin`;
pass a smaller value to ask for less than the full gate, e.g. a graduated startup
retry. `nothing` — send no constraint — when `margin` is 0, which is also where the
gate is off.

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
reply — and the run then adds `el_offset_wing` before checking, which
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
function min_turn_radius_request(fcs, tos; scale = 1.0, c1 = nothing,
                                 margin = tos.min_feasibility_margin)
    margin > 0 || return nothing
    scale >= 0 || error("min_turn_radius_request: scale must be >= 0, got $scale.")
    if !isnothing(c1) && isfinite(c1) && c1 > 0
        return scale * margin / (c1 * fcs.max_steering)
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
    return scale * margin / (coeffs.c1 * fcs.max_steering)
end

"AWETrim's own bounds on `input_depower`, from `src/awetrim/utils/defaults.py` [m]."
const DEPOWER_SEED_BOUNDS = (1.1, 2.3)

"""
    depower_seed(tos, wind_speed) -> Float64

The power-tape length `l_dp` [m] a request STARTS from, `tos.input_depower` plus
`tos.input_depower_per_wind` per m/s of wind above `tos.input_depower_wind_ref`,
clamped to [`DEPOWER_SEED_BOUNDS`](@ref) and, below that, to
`tos.input_depower_seed_max` when it is set.

A seed landing exactly on `DEPOWER_SEED_BOUNDS`' hard ceiling is itself a
failure mode, not just a value: measured 2026-08-20 at 150 m / 10 m/s, the ramp
wants 2.35 m, clamps to 2.3 m, and `/step` then runs to IPOPT's iteration cap
with no room left to move — `input_depower_seed_max` caps the ramp short of
that ceiling instead.

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
    soft_hi = tos.input_depower_seed_max > 0 ? min(hi, tos.input_depower_seed_max) : hi
    clamped = clamp(seed, lo, soft_hi)
    # Only warn when the EFFECTIVE seed still lands on AWETrim's own hard bound:
    # a clamp by input_depower_seed_max short of it is the deliberate, calibrated
    # cap that setting exists for, not the failure mode this warns about.
    if clamped != seed && (clamped == lo || clamped == hi)
        @warn @sprintf("Depower seed of %.3f m for %.1f m/s is outside AWETrim's \
                        bounds [%.3f, %.3f] m and was clamped to %.3f m. The ramp \
                        (input_depower %.2f + %.3f per m/s above %.1f m/s of \
                        data/traj_opt.yaml) has run out of tape.",
                       seed, wind_speed, lo, hi, clamped, tos.input_depower,
                       tos.input_depower_per_wind, tos.input_depower_wind_ref)
    end
    return clamped
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

"""
    elevation_min_request(fcs, tos, l_tether; extra = 0.0) -> Union{Float64, Nothing}

The elevation floor [deg] to send with a request made for tether length
`l_tether`: the highest of what the gates will demand there —
`asind(tos.min_height/l_tether)` for the clearance one and `fcs.min_elevation +
tos.candidate_elevation_margin` for the elevation one — and
`tos.pattern_elevation_min`, plus `extra`. `nothing` asks for nothing and leaves
the optimizer's own 0.6°; `tos.elevation_min_from_gates = false` sends
`tos.pattern_elevation_min` alone.

Inverting [`path_min_height`](@ref) at the length being asked for is what makes
the request and the gate the same question: AWETrim constrains HEIGHT and reaches
its floor at the END of the lap's reel-out, while the reply is installed as angles
at the anchor and judged there. The floor FALLS as the tether grows, so it belongs
with every request and not in the session — see the analogous argument for
`min_turn_radius` at the request site.

`extra` is what a retry raises the floor by after a reply was gated out, measured
off that reply and carried forward: the shortfall is structural and the next
length has it too.
"""
function elevation_min_request(fcs, tos, l_tether; extra = 0.0)
    el_min = Float64(tos.pattern_elevation_min)
    if tos.elevation_min_from_gates
        el_min = max(el_min, fcs.min_elevation + tos.candidate_elevation_margin)
        tos.min_height > 0 && l_tether > tos.min_height &&
            (el_min = max(el_min, asind(tos.min_height / l_tether)))
    end
    el_min += extra
    return el_min > 0 ? el_min : nothing
end

"""
    cap_wind_speed(tos, project_set, v_wind_gnd) -> Float64

The wind speed the elevation-cap step is keyed on: `v_wind_gnd` (the mean wind
at the project's `h_ref`, as passed to `init`) scaled to
`tos.pattern_elevation_amplitude_max_wind_height` by the project's own profile
law, or unscaled when that height is `0.0`.
"""
function cap_wind_speed(tos, project_set, v_wind_gnd)
    h = tos.pattern_elevation_amplitude_max_wind_height
    h > 0 || return Float64(v_wind_gnd)
    return calc_wind_factor(AtmosphericModel(project_set; nowindfield = true), h) *
           v_wind_gnd
end

"""
    elevation_amplitude_max_at(tos, wind_speed) -> Float64

The elevation half-span cap [deg] sent at `wind_speed`, the wind AT
`tos.pattern_elevation_amplitude_max_wind_height` (see [`cap_wind_speed`](@ref)):
`tos.pattern_elevation_amplitude_max_high` at and above
`tos.pattern_elevation_amplitude_max_wind_ref`, `tos.pattern_elevation_amplitude_max`
below it. `tos.pattern_elevation_amplitude_max_high == 0.0` disables the step;
`wind_speed = nothing` means the wind is not known and returns the base cap.

A STEP like [`guess_el_center_seed`](@ref)'s, and for the same reason: the cap
decides which basin the startup solve can reach.
"""
function elevation_amplitude_max_at(tos, wind_speed)
    tos.pattern_elevation_amplitude_max_high > 0 && !isnothing(wind_speed) &&
        wind_speed >= tos.pattern_elevation_amplitude_max_wind_ref ?
        tos.pattern_elevation_amplitude_max_high : tos.pattern_elevation_amplitude_max
end

"""
    pattern_limits_from(tos; elevation_min = nothing, wind_speed = nothing)
        -> Union{PatternLimits, Nothing}

The box the optimized pattern must stay in, from the `pattern_*` fields of
`data/traj_opt.yaml`; each is in degrees and each is off at `0.0`, and
`tos.pattern_symmetric` adds the mirror-symmetry rows. `nothing` when all six are
off, which leaves the optimizer's own defaults alone.

`elevation_min` overrides `tos.pattern_elevation_min`: it is the per-request floor
of [`elevation_min_request`](@ref), which depends on the length being asked for and
so cannot come from the file alone. `wind_speed` picks the elevation half-span cap
through [`elevation_amplitude_max_at`](@ref).
"""
function pattern_limits_from(tos; elevation_min = nothing, wind_speed = nothing)
    on(x) = !isnothing(x) && x > 0 ? Float64(x) : nothing
    limits = PatternLimits(; azimuth_max = on(tos.pattern_azimuth_max),
                           elevation_min = on(something(elevation_min,
                                                        tos.pattern_elevation_min)),
                           elevation_max = on(tos.pattern_elevation_max),
                           azimuth_amplitude_min = on(tos.pattern_azimuth_amplitude_min),
                           elevation_amplitude_max =
                               on(elevation_amplitude_max_at(tos, wind_speed)),
                           symmetric = tos.pattern_symmetric ? true : nothing)
    all(isnothing, (limits.azimuth_max, limits.elevation_min, limits.elevation_max,
                    limits.azimuth_amplitude_min, limits.elevation_amplitude_max,
                    limits.symmetric)) &&
        return nothing
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
