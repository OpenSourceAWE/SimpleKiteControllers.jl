# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# What a request of `awetrim_client.jl` asks the optimizer for, beside the pattern box of
# `pattern_limits.jl`: the depower it starts from and how its reply converts to V3Kite's, and the
# minimum turn radius. Pure: settings in, numbers out; only the client talks to the server.

"File in `data/` holding the identified depower conversion, see [`depower_conversion`](@ref)"
const DEPOWER_CONVERSION_FILE = "depower_conversion.yaml"

"The conversion in force; `nothing` until [`depower_conversion`](@ref) first reads the file"
const DEPOWER_CONVERSION = Ref{Union{Nothing, NamedTuple}}(nothing)

"""
    load_depower_conversion(file = joinpath(skc_data_path(), DEPOWER_CONVERSION_FILE))
        -> NamedTuple

The identified conversion `(; offset, pivot, slope, curvature, l_dp_min, l_dp_max)` of
`data/depower_conversion.yaml`, see [`awetrim_depower_to_v3kite`](@ref) and the file for
what each value is and how it was measured.
"""
function load_depower_conversion(file = joinpath(skc_data_path(), DEPOWER_CONVERSION_FILE))
    d = YAML.load_file(file)["depower_conversion"]
    return (; offset = Float64(d["offset"]), pivot = Float64(d["pivot"]),
            slope = Float64(d["slope"]), curvature = Float64(d["curvature"]),
            l_dp_min = Float64(d["l_dp_min"]), l_dp_max = Float64(d["l_dp_max"]))
end

"""
    depower_conversion() -> NamedTuple

The conversion in force, read from `data/depower_conversion.yaml` on first use. An
identification replaces it for its trial runs with [`with_depower_conversion`](@ref).
"""
function depower_conversion()
    isnothing(DEPOWER_CONVERSION[]) && (DEPOWER_CONVERSION[] = load_depower_conversion())
    return DEPOWER_CONVERSION[]
end

"""
    with_depower_conversion(f, conv)

Run `f()` with the conversion `conv` (a `(; offset, pivot, slope, curvature, l_dp_min, l_dp_max)`) in force,
restoring the previous one afterwards, also after an error.
"""
function with_depower_conversion(f, conv)
    old = depower_conversion()
    DEPOWER_CONVERSION[] = conv
    try
        return f()
    finally
        DEPOWER_CONVERSION[] = old
    end
end

"""
    awetrim_depower_to_v3kite(l_dp; conv = depower_conversion()) -> Float64

Convert an AWETrim `l_dp` [m] (`input_depower`, `l_dp = 0.6 + 5*u_p`) into the
V3Kite `rel_depower` expected to fly at the SAME tension: the plain tape geometry
`(pivot - 0.6)/5` plus the calibrated `offset` at the calibration point `pivot`, and
`slope*x + curvature*x^2` away from it, `x = l_dp - pivot`. Outside `[l_dp_min, l_dp_max]`,
the tape lengths it was identified on, it continues along the tangent at the nearer end
instead of the parabola. At slope 1/5 and no curvature this is the old
`(l_dp - 0.6)/5 + offset`. The values are identified, see `data/depower_conversion.yaml`.
"""
function awetrim_depower_to_v3kite(l_dp; conv = depower_conversion())
    x_end = clamp(l_dp, conv.l_dp_min, conv.l_dp_max) - conv.pivot
    tangent = conv.slope + 2 * conv.curvature * x_end
    return (conv.pivot - 0.6) / 5 + conv.offset + conv.slope * x_end + conv.curvature * x_end^2 +
           tangent * (l_dp - conv.pivot - x_end)
end

"AWETrim's own bounds on `input_depower`, from `src/awetrim/utils/defaults.py` [m]."
const DEPOWER_SEED_BOUNDS = (1.1, 2.3)

"""
    depower_seed(tos, wind_speed) -> Float64

The power-tape length `l_dp` [m] a request STARTS from, `tos.seed.input_depower` plus
`tos.seed.input_depower_per_wind` per m/s of wind above `tos.seed.input_depower_wind_ref`,
clamped to [`DEPOWER_SEED_BOUNDS`](@ref) and, below that, to
`tos.seed.input_depower_seed_max` when it is set.

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
    seed = tos.seed.input_depower +
           tos.seed.input_depower_per_wind * max(0.0, wind_speed - tos.seed.input_depower_wind_ref)
    lo, hi = DEPOWER_SEED_BOUNDS
    soft_hi = tos.seed.input_depower_seed_max > 0 ? min(hi, tos.seed.input_depower_seed_max) : hi
    clamped = clamp(seed, lo, soft_hi)
    # Only warn when the EFFECTIVE seed still lands on AWETrim's own hard bound:
    # a clamp by input_depower_seed_max short of it is the deliberate, calibrated
    # cap that setting exists for, not the failure mode this warns about.
    if clamped != seed && (clamped == lo || clamped == hi)
        @warn @sprintf("Depower seed of %.3f m for %.1f m/s is outside AWETrim's \
                        bounds [%.3f, %.3f] m and was clamped to %.3f m. The ramp \
                        (input_depower %.2f + %.3f per m/s above %.1f m/s of \
                        data/traj_opt.yaml) has run out of tape.",
                       seed, wind_speed, lo, hi, clamped, tos.seed.input_depower,
                       tos.seed.input_depower_per_wind, tos.seed.input_depower_wind_ref)
    end
    return clamped
end

"""
    min_turn_radius_request(fcs, tos; scale = 1.0, c1 = nothing,
                            margin = tos.gates.min_feasibility_margin) -> Union{Float64, Nothing}

`margin` in METRES — `margin/(c1*max_steering)`, the kite's own physical turning
limit scaled by the margin — sent WITH the request so the optimizer cannot answer
with a pattern that is about to be rejected. Defaults to `tos.gates.min_feasibility_margin`;
pass a smaller value to ask for less than the full gate, e.g. a graduated startup
retry. `nothing` — send no constraint — when `margin` is 0, which is also where the
gate is off.

`scale` asks for MORE than that, and a reel-out run has to: the two numbers do not
measure the same curve at the same radius, and the difference is NOT in the run's
favour. Pass `reelout_anchor_ratio(table) * tos.gates.turn_radius_headroom`.

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
`tos.gates.turn_radius_headroom` covers that half.

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

`c1` defaults to the identified turn-rate table at `fcs.course.depower_setpoint`, which is
the turn authority the pattern is flown with. PASS THE ONE THE GATE WILL USE: from
phase 5 the run flies `depower_final`, where c1 is ~23 % lower (0.2133 against
0.2752), and a request made at the pattern's c1 is then ~23 % short of what the
reply will be judged against — measured 2026-08-20, a reply of 12.15 m answering a
10.37 m request and rejected at margin 0.61, which is exactly `0.74 * c1_final/c1`
of the 0.79 it would have scored in phase 4. [`c1_at`](@ref)`(feas, phase, c1)`
is that number.

The table refuses to extrapolate off its grid. A `body_damping`/`depower_setpoint`
it cannot serve therefore sends no constraint and warns, exactly as the feasibility
GATE degrades — an off-grid run loses the advice, not the run.
"""
function min_turn_radius_request(fcs, tos; scale = 1.0, c1 = nothing,
                                 margin = tos.gates.min_feasibility_margin)
    margin > 0 || return nothing
    scale >= 0 || error("min_turn_radius_request: scale must be >= 0, got $scale.")
    if !isnothing(c1) && isfinite(c1) && c1 > 0
        return scale * margin / (c1 * fcs.course.max_steering)
    end
    coeffs = try_turn_rate_coeffs(fcs; info = false,
        consequence = "asking the optimizer for NO minimum turn radius, though \
                       min_feasibility_margin = $(tos.gates.min_feasibility_margin) will \
                       still gate the reply")
    isnothing(coeffs) && return nothing
    return scale * margin / (coeffs.c1 * fcs.course.max_steering)
end

"""
    request_constraints(tos, fcs, inflow, cap_wind, l_opt)
        -> (; turn_radius_reel, opt_r_scale, depower_request, c1_request, opt_r_min, opt_r_on,
             opt_r_sent, opt_box)

The constraints the startup solve must respect, sized at the tether length `l_opt` that is
sent to the optimizer and at the wind `inflow.wind_speed` (the only field of `inflow` read):
the minimum turn radius `opt_r_min` (with the anchor ratio `L/r` and
the gate's headroom in `opt_r_scale`; `nothing` when off, for margin 0 or an off-grid
turn-rate cell) and the pattern box `opt_box`. `opt_r_sent` starts as `opt_r_min`, the radius
the startup solve actually CONVERGED at, which the retry ladder bisects toward.

`depower_request` is the depower the reply will be FLOWN at, which is the c1 the request must
be sized at, see [`min_turn_radius_request`](@ref). That is the optimizer's
own and so unknown before the solve; the seed it starts from is the only estimate there is,
and the setpoint is NOT one: it is the depower the loop is tuned at, typically far more
powered, and a request sized there comes back a third too tight and is then gated out for a
curvature the kite never had.
"""
function request_constraints(tos, fcs, inflow, cap_wind, l_opt)
    turn_radius_reel = turn_radius_lap_reelout(tos, inflow.wind_speed)
    opt_r_scale = (1 + turn_radius_reel / l_opt) * tos.gates.turn_radius_headroom
    depower_request = awetrim_depower_to_v3kite(depower_seed(tos, inflow.wind_speed))
    c1_request = try
        turn_rate_coeffs(fcs.run.body_damping, depower_request).c1
    catch exc
        exc isa ArgumentError || rethrow()
        nothing       # off the grid: the request falls back to the setpoint and warns
    end
    opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale, c1 = c1_request)
    opt_r_on = !isnothing(opt_r_min)   # off for margin 0, or an off-grid turn-rate cell
    opt_r_sent = opt_r_min
    opt_box = pattern_limits_from(tos;
                                  elevation_min = elevation_min_request(fcs, tos, l_opt),
                                  wind_speed = cap_wind)
    isnothing(opt_r_min) && isnothing(opt_box) ||
        @info @sprintf("Constraints sent with the request: min_turn_radius %s, \
                        pattern box %s.",
                       isnothing(opt_r_min) ? "unset" :
                           @sprintf("%.2f m (min_feasibility_margin %.2f x the kite's \
                                    own at depower %.3f%s, x %.3f for %.0f m of assumed \
                                    reel-out per lap and %.2f of headroom)",
                                    opt_r_min, tos.gates.min_feasibility_margin,
                                    depower_request,
                                    " — the seed's, not the setpoint's, because the \
                                     reply is flown at its own",
                                    opt_r_scale, turn_radius_reel,
                                    tos.gates.turn_radius_headroom),
                       isnothing(opt_box) ? "unset" : string(opt_box))
    return (; turn_radius_reel, opt_r_scale, depower_request, c1_request, opt_r_min, opt_r_on,
            opt_r_sent, opt_box)
end
