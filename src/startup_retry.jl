# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The decisions of the corrected retries of the STARTUP solve (`examples/simple_opt_reelout.jl`), when the
# startup path's turn margin is below `min_feasibility_margin`: which of the levers (ceiling, width, radius)
# the next attempt pulls, and what the ladder learns from its answer. No optimizer, model or file is
# touched here, so every branch can be tested with hand-made numbers.

const RETRY_GAIN_MAX = 1.15     # largest per-attempt scaling of the turn-radius ask
const RETRY_CAP_SLACK = 0.5     # box height kept above the incumbent's own    [deg]
const RETRY_CAP_MIN_STEP = 0.5  # smallest ceiling step still worth a solve    [deg]

"The server's amplitude measure of a path's azimuth [deg]: the RMS-based
half-width its `azimuth_amplitude_min` row bounds, `sqrt(2 * mean((az - mean(az))^2))`."
azimuth_amplitude(az) = sqrt(2 * mean((az .- mean(az)) .^ 2))

"The server's amplitude measure of a path's elevation [deg]: the RMS-based
half-span its `elevation_amplitude_max` row caps, same formula as [`azimuth_amplitude`](@ref)."
elevation_amplitude(el) = azimuth_amplitude(el)

"""
    RetryLadder(; m_reply)

What the corrected startup retries know so far, starting from the incumbent's measured margin
`m_reply`. Each field is what the answers to the asks so far imply:

- `r_asked`: turn radius of the last ask that CONVERGED (`NaN` before any did) and `m_reply`
  the measured margin of that reply.
- `bisect_hi`: narrowest radius ask KNOWN to 422 (`NaN`: none yet).
- `cap_ok`: elevation ceiling of the last CONVERGED ask (`nothing`: none), `cap_bad` the highest
  ceiling KNOWN to 422 (`NaN`: none yet) and `relax_cap` whether to retry under `cap_ok`
  instead of ratcheting the ceiling down further.
- `width_ok`, `width_bad`, `relax_width`: the same for the azimuth half-width floor.
"""
Base.@kwdef mutable struct RetryLadder
    r_asked::Float64 = NaN
    m_reply::Float64
    bisect_hi::Float64 = NaN
    cap_ok::Union{Nothing, Float64} = nothing
    cap_bad::Float64 = NaN
    relax_cap::Bool = false
    width_ok::Union{Nothing, Float64} = nothing
    width_bad::Float64 = NaN
    relax_width::Bool = false
end

"""
    next_lever(ladder, tos, inc_az, inc_el, opt_r_sent, box_el_min, el_floor_start,
               radius_for_margin) -> NamedTuple

The next ask of the ladder, one lever per attempt, from the path `(inc_az, inc_el)` [deg] of
the incumbent. `tos` supplies `min_feasibility_margin` and the `startup_retry_step`, `_slack`,
`_el_cap_step` and `_az_widen_step`; `opt_r_sent` is the radius the startup solve was SENT (the
last ask that converged, before any retry did); `box_el_min` the elevation floor of the request's
box (`nothing`: none, `el_floor_start` is used) and `radius_for_margin(target)` the turn radius
the request builder asks for a margin `target`.

Returns `(; lever, r_ask, el_cap, az_min, target, prev_ask, inc_top, inc_height, inc_amp,
el_min_box, bisect_room)`, with `lever` one of `"radius bisection"`, `"radius correction"`,
`"ceiling step"`, `"width step"` and `"radius step"`; `el_cap` and `az_min` are `nothing` when
the ask leaves that side of the box alone. `lever === nothing` (then only `prev_ask` and
`bisect_room` are set) when nothing is left to try: no radius under the one that 422'd can reach
the gate, and the ceiling and width levers are spent.
"""
function next_lever(ladder::RetryLadder, tos, inc_az, inc_el, opt_r_sent, box_el_min,
                    el_floor_start, radius_for_margin)
    (; r_asked, m_reply, bisect_hi, cap_ok, cap_bad, relax_cap, width_ok, width_bad,
       relax_width) = ladder
    target = max(tos.startup_retry_step * m_reply,
                 tos.startup_retry_slack * tos.min_feasibility_margin)
    # The last CONVERGED ask; before any retry converged (`r_asked` NaN) the radius the startup solve
    # was SENT. It must be one that converged, or the bisection walks an interval with no solution at
    # either end — the request's own radius is re-measured off the reply by then and is NOT that number.
    prev_ask = isnan(r_asked) ? opt_r_sent : r_asked
    # The ceiling the ratchet would send next, clamped so the incumbent still fits above the box floor.
    inc_top = maximum(inc_el)
    inc_height = inc_top - minimum(inc_el)
    el_min_box = something(box_el_min, el_floor_start)
    cap_from = isnothing(cap_ok) ? inc_top : min(inc_top, cap_ok)
    cap_next = max(cap_from - tos.startup_retry_el_cap_step,
                   el_min_box + inc_height + RETRY_CAP_SLACK)
    cap_room = tos.startup_retry_el_cap_step > 0 &&
               cap_next <= cap_from - RETRY_CAP_MIN_STEP &&
               (isnan(cap_bad) || cap_next > cap_bad)
    # The width floor a width step would send, in the server's RMS measure, never at a floor that 422'd.
    inc_amp = azimuth_amplitude(inc_az)
    width_next = max(inc_amp, something(width_ok, 0.0)) + tos.startup_retry_az_widen_step
    width_room = tos.startup_retry_az_widen_step > 0 &&
                 (isnan(width_bad) || width_next < width_bad)
    # What a bisection can still REACH: the radius lever is proportional (the radius step scales the
    # ask by target/measured), so no radius below the 422'd `bisect_hi` beats
    # `m_reply * bisect_hi / prev_ask`. Once that ceiling is under the gate, bisecting only walks back
    # to the incumbent's own margin and the attempts belong to the geometry levers instead.
    bisect_room = !isnan(bisect_hi) &&
                  m_reply * bisect_hi / prev_ask >= tos.min_feasibility_margin
    # One lever per attempt; every rung carries the last converged ceiling and width floor.
    az_min = width_ok
    common = (; target, prev_ask, inc_top, inc_height, inc_amp, el_min_box, bisect_room)
    if bisect_room
        # A radius ask 422'd: bisect toward the last converged one instead of repeating it.
        return (; lever = "radius bisection", r_ask = (prev_ask + bisect_hi) / 2, el_cap = cap_ok,
                az_min, common...)
    elseif isnan(bisect_hi) && isnan(r_asked)
        # Attempt 1 corrects the ASSUMED lap reel-out to the measured ratio, capping the elevation if there is room.
        return (; lever = "radius correction", r_ask = radius_for_margin(target),
                el_cap = cap_room ? cap_next : cap_ok, az_min, common...)
    elseif !relax_cap && cap_room
        return (; lever = "ceiling step", r_ask = prev_ask, el_cap = cap_next, az_min, common...)
    elseif !relax_width && width_room
        return (; lever = "width step", r_ask = prev_ask, el_cap = cap_ok, az_min = width_next,
                common...)
    elseif isnan(bisect_hi)
        # Scale the previous REQUEST by target/measured, clamped to RETRY_GAIN_MAX per converged solve.
        return (; lever = "radius step",
                r_ask = prev_ask * clamp(target / m_reply, 1.0, RETRY_GAIN_MAX), el_cap = cap_ok,
                az_min, common...)
    end
    return (; lever = nothing, r_ask = NaN, el_cap = nothing, az_min = nothing, common...)
end

"""
    record_422!(ladder, ask) -> Symbol

Learn from an ask that did not converge (HTTP 422). `ask` is the `next_lever` result. Returns
`:ceiling` when the ceiling moved (never send it, or a higher one, again; the radius steps
next), `:width` when only the width floor moved (never ask for it, or a narrower one, again) and
`:radius` when only the radius did (`bisect_hi` becomes the ask, bisect toward the last
converged one).
"""
function record_422!(ladder::RetryLadder, ask)
    if !isequal(ask.el_cap, ladder.cap_ok)
        ladder.relax_cap = true
        isnothing(ask.el_cap) ||
            (ladder.cap_bad = isnan(ladder.cap_bad) ? ask.el_cap : max(ladder.cap_bad, ask.el_cap))
        return :ceiling
    elseif !isequal(ask.az_min, ladder.width_ok)
        ladder.relax_width = true
        ladder.width_bad = isnan(ladder.width_bad) ? ask.az_min : min(ladder.width_bad, ask.az_min)
        return :width
    end
    ladder.bisect_hi = ask.r_ask
    return :radius
end

"""
    record_converged!(ladder, ask, margin)

Learn from an ask that converged and measured `margin`: it is the last converged ask, no longer an
upper bound for a bisection, and its ceiling and width floor are the ones to carry on from.
"""
function record_converged!(ladder::RetryLadder, ask, margin)
    ladder.r_asked, ladder.m_reply = ask.r_ask, margin
    ladder.bisect_hi = NaN   # this ask converged, so it is no longer an upper bound
    ladder.cap_ok, ladder.relax_cap = ask.el_cap, false
    ladder.width_ok, ladder.relax_width = ask.az_min, false
    return ladder
end
