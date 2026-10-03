# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The box an optimized pattern must stay in, as sent with every request of `awetrim_client.jl`,
# and the edits the startup retries and re-optimizations of `examples/simple_opt_reelout.jl` make to
# it. Pure: settings and paths in, a box out. The client keeps only its JSON form.

"""
    PatternLimits(; azimuth_max, elevation_min, elevation_max,
                  azimuth_amplitude_min, elevation_amplitude_max, symmetric,
                  climb_angle_max)

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
is at the mirrored point). `climb_angle_max` caps how steeply the path may
CLIMB in the azimuth/elevation plane (elevation rising along the flight
direction: d(elevation) <= tan(value) * |d(azimuth)|, plain angles); descending
is free, so the vertical dives at the sides stay allowed. On `/step` the struct
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
    climb_angle_max::Union{Float64, Nothing} = nothing       # climb slope <= this [deg]
end

"""
    elevation_min_request(fcs, tos, l_tether; extra = 0.0) -> Union{Float64, Nothing}

The elevation floor [deg] to send with a request made for tether length
`l_tether`: the highest of what the gates will demand there —
`asind(tos.gates.min_height/l_tether)` for the clearance one and `fcs.run.min_elevation +
tos.gates.candidate_elevation_margin` for the elevation one — plus `extra`. `nothing`
asks for nothing and leaves the optimizer's own 0.6°.

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
    el_min = max(0.0, fcs.run.min_elevation + tos.gates.candidate_elevation_margin)
    tos.gates.min_height > 0 && l_tether > tos.gates.min_height &&
        (el_min = max(el_min, asind(tos.gates.min_height / l_tether)))
    el_min += extra
    return el_min > 0 ? el_min : nothing
end

"""
    elevation_amplitude_max_at(tos, wind_speed) -> Float64

The elevation half-span cap [deg] sent at `wind_speed`, the wind AT
`tos.box.pattern_elevation_amplitude_max_wind_height` (see `cap_wind_speed` in `awetrim_client.jl`):
`tos.box.pattern_elevation_amplitude_max_high` at and above
`tos.box.pattern_elevation_amplitude_max_wind_ref`, `tos.box.pattern_elevation_amplitude_max`
below it. `tos.box.pattern_elevation_amplitude_max_high == 0.0` disables the step;
`wind_speed = nothing` means the wind is not known and returns the base cap.

A STEP like `guess_el_center_seed`'s (`awetrim_client.jl`), and for the same reason: the cap
decides which basin the startup solve can reach.
"""
function elevation_amplitude_max_at(tos, wind_speed)
    tos.box.pattern_elevation_amplitude_max_high > 0 && !isnothing(wind_speed) &&
        wind_speed >= tos.box.pattern_elevation_amplitude_max_wind_ref ?
        tos.box.pattern_elevation_amplitude_max_high : tos.box.pattern_elevation_amplitude_max
end

"""
    azimuth_max_at(tos, wind_speed) -> Float64

The azimuth half-width cap [deg] sent at `wind_speed`, the same wind as
[`elevation_amplitude_max_at`](@ref) reads: `tos.box.pattern_azimuth_max_high` at and above
`tos.box.pattern_elevation_amplitude_max_wind_ref`, `tos.box.pattern_azimuth_max` below it.
`tos.box.pattern_azimuth_max_high == 0.0` disables the step; `wind_speed = nothing` returns
the base cap.
"""
function azimuth_max_at(tos, wind_speed)
    tos.box.pattern_azimuth_max_high > 0 && !isnothing(wind_speed) &&
        wind_speed >= tos.box.pattern_elevation_amplitude_max_wind_ref ?
        tos.box.pattern_azimuth_max_high : tos.box.pattern_azimuth_max
end

"""
    pattern_limits_from(tos; elevation_min = nothing, wind_speed = nothing)
        -> Union{PatternLimits, Nothing}

The box the optimized pattern must stay in, from the `pattern_*` fields of
`data/traj_opt.yaml`; each is in degrees and each is off at `0.0`, and
`tos.box.pattern_symmetric` adds the mirror-symmetry rows and `tos.box.pattern_climb_angle_max`
the climb-angle ceiling. `nothing` when all are
off, which leaves the optimizer's own defaults alone.

`elevation_min` is the per-request floor of [`elevation_min_request`](@ref), which
depends on the length being asked for and so cannot come from the file alone. `wind_speed` picks the elevation half-span cap
through [`elevation_amplitude_max_at`](@ref) and the azimuth cap through
[`azimuth_max_at`](@ref).
"""
function pattern_limits_from(tos; elevation_min = nothing, wind_speed = nothing)
    on(x) = !isnothing(x) && x > 0 ? Float64(x) : nothing
    limits = PatternLimits(; azimuth_max = on(azimuth_max_at(tos, wind_speed)),
                           elevation_min = on(elevation_min),
                           elevation_amplitude_max =
                               on(elevation_amplitude_max_at(tos, wind_speed)),
                           symmetric = tos.box.pattern_symmetric ? true : nothing,
                           climb_angle_max = on(tos.box.pattern_climb_angle_max))
    all(isnothing, (limits.azimuth_max, limits.elevation_min, limits.elevation_amplitude_max,
                    limits.symmetric, limits.climb_angle_max)) &&
        return nothing
    return limits
end

"""
    with_elevation_max(box, el_max) -> PatternLimits

`box` (a `PatternLimits` or `nothing`) with its `elevation_max` replaced by
`el_max` [deg]; every other side is kept.
"""
with_elevation_max(box, el_max) = isnothing(box) ?
    PatternLimits(; elevation_max = el_max) :
    PatternLimits(; azimuth_max = box.azimuth_max, elevation_min = box.elevation_min,
                  elevation_max = el_max,
                  azimuth_amplitude_min = box.azimuth_amplitude_min,
                  elevation_amplitude_max = box.elevation_amplitude_max,
                  symmetric = box.symmetric, climb_angle_max = box.climb_angle_max)

"""
    with_azimuth_amplitude_min(box, a_min) -> PatternLimits

`box` (a `PatternLimits` or `nothing`) with its `azimuth_amplitude_min` replaced
by `a_min` [deg]; every other side is kept.
"""
with_azimuth_amplitude_min(box, a_min) = isnothing(box) ?
    PatternLimits(; azimuth_amplitude_min = a_min) :
    PatternLimits(; azimuth_max = box.azimuth_max, elevation_min = box.elevation_min,
                  elevation_max = box.elevation_max,
                  azimuth_amplitude_min = a_min,
                  elevation_amplitude_max = box.elevation_amplitude_max,
                  symmetric = box.symmetric, climb_angle_max = box.climb_angle_max)

"""
    with_size_box(box, az_prev, el_prev, growth) -> Union{PatternLimits, Nothing}

`box` (a `PatternLimits` or `nothing`) tightened to `growth` times the size of
the path `(az_prev, el_prev)` [deg]: `azimuth_max` to `growth * max|az_prev|`,
`elevation_amplitude_max` to `growth * elevation_amplitude(el_prev)`, and the
elevation range `[elevation_min, elevation_max]` to the path's own, each end let
out by `(growth - 1)/2` of its span — every side only where that is TIGHTER
than what the box already holds, the rest kept. `growth <= 0` returns `box`
unchanged. See `TrajOptSettings.size_box_growth`.
"""
function with_size_box(box, az_prev, el_prev, growth)
    growth > 0 || return box
    az_lim = growth * maximum(abs, az_prev)
    el_lim = growth * elevation_amplitude(el_prev)
    # The RMS half-span does not bound the peak-to-peak span the gate reads, so the elevation RANGE is boxed too.
    el_lo, el_hi = extrema(el_prev)
    slack = 0.5 * (growth - 1) * (el_hi - el_lo)
    tighter(old, new) = isnothing(old) ? new : min(old, new)
    higher(old, new) = isnothing(old) ? new : max(old, new)
    isnothing(box) && return PatternLimits(; azimuth_max = az_lim,
                                           elevation_min = el_lo - slack,
                                           elevation_max = el_hi + slack,
                                           elevation_amplitude_max = el_lim)
    return PatternLimits(; azimuth_max = tighter(box.azimuth_max, az_lim),
                         elevation_min = higher(box.elevation_min, el_lo - slack),
                         elevation_max = tighter(box.elevation_max, el_hi + slack),
                         azimuth_amplitude_min = box.azimuth_amplitude_min,
                         elevation_amplitude_max = tighter(box.elevation_amplitude_max, el_lim),
                         symmetric = box.symmetric, climb_angle_max = box.climb_angle_max)
end

"""
    guess_in_box(a, b, el_center, box; fill = 0.9) -> (a, b, el_center)

The figure-eight guess of [`figure_eight_path`](@ref) — width `a` (azimuth spans ±`a`), height `b`
(peak to peak), centred at elevation `el_center`, all [deg] — shrunk and moved so it starts INSIDE
`box`: `a` to at most `fill * azimuth_max` (never below `azimuth_amplitude_min`), the half-span `b/2`
to at most `fill * elevation_amplitude_max` and to `fill` of the elevation range, and `el_center`
clamped into that `fill` band. Sides that are `nothing` (and `box = nothing`) leave the guess alone.

A cold re-optimization under the size box ([`with_size_box`](@ref)) from a guess that violates it
converges to local infeasibility (Cabauw 8 m/s, 2026-10-03: a ±30° guess against a 22.6° box).
"""
function guess_in_box(a, b, el_center, box; fill = 0.9)
    isnothing(box) && return (a, b, el_center)
    isnothing(box.azimuth_max) || (a = min(a, fill * box.azimuth_max))
    isnothing(box.azimuth_amplitude_min) || (a = max(a, box.azimuth_amplitude_min))
    isnothing(box.elevation_amplitude_max) || (b = min(b, 2 * fill * box.elevation_amplitude_max))
    lo, hi = box.elevation_min, box.elevation_max
    if !isnothing(lo) && !isnothing(hi)
        mid, half = 0.5 * (lo + hi), 0.5 * fill * (hi - lo)
        b = min(b, 2 * half)
        el_center = clamp(el_center, mid - half + b / 2, mid + half - b / 2)
    elseif !isnothing(lo)
        el_center = max(el_center, lo + b / (2 * fill))
    elseif !isnothing(hi)
        el_center = min(el_center, hi - b / (2 * fill))
    end
    return (a, b, el_center)
end
