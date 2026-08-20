# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Figure-of-eight path-following guidance for the V3 kite.

Implements the attractor-point ("L0") guidance of Fernandes et al., Energies
2022 (doi:10.3390/en15041390): the closest point Q on the reference path to the
kite defines the cross-track error d; the attractor point R lies a fixed arc
distance (`attractor_distance`, in degrees) ahead of Q along the path. The outer
loop steers towards R by great-circle navigation (course `chi_set`), which a
heading PID then tracks. Unlike L1 logic, the L0 attractor is well defined at any
distance from the path, so no controller switching or approach logic is needed.

The reference path is a lemniscate in (azimuth, elevation), both in degrees.

# Sign and frame conventions

Bearing convention throughout: `0` = towards zenith, positive towards larger
azimuth. In it, `chi_set` is directly comparable to `SysState.heading` — same
zero, same sign, no `neg_azimuth` and no π correction anywhere in this file. The
two differ only by the kite's ~13° course-minus-heading drift angle, which the
guidance absorbs as a small steady-state cross-track bias.

Positive `rel_steering` produces a positive heading rate on the V3, so the
heading PID output is fed to `rel_steering` unnegated. Both facts were measured;
the evidence is in `docs/fig8_tuning_log.md`.

# Turn-rate feasibility

The V3's identified turn-rate law is `ψ̇ = c1·v_a·u_s + c2/v_a·sin(ψ)·cos(β)`,
with `c1` depending on both `body_damping` and `depower` — always look it up
with [`turn_rate_coeffs`](@ref) for the settings actually flown. The minimum
angular turn radius on the sphere is `ρ = (v/L)/(c1·v·u_s) = 1/(L·c1·u_s)`: the
apparent wind speed cancels, so it depends only on tether length and steering
authority. Check a candidate pattern with [`min_turn_radius`](@ref) and
[`path_min_radius`](@ref) *before* simulating — a pattern tighter than `ρ`
cannot be flown however the PID is tuned.
"""

"""
    figure_eight_path(A, B, C, D, x0, y0, theta, num_points)

Figure-of-eight reference path. Returns `(x, y)`: azimuth and elevation
setpoints in degrees.

- `A`: width, `B`: height, `C`: size of the right part, `D`: asymmetry
- `x0`, `y0`: center coordinates, `theta`: rotation angle [rad]
"""
function figure_eight_path(A, B, C, D, x0, y0, theta, num_points)
    t = range(0, 2π, length=num_points)
    x = A * sin.(t)
    y = B * sin.(t) .* cos.(t) .+ C .* cos.(t) .+ D .* cos.(2t)
    x_rot = x .* cos(theta) .- y .* sin(theta)
    y_rot = x .* sin(theta) .+ y .* cos(theta)
    y_rot = y_rot / (maximum(y_rot)/(B/2))
    x_final = x_rot .+ x0
    y_final = y_rot .+ y0
    return x_final, y_final
end

"""
    FigureEightSettings

Settings of the figure-of-eight guidance. All angles in degrees unless noted.

`search_window` and `reacquire_dist` keep the closest point Q continuous: Q is
searched only within `search_window` of arc around its previous position, so it
cannot jump to the far branch at the self-intersection, where the tangent is
~180° opposed. `0` disables the restriction. Above `reacquire_dist` of
cross-track error the search goes global again, so a kite genuinely off-path is
not trapped. `branch_tol`, `branch_hysteresis` and `min_speed` govern the second,
weaker guard — disambiguating near-equal candidates by the current flight
direction, with `branch_hysteresis` the alignment a challenger must beat the
incumbent by before Q moves to it. Inside that band the tie goes to the candidate
nearest the previous Q, which is what keeps the pick from oscillating where the
kite runs parallel to the path and several minima sit within `branch_tol`. See
[`calc_attractor`](@ref).

`q_rate_gain` is the guard the two above cannot be: it bounds how far Q may move
per step (that many times the arc the kite itself flew), so an argmin that swaps
between two near-equal minima is walked to rather than teleported to. It is
suspended whenever the search goes global, since re-acquisition is the one case
where Q must jump. `0` disables it.

`search_window` is an absolute arc, and a path much smaller than twice it would
leave no window at all: an optimized pattern at a long tether measures a few tens
of degrees of arc in total, where a 45° half-width spans the whole curve and the
continuity guard silently becomes a global search.
`search_window_max_frac` caps the half-width at that fraction of the path's own
arc length, so the guard scales down with the pattern instead of switching off.
A window can also go stale — after a path is installed, or after an excursion
that ended within `reacquire_dist` — and `reacquire_margin` is the second exit:
Q leaves the window when the best point on the whole path is that much closer
than the best inside it. Both searches are restricted to points aligned with
the flight direction (see [`calc_attractor`](@ref)): near the crossing the
nearest point of all is often the reverse branch, and taking it flips the
commanded course by ~180°.
"""
@with_kw mutable struct FigureEightSettings @deftype Float64
    dt
    A = 30.0            # width of the figure-eight [deg]
    B = 12.0            # height of the figure-eight [deg]
    C = 0.0             # size of the right part [deg]
    D = 0.0             # asymmetry factor [-]
    az_center = 0.0     # azimuth of the path center [deg]
    el_center = 60.0    # elevation of the path center [deg]
    theta = 0.0         # rotation angle [rad]
    num_points::Int = 361
    attractor_distance = 7.0   # arc distance from Q to the attractor [deg]
    up_loops::Bool = true      # fly upwards during the turns at large |azimuth|
    branch_tol = 3.0           # candidates within dmin + branch_tol are disambiguated [deg]
    branch_hysteresis = 10.0   # how much better aligned a candidate must be to take Q [deg]
    q_rate_gain = 2.0          # Q may advance this many times the kite's own arc per step [-]
    min_speed = 1.0            # minimum angular speed to trust the course estimate [deg/s]
    course_tau = 0.5           # low-pass on the course estimate [s]
    search_window = 45.0       # arc half-width of the local Q search [deg]
    search_window_max_frac = 0.125 # cap on that half-width, as a fraction of the path [-]
    reacquire_dist = 25.0      # cross-track error above which the search goes global [deg]
    reacquire_margin = 3.0     # how much better the global best may be before Q jumps [deg]
end

"""
    FigureEightController(fes::FigureEightSettings)

Stateful figure-of-eight guidance: holds the discretized reference path and the
filtered course estimate used to disambiguate the two branches at the
self-intersection.
"""
mutable struct FigureEightController
    fes::FigureEightSettings
    az_path::Vector{Float64}   # [deg], cyclic (no duplicated end point)
    el_path::Vector{Float64}   # [deg]
    seg_len::Vector{Float64}   # [deg] arc length of segment i -> i+1
    tangent::Vector{Float64}   # [rad] path direction at point i (chi convention)
    last_idx::Int              # index of the last closest point Q
    course::Float64            # [rad] filtered course estimate
    speed::Float64             # [deg/s] angular speed estimate
    fx::Float64                # course filter state (elevation component)
    fy::Float64                # course filter state (azimuth component)
    prev_az::Float64           # [deg]
    prev_el::Float64           # [deg]
    has_prev::Bool
end

"""
    _path_geometry(az, el, up_loops)

Turn a closed curve in (azimuth, elevation) [deg] into what the controller
holds: `(az, el, seg_len, tangent)`, the cyclic path points [deg], the arc
length of each segment [deg] and the path direction at each point [rad]. A
duplicated closing point is dropped and the traversal direction is reversed if
it does not match `up_loops`.
"""
function _path_geometry(az, el, up_loops)
    az = collect(float.(az)); el = collect(float.(el))
    length(az) == length(el) ||
        throw(ArgumentError("azimuth and elevation must have the same length"))
    length(az) >= 4 ||
        throw(ArgumentError("a closed path needs at least 4 points, got $(length(az))"))
    # drop the duplicated closing point; the path is treated as cyclic
    if isapprox(az[end], az[1]; atol=1e-9) && isapprox(el[end], el[1]; atol=1e-9)
        pop!(az); pop!(el)
    end
    n = length(az)
    # Up- vs down-loops is decided by the forward direction at the right lobe.
    imax = argmax(az)
    going_up = el[mod1(imax + 1, n)] - el[imax] > 0
    if going_up != up_loops
        reverse!(az); reverse!(el)
    end
    seg_len = zeros(n)
    tangent = zeros(n)
    for i in 1:n
        j = mod1(i + 1, n)
        d_el = el[j] - el[i]
        d_az = (az[j] - az[i]) * cosd(0.5 * (el[i] + el[j]))
        seg_len[i] = hypot(d_az, d_el)
        tangent[i] = atan(d_az, d_el)
    end
    return az, el, seg_len, tangent
end

"""
    _build_path(fes::FigureEightSettings, az_center, el_center)

Discretize the reference lemniscate about `(az_center, el_center)` and hand it
to [`_path_geometry`](@ref).
"""
function _build_path(fes::FigureEightSettings, az_center, el_center)
    az, el = figure_eight_path(fes.A, fes.B, fes.C, fes.D,
                               az_center, el_center, fes.theta,
                               fes.num_points)
    return _path_geometry(az, el, fes.up_loops)
end

function FigureEightController(fes::FigureEightSettings)
    az, el, seg_len, tangent = _build_path(fes, fes.az_center, fes.el_center)
    FigureEightController(fes, az, el, seg_len, tangent, 1,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false)
end

"""
    set_path_center!(fec::FigureEightController, az_center, el_center)

Move the reference path to a new center [deg] and rebuild it in place. Intended
for walking the center gradually from the capture elevation down to the
force-optimal one — a large step demands a heading change the guidance cannot
capture smoothly.
"""
function set_path_center!(fec::FigureEightController, az_center, el_center)
    fec.fes.az_center = az_center
    fec.fes.el_center = el_center
    az, el, seg_len, tangent = _build_path(fec.fes, az_center, el_center)
    fec.az_path = az
    fec.el_path = el
    fec.seg_len = seg_len
    fec.tangent = tangent
    nothing
end

"""
    resample_path(az, el, n) -> (az, el)

Redistribute a closed path over `n` points equidistant in arc length, by linear
interpolation between the given ones. A duplicated closing point is dropped, and
the result is cyclic: it does not repeat its first point.

The metric is the guidance's own [`_dist`](@ref), the flat-sky distance with the
azimuth axis compressed by `cos(elevation)`, since that is the metric
`attractor_distance` and `search_window` are measured in.

Any path not built here needs this: [`calc_attractor`](@ref) turns
`search_window` into an index half-width from the mean spacing, and its branch
disambiguation looks for LOCAL minima of the distance array. Both assume points
spaced evenly along the path, which a curve sampled evenly in some other
parameter — a B-spline's `s`, say — is not.
"""
function resample_path(az, el, n::Integer)
    n >= 4 || throw(ArgumentError("resampling needs at least 4 points, got $n"))
    az = collect(float.(az)); el = collect(float.(el))
    length(az) == length(el) ||
        throw(ArgumentError("azimuth and elevation must have the same length"))
    if length(az) > 1 && _dist(az[1], el[1], az[end], el[end]) < 1e-9
        pop!(az); pop!(el)
    end
    m = length(az)
    m >= 3 || throw(ArgumentError("a closed path needs at least 3 distinct points"))
    # cumulative arc length of the closed polyline; s[m + 1] is the full lap
    s = zeros(m + 1)
    for i in 1:m
        j = mod1(i + 1, m)
        s[i + 1] = s[i] + _dist(az[i], el[i], az[j], el[j])
    end
    total = s[end]
    total > 0 || throw(ArgumentError("the path has zero length"))
    out_az = zeros(n); out_el = zeros(n)
    k = 1
    for q in 1:n
        target = total * (q - 1) / n
        while k < m && s[k + 1] <= target
            k += 1
        end
        seg = s[k + 1] - s[k]
        f = seg > 0 ? (target - s[k]) / seg : 0.0
        j = mod1(k + 1, m)
        out_az[q] = az[k] + f * (az[j] - az[k])
        out_el[q] = el[k] + f * (el[j] - el[k])
    end
    return out_az, out_el
end

"""
    prepare_path(az, el; resample = 0, up_loops = true) -> (az, el)

Bring a closed path into the canonical form two paths must share before they can
be interpolated point by point: `resample` points equidistant in arc length
([`resample_path`](@ref)), traversed in the `up_loops` direction, and rotated so
that index 1 is the point of maximum azimuth.

The rotation is what makes index `i` of one path the counterpart of index `i` of
the other. Without it, two paths resampled from different starting points are
blended across a phase offset, which sweeps the reference around the pattern
instead of morphing it. The azimuth extreme is used as the anchor because it is
the same landmark [`_path_geometry`](@ref) decides the traversal direction on.
"""
function prepare_path(az, el; resample = 0, up_loops = true)
    if resample > 0
        az, el = resample_path(az, el, resample)
    end
    az, el, _, _ = _path_geometry(az, el, up_loops)
    shift = argmax(az) - 1
    shift == 0 && return az, el
    return circshift(az, -shift), circshift(el, -shift)
end

"""
    blend_paths(az0, el0, az1, el1, w) -> (az, el)

Point-by-point interpolation between two closed paths, `w = 0` giving the first
and `w = 1` the second. Both must be in the canonical form of
[`prepare_path`](@ref) and of the same length; interpolating paths that are not
aligned produces a curve that resembles neither.

Used to swap a re-optimized path in gradually: installing one in a single step is
a step in the cross-track error, and the guidance answers a step with steering.
"""
function blend_paths(az0, el0, az1, el1, w)
    length(az0) == length(az1) == length(el0) == length(el1) ||
        throw(ArgumentError("blended paths must have the same length, got \
                             $(length(az0)) and $(length(az1))"))
    v = clamp(w, 0.0, 1.0)
    return (1 - v) .* az0 .+ v .* az1, (1 - v) .* el0 .+ v .* el1
end

"""
    lobe_lift(az, el; lift = 0.0, mode = "azimuth", az_full = 10.0, az_blend = 8.0,
              depth = 0.0) -> Vector{Float64}

Per-point elevation lift [deg] to be added to the closed path `(az, el)` [deg]:
zero in the middle of the pattern and `lift` where the kite sags, so the pattern
is raised where that buys clearance instead of rigidly.

`mode` selects the coordinate the profile is keyed on:

  * `"azimuth"` — zero inside `|az| = az_full - az_blend`, `lift` beyond
    `az_full`, smoothstep between, both in DEGREES. Raises the whole lobe, its
    top included, which is a vertical translation of the part of the path that
    turns hardest and is why this profile is nearly free.
  * `"azimuth_frac"` — the same, with `az_full` and `az_blend` read as fractions
    of the path's OWN azimuth amplitude, so the profile keeps its place on a
    pattern that shrinks by a third as the tether grows.
  * `"elevation"` — keyed on the depth below the path's own elevation centre in
    units of its half-span: zero at `depth` (`0` = the centre, `1` = the path's
    lowest point), `lift` at the lowest point, smoothstep between. Raises the
    bottom only.

The elevation profile is a function of the coordinate it displaces, so it
DEFORMS the pattern rather than translating it, and what it squashes is the
lobe — the tightest turn of the path lies below the elevation centre, inside any
ramp aimed at the bottom. It also FOLDS the path, elevation ceasing to increase
monotonically along the ramp, once `1.5 * lift >= (1 - depth) * half_span`. Both
are measured in `docs/fig8_tuning_log.md`; the curvature gate refuses the result
at any lift worth having.
"""
function lobe_lift(az, el; lift = 0.0, mode = "azimuth", az_full = 10.0,
                   az_blend = 8.0, depth = 0.0)
    length(az) == length(el) ||
        throw(ArgumentError("azimuth and elevation must have the same length, got \
                             $(length(az)) and $(length(el))"))
    out = zeros(Float64, length(az))
    lift == 0 && return out
    if mode == "azimuth" || mode == "azimuth_frac"
        az_blend > 0 || throw(ArgumentError("az_blend must be positive, got $az_blend"))
        lo, hi = extrema(az)
        scale = mode == "azimuth_frac" ? 0.5 * (hi - lo) : 1.0
        scale > 0 || return out
        az_c = mode == "azimuth_frac" ? 0.5 * (hi + lo) : 0.0
        out .= lift .* _smoothstep.((abs.(az .- az_c) ./ scale .-
                                     (az_full - az_blend)) ./ az_blend)
    elseif mode == "elevation"
        depth < 1 || throw(ArgumentError("depth must be below 1, got $depth"))
        lo, hi = extrema(el)
        hi > lo || return out
        el_c, half = 0.5 * (hi + lo), 0.5 * (hi - lo)
        out .= lift .* _smoothstep.(((el_c .- el) ./ half .- depth) ./ (1 - depth))
    else
        throw(ArgumentError("unknown lobe-lift mode \"$mode\"; use \"azimuth\", \
                             \"azimuth_frac\" or \"elevation\""))
    end
    return out
end

"Hermite smoothstep, clamped to `[0, 1]` outside the ramp."
_smoothstep(x) = (u = clamp(x, 0.0, 1.0); u * u * (3 - 2u))

"""
    azimuth_frac(az, az_lo, az_hi) -> Float64

Where `az` [deg] lies across a pattern spanning `az_lo … az_hi` [deg], as a
fraction of that pattern's own half-width: `0` at its centre — the crossing —
and `1` at either lobe. A pattern with no azimuth extent gives `0`.

Normalized rather than measured in degrees because the pattern shrinks by a
third in azimuth as the tether grows, so a profile keyed on degrees walks out of
it over a reel-out run while one keyed on this stays where it was put.
"""
function azimuth_frac(az, az_lo, az_hi)
    amp = 0.5 * (az_hi - az_lo)
    amp > 0 || return 0.0
    return abs(az - 0.5 * (az_lo + az_hi)) / amp
end

"""
    azimuth_bin(az, az_lo, az_hi, nbins) -> Int

Index in `1:nbins` of the azimuth band `az` falls in, bin 1 covering the
crossing and bin `nbins` the lobes. Bands are equal in [`azimuth_frac`](@ref),
so they keep their place on a shrinking pattern; anything past the lobe lands in
the last bin.
"""
function azimuth_bin(az, az_lo, az_hi, nbins)
    nbins >= 1 || throw(ArgumentError("nbins must be positive, got $nbins"))
    return clamp(1 + floor(Int, nbins * azimuth_frac(az, az_lo, az_hi)), 1, nbins)
end

"""
    bias_lift(frac::Real, bias) -> Float64
    bias_lift(az::AbstractVector, bias) -> Vector{Float64}

Elevation lift [deg] from a per-bin profile `bias`, one value per azimuth band
of [`azimuth_bin`](@ref), at the position `frac` ([`azimuth_frac`](@ref)) or at
every point of the path `az` [deg].

The bands are only where the profile is LEARNT; what is applied is
`bias[i]` at each band's centre with a smoothstep between neighbouring centres
and a flat hold outside the outermost two, so the curve leaves every knot with
zero slope. A staircase over the bands would put a corner in the reference at
every bin edge, and the tightest turn of the pattern moves onto the corner: a
ramp 4 deg wide costs more curvature margin than the whole lobe lift buys (see
`el_offset_wing_blend`).

A single bin is a rigid shift of the whole pattern, which is what the scalar
elevation-bias learner of `examples/simple_opt_reelout.jl` does.
"""
function bias_lift(frac::Real, bias)
    nb = length(bias)
    nb >= 1 || throw(ArgumentError("the bias profile must have at least one bin"))
    nb == 1 && return Float64(bias[1])
    x = clamp(Float64(frac), 0.0, 1.0) * nb + 0.5
    i = clamp(floor(Int, x), 1, nb - 1)
    return bias[i] + _smoothstep(x - i) * (bias[i + 1] - bias[i])
end

function bias_lift(az::AbstractVector, bias)
    isempty(az) && return Float64[]
    az_lo, az_hi = extrema(az)
    return [bias_lift(azimuth_frac(a, az_lo, az_hi), bias) for a in az]
end

"""
    smooth_bins(bias, lambda) -> Vector{Float64}

Couple neighbouring bins of a learnt profile: each bin is moved a fraction
`lambda` of the way to the mean of its neighbours, `0` leaving the bins
independent and `1` replacing each by that mean.

Shape only — the profile's mean is restored afterwards, so smoothing never
touches the rigid part a scalar learner would have found and only damps the
differences between bands. Without the coupling each band closes on its own
measurement and the noisiest one (the crossing, where the kite spends the fewest
samples per lap) can pull a step into the reference that the curvature gate then
refuses.
"""
function smooth_bins(bias, lambda)
    0 <= lambda <= 1 || throw(ArgumentError("lambda must be in 0 … 1, got $lambda"))
    nb = length(bias)
    out = Float64.(collect(bias))
    (nb < 2 || lambda == 0) && return out
    for i in 1:nb
        nb_mean = i == 1 ? bias[2] : i == nb ? bias[nb - 1] :
                  0.5 * (bias[i - 1] + bias[i + 1])
        out[i] = (1 - lambda) * bias[i] + lambda * nb_mean
    end
    out .+= mean(bias) - mean(out)
    return out
end

"""
    set_path!(fec::FigureEightController, az, el; resample = 0,
              up_loops = fec.fes.up_loops)

Install an externally supplied closed path [deg] — from a trajectory optimizer,
say — in place of the lemniscate. `resample > 0` first redistributes it over that
many equidistant points ([`resample_path`](@ref)), which any path not built here
needs; `0` takes the points as given and throws if any of them repeats its
predecessor.

The shape and center fields of `fes` are NOT updated: they describe the
lemniscate that is no longer installed, and a later [`set_path_center!`](@ref)
would rebuild it and discard this path. `up_loops` is, since the traversal
direction of what was installed is a property of the path.

The course filter keeps running on purpose — it is what disambiguates the two
branches at the crossing, and resetting it mid-flight loses branch lock for
`course_tau` seconds — while the closest-point index is remapped to the point of
the NEW path nearest the old Q, so the local search window stays where the kite
actually is.
"""
function set_path!(fec::FigureEightController, az, el; resample = 0,
                   up_loops = fec.fes.up_loops)
    if resample > 0
        az, el = resample_path(az, el, resample)
    end
    new_az, new_el, seg_len, tangent = _path_geometry(az, el, up_loops)
    all(>(0), seg_len) ||
        throw(ArgumentError("the path repeats a point ($(count(iszero, seg_len)) \
                             zero-length segment(s)); resample it"))
    q_az = fec.az_path[fec.last_idx]
    q_el = fec.el_path[fec.last_idx]
    fec.az_path = new_az
    fec.el_path = new_el
    fec.seg_len = seg_len
    fec.tangent = tangent
    fec.fes.up_loops = up_loops
    fec.last_idx = argmin(i -> _dist(q_az, q_el, new_az[i], new_el[i]),
                          eachindex(new_az))
    nothing
end

"""
    _dist(az1, el1, az2, el2)

Small-area spherical distance [deg] in the (azimuth, elevation) plane; azimuth
differences are compressed by `cos(elevation)`.
"""
@inline function _dist(az1, el1, az2, el2)
    hypot(el2 - el1, (az2 - az1) * cosd(0.5 * (el1 + el2)))
end

"""
    _update_course!(fec::FigureEightController, az_deg, el_deg)

Update the filtered course and angular speed estimates from the position
increment since the last call. The direction components are low-passed
separately, which avoids wrap problems at ±180°.
"""
function _update_course!(fec::FigureEightController, az_deg, el_deg)
    fes = fec.fes
    if fec.has_prev
        d_el = el_deg - fec.prev_el
        d_az = (az_deg - fec.prev_az) * cosd(0.5 * (el_deg + fec.prev_el))
        step = hypot(d_az, d_el)
        fec.speed = step / fes.dt
        if step > 0
            alpha = fes.course_tau > 0 ? fes.dt / (fes.dt + fes.course_tau) : 1.0
            fec.fx += alpha * (d_el / step - fec.fx)
            fec.fy += alpha * (d_az / step - fec.fy)
            if hypot(fec.fx, fec.fy) > 1e-6
                fec.course = atan(fec.fy, fec.fx)
            end
        end
    end
    fec.prev_az = az_deg
    fec.prev_el = el_deg
    fec.has_prev = true
    nothing
end

"""
    calc_attractor(fec::FigureEightController, azimuth, elevation)

All angles in degrees. Find the closest path point Q and return
`(az_attr, el_attr, dmin)`: the attractor point `attractor_distance` degrees of
arc ahead of Q along the path, and the cross-track error.

Near the self-intersection of the figure-of-eight two path branches are (almost)
equally close. Two mechanisms keep Q on the right one:

0. **Flight direction, as a filter** (`min_speed`): once the course estimate is
   trusted, only points whose tangent is within 90° of it can be Q at all. Index
   distance alone cannot separate the branches — at the crossing they come within
   a few tens of points of each other on a small path — but their tangents are
   ~180° apart there. Nothing qualifying leaves the plain nearest point.
1. **Continuity** (`search_window`): Q is searched only within a window of arc
   around the previous Q, so it advances along the path instead of teleporting
   to the far branch. This is the primary guard; without it the commanded course
   flips by ~180° at every crossing of the self-intersection. The window is
   dropped beyond `reacquire_dist` from the path, so a genuinely off-path kite
   re-acquires globally, and its half-width is capped at `search_window_max_frac`
   of the path's arc length, so a small pattern keeps a window rather than losing
   the guard to a window wider than the curve.
2. **Flight direction** (`branch_tol`, `branch_hysteresis`): among the remaining
   near-equal candidates, the branch whose tangent needs the smaller heading
   change wins — but only if it wins by `branch_hysteresis`, and inside that band
   the candidate nearest the previous Q is kept instead. Without the hysteresis
   the pick oscillates wherever the kite runs nearly parallel to the path, since
   the incumbent is re-derived from this step's distances rather than carried
   over (measured 2026-08-20; see the code). It only ever sees the far branch
   when the search is global — inside a window the far branch is not a candidate
   at all, and its ~180° of tangent difference is far outside the band.
"""
function calc_attractor(fec::FigureEightController, azimuth, elevation)
    fes = fec.fes
    _update_course!(fec, azimuth, elevation)
    az = fec.az_path
    el = fec.el_path
    n = length(az)
    dists = [_dist(azimuth, elevation, az[i], el[i]) for i in 1:n]

    # Local window around the previous Q, or the whole path when off-path.
    total_len = sum(fec.seg_len)
    use_window = fec.has_prev && fes.search_window > 0 && total_len > 0 &&
                 dists[fec.last_idx] <= fes.reacquire_dist
    idxs = if use_window
        # Capped: on a small path the raw window spans the curve and stops guarding.
        half = clamp(round(Int, n * fes.search_window / total_len), 1,
                     max(1, round(Int, n * fes.search_window_max_frac)))
        half >= n ÷ 2 ? (1:n) : (fec.last_idx - half):(fec.last_idx + half)
    else
        1:n
    end

    # Q must sit where the kite is actually going: at the crossing the nearest
    # point of all can belong to the branch traversed the other way.
    trust = fec.has_prev && fec.speed >= fes.min_speed
    aligned(i) = !trust || abs(wrap2pi(fec.tangent[i] - fec.course)) <= pi / 2
    went_global = false

    dmin = Inf
    imin = fec.last_idx
    d_any = Inf
    i_any = fec.last_idx
    for j in idxs
        i = mod1(j, n)
        if dists[i] < d_any
            d_any = dists[i]
            i_any = i
        end
        if dists[i] < dmin && aligned(i)
            dmin = dists[i]
            imin = i
        end
    end
    if isinf(dmin)
        dmin = d_any
        imin = i_any
    end
    if use_window
        d_global = Inf
        i_global = imin
        for i in 1:n
            (dists[i] < d_global && aligned(i)) || continue
            d_global = dists[i]
            i_global = i
        end
        if dmin > d_global + fes.reacquire_margin
            idxs = 1:n
            dmin = d_global
            imin = i_global
            went_global = true
        end
    end
    iq = imin
    if trust
        # Of the local minima within branch_tol, take the smallest steering effort.
        # The bar starts at Q's own effort: a candidate must be better ALIGNED to
        # take it, not merely be a local minimum.
        #
        # With HYSTERESIS, because that comparison alone is not stable. It is
        # rebuilt from scratch every step and its incumbent is whichever point is
        # nearest THIS step, not the Q chosen last step, so where the kite runs
        # nearly parallel to the path the distance profile is flat, several minima
        # sit inside `branch_tol`, and a wiggle in the course estimate changes the
        # winner. Measured 2026-08-20 on the 150 -> 380 m reel-out: 24 flips
        # between t = 20 and 121 s, every one of them at |azimuth| 8.5-10.2° (both
        # lobes, nowhere near the crossing this guard exists for), each moving Q
        # about five points, the attractor ~1° and the commanded course 6-17°.
        # The cross-track error jumped with it (2.54 <-> 3.50° at t = 120.5 s) and
        # the steering answered with steps of up to 0.38, into the clamp.
        #
        # So a challenger must be better aligned by `branch_hysteresis` to be taken
        # on merit; within that band the tie goes to the candidate nearest the
        # PREVIOUS Q in arc index, which is continuity. Neither guard can freeze Q:
        # the candidate set is rebuilt from the current distances every step, so it
        # walks forward with the kite. At the crossing the two branches differ by
        # ~180° of tangent, far outside the band, so that disambiguation is
        # untouched.
        prev = fec.last_idx
        idx_gap(i) = min(mod(i - prev, n), mod(prev - i, n))
        hyst = deg2rad(fes.branch_hysteresis)
        best = abs(wrap2pi(fec.tangent[iq] - fec.course))
        best_gap = idx_gap(iq)
        for j in idxs
            i = mod1(j, n)
            d = dists[i]
            if d <= dmin + fes.branch_tol &&
               d <= dists[mod1(i - 1, n)] && d <= dists[mod1(i + 1, n)]
                effort = abs(wrap2pi(fec.tangent[i] - fec.course))
                take = if effort < best - hyst
                    true                       # clearly better aligned
                elseif effort < best + hyst
                    idx_gap(i) < best_gap      # a tie: stay with the incumbent
                else
                    false
                end
                if take
                    best = min(best, effort)
                    best_gap = idx_gap(i)
                    iq = i
                end
            end
        end
    end
    # RATE LIMIT. Everything above RANKS candidates, and the jump is upstream of the
    # ranking: measured 2026-08-20 on the 150 -> 380 m reel-out, the plain nearest
    # point — alignment and window ignored — teleports 3 to 20 indices on a
    # 360-point path, 15-16 times between t = 20 and 121 s, whenever two minima of a
    # flat distance profile swap order. Q follows, the attractor `attractor_distance`
    # beyond it moves ~1°, the commanded course steps 6-17° and the steering answers
    # with up to 0.39 of `max_steering`, sometimes into the clamp. Neither
    # `branch_tol` nor `branch_hysteresis` can see it (the tuning log has both null
    # results), because by then the argmin has already moved.
    #
    # So Q is not allowed to move faster than the KITE does: at most `q_rate_gain`
    # times the arc flown this step, and never less than one point, in either
    # direction. A swapped argmin is then walked to over a few steps instead of
    # teleported to, which bounds the attractor's motion by construction. The gain
    # is above 1 so Q can still catch up and never falls permanently behind.
    #
    # NOT applied when the search went global (`reacquire_margin`, or a kite beyond
    # `reacquire_dist`): re-acquisition is exactly the case where Q must be allowed
    # to jump, and it is guarded by its own margin. Nor on the step a new path is
    # installed — `set_path!` re-indexes Q there, before this runs.
    if trust && use_window && !went_global && fes.q_rate_gain > 0 && total_len > 0
        mean_seg = total_len / n
        arc_max = max(mean_seg, fes.q_rate_gain * fec.speed * fes.dt)
        k_max = max(1, floor(Int, arc_max / mean_seg))
        step = mod(iq - fec.last_idx, n)
        step > n ÷ 2 && (step -= n)
        abs(step) > k_max && (iq = mod1(fec.last_idx + sign(step) * k_max, n))
    end
    # walk attractor_distance degrees of arc forward from Q
    k = iq
    cum = 0.0
    while cum < fes.attractor_distance
        cum += fec.seg_len[k]
        k = mod1(k + 1, n)
        k == iq && break  # safety: never walk more than once around
    end
    fec.last_idx = iq
    return az[k], el[k], dists[iq]
end

"""
    navigate_fig8(fec::FigureEightController, azimuth, elevation)

`azimuth`/`elevation` in radians (as in `SysState`). Returns
`(chi_set, az_attr, el_attr, dmin)`: the desired flight direction [rad] towards
the attractor point by great-circle navigation, the attractor position [deg],
and the cross-track error [deg].

`chi_set` is directly comparable to `SysState.heading` — same zero, same sign
(see the convention block at the top of this file).
"""
function navigate_fig8(fec::FigureEightController, azimuth, elevation)
    az_attr, el_attr, dmin =
        calc_attractor(fec, rad2deg(azimuth), rad2deg(elevation))
    phi = azimuth
    beta = elevation
    phi_a = deg2rad(az_attr)
    beta_a = deg2rad(el_attr)
    y = sin(phi_a - phi) * cos(beta_a)
    x = cos(beta) * sin(beta_a) - sin(beta) * cos(beta_a) * cos(phi_a - phi)
    chi_set = atan(y, x)
    return chi_set, az_attr, el_attr, dmin
end

"""
    path_tangent(fec::FigureEightController) -> Float64

Direction [rad] in which the reference path is traversed at the closest point Q
found by the last [`calc_attractor`](@ref)/[`navigate_fig8`](@ref) call, in the
same bearing convention as `chi_set`.

Useful as an entry reference when the kite is far from the path: the great-circle
course to the attractor is degenerate when the kite sits almost directly above
the pattern, where every attractor point is ~"straight down" and the sign of
`chi_set` is numerical noise on the ±180° branch cut. The tangent at Q has no
such degeneracy and encodes which way round the path is traversed, so an entry
flown along it arrives moving in the right direction.
"""
path_tangent(fec::FigureEightController) = fec.tangent[fec.last_idx]

"""
    min_turn_radius(l_tether, max_steering; c1=V3_TURN_RATE_C1)

Smallest angular turn radius [deg] the kite can fly on the sphere at steering
authority `max_steering` [-] and tether length `l_tether` [m].

From `ψ̇ = c1·v_a·u_s` and the kite's angular speed `ω = v/L`, the angular radius
of curvature is `ρ = ω/ψ̇ = 1/(L·c1·u_s)` — the apparent wind speed cancels, so
this is a property of the geometry and the steering authority alone.
"""
function min_turn_radius(l_tether, max_steering; c1 = V3_TURN_RATE_C1)
    rad2deg(1 / (l_tether * c1 * max_steering))
end

"""
    path_radius_profile(fec::FigureEightController) -> Vector{Float64}

Geodesic turn radius [deg] at every point of the reference path: the arc length
divided by the turning angle, both measured on the unit sphere. `Inf` marks a
locally straight point.

Computed in true spherical geometry rather than the flat-sky
`(azimuth·cos(elevation), elevation)` approximation the guidance itself uses —
the pattern spans tens of degrees, where the two differ materially (~15% at
`el_center = 60°`).
"""
path_radius_profile(fec::FigureEightController) =
    path_radius_profile(fec.az_path, fec.el_path)

function path_radius_profile(az::AbstractVector, el::AbstractVector)
    n = length(az)
    p = [SVector(cosd(el[i]) * cosd(az[i]),
                 cosd(el[i]) * sind(az[i]),
                 sind(el[i])) for i in 1:n]
    rs = fill(Inf, n)
    for i in 1:n
        h = mod1(i - 1, n); j = mod1(i + 1, n)
        # incoming/outgoing tangents, projected into the tangent plane at p[i]
        tin = p[i] - p[h]; tin -= dot(tin, p[i]) * p[i]
        tout = p[j] - p[i]; tout -= dot(tout, p[i]) * p[i]
        ni = norm(tin); no = norm(tout)
        (ni < 1e-12 || no < 1e-12) && continue
        tin /= ni; tout /= no
        dtheta = atan(norm(cross(tin, tout)), dot(tin, tout))   # turning angle [rad]
        dtheta < 1e-12 && continue
        ds = acos(clamp(dot(p[h], p[j]), -1, 1)) / 2            # half the h->j arc
        rs[i] = rad2deg(ds / dtheta)
    end
    return rs
end

"""
    path_min_radius(fec::FigureEightController) -> Float64

Tightest geodesic turn radius [deg] of the reference path — see
[`path_radius_profile`](@ref). Compare with [`min_turn_radius`](@ref): a pattern
tighter than the kite's minimum turn radius cannot be flown at any PID tuning.

Note that the tightest point of a lemniscate in this metric is **not** the lobe
tip (where the radius is `B²/(A·cos(el_center))`) but the upper shoulder of each
lobe, where the `cos(elevation)` compression of the azimuth axis bends the path
hardest. Use [`path_radius_profile`](@ref) to see where.
"""
path_min_radius(fec::FigureEightController) = path_min_radius(fec.az_path, fec.el_path)
path_min_radius(az::AbstractVector, el::AbstractVector) =
    minimum(path_radius_profile(az, el))

"""
    path_min_height(fec::FigureEightController, l_tether) -> Float64

Height above ground [m] of the lowest point of the reference path, flown at
tether radius `l_tether`: `l_tether * sin(elevation_min)`, the straight-tether
relation. Real tether sag puts the kite slightly lower, so this is an upper
bound on the clearance.

The SHORTEST tether is the worst case, the same way it is for
[`check_pattern_feasible`](@ref) — a path specified in (azimuth, elevation) rises
as the tether grows.
"""
path_min_height(fec::FigureEightController, l_tether) =
    path_min_height(fec.az_path, fec.el_path, l_tether)
path_min_height(az::AbstractVector, el::AbstractVector, l_tether) =
    l_tether * sind(minimum(el))

"""
    check_pattern_height(fec, l_tether, min_height; prn = true)

Compare the reference path's lowest point with a ground-clearance floor
[`path_min_height`](@ref). Returns `(; ok, height, elevation, min_height)`, with
`elevation` the path's lowest elevation [deg] and `height` its height [m] at
`l_tether`.

A clearance floor is not the same criterion as an elevation floor, and the two
scale opposite ways: at a short tether a fixed height needs a high elevation, at a
long one it permits a very low one. An externally optimized path makes the
difference concrete — AWETrim constrains height (50 m by default) and not
elevation at all, but it takes its clearance partly from reeling OUT within the
lap, so the same curve installed at the anchor radius and flown at constant length
can sit well below the floor it was designed under.
"""
check_pattern_height(fec::FigureEightController, args...; kwargs...) =
    check_pattern_height(fec.az_path, fec.el_path, args...; kwargs...)

function check_pattern_height(az::AbstractVector, el::AbstractVector, l_tether,
                              min_height; prn = true)
    elevation = minimum(el)
    height = path_min_height(az, el, l_tether)
    ok = height >= min_height
    if prn
        @info @sprintf(
            "Pattern clearance: lowest point %.1f m at elevation %.1f° (L=%.0f m), floor %.0f m%s",
            height, elevation, l_tether, min_height,
            ok ? "" : "  ** TOO LOW **")
    end
    return (; ok, height, elevation, min_height)
end

"""
    check_pattern_feasible(fec, l_tether, max_steering; c1=V3_TURN_RATE_C1, prn=true)

Compare the reference path's tightest curvature with the kite's minimum turn
radius. Returns `(; feasible, path_radius, kite_radius, margin)` where `margin`
is `path_radius / kite_radius` (needs to be ≥ 1, and comfortably so — the kite
must also correct cross-track error while turning, which costs authority).

Pass the `c1` matching the `body_damping` in use — see
[`turn_rate_coeffs`](@ref); the default is `init`'s default damping.
"""
check_pattern_feasible(fec::FigureEightController, args...; kwargs...) =
    check_pattern_feasible(fec.az_path, fec.el_path, args...; kwargs...)

function check_pattern_feasible(az::AbstractVector, el::AbstractVector, l_tether,
                                max_steering; c1 = V3_TURN_RATE_C1, prn = true)
    path_radius = path_min_radius(az, el)
    kite_radius = min_turn_radius(l_tether, max_steering; c1)
    margin = path_radius / kite_radius
    feasible = margin >= 1.0
    if prn
        @info @sprintf(
            "Pattern feasibility: tightest path radius %.1f°, kite min turn radius %.1f° (L=%.0f m, u_s=%.3f) → margin %.2f%s",
            path_radius, kite_radius, l_tether, max_steering, margin,
            feasible ? "" : "  ** PATTERN TOO TIGHT **")
    end
    return (; feasible, path_radius, kite_radius, margin)
end
