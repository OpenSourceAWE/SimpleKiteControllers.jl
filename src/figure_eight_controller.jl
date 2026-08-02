# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Figure-of-eight path-following guidance for the V3 kite.

Implements the attractor-point ("L0") guidance of Fernandes et al., Energies
2022 (doi:10.3390/en15041390): the closest point Q on the reference path to the
kite defines the cross-track error d; the attractor point R lies a fixed arc
distance (`attractor_distance`, in degrees) ahead of Q along the path. The outer
steers towards R by great-circle navigation (course `chi_set`), which a heading
PID then tracks. Unlike L1 logic, the L0 attractor is well defined at any
distance from the path, so no controller switching or approach logic is needed.

The reference path is a lemniscate in (azimuth, elevation), both in degrees.

# Sign and frame conventions (measured, see V3Kite.jl's PlanFig8.md STEP 0)

Verified against V3Kite.jl's `data/tmp_steering.arrow` and `data/tmp_sinus.arrow`
(`system_reelout.yaml`, v_wind 9.51 m/s):

- **Positive `rel_steering` produces a positive heading rate** (correlation
  +0.998 between tape position `sl.steering` and the frame-transport-corrected
  turn rate). The heading PID output is therefore fed to `rel_steering`
  *unnegated*, as in V3Kite.jl's `examples/simple_auto_parking.jl`. (The branch for the
  other kite negates; that does not transfer.)
- **The course computed from the (azimuth, elevation) trace in this module's
  convention matches `SysState.heading`**: same zero, same sign, circular-mean
  offset +13.3° with a 7.6° spread over the samples where the kite actually
  flies (>2°/s). No `neg_azimuth`, no π correction is needed anywhere in this
  file. The +13° is the kite's real course-minus-heading drift angle — the
  guidance commands a *course* while the inner loop regulates *heading*, so a
  small steady-state cross-track bias is expected and is what the heading PID's
  integral term absorbs.

Bearing convention throughout: `0` = towards zenith, positive towards larger
azimuth.

# Turn-rate feasibility

The identified turn-rate law of the V3 (V3Kite.jl's `identify_turn_rate_law` on
`tmp_steering`) is `ψ̇ = c1·v_a·u_s + c2/v_a·sin(ψ)·cos(β)`; `c1` depends on both
`body_damping` and `depower` — see [`V3_TURN_RATE_COEFFS`](@ref) and always look
it up with [`turn_rate_coeffs`](@ref) for the settings actually flown. The
minimum angular turn radius the kite can fly on the sphere follows from `ρ = (v/L)/(c1·v·u_s) = 1/(L·c1·u_s)` — note the
apparent wind speed cancels, so it depends only on tether length and steering
authority. Use [`min_turn_radius`](@ref) and [`path_min_radius`](@ref) to check
a candidate pattern against it *before* running a simulation; a pattern whose
tightest curvature is smaller than `ρ` cannot be flown however the PID is tuned.
"""

# `V3_TURN_RATE_COEFFS`, `turn_rate_coeffs`, `V3_TURN_RATE_C1` and
# `V3_TURN_RATE_C2` used to live here as a hand-maintained `Dict` (one row per
# identified `(body_damping, depower)`). They now live in `turn_rate_table.jl`
# (included before this file), backed by `data/turn_rate_coeffs.yaml` and
# interpolated in depower.

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
    # Apply rotation
    x_rot = x .* cos(theta) .- y .* sin(theta)
    y_rot = x .* sin(theta) .+ y .* cos(theta)
    y_rot = y_rot / (maximum(y_rot)/(B/2))
    # Apply translation
    x_final = x_rot .+ x0
    y_final = y_rot .+ y0
    return x_final, y_final
end

"""
    FigureEightSettings

Settings of the figure-of-eight guidance. All angles in degrees unless noted.
"""
@with_kw mutable struct FigureEightSettings @deftype Float64
    dt
    # path shape [deg]
    A = 30.0            # width of the figure-eight
    B = 12.0            # height of the figure-eight
    C = 0.0             # size of the right part
    D = 0.0             # asymmetry factor
    az_center = 0.0     # azimuth of the path center [deg]
    el_center = 60.0    # elevation of the path center [deg]
    theta = 0.0         # rotation angle [rad]
    num_points::Int = 361
    # guidance
    attractor_distance = 7.0   # [deg] arc distance from Q to the attractor
    up_loops::Bool = true      # fly upwards during the turns at large |azimuth|
    branch_tol = 3.0           # [deg] closest-point candidates within
                               # dmin + branch_tol are disambiguated by the
                               # current flight direction
    min_speed = 1.0            # [deg/s] minimum angular speed to trust the
                               # course estimate for disambiguation
    course_tau = 0.5           # [s] low-pass on the course estimate
    # Continuity of the closest point Q. The kite moves along the path
    # continuously, so Q must advance smoothly — it must not jump to the far
    # branch at the self-intersection, where the tangent is ~180° opposed.
    # Restrict the search to +-search_window degrees of arc around the previous
    # Q; 0 disables the restriction (pure global search, the original
    # behaviour). Re-acquisition is automatic: if the windowed search leaves the
    # kite further than reacquire_dist from the path, the next call searches
    # globally again, so a kite that is genuinely off-path is not trapped.
    search_window = 45.0       # [deg] arc half-width of the local search
    reacquire_dist = 25.0      # [deg] cross-track error above which the search
                               # falls back to global
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

function _build_path(fes::FigureEightSettings, az_center, el_center)
    az, el = figure_eight_path(fes.A, fes.B, fes.C, fes.D,
                               az_center, el_center, fes.theta,
                               fes.num_points)
    az = collect(az); el = collect(el)
    # drop the duplicated closing point; the path is treated as cyclic
    if isapprox(az[end], az[1]; atol=1e-9) && isapprox(el[end], el[1]; atol=1e-9)
        pop!(az); pop!(el)
    end
    n = length(az)
    # Traversal direction: the forward direction at the right lobe (max
    # azimuth) decides up- vs down-loops — during a turn the kite passes
    # the azimuth extreme moving either up (up-loop) or down (down-loop).
    imax = argmax(az)
    going_up = el[mod1(imax + 1, n)] - el[imax] > 0
    if going_up != fes.up_loops
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

function FigureEightController(fes::FigureEightSettings)
    az, el, seg_len, tangent = _build_path(fes, fes.az_center, fes.el_center)
    FigureEightController(fes, az, el, seg_len, tangent, 1,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false)
end

"""
    set_path_center!(fec::FigureEightController, az_center, el_center)

Move the reference path to a new center [deg] and rebuild it in place. Used to
walk the pattern center gradually from the capture elevation down to the
force-optimal one: a large instantaneous step demands a
heading change big enough to fight the airframe's own dynamics instead of being
smoothly captured by the guidance.
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

# Small-area spherical distance [deg] in the (azimuth, elevation) plane;
# azimuth differences are compressed by cos(elevation).
@inline function _dist(az1, el1, az2, el2)
    hypot(el2 - el1, (az2 - az1) * cosd(0.5 * (el1 + el2)))
end

# Update the course estimate (bearing convention: 0 = towards zenith,
# positive towards larger azimuth) from the position increment. The
# direction components are low-passed to avoid wrap problems.
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

1. **Continuity** (`search_window`): Q is searched only within a window of arc
   around the previous Q, so it advances along the path instead of teleporting
   to the far branch. This is the primary guard — without it the commanded
   course flips by ~180° each time the kite crosses the self-intersection, which
   slams the steering across and, on the V3, broke the model outright (a run
   diverged at t=13.3 s with `chi_set` jumping 90.6° -> -97.5°). The window is
   dropped whenever the kite is further than `reacquire_dist` from the path, so
   a genuinely off-path kite re-acquires globally.
2. **Flight direction** (`branch_tol`): among the remaining near-equal
   candidates, the branch whose tangent needs the smaller heading change wins.
"""
function calc_attractor(fec::FigureEightController, azimuth, elevation)
    fes = fec.fes
    _update_course!(fec, azimuth, elevation)
    az = fec.az_path
    el = fec.el_path
    n = length(az)
    dists = [_dist(azimuth, elevation, az[i], el[i]) for i in 1:n]

    # Candidate index set: local window around the previous Q for continuity,
    # or the whole path when off-path / not yet initialized.
    total_len = sum(fec.seg_len)
    use_window = fec.has_prev && fes.search_window > 0 && total_len > 0 &&
                 dists[fec.last_idx] <= fes.reacquire_dist
    idxs = if use_window
        half = max(1, round(Int, n * fes.search_window / total_len))
        half >= n ÷ 2 ? (1:n) : (fec.last_idx - half):(fec.last_idx + half)
    else
        1:n
    end

    dmin = Inf
    imin = fec.last_idx
    for j in idxs
        i = mod1(j, n)
        if dists[i] < dmin
            dmin = dists[i]
            imin = i
        end
    end
    iq = imin
    if fec.has_prev && fec.speed >= fes.min_speed
        # among the local distance minima within branch_tol of the minimum,
        # pick the one that requires the smaller steering effort
        best = Inf
        for j in idxs
            i = mod1(j, n)
            d = dists[i]
            if d <= dmin + fes.branch_tol &&
               d <= dists[mod1(i - 1, n)] && d <= dists[mod1(i + 1, n)]
                effort = abs(wrap2pi(fec.tangent[i] - fec.course))
                if effort < best
                    best = effort
                    iq = i
                end
            end
        end
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

Useful as an entry reference when the kite is far from the path. The great-circle
course to the attractor becomes degenerate when the kite sits almost directly
above the pattern — every attractor point is then ~"straight down", so `chi_set`
collapses onto the ±180° branch cut and its *sign* is numerical noise. The
tangent at Q has no such degeneracy (it is ~±108° from a 73° park above a
pattern centred at 30°) and additionally encodes which way round the path is
traversed, so an entry flown along it arrives moving in the right direction.
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
function path_radius_profile(fec::FigureEightController)
    az = fec.az_path; el = fec.el_path
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
path_min_radius(fec::FigureEightController) = minimum(path_radius_profile(fec))

"""
    check_pattern_feasible(fec, l_tether, max_steering; c1=V3_TURN_RATE_C1, prn=true)

Compare the reference path's tightest curvature with the kite's minimum turn
radius. Returns `(; feasible, path_radius, kite_radius, margin)` where `margin`
is `path_radius / kite_radius` (needs to be ≥ 1, and comfortably so — the kite
must also correct cross-track error while turning, which costs authority).

Pass the `c1` matching the `body_damping` in use — see
[`turn_rate_coeffs`](@ref); the default is `init`'s default damping.
"""
function check_pattern_feasible(fec::FigureEightController, l_tether,
                                max_steering; c1 = V3_TURN_RATE_C1, prn = true)
    path_radius = path_min_radius(fec)
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
