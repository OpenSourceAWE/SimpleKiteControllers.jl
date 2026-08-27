# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    ReeloutFeasibility

The turn-rate coefficients and curvature margins a reel-out run's reference path
is gated against, as returned by [`check_reelout_feasibility`](@ref). Fields:

* `c1`, `c2`, `delay` — the pattern depower's turn-rate law; `NaN` when the table
  could not serve the `(body_damping, depower_setpoint)` cell.
* `feas_start`, `feas_end` — [`check_pattern_feasible`](@ref) at the starting and
  the maximum tether length, or `nothing` when `c1` is `NaN`.
* `c1_final` — the gain at `depower_final` [1/m], `NaN` when unavailable or equal
  to the pattern's.
* `feas_final` — what phase 5 flies: the starting path lifted by `el_offset_final`,
  scored at `reelout_l_max` with `c1_final`. `nothing` when unavailable.

The example applies the abort/warn policy on top of these verdicts — which check
refuses the run and which only warns is a script decision, not a library one.
"""
Base.@kwdef struct ReeloutFeasibility
    c1::Float64 = NaN
    c2::Float64 = NaN
    delay::Float64 = NaN
    feas_start::Union{Nothing, NamedTuple} = nothing
    feas_end::Union{Nothing, NamedTuple} = nothing
    c1_final::Float64 = NaN
    feas_final::Union{Nothing, NamedTuple} = nothing
end

"""
    c1_at(f, phase) -> Float64

The turn-rate gain to score a path against at flight phase `phase`: from phase 5
that is `f.c1_final` (the depower flown then, ~22 % less authority than the
pattern's), before that `f.c1`. Falls back to the pattern's `c1` whenever the
table could not serve `depower_final`.
"""
c1_at(f::ReeloutFeasibility, phase::Integer) =
    phase >= 5 && !isnan(f.c1_final) ? f.c1_final : f.c1

"""
    phase5_margin(f, az, el) -> Float64

What phase 5 will fly a candidate path `(az, el)` with: its curvature margin at
`depower_final`'s `c1` and at `reelout_l_max`, the length the final laps happen
at. Evaluated at install time, because the path installed during the LAST reel-out
lap is the one phase 5 inherits. `NaN` when the table could not serve
`depower_final`.

NOT comparable to an install's own margin, which is read at the current length:
early in the reel-out the two differ by length more than by depower, converging as
the length approaches `reelout_l_max`. The number worth watching is the last one,
where only the depower is left.
"""
function phase5_margin(f::ReeloutFeasibility, az::AbstractVector,
                       el::AbstractVector, l_max::Real, max_steering::Real)
    isnan(f.c1_final) && return NaN
    return check_pattern_feasible(az, el, l_max, max_steering;
                                  c1 = f.c1_final, prn = false).margin
end

"""
    Phase5MarginState

Mutable state for the in-air phase-5 margin tracking during a run:
`margin` is the phase-5 margin of the path currently installed (`NaN` when the
table could not serve `depower_final`), `warned` records whether the one warning
per run has been spent.
"""
mutable struct Phase5MarginState
    margin::Float64
    warned::Bool
end
Phase5MarginState() = Phase5MarginState(NaN, false)

"""
    check_reelout_feasibility(fec, fcs, tos; l_tether) -> ReeloutFeasibility

Score an optimized reference path against the three static gates a reel-out run
needs, WITHOUT applying any policy: no `error`, no abort decision. Returns the
verdicts; the caller decides what refuses the run and what only warns.

Checks performed (all reported via `@info`/`@warn` here):

* elevation floor — the path's lowest elevation against
  `fcs.min_elevation + tos.candidate_elevation_margin`;
* ground clearance — [`check_pattern_height`](@ref) at `l_tether`, when
  `tos.min_height > 0`;
* turn-rate coefficients for `(fcs.body_damping, fcs.depower_setpoint)` — a cell
  the table cannot serve costs the diagnosis, not the run (warned, coefficients
  become `NaN`);
* curvature at the STARTING length (the worst case for one fixed path) and at
  `fcs.reelout_l_max`, plus the dead-time context for `fcs.attractor_dist`;
* phase 5 — the same path lifted by `fcs.el_offset_final`, scored at
  `depower_final`'s own `c1` (looked up separately; warned, not refused).
"""
function check_reelout_feasibility(fec::FigureEightController,
                                   fcs::FC_Settings, tos::TrajOptSettings;
                                   l_tether::Real)
    # The ELEVATION floor, which is not the clearance floor and does not follow
    # from it: `min_height` is satisfied at ever lower elevations as the tether
    # grows. Checked with `candidate_elevation_margin` on top, because this
    # compares the REFERENCE path while `fig8_metrics` scores the FLOWN one, and
    # the kite flies below its reference near the lobe tips.
    el_floor = fcs.min_elevation + tos.candidate_elevation_margin
    @info @sprintf("Elevation floor for re-optimized replies: %.2f° \
                    (min_elevation %.2f° + candidate_elevation_margin %.2f°).",
                   el_floor, fcs.min_elevation, tos.candidate_elevation_margin)

    # The clearance floor, checked at the tether length this run flies.
    if tos.min_height > 0
        clr = check_pattern_height(fec, l_tether, tos.min_height)
        clr.ok || @warn @sprintf("The optimized path's lowest point is %.1f m above \
                                  ground at L = %.0f m (elevation %.1f°), below \
                                  min_height = %.0f m.",
                                 clr.height, l_tether, clr.elevation, tos.min_height)
    end

    # The lookup key is `body_damping`, the value `init` was given: the damping
    # the model FLIES with is the floor that decays out of it, so the one value
    # identifies both. The coefficients are DIAGNOSTIC here — a damping/depower
    # the table cannot serve costs the diagnosis, not the run.
    coeffs = try
        turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
    catch e
        e isa ArgumentError || rethrow()
        @warn "No turn-rate coefficients for body_damping = $(fcs.body_damping), \
               depower = $(fcs.depower_setpoint) — flying WITHOUT the feasibility \
               check.\n$(e.msg)"
        nothing
    end

    feas = ReeloutFeasibility()
    isnothing(coeffs) && return feas

    c1, c2, delay = coeffs.c1, coeffs.c2, coeffs.delay
    @info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                    c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
                   fcs.body_damping, fcs.depower_setpoint,
                   coeffs.interpolated ? " (INTERPOLATED)" : "", c1, c2, delay)

    # At l_tether (the START) this is the WORST case for one fixed path: a longer
    # tether only ever shrinks the kite's minimum angular turn radius.
    feas_start = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                        c1, prn = false)
    @info @sprintf("Pattern feasibility at the STARTING length: margin %.2f — path \
                    radius %.1f°, kite %.1f° at L = %.0f m, u_s = %.3f. %s \
                    min_feasibility_margin = %.2f.",
                   feas_start.margin, feas_start.path_radius, feas_start.kite_radius,
                   l_tether, fcs.max_steering,
                   feas_start.margin >= tos.min_feasibility_margin ? "Clears" :
                       "BELOW the demanded",
                   tos.min_feasibility_margin)
    feas_end = check_pattern_feasible(fec, fcs.reelout_l_max, fcs.max_steering;
                                      c1, prn = false)
    @info @sprintf("The same path at reelout_l_max = %.0f m: margin %.2f — a fixed \
                    (azimuth, elevation) curve only gets easier as the tether grows.",
                   fcs.reelout_l_max, feas_end.margin)

    # Phase 5 flies `depower_final`, and c1 falls steeply with depower, so the
    # final laps have a margin the gates above never looked at. Evaluated at
    # `reelout_l_max`, on the path lifted by the fixed `el_offset_final`. The
    # LEARNT `el_bias` is not in it: it is not known until the run has flown.
    coeffs_final = if isapprox(fcs.depower_final, fcs.depower_setpoint; atol = 1e-6)
        coeffs
    else
        try
            turn_rate_coeffs(fcs.body_damping, fcs.depower_final)
        catch e
            e isa ArgumentError || rethrow()
            @warn "No turn-rate coefficients at depower_final = \
                   $(fcs.depower_final) — phase 5 flies UNCHECKED.\n$(e.msg)"
            nothing
        end
    end
    c1_final = NaN
    feas_final = nothing
    if !isnothing(coeffs_final)
        c1_final = coeffs_final.c1
        feas_final = check_pattern_feasible(fec.az_path,
            fec.el_path .+ fcs.el_offset_final, fcs.reelout_l_max, fcs.max_steering;
            c1 = c1_final, prn = false)
        @info @sprintf("Phase 5 at depower=%.3f%s: c1 = %.4f 1/m (%+.0f %% vs the \
                        pattern), curvature margin %.2f at %.0f m with the %.2f° \
                        lift (pattern margin there: %.2f).",
                       fcs.depower_final,
                       coeffs_final.interpolated ? " (INTERPOLATED)" : "",
                       c1_final, 100 * (c1_final / c1 - 1), feas_final.margin,
                       fcs.reelout_l_max, fcs.el_offset_final, feas_end.margin)
        # A warning, not a refusal: phase 5 is a handful of laps at the end of a
        # run that has already produced its power.
        feas_final.margin >= tos.min_feasibility_margin || @warn @sprintf(
            "Phase 5 asks for a turn radius of %.1f° where the kite manages %.1f° \
             at depower_final = %.3f: margin %.2f, below \
             min_feasibility_margin = %.2f.",
            feas_final.path_radius, feas_final.kite_radius, fcs.depower_final,
            feas_final.margin, tos.min_feasibility_margin)
    end

    # Dead-time context for attractor_dist: how long the lead arc takes to fly.
    lead_time = deg2rad(fcs.attractor_dist) * l_tether / fcs.v_app_ref
    @info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   fcs.attractor_dist, lead_time, fcs.v_app_ref, delay,
                   lead_time / delay)

    return ReeloutFeasibility(; c1, c2, delay, feas_start, feas_end,
                              c1_final, feas_final)
end
