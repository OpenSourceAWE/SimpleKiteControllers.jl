# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    ReeloutFeasibility

The turn-rate coefficients and curvature margins a reel-out run's reference path
is gated against, as returned by [`check_reelout_feasibility`](@ref).
`c1`, `c2` and `delay` are `NaN` when the table could not serve the
`(body_damping, depower_setpoint)` cell.

[`check_startup_path`](@ref) applies the abort policy on top of these verdicts:
which check refuses the run and which only warns.

# Fields

$(TYPEDFIELDS)
"""
Base.@kwdef struct ReeloutFeasibility
    "Turn-rate gain of the pattern depower [1/m]"
    c1::Float64 = NaN
    "Turn-rate coefficient `c2` of the pattern depower [-]"
    c2::Float64 = NaN
    "Steering delay of the pattern depower's turn-rate law [s]"
    delay::Float64 = NaN
    "[`check_pattern_feasible`](@ref) at the starting tether length, `nothing` when `c1` is `NaN`"
    feas_start::Union{Nothing, NamedTuple} = nothing
    "[`check_pattern_feasible`](@ref) at the maximum tether length, `nothing` when `c1` is `NaN`"
    feas_end::Union{Nothing, NamedTuple} = nothing
    "Turn-rate gain at `depower_final` [1/m], `NaN` when unavailable or equal to the pattern's"
    c1_final::Float64 = NaN
    """
    What phase 5 flies: the starting path lifted by `el_offset_final`, scored at
    `reelout_l_max` with `c1_final`; `nothing` when unavailable
    """
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

Mutable state for the in-air phase-5 margin tracking during a run.

# Fields

$(TYPEDFIELDS)
"""
mutable struct Phase5MarginState
    "Phase-5 margin of the installed path; `NaN` if not in the table"
    margin::Float64
    "Whether the one warning per run has been spent"
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
  `fcs.run.min_elevation + tos.candidate_elevation_margin`;
* ground clearance — [`check_pattern_height`](@ref) at `l_tether`, when
  `tos.min_height > 0`;
* turn-rate coefficients for `(fcs.run.body_damping, depower)` — `depower` is the
  one the pattern is FLOWN at, `fcs.course.depower_setpoint` unless the caller flies the
  optimizer's own, where a reply judged at the setpoint's c1
  is off by `c1(flown)/c1(setpoint)`, ~22 % at 0.33 against 0.274 (Cabauw 8 m/s,
  2026-09-18); a cell the table cannot serve costs the diagnosis, not the run
  (warned, coefficients become `NaN`);
* curvature at the STARTING length (the worst case for one fixed path) and at
  `fcs.reelout.reelout_l_max`, plus the dead-time context for `fcs.pattern.attractor_dist`;
* phase 5 — the same path lifted by `fcs.reelout.el_offset_final`, scored at
  `depower_final`'s own `c1` (looked up separately; warned, not refused).
"""
function check_reelout_feasibility(fec::FigureEightController,
                                   fcs::FC_Settings, tos::TrajOptSettings;
                                   l_tether::Real, depower::Real = fcs.course.depower_setpoint)
    # The ELEVATION floor, which is not the clearance floor and does not follow
    # from it: `min_height` is satisfied at ever lower elevations as the tether
    # grows. Checked with `candidate_elevation_margin` on top, because this
    # compares the REFERENCE path while `fig8_metrics` scores the FLOWN one, and
    # the kite flies below its reference near the lobe tips.
    el_floor = fcs.run.min_elevation + tos.candidate_elevation_margin
    @info @sprintf("Elevation floor for re-optimized replies: %.2f° \
                    (min_elevation %.2f° + candidate_elevation_margin %.2f°).",
                   el_floor, fcs.run.min_elevation, tos.candidate_elevation_margin)

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
        turn_rate_coeffs(fcs.run.body_damping, depower)
    catch e
        e isa ArgumentError || rethrow()
        @warn "No turn-rate coefficients for body_damping = $(fcs.run.body_damping), \
               depower = $(depower) — flying WITHOUT the feasibility \
               check.\n$(e.msg)"
        nothing
    end

    feas = ReeloutFeasibility()
    isnothing(coeffs) && return feas

    c1, c2, delay = coeffs.c1, coeffs.c2, coeffs.delay
    @info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                    c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
                   fcs.run.body_damping, depower,
                   coeffs.interpolated ? " (INTERPOLATED)" : "", c1, c2, delay)

    # At l_tether (the START) this is the WORST case for one fixed path: a longer
    # tether only ever shrinks the kite's minimum angular turn radius.
    feas_start = check_pattern_feasible(fec, l_tether, fcs.course.max_steering;
                                        c1, prn = false)
    @info @sprintf("Pattern feasibility at the STARTING length: margin %.2f — path \
                    radius %.1f°, kite %.1f° at L = %.0f m, u_s = %.3f. %s \
                    min_feasibility_margin = %.2f.",
                   feas_start.margin, feas_start.path_radius, feas_start.kite_radius,
                   l_tether, fcs.course.max_steering,
                   feas_start.margin >= tos.min_feasibility_margin ? "Clears" :
                       "BELOW the demanded",
                   tos.min_feasibility_margin)
    feas_end = check_pattern_feasible(fec, fcs.reelout.reelout_l_max, fcs.course.max_steering;
                                      c1, prn = false)
    @info @sprintf("The same path at reelout_l_max = %.0f m: margin %.2f — a fixed \
                    (azimuth, elevation) curve only gets easier as the tether grows.",
                   fcs.reelout.reelout_l_max, feas_end.margin)

    # Phase 5 flies `depower_final`, and c1 falls steeply with depower, so the
    # final laps have a margin the gates above never looked at. Evaluated at
    # `reelout_l_max`, on the path lifted by the fixed `el_offset_final`.
    coeffs_final = if isapprox(fcs.reelout.depower_final, depower; atol = 1e-6)
        coeffs
    else
        try
            turn_rate_coeffs(fcs.run.body_damping, fcs.reelout.depower_final)
        catch e
            e isa ArgumentError || rethrow()
            @warn "No turn-rate coefficients at depower_final = \
                   $(fcs.reelout.depower_final) — phase 5 flies UNCHECKED.\n$(e.msg)"
            nothing
        end
    end
    c1_final = NaN
    feas_final = nothing
    if !isnothing(coeffs_final)
        c1_final = coeffs_final.c1
        feas_final = check_pattern_feasible(fec.az_path,
            fec.el_path .+ fcs.reelout.el_offset_final, fcs.reelout.reelout_l_max, fcs.course.max_steering;
            c1 = c1_final, prn = false)
        @info @sprintf("Phase 5 at depower=%.3f%s: c1 = %.4f 1/m (%+.0f %% vs the \
                        pattern), curvature margin %.2f at %.0f m with the %.2f° \
                        lift (pattern margin there: %.2f).",
                       fcs.reelout.depower_final,
                       coeffs_final.interpolated ? " (INTERPOLATED)" : "",
                       c1_final, 100 * (c1_final / c1 - 1), feas_final.margin,
                       fcs.reelout.reelout_l_max, fcs.reelout.el_offset_final, feas_end.margin)
        # A warning, not a refusal: phase 5 is a handful of laps at the end of a
        # run that has already produced its power.
        feas_final.margin >= tos.min_feasibility_margin || @warn @sprintf(
            "Phase 5 asks for a turn radius of %.1f° where the kite manages %.1f° \
             at depower_final = %.3f: margin %.2f, below \
             min_feasibility_margin = %.2f.",
            feas_final.path_radius, feas_final.kite_radius, fcs.reelout.depower_final,
            feas_final.margin, tos.min_feasibility_margin)
    end

    # Dead-time context for the attractor lead: how long the lead arc takes to
    # fly, at the starting length and v_app_ref (the lead itself follows the
    # flown v_app / L when attractor_lead_time is set).
    lead_deg = attractor_distance(fcs, fcs.course.v_app_ref, l_tether)
    lead_time = deg2rad(lead_deg) * l_tether / fcs.course.v_app_ref
    @info @sprintf("Attractor lead %.1f°%s ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   lead_deg, fcs.pattern.attractor_lead_time > 0 ? " (lead time)" : "",
                   lead_time, fcs.course.v_app_ref, delay, lead_time / delay)

    return ReeloutFeasibility(; c1, c2, delay, feas_start, feas_end,
                              c1_final, feas_final)
end

"""
    c1_at(f, phase, c1) -> Float64

The turn-rate gain to check a path against at flight phase `phase`, given `c1`, the
table's gain at the depower asked about: the depower a reply
carries (a candidate's own, when scoring it) or the one currently flown (when sizing
a request). From phase 5 that is `depower_final`'s, [`c1_at(f, phase)`](@ref c1_at);
before that `c1`, falling back to the startup law `f.c1` when `c1` is `NaN`, a
depower the table cannot serve.
"""
c1_at(f::ReeloutFeasibility, phase::Integer, c1::Real) =
    phase >= 5 ? c1_at(f, phase) : (isnan(c1) ? f.c1 : Float64(c1))

"""
    check_startup_path(fec, fcs, tos; l_tether, depower = fcs.course.depower_setpoint)
        -> ReeloutFeasibility

The abort policy on the startup path installed in `fec`: the gates of
[`check_reelout_feasibility`](@ref) that REFUSE a reel-out run rather than only warn,
each an `error` that says why. Returns the verdicts when the path passes.

* The ELEVATION floor, `fcs.run.min_elevation + tos.candidate_elevation_margin`: this
  repo's own criterion is an angle and `fig8_metrics` fails a run that breaks it.
  AWETrim constrains height and not elevation, so this is not something the solve
  avoids on its own.
* The clearance floor `tos.min_height` at `l_tether`, when set: the optimizer earns
  part of its `min_height` by reeling out within the lap, which an installed
  (azimuth, elevation) curve does not, so it must be told here rather than flown past.
* The curvature margin at the STARTING length against `tos.min_feasibility_margin`,
  at the depower the pattern is FLOWN at (`depower`): the optimizer knows nothing of
  the V3's turn-rate law, so a path the kite cannot turn along is a plausible thing
  for it to return, and flying it measures the steering clamp instead of the path.
  Skipped when the table cannot serve `depower` (`feas_start` is `nothing`).
"""
function check_startup_path(fec::FigureEightController, fcs::FC_Settings,
                            tos::TrajOptSettings; l_tether::Real,
                            depower::Real = fcs.course.depower_setpoint)
    el_floor = fcs.run.min_elevation + tos.candidate_elevation_margin
    minimum(fec.el_path) >= el_floor ||
        error(@sprintf("The optimized path descends to %.1f°, below min_elevation \
                        %.1f° + candidate_elevation_margin %.1f° = %.1f°. AWETrim \
                        constrains height and not elevation, so this is not something \
                        the solve avoids on its own.",
                       minimum(fec.el_path), fcs.run.min_elevation,
                       tos.candidate_elevation_margin, el_floor))
    if tos.min_height > 0
        clr = check_pattern_height(fec, l_tether, tos.min_height)
        clr.ok ||
            error(@sprintf("The optimized path's lowest point is %.1f m above ground at \
                            L = %.0f m (elevation %.1f°), below the min_height = %.0f m \
                            of data/traj_opt.yaml. Raise the guess elevation, fly a \
                            longer tether, or lower min_height.",
                           clr.height, l_tether, clr.elevation, tos.min_height))
    end
    feas = check_reelout_feasibility(fec, fcs, tos; l_tether, depower)
    isnothing(feas.feas_start) ||
        feas.feas_start.margin >= tos.min_feasibility_margin ||
        error(@sprintf("The optimized path asks for a turn radius of %.1f° where the \
                        kite manages %.1f° at the STARTING length %.0f m: margin %.2f, \
                        below min_feasibility_margin = %.2f. Lower body_damping, raise \
                        max_steering, or lower the margin in data/traj_opt.yaml to fly \
                        it anyway.",
                       feas.feas_start.path_radius, feas.feas_start.kite_radius,
                       l_tether, feas.feas_start.margin, tos.min_feasibility_margin))
    return feas
end
