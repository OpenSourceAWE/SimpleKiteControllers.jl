# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    reelout_feasibility.jl — may this path be flown, and with how much turn authority?

`include`d by `simple_opt_reelout.jl` once the optimized startup path is
installed in `fec`, and before the simulation loop is built. Three gates on that
path — elevation floor, ground clearance, curvature at both ends of the reel-out
— of which the last two abort the run, plus the turn-rate coefficients they are
read against.

What the loop takes from here: `el_floor` (the elevation gate a re-optimized
reply must clear), `c1_at(phase)` (the turn-rate gain of the depower flown then)
and `phase5_margin(az, el)` (what phase 5 would fly a candidate path with),
along with `margin5_flown`/`margin5_warned`, the state of that last check.
"""

# The ELEVATION floor, which is not the clearance floor and does not follow from
# it: `min_height` is satisfied at ever lower elevations as the tether grows, so a
# path that clears 50 m at 350 m of tether is at 8.2°. This repo's own criterion is
# an angle, and `fig8_metrics` fails a run that breaks it.
#
# Checked with `candidate_elevation_margin` on top, because this compares the
# REFERENCE path while the criterion scores the FLOWN one, and the kite flies below
# its reference near the lobe tips.
el_floor = fcs.min_elevation + tos.candidate_elevation_margin
minimum(fec.el_path) >= el_floor ||
    error(@sprintf("The optimized path descends to %.1f°, below min_elevation \
                    %.1f° + candidate_elevation_margin %.1f° = %.1f°. AWETrim \
                    constrains height and not elevation, so this is not something \
                    the solve avoids on its own.",
                   minimum(fec.el_path), fcs.min_elevation,
                   tos.candidate_elevation_margin, el_floor))

# The clearance floor, checked at the tether length this run flies. Independent of
# the turn-rate table above, so it runs whether or not `coeffs` was available.
if tos.min_height > 0
    clr = check_pattern_height(fec, l_tether, tos.min_height)
    clr.ok ||
        error(@sprintf("The optimized path's lowest point is %.1f m above ground at \
                        L = %.0f m (elevation %.1f°), below the min_height = %.0f m \
                        of data/traj_opt.yaml. The optimizer's own floor is a HEIGHT \
                        and it earns part of it by reeling out within the lap, which \
                        an installed (azimuth, elevation) curve does not. Raise the \
                        guess elevation, fly a longer tether, or lower min_height.",
                       clr.height, l_tether, clr.elevation, tos.min_height))
end

# Never hardcode these: both arguments move them a lot. The lookup key is
# `body_damping`, the value `init` was given: the damping the model FLIES the
# pattern with is the floor that decays out of it (0.8x by default), so the one
# value identifies both the settling transient and the flown damping c1 belongs
# to.
#
# The coefficients are DIAGNOSTIC here — they feed the feasibility check and the
# dead-time context below, no gain and no control law — so a damping/depower the
# table cannot serve costs the diagnosis, not the run. `turn_rate_coeffs` refuses
# to extrapolate (by design: c1 moves violently with both arguments), so catch
# that and fly on unadvised rather than aborting a deliberate off-grid run.
coeffs = try
    turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
catch e
    e isa ArgumentError || rethrow()
    @warn "No turn-rate coefficients for body_damping = $(fcs.body_damping), \
           depower = $(fcs.depower_setpoint) — flying WITHOUT the feasibility \
           check. Identify this cell with V3Kite.jl's steering_test_v3.jl to get \
           it back.\n$(e.msg)"
    nothing
end

c1_final = NaN            # [1/m] turn-rate gain at depower_final; NaN = not looked up
feas_final = nothing      # curvature check of the path phase 5 flies, or nothing
if isnothing(coeffs)
    c1 = c2 = delay = NaN
else
    c1, c2, delay = coeffs.c1, coeffs.c2, coeffs.delay
    @info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                    c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
                   fcs.body_damping, fcs.depower_setpoint,
                   coeffs.interpolated ? " (INTERPOLATED)" : "",
                   c1, c2, delay)

    # c1 must match the damping in use; that is what makes this check meaningful.
    # At l_tether (the START, before any reel-out) this is the WORST case: a longer
    # tether only ever shrinks the kite's minimum angular turn radius.
    feas_start = check_pattern_feasible(fec, l_tether, fcs.max_steering; c1)
    # A refusal, not a warning: the optimizer knows nothing of the V3's turn-rate
    # law, so a path the kite cannot turn along is a plausible thing for it to
    # return, and flying it measures the steering clamp instead of the path.
    feas_start.margin >= tos.min_feasibility_margin ||
        error(@sprintf("The optimized path asks for a turn radius of %.1f° where the \
                        kite manages %.1f° at the STARTING length %.0f m: margin \
                        %.2f, below min_feasibility_margin = %.2f. Lower \
                        body_damping, raise max_steering, or lower the margin in \
                        data/traj_opt.yaml to fly it anyway.",
                       feas_start.path_radius, feas_start.kite_radius, l_tether,
                       feas_start.margin, tos.min_feasibility_margin))
    feas_end = check_pattern_feasible(fec, fcs.reelout_l_max, fcs.max_steering; c1)

    # Both checks above use the PATTERN's depower. Phase 5 flies `depower_final`,
    # and c1 falls steeply with depower (0.2752 -> 0.2133 between 0.275 and 0.328,
    # -22 %), so the final laps have a margin the gate never looked at. Evaluated
    # at `reelout_l_max`, the length phase 5 begins at — a longer tether only ever
    # helps — and on the path lifted by the fixed `el_offset_final`, which is
    # installed before phase 5 settles in. The LEARNT `el_bias` is not in it: it is
    # not known until the run has flown.
    coeffs_final = if isapprox(fcs.depower_final, fcs.depower_setpoint; atol = 1e-6)
        coeffs
    else
        try
            turn_rate_coeffs(fcs.body_damping, fcs.depower_final)
        catch e
            e isa ArgumentError || rethrow()
            @warn "No turn-rate coefficients at depower_final = \
                   $(fcs.depower_final) — phase 5 flies UNCHECKED, and the in-air \
                   curvature checks keep using the phase-4 c1.\n$(e.msg)"
            nothing
        end
    end
    if !isnothing(coeffs_final)
        global c1_final = coeffs_final.c1
        global feas_final = check_pattern_feasible(fec.az_path,
            fec.el_path .+ fcs.el_offset_final, fcs.reelout_l_max, fcs.max_steering;
            c1 = c1_final, prn = false)
        @info @sprintf("Phase 5 at depower=%.3f%s: c1 = %.4f 1/m (%+.0f %% vs the \
                        pattern), curvature margin %.2f at %.0f m with the %.2f° \
                        lift (pattern margin there: %.2f).",
                       fcs.depower_final,
                       coeffs_final.interpolated ? " (INTERPOLATED)" : "",
                       c1_final, 100 * (c1_final / c1 - 1), feas_final.margin,
                       fcs.reelout_l_max, fcs.el_offset_final, feas_end.margin)
        # A warning, not the refusal `feas_start` gets: phase 5 is a handful of
        # laps at the end of a run that has already produced its power, and the
        # in-air checks below hold back any FURTHER shift on the same margin.
        feas_final.margin >= tos.min_feasibility_margin ||
            @warn @sprintf("Phase 5 asks for a turn radius of %.1f° where the kite \
                            manages %.1f° at depower_final = %.3f: margin %.2f, \
                            below min_feasibility_margin = %.2f. Lower \
                            depower_final, lower el_offset_final, or expect the \
                            final laps to fly on the steering clamp.",
                           feas_final.path_radius, feas_final.kite_radius,
                           fcs.depower_final, feas_final.margin,
                           tos.min_feasibility_margin)
    end

    # Dead-time context for attractor_dist: how long the lead arc takes to fly.
    lead_time = deg2rad(fcs.attractor_dist) * l_tether / fcs.v_app_ref
    @info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   fcs.attractor_dist, lead_time, fcs.v_app_ref, delay, lead_time / delay)
end

# The c1 to check a path against at time t. The curvature checks in the loop
# below gate what the kite is asked to fly NEXT, so they have to use the turn
# authority of the depower flown then: from phase 5 that is `depower_final`,
# ~22 % less than the pattern's. Falls back to the pattern's c1 whenever the
# table could not serve `depower_final`, which is what the loop did before.
c1_at(phase) = phase >= 5 && !isnan(c1_final) ? c1_final : c1

# What phase 5 will fly a candidate path with: its curvature margin at
# `depower_final`'s c1 and at `reelout_l_max`, the length the final laps happen
# at. Evaluated at install time, because the path installed during the LAST
# reel-out lap is the one phase 5 inherits — the gate above it only ever saw the
# pattern's turn authority. NaN when the table could not serve `depower_final`.
#
# NOT comparable to the install's own margin, which is read at `l_now`: early in
# the reel-out the two differ by length more than by depower (0.98 at 184 m
# against 1.58 at 380 m), and they converge as `l_now` approaches `reelout_l_max`.
# The number worth watching is the LAST one, where only the depower is left.
phase5_margin(az, el) = isnan(c1_final) ? NaN :
    check_pattern_feasible(az, el, fcs.reelout_l_max, fcs.max_steering;
                           c1 = c1_final, prn = false).margin
margin5_flown = NaN     # [-] phase-5 margin of the path currently installed
margin5_warned = false  # one warning per run, not one per install
