# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    reelout_feasibility.jl — the abort/warn policy on the feasibility gates

`include`d by `simple_opt_reelout.jl` once the optimized startup path is
installed in `fec`, and before the simulation loop is built. The CHECKS live in
the package now ([`check_reelout_feasibility`](@ref)); this file applies what the
library deliberately does not: which verdict REFUSES the run and which only
warns, with the reasons spelled out.
"""

# `c1_at`, `phase5_margin`, `check_reelout_feasibility`, `ReeloutFeasibility` and
# `Phase5MarginState` are brought in at the top of `simple_opt_reelout.jl`.

"""
    reelout_feasibility(setup, st) -> (; feas, margin5, c1_at_phase, phase5_margin_at)

Refuse a startup path that breaks the elevation floor, the clearance floor or the turn margin at the
starting length; otherwise return what the loop takes from here: `feas` (the
[`ReeloutFeasibility`](@ref) verdicts), `margin5`, the [`Phase5MarginState`](@ref) of the in-air
phase-5 check, and the two laws read off `feas`, `c1_at_phase(phase, depower | st)` and
`phase5_margin_at(az, el)`.
"""
function reelout_feasibility(setup, st)
    (; fcs, tos, fec, l_tether, el_floor, c1_at_depower, pattern_depower) = setup
    # The ELEVATION floor is a refusal, not a warning: this repo's own criterion is an
    # angle and `fig8_metrics` fails a run that breaks it.
    minimum(fec.el_path) >= el_floor ||
        error(@sprintf("The optimized path descends to %.1f°, below min_elevation \
                        %.1f° + candidate_elevation_margin %.1f° = %.1f°. AWETrim \
                        constrains height and not elevation, so this is not something \
                        the solve avoids on its own.",
                       minimum(fec.el_path), fcs.min_elevation,
                       tos.candidate_elevation_margin, el_floor))

    # The clearance floor refuses too: the optimizer earns part of its min_height by
    # reeling out within the lap, which an installed (azimuth, elevation) curve does
    # not, so it must be told here rather than flown past.
    if tos.min_height > 0
        clr = check_pattern_height(fec, l_tether, tos.min_height)
        clr.ok ||
            error(@sprintf("The optimized path's lowest point is %.1f m above ground at \
                            L = %.0f m (elevation %.1f°), below the min_height = %.0f m \
                            of data/traj_opt.yaml. Raise the guess elevation, fly a \
                            longer tether, or lower min_height.",
                           clr.height, l_tether, clr.elevation, tos.min_height))
    end

    # At the depower the pattern is FLOWN at (`pattern_depower`), not the setpoint:
    # with fly_opt_depower the kite flies the optimizer's u_d from phase 3 on.
    feas = check_reelout_feasibility(fec, fcs, tos; l_tether,
                                     depower = pattern_depower(st.opt_result))

    # A refusal, not a warning: the optimizer knows nothing of the V3's turn-rate law,
    # so a path the kite cannot turn along is a plausible thing for it to return, and
    # flying it measures the steering clamp instead of the path.
    isnothing(feas.feas_start) ||
        feas.feas_start.margin >= tos.min_feasibility_margin ||
        error(@sprintf("The optimized path asks for a turn radius of %.1f° where the \
                        kite manages %.1f° at the STARTING length %.0f m: margin %.2f, \
                        below min_feasibility_margin = %.2f. Lower body_damping, raise \
                        max_steering, or lower the margin in data/traj_opt.yaml to fly \
                        it anyway.",
                       feas.feas_start.path_radius, feas.feas_start.kite_radius,
                       l_tether, feas.feas_start.margin, tos.min_feasibility_margin))

    # The c1 to check a path against at time t — from phase 5 that is depower_final's;
    # before that the one at `depower`, which with fly_opt_depower is the depower a
    # reply carries (a candidate's own, when scoring it) or the one currently flown
    # (`st.depower_flown_opt`, when sizing a request). A cell the table cannot serve
    # falls back to the startup law, as `c1_at(feas, phase)` itself does.
    c1_at_phase(phase::Integer, depower::Real) = phase >= 5 ? c1_at(feas, phase) :
        (c1 = c1_at_depower(depower); isnan(c1) ? feas.c1 : c1)
    c1_at_phase(phase::Integer, st::RunState) =
        c1_at_phase(phase, tos.fly_opt_depower ? st.depower_flown_opt : fcs.depower_setpoint)

    # What phase 5 will fly a candidate path with; NaN when the table could not serve
    # depower_final. See phase5_margin's docstring for why this is NOT comparable to
    # the install's own margin early in the reel-out.
    phase5_margin_at(az, el) = phase5_margin(feas, az, el, fcs.reelout_l_max,
                                             fcs.max_steering)
    return (; feas, margin5 = Phase5MarginState(), c1_at_phase, phase5_margin_at)
end
