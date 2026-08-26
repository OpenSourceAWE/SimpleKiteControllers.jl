# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Inner loop of the figure-of-eight flight controller: commanded course in,
`rel_steering` out. A gain-scheduled PD on a fused heading/course feedback
angle ψ' — see [`calc_steering`](@ref).
"""

"""
    CourseControllerSettings

Settings of [`CourseController`](@ref): the heading/course PID, the ψ'
fusion and the gain schedule. All angles in radians unless noted; `dt` has no
default, matching [`FigureEightSettings`](@ref).
"""
@with_kw mutable struct CourseControllerSettings @deftype Float64
    dt
    "Gain at `v_app == v_app_ref` — see `data/fc_settings.yaml`'s `heading_p`"
    heading_p = 0.1941
    "Integral time [s], or `false` for no integral action"
    heading_i::Union{Bool, Float64} = false
    "Derivative time [s]"
    heading_d = 0.12
    "Derivative filter's maximum gain"
    heading_d_n = 2.0
    "Steering command limit [-]; also the PID's output clamp"
    max_steering = 0.32
    "Apparent wind speed [m/s] the gain schedule is anchored to"
    v_app_ref = 27.0
    "Lower clamp on `v_app`, limits the gain boost [m/s]"
    v_app_min = 10.0
    "Factor on `heading_p` while `phase < 3`"
    entry_gain = 0.25
    "[m/s] at/below: pure heading feedback"
    v_kite_heading = 5.0
    "[m/s] at/above: pure course feedback; linearly blended in between"
    v_kite_course = 10.0
    "From `phase >= 3`, feed back course alone and ignore the `v_kite_*` schedule"
    fig8_pure_course::Bool = false
    """
    `SysState.course` has its zero pointing away from zenith; 0 for a source
    already in the bearing convention.
    """
    course_offset = π

    # ---- Entry descent limiter, active only while far off the path --------- #
    "Steepest commanded course while off-path [deg]; 180 disables the limiter"
    entry_chi_max = 95.0
    "Cross-track error [deg] below which the limiter is bypassed"
    entry_d_gate = 12.0
    "Width of the band [deg] above `entry_d_gate` over which the limited and raw courses are blended"
    entry_d_blend = 4.0
    "How close to ±180° [deg] `chi_set` must be before the latched tangent sign is used instead"
    entry_cut_margin = 30.0

    # ---- Open-loop entry: overrides the guidance for the dive and the hold - #
    "Course commanded during the dive [deg]"
    chi_dive = -85.0
    "Course commanded during the hold [deg]"
    chi_hold = -90.0

    # ---- Entry state machine: park -> dive -> hold -> transition ----------- #
    "Parking phase [s]: hold zero steering while settling transients decay"
    park_time = 2.0
    "Duration of the hold [s]"
    hold_time = 0.8
    "Margin above `el_center` [deg] at which the dive ends and the hold begins"
    dive_el_margin = 7.0
    "Pattern-centre elevation [deg], the ladder's 1->2 threshold"
    el_center = 26.0
    """
    Cross-track error [deg] below which phase 3 (transition) advances to phase
    4 (fig8), the first time it is crossed
    """
    fig8_d_gate = 5.0

    # ---- Depower: park and phase 3 fly at depower_setpoint ----------------- #
    "Depower held during the pattern [-]"
    depower_setpoint = 0.26
    "Depower held during the ENTRY phases (dive and hold) [-]"
    entry_depower = 0.34
    "Depower flown once phase 5 (\"final\", reel-out finished) is reached [-]"
    depower_final = 0.328
    """
    Seconds over which `rel_depower` ramps to a new phase-ladder target
    instead of stepping to it. `0` restores the hard switch.
    """
    depower_blend_time = 4.0
end

"""
    CourseControllerSettings(fcs::FC_Settings; dt) -> CourseControllerSettings

Build course-controller settings from an [`FC_Settings`](@ref), reading the
fields the two share unchanged. `course_offset` has no `FC_Settings`
counterpart and keeps its default.
"""
function CourseControllerSettings(fcs::FC_Settings; dt)
    CourseControllerSettings(; dt,
        heading_p = fcs.heading_p, heading_i = fcs.heading_i,
        heading_d = fcs.heading_d, heading_d_n = fcs.heading_d_n,
        max_steering = fcs.max_steering, v_app_ref = fcs.v_app_ref,
        v_app_min = fcs.v_app_min, entry_gain = fcs.entry_gain,
        v_kite_heading = fcs.v_kite_heading, v_kite_course = fcs.v_kite_course,
        fig8_pure_course = fcs.fig8_pure_course,
        entry_chi_max = fcs.entry_chi_max, entry_d_gate = fcs.entry_d_gate,
        entry_d_blend = fcs.entry_d_blend, entry_cut_margin = fcs.entry_cut_margin,
        chi_dive = fcs.chi_dive, chi_hold = fcs.chi_hold,
        park_time = fcs.park_time, hold_time = fcs.hold_time,
        dive_el_margin = fcs.dive_el_margin, el_center = fcs.el_center,
        fig8_d_gate = fcs.fig8_d_gate,
        depower_setpoint = fcs.depower_setpoint, entry_depower = fcs.entry_depower,
        depower_final = fcs.depower_final, depower_blend_time = fcs.depower_blend_time)
end

"""
    CourseController(ccs::CourseControllerSettings)

Stateful inner loop of the figure-of-eight flight controller: holds the
heading/course PID and the entry state machine (0 park, 1 dive, 2 hold,
3 transition, 4 fig8; 5 "final" is set from outside — see [`set_phase!`](@ref)).
"""
mutable struct CourseController
    ccs::CourseControllerSettings
    pid::DiscretePID
    "Entry state machine phase, 0-4 advanced by `calc_steering`, 5 by `set_phase!`"
    phase::Int
    "[s] sim time the hold (phase 2) began, `NaN` before it does"
    hold_start::Float64
    "Latched sign of the entry descent limiter at the ±180° cut; 0 = unset"
    entry_sign::Int
    "[rad] course actually commanded (post-limiter, post-override) of the last `calc_steering` call"
    chi_cmd::Float64
    "[-] descent-limiter blend weight of the last `calc_steering` call"
    w_lim::Float64
    "[rad] fused heading/course feedback angle of the last `calc_steering` call"
    psi_prime::Float64
    "[-] heading/course blend weight of the last `calc_steering` call"
    w_course::Float64
    "[rad] regulated error (`psi_prime - chi_cmd`) of the last `calc_steering` call"
    err::Float64
    "Phase-ladder depower TARGET of the last `calc_steering` call, `NaN` before the first one"
    depower_target::Float64
    "[-] depower value the current blend started from"
    depower_from::Float64
    "[s] sim time the current depower blend started"
    depower_t0::Float64
    "[-] rel_depower actually commanded (post-blend) of the last `calc_steering` call"
    depower_cmd::Float64
end

function CourseController(ccs::CourseControllerSettings)
    pid = DiscretePID(; K = ccs.heading_p, Ti = ccs.heading_i, Td = ccs.heading_d,
                      N = ccs.heading_d_n, Ts = ccs.dt,
                      umin = -ccs.max_steering, umax = ccs.max_steering)
    CourseController(ccs, pid, 0, NaN, 0, 0.0, 0.0, 0.0, 0.0, 0.0, NaN, NaN, NaN, NaN)
end

"""
    set_phase!(cc::CourseController, phase)

Force the entry state machine to `phase`, e.g. reel-out finishing
(`examples/simple_reelout.jl`'s phase 5, "final") — the ladder inside
[`calc_steering`](@ref) never reaches it on its own, 4 being its last state.
Rejects a LOWER phase than the current one, preserving the ladder's "never
backwards" invariant.
"""
function set_phase!(cc::CourseController, phase)
    phase >= cc.phase ||
        error("set_phase! cannot move phase backwards: $(cc.phase) -> $phase.")
    cc.phase = phase
    nothing
end

"""
    calc_steering(cc::CourseController, chi_set, heading, course;
                  t, elevation, v_kite, v_app, dmin, tangent)

Inner loop of the figure-of-eight flight controller: raw guidance course
`chi_set` [rad] in, `(rel_steering, rel_depower, phase)` out. `heading`/
`course` are `SysState.heading`/`SysState.course` [rad] (`course` shifted by
`ccs.course_offset` before use); `elevation` [rad] and `dmin` [deg, the
cross-track error] also come from `SysState`/the guidance, and `tangent`
[rad] is the reference path direction at the closest point — `dmin` and
`tangent` as scalars, so the controller never sees a `FigureEightController`.
`t` is the sim time [s].

**Entry state machine** (0 park, 1 dive, 2 hold, 3 transition, 4 fig8),
advanced first, never backwards: park -> dive at `t >= ccs.park_time`, dive ->
hold at `elevation <= ccs.el_center + ccs.dive_el_margin`, hold -> transition
`ccs.hold_time` later, transition -> fig8 at `dmin < ccs.fig8_d_gate`. Phase 5
("final") is winch-triggered and set from outside with [`set_phase!`](@ref);
this ladder never reaches it on its own.

`chi_set` then passes the descent limiter — active only while `dmin` is above
`ccs.entry_d_gate`, clamping steepness to `ccs.entry_chi_max` with the sign
latched (`cc.entry_sign`) from `tangent` near the ±180° cut — then the
open-loop override for `phase in (1, 2)` (`ccs.chi_dive`/`ccs.chi_hold`,
constant regardless of guidance). The result becomes the feedback loop's
reference `chi_cmd`.

The feedback angle ψ' blends `heading` and `course` by `v_kite` [m/s] between
`ccs.v_kite_heading` (pure heading) and `ccs.v_kite_course` (pure course);
`ccs.fig8_pure_course` forces pure course from `phase >= 3`. The gain is
scheduled by `v_app` [m/s] as `K = heading_p * v_app_ref / max(v_app, v_app_min)`,
and by phase (`entry_gain` below 3, full gain from 3); the PID output is
bypassed to `0.0` at `phase == 0` (park), though it is still stepped so
engagement stays bumpless. `rel_depower`'s TARGET is `ccs.entry_depower`
during the dive and hold, `ccs.depower_final` at phase 5, `ccs.depower_setpoint`
otherwise; a target change ramps `rel_depower` to it over `ccs.depower_blend_time`
(`0` steps instead), rather than at the phase-ladder's own boundary.

`chi_cmd`, the limiter weight, ψ', the blend weight and the regulated error
are left on `cc` as [`chi_cmd`](@ref CourseController)/`w_lim`/`psi_prime`/
`w_course`/`err` for a caller that logs them.
"""
function calc_steering(cc::CourseController, chi_set, heading, course;
                       t, elevation, v_kite, v_app, dmin, tangent)
    ccs = cc.ccs
    el_deg = rad2deg(elevation)
    if cc.phase == 0 && t >= ccs.park_time
        cc.phase = 1
    elseif cc.phase == 1 && el_deg <= ccs.el_center + ccs.dive_el_margin
        cc.phase = 2
        cc.hold_start = t
    elseif cc.phase == 2 && t - cc.hold_start >= ccs.hold_time
        cc.phase = 3
    elseif cc.phase == 3 && dmin < ccs.fig8_d_gate
        cc.phase = 4
    end
    phase = cc.phase

    chi_cmd = chi_set
    # 1 = fully limited, 0 = raw guidance, linear over entry_d_blend above the gate.
    w_lim = ccs.entry_d_blend > 0 ?
            clamp((dmin - ccs.entry_d_gate) / ccs.entry_d_blend, 0.0, 1.0) :
            (dmin > ccs.entry_d_gate ? 1.0 : 0.0)
    if w_lim > 0 && abs(chi_set) > deg2rad(ccs.entry_chi_max)
        # Only the STEEPNESS is limited; near the ±180° cut the sign is noise.
        cc.entry_sign == 0 && (cc.entry_sign = tangent >= 0 ? 1 : -1)
        sgn = abs(chi_set) < pi - deg2rad(ccs.entry_cut_margin) ?
              (chi_set >= 0 ? 1 : -1) : cc.entry_sign
        chi_lim = sgn * deg2rad(ccs.entry_chi_max)
        # Wrapped difference: a plain convex combination sweeps the long way at ±180°.
        chi_cmd = wrap2pi(chi_set + w_lim * wrap2pi(chi_lim - chi_set))
    end
    if phase == 1
        chi_cmd = deg2rad(ccs.chi_dive)
    elseif phase == 2
        chi_cmd = deg2rad(ccs.chi_hold)
    end
    cc.chi_cmd = chi_cmd
    cc.w_lim = w_lim
    w_course = if ccs.fig8_pure_course && phase >= 3
        1.0
    else
        clamp((v_kite - ccs.v_kite_heading) /
              (ccs.v_kite_course - ccs.v_kite_heading), 0.0, 1.0)
    end
    course_shifted = wrap2pi(course + ccs.course_offset)
    psi_prime = heading + w_course * wrap2pi(course_shifted - heading)
    err = wrap2pi(psi_prime - chi_cmd)
    cc.w_course = w_course
    cc.psi_prime = psi_prime
    cc.err = err
    v_app_eff = max(v_app, ccs.v_app_min)
    K_phase = phase >= 3 ? ccs.heading_p : ccs.entry_gain * ccs.heading_p
    set_K!(cc.pid, K_phase * ccs.v_app_ref / v_app_eff, 0.0, err)
    rel_steering = if phase == 0
        cc.pid(0.0, 0.0, 0.0)
        0.0
    else
        cc.pid(0.0, err, 0.0)
    end
    depower_target = if phase == 1 || phase == 2
        ccs.entry_depower
    elseif phase == 5
        ccs.depower_final
    else
        ccs.depower_setpoint
    end
    if isnan(cc.depower_target)
        # Bootstrap: nothing has flown yet, so there is no value to ramp from.
        cc.depower_from = depower_target
        cc.depower_t0 = t
    elseif depower_target != cc.depower_target
        cc.depower_from = cc.depower_cmd
        cc.depower_t0 = t
    end
    cc.depower_target = depower_target
    w_dp = ccs.depower_blend_time > 0 ?
           clamp((t - cc.depower_t0) / ccs.depower_blend_time, 0.0, 1.0) : 1.0
    rel_depower = (1 - w_dp) * cc.depower_from + w_dp * depower_target
    cc.depower_cmd = rel_depower
    return rel_steering, rel_depower, phase
end
