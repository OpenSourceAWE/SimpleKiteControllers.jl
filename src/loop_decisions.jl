# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Decisions of the reel-out loop of `examples/simple_opt_reelout.jl`, one step at a time: numbers in,
# numbers out, no model, no optimizer, no global. The order of every arithmetic operation is that of
# the loop body they were taken from, so a run flies bit-for-bit as before.

"""
    loop_gain_scale(c1_setpoint, rel_depower_prev, c1_depower_max, c1_ctrl_at) -> Float64

Factor on the feedback gain that keeps the loop gain `heading_p * c1` at that of the depower
setpoint whatever depower is flown: `c1(setpoint) / c1(u_d)`, with `u_d` the depower of the
previous step, capped at `c1_depower_max` (`Inf`: uncapped) and rounded to three digits for the
memo of `c1_ctrl_at`. `1.0` when the setpoint's `c1` is unknown or the flown one is not positive.
"""
function loop_gain_scale(c1_setpoint, rel_depower_prev, c1_depower_max, c1_ctrl_at)
    gain_scale = 1.0
    if isfinite(c1_setpoint)
        dp_prev = round(isfinite(c1_depower_max) ?
                        min(rel_depower_prev, c1_depower_max) : rel_depower_prev;
                        digits = 3)
        c1_now = c1_ctrl_at(dp_prev)
        isfinite(c1_now) && c1_now > 0 && (gain_scale = c1_setpoint / c1_now)
    end
    return gain_scale
end

"""
    feedforward_step(fcs, dt, fec, phase, err, v_app, l_tether, v_kite, dmin, c1_setpoint,
                     gain_scale, u_filt, chi_filt) -> (u_ff, chi_ff, u_filt, chi_filt)

Curvature feed-forward plus chord correction of one step, low-passed over `fcs.feedforward.ff_tau` (see
`FC_Settings.feedforward.ff_gain`), from the path `fec` in the air. Active from phase 4 on, and while the
turn-rate coefficient at the flown depower and the kite's speed are known; otherwise `(0, 0)` and
the filters are returned unchanged. It is faded out when the kite is off this branch of the path,
by the cross-track error `dmin` and the course error `err` [rad]; `u_filt` and `chi_filt` are the
filter states of the previous step.

With `fcs.feedforward.ff_gravity_rate > 0`, `u_ff` also cancels the gravity turn
`ff_gravity_rate·sin(χ)·cos(β)` of the turn-rate law in its fixed-`c3` form, read off the path at
the same lead point ([`path_gravity_shape`](@ref)) and faded and filtered with the curvature term.
"""
function feedforward_step(fcs, dt, fec, phase, err, v_app, l_tether, v_kite, dmin, c1_setpoint,
                          gain_scale, u_filt, chi_filt)
    u_ff = 0.0
    chi_ff = 0.0
    if fcs.feedforward.ff_gain > 0 && phase >= 4
        c1_ff = c1_setpoint / gain_scale     # c1 at the flown depower
        v_app_ff = max(v_app, fcs.course.v_app_min)
        speed_ff = rad2deg(v_kite / l_tether)  # [deg/s]
        if isfinite(c1_ff) && c1_ff > 0 && speed_ff > 0
            lead_ff = fcs.feedforward.ff_lead_time * speed_ff
            psi_dot_ff = path_turn_rate(fec, lead_ff, speed_ff; smooth = fcs.feedforward.ff_smooth)
            # Faded out when the kite is not on this branch (a Q swap hands it the other lobe's curvature).
            fade_d = clamp((fcs.feedforward.ff_d_fade - dmin) / (0.5 * fcs.feedforward.ff_d_fade), 0.0, 1.0)
            fade_e = clamp((deg2rad(fcs.feedforward.ff_err_fade) - abs(err)) /
                           (0.5 * deg2rad(fcs.feedforward.ff_err_fade)), 0.0, 1.0)
            g_ff = fcs.feedforward.ff_gain * fade_d * fade_e
            alpha_ff = fcs.feedforward.ff_tau > 0 ? dt / (dt + fcs.feedforward.ff_tau) : 1.0
            # The gravity turn the kite makes unsteered; 0 = the PD holds it off with a course error.
            psi_dot_grav = fcs.feedforward.ff_gravity_rate > 0 ?
                           fcs.feedforward.ff_gravity_rate * fade_d * fade_e * path_gravity_shape(fec, lead_ff) : 0.0
            u_filt += alpha_ff * ((g_ff * psi_dot_ff - psi_dot_grav) / (c1_ff * v_app_ff) - u_filt)
            chi_filt += alpha_ff * (g_ff * path_chord_offset(fec) - chi_filt)
            u_ff = u_filt
            chi_ff = chi_filt
        end
    end
    return u_ff, chi_ff, u_filt, chi_filt
end

"""
    blended_depower(blend_from, blend_to, t0, t, T, flown_opt) -> (w, depower)

The optimizer's depower as flown in phases 3 and 4: a linear ramp over `T` seconds from `blend_from` to
`blend_to`, started at `t0`, with weight `w` in [0, 1]; `flown_opt` at once when no ramp is
running (`blend_to === nothing`).
"""
function blended_depower(blend_from, blend_to, t0, t, T, flown_opt)
    w = isnothing(blend_to) ? 1.0 : clamp((t - t0) / T, 0.0, 1.0)
    depower = isnothing(blend_to) ? flown_opt : (1 - w) * blend_from + w * blend_to
    return w, depower
end

"""
    stop_depower(fcs, dp_entry, stop_start, stop_T, t) -> Float64

Depower during the soft stop of the reel-out: a ramp over `stop_T` seconds from the depower the
stop latched at (`dp_entry`) to `fcs.reelout.depower_final`, never below `dp_entry`.
"""
function stop_depower(fcs, dp_entry, stop_start, stop_T, t)
    dp_stop_target = max(fcs.reelout.depower_final, dp_entry)
    return dp_entry + (dp_stop_target - dp_entry) * clamp((t - stop_start) / stop_T, 0.0, 1.0)
end

"""
    final_force_extra(fcs, extra, f_now, v_app, v_reelout, ramping, dt) -> Float64

Extra depower [-] of the force limiter after the stop latch: `extra` integrated on the force the
stopped drum is about to see (`f_now` scaled up by the apparent-wind ratio of the reel-out speed
going to zero) against `fcs.reelout.depower_final_f_target`, at the higher gain while the stop `ramping`, and
kept within `0..fcs.reelout.depower_final_max - fcs.reelout.depower_final`.
"""
function final_force_extra(fcs, extra, f_now, v_app, v_reelout, ramping, dt)
    v_ro = max(v_reelout, 0.0)
    f_stopped = v_app > 0 ? f_now * ((v_app + v_ro) / v_app)^2 : f_now
    f_gain = ramping ? fcs.reelout.depower_final_f_gain_stop : fcs.reelout.depower_final_f_gain
    return clamp(extra + f_gain * (f_stopped - fcs.reelout.depower_final_f_target) * dt,
                 0.0, fcs.reelout.depower_final_max - fcs.reelout.depower_final)
end

"""
    lift_should_start(fcs, stop_start, phase, v_reelout, l_set) -> Bool

Whether the elevation lift `el_offset_final` starts now: at the stop latch or in phase 5, or
`fcs.reelout.el_offset_lead` seconds before the reel-out would end at the current speed.
"""
lift_should_start(fcs, stop_start, phase, v_reelout, l_set) =
    !isnan(stop_start) || phase >= 5 ||
    (fcs.reelout.el_offset_lead > 0 && v_reelout > 0 &&
     fcs.reelout.reelout_l_max - l_set <= v_reelout * fcs.reelout.el_offset_lead)

"""
    lap_index_step(last_idx, idx_prev, n_path) -> Int

Points the closest path point Q moved since the last step, unwrapped across the `mod1` wrap of
an `n_path`-point closed path. A step moves Q by a fraction of a point; a jump of more than an
eighth of the path is Q changing branch and counts as no progress.
"""
function lap_index_step(last_idx, idx_prev, n_path)
    delta = last_idx - idx_prev
    delta < -(n_path ÷ 2) && (delta += n_path)
    delta > n_path ÷ 2 && (delta -= n_path)
    abs(delta) > n_path ÷ 8 && (delta = 0)
    return delta
end

"""
    reelout_release(fcs, t, transition_start, f_now) -> (by_timer, by_force)

Why the reel-out gate opens now, if it does: `fcs.reelout.reelout_delay` seconds after the transition began,
or earlier once the tether force reaches `fcs.reelout.reelout_f_trigger`.
"""
reelout_release(fcs, t, transition_start, f_now) =
    (t - transition_start >= fcs.reelout.reelout_delay, f_now >= fcs.reelout.reelout_f_trigger)

"""
    reelout_command(fcs, v_raw, t, start_t, f_now, f_low, f_high) -> Float64

The reel-out speed command from the winch law's `v_raw`: ramped up over `fcs.reelout.reelout_softstart`
from when the gate opened at `start_t`, but released in proportion to the tether load, so the
soft start never overrides the force limiter.
"""
function reelout_command(fcs, v_raw, t, start_t, f_now, f_low, f_high)
    ramp = fcs.reelout.reelout_softstart > 0 ?
        clamp((t - start_t) / fcs.reelout.reelout_softstart, 0.0, 1.0) : 1.0
    force_release = clamp((f_now - f_low) / (f_high - f_low), 0.0, 1.0)
    return max(ramp, force_release) * v_raw
end

"""
    soft_stop_speed(v_entry, t, stop_start, stop_T) -> Float64

Reel-out speed during the soft stop: `v_entry`, the speed at the latch, decelerated linearly to 0 over `stop_T`.
"""
soft_stop_speed(v_entry, t, stop_start, stop_T) =
    v_entry * (1 - clamp((t - stop_start) / stop_T, 0.0, 1.0))
