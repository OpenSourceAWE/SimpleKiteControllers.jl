# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The accept gate of a re-optimized path (`examples/simple_opt_reelout.jl`): a reply of the
# optimizer is installed only if it clears every check below, in this order. Pure: numbers in,
# a verdict out, so each branch can be tested with hand-made numbers.

"""
    retried(n) -> String

How a rejection reports the retries that preceded it, `""` for the first try.
"""
retried(n) = n > 0 ? @sprintf(", after %d retries", n) : ""

"""
    gate_candidate(tos, c) -> (; verdict, reason, detail, raise, low)

Judge the candidate `c`, a NamedTuple of what was measured on the reply as it will be flown:

- `margin`: curvature margin at the current length `l_now`; `opt_r_reply`, `r_span` and
  `opt_r_min` describe the optimizer's own measure of the same curve, for the message.
- `clearance` [m] and `chk_el_min` [deg]: lowest height and elevation of the path, against
  `tos.min_height` and `el_floor`.
- `folds`: whether the blend from the path in the air folds over itself.
- `new_pred`, `opt_power_pred`, `prev_install_pred` [W]: predicted power of the candidate, of
  the startup path and of the previous install, and `power_gate_off`, whether the power gates
  are bypassed for this candidate.
- `size`: the candidate's growth against the previous install (`growth`, `az_ratio`, `el_ratio`).
- `blend_attempt`: how many cold restarts this cycle has spent already.

`verdict` is `:accept`; `:retry`, to ask for a fresh reply (`reason` says why, `low` whether it
is about height and `raise`, for a height shortfall, how many degrees to add to the elevation
floor of the next request, else `nothing`); or `:reject`, to give up on this cycle with `detail` as
the event text. The checks run in the order of the gates of `data/traj_opt.yaml`: turn margin (never
retried), clearance, elevation floor, blend fold or power, size growth.
"""
function gate_candidate(tos, c)
    retry_room = c.blend_attempt < tos.blend_max_retries
    if c.margin < tos.min_feasibility_margin
        detail = @sprintf("curvature margin %.2f%s", c.margin,
                          isnothing(c.opt_r_reply) ? "" :
                              @sprintf(" (the optimizer measured %.2f m at r = %.0f-%.0f m, \
                                        asked for >= %.2f m)",
                                       c.opt_r_reply, c.r_span[1], c.r_span[2],
                                       something(c.opt_r_min, 0.0)))
        return (; verdict = :reject, reason = "", detail, raise = nothing, low = false)
    elseif tos.min_height > 0 && c.clearance < tos.min_height
        reason = @sprintf("clearance %.1f m", c.clearance)
        # In DEGREES, the request's currency: how far the lowest point sits below the elevation demanded.
        deficit = asind(min(1.0, tos.min_height / c.l_now)) - c.chk_el_min
        if retry_room
            return (; verdict = :retry, reason, detail = "",
                    raise = deficit + tos.elevation_min_retry_margin, low = true)
        end
        return (; verdict = :reject, reason, detail = reason * retried(c.blend_attempt),
                raise = nothing, low = false)
    elseif c.chk_el_min < c.el_floor
        # The clearance floor does NOT imply this one: at 318 m, 50 m of height is 9° of elevation.
        reason = @sprintf("descends to %.1f°, below min_elevation + margin = %.1f°",
                          c.chk_el_min, c.el_floor)
        if retry_room
            return (; verdict = :retry, reason, detail = "",
                    raise = c.el_floor - c.chk_el_min + tos.elevation_min_retry_margin, low = true)
        end
        return (; verdict = :reject, reason, detail = reason * retried(c.blend_attempt),
                raise = nothing, low = false)
    elseif c.folds ||
           (!c.power_gate_off &&
            (c.new_pred < tos.min_power_frac * c.opt_power_pred ||
             c.new_pred < tos.min_power_frac_prev * c.prev_install_pred))
        reason = if c.folds
            "blend folds"
        elseif c.new_pred < tos.min_power_frac * c.opt_power_pred
            @sprintf("%.0f W predicted, below %.0f%% of the startup prediction (%.0f W)",
                     c.new_pred, 100 * tos.min_power_frac, c.opt_power_pred)
        else
            @sprintf("%.0f W predicted, below %.0f%% of the previous install's (%.0f W)",
                     c.new_pred, 100 * tos.min_power_frac_prev, c.prev_install_pred)
        end
        retry_room && return (; verdict = :retry, reason, detail = "", raise = nothing, low = false)
        return (; verdict = :reject, reason,
                detail = @sprintf("%s, after %d retries", reason, tos.blend_max_retries),
                raise = nothing, low = false)
    elseif tos.max_size_growth > 0 && c.size.growth > tos.max_size_growth
        # The continuity gate: a reply from another basin passes every gate above BY BEING BIG.
        reason = @sprintf("%.2fx the previous install's size (azimuth half-width x%.2f, \
                           elevation span x%.2f), above max_size_growth = %.2f",
                          c.size.growth, c.size.az_ratio, c.size.el_ratio, tos.max_size_growth)
        retry_room && return (; verdict = :retry, reason, detail = "", raise = nothing, low = false)
        return (; verdict = :reject, reason,
                detail = @sprintf("%s, after %d retries", reason, tos.blend_max_retries),
                raise = nothing, low = false)
    end
    return (; verdict = :accept, reason = "", detail = "", raise = nothing, low = false)
end

"""
    wants_challenge(tos, c) -> Bool

Whether an ACCEPTED reply looks like a step into a worse basin and is cross-checked by a cold
solve (`challenge_growth`): it GREW the pattern by more than `tos.challenge_growth` against the
previous install and predicts LESS power than that install. `c` holds `size`, `new_pred` and
`prev_install_pred` as in [`gate_candidate`](@ref); a `NaN` `prev_install_pred` (before the
first re-optimization has installed a path) never asks.
"""
wants_challenge(tos, c) =
    tos.challenge_growth > 0 && !isnan(c.prev_install_pred) &&
    c.size.growth > tos.challenge_growth && c.new_pred < c.prev_install_pred

"""
    blend_folds(tos, az0, el0, az1, el1) -> Bool

Does `blend_paths` between these two closed curves collapse `path_min_radius`
anywhere across `w` in `[0, 1]`, relative to the smaller of the two endpoints'
own radius? Sampled at `tos.blend_probe_points` points; a fold shows up as a
near-zero radius against endpoints that are not, so a coarse sweep catches it —
see the tuning log entry on why this replaced a runtime hold/jump-cap instead.
"""
function blend_folds(tos, az0, el0, az1, el1)
    r0 = min(path_min_radius(az0, el0), path_min_radius(az1, el1))
    r0 <= 0 && return false   # degenerate endpoint; not this check's job
    any(w -> path_min_radius(blend_paths(az0, el0, az1, el1, w)...) <
             tos.blend_fold_margin * r0,
        range(0.0, 1.0; length = tos.blend_probe_points))
end
