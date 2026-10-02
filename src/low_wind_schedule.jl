# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    low_wind_schedule(fcs::FC_Settings, v_ref) -> Union{NamedTuple, Nothing}

What to fly at the wind speed `v_ref` [m/s] at `fcs.low_wind.low_wind_height`:
`(; l_tether, guess_el_center, v_app_min, el_offset_final)`, linear between the
rows of `fcs.low_wind` ([`FC_LowWind`](@ref)) and the first row's values below it.
`nothing` at and above the last row's wind speed, and for an empty schedule: the
settings files' own values are flown there.

Keyed on the wind at height and not at `h_ref`, because that orders the sites by what
the kite meets: Cabauw's 3 m/s is 5.8 m/s at 100 m, Maasvlakte's 3.5 m/s only 4.5 m/s
(2026-10-02). Errors for vectors of unequal length or wind speeds that do not ascend.
"""
function low_wind_schedule(fcs::FC_Settings, v_ref)
    lw = fcs.low_wind
    speeds = lw.low_wind_speeds
    columns = (; l_tether = lw.low_wind_l_tether, guess_el_center = lw.low_wind_guess_el_center,
               v_app_min = lw.low_wind_v_app_min, el_offset_final = lw.low_wind_el_offset_final)
    all(c -> length(c) == length(speeds), columns) ||
        error("low_wind: every column needs one value per wind speed, $(length(speeds)) of them.")
    all(>(0), diff(speeds)) || error("low_wind: low_wind_speeds must ascend, got $speeds.")
    (isempty(speeds) || v_ref >= speeds[end]) && return nothing
    i = searchsortedlast(speeds, v_ref)
    i == 0 && return map(first, columns)
    frac = (v_ref - speeds[i]) / (speeds[i + 1] - speeds[i])
    return map(c -> c[i] + frac * (c[i + 1] - c[i]), columns)
end

"""
    low_wind_reference(fcs::FC_Settings, project_set) -> Float64

The wind speed [m/s] [`low_wind_schedule`](@ref) is keyed on: the project's `v_wind`
(at `h_ref`, after any wind-speed override) scaled to `fcs.low_wind.low_wind_height`
by the project's own profile law.
"""
low_wind_reference(fcs::FC_Settings, project_set) =
    calc_wind_factor(AtmosphericModel(project_set; nowindfield = true), fcs.low_wind.low_wind_height) *
    project_set.v_wind

"""
    apply_low_wind_schedule!(fcs, tos, project_set; keep = ()) -> NamedTuple

Overwrite the starting tether length `project_set.l_tether`, `tos.guess_el_center`,
`fcs.course.v_app_min` and `fcs.reelout.el_offset_final` with
[`low_wind_schedule`](@ref) at [`low_wind_reference`](@ref). A name in `keep` (e.g. the
keys of a run's overrides) is left alone; `tos = nothing` skips the guess, for a caller
without the optimizer's settings. Call it once, after the wind-speed override and
before anything is built from the settings. Returns `(; v_ref, values)`, `values`
being what was applied, or `nothing` when the schedule is off at this wind.

Warns when the last row differs from the settings files' own values: the schedule then
steps at its last wind speed.
"""
function apply_low_wind_schedule!(fcs::FC_Settings, tos, project_set; keep = ())
    lw = fcs.low_wind
    if !isempty(lw.low_wind_speeds)
        base = (; l_tether = project_set.l_tether,
                guess_el_center = isnothing(tos) ? lw.low_wind_guess_el_center[end] : tos.guess_el_center,
                v_app_min = fcs.course.v_app_min, el_offset_final = fcs.reelout.el_offset_final)
        last_row = map(last, (; l_tether = lw.low_wind_l_tether, guess_el_center = lw.low_wind_guess_el_center,
                              v_app_min = lw.low_wind_v_app_min, el_offset_final = lw.low_wind_el_offset_final))
        last_row == base || @warn "low_wind: the last row $last_row is not the settings files' own \
                                   $base; the schedule steps at $(lw.low_wind_speeds[end]) m/s."
    end
    v_ref = low_wind_reference(fcs, project_set)
    values = low_wind_schedule(fcs, v_ref)
    isnothing(values) && return (; v_ref, values)
    keep = Symbol.(collect(keep))
    :l_tether in keep || :l_tethers in keep || (project_set.l_tether = values.l_tether)
    isnothing(tos) || :guess_el_center in keep || (tos.guess_el_center = values.guess_el_center)
    :v_app_min in keep || (fcs.course.v_app_min = values.v_app_min)
    :el_offset_final in keep || (fcs.reelout.el_offset_final = values.el_offset_final)
    @info @sprintf("Low-wind schedule at %.2f m/s (%.0f m): l_tether = %.1f m, guess_el_center = %.2f°, \
                    v_app_min = %.2f m/s, el_offset_final = %.2f°%s.", v_ref, lw.low_wind_height,
                   values.l_tether, values.guess_el_center, values.v_app_min, values.el_offset_final,
                   isempty(keep) ? "" : ", overrides kept: " * join(keep, ", "))
    return (; v_ref, values)
end
