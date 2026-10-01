# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Settings of the figure-of-eight FLIGHT CONTROLLER flown by
`examples/simple_fig8.jl`: the simulation conditions, the pattern geometry, the
entry state machine, the heading/course PID and the metrics window. Loaded from
a YAML file (`fc_settings.yaml`).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_Settings @deftype Float64
    "Steps between VSM aero updates"
    vsm_interval::Int64 = 1
    "How soft the winch is [-]: divides `winch_len_kp` and `winch_damp`; exactly 0 switches to POSITION mode"
    compliance = 0.5
    "Depower held during the run [-]; sets the operating point of the turn-rate law"
    depower_setpoint = 0.26
    "Depower [-] held at and above `wind_ramp_high`: `NaN` (the default) keeps `depower_setpoint`"
    depower_high = NaN
    "Pattern width [deg] held from `wind_ramp_high` on; `NaN` keeps `f8_a`"
    f8_a_high = NaN
    "Pattern height [deg] held from `wind_ramp_high` on; `NaN` keeps `f8_b`"
    f8_b_high = NaN
    "Wind speed [m/s] up to which `depower_setpoint`, `f8_a` and `f8_b` are flown unchanged"
    wind_ramp_low = 7.0
    "Wind speed [m/s] from which the `*_high` values are flown; linear in between"
    wind_ramp_high = 10.0
    "Settling elevation [deg]; currently has NO EFFECT, `settle_wing`'s cache key does not include it"
    elevation = 73.0
    "Parking phase [s]: zero steering while the init/settling transients decay"
    park_time = 2.0
    "Warm-up [s] run inside `init` and discarded (V3Kite's `warmup!`), so transients miss the log; 0 disables"
    warmup_time = 2.0

    # ---- Force-mode winch, read only when `compliance > 0` and before it scales these #
    "Low-pass time constant [s] from measured winch force to reference force; not scaled by `compliance`"
    winch_force_tau = 10.0
    "Length-trim gain, length error [m] -> reference force [N/m], before `compliance` scaling"
    winch_len_kp = 100.0
    "Drum damping, reel-out speed [m/s] -> reference force [N·s/m], before `compliance` scaling; required"
    winch_damp = 500.0
    "Floor on the reference force, keeps the tether taut [N]"
    winch_force_min = 100.0

    # ---- REEL_OUT winch; mutually exclusive with `compliance > 0` ----------- #
    "Tether length [m] at which reel-out stops and the run holds the final length"
    reelout_l_max = 250.0
    "Figures of eight [-] after which reel-out stops (besides `reelout_l_max`, first wins); 0 disables"
    n_fig_eight::Int64 = 0
    "Depower [-] flown once reel-out stops (phase 5); tuned for `reelout_l_max = 350` at 6 m/s wind"
    depower_final = 0.328
    "Ceiling [-] of the phase-5 force limiter on `rel_depower`; equal to `depower_final` (default) = off"
    depower_final_max = 0.328
    "Winch force [N] the phase-5 force limiter holds; set below the force criterion by the lobe-to-lobe swing"
    depower_final_f_target = 7500.0
    "Integrator gain [1/(N s)] of the phase-5 force limiter"
    depower_final_f_gain = 2e-5
    "Integrator gain [1/(N s)] of the phase-5 force limiter during the reel-out soft-stop ramp"
    depower_final_f_gain_stop = 2e-5
    "Soft-start time [s]: ramp the commanded reel-out speed up from 0 after reel-out engages; 0 disables"
    reelout_softstart = 0.0
    "Soft-stop trigger [s]: decelerate `v_set` linearly to 0 at `reelout_l_max` once this close; 0 disables"
    reelout_softstop = 0.0
    "Delay [s] between the guidance engaging (phase 3) and reel-out starting; 0 starts at phase 3"
    reelout_delay = 0.0
    "Force [N] that engages reel-out regardless of `reelout_delay` (latching); `Inf` disables"
    reelout_f_trigger = Inf
    "Time [s] flown in phase 5 before the run ends; `Inf` flies until the configured run length"
    final_time = Inf

    # ---- Entry state machine: park -> dive -> hold -> transition ----------- #
    "Course commanded during the dive [deg]; |chi| > 90 is descending, negative for the rightmost-point entry"
    chi_dive = -85.0
    "Course commanded during the hold [deg]: horizontal, so the kite arrives flat"
    chi_hold = -90.0
    "Margin above `el_center` [deg] at which the dive ends and the hold begins"
    dive_el_margin = 7.0
    "Duration of the hold [s]"
    hold_time = 0.8
    "Cross-track error [deg] below which phase 3 advances to phase 4 (a log milestone only)"
    fig8_d_gate = 5.0

    "Body damping settling starts from, per axis; the key [`turn_rate_coeffs`](@ref) is looked up with"
    body_damping::Vector{Float64} = [0.0, 0.0, 40.0]

    # ---- Pattern geometry [deg]; a SMALLER lemniscate is a TIGHTER one ------ #
    "Width of the eight [deg] (azimuth spans +-`f8_a`)"
    f8_a = 40.0
    "Height of the eight [deg] (elevation spans +-`f8_b`/2)"
    f8_b = 15.0
    "Pattern-centre elevation [deg]: lower improves the curvature margin but costs energy"
    el_center = 26.0
    "Arc distance Q -> attractor [deg]; the floor of the lead when `attractor_lead_time` is on"
    attractor_dist = 10.0
    "Attractor lead as a time [s], clamped to `[attractor_dist, 2 * attractor_dist]`; 0 = constant arc"
    attractor_lead_time = 0.0
    "How much closer [deg] the global nearest path point must be than the local one before Q jumps to it"
    reacquire_margin = 3.0
    "Fly up-loops instead of down-loops (reverses the traversal direction of the path)"
    up_loops::Bool = false
    "Extra elevation [deg] the path is lifted by from the reel-out stop latch on"
    el_offset_final = 0.0
    "Minimum phase-5 curvature margin [-]; a path below it is blended back to the last one meeting it; 0 = off"
    final_margin_min = 0.0
    "Time [s] before reel-out ends at which the `el_offset_final` lift starts; 0 starts it at the end"
    el_offset_lead = 0.0
    "Extra elevation [deg] added to the path's lobes only, ramped in over azimuth; 0 = off"
    el_offset_wing = 0.0
    "Azimuth beyond which `el_offset_wing` is applied in full [deg]"
    el_offset_wing_az = 10.0
    "Ramp width [deg] of `el_offset_wing` below `el_offset_wing_az`; keep their difference in 0 … 3 deg"
    el_offset_wing_blend = 8.0
    "Units of `el_offset_wing_az`/`_blend`: azimuth [deg] or azimuth_frac [fraction of path amplitude]"
    el_offset_wing_mode::String = "azimuth"

    # ---- Heading PID; output is rel_steering (-1..1), fed UNNEGATED --------- #
    "Heading gain at `v_app == v_app_ref` (phase 3); only `heading_p * v_app_ref` is physical"
    heading_p = 0.1941
    "Integral time [s], or `false` for no integral action (the default)"
    heading_i::Union{Bool, Float64} = false
    "Derivative time [s], damps the initial transient"
    heading_d = 0.12
    "Derivative filter: maximum gain N of the D path, `K*Td*s/(1 + s*Td/N)`"
    heading_d_n = 2.0
    "Apparent wind speed [m/s] flown in phase 3; anchors the 1/v_app gain schedule and attractor lead"
    v_app_ref = 27.0
    "Lower clamp on v_app, limits the gain boost [m/s]"
    v_app_min = 10.0
    "Lower clamp on v_app [m/s] in the gain schedule from phase 3 on, on top of `v_app_min`; 0 = off"
    v_app_min_pattern = 0.0
    "Factor on `heading_p` during the entry phases (dive and hold)"
    entry_gain = 0.25
    "Depower [-] held during the entry phases (dive and hold)"
    entry_depower = 0.34
    "Time [s] over which `rel_depower` ramps to a new phase target; 0 = hard switch"
    depower_blend_time = 4.0

    # ---- Steering feed-forward from the reference path's curvature ---------- #
    "Gain on the curvature feed-forward `u_ff = psi_dot_path / (c1 * v_app)`; 0 = off, 1 = the law's value"
    ff_gain = 0.0
    "Flight time [s] ahead of Q at which the feed-forward reads the course rate; ~ the steering dead time"
    ff_lead_time = 0.45
    "Arc [deg] over which the path's tangent change is averaged for the feed-forward"
    ff_smooth = 3.0
    "Low-pass time constant [s] on the feed-forward steering and its chord correction; 0 = none"
    ff_tau = 0.2
    "Cross-track error [deg] at which the feed-forward is fully faded out (fading starts at half)"
    ff_d_fade = 6.0
    "Course error [deg] at which the feed-forward is fully faded out (fading starts at half)"
    ff_err_fade = 60.0

    # ---- Feedback: heading when slow, course when fast, on |vel_kite| ------- #
    "[m/s] at/below: pure heading feedback"
    v_kite_heading = 5.0
    "[m/s] at/above: pure course feedback; linearly blended in between"
    v_kite_course = 10.0
    "From phase 3 on, feed back course alone, ignoring the `v_kite_*` schedule; `false` keeps the schedule"
    fig8_pure_course::Bool = false
    "Steering command limit [-]; above ~0.33 the loop, and above 0.375 the plant itself, goes unstable"
    max_steering = 0.32

    # ---- Entry descent limiter, active only while far off the path --------- #
    "Steepest commanded course while off-path [deg]; 90 = level, above 90 descending, 180 disables"
    entry_chi_max = 95.0
    "Cross-track error [deg] below which the limiter is bypassed"
    entry_d_gate = 12.0
    "Band [deg] above `entry_d_gate` over which limited and raw courses are blended; 0 = hard switch"
    entry_d_blend = 4.0
    "Distance from ±180° [deg] within which `chi_set`'s sign is replaced by the latched tangent sign"
    entry_cut_margin = 30.0

    "Force floor [N] of the entry guard (`guard_lfc`, phases 0-2); deliberately not `WCSettings.f_low`"
    entry_f_min = 350.0

    "Fraction of `WCSettings.f_high` used as upper force limit during the first lap; 1.0 = off"
    first_lap_force_frac = 1.0

    "Abort the run above this apparent wind speed [m/s]"
    v_app_abort = 45.0

    # ---- Metrics window ---------------------------------------------------- #
    "Settle time [s] after `park_time` before the tracking statistics start"
    entry_time = 52.0
    "Elevation floor criterion [deg], evaluated over the WHOLE run"
    min_elevation = 10.0
    "Minimum pattern size as fraction of `f8_a` (azimuth reach per side) and `f8_b` (elevation span)"
    min_span_frac = 0.7
end
"""
    FC_Settings(filename::String; path=skc_data_path()) -> FC_Settings

Load figure-eight flight-controller settings from the YAML file `filename` under
`path`, which defaults to this package's own [`skc_data_path`](@ref) rather than
`KiteUtils.get_data_path()` — the latter points at the *kite model's* data
directory during a run, and these settings belong to the controller. Pass `path`
explicitly to load a variant from elsewhere; an absolute `filename` is used
as-is.

The file must have a top-level `fc_settings:` mapping whose keys are the field
names of `FC_Settings`; any missing key falls back to the struct default, and an
unknown key is an error. Built on [`load_yaml_fields!`](@ref).
"""
function FC_Settings(filename::String; path = skc_data_path())
    load_yaml_fields!(FC_Settings(), filename, "fc_settings"; path)
end

"""
Keys that were removed from the settings structs together with the shape
parameters `C` and `D` of [`figure_eight_path`](@ref). Archived settings files
still carry them, always as `0`, which is the shape the path now always has.
"""
const RETIRED_YAML_KEYS = ("f8_c", "f8_d", "guess_c", "guess_d")

"""
    load_yaml_fields!(obj, filename, section; path = skc_data_path()) -> obj

Set every field `section` (a top-level key in the YAML file `filename`) names
on the mutable struct `obj`, converting each value to the field's declared
type; `filename` is resolved under `path` unless already absolute. An unknown
key errors, a key the file omits leaves `obj`'s existing value (its struct
default, for a freshly constructed `obj`) untouched. A key in
[`RETIRED_YAML_KEYS`](@ref) is skipped if it is `0` and errors otherwise, so
archived settings files still load.

Purely reflective (`hasfield`/`setfield!`/`fieldtype` on `typeof(obj)`), so it
works on any mutable struct without `src/` depending on the struct's package.
[`FC_Settings`](@ref) is built on it. It also loaded WinchControllers.jl's
`WCSettings` in `examples/simple_reelout.jl` until the 2026-08-16 winch merge
gave that struct a single file reached through the project's `wc_settings:` key
(PlanWinchcontrol.md), which is V3Kite's `WC_Settings(filename)`'s job now.
"""
function load_yaml_fields!(obj, filename::AbstractString, section::AbstractString;
                            path = skc_data_path())
    dict = YAML.load_file(isabspath(filename) ? filename :
                          joinpath(path, filename))[section]
    T = typeof(obj)
    for (key, value) in dict
        sym = Symbol(key)
        if !hasfield(T, sym) && key in RETIRED_YAML_KEYS
            iszero(value) ||
                error("Retired key \"$key\" in $filename must be 0, got $value.")
            continue
        end
        hasfield(T, sym) ||
            error("Unknown key \"$key\" in $filename — not a field of $T.")
        setfield!(obj, sym, convert(fieldtype(T, sym), value))
    end
    return obj
end

"""
    apply_overrides!(obj, overrides, label, typename, what)

Set each `key => value` of `overrides` as a field of the settings struct `obj`,
converted to the field's type, and log the ones in force as "`what` overrides in
force". `label` is the name of the input that carried them and `typename` the
type's name, both for the error of a key that is not a field of `obj`.
"""
function apply_overrides!(obj, overrides, label, typename, what)
    for (key, value) in overrides
        hasfield(typeof(obj), key) ||
            error("$label: \"$key\" is not a field of $typename.")
        setfield!(obj, key, convert(fieldtype(typeof(obj), key), value))
    end
    isempty(overrides) ||
        @info "$what overrides in force: " * join(("$k = $v" for (k, v) in overrides), ", ")
    return obj
end

"""
    FigureEightController(fcs::FC_Settings; dt, A = fcs.f8_a, B = fcs.f8_b)

The figure-eight controller of a run flown with `fcs`: the lemniscate `A` x `B` [deg]
centred at azimuth 0 and `fcs.el_center`, with `fcs`'s attractor distance, loop direction
and reacquire margin, stepped at `dt` [s]. `A`/`B` default to the settings' own size; a
sweep over the size passes its own.
"""
FigureEightController(fcs::FC_Settings; dt, A = fcs.f8_a, B = fcs.f8_b) =
    FigureEightController(FigureEightSettings(;
        dt, A, B, az_center = 0.0, el_center = fcs.el_center,
        attractor_distance = fcs.attractor_dist, up_loops = fcs.up_loops,
        reacquire_margin = fcs.reacquire_margin))

"""
    project_file(project = "system_fig8_200m.yaml") -> String

Path of the system project to hand to the kite model, absolute when this package
carries it in [`skc_data_path`](@ref) and unchanged otherwise, which leaves it a
lookup under the active data path.

The absolute form is what makes `sim_settings` this package's file: KiteUtils
resolves it relative to the project. Bare names like `wc_settings` instead follow
the active data path, which `examples/simple_fig8.jl` also points here.

Not a field of [`FC_Settings`](@ref): which plant a run is flown against is not a
tuning parameter of the controller.
"""
function project_file(project::String = "system_fig8_200m.yaml")
    path = joinpath(skc_data_path(), project)
    return isfile(path) ? path : project
end

"""
    fc_settings(project = project_file()) -> String

Get the flight-controller (FC) settings filename from the system project,
analogous to `KiteUtils.wc_settings`. Returns the value of the `fc_settings`
field of the project's `system` section; `project` defaults to this package's
own [`project_file`](@ref) rather than `KiteUtils.PROJECT`.
"""
function fc_settings(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["fc_settings"]
end

"""
    turn_rate_coeffs_file(project = project_file()) -> String

Get the turn-rate table filename from the system project, the same way as
[`fc_settings`](@ref). Returns the value of the `turn_rate_coeffs` field of
the project's `system` section; present in every project.
"""
function turn_rate_coeffs_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["turn_rate_coeffs"]
end

"""
    winch_table_file(project = project_file()) -> String

Get the winch table filename from the system project, the same way as
[`fc_settings`](@ref). Returns the value of the `winch_table` field of the
project's `system` section; only reel-out projects carry it.
"""
function winch_table_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["winch_table"]
end

"""
    traj_opt_settings_file(project = project_file()) -> String

Get the trajectory-optimizer settings filename from the system project, the
same way as [`fc_settings`](@ref). Returns the value of the
`traj_opt_settings` field of the project's `system` section; present in every
project that flies against an externally optimized path (both fig8 and
reel-out).
"""
function traj_opt_settings_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["traj_opt_settings"]
end

"""
    winch_force_gains(fcs::FC_Settings) -> NamedTuple

Force-mode winch gains with the `compliance` scaling applied, as a NamedTuple
keyed to match a force-mode winch controller's fields
(`force_tau`, `len_kp`, `damp`, `force_min`). Splat it into whichever winch the
kite model provides:

    wfc = WinchForceController(; winch_force_gains(fcs)...)

`winch_len_kp` and `winch_damp` are both divided by `fcs.compliance`, so the
yield scales linearly with it while their ratio — the length loop's own time
constant — is unchanged. `winch_force_tau` is passed through untouched: it sets
WHICH frequencies the drum yields to, not by how much.

Plain numbers on purpose. The scaling is the part worth keeping in this package;
the controller object it feeds belongs to the kite model, which this package
does not depend on.

Errors at `compliance == 0`: that is position mode and must not be flown through
a force-mode winch (an infinitely stiff spring is not representable — see
[`FC_Settings`](@ref)).
"""
function winch_force_gains(fcs::FC_Settings)
    fcs.compliance > 0 ||
        error("winch_force_gains needs compliance > 0; at 0 use position mode.")
    return (force_tau = fcs.winch_force_tau,
            len_kp = fcs.winch_len_kp / fcs.compliance,
            damp = fcs.winch_damp / fcs.compliance,
            force_min = fcs.winch_force_min)
end

"""
    wind_schedule(fcs::FC_Settings, v_wind) -> (; depower_setpoint, f8_a, f8_b)

What to fly on the pattern at wind speed `v_wind` [m/s, reference height]: each of
`depower_setpoint`, `f8_a`, `f8_b` as set up to `fcs.wind_ramp_low`, its `*_high`
counterpart from `fcs.wind_ramp_high` on, linear in between; a `*_high` that is
`NaN` leaves its value alone. The depower is rounded to 0.01, the angles to 0.5°:
the settled-geometry cache is keyed on the depower, so this costs one settle per
step, not one per wind speed. Applied by [`apply_wind_schedule!`](@ref).
"""
function wind_schedule(fcs::FC_Settings, v_wind)
    frac = clamp((v_wind - fcs.wind_ramp_low) / (fcs.wind_ramp_high - fcs.wind_ramp_low),
                 0.0, 1.0)
    ramp(lo, hi, step) = isnan(hi) ? Float64(lo) : round((lo + frac * (hi - lo)) / step) * step
    return (; depower_setpoint = round(ramp(fcs.depower_setpoint, fcs.depower_high, 0.01); digits = 2),
            f8_a = ramp(fcs.f8_a, fcs.f8_a_high, 0.5),
            f8_b = ramp(fcs.f8_b, fcs.f8_b_high, 0.5))
end

"""
    apply_wind_schedule!(fcs::FC_Settings, v_wind) -> FC_Settings

Overwrite `fcs.depower_setpoint`, `fcs.f8_a` and `fcs.f8_b` with
[`wind_schedule`](@ref)`(fcs, v_wind)`. Call it once, before anything is built
from `fcs`: applied twice, the second call ramps from the first one's result.
"""
function apply_wind_schedule!(fcs::FC_Settings, v_wind)
    (; depower_setpoint, f8_a, f8_b) = wind_schedule(fcs, v_wind)
    fcs.depower_setpoint = depower_setpoint
    fcs.f8_a = f8_a
    fcs.f8_b = f8_b
    return fcs
end

"""
    attractor_distance(fcs::FC_Settings, v_app, l_tether) -> Float64

The attractor lead [deg] to fly at apparent wind `v_app` [m/s] and tether length
`l_tether` [m]: a constant `fcs.attractor_dist` while `fcs.attractor_lead_time`
is off, otherwise the arc that takes `attractor_lead_time` seconds to fly,
`v_app` floored at `fcs.v_app_min` and the result clamped to
`[attractor_dist, 2 * attractor_dist]`. Pure kinematics, no plant: the caller
writes it into `FigureEightSettings.attractor_distance` before each
`navigate_fig8`.
"""
function attractor_distance(fcs::FC_Settings, v_app, l_tether)
    fcs.attractor_lead_time > 0 || return fcs.attractor_dist
    lead = rad2deg(fcs.attractor_lead_time * max(v_app, fcs.v_app_min) / l_tether)
    return clamp(lead, fcs.attractor_dist, 2 * fcs.attractor_dist)
end

"""
    guidance_rate(fcs::FC_Settings, v_app, l_tether, v_kite) -> ω_g

Corner frequency `ω_g = v_kite/(l_tether·D)` [rad/s] of the attractor guidance, with
`D` the [`attractor_distance`](@ref) at `v_app` [m/s] and `l_tether` [m] in rad and
`v_kite` the kite's speed [m/s]: the corner of [`guidance_tf`](@ref).
"""
guidance_rate(fcs::FC_Settings, v_app, l_tether, v_kite) =
    v_kite / (l_tether * deg2rad(attractor_distance(fcs, v_app, l_tether)))
