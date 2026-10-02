# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The heading/course loop of [`FC_Settings`](@ref), section `course:` of the YAML
file: the entry state machine, the depower flown on the pattern, the heading PID and
its gain schedule, the heading/course feedback blend and the entry descent limiter.
[`CourseControllerSettings`](@ref) is built from it.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_Course @deftype Float64
    # ---- Entry state machine: park -> dive -> hold -> transition ---------------
    "Parking [s]: zero steering while init transients decay"
    park_time = 2.0
    "Dive course [deg]; beyond ±90 descends, < 0: rightmost entry"
    chi_dive = -85.0
    "Hold course [deg]: horizontal, so the kite arrives flat"
    chi_hold = -90.0
    "Dive ends this far above `el_center` [deg]"
    dive_el_margin = 7.0
    "Duration of the hold [s]"
    hold_time = 0.8
    "Cross-track error [deg] for phase 3 -> 4; log only"
    fig8_d_gate = 5.0
    "Factor on `heading_p` during the entry phases (dive and hold)"
    entry_gain = 0.25
    "Depower [-] held during the entry phases (dive and hold)"
    entry_depower = 0.34

    # ---- Depower on the pattern ------------------------------------------------
    "Run depower [-]; the turn-rate law's operating point"
    depower_setpoint = 0.26
    "Ramp time [s] to a new depower target; 0 = hard switch"
    depower_blend_time = 4.0

    # ---- Heading PID; output is rel_steering (-1..1), fed UNNEGATED ------------
    "Gain at `v_app_ref`; only `heading_p * v_app_ref` matters"
    heading_p = 0.1941
    "Integral time [s], or `false` for none"
    heading_i::Union{Bool, Float64} = false
    "Derivative time [s], damps the initial transient"
    heading_d = 0.12
    "Derivative filter N: `K*Td*s/(1 + s*Td/N)`"
    heading_d_n = 2.0
    "Phase-3 apparent wind [m/s]; anchors gain schedule, lead"
    v_app_ref = 27.0
    "Lower clamp on v_app, limits the gain boost [m/s]"
    v_app_min = 10.0
    "Extra `v_app` clamp [m/s] from phase 3 on; 0 = off"
    v_app_min_pattern = 0.0

    # ---- Feedback: heading when slow, course when fast, on |vel_kite| ----------
    "[m/s] at/below: pure heading feedback"
    v_kite_heading = 5.0
    "[m/s] at/above: pure course feedback; blended below"
    v_kite_course = 10.0
    "Course-only feedback from phase 3 on, ignoring `v_kite_*`"
    fig8_pure_course::Bool = false
    "Steering limit [-]; unstable above ~0.33 (loop), 0.375 (plant)"
    max_steering = 0.32

    # ---- Entry descent limiter, active only while far off the path -------------
    "Steepest off-path course [deg]; 90 = level, 180 = off"
    entry_chi_max = 95.0
    "Cross-track error [deg] below which the limiter is bypassed"
    entry_d_gate = 12.0
    "Blend band [deg] above `entry_d_gate`; 0 = hard switch"
    entry_d_blend = 4.0
    "Band around ±180° [deg] using the latched sign"
    entry_cut_margin = 30.0
end

"""
The steering feed-forward from the reference path's curvature, section
`feedforward:` of [`FC_Settings`](@ref)'s YAML file. Active from phase 4 on.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_FeedForward @deftype Float64
    "Gain on `u_ff = psi_dot_path / (c1 * v_app)`; 0 = off"
    ff_gain = 0.0
    "Feed-forward look-ahead [s] past Q; ~ steering dead time"
    ff_lead_time = 0.45
    "Arc [deg] the feed-forward averages the tangent over"
    ff_smooth = 3.0
    "Feed-forward low-pass [s], incl. chord term; 0 = none"
    ff_tau = 0.2
    "Cross-track error [deg] of full feed-forward fade-out"
    ff_d_fade = 6.0
    "Course error [deg] of full feed-forward fade-out"
    ff_err_fade = 60.0
end

"""
The pattern geometry and the attractor guidance, section `pattern:` of
[`FC_Settings`](@ref)'s YAML file, all angles in degrees; a SMALLER lemniscate is a
TIGHTER one. [`FigureEightController`](@ref) is built from it.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_Pattern @deftype Float64
    "Width of the eight [deg] (azimuth spans +-`f8_a`)"
    f8_a = 40.0
    "Height of the eight [deg] (elevation spans +-`f8_b`/2)"
    f8_b = 15.0
    "Centre elevation [deg]; lower: more margin, less energy"
    el_center = 26.0
    "Arc Q -> attractor [deg]; the floor of a timed lead"
    attractor_dist = 10.0
    "Attractor lead [s], 1-2 × `attractor_dist`; 0 = fixed"
    attractor_lead_time = 0.0
    "How much closer [deg] a global point must be for Q to jump"
    reacquire_margin = 3.0
    "Fly up-loops, not down-loops (reverses the path direction)"
    up_loops::Bool = false
end

"""
The wind schedule, section `wind_ramp:` of [`FC_Settings`](@ref)'s YAML
file: `course.depower_setpoint`, `pattern.f8_a` and `pattern.f8_b` are flown up to
`wind_ramp_low`, the `*_high` values from `wind_ramp_high` on, linear in between. See
[`wind_schedule`](@ref).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_WindRamp @deftype Float64
    "Wind speed [m/s] below which no `*_high` value is blended in"
    wind_ramp_low = 7.0
    "Wind [m/s] from which `*_high` apply; linear below"
    wind_ramp_high = 10.0
    "Depower [-] from `wind_ramp_high`; `NaN` keeps `depower_setpoint`"
    depower_high = NaN
    "Pattern width [deg] held from `wind_ramp_high` on; `NaN` keeps `f8_a`"
    f8_a_high = NaN
    "Pattern height [deg] held from `wind_ramp_high` on; `NaN` keeps `f8_b`"
    f8_b_high = NaN
end

"""
The winch settings of [`FC_Settings`](@ref), section `winch:` of its YAML
file: how compliant the force-mode winch is (see [`winch_force_gains`](@ref)) and the
force guards of the entry and the first lap. The winch gains themselves, of both
modes, and the reel-out law are in `wc_settings.yaml`.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_Winch @deftype Float64
    # ---- Force-mode winch ------------------------------------------------------
    "Winch softness [-]: divides the force-mode gains; 0 = POSITION mode"
    compliance = 0.5

    # ---- Force guards ----------------------------------------------------------
    "Entry-guard force floor [N], phases 0-2; not `WCSettings.f_low`"
    entry_f_min = 350.0
    "First-lap force limit / `WCSettings.f_high`; 1 = off"
    first_lap_force_frac = 1.0
end

"""
The reel-out run of [`FC_Settings`](@ref), section `reelout:` of its YAML
file: when reel-out starts and stops, and the depower and path flown once it has
stopped (phase 5). Only read by reel-out runs.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_Reelout @deftype Float64
    # ---- REEL_OUT winch; mutually exclusive with `compliance > 0` --------------
    "Tether length [m] at which reel-out stops and is held"
    reelout_l_max = 250.0
    "Figures of eight until reel-out stops (or `reelout_l_max`); 0 = off"
    n_fig_eight::Int64 = 0
    "Ramp-up time [s] of the reel-out speed; 0 = off"
    reelout_softstart = 0.0
    "Soft-stop lead [s]: `v_set` -> 0 at `reelout_l_max`; 0 = off"
    reelout_softstop = 0.0
    "Delay [s] from phase 3 to the reel-out start"
    reelout_delay = 0.0
    "Latching force [N] that starts reel-out early; `Inf` = off"
    reelout_f_trigger = Inf
    "Time [s] in phase 5 before the run ends; `Inf` = full run"
    final_time = Inf

    # ---- Phase-5 depower and force limiter -------------------------------------
    "Phase-5 depower [-]; tuned for 350 m tether, 6 m/s wind"
    depower_final = 0.328
    "Phase-5 force-limiter ceiling [-]; `depower_final` = off"
    depower_final_max = 0.328
    "Phase-5 limiter force [N]: criterion - lobe swing"
    depower_final_f_target = 7500.0
    "Integrator gain [1/(N s)], phase-5 force limiter"
    depower_final_f_gain = 2e-5
    "`depower_final_f_gain` during the soft stop"
    depower_final_f_gain_stop = 2e-5

    # ---- Phase-5 path lift -----------------------------------------------------
    "Path lift [deg] from the reel-out stop latch on"
    el_offset_final = 0.0
    "Lead [s] of the `el_offset_final` lift before reel-out ends"
    el_offset_lead = 0.0
    "Min. phase-5 curvature margin [-], else blend back; 0 = off"
    final_margin_min = 0.0
    "Lobe-only elevation lift [deg], ramped over azimuth; 0 = off"
    el_offset_wing = 0.0
    "Azimuth [deg] beyond which `el_offset_wing` is full"
    el_offset_wing_az = 10.0
    "Ramp width [deg] below `el_offset_wing_az`; gap 0-3"
    el_offset_wing_blend = 8.0
    "Wing-offset unit: `azimuth` [deg] or `azimuth_frac`"
    el_offset_wing_mode::String = "azimuth"
end

"""
The simulation conditions and the pass criteria of [`FC_Settings`](@ref),
section `run:` of its YAML file.

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct FC_Run @deftype Float64
    # ---- Simulation ------------------------------------------------------------
    "Steps between VSM aero updates"
    vsm_interval::Int64 = 1
    "Elevation [deg] init settles at; in the settled-state cache key"
    elevation = 73.0
    "Unlogged warm-up [s] inside `init` (V3Kite's `warmup!`); 0 = off"
    warmup_time = 2.0
    "Per-axis damping, [`turn_rate_coeffs`](@ref) key"
    body_damping::Vector{Float64} = [0.0, 0.0, 40.0]
    "Abort the run above this apparent wind speed [m/s]"
    v_app_abort = 45.0

    # ---- Metrics window and pass criteria --------------------------------------
    "Settle time [s] after `park_time` before statistics"
    entry_time = 52.0
    "Elevation floor criterion [deg], evaluated over the WHOLE run"
    min_elevation = 10.0
    "Min. size, fraction of `f8_a` (per side), `f8_b` (span)"
    min_span_frac = 0.7
end

# The field list is written out rather than `$(TYPEDFIELDS)`, so that each type links to
# its docstring; keep it in step with the field docstrings.
"""
Settings of the flight controller of a run, flown by `examples/simple_fig8.jl`,
`examples/simple_reelout.jl` and their variants, loaded from a YAML file such as
`fc_settings.yaml`. The settings are split by what reads them; each part is its own
struct and its own section of the YAML file.

A field is reached through its part, e.g. `fcs.pattern.f8_a`. The field names are
unique over all parts, so the constructors and [`apply_overrides!`](@ref) also take a
field by its bare name: `FC_Settings(; f8_a = 25.0)`, `FC_Settings(fcs; f8_a = 25.0)`
for a modified copy.

# Fields

- `course::`[`FC_Course`](@ref): Heading/course loop, entry state machine, depower on the pattern
- `feedforward::`[`FC_FeedForward`](@ref): Steering feed-forward from the path's curvature
- `pattern::`[`FC_Pattern`](@ref): Pattern geometry and attractor guidance
- `wind_ramp::`[`FC_WindRamp`](@ref): Wind schedule of depower and pattern size
- `winch::`[`FC_Winch`](@ref): Force-mode winch and force guards
- `reelout::`[`FC_Reelout`](@ref): Reel-out start and stop, phase-5 depower and path
- `run::`[`FC_Run`](@ref): Simulation conditions and pass criteria
"""
mutable struct FC_Settings
    "Heading/course loop, entry state machine, depower on the pattern"
    course::FC_Course
    "Steering feed-forward from the path's curvature"
    feedforward::FC_FeedForward
    "Pattern geometry and attractor guidance"
    pattern::FC_Pattern
    "Wind schedule of depower and pattern size"
    wind_ramp::FC_WindRamp
    "Force-mode winch and force guards"
    winch::FC_Winch
    "Reel-out start and stop, phase-5 depower and path"
    reelout::FC_Reelout
    "Simulation conditions and pass criteria"
    run::FC_Run
end
"""
The parts of [`FC_Settings`](@ref): field name => type, in the order of the struct and
of the sections of its YAML file.
"""
const FC_PARTS = (; course = FC_Course, feedforward = FC_FeedForward, pattern = FC_Pattern,
                  wind_ramp = FC_WindRamp, winch = FC_Winch, reelout = FC_Reelout,
                  run = FC_Run)

"""
Which part of [`FC_Settings`](@ref) holds a setting: setting name => part name, e.g.
`:f8_a => :pattern`. Built from [`FC_PARTS`](@ref); a name in two parts is an error at
load time, since the bare-name lookups of [`set_fc_field!`](@ref) rely on it.
"""
const FC_FIELD_PART = let parts = Dict{Symbol, Symbol}()
    for (part, T) in pairs(FC_PARTS), name in fieldnames(T)
        haskey(parts, name) &&
            error("FC_Settings: \"$name\" is in both $(parts[name]) and $part.")
        parts[name] = part
    end
    parts
end

"""
    FC_Settings(; kwargs...) -> FC_Settings
    FC_Settings(fcs::FC_Settings; kwargs...) -> FC_Settings

Default settings, or a copy of `fcs`, with `kwargs` set on top. A keyword is either a
part (`pattern = FC_Pattern(f8_a = 25.0)`) or a setting by its bare name
(`f8_a = 25.0`), see [`set_fc_field!`](@ref). `fcs` itself is left unchanged.
"""
function FC_Settings(; kwargs...)
    fcs = FC_Settings((T() for T in FC_PARTS)...)
    for (key, value) in kwargs
        set_fc_field!(fcs, key, value)
    end
    return fcs
end

function FC_Settings(fcs::FC_Settings; kwargs...)
    copy = deepcopy(fcs)
    for (key, value) in kwargs
        set_fc_field!(copy, key, value)
    end
    return copy
end

"""
    set_fc_field!(fcs::FC_Settings, key::Symbol, value) -> fcs

Set the part `key` of `fcs`, or the setting `key` in whichever part holds it
([`FC_FIELD_PART`](@ref)), converting `value` to the field's type. Errors for a `key`
that is neither.
"""
function set_fc_field!(fcs::FC_Settings, key::Symbol, value)
    if hasfield(FC_Settings, key)
        setfield!(fcs, key, value)
    else
        part = get(FC_FIELD_PART, key, nothing)
        isnothing(part) && error("\"$key\" is not a setting of FC_Settings.")
        obj = getfield(fcs, part)
        setfield!(obj, key, convert(fieldtype(typeof(obj), key), value))
    end
    return fcs
end

"""
    get_fc_field(fcs::FC_Settings, key::Symbol)

The setting `key` of `fcs` by its bare name, from whichever part holds it
([`FC_FIELD_PART`](@ref)); the counterpart of [`set_fc_field!`](@ref).
"""
function get_fc_field(fcs::FC_Settings, key::Symbol)
    part = get(FC_FIELD_PART, key, nothing)
    isnothing(part) && error("\"$key\" is not a setting of FC_Settings.")
    return getfield(getfield(fcs, part), key)
end

"""
    FC_Settings(filename::String; path=skc_data_path()) -> FC_Settings

Load flight-controller settings from the YAML file `filename` under `path`, which
defaults to this package's own [`skc_data_path`](@ref) rather than
`KiteUtils.get_data_path()` — the latter points at the *kite model's* data
directory during a run, and these settings belong to the controller. Pass `path`
explicitly to load a variant from elsewhere; an absolute `filename` is used
as-is.

The file must have a top-level `fc_settings:` mapping with one section per part of
`FC_Settings` (`course:`, `pattern:`, ...), each holding that part's settings; a
section or setting the file omits keeps its default, an unknown one is an error. A
setting directly under `fc_settings:` is accepted too: that is the layout of the
files archived before the split into parts. Such a file may still carry the keys
of [`MOVED_FC_KEYS`](@ref), which are skipped.
"""
function FC_Settings(filename::String; path = skc_data_path())
    file = isabspath(filename) ? filename : joinpath(path, filename)
    fcs = FC_Settings()
    for (key, value) in YAML.load_file(file)["fc_settings"]
        sym = Symbol(key)
        if hasfield(FC_Settings, sym)
            value isa AbstractDict ||
                error("Section \"$key\" in $filename must be a mapping of settings.")
            set_yaml_fields!(getfield(fcs, sym), value, filename)
        elseif haskey(FC_FIELD_PART, sym)
            set_fc_field!(fcs, sym, value)
        elseif key in RETIRED_YAML_KEYS
            iszero(value) ||
                error("Retired key \"$key\" in $filename must be 0, got $value.")
        elseif key in MOVED_FC_KEYS
            continue
        else
            error("Unknown key \"$key\" in $filename — neither a part nor a setting of FC_Settings.")
        end
    end
    return fcs
end

"""
The force-mode winch gains, which moved from `FC_Settings` to `wc_settings.yaml`
(`WCSettings`). Settings files archived before still carry them; [`FC_Settings`](@ref)
skips them when it loads such a file.
"""
const MOVED_FC_KEYS = ("winch_force_tau", "winch_len_kp", "winch_damp", "winch_force_min")

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
    return set_yaml_fields!(obj, dict, filename)
end

"""
    set_yaml_fields!(obj, dict, filename) -> obj

The body of [`load_yaml_fields!`](@ref): set each `key => value` of `dict` as a field
of `obj`, with the same conversion, retired-key and unknown-key rules. `filename`
only names the file in the errors.
"""
function set_yaml_fields!(obj, dict, filename)
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
type's name, both for the error of a key that is not a field of `obj`. For an
[`FC_Settings`](@ref), a key is a setting by its bare name, in whichever part holds it
([`set_fc_field!`](@ref)).
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

function apply_overrides!(fcs::FC_Settings, overrides, label, typename, what)
    for (key, value) in overrides
        hasfield(FC_Settings, Symbol(key)) || haskey(FC_FIELD_PART, Symbol(key)) ||
            error("$label: \"$key\" is not a setting of $typename.")
        set_fc_field!(fcs, Symbol(key), value)
    end
    isempty(overrides) ||
        @info "$what overrides in force: " * join(("$k = $v" for (k, v) in overrides), ", ")
    return fcs
end

"""
    FigureEightController(fcs::FC_Settings; dt, A = fcs.pattern.f8_a, B = fcs.pattern.f8_b)

The figure-eight controller of a run flown with `fcs`: the lemniscate `A` x `B` [deg]
centred at azimuth 0 and `fcs.pattern.el_center`, with `fcs`'s attractor distance, loop direction
and reacquire margin, stepped at `dt` [s]. `A`/`B` default to the settings' own size; a
sweep over the size passes its own.
"""
FigureEightController(fcs::FC_Settings; dt, A = fcs.pattern.f8_a, B = fcs.pattern.f8_b) =
    FigureEightController(FigureEightSettings(;
        dt, A, B, az_center = 0.0, el_center = fcs.pattern.el_center,
        attractor_distance = fcs.pattern.attractor_dist, up_loops = fcs.pattern.up_loops,
        reacquire_margin = fcs.pattern.reacquire_margin))

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
    course_loop_model_file(project = project_file()) -> String

Get the filename of the identified course-loop model ([`CourseLoopModel`](@ref)) from
the system project, the same way as [`fc_settings`](@ref). Returns the value of the
`course_loop_model` field of the project's `system` section; present in every project.
"""
function course_loop_model_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["course_loop_model"]
end

"""
    kite_correction_file(project = project_file()) -> String

Get the filename of the measured kite correction (written by
`examples/identify_kite_correction.jl`, read with [`load_course_correction`](@ref)) from the
system project, the same way as [`fc_settings`](@ref). Returns the value of the
`kite_correction` field of the project's `system` section; present in every project.
"""
function kite_correction_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["kite_correction"]
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
    winch_force_gains(fcs::FC_Settings, wcs) -> NamedTuple

The force-mode winch gains of the winch settings `wcs` (a `WCSettings`, loaded from
the project's `wc_settings.yaml`) with the `compliance` of `fcs` applied, as a
NamedTuple keyed to match a force-mode winch controller's fields (`force_tau`,
`len_kp`, `damp`, `force_min`). Splat it into whichever winch the kite model
provides:

    wfc = WinchForceController(; winch_force_gains(fcs, wcs)...)

`wcs.winch_len_kp` and `wcs.winch_damp` are both divided by `fcs.winch.compliance`, so the
yield scales linearly with it while their ratio — the length loop's own time
constant — is unchanged. `wcs.winch_force_tau` is passed through untouched: it sets
WHICH frequencies the drum yields to, not by how much.

Plain numbers on purpose. The scaling is the part worth keeping in this package;
the controller object it feeds belongs to the kite model, which this package
does not depend on.

Errors at `compliance == 0`: that is position mode and must not be flown through
a force-mode winch (an infinitely stiff spring is not representable — see
[`FC_Settings`](@ref)).
"""
function winch_force_gains(fcs::FC_Settings, wcs)
    fcs.winch.compliance > 0 ||
        error("winch_force_gains needs compliance > 0; at 0 use position mode.")
    return (force_tau = wcs.winch_force_tau,
            len_kp = wcs.winch_len_kp / fcs.winch.compliance,
            damp = wcs.winch_damp / fcs.winch.compliance,
            force_min = wcs.winch_force_min)
end

"""
    wind_schedule(fcs::FC_Settings, v_wind) -> (; depower_setpoint, f8_a, f8_b)

What to fly on the pattern at wind speed `v_wind` [m/s, reference height]: each of
`depower_setpoint`, `f8_a`, `f8_b` as set up to `fcs.wind_ramp.wind_ramp_low`, its `*_high`
counterpart from `fcs.wind_ramp.wind_ramp_high` on, linear in between; a `*_high` that is
`NaN` leaves its value alone. The depower is rounded to 0.01, the angles to 0.5°:
the settled-geometry cache is keyed on the depower, so this costs one settle per
step, not one per wind speed. Applied by [`apply_wind_schedule!`](@ref).
"""
function wind_schedule(fcs::FC_Settings, v_wind)
    frac = clamp((v_wind - fcs.wind_ramp.wind_ramp_low) / (fcs.wind_ramp.wind_ramp_high - fcs.wind_ramp.wind_ramp_low),
                 0.0, 1.0)
    ramp(lo, hi, step) = isnan(hi) ? Float64(lo) : round((lo + frac * (hi - lo)) / step) * step
    return (; depower_setpoint = round(ramp(fcs.course.depower_setpoint, fcs.wind_ramp.depower_high, 0.01); digits = 2),
            f8_a = ramp(fcs.pattern.f8_a, fcs.wind_ramp.f8_a_high, 0.5),
            f8_b = ramp(fcs.pattern.f8_b, fcs.wind_ramp.f8_b_high, 0.5))
end

"""
    apply_wind_schedule!(fcs::FC_Settings, v_wind) -> FC_Settings

Overwrite `fcs.course.depower_setpoint`, `fcs.pattern.f8_a` and `fcs.pattern.f8_b` with
[`wind_schedule`](@ref)`(fcs, v_wind)`. Call it once, before anything is built
from `fcs`: applied twice, the second call ramps from the first one's result.
"""
function apply_wind_schedule!(fcs::FC_Settings, v_wind)
    (; depower_setpoint, f8_a, f8_b) = wind_schedule(fcs, v_wind)
    fcs.course.depower_setpoint = depower_setpoint
    fcs.pattern.f8_a = f8_a
    fcs.pattern.f8_b = f8_b
    return fcs
end

"""
    attractor_distance(fcs::FC_Settings, v_app, l_tether) -> Float64

The attractor lead [deg] to fly at apparent wind `v_app` [m/s] and tether length
`l_tether` [m]: a constant `fcs.pattern.attractor_dist` while `fcs.pattern.attractor_lead_time`
is off, otherwise the arc that takes `attractor_lead_time` seconds to fly,
`v_app` floored at `fcs.course.v_app_min` and the result clamped to
`[attractor_dist, 2 * attractor_dist]`. Pure kinematics, no plant: the caller
writes it into `FigureEightSettings.attractor_distance` before each
`navigate_fig8`.
"""
function attractor_distance(fcs::FC_Settings, v_app, l_tether)
    fcs.pattern.attractor_lead_time > 0 || return fcs.pattern.attractor_dist
    lead = rad2deg(fcs.pattern.attractor_lead_time * max(v_app, fcs.course.v_app_min) / l_tether)
    return clamp(lead, fcs.pattern.attractor_dist, 2 * fcs.pattern.attractor_dist)
end

"""
    guidance_rate(fcs::FC_Settings, v_app, l_tether, v_kite) -> ω_g

Corner frequency `ω_g = v_kite/(l_tether·D)` [rad/s] of the attractor guidance, with
`D` the [`attractor_distance`](@ref) at `v_app` [m/s] and `l_tether` [m] in rad and
`v_kite` the kite's speed [m/s]: the corner of [`guidance_tf`](@ref).
"""
guidance_rate(fcs::FC_Settings, v_app, l_tether, v_kite) =
    v_kite / (l_tether * deg2rad(attractor_distance(fcs, v_app, l_tether)))
