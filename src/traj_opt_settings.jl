# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The AWETrim server, the request caches and the winch law sent, part `server` of
[`TrajOptSettings`](@ref), section `server:` of the YAML file.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptServer
    "Address of the AWETrim server"
    base_url::String = "http://127.0.0.1:8000"
    "Skip optimizer requests recorded as failed before (`OPT_FAILURE_CACHE`)"
    opt_failure_cache::Bool = true
    "Replay previously applied optimizer results (`OPT_CHAIN_CACHE`)"
    opt_success_cache::Bool = true
    "`use_awe_trim` of a warm-up solve sent before the startup request; `0.0` = off"
    opt_warm_start_awe_trim::Float64 = 0.0
    "`use_awe_trim` sent to AWETrim; negative follows `wc.use_awe_trim`"
    opt_awe_trim::Float64 = -1.0
    "Lengths of the post-run `free_speed` reference power solve; `0` = off"
    free_speed_reference_points::Int64 = 0
end

"""
The initial guess the startup solve starts from, part `guess` of
[`TrajOptSettings`](@ref), section `guess:` of the YAML file.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptGuess @deftype Float64
    "Width of the guess lemniscate; azimuth spans ±`guess_a` [deg]"
    guess_a = 30.0
    "Guess height; elevation spans `guess_b` peak to peak [deg]"
    guess_b = 12.0
    "Centre elevation of the guess [deg]"
    guess_el_center = 26.0
    "Guess centre elevation at and above `guess_el_center_wind_ref` [deg]; `0.0` = off"
    guess_el_center_high = 0.0
    "Wind speed at and above which `guess_el_center_high` is used [m/s]"
    guess_el_center_wind_ref = 0.0
    "Guess elevation offsets tried in order when the startup solve fails [deg]"
    startup_retry_el_offsets::Vector{Float64} = Float64[]
end

"""
The depower seed of the solve, ramped with the wind, part `seed` of
[`TrajOptSettings`](@ref), section `seed:` of the YAML file.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptSeed @deftype Float64
    "Power-tape length seed `l_dp` on AWETrim's scale [m]"
    input_depower = 1.6
    "Wind speed at which `input_depower` is the seed [m/s]"
    input_depower_wind_ref = 7.0
    "Tape length added to the seed per m/s above `input_depower_wind_ref` [m/(m/s)]"
    input_depower_per_wind = 0.0
    "Soft ceiling on the ramped depower seed [m]; `0.0` = AWETrim's hard bound only"
    input_depower_seed_max = 0.0
end

"""
The pattern box the optimizer solves UNDER, sent with every request so a reply cannot
break it, part `box` of [`TrajOptSettings`](@ref), section `box:` of the YAML file.
Each bound is off at `0.0`.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptBox @deftype Float64
    "Azimuth half-width limit of the optimized pattern [deg]; `0.0` = optimizer's 45.8°"
    pattern_azimuth_max = 0.0
    "Azimuth half-width limit at and above the high-wind step [deg]; `0.0` = off"
    pattern_azimuth_max_high = 0.0
    "Largest (RMS) elevation half-span of the optimized figure [deg]; `0.0` = off"
    pattern_elevation_amplitude_max = 0.0
    "Elevation half-span cap at and above the high-wind step [deg]; `0.0` = off"
    pattern_elevation_amplitude_max_high = 0.0
    "Wind speed aloft at and above which the high-wind caps apply [m/s]"
    pattern_elevation_amplitude_max_wind_ref = 0.0
    "Height the high-wind step's wind speed is measured at [m]; `0.0` = ground wind"
    pattern_elevation_amplitude_max_wind_height = 0.0
    "Force a mirror-symmetric figure-eight"
    pattern_symmetric::Bool = false
    "Steepest climb angle of the optimized path [deg]; `0.0` = off"
    pattern_climb_angle_max = 0.0
end

"""
The gates every returned path must pass before it is flown, at startup and after a
re-optimization, part `gates` of [`TrajOptSettings`](@ref), section `gates:` of the YAML
file. `min_feasibility_margin` is also sent with the request, as a minimum turn radius.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptGates @deftype Float64
    "Minimum curvature margin of the returned path, also sent as turn radius; `0.0` = off"
    min_feasibility_margin = 1.0
    "Factor on the turn radius requested from the optimizer, on top of the gate's [-]"
    turn_radius_headroom = 1.0
    "Ground clearance the returned path must have [m]; `0.0` = off"
    min_height = 50.0
    "Margin above `min_elevation` a path's lowest point must have [deg]"
    candidate_elevation_margin = 3.0
end

"""
Re-optimization while the tether grows (`simple_opt_reelout.jl`), part `reopt` of
[`TrajOptSettings`](@ref), section `reopt:` of the YAML file.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptReopt @deftype Float64
    "Laps between re-optimizations"
    reopt_every_n_laps::Int64 = 2
    "Max re-optimizations per run; bounds wall time"
    max_reopt::Int64 = 4
    "Halt the simulation while a re-optimization runs"
    reopt_blocking::Bool = true
    "Time over which a new path is blended into the old one [s]"
    path_blend_time = 4.0
    "Fresh solves allowed when a reply is rejected by the blend or power gates"
    blend_max_retries::Int64 = 3
    "Box sent with re-optimizations, as factor on the previous install's size; `0` = off"
    size_box_growth = 1.3
    "Growth above which a power-losing reply is challenged by a cold solve; `0` = off"
    challenge_growth = 1.1
end

"""
The extra gates of a re-optimization reply, part `reopt_gates` of
[`TrajOptSettings`](@ref), section `reopt_gates:` of the YAML file.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptReoptGates @deftype Float64
    "Min fraction of the startup install's predicted power a reply must reach [-]"
    min_power_frac = 0.3
    "Min fraction of the previous install's predicted power a reply must reach [-]"
    min_power_frac_prev = 0.85
    "Wind below which negative power predictions bypass the power gates [m/s]"
    power_gate_wind_min = 4.0
    "Max size growth of a reply relative to the previous install [-]; `0.0` = off"
    max_size_growth = 1.3
    "Fraction of the endpoints' min radius every blended path must clear [-]"
    blend_fold_margin = 0.5
end

# The field list is written out rather than `$(TYPEDFIELDS)`, so that each type links to
# its docstring; keep it in step with the field docstrings.
"""
    TrajOptSettings

Settings of a run flown along an EXTERNALLY optimized path
(`examples/simple_opt_fig8.jl`, `examples/simple_opt_reelout.jl`): where the AWETrim
server is, the initial guess the solve starts from, the box it solves under, and the
gates the path that comes back must pass. Loaded from `data/traj_opt.yaml` the same way
[`FC_Settings`](@ref) is loaded from its own — a run is defined by a file, not by editing
a script. The settings are split by what reads them; each part is its own struct and its
own section of the YAML file.

A field is reached through its part, e.g. `tos.box.pattern_azimuth_max`. The field names
are unique over all parts, so the constructors and [`apply_overrides!`](@ref) also take a
field by its bare name: `TrajOptSettings(; min_height = 40.0)`.

The conditions are NOT here: the wind comes from the system project's settings
file and the winch law from its `wc_settings`, both read off the same files the
plant is built from (`inflow_from_settings`, `winch_from_wc` in
`awetrim_client.jl`). A path optimized for a wind the kite does not fly
in is not the path for the run.

The **initial guess is not a formality**, which is why it has its own fields here
rather than borrowing `FC_Settings`' `f8_a`/`f8_b`/`el_center`. Those size the
lemniscate that `simple_fig8.jl` and `simple_reelout.jl` actually FLY; here the
lemniscate is only a seed, and the two roles pull in different directions.
Measured 2026-08-18 at 150 m and 6 m/s: the reel-out pattern (20°/11° at 18°)
makes the solve fail to converge, while 30°/12°, or the same eight centred at
26°, converge — and to the same optimum, worth 6080 W, while the server's own
parametric guess converges to a different one worth 1431 W. The problem is
multi-modal, so the guess is a choice about the answer.

YAML: [`traj_opt.yaml`](@ref example_traj_opt).

# Fields

- `server::`[`TrajOptServer`](@ref): AWETrim server, request caches and the winch law sent
- `guess::`[`TrajOptGuess`](@ref): The initial guess the startup solve starts from
- `seed::`[`TrajOptSeed`](@ref): The depower seed of the solve
- `box::`[`TrajOptBox`](@ref): The pattern box the optimizer solves under
- `gates::`[`TrajOptGates`](@ref): The gates every returned path must pass
- `reopt::`[`TrajOptReopt`](@ref): Re-optimization while the tether grows
- `reopt_gates::`[`TrajOptReoptGates`](@ref): The extra gates of a re-optimization reply
"""
mutable struct TrajOptSettings
    "AWETrim server, request caches and the winch law sent"
    server::TrajOptServer
    "The initial guess the startup solve starts from"
    guess::TrajOptGuess
    "The depower seed of the solve"
    seed::TrajOptSeed
    "The pattern box the optimizer solves under"
    box::TrajOptBox
    "The gates every returned path must pass"
    gates::TrajOptGates
    "Re-optimization while the tether grows"
    reopt::TrajOptReopt
    "The extra gates of a re-optimization reply"
    reopt_gates::TrajOptReoptGates
end

"""
The parts of [`TrajOptSettings`](@ref): field name => type, in the order of the struct
and of the sections of its YAML file.
"""
const TO_PARTS = (; server = TrajOptServer, guess = TrajOptGuess, seed = TrajOptSeed,
                  box = TrajOptBox, gates = TrajOptGates, reopt = TrajOptReopt,
                  reopt_gates = TrajOptReoptGates)

"""
Which part of [`TrajOptSettings`](@ref) holds a setting: setting name => part name, e.g.
`:min_height => :gates`. Built from [`TO_PARTS`](@ref); a name in two parts is an error
at load time, since the bare-name lookups of [`set_tos_field!`](@ref) rely on it.
"""
const TO_FIELD_PART = let parts = Dict{Symbol, Symbol}()
    for (part, T) in pairs(TO_PARTS), name in fieldnames(T)
        haskey(parts, name) &&
            error("TrajOptSettings: \"$name\" is in both $(parts[name]) and $part.")
        parts[name] = part
    end
    parts
end

"""
    TrajOptSettings(; kwargs...) -> TrajOptSettings
    TrajOptSettings(tos::TrajOptSettings; kwargs...) -> TrajOptSettings

Default settings, or a copy of `tos`, with `kwargs` set on top. A keyword is either a
part (`box = TrajOptBox(pattern_azimuth_max = 28.0)`) or a setting by its bare name
(`pattern_azimuth_max = 28.0`), see [`set_tos_field!`](@ref). `tos` itself is left
unchanged.
"""
function TrajOptSettings(; kwargs...)
    tos = TrajOptSettings((T() for T in TO_PARTS)...)
    for (key, value) in kwargs
        set_tos_field!(tos, key, value)
    end
    return tos
end

function TrajOptSettings(tos::TrajOptSettings; kwargs...)
    copy = deepcopy(tos)
    for (key, value) in kwargs
        set_tos_field!(copy, key, value)
    end
    return copy
end

"""
    set_tos_field!(tos::TrajOptSettings, key::Symbol, value) -> tos

Set the part `key` of `tos`, or the setting `key` in whichever part holds it
([`TO_FIELD_PART`](@ref)), converting `value` to the field's type. Errors for a `key`
that is neither.
"""
function set_tos_field!(tos::TrajOptSettings, key::Symbol, value)
    if hasfield(TrajOptSettings, key)
        setfield!(tos, key, value)
    else
        part = get(TO_FIELD_PART, key, nothing)
        isnothing(part) && error("\"$key\" is not a setting of TrajOptSettings.")
        obj = getfield(tos, part)
        setfield!(obj, key, convert(fieldtype(typeof(obj), key), value))
    end
    return tos
end

"""
    get_tos_field(tos::TrajOptSettings, key::Symbol)

The setting `key` of `tos` by its bare name, from whichever part holds it
([`TO_FIELD_PART`](@ref)); the counterpart of [`set_tos_field!`](@ref).
"""
function get_tos_field(tos::TrajOptSettings, key::Symbol)
    part = get(TO_FIELD_PART, key, nothing)
    isnothing(part) && error("\"$key\" is not a setting of TrajOptSettings.")
    return getfield(getfield(tos, part), key)
end

function apply_overrides!(tos::TrajOptSettings, overrides, label, typename, what)
    for (key, value) in overrides
        hasfield(TrajOptSettings, Symbol(key)) || haskey(TO_FIELD_PART, Symbol(key)) ||
            error("$label: \"$key\" is not a setting of $typename.")
        set_tos_field!(tos, Symbol(key), value)
    end
    isempty(overrides) ||
        @info "$what overrides in force: " * join(("$k = $v" for (k, v) in overrides), ", ")
    return tos
end

# ---- Fixed parameters of the optimizer client ---------------------------- #
# Former `TrajOptSettings` fields that no run ever changed. Settings files that
# still carry them load only at these values, see `RETIRED_YAML_KEYS`.

"Name the optimization is registered under on the server"
const OPT_NAME = "simple_opt_fig8"
"Points the guess is sent with; also the resolution of the reply"
const GUESS_POINTS = 361
"Upper bound on the points the optimized path is resampled to"
const RESAMPLE_POINTS = 361
"Regularization weight of the solve [-]"
const REG_WEIGHT = 1.0
"Interval between `/status` polls while a solve runs [s]"
const REOPT_POLL_INTERVAL = 0.5
"The tether length sent to the optimizer is rounded to a multiple of this [m]"
const OPT_LENGTH_ROUND = 1.0
"Points at which a prospective blend is sampled for folds"
const BLEND_PROBE_POINTS = 21
"Corrected startup re-solves allowed when the first reply's margin is too small"
const STARTUP_RETRIES_MAX = 4
"First startup retry target, as a multiple of the installed path's margin [-]"
const STARTUP_RETRY_STEP = 1.05
"Floor on a retry's target margin, as a factor on `min_feasibility_margin` [-]"
const STARTUP_RETRY_SLACK = 1.03
"Elevation cap below the incumbent's top for each startup retry [deg]"
const STARTUP_RETRY_EL_CAP_STEP = 2.0
"Extra azimuth half-width a startup width retry asks for [deg]"
const STARTUP_RETRY_AZ_WIDEN_STEP = 2.0
"Guess elevation offset for one retry of a failed re-optimization [deg]"
const REOPT_RETRY_EL_OFFSET = 2.0
"Extra elevation on top of the shortfall when re-asking a rejected reply [deg]"
const ELEVATION_MIN_RETRY_MARGIN = 0.5

merge!(RETIRED_YAML_KEYS, Dict{String, Any}(
    "name" => OPT_NAME, "guess_points" => GUESS_POINTS, "resample_points" => RESAMPLE_POINTS,
    "reg_weight" => REG_WEIGHT, "reopt_poll_interval" => REOPT_POLL_INTERVAL,
    "opt_length_round" => OPT_LENGTH_ROUND, "blend_probe_points" => BLEND_PROBE_POINTS,
    "startup_retries_max" => STARTUP_RETRIES_MAX, "startup_retry_step" => STARTUP_RETRY_STEP,
    "startup_retry_slack" => STARTUP_RETRY_SLACK,
    "startup_retry_el_cap_step" => STARTUP_RETRY_EL_CAP_STEP,
    "startup_retry_az_widen_step" => STARTUP_RETRY_AZ_WIDEN_STEP,
    "reopt_retry_el_offset" => REOPT_RETRY_EL_OFFSET,
    "elevation_min_retry_margin" => ELEVATION_MIN_RETRY_MARGIN))

"""
    TrajOptSettings(filename::String; path = skc_data_path())

Load the settings from the `traj_opt:` section of `filename`, with one section per part
of `TrajOptSettings` (`server:`, `guess:`, ...), each holding that part's settings; a
section or setting the file omits keeps its default, an unknown one is an error. A
setting directly under `traj_opt:` is accepted too: that is the layout of the files
archived before the split into parts. Such a file may still carry the keys of
[`RETIRED_YAML_KEYS`](@ref), at the value listed there.
"""
function TrajOptSettings(filename::String; path = skc_data_path())
    file = isabspath(filename) ? filename : joinpath(path, filename)
    tos = TrajOptSettings()
    for (key, value) in YAML.load_file(file)["traj_opt"]
        sym = Symbol(key)
        if hasfield(TrajOptSettings, sym)
            value isa AbstractDict ||
                error("Section \"$key\" in $filename must be a mapping of settings.")
            set_yaml_fields!(getfield(tos, sym), value, filename)
        elseif haskey(TO_FIELD_PART, sym)
            set_tos_field!(tos, sym, value)
        elseif haskey(RETIRED_YAML_KEYS, key)
            retired_ok(key, value) ||
                error("Retired key \"$key\" in $filename must be \
                       $(repr(RETIRED_YAML_KEYS[key])), got $value.")
        else
            error("Unknown key \"$key\" in $filename — neither a part nor a setting of \
                   TrajOptSettings.")
        end
    end
    tos.gates.min_height >= 0 || error("min_height must be >= 0, got $(tos.gates.min_height).")
    tos.reopt.reopt_every_n_laps >= 1 ||
        error("reopt_every_n_laps must be >= 1, got $(tos.reopt.reopt_every_n_laps).")
    for (name, value) in (("pattern_azimuth_max", tos.box.pattern_azimuth_max),
                          ("pattern_azimuth_max_high", tos.box.pattern_azimuth_max_high),
                          ("pattern_elevation_amplitude_max",
                           tos.box.pattern_elevation_amplitude_max),
                          ("pattern_elevation_amplitude_max_high",
                           tos.box.pattern_elevation_amplitude_max_high))
        0 <= value <= 90 || error("$name must be in [0, 90], got $value.")
    end
    tos.box.pattern_elevation_amplitude_max_wind_ref >= 0 ||
        error("pattern_elevation_amplitude_max_wind_ref must be >= 0, got "*
              "$(tos.box.pattern_elevation_amplitude_max_wind_ref).")
    tos.box.pattern_elevation_amplitude_max_wind_height >= 0 ||
        error("pattern_elevation_amplitude_max_wind_height must be >= 0, got "*
              "$(tos.box.pattern_elevation_amplitude_max_wind_height).")
    tos.seed.input_depower_wind_ref >= 0 ||
        error("input_depower_wind_ref must be >= 0, got "*
              "$(tos.seed.input_depower_wind_ref).")
    tos.seed.input_depower_per_wind >= 0 ||
        error("input_depower_per_wind must be >= 0, got "*
              "$(tos.seed.input_depower_per_wind).")
    tos.seed.input_depower_seed_max >= 0 ||
        error("input_depower_seed_max must be >= 0, got "*
              "$(tos.seed.input_depower_seed_max).")
    tos.gates.turn_radius_headroom >= 1 ||
        error("turn_radius_headroom must be >= 1, got $(tos.gates.turn_radius_headroom).")
    tos.gates.candidate_elevation_margin >= 0 ||
        error("candidate_elevation_margin must be >= 0, got "*
              "$(tos.gates.candidate_elevation_margin).")
    tos.reopt.path_blend_time > 0 ||
        error("path_blend_time must be > 0, got $(tos.reopt.path_blend_time).")
    0 < tos.reopt_gates.blend_fold_margin <= 1 ||
        error("blend_fold_margin must be in (0, 1], got $(tos.reopt_gates.blend_fold_margin).")
    tos.reopt.blend_max_retries >= 0 ||
        error("blend_max_retries must be >= 0, got $(tos.reopt.blend_max_retries).")
    0 < tos.reopt_gates.min_power_frac <= 1 ||
        error("min_power_frac must be in (0, 1], got $(tos.reopt_gates.min_power_frac).")
    0 <= tos.reopt_gates.min_power_frac_prev <= 1 ||
        error("min_power_frac_prev must be in [0, 1], got "*
              "$(tos.reopt_gates.min_power_frac_prev).")
    tos.reopt_gates.power_gate_wind_min >= 0 ||
        error("power_gate_wind_min must be >= 0, got $(tos.reopt_gates.power_gate_wind_min).")
    tos.reopt_gates.max_size_growth == 0 || tos.reopt_gates.max_size_growth >= 1 ||
        error("max_size_growth must be 0 (off) or >= 1, got $(tos.reopt_gates.max_size_growth).")
    tos.reopt.size_box_growth == 0 || tos.reopt.size_box_growth >= 1 ||
        error("size_box_growth must be 0 (off) or >= 1, got $(tos.reopt.size_box_growth).")
    0 <= tos.box.pattern_climb_angle_max < 90 ||
        error("pattern_climb_angle_max must be in [0, 90), got $(tos.box.pattern_climb_angle_max).")
    tos.reopt.challenge_growth == 0 || tos.reopt.challenge_growth >= 1 ||
        error("challenge_growth must be 0 (off) or >= 1, got $(tos.reopt.challenge_growth).")
    tos.guess.guess_a > 0 && tos.guess.guess_b > 0 ||
        error("guess_a and guess_b must be > 0, got $(tos.guess.guess_a) and $(tos.guess.guess_b).")
    tos.guess.guess_el_center_high >= 0 ||
        error("guess_el_center_high must be >= 0, got $(tos.guess.guess_el_center_high).")
    tos.guess.guess_el_center_wind_ref >= 0 ||
        error("guess_el_center_wind_ref must be >= 0, got "*
              "$(tos.guess.guess_el_center_wind_ref).")
    return tos
end

"""
    turn_radius_lap_reelout(tos::TrajOptSettings, v_wind::Float64)

Reel-out per lap [m] assumed for the startup turn-radius request, from a linear
fit to measured data across wind speeds 4-9 m/s:
`1.987 * v_wind + 14.18` (R² = 0.949).
"""
function turn_radius_lap_reelout(tos::TrajOptSettings, v_wind::Float64)
    return 1.987 * v_wind + 14.18
end

"""
    opt_length(l) -> Float64

Tether length to SEND to the optimizer, rounded to [`OPT_LENGTH_ROUND`](@ref) [m].
The flown `l_set` is never rounded.

Every constraint that depends on the length is sized at this one too, not at the
flown length: the settled `l_set` moves in the 5th decimal with the plant
(150.00282 against 150.00290 m after the SymbolicAWEModels 0.18 bump), and a
`min_turn_radius` changed by 1e-7 of itself missed the failure cache and flipped
which startup seed converges, and so which of two optima the run flew
(2026-09-26, 10 m/s: path centre 26.7° or 40.8°).
"""
opt_length(l; step = OPT_LENGTH_ROUND) = step > 0 ? round(l / step) * step : l
