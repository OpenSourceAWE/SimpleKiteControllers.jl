# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    TrajOptSettings

Settings of a run flown along an EXTERNALLY optimized path
(`examples/simple_opt_fig8.jl`): where the AWETrim server is, the initial guess
the solve starts from, the solver knobs the API exposes, and what is done with
the path that comes back. Loaded from `data/traj_opt.yaml` the same way
[`FC_Settings`](@ref) is loaded from its own — a run is
defined by a file, not by editing a script.

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

# Fields

$(TYPEDFIELDS)
"""
@with_kw mutable struct TrajOptSettings @deftype Float64
    "Address of the AWETrim server"
    base_url::String = "http://127.0.0.1:8000"

    # ---- The initial guess the solve starts from ------------------------- #
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

    # ---- Solver knobs the API exposes ------------------------------------ #
    "Power-tape length seed `l_dp` on AWETrim's scale [m]"
    input_depower = 1.6
    "Wind speed at which `input_depower` is the seed [m/s]"
    input_depower_wind_ref = 7.0
    "Tape length added to the seed per m/s above `input_depower_wind_ref` [m/(m/s)]"
    input_depower_per_wind = 0.0
    "Soft ceiling on the ramped depower seed [m]; `0.0` = AWETrim's hard bound only"
    input_depower_seed_max = 0.0

    # ---- What is done with the path that comes back ---------------------- #
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
    "Minimum curvature margin of the returned path, also sent as turn radius; `0.0` = off"
    min_feasibility_margin = 1.0
    "Factor on the turn radius requested from the optimizer, on top of the gate's [-]"
    turn_radius_headroom = 1.0
    "Ground clearance the returned path must have [m]; `0.0` = off"
    min_height = 50.0

    # ---- Constraints the optimizer solves UNDER -------------------------- #
    # `min_height` above is a gate: the reply is scored and thrown away if it
    # fails, after the solve has been paid for. What follows is sent WITH the
    # request, so the optimizer cannot offer a path that breaks it (AWETrim,
    # 2026-08-19).
    #
    # `min_feasibility_margin` is BOTH now: it still gates the reply, and
    # `min_turn_radius_request` converts it to metres and sends it, so the two
    # cannot disagree and "PATTERN TOO TIGHT" stops being something the solve
    # discovers only after it is paid for. The price is that a constrained cold
    # solve lands on the best branch less often than a free one (11/18 lengths
    # against 16/18, measured by AWETrim on the LEI-V3 reference), so a 422 where
    # there used to be a rejected reply is the expected new failure mode.
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

    # ---- Re-optimization while the tether grows (simple_opt_reelout.jl) --- #
    "Laps between re-optimizations"
    reopt_every_n_laps::Int64 = 2
    "Max re-optimizations per run; bounds wall time"
    max_reopt::Int64 = 4
    "Time over which a new path is blended into the old one [s]"
    path_blend_time = 4.0
    "Fraction of the endpoints' min radius every blended path must clear [-]"
    blend_fold_margin = 0.5
    "Fresh solves allowed when a reply is rejected by the blend or power gates"
    blend_max_retries::Int64 = 3
    "Min fraction of the startup install's predicted power a reply must reach [-]"
    min_power_frac = 0.3
    "Min fraction of the previous install's predicted power a reply must reach [-]"
    min_power_frac_prev = 0.85
    "Wind below which negative power predictions bypass the power gates [m/s]"
    power_gate_wind_min = 4.0
    "Max size growth of a reply relative to the previous install [-]; `0.0` = off"
    max_size_growth = 1.3
    "Box sent with re-optimizations, as factor on the previous install's size; `0` = off"
    size_box_growth = 1.3
    "Growth above which a power-losing reply is challenged by a cold solve; `0` = off"
    challenge_growth = 1.1
    "Halt the simulation while a re-optimization runs"
    reopt_blocking::Bool = true
    "Guess elevation offsets tried in order when the startup solve fails [deg]"
    startup_retry_el_offsets::Vector{Float64} = Float64[]
    "Margin above `min_elevation` a path's lowest point must have [deg]"
    candidate_elevation_margin = 3.0
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

Load the settings from the `traj_opt:` section of `filename`. An unknown key is
an error; a missing one keeps the struct default.
"""
function TrajOptSettings(filename::String; path = skc_data_path())
    tos = load_yaml_fields!(TrajOptSettings(), filename, "traj_opt"; path)
    tos.min_height >= 0 || error("min_height must be >= 0, got $(tos.min_height).")
    tos.reopt_every_n_laps >= 1 ||
        error("reopt_every_n_laps must be >= 1, got $(tos.reopt_every_n_laps).")
    for (name, value) in (("pattern_azimuth_max", tos.pattern_azimuth_max),
                          ("pattern_azimuth_max_high", tos.pattern_azimuth_max_high),
                          ("pattern_elevation_amplitude_max",
                           tos.pattern_elevation_amplitude_max),
                          ("pattern_elevation_amplitude_max_high",
                           tos.pattern_elevation_amplitude_max_high))
        0 <= value <= 90 || error("$name must be in [0, 90], got $value.")
    end
    tos.pattern_elevation_amplitude_max_wind_ref >= 0 ||
        error("pattern_elevation_amplitude_max_wind_ref must be >= 0, got "*
              "$(tos.pattern_elevation_amplitude_max_wind_ref).")
    tos.pattern_elevation_amplitude_max_wind_height >= 0 ||
        error("pattern_elevation_amplitude_max_wind_height must be >= 0, got "*
              "$(tos.pattern_elevation_amplitude_max_wind_height).")
    tos.input_depower_wind_ref >= 0 ||
        error("input_depower_wind_ref must be >= 0, got "*
              "$(tos.input_depower_wind_ref).")
    tos.input_depower_per_wind >= 0 ||
        error("input_depower_per_wind must be >= 0, got "*
              "$(tos.input_depower_per_wind).")
    tos.input_depower_seed_max >= 0 ||
        error("input_depower_seed_max must be >= 0, got "*
              "$(tos.input_depower_seed_max).")
    tos.turn_radius_headroom >= 1 ||
        error("turn_radius_headroom must be >= 1, got $(tos.turn_radius_headroom).")
    tos.candidate_elevation_margin >= 0 ||
        error("candidate_elevation_margin must be >= 0, got "*
              "$(tos.candidate_elevation_margin).")
    tos.path_blend_time > 0 ||
        error("path_blend_time must be > 0, got $(tos.path_blend_time).")
    0 < tos.blend_fold_margin <= 1 ||
        error("blend_fold_margin must be in (0, 1], got $(tos.blend_fold_margin).")
    tos.blend_max_retries >= 0 ||
        error("blend_max_retries must be >= 0, got $(tos.blend_max_retries).")
    0 < tos.min_power_frac <= 1 ||
        error("min_power_frac must be in (0, 1], got $(tos.min_power_frac).")
    0 <= tos.min_power_frac_prev <= 1 ||
        error("min_power_frac_prev must be in [0, 1], got "*
              "$(tos.min_power_frac_prev).")
    tos.power_gate_wind_min >= 0 ||
        error("power_gate_wind_min must be >= 0, got $(tos.power_gate_wind_min).")
    tos.max_size_growth == 0 || tos.max_size_growth >= 1 ||
        error("max_size_growth must be 0 (off) or >= 1, got $(tos.max_size_growth).")
    tos.size_box_growth == 0 || tos.size_box_growth >= 1 ||
        error("size_box_growth must be 0 (off) or >= 1, got $(tos.size_box_growth).")
    0 <= tos.pattern_climb_angle_max < 90 ||
        error("pattern_climb_angle_max must be in [0, 90), got $(tos.pattern_climb_angle_max).")
    tos.challenge_growth == 0 || tos.challenge_growth >= 1 ||
        error("challenge_growth must be 0 (off) or >= 1, got $(tos.challenge_growth).")
    tos.guess_a > 0 && tos.guess_b > 0 ||
        error("guess_a and guess_b must be > 0, got $(tos.guess_a) and $(tos.guess_b).")
    tos.guess_el_center_high >= 0 ||
        error("guess_el_center_high must be >= 0, got $(tos.guess_el_center_high).")
    tos.guess_el_center_wind_ref >= 0 ||
        error("guess_el_center_wind_ref must be >= 0, got "*
              "$(tos.guess_el_center_wind_ref).")
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
