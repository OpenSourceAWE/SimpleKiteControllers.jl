# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The simulated time a reel-out run of `examples/simple_opt_reelout.jl` asks for under a
# wind-speed override. `reelout_budget` reads no model, winch file or wind profile: the caller
# passes the drum's speed limit, the winch's kv and the profile's wind factor, so every branch can
# be tested with hand-made numbers. `sim_budget` reads those three from the project's files.

"Height the budget's wind is taken at [m]."
const BUDGET_HEIGHT = 100.0
"Wind at [`BUDGET_HEIGHT`](@ref) at and above which the square-root law of the budget holds [m/s]."
const BUDGET_KNOT = 7.7
const BUDGET_F_COEF = 48.0             # low-side fit of reeling-mean force ~ w_100²     [N/(m/s)²]
const BUDGET_REEL_MARGIN = 0.9         # achievable fraction of nominal speed (rings, soft-start)
const BUDGET_ENTRY = 25.0              # park + dive + hold + reelout_delay              [s]
const BUDGET_TAIL = 10.0               # soft-stop ramp + phase-5 hold after length stop [s]
const BUDGET_BELOW_KNOT_EXPONENT = 1.6 # exponent for scaling sim_time below the knot

"""
    reelout_budget(wind_speed, default_v_wind, sim_time; l_reel, kv, v_cap, wind_factor)
        -> (; time, below_knot, v_nominal)

Simulated time [s] to fly at the overriding `wind_speed` [m/s] (ground wind at `h_ref`).

At and above [`BUDGET_KNOT`](@ref) (the wind at [`BUDGET_HEIGHT`](@ref), i.e.
`wind_speed * wind_factor`) it is the reel-out budget: `BUDGET_ENTRY`, plus `l_reel` [m] of
tether reeled out at `BUDGET_REEL_MARGIN` of the nominal speed `v_nominal`, plus
`BUDGET_TAIL`. `v_nominal` is the winch's own law `kv * sqrt(force)` at the conservative
force estimate `BUDGET_F_COEF * (wind_speed * wind_factor)²`, capped by the drum's speed
limit `v_cap` [m/s].

Below the knot, which the sqrt-law does not cover, it is `sim_time` [s] (the project's) scaled
by the ratio of the project's `default_v_wind` to `wind_speed`, raised to
`BUDGET_BELOW_KNOT_EXPONENT` when that ratio exceeds 1.

`kv` is the winch's `kv` and `wind_factor` the ratio of
the wind at `BUDGET_HEIGHT` to the one at `h_ref`, from the project's profile law.
"""
function reelout_budget(wind_speed, default_v_wind, sim_time; l_reel, kv, v_cap, wind_factor)
    v_nominal = min(kv * sqrt(BUDGET_F_COEF) * wind_speed * wind_factor, v_cap)
    below_knot = wind_speed * wind_factor < BUDGET_KNOT
    time = if below_knot
        wind_ratio = default_v_wind / wind_speed
        sim_time * (wind_ratio <= 1 ? wind_ratio : wind_ratio^BUDGET_BELOW_KNOT_EXPONENT)
    else
        BUDGET_ENTRY + l_reel / (BUDGET_REEL_MARGIN * v_nominal) + BUDGET_TAIL
    end
    return (; time, below_knot, v_nominal)
end

"""
    _wc_settings_value(project, key) -> Float64

Field `key` of the project's winch-controller file (`wc_settings`, relative to the data
path), read so the budget follows a retune.
"""
function _wc_settings_value(project, key)
    file = KiteUtils.wc_settings(project)
    path = isabspath(file) ? file : joinpath(KiteUtils.get_data_path(), file)
    wcs = YAML.load_file(path)["wc_settings"]
    haskey(wcs, key) || error("No $key in the wc_settings of $path.")
    return Float64(wcs[key])
end

"""
    _drum_speed_limit(project) -> Float64

The drum's reel-out speed limit `v_sat` [m/s] from the project's winch-controller file.
"""
_drum_speed_limit(project) = _wc_settings_value(project, "v_sat")

"""
    sim_budget(project, project_set, fcs, sim_time, wind_speed, default_v_wind) -> Union{Float64, Nothing}

Simulated time [s] to ask `init` for, and the message that says how it was chosen.

With no wind-speed override (`wind_speed` `nothing`) it is `sim_time` (`nothing`: the
project's own). With one, it is [`reelout_budget`](@ref), fed with the drum's `v_sat` from the
project's winch file, the winch's `kv` from the same file and the wind factor
at `BUDGET_HEIGHT` of the project's own profile law. `project_set` is the project's `Settings`,
`fcs` its `FC_Settings` (for `reelout_l_max`) and `default_v_wind` the project's wind before
the override.
"""
function sim_budget(project, project_set, fcs, sim_time, wind_speed, default_v_wind)
    isnothing(wind_speed) && return sim_time
    v_cap = _drum_speed_limit(project)
    # Ratio of the wind at BUDGET_HEIGHT to the one at h_ref, from the project's own profile law.
    wind_factor = calc_wind_factor(AtmosphericModel(project_set; nowindfield = true),
                                   BUDGET_HEIGHT)
    l_reel = fcs.reelout_l_max - project_set.l_tether
    b = reelout_budget(wind_speed, default_v_wind, something(sim_time, project_set.sim_time);
                       l_reel, kv = _wc_settings_value(project, "kv"), v_cap, wind_factor)
    w_budget = wind_speed * wind_factor
    @info "Wind-speed override active, " * (b.below_knot ?
        @sprintf("sim_time scaled to %.1f s (%.1f m/s at %.0f m, below the %.1f m/s knot).",
                 b.time, w_budget, BUDGET_HEIGHT, BUDGET_KNOT) :
        @sprintf("reel-out budget %.1f s (%.0f s entry + %.0f m at %.2f m/s of %.2f nominal \
                  + %.0f s tail; %.1f m/s at %.0f m)",
                 b.time, BUDGET_ENTRY, l_reel, b.v_nominal * BUDGET_REEL_MARGIN, b.v_nominal,
                 BUDGET_TAIL, w_budget, BUDGET_HEIGHT))
    return b.time
end
