# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The simulated time a reel-out run of `examples/simple_opt_reelout.jl` asks for under a
# wind-speed override. No model, winch file or wind profile is read here: the caller passes the
# drum's speed limit, the winch's kv and the profile's wind factor, so every branch can be tested
# with hand-made numbers.

const BUDGET_HEIGHT = 100.0            # height the budget's wind is taken at            [m]
const BUDGET_KNOT = 7.7                # wind at BUDGET_HEIGHT, sqrt-law valid at/above  [m/s]
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

`kv` is the winch's `kv` at `wind_speed` ([`winch_kv`](@ref)) and `wind_factor` the ratio of
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
