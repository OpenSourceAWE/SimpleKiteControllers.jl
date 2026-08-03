module SimpleKiteControllers

using DiscretePIDs, Parameters, KiteUtils, YAML
using LinearAlgebra: dot, cross, norm
using StaticArrays: SVector
using Statistics: mean, std
using Printf: @printf, @sprintf

export ParkingController, ParkingControllerSettings
export linearize, calc_steering, navigate

# Figure-of-eight guidance
export FigureEightSettings, FigureEightController
export figure_eight_path, calc_attractor, navigate_fig8, set_path_center!
export path_tangent
export min_turn_radius, path_min_radius, path_radius_profile, check_pattern_feasible

# Turn-rate-law lookup table
export V3_TURN_RATE_COEFFS, turn_rate_coeffs, V3_TURN_RATE_C1, V3_TURN_RATE_C2
export reload_turn_rate_table!, set_turn_rate_conditions!

# Figure-of-eight run metrics
export fig8_metrics, print_fig8_metrics

# Flight-controller settings
export FC_Settings, winch_force_gains

# Data
export skc_data_path

"""
    skc_data_path() -> String

Absolute path of this package's bundled `data/` directory, holding
`fc_settings.yaml` ([`FC_Settings`](@ref)) and `turn_rate_coeffs.yaml`
([`turn_rate_coeffs`](@ref)).

Deliberately NOT `KiteUtils.get_data_path()`: a run drives a *kite model*
package (e.g. V3Kite) whose own `set_data_path` points the global data path at
that model's geometry, settings and settled-state cache. The controller's own
settings live here rather than being copied into every model package that uses
it, so they are looked up under this path instead of the global one.
"""
skc_data_path() = joinpath(dirname(@__DIR__), "data")

include("parking_controller.jl")
# `turn_rate_table.jl` before `figure_eight_controller.jl`: the guidance's
# feasibility helpers default their `c1` to `V3_TURN_RATE_C1`, which is defined
# there.
include("turn_rate_table.jl")
include("figure_eight_controller.jl")
include("fig8_metrics.jl")
include("fc_settings.jl")

function __init__()
    reload_turn_rate_table!()
end

end
