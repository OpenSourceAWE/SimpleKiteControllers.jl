# Copyright (c) 2025, 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

module SimpleKiteControllers

using DiscretePIDs, Parameters, KiteUtils, YAML
using LinearAlgebra: dot, cross, norm
using StaticArrays: SVector
using Statistics: mean, std
using Printf: @printf, @sprintf

export ParkingController, ParkingControllerSettings
export linearize, calc_steering, navigate

# Figure-of-eight inner-loop (course) controller
export CourseController, CourseControllerSettings, set_phase!

# Figure-of-eight guidance
export FigureEightSettings, FigureEightController
export figure_eight_path, calc_attractor, navigate_fig8, set_path_center!
export set_path!, resample_path, prepare_path, blend_paths, lobe_lift
export bias_lift, smooth_bins, azimuth_frac, azimuth_bin
export path_tangent
export min_turn_radius, path_min_radius, path_radius_profile, check_pattern_feasible
export path_min_height, check_pattern_height

# Turn-rate-law lookup table
export V3_TURN_RATE_COEFFS, turn_rate_coeffs, V3_TURN_RATE_C1, V3_TURN_RATE_C2
export reload_turn_rate_table!

# Figure-of-eight run metrics
export fig8_metrics, print_fig8_metrics, reelout_power, reelout_ringing
export winch_state_pct

# Flight-controller settings
export FC_Settings, winch_force_gains, project_file, fc_settings, load_yaml_fields!

# Externally optimized flight path (examples/simple_opt_fig8.jl)
export TrajOptSettings

# Parallel shape optimization (examples/optimize_fig8.jl)
export OptSettings, opt_grid, task_key, pattern_margin, filter_grid
export with_file_lock, init_results_file, record_result!, load_results
export claim_task!, release_claims!, reset_claims!, n_unclaimed
export run_metrics, side_conditions, rank_results, unique_results
export format_results_table

# Data
export skc_data_path

"""
    skc_data_path() -> String

Absolute path of this package's bundled `data/` directory, holding
`fc_settings.yaml` ([`FC_Settings`](@ref)), `turn_rate_coeffs.yaml`
([`turn_rate_coeffs`](@ref)) and the system project of a run
([`project_file`](@ref)).

Not `KiteUtils.get_data_path()`: that points at the *kite model's* data
directory during a run, while these settings belong to the controller.
"""
skc_data_path() = joinpath(dirname(@__DIR__), "data")

include("parking_controller.jl")
# Before figure_eight_controller.jl: its feasibility helpers default c1 to V3_TURN_RATE_C1.
include("turn_rate_table.jl")
include("figure_eight_controller.jl")
include("fig8_metrics.jl")
include("fc_settings.jl")
# After fc_settings.jl: the FC_Settings constructor is defined with the type it constructs.
include("course_controller.jl")
# After fc_settings.jl: OptSettings is loaded the same way and documents itself
# against FC_Settings, whose fields the sweep overrides.
include("optimization.jl")
# After fc_settings.jl too: TrajOptSettings is loaded through load_yaml_fields!.
include("traj_opt_settings.jl")

function __init__()
    reload_turn_rate_table!()
end

end
