# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The run inputs of `examples/simple_opt_reelout.jl` and their defaults.

"""
    run_input_defaults() -> NamedTuple

Every input a caller of `simple_opt_reelout.jl` may pass with
`run_example("simple_opt_reelout.jl"; ...)` (see `examples/script_inputs.jl`), at its default:

- `show_plots`: draw the figures.
- `run_archive`: copy the log and its inputs to `output/archives/<stamp>/`.
- `path_tr_project`: a system project whose turn-rate table sizes the path,
  `nothing` for the run's own.
- `fcs_overrides`, `tos_overrides`, `wc_overrides`, `set_overrides`: sweep and
  test overrides of the controller, optimizer, winch and plant settings.
- `steer_disturbance`, `xtrack_offset`, `xtrack_phase`, `hold_compliance`: test
  inputs of the stability and cross-track analyses.
- `steer_gain_factor`, `steer_gain_feedback_only`, `extra_steer_delay`,
  `hook_settle`: the V1 stability hook.
- `replay_paths`: a scenario folder whose optimizer results are replayed.
- `output_path`: where the log goes instead of `output/`, `nothing` for `output/`.
"""
run_input_defaults() =
    (; show_plots = true, run_archive = true, path_tr_project = nothing,
       fcs_overrides = Dict{Symbol, Any}(), tos_overrides = Dict{Symbol, Any}(),
       wc_overrides = Dict{Symbol, Any}(), set_overrides = Dict{Symbol, Any}(),
       steer_disturbance = nothing, xtrack_offset = nothing, xtrack_phase = 5,
       hold_compliance = nothing, steer_gain_factor = 1.0, steer_gain_feedback_only = false,
       extra_steer_delay = 0, hook_settle = 15.0, replay_paths = nothing, output_path = nothing)
