# Copyright (c) 2025, 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using SimpleKiteControllers
using Test

@testset verbose=true "SimpleKiteControllers.jl" begin
    # Pure geometry: no simulation and no kite model, so this runs in under a second.
    include("test_fig8_controller.jl")
    include("test_course_controller.jl")
    include("test_optimization.jl")
    include("test_reelout_metrics.jl")
    # V5 of docs/Plan_model_validation.md: course_loop_model.jl, no simulation.
    include("test_course_loop_model.jl")
end
nothing