# Copyright (c) 2025, 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using Pkg
if dirname(Pkg.project().path) != @__DIR__
    Pkg.activate(@__DIR__)
end

using SimpleKiteControllers
using Test

@testset verbose=true "SimpleKiteControllers.jl" begin
    # Pure geometry: no simulation and no kite model, so this runs in under a second.
    include("test_fig8_controller.jl")
    include("test_course_controller.jl")
    include("test_optimization.jl")
    include("test_reelout_metrics.jl")
    include("test_startup_retry.jl")
    include("test_reelout_budget.jl")
    include("test_reopt_gate.jl")
    include("test_pattern_limits.jl")
    include("test_loop_decisions.jl")
    # V5 of oldplans/Plan_model_validation.md: course_loop_model.jl, no simulation.
    include("test_course_loop_model.jl")
end
nothing