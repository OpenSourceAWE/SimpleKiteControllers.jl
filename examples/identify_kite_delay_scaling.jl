# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Identify how the kite's dead time and lag scale with the apparent wind speed,
`x ∝ v_a^-exp`, and write the exponents `kite_dead_time_exp` and `kite_lag_exp`
into the course-loop model file of the selected project
([`course_loop_model_file`](@ref), `data/course_loop_model.yaml`). Step 2 of
"Re-identifying after a change of the kite" (documentation page "Examples -
identification"); run it after `build_turn_rate_table.jl`.

At one depower, the low-elevation relay flights of `build_turn_rate_table.jl`
(`_fly_low_flights`) are flown at each wind speed of `winds`, and the dead time and
lag of their joint fit are taken at the mean `v_a` of the fit windows. The exponent
of each is the slope of `log(x)` over `log(v_a)`, weighted with the standard errors
of `block_standard_errors`; with two wind speeds it is `log(x₁/x₂) / log(v_a₂/v_a₁)`.
The dead time is fitted on whole samples of `DT`, the project's time step (1/90 s for
the reel-out projects), so its exponent is coarse: check the printed standard errors. The two values and the comment above them are
rewritten in place (`update_yaml_values!` of `identification_utils.jl`); every other
line of the file is kept.

About 5 minutes for two wind speeds (three flights each; the whole grid of
`build_turn_rate_table.jl`, 18 flights, took about 12 minutes). The inputs are
passed with `run_example` (`src/script_inputs.jl`):

    include("examples/identify_kite_delay_scaling.jl")                           # 0.275, 6.5 and 9.51 m/s
    run_example("identify_kite_delay_scaling.jl"; winds = [6.5, 8.0, 9.51])      # more wind speeds
    run_example("identify_kite_delay_scaling.jl"; save = false)                  # print only

At 6.5 m/s the flights cover `v_a` ≈ 13 – 36 m/s; at 5 m/s they drift out of the
wind window (2026-09-29).
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using SimpleKiteControllers: run_example, script_inputs, project_file, skc_data_path
using Printf
using Statistics: mean
import Dates

# `_fly_low_flights`, `block_standard_errors`, `V_WIND`, `DT` and `sweep_project`.
run_example("build_turn_rate_table.jl"; identify = false)
# `wrap_comment` and `update_yaml_values!`.
include(joinpath(@__DIR__, "identification_utils.jl"))

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
(; depower, winds, save) =
    script_inputs(@__FILE__, (; depower = 0.275, winds = [6.5, V_WIND], save = true))
length(winds) >= 2 || error("identify_kite_delay_scaling.jl needs at least two wind speeds, got $winds.")

"""
    fit_exponent(v_a, values, se) -> (; exp, se)

`exp` of `values ∝ v_a^-exp`: the slope of `log(values)` over `log(v_a)`, weighted with
the relative standard errors `se ./ values`, and its standard error. Unweighted, with a `NaN`
standard error, when a standard error is missing.
"""
function fit_exponent(v_a, values, se)
    lv, lx = log.(v_a), log.(values)
    weights = (se ./ values) .^ -2
    weighted = all(isfinite, weights)
    weighted || (weights = ones(length(values)))
    lv0, lx0 = sum(weights .* lv) / sum(weights), sum(weights .* lx) / sum(weights)
    sxx = sum(weights .* (lv .- lv0) .^ 2)
    return (; exp = -sum(weights .* (lv .- lv0) .* (lx .- lx0)) / sxx, se = weighted ? 1 / sqrt(sxx) : NaN)
end

project = sweep_project()
# Step 2: the turn-rate law of step 1 must be the kite's own.
check_model_provenance(project; through = 1)
@info @sprintf("identify_kite_delay_scaling: project %s, depower %.3f, wind speeds %s m/s.",
               basename(project), depower, join(winds, ", "))
points = map(winds) do v_wind
    (; joint_flights, joint) = _fly_low_flights(depower; v_wind)
    isnothing(joint) && error(@sprintf("No flight at %.2f m/s came below MAX_ELEVATION; nothing to fit.", v_wind))
    se = block_standard_errors([f.fit for f in joint_flights])
    (; v_wind, v_a = mean(reduce(vcat, [f.fit.v_app for f in joint_flights])),
       dead_time = joint.dead_time, lag = joint.lag, se_dead_time = se.dead_time, se_lag = se.lag,
       n_flights = length(joint_flights))
end

v_a = [point.v_a for point in points]
dead_fit = fit_exponent(v_a, [point.dead_time for point in points], [point.se_dead_time for point in points])
lag_fit = fit_exponent(v_a, [point.lag for point in points], [point.se_lag for point in points])

println("\n v_wind    v_a     dead time         lag               flights")
for point in points
    @printf("  %5.2f  %6.2f   %.3f ± %.3f s   %.3f ± %.3f s   %d\n", point.v_wind, point.v_a,
            point.dead_time, point.se_dead_time, point.lag, point.se_lag, point.n_flights)
end
clm = course_loop_model()
@printf("\n kite_dead_time_exp = %.3f ± %.3f   (was %.3f)\n", dead_fit.exp, dead_fit.se, clm.kite_dead_time_exp)
@printf(" kite_lag_exp       = %.3f ± %.3f   (was %.3f)\n", lag_fit.exp, lag_fit.se, clm.kite_lag_exp)

if save
    file = joinpath(skc_data_path(), course_loop_model_file(project))
    fmt(values) = join([@sprintf("%.3f", value) for value in values], ", ")
    provenance = wrap_comment(
        "The kite's dead time and lag over v_a, x ∝ v_a^-exp: joint fits of the low-elevation " *
        "relay flights of build_turn_rate_table.jl at depower $depower, " *
        "v_wind $(join(winds, ", ")) m/s (v_a $(join([@sprintf("%.1f", va) for va in v_a], ", ")) m/s): " *
        "dead time $(fmt(point.dead_time for point in points)) s, lag $(fmt(point.lag for point in points)) s. " *
        @sprintf("Standard errors of the exponents ±%.3f and ±%.3f. ", dead_fit.se, lag_fit.se) *
        "identify_kite_delay_scaling.jl, $(basename(project)), $(Dates.today()).")
    update_yaml_values!(file, ["kite_dead_time_exp" => @sprintf("%.3f", dead_fit.exp),
                               "kite_lag_exp" => @sprintf("%.3f", lag_fit.exp),
                               "kite_delay_scaling" => "\"$(kite_id(project))\""];
                        comments = Dict("kite_dead_time_exp" => provenance))
    reload_course_loop_model!(project)
    @info "identify_kite_delay_scaling: wrote both exponents to data/$(basename(file))."
end
nothing
