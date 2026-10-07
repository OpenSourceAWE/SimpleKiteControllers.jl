# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Identify how the kite's response time in pattern flight grows with the depower,

    τ_kite + T_kite = pattern delay approximation at v_a · exp(pattern_depower_exp · (depower − pattern_delay_depower)),

and write `pattern_depower_exp` into the course-loop model file of the system projects
([`course_loop_model_file`](@ref), `data/course_loop_model.yaml`). Step 4 of
"Re-identifying after a change of the kite" (documentation page "Examples -
identification"); run it after `identify_pattern_delay.jl`, whose approximation it divides by.

Point D of the model validation (`system_fig8_300m.yaml`, 7 m/s, 120 s, no turbulence)
is flown with `simple_fig8.jl` at each depower of `depowers`, the depower set with the
input `fcs_overrides` (`depower_setpoint`). On each log the pure delay of the turn rate
behind the steering is identified on phase 4, from `T_SETTLE` after its start
(`point_delay` of `identification_utils.jl`), and divided by the pattern delay approximation of the
course-loop model at the log's median `v_a`. The exponent is the slope of the log of that
ratio over `depower − pattern_delay_depower`, fitted through the origin, since the approximation holds
at `pattern_delay_depower` by definition; the run at `pattern_delay_depower` itself is the
check that it does.

The logs are kept in `output/depower_factor/`, so `fly = false` refits them without
flying. About 1.5 minutes per run. The inputs are passed with `run_example`
(`src/script_inputs.jl`):

    include("examples/identify_depower_factor.jl")                           # fly, fit, write
    run_example("identify_depower_factor.jl"; fly = false)                   # refit the saved logs
    run_example("identify_depower_factor.jl"; save = false)                  # print only
    run_example("identify_depower_factor.jl"; depowers = [0.27, 0.30, 0.33]) # other depowers
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using SimpleKiteControllers: run_example, script_inputs, project_file, skc_data_path
using Printf
import Dates
# `fly_point`, `point_delay`, `wrap_comment` and `update_yaml_values!`.
include(joinpath(@__DIR__, "identification_utils.jl"))

"The operating point flown at every depower: project, wind speed [m/s], simulation time [s], script"
const DEPOWER_POINT = (project = "system_fig8_300m.yaml", wind = 7.0, sim_time = 120.0,
                       script = "simple_fig8.jl")
"Where the logs of the runs are kept"
const LOG_DIR = normpath(joinpath(@__DIR__, "..", "output", "depower_factor"))
"Start of the fit window after the start of phase 4 [s]: the transient of the hand-over is left out"
const T_SETTLE = 15.0

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
(; depowers, fly, save) =
    script_inputs(@__FILE__, (; depowers = [0.27, 0.30, 0.33, 0.36], fly = true, save = true))

points = [merge(DEPOWER_POINT, (; depower = Float64(depower))) for depower in depowers]
# Step 4: steps 1 - 3 must be the kite's own, and point D must fly that kite.
check_model_provenance(project_file(selected_project()); through = 3, flown = (DEPOWER_POINT.project,))
fly && foreach(point -> fly_point(point, LOG_DIR), points)
clm = course_loop_model()
"The pattern delay approximation at `pattern_delay_depower` and `v_a` [m/s], floored as `pattern_dead_time_lag` does"
pattern_delay(clm, v_a) = clm.pattern_delay_ref *
                        (clm.pattern_v_ref / max(v_a, clm.pattern_v_floor))^clm.pattern_delay_exp
results = map(points) do point
    res = point_delay(point, LOG_DIR; t_settle = T_SETTLE)
    merge(res, (; ratio = res.delay / pattern_delay(clm, res.v_a)))
end

# log(ratio) = pattern_depower_exp · (depower − pattern_delay_depower), through the origin.
offset = [res.depower - clm.pattern_delay_depower for res in results]
log_ratio = log.([res.ratio for res in results])
sxx = sum(offset .^ 2)
sxx > 0 || error("All runs flew at pattern_delay_depower; nothing to fit.")
depower_exp = sum(offset .* log_ratio) / sxx
residual = log_ratio .- depower_exp .* offset
se_exp = length(results) > 1 ? sqrt(sum(residual .^ 2) / (length(results) - 1) / sxx) : NaN

println("\n log                                   depower   v_a     delay     approx      ratio   model   corr")
for res in results
    @printf("  %-36s %.3f    %5.1f   %.3f s   %.3f s   %.2f    %.2f    %.3f\n", res.label, res.depower,
            res.v_a, res.delay, pattern_delay(clm, res.v_a), res.ratio,
            exp(depower_exp * (res.depower - clm.pattern_delay_depower)), res.corr)
end
@printf("\n pattern_depower_exp = %.2f ± %.2f   (was %.2f)\n", depower_exp, se_exp, clm.pattern_depower_exp)

if save
    project = project_file(selected_project())
    file = joinpath(skc_data_path(), course_loop_model_file(project))
    fmt(values) = join([@sprintf("%.3f", value) for value in values], " / ")
    comment = wrap_comment(
        "Growth with depower, exp(pattern_depower_exp * (depower - pattern_delay_depower)): " *
        "point D ($(splitext(DEPOWER_POINT.project)[1]), $(DEPOWER_POINT.wind) m/s) flown at depower " *
        "$(fmt(res.depower for res in results)) gave x$(join([@sprintf("%.2f", res.ratio) for res in results], " / ")) " *
        "over the pattern delay approximation at the same v_a (identify_turn_rate_law, phase 4 from $(T_SETTLE) s after its start); " *
        @sprintf("standard error of the exponent ±%.2f. ", se_exp) *
        "identify_depower_factor.jl, $(Dates.today()).")
    update_yaml_values!(file, ["pattern_depower_exp" => @sprintf("%.2f", depower_exp),
                               "depower_factor" => "\"$(kite_id(project))\""];
                        comments = Dict("pattern_depower_exp" => comment))
    reload_course_loop_model!(project)
    @info "identify_depower_factor: wrote pattern_depower_exp to data/$(basename(file))."
end
nothing
