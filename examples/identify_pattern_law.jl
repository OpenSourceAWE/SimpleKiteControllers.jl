# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Identify the pattern law, the kite's response time in pattern flight,

    τ_kite + T_kite = pattern_delay_ref · (pattern_v_ref / v_a)^pattern_delay_exp,

and write `pattern_delay_ref`, `pattern_delay_exp` and `pattern_v_floor` into the
course-loop model file of the system projects ([`course_loop_model_file`](@ref),
`data/course_loop_model.yaml`). Step 3 of "Re-identifying after a change of the kite"
(documentation page "Examples - identification").

Each entry of `points` is flown once, without turbulence: the figure of eight
(`simple_fig8.jl`) at 150, 200 and 300 m and several wind speeds, and a weak-wind
reel-out (`simple_reelout.jl`), so that `v_a` spans about 13 – 35 m/s. On each log
the pure delay of the turn rate behind the steering is identified with V3Kite's
`identify_turn_rate_law` on phase 4, from `T_SETTLE` after its start, together with
the median `v_a` and depower of that window. Only logs flown at `pattern_law_depower`
(within `DEPOWER_TOL`) enter the fit: above `wind_ramp_low` the figure-of-eight
projects raise the depower, which step 4 (`pattern_depower_exp`) is about. The fit
is linear least squares of `log(delay)` over `log(pattern_v_ref / v_a)`;
`pattern_v_ref` stays as it is (a reference speed, not identified), and
`pattern_v_floor` becomes the lowest `v_a` of the logs fitted.

The logs are kept in `output/pattern_law/`, so `fly = false` refits them without
flying. The selections of the example menu (project, wind speed, simulation time,
turbulence) are restored afterwards, also after an error. Each run takes one to two
minutes. The inputs are passed with `run_example` (`src/script_inputs.jl`):

    include("examples/identify_pattern_law.jl")                        # fly every point, fit, write
    run_example("identify_pattern_law.jl"; fly = false)                # refit the saved logs
    run_example("identify_pattern_law.jl"; save = false)               # print only
    run_example("identify_pattern_law.jl";
                points = [(project = "system_fig8_300m.yaml", wind = 6.0, sim_time = 120.0,
                           script = "simple_fig8.jl")])                # other points
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using V3Kite: load_log, identify_turn_rate_law, set_default_turbulence
using KiteUtils: Settings
using SimpleKiteControllers
using SimpleKiteControllers: run_example, script_inputs, project_file, skc_data_path
using Printf
using Statistics: median
import Dates
# `wrap_comment` and `update_yaml_values!`.
include(joinpath(@__DIR__, "identification_utils.jl"))

"The runs flown: system project, wind speed [m/s], simulation time [s] (`nothing`: the project's) and script"
const PATTERN_POINTS = [
    (project = "system_fig8_200m.yaml", wind = 4.5, sim_time = 120.0, script = "simple_fig8.jl"),
    (project = "system_fig8_300m.yaml", wind = 5.0, sim_time = 150.0, script = "simple_fig8.jl"),
    (project = "system_fig8_150m.yaml", wind = 7.0, sim_time = 120.0, script = "simple_fig8.jl"),
    (project = "system_fig8_200m.yaml", wind = 7.0, sim_time = 120.0, script = "simple_fig8.jl"),
    (project = "system_fig8_300m.yaml", wind = 7.0, sim_time = 120.0, script = "simple_fig8.jl"),
    (project = "system_reelout_maasvlakte.yaml", wind = 4.0, sim_time = nothing, script = "simple_reelout.jl"),
]
"Where the logs of the runs are kept"
const LOG_DIR = normpath(joinpath(@__DIR__, "..", "output", "pattern_law"))
"Start of the fit window after the start of phase 4 [s]: the transient of the hand-over is left out"
const T_SETTLE = 15.0
"Largest difference of a log's median depower from `pattern_law_depower` that is fitted [-]"
const DEPOWER_TOL = 0.01

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
(; points, fly, save) = script_inputs(@__FILE__, (; points = PATTERN_POINTS, fly = true, save = true))

"File name of the log of `point` in `LOG_DIR`, without extension"
log_label(point) = @sprintf("%s_%.1f", splitext(point.project)[1], point.wind)

"""
    fly_point(point)

Fly `point` without turbulence and copy its log to `LOG_DIR`. The menu's selections
are restored afterwards.
"""
function fly_point(point)
    project0, wind0, sim_time0 = selected_project(), selected_windspeed(), selected_sim_time()
    turbulence0 = selected_turbulence()
    try
        set_selected_project(point.project)
        set_selected_windspeed(point.wind)
        set_selected_sim_time(point.sim_time)
        set_default_turbulence(0.0; data_path = skc_data_path())
        inputs = point.script == "simple_reelout.jl" ? (; show_plots = false, run_archive = false) :
                                                     (; show_plots = false)
        run_example(point.script; inputs...)
    finally
        set_selected_project(project0)
        set_selected_windspeed(wind0)
        set_selected_sim_time(sim_time0)
        set_default_turbulence(turbulence0; data_path = skc_data_path())
    end
    log_name = basename(Settings(project_file(point.project)).log_file)
    src = normpath(joinpath(@__DIR__, "..", "output", log_name * ".arrow"))
    mkpath(LOG_DIR)
    cp(src, joinpath(LOG_DIR, log_label(point) * ".arrow"); force = true)
    return nothing
end

"""
    point_delay(point) -> NamedTuple

The pure delay [s] of the turn rate behind the steering on phase 4 of `point`'s log,
from `T_SETTLE` after its start, with the median `v_a` [m/s] and depower [-] there.
"""
function point_delay(point)
    sl = load_log(log_label(point); path = LOG_DIR).syslog
    p4 = findall(==(4), Int.(sl.sys_state))
    isempty(p4) && error("$(log_label(point)): phase 4 never reached.")
    dt = median(diff(Float64.(sl.time)))
    i1, i2 = p4[1] + round(Int, T_SETTLE / dt), p4[end]
    (i2 - i1) * dt > 20 || error(@sprintf("%s: only %.0f s of phase 4 after %.0f s; lengthen sim_time.",
                                          log_label(point), (i2 - i1) * dt, T_SETTLE))
    id = identify_turn_rate_law(sl[i1:i2]; dt)
    return (; label = log_label(point), delay = id.delay_sec, corr = id.delay_corr,
            v_a = median(Float64.(sl.v_app[i1:i2])), depower = median(Float64.(sl.depower[i1:i2])),
            window = (i2 - i1) * dt)
end

fly && foreach(fly_point, points)
clm = course_loop_model()
results = map(point_delay, points)
used = filter(res -> abs(res.depower - clm.pattern_law_depower) <= DEPOWER_TOL, results)
length(used) >= 3 || error(@sprintf("Only %d logs at depower %.2f ± %.2f; the fit needs at least 3.",
                                    length(used), clm.pattern_law_depower, DEPOWER_TOL))

# log(delay) = log(pattern_delay_ref) + pattern_delay_exp · log(pattern_v_ref / v_a)
log_ratio = log.(clm.pattern_v_ref ./ [res.v_a for res in used])
log_delay = log.([res.delay for res in used])
ratio0, delay0 = sum(log_ratio) / length(log_ratio), sum(log_delay) / length(log_delay)
sxx = sum((log_ratio .- ratio0) .^ 2)
delay_exp = sum((log_ratio .- ratio0) .* (log_delay .- delay0)) / sxx
delay_ref = exp(delay0 - delay_exp * ratio0)
residual = log_delay .- (delay0 .+ delay_exp .* (log_ratio .- ratio0))
se_exp = sqrt(sum(residual .^ 2) / (length(used) - 2) / sxx)
v_floor = minimum(res.v_a for res in used)

println("\n log                               v_a     depower   delay     law      corr   window   fitted")
for res in results
    law = delay_ref * (clm.pattern_v_ref / max(res.v_a, v_floor))^delay_exp
    @printf("  %-32s %5.1f   %.3f     %.3f s   %.3f s   %.3f   %3.0f s    %s\n", res.label, res.v_a,
            res.depower, res.delay, law, res.corr, res.window, res in used ? "yes" : "no (depower)")
end
@printf("\n pattern_delay_ref = %.3f s at %.1f m/s   (was %.3f)\n", delay_ref, clm.pattern_v_ref, clm.pattern_delay_ref)
@printf(" pattern_delay_exp = %.3f ± %.3f        (was %.3f)\n", delay_exp, se_exp, clm.pattern_delay_exp)
@printf(" pattern_v_floor   = %.1f m/s              (was %.1f)\n", v_floor, clm.pattern_v_floor)
@printf(" log-RMS of the fit %.3f\n", sqrt(sum(residual .^ 2) / length(used)))

if save
    project = project_file(selected_project())
    file = joinpath(skc_data_path(), course_loop_model_file(project))
    projects = sort(unique(splitext(point.project)[1] for point in points
                           if any(res -> res.label == log_label(point), used)))
    law_comment = wrap_comment(
        "Pattern law: the kite's response time in pattern flight, dead time + lag = " *
        "pattern_delay_ref * (pattern_v_ref / v_a)^pattern_delay_exp. Identified " *
        "(identify_turn_rate_law, phase 4 from $(T_SETTLE) s after its start) on $(length(used)) logs " *
        @sprintf("at depower %.2f, v_a %.1f - %.1f m/s, ", clm.pattern_law_depower, v_floor,
                 maximum(res.v_a for res in used)) *
        "projects $(join(projects, ", ")); " *
        @sprintf("standard error of the exponent ±%.3f, log-RMS of the fit %.3f. ", se_exp,
                 sqrt(sum(residual .^ 2) / length(used))) *
        "identify_pattern_law.jl, $(Dates.today()).")
    floor_comment = wrap_comment("The lowest v_a the law was identified at; below it the law holds its value.")
    update_yaml_values!(file, ["pattern_delay_ref" => @sprintf("%.3f", delay_ref),
                               "pattern_delay_exp" => @sprintf("%.3f", delay_exp),
                               "pattern_v_floor" => @sprintf("%.1f", v_floor)];
                        comments = Dict("pattern_delay_ref" => law_comment,
                                        "pattern_v_floor" => floor_comment))
    reload_course_loop_model!(project)
    @info "identify_pattern_law: wrote the pattern law to data/$(basename(file))."
end
nothing
