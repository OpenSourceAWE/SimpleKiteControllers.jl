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
rewritten in place; every other line of the file is kept.

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

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
(; depower, winds, save) =
    script_inputs(@__FILE__, (; depower = 0.275, winds = [6.5, V_WIND], save = true))
length(winds) >= 2 || error("identify_kite_delay_scaling.jl needs at least two wind speeds, got $winds.")

"""
    fit_exponent(v_a, x, se) -> (; exp, se)

`exp` of `x ∝ v_a^-exp`: the slope of `log(x)` over `log(v_a)`, weighted with the
relative standard errors `se ./ x`, and its standard error. Unweighted, with a `NaN`
standard error, when a standard error is missing.
"""
function fit_exponent(v_a, x, se)
    lv, lx = log.(v_a), log.(x)
    w = (se ./ x) .^ -2
    weighted = all(isfinite, w)
    weighted || (w = ones(length(x)))
    lv0, lx0 = sum(w .* lv) / sum(w), sum(w .* lx) / sum(w)
    sxx = sum(w .* (lv .- lv0) .^ 2)
    return (; exp = -sum(w .* (lv .- lv0) .* (lx .- lx0)) / sxx, se = weighted ? 1 / sqrt(sxx) : NaN)
end

"""
    wrap_comment(text; indent = "  # ", width = 92) -> Vector{String}

`text` as YAML comment lines of at most `width` characters.
"""
function wrap_comment(text; indent = "  # ", width = 92)
    lines, line = String[], indent
    for word in split(text)
        if length(line) + length(word) + 1 > width && line != indent
            push!(lines, rstrip(line))
            line = indent
        end
        line *= (line == indent ? "" : " ") * word
    end
    push!(lines, line)
    return lines
end

"""
    write_exponents!(file, dead_time_exp, lag_exp, provenance)

Set `kite_dead_time_exp` and `kite_lag_exp` in `file` and replace the comment lines
directly above `kite_dead_time_exp` with `provenance` (comment lines); the column of
the inline comments and every other line are kept.
"""
function write_exponents!(file, dead_time_exp, lag_exp, provenance)
    lines = readlines(file)
    function set_value!(key, x)
        row = findfirst(l -> occursin(Regex("^\\s*$key:"), l), lines)
        isnothing(row) && error("$file has no key $key.")
        m = match(r"^(\s*\w+:\s*)(\S+)(\s*)(#.*)?$", lines[row])
        value = @sprintf("%.3f", x)
        pad = max(length(m[2]) + length(m[3]) - length(value), 1)
        lines[row] = m[1] * value * " "^pad * something(m[4], "")
        return row
    end
    i = set_value!("kite_dead_time_exp", dead_time_exp)
    set_value!("kite_lag_exp", lag_exp)
    k = i
    while k > 1 && startswith(lstrip(lines[k - 1]), "#")
        k -= 1
    end
    lines = [lines[1:k - 1]; provenance; lines[i:end]]
    open(io -> foreach(l -> println(io, l), lines), file, "w")
    return nothing
end

project = sweep_project()
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

v_a = [p.v_a for p in points]
dead_fit = fit_exponent(v_a, [p.dead_time for p in points], [p.se_dead_time for p in points])
lag_fit = fit_exponent(v_a, [p.lag for p in points], [p.se_lag for p in points])

println("\n v_wind    v_a     dead time         lag               flights")
for p in points
    @printf("  %5.2f  %6.2f   %.3f ± %.3f s   %.3f ± %.3f s   %d\n", p.v_wind, p.v_a,
            p.dead_time, p.se_dead_time, p.lag, p.se_lag, p.n_flights)
end
clm = course_loop_model()
@printf("\n kite_dead_time_exp = %.3f ± %.3f   (was %.3f)\n", dead_fit.exp, dead_fit.se, clm.kite_dead_time_exp)
@printf(" kite_lag_exp       = %.3f ± %.3f   (was %.3f)\n", lag_fit.exp, lag_fit.se, clm.kite_lag_exp)

if save
    file = joinpath(skc_data_path(), course_loop_model_file(project))
    fmt(xs) = join([@sprintf("%.3f", x) for x in xs], ", ")
    provenance = wrap_comment(
        "The kite's dead time and lag over v_a, x ∝ v_a^-exp: joint fits of the low-elevation " *
        "relay flights of build_turn_rate_table.jl at depower $depower, " *
        "v_wind $(join(winds, ", ")) m/s (v_a $(join([@sprintf("%.1f", v) for v in v_a], ", ")) m/s): " *
        "dead time $(fmt(p.dead_time for p in points)) s, lag $(fmt(p.lag for p in points)) s. " *
        @sprintf("Standard errors of the exponents ±%.3f and ±%.3f. ", dead_fit.se, lag_fit.se) *
        "identify_kite_delay_scaling.jl, $(basename(project)), $(Dates.today()).")
    write_exponents!(file, dead_fit.exp, lag_fit.exp, provenance)
    reload_course_loop_model!(project)
    @info "identify_kite_delay_scaling: wrote both exponents to data/$(basename(file))."
end
nothing
