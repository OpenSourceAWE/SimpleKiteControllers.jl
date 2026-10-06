# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Helpers of the identification scripts that write their results into the course-loop model
# file (`identify_kite_delay_scaling.jl`, `identify_pattern_law.jl`,
# `identify_depower_factor.jl`): flying a run of an example script and identifying the kite's
# response time on its log, recording the kite each log was flown on, and rewriting values of
# the file, which keeps its comments, line by line instead of with YAML.write_file.

using Printf
using Statistics: median
using V3Kite: load_log, identify_turn_rate_law, set_default_turbulence
using KiteUtils: Settings
using SimpleKiteControllers

"""
    log_label(point) -> String

File name of the log of `point` (a NamedTuple with `project` and `wind`, optionally
`depower`), without extension.
"""
log_label(point) = @sprintf("%s_%.1f", splitext(point.project)[1], point.wind) *
                   (haskey(point, :depower) ? @sprintf("_dp%.3f", point.depower) : "")

"""
    fly_point(point, log_dir)

Fly `point` without turbulence: its `script` (`simple_fig8.jl` or `simple_reelout.jl`) with
its `project`, `wind` [m/s] and `sim_time` [s] (`nothing`: the project's), and, when the
point has one, its `depower` as `depower_setpoint` (the input `fcs_overrides`). Its log is
copied to `log_dir` as `log_label(point)`. The menu's selections are restored afterwards,
also after an error.
"""
function fly_point(point, log_dir)
    project0, wind0, sim_time0 = selected_project(), selected_windspeed(), selected_sim_time()
    turbulence0 = selected_turbulence()
    try
        set_selected_project(point.project)
        set_selected_windspeed(point.wind)
        set_selected_sim_time(point.sim_time)
        set_default_turbulence(0.0; data_path = skc_data_path())
        overrides = haskey(point, :depower) ? Dict{Symbol, Any}(:depower_setpoint => point.depower) :
                                              Dict{Symbol, Any}()
        inputs = point.script == "simple_reelout.jl" ?
            (; show_plots = false, run_archive = false, fcs_overrides = overrides) :
            (; show_plots = false, fcs_overrides = overrides)
        run_example(point.script; inputs...)
    finally
        set_selected_project(project0)
        set_selected_windspeed(wind0)
        set_selected_sim_time(sim_time0)
        set_default_turbulence(turbulence0; data_path = skc_data_path())
    end
    log_name = basename(Settings(project_file(point.project)).log_file)
    src = normpath(joinpath(skc_data_path(), "..", "output", log_name * ".arrow"))
    mkpath(log_dir)
    cp(src, joinpath(log_dir, log_label(point) * ".arrow"); force = true)
    record_log_kite!(joinpath(log_dir, log_label(point)), point.project)
    return nothing
end

"""
    record_log_kite!(log_path, project)

Record the `kite_id` of `project` ([`kite_id`](@ref)) beside the log `log_path` (without
extension), in `<log_path>.kite_id`, for [`check_log_kite`](@ref).
"""
record_log_kite!(log_path, project) = write(log_path * ".kite_id", kite_id(project))

"""
    check_log_kite(log_path, project)

Throw if the log `log_path` (without extension) was flown on another kite than `project`
flies now, so that a refit of saved logs (`fly = false`) cannot write a value of an old kite
with the provenance of the new one. Warns for a log flown before the record was kept.
"""
function check_log_kite(log_path, project)
    file = log_path * ".kite_id"
    if !isfile(file)
        @warn "$(basename(log_path)): no record of the kite it was flown on; fly it again if the kite changed since."
        return nothing
    end
    flown, current = strip(read(file, String)), kite_id(project)
    flown == current || error("$(basename(log_path)) was flown on kite $flown, $(basename(project)) " *
                              "flies kite $current: fly it again (fly = true).")
    return nothing
end

"""
    point_delay(point, log_dir; t_settle = 15.0) -> NamedTuple

The pure delay [s] of the turn rate behind the steering on phase 4 of `point`'s log in
`log_dir` (V3Kite's `identify_turn_rate_law`), from `t_settle` [s] after its start, with
its correlation and the median `v_a` [m/s] and depower [-] of that window.
"""
function point_delay(point, log_dir; t_settle = 15.0)
    check_log_kite(joinpath(log_dir, log_label(point)), point.project)
    sl = load_log(log_label(point); path = log_dir).syslog
    p4 = findall(==(4), Int.(sl.sys_state))
    isempty(p4) && error("$(log_label(point)): phase 4 never reached.")
    dt = median(diff(Float64.(sl.time)))
    i1, i2 = p4[1] + round(Int, t_settle / dt), p4[end]
    (i2 - i1) * dt > 20 || error(@sprintf("%s: only %.0f s of phase 4 after %.0f s; lengthen sim_time.",
                                          log_label(point), (i2 - i1) * dt, t_settle))
    id = identify_turn_rate_law(sl[i1:i2]; dt)
    return (; label = log_label(point), delay = id.delay_sec, corr = id.delay_corr,
            v_a = median(Float64.(sl.v_app[i1:i2])), depower = median(Float64.(sl.depower[i1:i2])),
            window = (i2 - i1) * dt)
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
    update_yaml_values!(file, values; comments = Dict())

Set each `key => value` of `values` (the value already formatted as a string) in the YAML
file `file`, keeping the column of the inline comment, and replace the comment lines
directly above each key of `comments` with its lines. Every other line is kept.
"""
function update_yaml_values!(file, values; comments = Dict{String, Vector{String}}())
    lines = readlines(file)
    function find_key(key)
        found = findfirst(line -> occursin(Regex("^\\s*$key:"), line), lines)
        isnothing(found) && error("$file has no key $key.")
        return found
    end
    for (key, value) in values
        row = find_key(key)
        m = match(r"^(\s*\w+:\s*)(\S+)(\s*)(#.*)?$", lines[row])
        pad = max(length(m[2]) + length(m[3]) - length(value), 1)
        lines[row] = m[1] * value * " "^pad * something(m[4], "")
    end
    for (key, block) in comments
        row = find_key(key)
        first_comment = row
        while first_comment > 1 && startswith(lstrip(lines[first_comment - 1]), "#")
            first_comment -= 1
        end
        lines = [lines[1:first_comment - 1]; block; lines[row:end]]
    end
    open(io -> foreach(line -> println(io, line), lines), file, "w")
    return nothing
end
