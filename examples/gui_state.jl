# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Read/write access to `data/gui.yaml`, the single state file of the example
menu: which system project (`system_*.yaml`), simulation time, set of
plots, turbulence level and wind speed the example scripts fly/show.

Not `Main` globals: `simple_fig8.jl` and `simple_fig8_plots.jl` call
[`selected_project`](@ref)/[`selected_sim_time`](@ref)/[`selected_plots`](@ref)
/[`selected_turbulence`](@ref)/[`selected_windspeed`](@ref) fresh on every
`include`, so a manual re-include always reflects the file on disk rather than
a REPL variable left over from an earlier run. `select_project()`,
`select_sim_time()`, `select_plots()`, `select_turbulence()` and
`select_windspeed()` (`examples/select_*.jl`) are the only writers.

`default_turbulence` shares the file because V3Kite's
`get_default_turbulence`/`set_default_turbulence` read and write it there, in
this package's `data/` rather than the model's read-only one; the writes are
line-based, so all four keys and the comments survive each other.
"""

using SimpleKiteControllers: skc_data_path
using KiteUtils: readfile, writefile, update_yaml_scalar, insert_yaml_scalar_in_section,
                 wind_vec_from_angles

gui_state_file() = joinpath(skc_data_path(), "gui.yaml")
default_project() = "system_fig8_200m.yaml"
default_reelout_project() = "system_reelout_150m.yaml"
default_plots() = ["pattern", "path_3d", "time_series", "power", "aerodynamics"]

function ensure_gui_state_file()
    state_file = gui_state_file()
    if !isfile(state_file)
        default_file = state_file * ".default"
        isfile(default_file) && cp(default_file, state_file)
        # cp preserves the mode, and a read-only checkout would make the file unwritable.
        isfile(state_file) && chmod(state_file, 0o644)
    end
    return state_file
end

function read_gui_field(name::String)
    state_file = ensure_gui_state_file()
    isfile(state_file) || return nothing
    for line in eachline(state_file)
        m = match(Regex("^\\s*" * name * ":\\s*([^#]*?)\\s*(?:#.*)?\$"), line)
        isnothing(m) && continue
        # String(): strip returns a SubString, and callers dispatch on ::String.
        value = String(strip(m.captures[1], ['"']))
        return isempty(value) ? nothing : value
    end
    return nothing
end

function write_gui_field(name::String, value)
    state_file = ensure_gui_state_file()
    lines = readfile(state_file)
    new_lines, updated = update_yaml_scalar(lines, name * ":", value)
    updated || ((new_lines, updated) = insert_yaml_scalar_in_section(lines, "gui:",
                                                                     name * ":", value))
    writefile(new_lines, state_file)
    return nothing
end

"""
    selected_project() -> String

Currently selected system project file name, falling back to
[`default_project`](@ref) if `gui.yaml` has no `project:` entry.
"""
function selected_project()
    value = read_gui_field("project")
    return isnothing(value) ? default_project() : value
end

"""
    selected_reelout_project() -> String

Project the `simple_reelout*.jl` scripts fly: the persisted selection when it
already names a reel-out project (`system_reelout_*.yaml`), otherwise
[`default_reelout_project`](@ref).

`project:` is a single shared key, so without this a fig8 selection left over
from `simple_fig8.jl` would run the reel-out scripts against that project's
`fc_settings.yaml`, whose `compliance: 0.5` (FORCE mode) `simple_reelout.jl`
rejects at startup. Falling back here keeps both families runnable without
re-selecting in between, while `select_project()` still decides between several
reel-out projects once there is more than one.
"""
function selected_reelout_project()
    project = selected_project()
    return startswith(project, "system_reelout") ? project : default_reelout_project()
end

"""
    selected_sim_time() -> Union{Float64, Nothing}

Persisted simulation-time override in seconds, or `nothing` for the
`default` choice (the selected project's own `sim_time`).
"""
function selected_sim_time()
    value = read_gui_field("sim_time")
    (isnothing(value) || value == "default") && return nothing
    return parse(Float64, value)
end

"""
    selected_plots() -> Vector{String}

Persisted set of plots `simple_fig8_plots.jl` shows, falling back to
[`default_plots`](@ref) (all of them) if `gui.yaml` has no `plots:` entry.
"""
function selected_plots()
    value = read_gui_field("plots")
    isnothing(value) && return default_plots()
    value == "none" && return String[]
    return String.(split(value, ','))
end

"""
    selected_windspeed() -> Union{Float64, Nothing}

Persisted wind-speed override in m/s at the project's reference height, or
`nothing` for the `default` choice (the selected project's own `v_wind`).
Pass `something(selected_windspeed(), project_set.v_wind)` to `init`.
"""
function selected_windspeed()
    value = read_gui_field("wind_speed")
    (isnothing(value) || value == "default") && return nothing
    return parse(Float64, value)
end

"""
    selected_turbulence() -> Union{Float64, String, Nothing}

Persisted turbulence level `init` applies, or the `"default"` keyword when the
settings YAML stays in charge. Pass it straight to `init`'s `use_turbulence`.
Read here rather than through V3Kite's `get_default_turbulence` so that picking
a project does not pull the kite model into the session.
"""
function selected_turbulence()
    value = read_gui_field("default_turbulence")
    (isnothing(value) || lowercase(value) == "default") && return "default"
    return parse(Float64, value)
end

"""
    set_selected_project(project::String)

Persist `project` as the current selection, keeping the other entries unchanged.
"""
set_selected_project(project::String) = write_gui_field("project", project)

"""
    set_selected_sim_time(sim_time::Union{Real, Nothing})

Persist `sim_time` (seconds, or `nothing` for `default`) as the current
selection, keeping the other entries unchanged.
"""
set_selected_sim_time(sim_time::Union{Real, Nothing}) =
    write_gui_field("sim_time", isnothing(sim_time) ? "default" : Float64(sim_time))

"""
    set_selected_plots(plots::Vector{String})

Persist `plots` (a subset of [`default_plots`](@ref)) as the current selection,
keeping the other entries unchanged.
"""
set_selected_plots(plots::Vector{String}) =
    write_gui_field("plots", isempty(plots) ? "none" : join(plots, ","))

"""
    set_selected_windspeed(wind_speed::Union{Real, Nothing})

Persist `wind_speed` (m/s, or `nothing` for `default`) as the current
selection, keeping the other entries unchanged.
"""
set_selected_windspeed(wind_speed::Union{Real, Nothing}) =
    write_gui_field("wind_speed", isnothing(wind_speed) ? "default" : Float64(wind_speed))

"""
    apply_windspeed_override!(project_set, wind_speed::Union{Real, Nothing})

Overwrite `project_set`'s wind speed with `wind_speed` [m/s], keeping its
direction and elevation. A no-op for `nothing` (the `default` choice).

`project_set.v_wind = wind_speed` alone only works when the project has
`use_wind_vec: false`: `KiteUtils.Settings` re-derives `v_wind` from
`wind_vec` on every property write when `use_wind_vec` is `true` (all of this
package's settings files set it), so a plain `v_wind` assignment is silently
reverted by that same write. Setting `wind_vec` instead — at the project's
existing `upwind_dir`/`upwind_elevation` — keeps both representations
consistent regardless of which one is authoritative.
"""
function apply_windspeed_override!(project_set, wind_speed::Union{Real, Nothing})
    isnothing(wind_speed) && return nothing
    if project_set.use_wind_vec
        project_set.wind_vec = wind_vec_from_angles(wind_speed,
            deg2rad(project_set.upwind_dir), deg2rad(project_set.upwind_elevation))
    else
        project_set.v_wind = wind_speed
    end
    return nothing
end
