# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Read/write access to `data/menu_state.yaml`, the persisted choice of which
system project (`system_fig8_*.yaml`) and simulation time the example
scripts fly.

Not `Main` globals: `simple_fig8.jl` and `simple_fig8_plots.jl` call
[`selected_project`](@ref)/[`selected_sim_time`](@ref) fresh on every
`include`, so a manual re-include always reflects the file on disk rather
than a REPL variable left over from an earlier run. `select_project()`
(`examples/select_project.jl`) and `select_sim_time()`
(`examples/select_sim_time.jl`) are the only writers.
"""

using SimpleKiteControllers: skc_data_path

menu_state_file() = joinpath(skc_data_path(), "menu_state.yaml")
default_project() = "system_fig8_200m.yaml"

function ensure_menu_state_file()
    state_file = menu_state_file()
    if !isfile(state_file)
        default_file = state_file * ".default"
        isfile(default_file) && cp(default_file, state_file)
    end
    return state_file
end

function read_menu_state_field(name::String)
    state_file = ensure_menu_state_file()
    isfile(state_file) || return nothing
    for line in eachline(state_file)
        m = match(Regex("^\\s*" * name * ":\\s*(\\S+)"), line)
        isnothing(m) || return String(m.captures[1])
    end
    return nothing
end

"""
    selected_project() -> String

Currently selected system project file name, falling back to
[`default_project`](@ref) if `menu_state.yaml` has no `project:` entry.
"""
function selected_project()
    value = read_menu_state_field("project")
    return isnothing(value) ? default_project() : value
end

"""
    selected_sim_time() -> Union{Float64, Nothing}

Persisted simulation-time override in seconds, or `nothing` for the
`default` choice (the selected project's own `sim_time`).
"""
function selected_sim_time()
    value = read_menu_state_field("sim_time")
    (isnothing(value) || value == "default") && return nothing
    return parse(Float64, value)
end

"""
    write_menu_state(; project = selected_project(), sim_time = selected_sim_time())

Rewrite `menu_state.yaml` with both fields, defaulting each to its current
value so setting one leaves the other untouched.
"""
function write_menu_state(; project::String = selected_project(),
        sim_time::Union{Real, Nothing} = selected_sim_time())
    open(menu_state_file(), "w") do io
        println(io, "# Session state written by `select_project()`/`select_sim_time()`")
        println(io, "# (examples/select_project.jl, examples/select_sim_time.jl).")
        println(io, "# Not a tunable — see menu_state.yaml.default.")
        println(io, "project: ", project)
        println(io, "sim_time: ", isnothing(sim_time) ? "default" : sim_time)
    end
    return nothing
end

"""
    set_selected_project(project::String)

Persist `project` as the current selection, keeping `sim_time` unchanged.
"""
set_selected_project(project::String) = write_menu_state(; project)

"""
    set_selected_sim_time(sim_time::Union{Real, Nothing})

Persist `sim_time` (seconds, or `nothing` for `default`) as the current
selection, keeping `project` unchanged.
"""
set_selected_sim_time(sim_time::Union{Real, Nothing}) = write_menu_state(; sim_time)
