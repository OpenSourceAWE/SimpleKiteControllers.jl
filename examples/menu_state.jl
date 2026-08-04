# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Read/write access to `data/menu_state.yaml`, the persisted choice of which
system project (`system_fig8_*.yaml`), simulation time and set of plots the
example scripts fly/show.

Not `Main` globals: `simple_fig8.jl` and `simple_fig8_plots.jl` call
[`selected_project`](@ref)/[`selected_sim_time`](@ref)/[`selected_plots`](@ref)
fresh on every `include`, so a manual re-include always reflects the file on
disk rather than a REPL variable left over from an earlier run.
`select_project()` (`examples/select_project.jl`), `select_sim_time()`
(`examples/select_sim_time.jl`) and `select_plots()`
(`examples/select_plots.jl`) are the only writers.
"""

using SimpleKiteControllers: skc_data_path

menu_state_file() = joinpath(skc_data_path(), "menu_state.yaml")
default_project() = "system_fig8_200m.yaml"
default_plots() = ["pattern", "time_series", "aerodynamics"]

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
    selected_plots() -> Vector{String}

Persisted set of plots `simple_fig8_plots.jl` shows, falling back to
[`default_plots`](@ref) (all of them) if `menu_state.yaml` has no `plots:`
entry.
"""
function selected_plots()
    value = read_menu_state_field("plots")
    isnothing(value) && return default_plots()
    value == "none" && return String[]
    return String.(split(value, ','))
end

"""
    write_menu_state(; project = selected_project(), sim_time = selected_sim_time(),
                        plots = selected_plots())

Rewrite `menu_state.yaml` with all fields, defaulting each to its current
value so setting one leaves the others untouched.
"""
function write_menu_state(; project::String = selected_project(),
        sim_time::Union{Real, Nothing} = selected_sim_time(),
        plots::Vector{String} = selected_plots())
    open(menu_state_file(), "w") do io
        println(io, "# Session state written by `select_project()`/`select_sim_time()`/")
        println(io, "# `select_plots()` (examples/select_project.jl, examples/select_sim_time.jl,")
        println(io, "# examples/select_plots.jl). Not a tunable — see menu_state.yaml.default.")
        println(io, "project: ", project)
        println(io, "sim_time: ", isnothing(sim_time) ? "default" : sim_time)
        println(io, "plots: ", isempty(plots) ? "none" : join(plots, ","))
    end
    return nothing
end

"""
    set_selected_project(project::String)

Persist `project` as the current selection, keeping `sim_time`/`plots` unchanged.
"""
set_selected_project(project::String) = write_menu_state(; project)

"""
    set_selected_sim_time(sim_time::Union{Real, Nothing})

Persist `sim_time` (seconds, or `nothing` for `default`) as the current
selection, keeping `project`/`plots` unchanged.
"""
set_selected_sim_time(sim_time::Union{Real, Nothing}) = write_menu_state(; sim_time)

"""
    set_selected_plots(plots::Vector{String})

Persist `plots` (a subset of [`default_plots`](@ref)) as the current
selection, keeping `project`/`sim_time` unchanged.
"""
set_selected_plots(plots::Vector{String}) = write_menu_state(; plots)
