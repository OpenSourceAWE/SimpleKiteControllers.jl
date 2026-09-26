# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Re-fly every archived scenario of both sites (`output/scenarios/maasvlakte/`
and `output/scenarios/cabauw/`) with the current code and settings, and
replace each scenario folder with the new run.

For each site the wind speeds are read from the run summaries of its scenario
folders; a `_2`, `_3`, ... repeat shares its wind speed with the unsuffixed
folder, so each wind speed is flown once and replaces the unsuffixed folder
only (the repeats are kept as they are). Each run is
`simple_opt_reelout.jl` without plots, at the project and wind speed set in
`data/gui.yaml`, and is moved in by `move_scenario.jl`, but only if it finished
cleanly and passed all success criteria: otherwise the old scenario is kept and
the failure is logged. After a site, `create_overview.jl` rewrites its
`overview.md`. The `gui.yaml` selection is restored at the end, also after an
error.

Progress, one line per run, goes to `output/build_all_scenarios.txt`. About
30 minutes for the 22 scenarios of 2026-09-26. Do not start other runs
meanwhile: they write the same `output/` files.

    include("build_all_scenarios.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

import YAML
include(joinpath(@__DIR__, "gui_state.jl"))

"Project flown at each site, see `scenario_site`"
const SITE_PROJECTS = ("maasvlakte" => "system_reelout_maasvlakte.yaml",
                       "cabauw" => "system_reelout_cabauw.yaml")
output_dir = normpath(joinpath(@__DIR__, "..", "output"))
run_done_file = joinpath(output_dir, "last_run_done.txt")
progress_file = joinpath(output_dir, "build_all_scenarios.txt")

"""
    scenario_winds(site) -> Vector{Float64}

The distinct wind speeds of the scenario folders of `site`, from each folder's
run summary (the YAML with a `simulation:` section), sorted ascending.
"""
function scenario_winds(site)
    winds = Float64[]
    for dir in readdir(joinpath(output_dir, "scenarios", site); join = true)
        isdir(dir) || continue
        for f in filter(endswith("_opt.yaml"), readdir(dir))
            y = YAML.load_file(joinpath(dir, f))
            y isa AbstractDict && haskey(y, "simulation") &&
                push!(winds, Float64(y["simulation"]["wind_speed"]))
        end
    end
    return sort(unique(winds))
end

first_line(e) = first(split(sprint(showerror, e), '\n'))
log_line(s) = (open(io -> println(io, s), progress_file, "a"); @info s)

gui_project0, gui_wind0 = read_gui_field("project"), read_gui_field("wind_speed")
write(progress_file, "")
try
    for (site, project) in SITE_PROJECTS
        set_selected_project(project)
        for wind in scenario_winds(site)
            write_gui_field("wind_speed", wind)
            rm(run_done_file; force = true)
            t0 = time()
            try
                global SHOW_PLOTS = false
                include(joinpath(@__DIR__, "simple_opt_reelout.jl"))
            catch e
                log_line("$site $wind m/s: the run threw $(first_line(e))")
            end
            done = isfile(run_done_file) ? read(run_done_file, String) : ""
            criteria = match(r"criteria: (.*)", done)
            archive = match(r"archive: (.*)", done)
            summary = replace(strip(done), '\n' => "; ")
            if occursin("status: ok", done) && !isnothing(criteria) && !isnothing(archive) &&
               occursin(r"^all \d+ passed$", strip(criteria.captures[1]))
                global SCENARIO_ARCHIVE = strip(archive.captures[1])
                include(joinpath(@__DIR__, "move_scenario.jl"))
                log_line("$site $wind m/s: moved in after $(round(Int, time() - t0)) s; $summary")
            else
                log_line("$site $wind m/s: NOT moved, the old scenario is kept; $summary")
            end
        end
        try
            include(joinpath(@__DIR__, "create_overview.jl"))
        catch e
            log_line("$site: create_overview.jl threw $(first_line(e))")
        end
    end
finally
    set_selected_project(gui_project0)
    write_gui_field("wind_speed", something(gui_wind0, "default") == "default" ? "default" :
                                   parse(Float64, gui_wind0))
    log_line("done")
end
nothing
