# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Move the last finished `simple_opt_reelout.jl` run into a site subfolder of
`output/scenarios/` — `cabauw/` for a run flown with the
`system_reelout_cabauw.yaml` project, `maasvlakte/` for every other project
(see `archive_site`) — named after the wind speed it was flown at: 6.0 m/s
becomes `v06`, a fractional wind speed like 3.5 m/s becomes `v03.5` (see
`scenario_name`).

The archive to move is read from `output/last_run_done.txt`'s `archive:`
line, written by `reelout_results.jl` as the very last thing a run does. The
wind speed and the project come from that archive's OWN run-summary YAML
(found via its one `.arrow` log): the values actually passed to `init`, not a
project's `v_wind` default a `WIND_SPEED` override may have replaced, nor
whatever `gui.yaml` selects by now. `move_scenario` itself
refuses to overwrite a non-empty scenario folder unless `overwrite = true`,
but running this file always passes `overwrite = true` — a later run at the
same wind speed is meant to replace the one before it. Set `UNIQUE_SCENARIO =
true` before the include (one-shot, like `SHOW_PLOTS`) to keep both instead:
the run then lands in `v08_2`, `v08_3`, ... rather than replacing `v08`.

The log is compressed on the way in (`examples/compress.jl`): the VSM panel
corners, ~78 % of an `.arrow` file and pure visualisation data, are dropped,
which is what keeps a scenario folder pushable to `SimulationResults` — see
`docs/log_size.md`. Nothing downstream notices; `plot_scenario.jl` and
`simple_reelout_play.jl` read a trimmed log unchanged. Pass `compress = false`
to keep the corners.

    include("move_scenario.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using YAML
# `compress_scenario`, run on the folder once the files have landed in it; it
# pulls in `gui_state.jl`, the home of `scenario_site`.
include(joinpath(@__DIR__, "compress.jl"))

const OUTPUT_DIR = normpath(joinpath(@__DIR__, "..", "output"))
const RUN_DONE_FILE = joinpath(OUTPUT_DIR, "last_run_done.txt")
const SCENARIOS_DIR = joinpath(OUTPUT_DIR, "scenarios")

"""
    last_run_archive()

The archive directory of the most recently finished run, from
`output/last_run_done.txt`. Errors if the marker is missing or names no
archive (an early crash, before `archive_dir` was even assigned).
"""
function last_run_archive()
    isfile(RUN_DONE_FILE) ||
        error("$RUN_DONE_FILE not found; run simple_opt_reelout.jl first.")
    line = only(filter(startswith("archive: "), readlines(RUN_DONE_FILE)))
    archive = strip(replace(line, "archive: " => ""))
    archive == "none" && error("Last run has no archive (see $RUN_DONE_FILE).")
    isdir(archive) || error("Archived directory $archive no longer exists.")
    return archive
end

"""
    archive_summary(archive_dir)

The run-summary YAML of the run in `archive_dir`: the log named by the
archive's one `.arrow` file, with the `.yaml` extension instead.
"""
function archive_summary(archive_dir::AbstractString)
    arrow_files = filter(f -> endswith(f, ".arrow"), readdir(archive_dir))
    isempty(arrow_files) && error("No .arrow log found in $archive_dir")
    log_name = replace(only(arrow_files), ".arrow" => "")
    return YAML.load_file(joinpath(archive_dir, log_name * ".yaml"))
end

"""
    archive_site(archive_dir)

The site subfolder of `output/scenarios/` a run belongs in — `scenario_site`
(`gui_state.jl`) of the system project its own summary YAML records, not of
whatever `gui.yaml` selects by now.
"""
function archive_site(archive_dir::AbstractString)
    return scenario_site(String(archive_summary(archive_dir)["simulation"]["project"]))
end

"""
    scenario_name(archive_dir)

`"vNN"` for the wind speed the run in `archive_dir` was flown at, read from
that run's own summary YAML (see `archive_summary`). An integer wind speed
(6.0 m/s) keeps the plain `"v06"` every existing scenario folder uses; a
fractional one (3.5 m/s) becomes `"v03.5"` rather than being rounded into an
integer bucket — `round(Int, 3.5) == round(Int, 4.5) == 4`
(round-half-to-even), which silently collided a 3.5 m/s run into the same
`v04` folder an actual 4.0 m/s run already used. The zero-padded whole part
keeps every name the same length as its neighbours, so `sort`ing folder names
(`plot_scenario.jl`'s menu) still orders by wind speed.
"""
function scenario_name(archive_dir::AbstractString)
    summary = archive_summary(archive_dir)
    w = round(Float64(summary["simulation"]["wind_speed"]); digits = 2)
    whole = Int(floor(w))
    frac = round(w - whole; digits = 2)
    name = "v" * lpad(whole, 2, '0')
    isapprox(frac, 0.0; atol = 1e-9) || (name *= lstrip(string(frac), '0'))
    return name
end

"""
    unique_scenario_dir(site_dir, name)

The first of `<site_dir>/<name>`, `<name>_2`, `<name>_3`, ... that is missing
or empty, so a run can be archived alongside an existing scenario at the same
wind speed instead of replacing it.
"""
function unique_scenario_dir(site_dir::AbstractString, name::AbstractString)
    dir = joinpath(site_dir, name)
    (isdir(dir) && !isempty(readdir(dir))) || return dir
    i = 2
    while true
        candidate = joinpath(site_dir, "$(name)_$i")
        (isdir(candidate) && !isempty(readdir(candidate))) || return candidate
        i += 1
    end
end

"""
    move_scenario(; overwrite = false, unique = false, compress = true, every = 1)

Move the last run's archive (see `last_run_archive`) into
`output/scenarios/<site>/vNN`, `site` the project's site (see `archive_site`)
and `NN` its wind speed (see `scenario_name`). Refuses
when the target folder already has files in it unless `overwrite = true` (in
which case they are replaced) or `unique = true` (in which case the run is
archived into `vNN_2`, `vNN_3`, ... instead — see `unique_scenario_dir`).
`unique` wins if both are set.

`compress = true` then trims the panel corners out of the moved `.arrow` log
(`compress_scenario`), roughly 4x smaller at the full sample rate. `every = n`
additionally keeps only every n-th row, resampling to 30 Hz by default
(`every = 3`) to keep a scenario folder pushable to `SimulationResults` —
a viewing artefact, not a scoring input (see the caveat in `docs/log_size.md`).
Pass `every = 1` to keep the full sample rate instead.
"""
function move_scenario(; overwrite::Bool = false, unique::Bool = false, compress::Bool = true, every::Int = 3)
    archive_dir = last_run_archive()
    status = only(filter(startswith("status: "), readlines(RUN_DONE_FILE)))
    occursin("ok", status) || @warn "Last run did not finish cleanly: $status"
    site_dir = joinpath(SCENARIOS_DIR, archive_site(archive_dir))
    name = scenario_name(archive_dir)
    target_dir = joinpath(site_dir, name)
    if isdir(target_dir) && !isempty(readdir(target_dir))
        if unique
            target_dir = unique_scenario_dir(site_dir, name)
        elseif !overwrite
            error("$target_dir already has files in it; pass overwrite = true to replace them, or unique = true to keep both.")
        end
    end
    mkpath(target_dir)
    # A scenario is ONE run's record, so replacing it means replacing the folder,
    # not the files that happen to share a name: a project renamed between runs
    # (system_reelout_150m -> system_reelout_maasvlakte, 2026-09-20) left the old
    # copy beside the new one in every overwritten folder, and the plots' project
    # lookup found two where it expected one.
    overwrite && foreach(rm, readdir(target_dir; join = true))
    for f in readdir(archive_dir; join = true)
        mv(f, joinpath(target_dir, basename(f)); force = overwrite)
    end
    rm(archive_dir)
    compress && compress_scenario(target_dir; every)
    @info "Moved $archive_dir to $target_dir"
    return target_dir
end

# One-shot override, like `SHOW_PLOTS`: set `UNIQUE_SCENARIO = true` before the
# include to archive alongside the existing scenario instead of replacing it.
unique_scenario = @isdefined(UNIQUE_SCENARIO) ? UNIQUE_SCENARIO : false
UNIQUE_SCENARIO = false
move_scenario(overwrite = true, unique = unique_scenario)
