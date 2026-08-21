# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Move the last finished `simple_opt_reelout.jl` run into `output/scenarios/`,
named after the wind speed it was flown at — 6.0 m/s becomes `v06`.

The archive to move is read from `output/last_run_done.txt`'s `archive:`
line, written by `reelout_results.jl` as the very last thing a run does. The
wind speed comes from that archive's OWN run-summary YAML (found via its one
`.arrow` log), the value actually passed to `init`, not a project's `v_wind`
default a `WIND_SPEED` override may have replaced. `move_scenario` itself
refuses to overwrite a non-empty scenario folder unless `overwrite = true`,
but running this file always passes `overwrite = true` — a later run at the
same wind speed is meant to replace the one before it. Set `UNIQUE_SCENARIO =
true` before the include (one-shot, like `SHOW_PLOTS`) to keep both instead:
the run then lands in `v08_2`, `v08_3`, ... rather than replacing `v08`.
`plot_scenario.jl` and `plot_powercurve.jl` list/read every non-empty folder
regardless of name, so nothing downstream needs to change to compare them.

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
# `compress_scenario`, run on the folder once the files have landed in it.
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
    scenario_name(archive_dir)

`"vNN"` for the wind speed the run in `archive_dir` was flown at, read from
that run's own summary YAML (the log named by the archive's one `.arrow`
file), rounded to the nearest integer m/s.
"""
function scenario_name(archive_dir::AbstractString)
    arrow_files = filter(f -> endswith(f, ".arrow"), readdir(archive_dir))
    isempty(arrow_files) && error("No .arrow log found in $archive_dir")
    log_name = replace(only(arrow_files), ".arrow" => "")
    summary = YAML.load_file(joinpath(archive_dir, log_name * ".yaml"))
    wind_speed = summary["simulation"]["wind_speed"]
    return "v" * lpad(round(Int, wind_speed), 2, '0')
end

"""
    unique_scenario_dir(name)

The first of `output/scenarios/<name>`, `<name>_2`, `<name>_3`, ... that is
missing or empty, so a run can be archived alongside an existing scenario at
the same wind speed instead of replacing it.
"""
function unique_scenario_dir(name::AbstractString)
    dir = joinpath(SCENARIOS_DIR, name)
    (isdir(dir) && !isempty(readdir(dir))) || return dir
    i = 2
    while true
        candidate = joinpath(SCENARIOS_DIR, "$(name)_$i")
        (isdir(candidate) && !isempty(readdir(candidate))) || return candidate
        i += 1
    end
end

"""
    move_scenario(; overwrite = false, unique = false, compress = true, every = 1)

Move the last run's archive (see `last_run_archive`) into
`output/scenarios/vNN`, `NN` its wind speed (see `scenario_name`). Refuses
when the target folder already has files in it unless `overwrite = true` (in
which case they are replaced) or `unique = true` (in which case the run is
archived into `vNN_2`, `vNN_3`, ... instead — see `unique_scenario_dir`).
`unique` wins if both are set.

`compress = true` then trims the panel corners out of the moved `.arrow` log
(`compress_scenario`), roughly 4x smaller at the full sample rate. `every = n`
additionally keeps only every n-th row — a viewing artefact, not a scoring
input, so leave it at 1 unless the folder is only ever going to be replayed
(see the caveat in `docs/log_size.md`).
"""
function move_scenario(; overwrite::Bool = false, unique::Bool = false, compress::Bool = true, every::Int = 1)
    archive_dir = last_run_archive()
    status = only(filter(startswith("status: "), readlines(RUN_DONE_FILE)))
    occursin("ok", status) || @warn "Last run did not finish cleanly: $status"
    name = scenario_name(archive_dir)
    target_dir = joinpath(SCENARIOS_DIR, name)
    if isdir(target_dir) && !isempty(readdir(target_dir))
        if unique
            target_dir = unique_scenario_dir(name)
        elseif !overwrite
            error("$target_dir already has files in it; pass overwrite = true to replace them, or unique = true to keep both.")
        end
    end
    mkpath(target_dir)
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
