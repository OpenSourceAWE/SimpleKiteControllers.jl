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
same wind speed is meant to replace the one before it. `plot_scenario.jl`
replots whatever lands here.

    include("move_scenario.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using YAML

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
    move_scenario(; overwrite = false)

Move the last run's archive (see `last_run_archive`) into
`output/scenarios/vNN`, `NN` its wind speed (see `scenario_name`). Refuses
when the target folder already has files in it unless `overwrite = true`, in
which case they are replaced.
"""
function move_scenario(; overwrite::Bool = false)
    archive_dir = last_run_archive()
    status = only(filter(startswith("status: "), readlines(RUN_DONE_FILE)))
    occursin("ok", status) || @warn "Last run did not finish cleanly: $status"
    name = scenario_name(archive_dir)
    target_dir = joinpath(SCENARIOS_DIR, name)
    if isdir(target_dir) && !isempty(readdir(target_dir)) && !overwrite
        error("$target_dir already has files in it; pass overwrite = true to replace them.")
    end
    mkpath(target_dir)
    for f in readdir(archive_dir; join = true)
        mv(f, joinpath(target_dir, basename(f)); force = overwrite)
    end
    rm(archive_dir)
    @info "Moved $archive_dir to $target_dir"
    return target_dir
end

move_scenario(overwrite = true)
