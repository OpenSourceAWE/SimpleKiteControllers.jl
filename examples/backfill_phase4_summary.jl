# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
One-time backfill: add the phase-four `av_power_W`/`min_power_W`/`max_power_W`/
`min_force_N`/`av_force_N`/`max_force_N`/`v_ro_min_m_s`/`v_ro_av_m_s`/
`v_ro_max_m_s`/`av_depower` fields to the `summary:` block of every existing
`reelout_150m_opt.yaml` under `output/scenarios/*`, recomputed from that
scenario's own `reelout_150m_opt.arrow` log — the same fields
`examples/reelout_results.jl` now writes for every future run. Also drops the
`mean_N`/`peak_N` (`reelout.force`) and `mean_W`/`peak_W` (`reelout.power`)
fields those new ones supersede, matching that same edit.

Edits each YAML file in place, line by line, so every other key and comment
survives untouched. Run once, not part of any regular workflow:

    include("examples/backfill_phase4_summary.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using KiteUtils: load_log
using Statistics: mean

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "output", "scenarios"))
const LOG_NAME = "reelout_150m_opt"

"""
    phase4_stats(dir) -> NamedTuple or `nothing`

Reload `dir`'s `reelout_150m_opt.arrow` and recompute the phase-four
(`sys_state == 4`) power/force/reel-out-speed/depower stats exactly as
`examples/reelout_results.jl` does. `nothing` if the arrow log is missing or
phase four was never reached.
"""
function phase4_stats(dir)
    isfile(joinpath(dir, LOG_NAME * ".arrow")) || return nothing
    sl = load_log(LOG_NAME; path = dir).syslog
    fig8 = findall(x -> Int(x) == 4, sl.sys_state)
    isempty(fig8) && return nothing
    f4 = Float64.(getindex.(sl.winch_force, 1))[fig8]
    vro4 = Float64.(getindex.(sl.v_reelout, 1))[fig8]
    p4 = f4 .* vro4
    dp4 = Float64.(sl.depower[fig8])
    return (; av_power = mean(p4), min_power = minimum(p4), max_power = maximum(p4),
            av_force = mean(f4), min_force = minimum(f4), max_force = maximum(f4),
            v_ro_av = mean(vro4), v_ro_min = minimum(vro4), v_ro_max = maximum(vro4),
            av_depower = mean(dp4))
end

"""
    yaml_kv_line(key, value, comment; indent = 1, comment_col = 36)

One line in `write_yaml_commented`'s format (`examples/reelout_results.jl`):
`key: value` at `indent` levels (2 spaces each), with `comment` right-aligned
to `comment_col` where the line is short enough. Used here so a backfilled
line is indistinguishable from one a fresh run writes.
"""
function yaml_kv_line(key, value, comment; indent = 1, comment_col = 36)
    pad = "  "^indent
    prefix = string(pad, key, ": ", value)
    return string(prefix, " "^max(1, comment_col - length(prefix)), "# ", comment)
end

"""
    rewrite_yaml(yaml_file, stats)

Rewrite `yaml_file` line by line: drop the `mean_N`/`peak_N` lines from
`reelout.force` and `mean_W`/`peak_W` from `reelout.power`, reword the
`cf_force_ro`/`cf_power_ro` comments that referenced them, then append the ten
new `stats` fields to the end of the `summary:` block, right after
`optimizations_installed`. Every other line is copied through unchanged.
"""
function rewrite_yaml(yaml_file, stats)
    out = String[]
    for line in readlines(yaml_file)
        s = strip(line)
        (startswith(s, "mean_N:") || startswith(s, "peak_N:") ||
         startswith(s, "mean_W:") || startswith(s, "peak_W:")) && continue
        if startswith(s, "cf_force_ro:")
            line = replace(line, "crest factor: peak_N / mean_N" =>
                                  "crest factor: peak / mean tether force over the reeling window")
        elseif startswith(s, "cf_power_ro:")
            line = replace(line, "crest factor: peak_W / mean_W" =>
                                  "crest factor: peak / mean reel-out power over the reeling window")
        end
        push!(out, line)
        if startswith(s, "optimizations_installed:")
            push!(out, yaml_kv_line("av_power_W", round(Int, stats.av_power),
                                     "mean reel-out power over phase four [W]"))
            push!(out, yaml_kv_line("min_power_W", round(Int, stats.min_power),
                                     "min reel-out power over phase four [W]"))
            push!(out, yaml_kv_line("max_power_W", round(Int, stats.max_power),
                                     "max reel-out power over phase four [W]"))
            push!(out, yaml_kv_line("min_force_N", round(Int, stats.min_force),
                                     "min tether force over phase four [N]"))
            push!(out, yaml_kv_line("av_force_N", round(Int, stats.av_force),
                                     "mean tether force over phase four [N]"))
            push!(out, yaml_kv_line("max_force_N", round(Int, stats.max_force),
                                     "max tether force over phase four [N]"))
            push!(out, yaml_kv_line("v_ro_min_m_s", round(stats.v_ro_min; digits = 2),
                                     "min reel-out speed over phase four [m/s]"))
            push!(out, yaml_kv_line("v_ro_av_m_s", round(stats.v_ro_av; digits = 2),
                                     "mean reel-out speed over phase four [m/s]"))
            push!(out, yaml_kv_line("v_ro_max_m_s", round(stats.v_ro_max; digits = 2),
                                     "max reel-out speed over phase four [m/s]"))
            push!(out, yaml_kv_line("av_depower", round(stats.av_depower; digits = 3),
                                     "mean KCU depower over phase four [-]"))
        end
    end
    write(yaml_file, join(out, "\n") * "\n")
end

"""
    backfill_scenario(dir) -> Bool

Backfill one scenario folder's `reelout_150m_opt.yaml`. Returns `true` if the
file was rewritten, `false` (with a warning) if the arrow log, the YAML file,
or phase four itself is missing.
"""
function backfill_scenario(dir)
    yaml_file = joinpath(dir, LOG_NAME * ".yaml")
    if !isfile(yaml_file)
        @warn "Skipping $(basename(dir)): $LOG_NAME.yaml not found"
        return false
    end
    stats = phase4_stats(dir)
    if isnothing(stats)
        @warn "Skipping $(basename(dir)): no phase-four samples in $LOG_NAME.arrow"
        return false
    end
    rewrite_yaml(yaml_file, stats)
    @info "Backfilled $(basename(dir))/$LOG_NAME.yaml"
    return true
end

"""
    backfill_all()

Backfill every scenario folder directly under `output/scenarios`.
"""
function backfill_all()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")
    dirs = filter(isdir, readdir(SCENARIOS_DIR; join = true))
    n = count(backfill_scenario, dirs)
    @info "Backfilled $n of $(length(dirs)) scenario folders."
end

backfill_all()
