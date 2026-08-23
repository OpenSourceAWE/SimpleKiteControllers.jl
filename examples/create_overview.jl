# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Write `SimulationResults/scenarios/overview.md`, a table with one row per
`vNN` scenario folder (`v03`…`v10`, excluding `v08_2` — same wind speed as
`v08`), read from each folder's own `reelout_150m_opt.yaml` `summary:` block.

Columns: `date`, `time`, `v_wind`, `power`, `power_ratio`, `total_time`,
`rt_factor`, `opt_requests`, `opts_installed`, `av_power`, `min_power`,
`max_power`, `min_force`, `av_force`, `max_force`, `v_ro_min`, `v_ro_av`,
`v_ro_max`, `av_depower` — read from the YAML `summary:` section's
`date`, `time`, `wind_speed_m_s`, `mean_power_W`, `power_ratio`,
`total_wall_time_s`, `realtime_factor`, `optimization_requests`,
`optimizations_installed`, `av_power_W`, `min_power_W`, `max_power_W`,
`min_force_N`, `av_force_N`, `max_force_N`, `v_ro_min_m_s`, `v_ro_av_m_s`,
`v_ro_max_m_s` and `av_depower` respectively (see
`examples/reelout_results.jl`). The `av_`/`min_`/`max_` fields are all
averaged over phase four (the figure-eight flying phase), distinct from
`power`/`mean_power_W`, which is scored over the reeling window. Rows are
sorted by wind speed ascending. A folder missing the YAML file or its
`summary:` block is skipped with a warning, not an error.

Also opens the same table as a nicely formatted HTML pop-up in the default
browser, via PrettyTables' `:html` backend (`xdg-open`), with the power
curve (mean reel-out power vs. wind speed, as in `plot_powercurve.jl`)
embedded below it.

Run from the menu, or by

    include("examples/create_overview.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using YAML
using PrettyTables
using MakieControlPlots
using Base64

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "..", "SimulationResults", "scenarios"))
const SUMMARY_FILE = "reelout_150m_opt.yaml"
const SKIPPED_FOLDERS = ("v08_2",)
# yaml key => displayed column header
const COLUMNS = ("date" => "date", "time" => "time", "wind_speed_m_s" => "v_wind",
                 "mean_power_W" => "power", "power_ratio" => "power_ratio",
                 "total_wall_time_s" => "total_time", "realtime_factor" => "rt_factor",
                 "optimization_requests" => "opt_requests",
                 "optimizations_installed" => "opts_installed",
                 "av_power_W" => "av_power", "min_power_W" => "min_power",
                 "max_power_W" => "max_power", "min_force_N" => "min_force",
                 "av_force_N" => "av_force", "max_force_N" => "max_force",
                 "v_ro_min_m_s" => "v_ro_min", "v_ro_av_m_s" => "v_ro_av",
                 "v_ro_max_m_s" => "v_ro_max", "av_depower" => "av_depower")

"""
    scenario_summary(dir) -> Union{Dict, Nothing}

The `summary:` block of `dir`'s `reelout_150m_opt.yaml`, or `nothing` (with a
warning) if the file or the block is missing.
"""
function scenario_summary(dir::AbstractString)
    yaml_file = joinpath(dir, SUMMARY_FILE)
    if !isfile(yaml_file)
        @warn "Skipping $(basename(dir)): $SUMMARY_FILE not found"
        return nothing
    end
    y = YAML.load_file(yaml_file)
    if !haskey(y, "summary")
        @warn "Skipping $(basename(dir)): no summary: block in $SUMMARY_FILE"
        return nothing
    end
    return y["summary"]
end

"""
    write_overview_md(rows, md_file)

Write `rows` (each a `summary:` `Dict`) as a Markdown table to `md_file`,
with every column centered.
"""
function write_overview_md(rows, md_file)
    headers = last.(COLUMNS)
    open(md_file, "w") do io
        println(io, "| " * join(headers, " | ") * " |")
        println(io, "| " * join(fill(":---:", length(headers)), " | ") * " |")
        for row in rows
            println(io, "| " * join((row[key] for (key, _) in COLUMNS), " | ") * " |")
        end
    end
end

"""
    powercurve_png(rows) -> String

Plot mean reel-out power [kW] against wind speed [m/s] for `rows` (already
sorted by wind speed) with MakieControlPlots, the same curve as
`plot_powercurve.jl`, and save it as a temporary PNG file, returning its path.
"""
function powercurve_png(rows)
    v_wind = [row["wind_speed_m_s"] for row in rows]
    p_kw = [row["mean_power_W"] / 1000 for row in rows]
    plotxy(v_wind, p_kw; xlabel = "wind speed [m/s]", ylabel = "mean reel-out power [kW]",
           title = "V3 reel-out power curve", scatter = true, disp = true, fig = "powercurve")
    png_file = joinpath(tempdir(), "powercurve.png")
    savefig(png_file)
    MakieControlPlots.close("powercurve")
    return png_file
end

"""
    show_overview_html(rows)

Render `rows` as an HTML table with PrettyTables, with the power curve
embedded below it as a base64-encoded PNG, and open it in the default
browser, giving a nicely formatted pop-up view of the same data as the
markdown table.
"""
function show_overview_html(rows)
    headers = collect(last.(COLUMNS))
    data = [row[key] for row in rows, (key, _) in COLUMNS]
    table_html = pretty_table(String, data; column_labels = headers, backend = :html, alignment = :c)
    img_b64 = base64encode(read(powercurve_png(rows)))
    # a snap-confined browser (e.g. Firefox) cannot see dotfiles/dotdirs under $HOME
    html_file = joinpath(homedir(), "overview.html")
    open(html_file, "w") do io
        println(io, "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"></head><body>")
        println(io, table_html)
        println(io, "<img src=\"data:image/png;base64,$img_b64\" alt=\"power curve\">")
        println(io, "</body></html>")
    end
    run(`xdg-open $html_file`; wait = false)
end

"""
    create_overview()

Scan `SimulationResults/scenarios/` and write `overview.md` there, one row
per scenario ordered by wind speed, then display the same table as a
pop-up in the default browser.
"""
function create_overview()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")
    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && basename(dir) ∉ SKIPPED_FOLDERS
    end
    summaries = filter(!isnothing, scenario_summary.(dirs))
    isempty(summaries) && error("No usable $SUMMARY_FILE summary: blocks found in $SCENARIOS_DIR")

    rows = sort(summaries; by = row -> row["wind_speed_m_s"])
    md_file = joinpath(SCENARIOS_DIR, "overview.md")
    write_overview_md(rows, md_file)
    @info "Wrote overview" md_file rows=length(rows)
    show_overview_html(rows)
end

create_overview()
