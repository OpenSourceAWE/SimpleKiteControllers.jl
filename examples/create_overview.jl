# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Write `SimulationResults/scenarios/overview.md`, a table with one row per
`vNN` scenario folder (`v03`…`v10`, excluding `v08_2` — same wind speed as
`v08`), read from each folder's own `reelout_150m_opt.yaml` `summary:` block.

Columns: `date`, `time`, `v_wind`, `power_ratio`, `total_time`,
`rt_factor`, `opt_requests`, `opts_installed`, `av_power`, `min_power`,
`max_power`, `min_force`, `av_force`, `max_force`, `v_ro_min`, `v_ro_av`,
`v_ro_max`, `av_depower`, `rms_error`, `success_criteria` — read from the YAML `summary:`
section's
`date`, `time`, `wind_speed_gnd`, `power_ratio`,
`total_wall_time_s`, `realtime_factor`, `optimization_requests`,
`optimizations_installed`, `av_power_ro`, `min_power_ro`, `max_power_ro`,
`min_force_ro`, `av_force_ro`, `max_force_ro`, `v_ro_min`, `v_ro_av`,
`v_ro_max`, `av_depower_ro`, `cross_track_rms_deg` and `success_criteria` respectively (see
`examples/reelout_results.jl`). `power_ratio` is read from
`power_ratio_free_speed` instead when that key is present. All
`power`/`force`/`v_ro`/`depower` fields
are averaged over phase four (the figure-eight flying phase). Rows are
sorted by wind speed ascending. A folder missing the YAML file or its
`summary:` block is skipped with a warning, not an error; a column missing
from an otherwise present `summary:` block (older runs predate
`success_criteria`) renders as an empty cell instead of erroring.

Also opens the same table as a nicely formatted HTML pop-up in the default
browser, via PrettyTables' `:html` backend (`xdg-open`), with the power
curve (mean/max/min reel-out power, tether force and reel-out speed over
phase four vs. wind speed, stacked as in `plot_powercurve.jl`) embedded
below it as a vector PDF (`<embed>`, base64-encoded) so it stays sharp at
any zoom.

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
using SimpleKiteControllers: skc_data_path

const SCENARIOS_DIR = normpath(joinpath(@__DIR__, "..", "..", "SimulationResults", "scenarios"))
const SUMMARY_FILE = "reelout_150m_opt.yaml"
const SKIPPED_FOLDERS = ("v08_2",)
const MAX_TETHER_FORCE_N =
    YAML.load_file(joinpath(skc_data_path(), "settings_reelout_150m.yaml"))["winch"]["max_force"]
# yaml key => displayed column header
const COLUMNS = ("date" => "date", "time" => "time", "wind_speed_gnd" => "v_wind",
                 "power_ratio" => "power_ratio",
                 "total_wall_time_s" => "total_time", "realtime_factor" => "rt_factor",
                 "optimization_requests" => "opt_requests",
                 "optimizations_installed" => "opts_installed",
                 "av_power_ro" => "av_power", "min_power_ro" => "min_power",
                 "max_power_ro" => "max_power", "min_force_ro" => "min_force",
                 "av_force_ro" => "av_force", "max_force_ro" => "max_force",
                 "v_ro_min" => "v_ro_min", "v_ro_av" => "v_ro_av",
                 "v_ro_max" => "v_ro_max", "av_depower_ro" => "av_depower",
                 "cross_track_rms_deg" => "rms_error",
                 "success_criteria" => "success_criteria")

"""
    scenario_summary(dir) -> Union{Dict, Nothing}

The `summary:` block of `dir`'s `reelout_150m_opt.yaml`, or `nothing` (with a
warning) if the file or the block is missing. If the block has a
`power_ratio_free_speed` entry, it replaces `power_ratio` under that same
key, so the displayed `power_ratio` column shows it without a column rename.
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
    summary = y["summary"]
    if haskey(summary, "power_ratio_free_speed")
        summary["power_ratio"] = summary["power_ratio_free_speed"]
    end
    return summary
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
            println(io, "| " * join((get(row, key, "") for (key, _) in COLUMNS), " | ") * " |")
        end
    end
end

"""
    powercurve_images(rows) -> (png_file, pdf_file)

Plot mean, max and min reel-out power, tether force and reel-out speed over
phase four against wind speed [m/s] for `rows` (already sorted by wind speed)
with MakieControlPlots, the same stacked plot as `plot_powercurve.jl` (max
dashed grey and min dash-dotted grey in every panel, plus the force panel's
`MAX_TETHER_FORCE_N` dotted black reference line), and save it as both a
temporary PNG file (for the `notebooks/` markdown embed) and a temporary PDF
file (vector, for the HTML pop-up), returning both paths.
"""
function powercurve_images(rows)
    v_wind = [row["wind_speed_gnd"] for row in rows]
    p_kw = [row["av_power_ro"] / 1000 for row in rows]
    p_max_kw = [row["max_power_ro"] / 1000 for row in rows]
    p_min_kw = [row["min_power_ro"] / 1000 for row in rows]
    f_kn = [row["av_force_ro"] / 1000 for row in rows]
    f_max_kn = [row["max_force_ro"] / 1000 for row in rows]
    f_min_kn = [row["min_force_ro"] / 1000 for row in rows]
    v_ro = [row["v_ro_av"] for row in rows]
    v_ro_max = [row["v_ro_max"] for row in rows]
    v_ro_min = [row["v_ro_min"] for row in rows]
    f_limit_kn = fill(MAX_TETHER_FORCE_N / 1000, length(v_wind))

    plotx(v_wind, [p_kw, p_max_kw, p_min_kw], [f_kn, f_max_kn, f_min_kn, f_limit_kn],
          [v_ro, v_ro_max, v_ro_min];
          xlabel = "wind speed [m/s]",
          ylabels = ["power [kW]", "force [kN]", "speed [m/s]"],
          labels = [["mean", "max", "min"], ["mean", "max", "min", "limit"], ["mean", "max", "min"]],
          linestyle = [[nothing, :dash, :dashdot], [nothing, :dash, :dashdot, :dot],
                       [nothing, :dash, :dashdot]],
          color = [[nothing, :grey, :grey], [nothing, :grey, :grey, :black],
                   [nothing, :grey, :grey]],
          legend_position = [:auto, :lt, :auto],
          title = "V3 reel-out power curve", scatter = true, disp = true,
          fig = "powercurve")
    png_file = joinpath(tempdir(), "powercurve.png")
    pdf_file = joinpath(tempdir(), "powercurve.pdf")
    savefig(png_file)
    savefig(pdf_file)
    MakieControlPlots.close("powercurve")
    return png_file, pdf_file
end

"""
    show_overview_html(rows, pdf_file)

Render `rows` as an HTML table with PrettyTables, with the power curve
`pdf_file` (from `powercurve_images`) embedded below it as a base64-encoded
PDF via `<embed>` (kept vector, unlike a rasterized `<img>`), opened at 75%
zoom via the `#zoom=75` fragment, and open it in the default browser, giving
a nicely formatted pop-up view of the same data as the markdown table.
"""
function show_overview_html(rows, pdf_file)
    headers = collect(last.(COLUMNS))
    data = [get(row, key, "") for row in rows, (key, _) in COLUMNS]
    table_html = pretty_table(String, data; column_labels = headers, backend = :html, alignment = :c)
    pdf_b64 = base64encode(read(pdf_file))
    # a snap-confined browser (e.g. Firefox) cannot see dotfiles/dotdirs under $HOME
    html_file = joinpath(homedir(), "overview.html")
    open(html_file, "w") do io
        println(io, "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"></head><body>")
        println(io, table_html)
        println(io, "<div style=\"text-align:center;\">")
        println(io, "<embed src=\"data:application/pdf;base64,$pdf_b64#zoom=75\" type=\"application/pdf\" " *
                     "width=\"100%\" height=\"900px\">")
        println(io, "</div>")
        println(io, "</body></html>")
    end
    run(`xdg-open $html_file`; wait = false)
end

"""
    create_overview()

Scan `SimulationResults/scenarios/` and write `overview.md` there, one row
per scenario ordered by wind speed, mirror `overview.md` into `notebooks/`
and the power curve PNG into `notebooks/images/` for `notebooks/results.jl`
to embed, then display the same table as a pop-up in the default browser.
"""
function create_overview()
    isdir(SCENARIOS_DIR) || error("$SCENARIOS_DIR does not exist.")
    dirs = filter(readdir(SCENARIOS_DIR; join = true)) do dir
        isdir(dir) && basename(dir) ∉ SKIPPED_FOLDERS
    end
    summaries = filter(!isnothing, scenario_summary.(dirs))
    isempty(summaries) && error("No usable $SUMMARY_FILE summary: blocks found in $SCENARIOS_DIR")

    rows = sort(summaries; by = row -> row["wind_speed_gnd"])
    md_file = joinpath(SCENARIOS_DIR, "overview.md")
    write_overview_md(rows, md_file)
    @info "Wrote overview" md_file rows=length(rows)

    notebooks_dir = joinpath(@__DIR__, "..", "notebooks")
    mkpath(notebooks_dir)
    notebooks_file = joinpath(notebooks_dir, "overview.md")
    write_overview_md(rows, notebooks_file)
    @info "Wrote overview" notebooks_file rows=length(rows)

    png_file, pdf_file = powercurve_images(rows)
    notebooks_images_dir = joinpath(notebooks_dir, "images")
    mkpath(notebooks_images_dir)
    notebooks_png_file = joinpath(notebooks_images_dir, "powercurve.png")
    cp(png_file, notebooks_png_file; force = true)
    @info "Wrote power curve" notebooks_png_file

    show_overview_html(rows, pdf_file)
end

create_overview()
