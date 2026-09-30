# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Export the running `results` KaimonSlate notebook to an HTML file in `output/`.

Hits the same `GET /api/<id>/export.html` route as the notebook's own **☰ → Export HTML** menu
entry, so it needs the notebook already open and served — start it first (Kaimon Slate, or
`KaimonSlate.serve_notebook("notebooks/results.jl")`). The notebook id and hub URL default to
`results` and `http://127.0.0.1:8765`; override with the `SLATE_NOTEBOOK`/`SLATE_HUB_URL`
environment variables, or with the inputs `notebook`/`hub_url` of
`run_example(joinpath("..", "notebooks", "export_html.jl"); ...)` (`src/script_inputs.jl`),
as `publish.jl` does.

The export inlines every image, but not the interactive 3D-path pages
(`notebooks/images/<site>/path_webgl_*.html`): the notebook's `path3d_plot` cell loads them
into an iframe by bare file name, so they are copied next to the export here, and `publish.jl`
copies them next to `index.html` (their list is left in `PATH3D_FILES` for it). That keeps the
page itself at ~8 MB instead of ~50, and only the selected wind speed's WebGL page ever loads.
`SITE` names the site folder the notebook's images live in: `cabauw` for the `results_cabauw`
notebook, `maasvlakte` (shown by `results`) otherwise; override with `SLATE_SITE` or the input
`site`. The export goes to `output/export_<site>/` (the input `export_path` replaces the file),
since both sites' 3D-path pages share the same file names; `EXPORT_OUTPUT_PATH` names it after
the run.
"""

using Downloads

using SimpleKiteControllers: run_example, script_inputs

# The caller's inputs, see the docstring; a plain `include` takes the environment's. Plain globals
# instead of consts so this file can be included repeatedly into Main without "already declared"
# constant errors.
inputs = script_inputs(@__FILE__, (; notebook = get(ENV, "SLATE_NOTEBOOK", "results"),
                                    hub_url = get(ENV, "SLATE_HUB_URL", "http://127.0.0.1:8765"),
                                    site = nothing, export_path = nothing))
NOTEBOOK = inputs.notebook
HUB_URL = inputs.hub_url
SITE = something(inputs.site,
                 get(ENV, "SLATE_SITE", NOTEBOOK == "results_cabauw" ? "cabauw" : "maasvlakte"))
# One folder per site: both sites' 3D-path pages share the same file names.
EXPORT_OUTPUT_PATH = something(inputs.export_path,
                               joinpath(@__DIR__, "..", "output", "export_$SITE",
                                        "$(NOTEBOOK)_export.html"))

url = "$HUB_URL/api/$NOTEBOOK/export.html?dl=1"
mkpath(dirname(EXPORT_OUTPUT_PATH))
try
    Downloads.download(url, EXPORT_OUTPUT_PATH)
catch e
    error("Could not reach the `$NOTEBOOK` notebook at $url — open it in Kaimon Slate first " *
          "(or set SLATE_NOTEBOOK/SLATE_HUB_URL). Underlying error: $e")
end
println("Exported to $EXPORT_OUTPUT_PATH ($(filesize(EXPORT_OUTPUT_PATH)) bytes)")

PATH3D_DIR = joinpath(@__DIR__, "images", SITE)
PATH3D_FILES = isdir(PATH3D_DIR) ? filter(readdir(PATH3D_DIR; join = true)) do f
    startswith(basename(f), "path_webgl_") && endswith(f, ".html")
end : String[]
isempty(PATH3D_FILES) &&
    @warn "No notebooks/images/$SITE/path_webgl_*.html found — run examples/create_plots.jl with the $SITE project selected first, or the 3D-path cell stays empty."
for f in PATH3D_FILES
    cp(f, joinpath(dirname(EXPORT_OUTPUT_PATH), basename(f)); force = true)
end
