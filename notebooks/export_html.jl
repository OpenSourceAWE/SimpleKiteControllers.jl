# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Export the running `results` KaimonSlate notebook to a self-contained HTML file in `output/`.

Hits the same `GET /api/<id>/export.html` route as the notebook's own **☰ → Export HTML** menu
entry, so it needs the notebook already open and served — start it first (Kaimon Slate, or
`KaimonSlate.serve_notebook("notebooks/results.jl")`). The notebook id and hub URL default to
`results` and `http://127.0.0.1:8765`; override with the `SLATE_NOTEBOOK`/`SLATE_HUB_URL`
environment variables.
"""

using Downloads

# Plain globals instead of consts so this file can be included repeatedly into
# Main (e.g. by publish.jl after a direct include) without "already declared"
# constant errors. EXPORT_OUTPUT_PATH is deliberately its own name, distinct from
# OUTPUT_PATH: simple_reelout.jl and simple_opt_reelout.jl read-and-clear
# OUTPUT_PATH as a sweep-output-directory override, and this script's export
# file path left behind under that same name was picked up by the next
# simple_opt_reelout.jl run as its output directory and crashed mkpath().
isdefined(Main, :NOTEBOOK) && NOTEBOOK !== nothing ||
    (NOTEBOOK = get(ENV, "SLATE_NOTEBOOK", "results"))
isdefined(Main, :HUB_URL) && HUB_URL !== nothing ||
    (HUB_URL = get(ENV, "SLATE_HUB_URL", "http://127.0.0.1:8765"))
isdefined(Main, :EXPORT_OUTPUT_PATH) && EXPORT_OUTPUT_PATH !== nothing ||
    (EXPORT_OUTPUT_PATH = joinpath(@__DIR__, "..", "output", "$(NOTEBOOK)_export.html"))

url = "$HUB_URL/api/$NOTEBOOK/export.html?dl=1"
mkpath(dirname(EXPORT_OUTPUT_PATH))
try
    Downloads.download(url, EXPORT_OUTPUT_PATH)
catch e
    error("Could not reach the `$NOTEBOOK` notebook at $url — open it in Kaimon Slate first " *
          "(or set SLATE_NOTEBOOK/SLATE_HUB_URL). Underlying error: $e")
end
println("Exported to $EXPORT_OUTPUT_PATH ($(filesize(EXPORT_OUTPUT_PATH)) bytes)")
