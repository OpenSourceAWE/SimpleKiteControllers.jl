# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Export the running `results` KaimonSlate notebook and publish it to the
[SimulationResults](https://github.com/OpenSourceAWE/SimulationResults) GitHub Pages site.

Runs `export_html.jl` (needs the `results` notebook already open and served — see its own
docstring), copies the result over `docs/index.html` in a SimulationResults checkout, then
commits and pushes it there. The checkout path defaults to `../../SimulationResults` next to
this package; override with the `SIMRESULTS_REPO` environment variable. Does nothing beyond
the export if `docs/index.html` comes out byte-identical to what is already committed.

    include("publish.jl")
"""

using Dates

const SIMRESULTS_REPO = get(ENV, "SIMRESULTS_REPO",
                             normpath(joinpath(@__DIR__, "..", "..", "SimulationResults")))
const INDEX_HTML = joinpath(SIMRESULTS_REPO, "docs", "index.html")

isdir(joinpath(SIMRESULTS_REPO, ".git")) ||
    error("No SimulationResults checkout at $SIMRESULTS_REPO — clone it there, or set " *
          "SIMRESULTS_REPO to point at an existing one.")

include(joinpath(@__DIR__, "export_html.jl"))   # produces OUTPUT_PATH

git(args) = Cmd(`git $args`; dir = SIMRESULTS_REPO)

cp(OUTPUT_PATH, INDEX_HTML; force = true)

if isempty(read(git(`status --porcelain -- docs/index.html`), String))
    println("Nothing to publish — docs/index.html already matches the export.")
else
    run(git(`add docs/index.html`))
    src_hash = strip(read(`git -C $(@__DIR__) rev-parse --short HEAD`, String))
    src_dirty = !isempty(read(`git -C $(@__DIR__) status --porcelain`, String))
    message = "Update exported results notebook\n\n" *
              "SimpleKiteControllers @ $src_hash$(src_dirty ? "-dirty" : ""), " *
              "$(Dates.format(now(), "yyyy-mm-dd HH:MM"))"
    run(git(`commit -m $message`))
    run(git(`push`))
    println("Published to SimulationResults (", strip(read(git(`rev-parse --short HEAD`), String)), ").")
end
