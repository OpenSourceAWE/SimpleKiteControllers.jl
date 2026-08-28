# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Export the running `results` KaimonSlate notebook and publish it to the
[SimulationResults](https://github.com/OpenSourceAWE/SimulationResults) GitHub Pages site.

Runs `export_html.jl` (needs the `results` notebook already open and served — see its own
docstring), copies the result over `docs/index.html` in a SimulationResults checkout, then
commits and pushes it there. If no Slate server answers, it is started first with
`bin/run_slate start`. The checkout path defaults to `../../SimulationResults` next to
this package; override with the `SIMRESULTS_REPO` environment variable. Does nothing beyond
the export if `docs/index.html` comes out byte-identical to what is already committed.

Before exporting, checks the served notebook's `/api/<id>/state` for 0 cells, any non-fresh
cell, or a source file newer than what the server last loaded, and refuses to publish if so —
a notebook server that was reopened without re-running still answers requests but exports a
near-empty stub page. Close and reopen the notebook, let it finish running, then publish again.

    include("publish.jl")
"""

using Dates
using Downloads

const SIMRESULTS_REPO = get(ENV, "SIMRESULTS_REPO",
                             normpath(joinpath(@__DIR__, "..", "..", "SimulationResults")))
const INDEX_HTML = joinpath(SIMRESULTS_REPO, "docs", "index.html")

isdir(joinpath(SIMRESULTS_REPO, ".git")) ||
    error("No SimulationResults checkout at $SIMRESULTS_REPO — clone it there, or set " *
          "SIMRESULTS_REPO to point at an existing one.")

notebook = get(ENV, "SLATE_NOTEBOOK", "results")
hub_url = get(ENV, "SLATE_HUB_URL", "http://127.0.0.1:8765")
state_url = "$hub_url/api/$notebook/state"
open_url = "$hub_url/api/open"

fetch_body(url) = try
    io = IOBuffer()
    Downloads.download(url, io)
    String(take!(io))
catch
    nothing
end

post_json(url, body) = try
    io = IOBuffer()
    Downloads.request(url; method = "POST",
                      headers = Dict("Content-Type" => "application/json"),
                      input = IOBuffer(body), output = io)
    String(take!(io))
catch e
    error("POST $url failed: $e")
end

notebook_path = normpath(joinpath(@__DIR__, "$notebook.jl"))

# Start the Slate server via bin/run_slate if it is not reachable yet.
reachable = fetch_body(state_url) !== nothing || fetch_body("$hub_url/api/version") !== nothing
if !reachable
    run_slate = normpath(joinpath(@__DIR__, "..", "bin", "run_slate"))
    isfile(run_slate) || error("Slate not reachable at $hub_url and no $run_slate found.")
    println("Slate not reachable — starting it with bin/run_slate ...")
    run(`$run_slate start`)
    # Wait until the hub answers (up to ~120 s; first start compiles KaimonSlate).
    for _ in 1:120
        sleep(1)
        if fetch_body("$hub_url/api/version") !== nothing
            global reachable = true
            break
        end
    end
end
reachable ||
    error("Started Slate but the hub is still not reachable at $hub_url.")

# The hub may be up without this notebook open (e.g. the Kaimon slate extension's
# shared hub) — open it via the API instead of asking the user to do it.
if fetch_body(state_url) === nothing
    println("Notebook `$notebook` not open on the hub — opening $(notebook_path) ...")
    reply = post_json(open_url, "{\"path\":$(repr(notebook_path))}")
    occursin("\"id\"", reply) ||
        error("Could not open `$notebook` on the hub at $hub_url — reply: $reply")
end

# Run all cells so every cell reports fresh (publish refuses stale notebooks).
println("Running all cells of `$notebook` ...")
post_json("$hub_url/api/$notebook/run", "")

state = fetch_body(state_url)
state === nothing &&
    error("Could not reach the `$notebook` notebook at $state_url after opening it.")

cell_states = [m.captures[1] for m in eachmatch(r"\"state\":\"(\w+)\"", state)]
isempty(cell_states) &&
    error("Notebook `$notebook` reports 0 cells — it was likely reopened without reloading " *
          "$notebook.jl. Close and reopen it, let it run, then publish again.")
stale = filter(!=("fresh"), cell_states)
isempty(stale) ||
    error("Notebook `$notebook` has $(length(stale)) non-fresh cell(s) " *
          "($(join(sort(unique(stale)), ", "))) — run all cells before publishing.")
occursin("\"src_stale\":true", state) &&
    error("Notebook `$notebook` is stale relative to $notebook.jl on disk — close and " *
          "reopen it so it re-parses the file, then run it before publishing.")

include(joinpath(@__DIR__, "export_html.jl"))   # produces EXPORT_OUTPUT_PATH

git(args) = Cmd(`git $args`; dir = SIMRESULTS_REPO)

cp(EXPORT_OUTPUT_PATH, INDEX_HTML; force = true)

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
