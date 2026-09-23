# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Export the results KaimonSlate notebook of the active project's site and publish it to the
[SimulationResults](https://github.com/OpenSourceAWE/SimulationResults) GitHub Pages site:
`results` (Maasvlakte) to `docs/index.html`, `results_cabauw` (Cabauw) to
`docs/cabauw/index.html`.

Runs `export_html.jl` (opening the notebook on the hub first if needed), copies the
result over that page in a SimulationResults checkout, then
commits and pushes it there. If no Slate server answers, it is started first with
`bin/run_slate start`. The checkout path defaults to `../../SimulationResults` next to
this package; override with the `SIMRESULTS_REPO` environment variable. Does nothing beyond
the export if `docs/index.html` comes out byte-identical to what is already committed.

Before exporting, checks the served notebook's `/api/<id>/state` for 0 cells, any non-fresh
cell after waiting up to 600 s for the run to finish, or a source file newer than what
the server last loaded, and refuses to publish if so —
a notebook server that was reopened without re-running still answers requests but exports a
near-empty stub page. Close and reopen the notebook, let it finish running, then publish again.

    include("publish.jl")
"""

using Dates
using Downloads

# `scenario_site`: the site of the active reel-out project.
include(joinpath(@__DIR__, "..", "examples", "gui_state.jl"))

SIMRESULTS_REPO = get(ENV, "SIMRESULTS_REPO",
                      normpath(joinpath(@__DIR__, "..", "..", "SimulationResults")))

isdir(joinpath(SIMRESULTS_REPO, ".git")) ||
    error("No SimulationResults checkout at $SIMRESULTS_REPO — clone it there, or set " *
          "SIMRESULTS_REPO to point at an existing one.")

# Each site has its own notebook and page; the 3D-path pages sit next to it, and their file
# names repeat between sites, so Cabauw needs a folder of its own.
SITE = scenario_site()
notebook = get(ENV, "SLATE_NOTEBOOK", SITE == "cabauw" ? "results_cabauw" : "results")
PAGE_DIR = SITE == "cabauw" ? joinpath(SIMRESULTS_REPO, "docs", "cabauw") : joinpath(SIMRESULTS_REPO, "docs")
INDEX_HTML = joinpath(PAGE_DIR, "index.html")
println("Publishing the $SITE results (notebook `$notebook`) to $INDEX_HTML")
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

cell_states_of(state) = [m.captures[1] for m in eachmatch(r"\"state\":\"(\w+)\"", state)]

# `/run` returns before the cells have finished, so wait for them (up to 600 s).
state = nothing
for _ in 1:600
    global state = fetch_body(state_url)
    state !== nothing && all(==("fresh"), cell_states_of(state)) &&
        !isempty(cell_states_of(state)) && break
    sleep(1)
end
state === nothing &&
    error("Could not reach the `$notebook` notebook at $state_url after opening it.")

cell_states = cell_states_of(state)
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

# Set explicitly: export_html.jl keeps any of these already defined in Main, e.g. from
# publishing the other site earlier in this session.
NOTEBOOK = notebook
HUB_URL = hub_url
EXPORT_OUTPUT_PATH = nothing
include(joinpath(@__DIR__, "export_html.jl"))   # produces EXPORT_OUTPUT_PATH

git(args) = Cmd(`git $args`; dir = SIMRESULTS_REPO)

mkpath(PAGE_DIR)
cp(EXPORT_OUTPUT_PATH, INDEX_HTML; force = true)
# The 3D-path pages are not inlined into the export (the `path3d_plot` cell loads
# them into an iframe by bare file name), so they have to sit next to index.html.
page_files = [INDEX_HTML; [joinpath(PAGE_DIR, basename(f)) for f in PATH3D_FILES]]
for f in PATH3D_FILES
    cp(f, joinpath(PAGE_DIR, basename(f)); force = true)
end

if isempty(read(git(`status --porcelain -- docs/`), String))
    println("Nothing to publish — docs/ already matches the export.")
else
    run(git(`add $([relpath(f, SIMRESULTS_REPO) for f in page_files])`))
    src_hash = strip(read(`git -C $(@__DIR__) rev-parse --short HEAD`, String))
    src_dirty = !isempty(read(`git -C $(@__DIR__) status --porcelain`, String))
    message = "Update exported $SITE results notebook\n\n" *
              "SimpleKiteControllers @ $src_hash$(src_dirty ? "-dirty" : ""), " *
              "$(Dates.format(now(), "yyyy-mm-dd HH:MM"))"
    run(git(`commit -m $message`))
    run(git(`push`))
    println("Published to SimulationResults (", strip(read(git(`rev-parse --short HEAD`), String)), ").")
end
