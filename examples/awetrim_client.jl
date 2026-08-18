# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Julia client for the AWETrim reelout flight-path optimizer (REST). `include`
# it; it defines no globals of its own beyond the functions below.
#
# This file currently carries the PROCESS side only — making sure a server is
# there to talk to. The request/reply structs and the /init, /step, /status
# calls still live in examples/optimize_path.jl and move here next
# (PlanOptFig8.md, step 2).
#
#     include("awetrim_client.jl")
#     ensure_server()      # start one if none answers
#     stop_server()        # ... and stop it again

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using HTTP

const SKC_ROOT = normpath(joinpath(@__DIR__, ".."))

"""
    server_running(url; timeout = 2) -> Bool

`true` if an AWETrim server answers `GET \$url/health`. Not a liveness check on a
process: a foreground server, one started by `bin/run_server start` and one on
another machine are all equally usable, and all three answer this.
"""
function server_running(url::AbstractString; timeout = 2)
    try
        response = HTTP.get(url * "/health"; connect_timeout = timeout,
                            request_timeout = timeout, retry = false,
                            status_exception = false)
        return response.status == 200
    catch
        return false
    end
end

"""
    ensure_server(url = "http://127.0.0.1:8000"; autostart = true, verbose = true)

Return once an AWETrim server answers at `url`, starting a detached one with
`bin/run_server start` if none does. Throws if the server cannot be started or
does not come up — `bin/run_server` blocks until `/health` answers and exits
non-zero otherwise, so the waiting and the diagnosis are its job, not this one's.

`autostart = false` turns a missing server into an error instead, which is what a
run against a server on another machine wants.
"""
function ensure_server(url::AbstractString = "http://127.0.0.1:8000";
                       autostart = true, verbose = true)
    server_running(url) && return url
    autostart || error("No AWETrim server at $url, and autostart is off. Start one \
                        with `bin/run_server start`.")
    m = match(r"^https?://([^:/]+)(?::(\d+))?", url)
    isnothing(m) && error("Cannot parse a host and port out of $url.")
    host = m[1]
    port = isnothing(m[2]) ? "8000" : m[2]
    verbose && @info "No AWETrim server at $url — starting one (bin/run_server start)."
    script = joinpath(SKC_ROOT, "bin", "run_server")
    run(Cmd(`$script start --host $host --port $port`; dir = SKC_ROOT))
    server_running(url) ||
        error("bin/run_server reported success, but $url/health still does not answer.")
    return url
end

"""
    stop_server(url = "http://127.0.0.1:8000"; verbose = true) -> Bool

Stop the detached server, through `bin/run_server stop`. Returns `true` once
nothing answers at `url` any more.

Only reaches a server that `bin/run_server start` (or [`ensure_server`](@ref))
launched, since that is the one whose pid it recorded: a server started in the
foreground belongs to the terminal that runs it and is stopped there with Ctrl-C,
and one on another machine is not this script's to stop. Both cases leave a
server answering at `url`, which is what the `false` return reports.
"""
function stop_server(url::AbstractString = "http://127.0.0.1:8000"; verbose = true)
    script = joinpath(SKC_ROOT, "bin", "run_server")
    run(Cmd(`$script stop`; dir = SKC_ROOT))
    stopped = !server_running(url)
    if !stopped && verbose
        @warn "Something still answers at $url — a server started in the foreground, \
               or one this machine did not start."
    end
    return stopped
end
