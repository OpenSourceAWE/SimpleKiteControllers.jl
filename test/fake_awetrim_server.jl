# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# A fake AWETrim server for the unit tests (test_awetrim_server.jl, test_reopt_chain.jl): it
# answers `/health`, `/init`, `/step`, `/status` and `/trajectory` at once, on a free local port,
# and records every request. It knows nothing of the physics.

import SimpleKiteControllers: HTTP, JSON3

const FAKE_AZ = [0.0, 10.0, 0.0, -10.0]
const FAKE_EL = [25.0, 30.0, 25.0, 20.0]

json(status, d) = HTTP.Response(status, ["Content-Type" => "application/json"], JSON3.write(d))
step_reply(length, power) = Dict(
    "length" => length, "trajectory" => Dict("azimuth" => FAKE_AZ, "elevation" => FAKE_EL .+ 1),
    "state" => "converged", "step_index" => 1,
    "metrics" => Dict("energy_J" => 1e5, "total_time_s" => 100.0, "avg_power_W" => power))
# The `/trajectory` table, in RADIANS as the server serves it.
fake_table(power) = Dict(
    "table" => Dict("azimuth" => deg2rad.(FAKE_AZ), "elevation" => deg2rad.(FAKE_EL .+ 1),
                    "distance_radial" => [150.0, 165.0]),
    "spline" => Dict("downloops" => true), "metrics" => Dict("avg_power_W" => power),
    "optimized_parameters" => Dict("input_depower" => 1.42))

"""
    fake_server(; fail, fail_warm, detail, validation, instant, table) -> (; server, url, log, state)

A fake server: `log` holds `(path, body)` of every request and `state` is what `/status` reports.
A `/step` whose length is in `fail`, or that is warm (no trajectory) under `fail_warm`, fails:
blocking with a 422 carrying `detail`, non-blocking as a `"failed"` status; `validation` answers
every `/step` with a validation 422. A non-blocking step reports `"solving"` until the test sets
`state`, or `"converged"` at once under `instant`. The power grows by 1000 W with every converged
step, so replies can be told apart; `/trajectory` serves `table(power)`.
"""
function fake_server(; fail = Float64[], fail_warm = false,
                     detail = "optimization did not converge", validation = false,
                     instant = false, table = fake_table)
    log = Tuple{String, Any}[]
    state = Ref("ready")
    power = Ref(0.0)
    function handle(req)
        path = first(split(req.target, '?'))
        # HTTP.jl 2 wraps the bytes of a POST in a `BytesBody`; a GET has an `EmptyBody`.
        raw = req.body isa HTTP.BytesBody ? String(copy(req.body.data)) : ""
        body = isempty(raw) ? nothing : JSON3.read(raw, Dict{String, Any})
        push!(log, (path, body))
        path == "/health" && return json(200, Dict("status" => "ok"))
        path == "/status" && return json(200, Dict("state" => state[]))
        path == "/trajectory" && return json(200, table(power[]))
        path == "/init" &&
            return json(200, Dict("name" => body["name"], "length" => body["length"],
                                  "trajectory" => body["trajectory"], "state" => "ready"))
        if path == "/step"
            validation && return json(422, Dict("detail" => [Dict("loc" => ["body", "bogus"],
                                                                   "msg" => "extra field")]))
            failing = body["length"] in fail || (fail_warm && isnothing(body["trajectory"]))
            if !body["wait"]
                failing || (power[] += 1000.0)
                state[] = failing ? "failed" : instant ? "converged" : "solving"
                return json(200, Dict("step_index" => 7))
            end
            failing && return json(422, Dict("detail" => detail))
            power[] += 1000.0
            return json(200, step_reply(body["length"], power[]))
        end
        return json(404, Dict("detail" => "no route"))
    end
    server = HTTP.serve!(handle, "127.0.0.1", 0)
    return (; server, url = "http://127.0.0.1:$(HTTP.port(server))", log, state)
end
paths(fs) = first.(fs.log)
