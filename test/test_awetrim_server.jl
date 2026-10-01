# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the optimizer client's side of the wire (`src/awetrim_client.jl`) against a fake
AWETrim server started here, on a free local port: what `post` sends and how it reports the two
kinds of 422, the endpoints, and the solution cache's miss path, where the chain asks the server,
stores what was applied, rebuilds the session after cache hits and polls a non-blocking solve.
The fake answers every request at once and records it; it knows nothing of the physics.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: InflowConditions, WinchParams, Trajectory, InitParams, StepParams,
    OptChain, opt_request_key, post, opt_init, opt_step, opt_status, opt_trajectory,
    chain_init, chain_step, chain_status, chain_trajectory, record_opt_success!,
    stale_conn_error, server_running, ensure_server, HTTP, JSON3

include(joinpath(@__DIR__, "fake_awetrim_server.jl"))
const AZ, EL = FAKE_AZ, FAKE_EL
const INFLOW = InflowConditions(; wind_speed = 8.0, wind_direction = 270.0, profile_law = 3)
const WINCH = WinchParams("reelout", 0.04, 700.0, 7200.0)
params(; kw...) = InitParams(; name = "v4", length = 150.0, winch_params = WINCH,
                             inflow_conditions = INFLOW, trajectory = Trajectory(AZ, EL), kw...)


@testset verbose = true "awetrim_server" begin
    @testset "post_and_endpoints" begin
        fs = fake_server()
        try
            @test server_running(fs.url) && ensure_server(fs.url; autostart = false) == fs.url
            r = opt_init(params(); url = fs.url)
            @test r.name == "v4" && r.length == 150.0 && r.trajectory.azimuth == AZ
            # The request as sent: every field of InitParams, in JSON.
            body = fs.log[end][2]
            @test body["length"] == 150.0 && body["winch_params"]["f_max"] == 7200.0
            @test body["trajectory"]["elevation"] == EL && isnothing(body["min_turn_radius"])
            s = opt_step(StepParams(160.0, WINCH); url = fs.url, max_iter = 50)
            @test s.length == 160.0 && s.metrics.avg_power_W == 1000.0
            @test fs.log[end][2]["wait"] && fs.log[end][2]["max_iter"] == 50
            @test opt_step(StepParams(170.0, WINCH); url = fs.url, wait = false) == 7
            @test !fs.log[end][2]["wait"]
            @test opt_status(fs.url)["state"] == "solving"
            @test opt_trajectory(; url = fs.url)["optimized_parameters"]["input_depower"] == 1.42
            @test fs.log[end][1] == "/trajectory"
        finally
            close(fs.server)
        end
        # Nothing answers on a closed port.
        @test !server_running(fs.url; timeout = 1)
        @test_throws ErrorException ensure_server(fs.url; autostart = false, verbose = false)
    end

    @testset "status_errors" begin
        # An infeasible solve: a plain-string detail, logged at info level, rethrown as it came.
        fs = fake_server(; fail = [160.0], detail = "IPOPT failure")
        try
            err = nothing
            payload = merge(SimpleKiteControllers.as_dict(StepParams(160.0, WINCH)),
                            Dict("wait" => true))
            @test_logs (:info, r"HTTP 422: IPOPT failure") begin
                err = try
                    post("/step", payload; url = fs.url)
                    nothing
                catch exc
                    exc
                end
            end
            @test err isa HTTP.StatusError && err.status == 422
            @test count(==("/step"), paths(fs)) == 1      # a 422 is an answer, not resent
        finally
            close(fs.server)
        end
        # A request the server rejects (a field it does not accept): a client bug, warned.
        fs = fake_server(; validation = true)
        try
            @test_logs (:warn, r"REJECTED[\s\S]*body\.bogus: extra field") match_mode = :any begin
                @test_throws HTTP.StatusError opt_step(StepParams(160.0, WINCH); url = fs.url)
            end
        finally
            close(fs.server)
        end
    end

    @testset "stale_conn_error" begin
        @test stale_conn_error(SystemError("write", 32))
        @test stale_conn_error(EOFError())
        @test stale_conn_error(CapturedException(EOFError(), []))   # unwrapped
        @test stale_conn_error(ErrorException("write: Broken pipe (EPIPE)"))
        @test !stale_conn_error(ErrorException("a different error"))
        @test !stale_conn_error(ArgumentError("bad"))
    end

    @testset "chain_miss_store_and_serve" begin
        dir = mktempdir()
        fs = fake_server(; fail = [180.0])
        try
            oc = OptChain(fs.url; dir)
            chain_init(oc, params())
            @test oc.state == oc.server == opt_request_key(params())
            r = chain_step(oc, StepParams(160.0, WINCH))   # a miss: asks the server
            @test r.metrics.avg_power_W == 1000.0 && oc.misses == 1 && !oc.served
            @test paths(fs)[end-1:end] == ["/step", "/trajectory"]   # the table of a converged step
            @test chain_trajectory(oc)["metrics"]["avg_power_W"] == 1000.0
            @test isempty(readdir(dir))                     # not stored before it is applied
            record_opt_success!(oc)
            @test length(readdir(dir)) == 1
            # A failed step is stored at once, so no rerun pays for it again.
            @test_throws HTTP.StatusError chain_step(oc, StepParams(180.0, WINCH))
            @test length(readdir(dir)) == 2

            # The same chain again: served from the cache, the server is not asked.
            n = length(fs.log)
            oc2 = OptChain(fs.url; dir)
            chain_init(oc2, params())                        # /init always goes out
            @test chain_step(oc2, StepParams(160.0, WINCH)).metrics.avg_power_W == 1000.0
            @test oc2.hits == 1 && oc2.served
            @test_throws HTTP.StatusError chain_step(oc2, StepParams(180.0, WINCH))
            @test oc2.hits == 2
            @test paths(fs)[n+1:end] == ["/init"]

            # A request the cache has not seen, after hits: the server's session is REBUILT first
            # (/init seeded with the cached optimum and its depower, one /step), then it is sent.
            oc3 = OptChain(fs.url; dir)
            chain_init(oc3, params())
            chain_step(oc3, StepParams(160.0, WINCH))      # hit
            n = length(fs.log)
            r = @test_logs (:info, r"Rebuilding the optimizer's session") match_mode = :any chain_step(
                oc3, StepParams(170.0, WINCH))
            @test oc3.rebuilds == 1 && oc3.misses == 1
            @test paths(fs)[n+1:end] == ["/init", "/step", "/trajectory", "/step", "/trajectory"]
            init_body = fs.log[n+1][2]
            @test init_body["trajectory"]["elevation"] ≈ EL .+ 1   # the cached optimum, in degrees
            @test init_body["input_depower"] == 1.42
            # The rebuild re-creates the session where the chain had got to: 160 m, then sends 170 m.
            @test init_body["length"] == 160.0 && fs.log[n+2][2]["length"] == 160.0
            @test fs.log[n+4][2]["length"] == 170.0
            @test oc3.server == oc3.state                    # the server holds the chain's state
            # What is solved from a rebuilt session is stored under its own lineage.
            record_opt_success!(oc3)
            oc4 = OptChain(fs.url; dir)
            chain_init(oc4, params())
            chain_step(oc4, StepParams(160.0, WINCH))
            n = length(fs.log)
            chain_step(oc4, StepParams(170.0, WINCH))      # found under the rebuilt lineage
            @test oc4.hits == 2 && oc4.rebuilds == 0 && length(fs.log) == n
        finally
            close(fs.server)
        end
    end

    @testset "chain_non_blocking" begin
        fs = fake_server()
        try
            oc = OptChain(fs.url; dir = mktempdir())
            chain_init(oc, params())
            @test chain_step(oc, StepParams(160.0, WINCH); wait = false) == 7
            @test chain_status(oc)["state"] == "solving"   # asked: the outcome is not known yet
            fs.state[] = "converged"
            @test chain_status(oc)["state"] == "converged"
            @test oc.current["status"] == "converged" && fs.log[end][1] == "/trajectory"
            n = length(fs.log)
            @test chain_status(oc) == Dict("state" => "converged")   # known: not asked again
            @test length(fs.log) == n
        finally
            close(fs.server)
        end
    end
end
nothing
