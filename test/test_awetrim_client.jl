# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the parts of the optimizer client (`src/awetrim_client.jl`) that decide whether a
cached or replayed run reproduces a live one: the request keys, the failure cache, the solution
cache, the startup solve's seed retries, the reply parsing and two small helpers. Nothing here
reaches the server: the caches live in temporary files, and the chain is set up as `chain_init`
leaves it, without sending the `/init`.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: InflowConditions, WinchParams, Trajectory, InitParams, StepParams,
    PatternLimits, DepowerSpec, opt_request_key, stable_hash, chain_key, _key_fields,
    opt_failures, opt_failed_before, record_opt_failure!, clear_opt_failures, OptChain,
    _write_chain_entry, _chain_entry, _cached_422, chain_step, chain_status, chain_trajectory,
    record_opt_success!, solve_startup, as_step_reply, as_pattern_limits, guess_el_center_seed,
    reelout_anchor_ratio, with_file_lock, HTTP, JSON3

const INFLOW = InflowConditions(; wind_speed = 8.0, wind_direction = 270.0, profile_law = 3)
const WINCH = WinchParams("reelout", 0.04, 700.0, 7200.0)
traj(el_center = 25.0) = Trajectory([0.0, 10.0, 0.0, -10.0], el_center .+ [0.0, 5.0, 0.0, -5.0])
params(; el_center = 25.0, kw...) =
    InitParams(; name = "v4", length = 150.0, winch_params = WINCH, inflow_conditions = INFLOW,
               trajectory = traj(el_center), kw...)

# A reply as the server sends it, with only the fields `as_step_reply` requires.
reply_dict(power = 1000.0) = Dict{String, Any}(
    "trajectory" => Dict{String, Any}("azimuth" => [0.0, 10.0, 0.0, -10.0],
                                      "elevation" => [25.0, 30.0, 25.0, 20.0]),
    "state" => "converged", "step_index" => 1,
    "metrics" => Dict{String, Any}("energy_J" => 1e5, "total_time_s" => 100.0,
                                   "avg_power_W" => power))

# A chain as `chain_init` leaves it after a successful `/init`, without asking the server.
function inited_chain(dir; kw...)
    oc = OptChain("http://127.0.0.1:1"; dir, kw...)   # a port nothing answers on
    oc.state = oc.server = opt_request_key(params())
    oc.config = params()
    return oc
end
step_key(oc, sp) = chain_key(oc.state, (_key_fields(sp), _key_fields(nothing), nothing))

@testset verbose = true "awetrim_client" begin
    @testset "request_keys" begin
        p = params()
        k = opt_request_key(p)
        # Pinned: a change here silently orphans every cached entry, so it must be deliberate.
        @test k == "v4-2549e297cc9588f5"
        @test opt_request_key(params(; name = "other run")) == k   # the label is not the problem
        p2 = InitParams(; name = "v4", length = 150.0, winch_params = WINCH,
                        inflow_conditions = INFLOW,
                        trajectory = Trajectory(copy(p.trajectory.azimuth),
                                                copy(p.trajectory.elevation)))
        @test opt_request_key(p2) == k                              # by value, not identity
        # Every field the server sees is in the key, down to the exact length.
        for q in (params(; length = 150.00027), params(; el_center = 26.0),
                  params(; min_turn_radius = 12.0), params(; depower = DepowerSpec()),
                  params(; input_depower = 1.7),
                  params(; pattern_limits = PatternLimits(; azimuth_max = 28.0)),
                  params(; winch_params = WinchParams(; mode = "reelout", k_v = 0.04,
                                                      f_min = 700.0, f_max = 7200.0,
                                                      softplus_beta = 0.03)),
                  params(; inflow_conditions = InflowConditions(; wind_speed = 8.0,
                                                                wind_direction = 270.0,
                                                                profile_law = 3, z0 = 0.001)))
            @test opt_request_key(q) != k
        end
        @test opt_request_key(params(; length = -0.0)) != opt_request_key(params(; length = 0.0))
        @test opt_request_key(params(; length = NaN)) == opt_request_key(params(; length = NaN))
        # `symmetric` is left out of the key while unset, so the keys from before it stay valid.
        @test length(_key_fields(PatternLimits(; azimuth_max = 28.0))) == 5
        @test length(_key_fields(PatternLimits(; azimuth_max = 28.0, symmetric = true))) == 6
        # So is `climb_angle_max`, and an unset one is left out of the JSON: a server older
        # than the field rejects the key even as null.
        @test length(_key_fields(PatternLimits(; azimuth_max = 28.0, climb_angle_max = 45.0))) == 6
        @test !occursin("climb_angle_max", JSON3.write(PatternLimits(; azimuth_max = 28.0)))
        @test occursin("\"climb_angle_max\":45.0", JSON3.write(PatternLimits(; climb_angle_max = 45.0)))
        @test stable_hash((1, "a")) == stable_hash((1, "a")) != stable_hash((1, "b"))
        # A step's key carries its parent's: equal keys mean equal lineage.
        sp = StepParams(150.0, WINCH)
        @test chain_key(k, _key_fields(sp)) != chain_key("v4-0000000000000000", _key_fields(sp))
        @test startswith(chain_key(k, _key_fields(sp)), "c2-")
    end

    @testset "with_file_lock" begin
        mktempdir() do dir
            lockp = joinpath(dir, "x.lock")

            @test with_file_lock(() -> 42, lockp) == 42
            @test !isdir(lockp)

            # Released even when the protected function throws.
            @test_throws ErrorException with_file_lock(lockp) do
                error("boom")
            end
            @test !isdir(lockp)

            # A held, non-stale lock makes a waiter time out rather than proceed.
            mkdir(lockp)
            try
                @test_throws ErrorException with_file_lock(() -> 1, lockp;
                                                            timeout = 0.2,
                                                            stale_after = 1e6,
                                                            poll = 0.02)
            finally
                isdir(lockp) && rm(lockp; recursive = true, force = true)
            end

            # A lock far older than stale_after is broken (with a warning) instead
            # of blocking every later process behind a dead one.
            mkdir(lockp)
            run(`touch -d 1970-01-01 $lockp`)
            ran = Ref(false)
            @test_logs (:warn,) match_mode = :any with_file_lock(lockp;
                                                                 stale_after = 1.0) do
                ran[] = true
            end
            @test ran[]
            @test !isdir(lockp)
        end
    end

    @testset "failure_cache" begin
        file = joinpath(mktempdir(), "sub", "failures.yaml")
        @test isempty(opt_failures(; file))                       # no file: an empty cache
        @test isnothing(opt_failed_before(params(); file))
        record_opt_failure!(params(; min_turn_radius = 11.85), "422 from /step"; file)
        e = opt_failed_before(params(; min_turn_radius = 11.85); file)
        @test e["reason"] == "422 from /step" && e["length_m"] == 150.0
        @test e["wind_speed_m_s"] == 8.0 && e["min_turn_radius_m"] == 11.85
        @test e["guess_el_center_deg"] == 25.0 && e["depower_mode"] == "optimize"
        @test isnothing(opt_failed_before(params(); file))        # another radius, another request
        record_opt_failure!(params(; length = 200.0), "solver failed"; file)
        @test length(opt_failures(; file)) == 2                   # entries accumulate
        @test clear_opt_failures(; file) && !clear_opt_failures(; file)
        @test isempty(opt_failures(; file))
        write(file, "entries: {a: [1, 2")                         # unreadable: empty, never a stop
        @test (@test_logs (:warn, r"unreadable") match_mode = :any opt_failures(; file)) ==
              Dict{String, Any}()
    end

    @testset "solution_cache" begin
        dir = mktempdir()
        oc = inited_chain(dir)
        sp = StepParams(150.0, WINCH)
        key = step_key(oc, sp)
        @test key == "c2-645a7afd33368830"
        table = Dict{String, Any}("metrics" => Dict{String, Any}("avg_power_W" => 1000.0))
        _write_chain_entry(oc, Dict{String, Any}("key" => key, "status" => "converged",
                                                 "reply" => reply_dict(), "table" => table,
                                                 "when" => "2026-10-01 08:00:00"))
        @test isfile(joinpath(dir, key * ".json"))
        r = @test_logs (:info, r"served from the solution cache") chain_step(oc, sp)
        @test r.trajectory.elevation == [25.0, 30.0, 25.0, 20.0] && r.metrics.avg_power_W == 1000.0
        @test oc.hits == 1 && oc.misses == 0 && oc.served && oc.state == key
        @test oc.server != oc.state                  # the server did not see that step
        tab = chain_trajectory(oc)
        @test tab == table
        tab["metrics"]["avg_power_W"] = 0.0          # a copy: what is stored cannot be edited
        @test chain_trajectory(oc)["metrics"]["avg_power_W"] == 1000.0
        @test chain_status(oc) == Dict("state" => "converged")   # known, not asked

        # A cached failure throws the 422 the server would, so callers need not know the difference.
        oc = inited_chain(dir)
        sp2 = StepParams(160.0, WINCH)
        _write_chain_entry(oc, Dict{String, Any}("key" => step_key(oc, sp2), "status" => "failed",
                                                 "reason" => "IPOPT failure",
                                                 "when" => "2026-10-01 08:00:00"))
        err = try
            chain_step(oc, sp2)
            nothing
        catch exc
            exc
        end
        @test err isa HTTP.StatusError && err.status == 422
        @test occursin("IPOPT failure", String(copy(err.response.body)))
        @test oc.hits == 1
        oc = inited_chain(dir)                       # not blocking: returns at once
        @test chain_step(oc, sp2; wait = false) == 0
        @test chain_status(oc) == Dict("state" => "failed")

        # An unreadable entry is no entry.
        write(joinpath(dir, "c2-broken.json"), "{not json")
        @test isnothing(@test_logs (:warn, r"unreadable") match_mode = :any _chain_entry(oc, "c2-broken"))

        # Applying a result stores the converged steps of its lineage, and only those.
        oc = inited_chain(mktempdir())
        e1 = Dict{String, Any}("key" => "c2-a", "status" => "converged", "reply" => reply_dict())
        e2 = Dict{String, Any}("key" => "c2-b", "status" => "failed")
        e3 = Dict{String, Any}("key" => "c2-c", "status" => "converged", "reply" => reply_dict())
        append!(oc.pending, [e1, e2, e3])
        oc.current = e3
        record_opt_success!(oc)
        @test e3["applied"] && isempty(oc.pending)
        @test sort(readdir(oc.dir)) == ["c2-a.json", "c2-c.json"]
        oc = inited_chain(mktempdir(); successes = false)   # successes off: nothing stored
        push!(oc.pending, e1); oc.current = e1
        record_opt_success!(oc)
        @test isempty(readdir(oc.dir)) && isempty(oc.pending)
        oc.current = e2                                   # nothing converged to apply
        @test_logs (:warn, r"without a converged result") record_opt_success!(oc)
    end

    @testset "replay" begin
        entries = [Dict{String, Any}("key" => "c2-$i", "status" => "converged",
                                     "reply" => reply_dict(1000.0 * i), "table" => Dict{String, Any}(),
                                     "length_m" => 150.0 + 10i, "when" => "2026-10-01")
                   for i in 1:2]
        oc = inited_chain(mktempdir(); replay = copy(entries))
        # Served in order, whatever is asked; then every further step fails as a 422.
        @test chain_step(oc, StepParams(999.0, WINCH)).metrics.avg_power_W == 1000.0
        @test chain_step(oc, StepParams(150.0, WINCH)).metrics.avg_power_W == 2000.0
        @test oc.hits == 2 && oc.state == "c2-2"
        @test_throws HTTP.StatusError chain_step(oc, StepParams(150.0, WINCH))
    end

    @testset "solve_startup_seed_retries" begin
        tos = (; name = "v4", startup_retry_el_offsets = [-1.0, 2.0], opt_failure_cache = true,
               guess_a = 30.0, guess_b = 12.0)
        center(p) = sum(p.trajectory.elevation) / length(p.trajectory.elevation)
        # `solve` answers 422 for the seeds in `bad` and converges elsewhere; `sent` records each call.
        function run(bad; file, tos = tos)
            sent = Float64[]
            solve = function (p)
                push!(sent, center(p))
                center(p) in bad && throw(_cached_422(Dict("reason" => "test")))
                return (; seed = center(p)), traj(center(p))
            end
            r = solve_startup(tos, el -> params(; el_center = el), solve, params(), 25.0, 150.0,
                              WINCH, INFLOW; failure_file = file)
            return r, sent
        end
        file = joinpath(mktempdir(), "failures.yaml")
        r, sent = run(Float64[]; file)                       # the shipped guess converges
        @test sent == [25.0] && r.startup_seed_offset == 0.0 && r.el_center_seed == 25.0
        @test isempty(opt_failures(; file))

        r, sent = @test_logs (:warn, r"retry 1/2") match_mode = :any run([25.0]; file)
        @test sent == [25.0, 24.0] && r.startup_seed_offset == -1.0 && r.el_center_seed == 24.0
        @test r.guess_el == traj(24.0).elevation
        @test !isnothing(opt_failed_before(params(); file))  # the 422 is recorded

        file = joinpath(mktempdir(), "failures.yaml")
        err = try
            run([25.0, 24.0, 27.0]; file)
            nothing
        catch exc
            exc
        end
        @test err isa ErrorException && occursin("The optimizer returned no path", err.msg)
        @test length(opt_failures(; file)) == 3              # the budget: the guess and two retries
        # The same run again: the three are cached as bad and skipped without costing a retry,
        # so the seeds walk outward past the listed ones: offset +1 is the next one sent.
        r, sent = run([25.0, 24.0, 27.0]; file)
        @test sent == [26.0] && r.startup_seed_offset == 1.0

        # No retry seeds and the only one cached as bad: nothing is sent, and the error says why.
        tos0 = merge(tos, (; startup_retry_el_offsets = Float64[]))
        file = joinpath(mktempdir(), "failures.yaml")
        record_opt_failure!(params(), "422 from /step"; file)
        err = try
            run(Float64[]; file, tos = tos0)
            nothing
        catch exc
            exc
        end
        @test err isa ErrorException && occursin("cached as bad", err.msg)
        # With the failure cache off it is neither read nor written.
        tosoff = merge(tos, (; opt_failure_cache = false))
        r, sent = run([25.0]; file, tos = tosoff)
        @test sent == [25.0, 24.0] && length(opt_failures(; file)) == 1
    end

    @testset "reply_parsing" begin
        d = reply_dict()
        r = as_step_reply(d)                     # an older server: the optional fields absent
        @test isnothing(r.depower) && isnothing(r.pattern_limits) && isnothing(r.winch_params)
        @test isnothing(r.metrics.turn_radius_min_m) && r.step_index == 1
        d["depower"] = Dict{String, Any}("mode" => "optimize", "value" => 1.42)
        d["pattern_limits"] = Dict{String, Any}("azimuth_max" => 28, "symmetric" => true,
                                                "climb_angle_max" => 45)
        d["winch_params"] = Dict{String, Any}("mode" => "reelout", "k_v" => 0.04, "f_min" => 700,
                                              "f_max" => 7200, "v_max" => nothing)
        d["metrics"]["turn_radius_min_m"] = 12.48
        r = as_step_reply(d)
        @test r.depower.value == 1.42 && isnothing(r.depower.profile)
        @test r.pattern_limits.azimuth_max === 28.0 && r.pattern_limits.symmetric
        @test r.pattern_limits.climb_angle_max === 45.0
        @test isnothing(r.pattern_limits.elevation_min)
        @test r.winch_params.f_max === 7200.0 && isnothing(r.winch_params.v_max)
        @test !r.winch_params.optimize_k_v && r.metrics.turn_radius_min_m == 12.48
        @test isnothing(as_pattern_limits(nothing))
    end

    @testset "seed_and_anchor_ratio" begin
        tos = (; guess_el_center = 30.0, guess_el_center_high = 32.0, guess_el_center_wind_ref = 8.0)
        @test guess_el_center_seed(tos, 7.99) == 30.0
        @test guess_el_center_seed(tos, 8.0) == 32.0             # a step at the reference wind
        @test guess_el_center_seed(merge(tos, (; guess_el_center_high = 0.0)), 12.0) == 30.0
        table(r) = Dict("table" => Dict("distance_radial" => r))
        @test reelout_anchor_ratio(table([150.0, 165.0, 180.0])) ≈ 1.2
        # Anything unusable is the neutral factor, never an error.
        for t in (Dict(), table(Float64[]), table([150.0, NaN]), table([0.0, 180.0]),
                  Dict("table" => Dict()))
            @test reelout_anchor_ratio(t) == 1.0
        end
    end
end
nothing
