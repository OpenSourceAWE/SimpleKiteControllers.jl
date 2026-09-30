# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The AWETrim REST client of the example scripts. It lives in the package
# (`src/awetrim_client.jl`, not exported); `include` this file to bring its names,
# and the packages its callers use with it (`HTTP.StatusError` for a 422), into scope:
#
#     include("awetrim_client.jl")
#     ensure_server()
#     reply  = opt_init(InitParams(; name = "run-1", length = 200.0, ...))
#     result = opt_step(StepParams(200.0, winch, reply.trajectory))

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using HTTP, JSON3, StructTypes
using YAML
using AtmosphericModels: AtmosphericModel, calc_wind_factor
using SimpleKiteControllers: with_file_lock, turn_rate_coeffs
# The pure parts of a request: the pattern box and floors, the depower conversion and
# seed, the turn-radius request, and the wind and winch read off the run's settings.
using SimpleKiteControllers: PatternLimits, pattern_limits_from, elevation_min_request,
    elevation_amplitude_max_at
using SimpleKiteControllers: AWETRIM_V3KITE_DEPOWER_OFFSET, awetrim_depower_to_v3kite,
    DEPOWER_SEED_BOUNDS, depower_seed, min_turn_radius_request
using SimpleKiteControllers: InflowConditions, WinchParams, inflow_from_settings, winch_from_wc,
    cap_wind_speed, AWETRIM_SOFTMINUS_BETA, SEND_V_SAT_BETA
# The client: request and reply structs, the endpoints, the server process and the caches.
using SimpleKiteControllers: SKC_ROOT, AWETRIM_URL, Trajectory, DepowerSpec, DepowerReply,
    SolveMetrics, InitParams, StepParams, InitReply, StepReply
using SimpleKiteControllers: opt_init, opt_step, opt_status, opt_trajectory, opt_get, opt_float,
    server_running, ensure_server, stop_server
using SimpleKiteControllers: OPT_FAILURE_CACHE, opt_request_key, opt_failures, opt_failed_before,
    record_opt_failure!, clear_opt_failures, stable_hash
using SimpleKiteControllers: OPT_CHAIN_CACHE, OptChain, chain_key, chain_init, chain_step,
    chain_status, chain_trajectory, record_opt_success!, rebuild_session!, replay_entries,
    clear_opt_chain_cache
using SimpleKiteControllers: guess_el_center_seed, reelout_anchor_ratio, optimizer_session
