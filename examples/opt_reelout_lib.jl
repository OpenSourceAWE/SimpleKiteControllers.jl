# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Functions of `simple_opt_reelout.jl`, moved out of its top level step by step
(see `Plan_refactor_opt_reelout.md`). Included at the top of the script, into
the same module.
"""

"""
    run_input_defaults() -> NamedTuple

Every input a caller of `simple_opt_reelout.jl` may pass with
`run_example("simple_opt_reelout.jl"; ...)` (see `script_inputs.jl`), at its default:

- `show_plots`: draw the figures.
- `run_archive`: copy the log and its inputs to `output/archives/<stamp>/`.
- `path_tr_project`: a system project whose turn-rate table sizes the path,
  `nothing` for the run's own.
- `fcs_overrides`, `tos_overrides`, `wc_overrides`, `set_overrides`: sweep and
  test overrides of the controller, optimizer, winch and plant settings.
- `steer_disturbance`, `xtrack_offset`, `xtrack_phase`, `hold_compliance`: test
  inputs of the stability and cross-track analyses.
- `steer_gain_factor`, `steer_gain_feedback_only`, `extra_steer_delay`,
  `hook_settle`: the V1 stability hook.
- `replay_paths`: a scenario folder whose optimizer results are replayed.
- `output_path`: where the log goes instead of `output/`, `nothing` for `output/`.
"""
run_input_defaults() =
    (; show_plots = true, run_archive = true, path_tr_project = nothing,
       fcs_overrides = Dict{Symbol, Any}(), tos_overrides = Dict{Symbol, Any}(),
       wc_overrides = Dict{Symbol, Any}(), set_overrides = Dict{Symbol, Any}(),
       steer_disturbance = nothing, xtrack_offset = nothing, xtrack_phase = 5,
       hold_compliance = nothing, steer_gain_factor = 1.0, steer_gain_feedback_only = false,
       extra_steer_delay = 0, hook_settle = 15.0, replay_paths = nothing, output_path = nothing)

"""
    build_winch(project, project_set, fcs) -> (; wc, wpc, dt0)

The winch settings and the length loop of the run. ONE `WCSettings` (`wc`) serves BOTH
winch loops, the POSITION-mode torque gains (`wpc`) and the speed-controller tuning of the
reel-out controller, so the wind-dependent `kv`, force floor and force-limit law of the
project's tables are set on it here. Refuses a `compliance` other than 0: REEL_OUT and
V3Kite's own FORCE mode both drive the winch, and only one can hold the drum at a time.
"""
function build_winch(project, project_set, fcs)
    fcs.compliance >= 0 ||
        error("compliance must be >= 0, got $(fcs.compliance)")
    fcs.compliance == 0 ||
        error("REEL_OUT needs compliance = 0 (POSITION mode) — REEL_OUT and V3Kite's own \
               FORCE mode both drive the winch and only one can hold the drum at a time.")
    dt0 = 1 / project_set.sample_freq
    wc = load_wc_settings(wc_settings(project); dt = dt0)
    wc.kv = winch_kv(project_set.v_wind; project) # overrides the file's flat kv, see data/winch_kv_table.yaml
    # Wind-dependent floor too; NOT the entry guard's floor, that is fcs.entry_f_min.
    wc.f_low = winch_f_low(project_set.v_wind; project)
    # The soft law's floor cannot go below ~700 N, so it is off at low wind; see `winch_force_limit`'s docstring.
    wc.force_limit = winch_force_limit(project_set.v_wind; project)
    wpc = WinchPosController(wc; dt = dt0)   # the length loop `step!` used to own
    return (; wc, wpc, dt0)
end

"""
    init_model(project, project_set, fcs, wpc, sim_time; turbulence, aero_mode,
               damping_per_stiffness, set_overrides)

The model, initialised and settled: `init` at the project's wind and tether length,
`sim_time` plus a full phase 5 (`fcs.final_time`) long, with the winch loop `wpc`
holding the length during the warm-up. `set_overrides` are applied to
the model's own `Settings` afterwards, e.g. `v_steering`, the tape's rate limit.
"""
function init_model(project, project_set, fcs, wpc, sim_time; turbulence, aero_mode,
                    damping_per_stiffness, set_overrides)
    # dt, sim_time and wind come from the project settings (overridden above); the default cache_path avoids a re-JIT.
    s = init(project_set.v_wind, project_set.l_tether; body_start_damping = fcs.body_damping,
        body_sim_damping = 0.8 .* fcs.body_damping,
        damping_per_stiffness = damping_per_stiffness,
        elevation = fcs.elevation, depower_setpoint = fcs.depower_setpoint,
        system_yaml = project, use_turbulence = turbulence, aero_mode = aero_mode,
        # Room for a full phase 5 past the budget: the budget ends a run only BEFORE phase 5,
        # which then always flies `final_time` (see the loop). At 10 m/s the budget cut it at 19.7 s.
        sim_time = sim_time + (isfinite(fcs.final_time) ? fcs.final_time : 0.0),
        warmup_time = fcs.warmup_time,
        # The warm-up relaxes at constant length, against the same loop the run uses.
        warmup_torque = (m, l) -> winch_torque!(wpc, m, l), remake_model = false)
    @info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)
    apply_overrides!(s.kcu.set, set_overrides, "set_overrides", "Settings", "plant")
    return s
end

"""
    build_controllers(fcs, rcs, s) -> (; rc, f_high_nominal, stop_criteria, guard_lfc, l_set, fec)

The controllers of the run, built on the settled model `s`: the reel-out winch controller
`rc` (built after `init`, so its soft-start ramp begins when reel-out starts; `rcs` is the
one `WCSettings` of both winches), the nominal force ceiling `f_high_nominal` captured
before the first-lap reduction (`winch_from_wc` sends this one to the optimizer), the
standalone force-floor guard for phases 0-2 (`rc`'s own `SpeedController` would wind up
while its output is ignored), the length setpoint `l_set` (the settled length, growing
from phase 3 until it reaches `reelout_l_max`) and the figure-of-eight controller `fec`.
"""
function build_controllers(fcs, rcs, s)
    rcs.dt = s.dt
    rc = WinchController(rcs)
    f_high_nominal = rcs.f_high
    stop_criteria = fcs.n_fig_eight > 0 ?
        @sprintf("%.0f m or after %d figures of eight", fcs.reelout_l_max, fcs.n_fig_eight) :
        @sprintf("%.0f m", fcs.reelout_l_max)
    @info @sprintf("Winch: REEL_OUT mode — %s, stopping at %s.",
                   rcs.force_limit == "soft" ?
                       @sprintf("soft force limit inverting kv = %.4f saturated at [%.0f, %.0f] N \
                                 (beta %.0e/%.0e, force filtered at tau = %.2f s); the \
                                 UpperForceController is held in reset",
                                rcs.kv, rcs.f_low, rcs.f_high, rcs.softminus_beta,
                                rcs.softplus_beta, rcs.force_limit_tau) :
                       @sprintf("v_set = %.3f * sqrt(force)", rcs.kv),
                   stop_criteria)
    guard_lfc = LowerForceController(rcs)
    l_set = s.sys_state.l_tether[1]
    fec = FigureEightController(fcs; dt = s.dt)
    return (; rc, f_high_nominal, stop_criteria, guard_lfc, l_set, fec)
end

"""
    optimizer_conditions(tos, fcs, project_set, rcs, f_high_nominal)
        -> (; inflow, cap_wind, opt_awe_trim, opt_winch_mode, winch, winch_first_lap, winch_reopt)

What the optimizer is SENT about THIS run, decoupled from the local winch law (the solve
must converge at a force the kite can pull): the `inflow`, the wind the elevation-cap step
reads (`cap_wind`), the AWETrim setting and mode, and the winch of the three kinds of solve:
`winch` for the STARTUP solve, the plain runtime ceiling `f_high_nominal`, never
`rcs.f_high_awe_trim`; `winch_first_lap` under `first_lap_force_frac`, so the startup path is
solved against the ceiling lap 1 flies under; and `winch_reopt` for the re-optimizations (lap 2
on), the one caller that may fly `rcs.f_high_awe_trim`.
"""
function optimizer_conditions(tos, fcs, project_set, rcs, f_high_nominal)
    inflow = inflow_from_settings(project_set)
    cap_wind = cap_wind_speed(tos, project_set, inflow.wind_speed)
    opt_awe_trim = tos.opt_awe_trim >= 0 ? tos.opt_awe_trim : rcs.use_awe_trim
    opt_winch_mode = isempty(tos.opt_winch_mode) ? nothing : tos.opt_winch_mode
    winch = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                          winch_mode = opt_winch_mode, f_max = f_high_nominal)
    winch_first_lap = fcs.first_lap_force_frac < 1 ?
        winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                      winch_mode = opt_winch_mode,
                      f_max = f_high_nominal * fcs.first_lap_force_frac) : winch
    winch_reopt = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                                winch_mode = opt_winch_mode)
    @info @sprintf("Optimizer conditions: %.1f m/s at 6 m from %.0f°, profile_law %d, \
                    z0 = %g m | winch kv = %.4f, i.e. %.1f m/s at f_high = %.0f N | \
                    depower seed %.3f m%s.",
                   inflow.wind_speed, inflow.wind_direction, inflow.profile_law, inflow.z0,
                   winch.k_v, winch.k_v * sqrt(winch.f_max), winch.f_max,
                   depower_seed(tos, inflow.wind_speed),
                   inflow.wind_speed > tos.input_depower_wind_ref ?
                       @sprintf(" (%.2f + %.3f per m/s above %.1f m/s)", tos.input_depower,
                                tos.input_depower_per_wind, tos.input_depower_wind_ref) : "")
    return (; inflow, cap_wind, opt_awe_trim, opt_winch_mode, winch, winch_first_lap, winch_reopt)
end

"""
    optimizer_session(tos, inflow, replay_paths, log_name)
        -> (; el_center_seed_base, el_center_seed, startup_seed_offset, guess_az, guess_el, opt_chain)

The connection to the optimizer and the seed of the startup solve. The seed is the guess
lemniscate of `data/traj_opt.yaml`, centred at `el_center_seed` (`el_center_seed_base` until
the startup solve converges from another; `startup_seed_offset` is how far). Every request
of the run goes through `opt_chain`, which replays applied results and known failures (see
`OptChain`); `replay_paths`, a scenario folder, flies that run's optimizer results
instead of asking the optimizer, see `replay_entries`.
"""
function optimizer_session(tos, inflow, replay_paths, log_name)
    el_center_seed_base = guess_el_center_seed(tos, inflow.wind_speed)
    el_center_seed = el_center_seed_base
    startup_seed_offset = 0.0
    guess_az, guess_el = figure_eight_path(tos.guess_a, tos.guess_b,
                                           0.0, el_center_seed,
                                           0.0, tos.guess_points)
    @info @sprintf("Initial guess: %.0f° x %.0f° at %.0f°, %d points.",
                   tos.guess_a, tos.guess_b, el_center_seed, tos.guess_points)
    ensure_server(tos.base_url; autostart = tos.autostart_server)
    opt_chain = OptChain(tos.base_url; successes = tos.opt_success_cache,
                         failures = tos.opt_failure_cache,
                         replay = isnothing(replay_paths) ? nothing :
                                  replay_entries(replay_paths, log_name))
    isnothing(replay_paths) ||
        @info "Replaying the $(length(opt_chain.replay)) optimizer results of $replay_paths; the optimizer is not asked."
    return (; el_center_seed_base, el_center_seed, startup_seed_offset, guess_az, guess_el, opt_chain)
end


"""
    solve_startup(tos, make_params, solve, start_params, el_center_seed_base, l_set, winch, inflow)
        -> (; opt_result, opt_seed_trajectory, start_params, el_center_seed, startup_seed_offset,
             guess_az, guess_el)

The STARTUP solve, from `start_params` (the shipped guess) and, if the optimizer answers
422 (no path from that seed), from the seeds of `startup_retry_el_offsets` in order, then
whole degrees walked outward. `make_params(el_center)` builds the `/init` request seeded at
that centre elevation and `solve(params)` sends it, returning `(result, seed_trajectory)`.
A request the failure cache records as bad is skipped and costs no retry; a 422 is recorded.

Returns the result with the request and seed that produced it. Throws when every seed
within reach is cached as bad, or when the optimizer answered 422 to all that were sent;
both messages say what to try.
"""
function solve_startup(tos, make_params, solve, start_params, el_center_seed_base, l_set, winch,
                       inflow)
    # A 422 is retried from `startup_retry_el_offsets` in order; cached failures are skipped and cost no retry.
    opt_result = nothing
    opt_seed_trajectory = nothing
    el_center_seed = el_center_seed_base
    startup_seed_offset = 0.0
    guess_az = guess_el = nothing
    last_422 = nothing
    cached_msg = nothing
    sent = 0
    budget = 1 + length(tos.startup_retry_el_offsets)
    sent_offsets = Float64[]
    for offset in startup_seed_offsets(tos.startup_retry_el_offsets)
        sent < budget || break
        el_center = el_center_seed_base + offset
        params = offset == 0 ? start_params : make_params(el_center)
        cached = tos.opt_failure_cache ? opt_failed_before(params) : nothing
        if !isnothing(cached)
            cached_msg = @sprintf("This exact request failed before (%s, recorded \
                                   %s) and is cached as bad, so it was not sent: %s \
                                   at %.5f m, guess centred at %.0f°.",
                                  get(cached, "reason", "no reason recorded"),
                                  get(cached, "when", "at an unknown time"),
                                  tos.name, l_set, el_center)
            @warn cached_msg
            continue
        end
        sent += 1
        push!(sent_offsets, offset)
        sent == 1 ||
            @warn @sprintf("Startup solve at %.0f° failed: retry %d/%d from a \
                            guess centred at %.0f° (%+.1f°, startup_retry_el_offsets%s).",
                           el_center_seed_base, sent - 1, budget - 1, el_center, offset,
                           offset in tos.startup_retry_el_offsets ? "" :
                               " walked outward past the listed seeds")
        try
            opt_result, opt_seed_trajectory = solve(params)
            start_params = params
            el_center_seed = el_center
            startup_seed_offset = offset
            guess_az, guess_el = params.trajectory.azimuth, params.trajectory.elevation
            break
        catch exc
            exc isa HTTP.StatusError && exc.status == 422 || rethrow()
            tos.opt_failure_cache && record_opt_failure!(params, "422 from /step")
            last_422 = exc
        end
    end
    isnothing(opt_result) && isnothing(last_422) &&
        error(cached_msg * "\n\nEvery seed within reach of startup_retry_el_offsets \
              is cached as bad. Retry them with `clear_opt_failures()`, drop an entry \
              from $OPT_FAILURE_CACHE, or set opt_failure_cache: false in \
              data/traj_opt.yaml.")
    isnothing(opt_result) && error("""
          The optimizer returned no path: $(String(copy(last_422.response.body)))

          Three candidates, most likely first:
            * the INITIAL GUESS is too far from the optimum for IPOPT to reach \
              it. Here that is guess_a = $(tos.guess_a)°, guess_b = \
              $(tos.guess_b)°, guess_el_center = $(el_center_seed_base)° of \
              data/traj_opt.yaml$(length(sent_offsets) == 1 ? "" :
              ", and the retry seeds at offsets $(sent_offsets[2:end])° " *
              "failed too (startup_retry_el_offsets)"), which seeds the \
              request and nothing else — widening or raising it changes the \
              guess, not the flown path. Measured at 150 m and 6 m/s: 20°/11° \
              at 18° does not converge, 30°/12° and 20°/11°-at-26° do.
            * the winch is too stiff to reel out at the optimum: \
              kv*sqrt(f_high) = $(round(winch.k_v * sqrt(winch.f_max); digits = 1)) \
              m/s against $(inflow.wind_speed) m/s of wind at 6 m.
            * these conditions genuinely have no solution.

          `bin/run_server log` carries the solver's own output.""")
    return (; opt_result, opt_seed_trajectory, start_params, el_center_seed, startup_seed_offset,
            guess_az, guess_el)
end
