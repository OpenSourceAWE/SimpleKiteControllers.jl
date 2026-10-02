# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# One step of the reel-out loop of examples/simple_opt_reelout.jl, one function per block, in the
# order `step_commands!` calls them. Each takes the run's state `st` (`RunState`) and the fixed
# `setup` (the `RunSetup` of `setup_run`, src/run_setup.jl), plus the step's time `t` and whatever the
# earlier blocks of the same step produced, and returns what the later blocks read. The pure
# decisions they call are in loop_decisions.jl and reopt_gate.jl.
#
# None of them touches the kite model, so the package does not depend on it: what they need of it
# comes in `plant`, read once per step by the caller, `(; ss, dt, force, v_reel)` before `step!`
# (the `KiteUtils.SysState`, the time step [s], the winch force [N] and the reel-out speed [m/s])
# and `(; ss, dt, aoa, wind_factor_200)` after it, for `check_overspeed` and `record_step!`.

"""
    step_commands!(st, setup, plant, t) -> (; rel_depower, rel_steering, v_set, phase, guide, cmd)

Everything of one step of the reel-out loop before the model is stepped: the attractor guidance
(`guide`, from `navigate_fig8`, with the cross-track test input), the steering and depower
(`cmd`, see [`steering_command!`](@ref)), the lift target, the lap count, the re-optimization, the
path blends, the phase-5 fallback, the winch setpoint `v_set` and the steering hooks. `plant` is the
model as read at the start of the step, see the top of this file.
"""
function step_commands!(st::RunState, setup, plant, t)
    (; fcs, fec, tos) = setup
    ss = plant.ss
    # L0 attractor guidance -> commanded course [rad]; the lead is re-read every step.
    fec.fes.attractor_distance = attractor_distance(fcs, Float64(ss.v_app),
                                                    Float64(ss.l_tether[1]))
    chi_set, az_attr, el_attr, dmin =
        navigate_fig8(fec, Float64(ss.azimuth),
                      Float64(ss.elevation))
    chi_set, az_attr, el_attr = xtrack_input!(st, setup, plant, t, chi_set, az_attr, el_attr)

    cmd = steering_command!(st, setup, plant, t, chi_set, dmin)
    (; rel_depower, phase) = cmd
    el_target = update_lift_target!(st, setup, plant, t, phase)
    phase >= 4 && count_laps!(st, setup, plant, t)
    tos.reopt_enabled && phase == 4 && reoptimize!(st, setup, plant, t, phase, el_target)
    # OUTSIDE the re-optimization block: the in-air lift queues blends too, with re-optimization off and
    # into phase 5; inside it they were reported as delivered but never ran (2026-09-26).
    phase >= 4 && advance_blend!(st, setup, t)
    phase >= 4 && deliver_lift_in_air!(st, setup, plant, t, phase, el_target)
    phase5_fallback!(st, setup, t, phase)

    v_set = winch_setpoint!(st, setup, plant, t, phase, rel_depower)
    rel_steering = steering_hooks!(st, setup, t, cmd.rel_steering, cmd.u_ff)
    return (; rel_depower, rel_steering, v_set, phase, guide = (; chi_set, az_attr, el_attr, dmin), cmd)
end

"""
    xtrack_input!(st, setup, plant, t, chi_set, az_attr, el_attr) -> (chi_set, az_attr, el_attr)

The cross-track test input (`xtrack_offset`, from phase `xtrack_phase` on): moves the attractor
along the path normal and re-aims the guidance course at it, and logs the response in `st.xt_*`.
Returns the guidance unchanged without the input.
"""
function xtrack_input!(st::RunState, setup, plant, t, chi_set, az_attr, el_attr)
    (; xtrack_offset, xtrack_phase, fec) = setup
    ss = plant.ss
    (isnothing(xtrack_offset) || st.cc.phase < xtrack_phase) && return chi_set, az_attr, el_attr
    isnan(st.xt_start) && (st.xt_start = t)
    offset = xtrack_offset(t - st.xt_start)
    if offset != 0
        na, ne = path_normal(fec, attractor_index(fec))
        az_attr += offset * na / cosd(el_attr)
        el_attr += offset * ne
        chi_set = _bearing(Float64(ss.azimuth),
                                                 Float64(ss.elevation),
                                                 deg2rad(az_attr), deg2rad(el_attr))
    end
    push!(st.xt_t, t); push!(st.xt_delta, offset)
    push!(st.xt_d, signed_cross_track(fec, rad2deg(Float64(ss.azimuth)),
                                   rad2deg(Float64(ss.elevation))))
    push!(st.xt_q, fec.last_idx)
    push!(st.xt_phase, st.cc.phase); push!(st.xt_L, Float64(ss.l_tether[1]))
    push!(st.xt_va, Float64(ss.v_app)); push!(st.xt_dp, st.rel_depower_prev)
    push!(st.xt_vk, sqrt(max(norm(ss.vel_kite)^2 - Float64(ss.v_reelout[1])^2, 0.0)))
    return chi_set, az_attr, el_attr
end

"""
    steering_command!(st, setup, plant, t, chi_set, dmin)
        -> (; rel_steering, rel_depower, phase, u_ff, chi_cmd, w_lim, w_course, err)

Entry state machine, descent limiter, feedback fusion, PID and `rel_depower` (see `CourseController`),
with the gain scale and the curvature feed-forward, then the depower of [`depower_command!`](@ref).
The phase is the one after the switch to phase 5 when the reel-out is done.
"""
function steering_command!(st::RunState, setup, plant, t, chi_set, dmin)
    (; c1_setpoint, c1_depower_max, c1_ctrl_at, fcs, dt0, fec) = setup
    ss = plant.ss
    heading = Float64(ss.heading)
    v_kite = norm(ss.vel_kite)
    phase_before = st.cc.phase
    # Loop gain is heading_p * c1, so every phase flies heading_p * c1(setpoint)/c1(u_d), u_d rounded for the memo.
    gain_scale = loop_gain_scale(c1_setpoint, st.rel_depower_prev, c1_depower_max, c1_ctrl_at)
    # Curvature feed-forward plus chord correction, low-passed over ff_tau; see FC_Settings.feedforward.ff_gain.
    u_ff, chi_ff, ff_u_next, ff_chi_next =
        feedforward_step(fcs, dt0, fec, st.cc.phase, st.cc.err, Float64(ss.v_app),
                         Float64(ss.l_tether[1]), v_kite, dmin, c1_setpoint,
                         gain_scale, st.ff_u_filt, st.ff_chi_filt)
    st.ff_u_filt = ff_u_next
    st.ff_chi_filt = ff_chi_next
    push!(st.ff_log, u_ff)
    push!(st.ff_chi_log, chi_ff)
    rel_steering, rel_depower, phase = calc_steering(st.cc, chi_set, heading,
        Float64(ss.course);
        t, elevation = Float64(ss.elevation),
        v_kite, v_app = Float64(ss.v_app),
        dmin, tangent = path_tangent(fec), gain_scale, u_ff, chi_ff)
    phase_before == 2 && phase == 3 && (st.transition_start = t)
    rel_depower, phase = depower_command!(st, setup, plant, t, phase_before, phase, rel_depower)
    return (; rel_steering, rel_depower, phase, u_ff, chi_cmd = st.cc.chi_cmd, w_lim = st.cc.w_lim,
            w_course = st.cc.w_course, err = st.cc.err)
end

"""
    depower_command!(st, setup, plant, t, phase_before, phase, rel_depower) -> (rel_depower, phase)

The depower actually flown: the optimizer's, ramped over `path_blend_time` in phases 3 and 4; phase 5
once the reel-out is done; the soft-stop ramp toward `depower_final`; and the phase-5 force limiter.
"""
function depower_command!(st::RunState, setup, plant, t, phase_before, phase, rel_depower)
    (; tos, fcs) = setup
    ss = plant.ss
    # The optimizer's depower from phase 3 on, ramped over path_blend_time; phase 5 below still wins.
    if tos.fly_opt_depower && phase in (3, 4)
        # The entry ladder's depower is the FROM endpoint the first time, so the 2->3 hand-over ramps too.
        if phase_before < 3 && isnothing(st.depower_blend_to)
            st.depower_blend_from = rel_depower
            st.depower_blend_to = st.depower_flown_opt
            st.depower_blend_t0 = t
        end
        w_dp, dp_flown = blended_depower(st.depower_blend_from, st.depower_blend_to,
                                         st.depower_blend_t0, t, tos.path_blend_time,
                                         st.depower_flown_opt)
        st.depower_flown = dp_flown
        w_dp >= 1.0 && (st.depower_blend_to = nothing)
        rel_depower = st.depower_flown
    end
    # Separate from calc_steering's ladder so it can fire the SAME step as a 3->4 transition.
    if phase in (3, 4) && st.reelout_done
        set_phase!(st.cc, 5)
        phase = 5
        isnan(st.final_start) && (st.final_start = t)
    end
    # Ramps depower toward depower_final with the soft-stop, never BELOW the depower the stop latched at.
    if !isnan(st.stop_start)
        rel_depower = stop_depower(fcs, st.stop_dp_entry, st.stop_start, st.stop_T, t)
    elseif phase == 5
        rel_depower = fcs.reelout.depower_final
    end
    # Force limiter from the STOP LATCH on: integrates on the force the stopped drum is about to see.
    if fcs.reelout.depower_final_max > fcs.reelout.depower_final && (phase == 5 || !isnan(st.stop_start))
        ramping = !isnan(st.stop_start) && t - st.stop_start < st.stop_T
        st.dp_final_extra = final_force_extra(fcs, st.dp_final_extra, plant.force,
                                              Float64(ss.v_app),
                                              Float64(ss.v_reelout[1]),
                                              ramping, plant.dt)
        rel_depower = min(rel_depower + st.dp_final_extra, fcs.reelout.depower_final_max)
        st.dp_final_extra > st.dp_final_extra_peak && (st.dp_final_extra_peak = st.dp_final_extra)
    end
    return rel_depower, phase
end

"""
    update_lift_target!(st, setup, plant, t, phase) -> el_target

The elevation shift the path should carry [deg]: `el_offset_final`, latched at the stop latch (or
phase 5, see `lift_should_start`), else 0.
"""
function update_lift_target!(st::RunState, setup, plant, t, phase)
    (; fcs) = setup
    ss = plant.ss
    if !st.lift_on && phase >= 4
        if lift_should_start(fcs, st.stop_start, phase, Float64(ss.v_reelout[1]), st.l_set)
            st.lift_on = true
            st.lift_t = t
            st.lift_remaining = fcs.reelout.reelout_l_max - st.l_set
            @info @sprintf("Elevation lift of %+.2f° starting at t = %.1f s \
                            (%.1f m of reel-out left, phase %d).",
                           fcs.reelout.el_offset_final, t, fcs.reelout.reelout_l_max - st.l_set, phase)
        end
    end
    return st.lift_on ? fcs.reelout.el_offset_final : 0.0
end

"""
    count_laps!(st, setup, plant, t)

Phase 4 on: the live lap count `fig8_n` (1 the instant phase first reaches 4, then +1 per traversal,
unwrapped across the `mod1` wrap), the lap-1 upper force limit, and the droop bins.
"""
function count_laps!(st::RunState, setup, plant, t)
    (; fcs, fec, rcs, f_high_nominal) = setup
    ss = plant.ss
    if st.fig8_n == 0
        st.fig8_n = 1
        st.fig8_idx_prev = fec.last_idx
        st.t_phase4 = t   # V1 hook: this run's phase-4 start
        if fcs.winch.first_lap_force_frac < 1
            rcs.f_high = f_high_nominal * fcs.winch.first_lap_force_frac
            st.first_lap_f_high_applied = true
            @info @sprintf("Lap 1: upper force limit held at %.0f N \
                            (%.0f %% of %.0f N) for this lap.",
                           rcs.f_high, 100 * fcs.winch.first_lap_force_frac,
                           f_high_nominal)
        end
    else
        # A step moves Q by a fraction of a point; a jump is Q changing branch.
        st.fig8_idx_progress += lap_index_step(fec.last_idx, st.fig8_idx_prev, st.n_path)
        st.fig8_idx_prev = fec.last_idx
        # Never counted DOWN: Q can slip a fraction of a point backwards at an install.
        st.fig8_n = max(st.fig8_n, 1 + floor(Int, st.fig8_idx_progress / st.n_path))
        # Lap 1 only: the upper force limit is held down; `f_high_nominal` goes back on lap 2.
        if fcs.winch.first_lap_force_frac < 1 && st.fig8_n > 1 && st.first_lap_f_high_applied
            rcs.f_high = f_high_nominal
            st.first_lap_f_high_applied = false
            @info @sprintf("Lap %d: upper force limit back to %.0f N.",
                           st.fig8_n, f_high_nominal)
        end
    end

    az_lo, az_hi = extrema(fec.az_path)
    el_lo, el_hi = extrema(fec.el_path)
    az_amp, el_half = 0.5 * (az_hi - az_lo), 0.5 * (el_hi - el_lo)
    el_kite = rad2deg(Float64(ss.elevation))
    if az_amp > 0 && el_half > 0
        el_c = 0.5 * (el_hi + el_lo)
        bin = azimuth_bin(fec.az_path[fec.last_idx], az_lo, az_hi, st.n_droop_bins)
        st.droop_n[bin] += 1
        st.droop_flown[bin] += el_c - el_kite
        st.droop_ref[bin] += (el_c - fec.el_path[fec.last_idx]) / el_half
        st.droop_sag[bin] += el_kite - fec.el_path[fec.last_idx]
    end
    return nothing
end

# ---- Re-optimization (phase 4 only) ---------------------------------------------------------- #

"""
    reoptimize!(st, setup, plant, t, phase, el_target)

Re-optimize the path for the length now being flown: queue a request on a lap boundary
([`request_reopt!`](@ref)), then poll for the reply and gate and install it ([`collect_reopt!`](@ref)).
"""
function reoptimize!(st::RunState, setup, plant, t, phase, el_target)
    (; tos) = setup
    ss = plant.ss
    l_now = Float64(ss.l_tether[1])
    # Queue on a lap boundary, never while a solve or a blend is running, never past max_reopt.
    if !st.reopt_pending && isnothing(st.blend_to) && st.reopt_n < tos.max_reopt &&
       st.fig8_idx_progress >= (st.reopt_lap + tos.reopt_every_n_laps) * st.n_path
        request_reopt!(st, setup, t, phase, l_now)
    end
    # Collect: poll rather than block, and validate before installing.
    if st.reopt_pending && t >= st.reopt_next_poll
        collect_reopt!(st, setup, t, phase, l_now, el_target)
    end
    return nothing
end

"""
    request_reopt!(st, setup, t, phase, l_now)

Send a re-optimization request for `l_now`: a warm `/step` or a cold `/init` from the guess (see
`use_step`), with the turn-radius request and the pattern box rebuilt at this length. Blocking
(`reopt_blocking`) holds the simulation until the solve is over and retries a failed one from the
next seed. A refused request is recorded and the run flies on with the path it has.
"""
function request_reopt!(st::RunState, setup, t, phase, l_now)
    (; tos, fcs, opt_chain, opt_r_on, c1_at_phase, cap_wind, el_center_seed, winch_reopt,
       inflow) = setup
    try
        # Clocked from here, so a request that fails while being BUILT still has a start.
        st.reopt_t_wall_request = time()
        # Seeds: `nothing` is a warm `/step`, a number a cold `/init` from the guess (see `use_step`).
        el_seeds = if tos.use_step
            tos.reopt_blocking ? (nothing, el_center_seed) :
                                 (nothing,)
        elseif tos.reopt_blocking && tos.reopt_retry_el_offset != 0
            (el_center_seed,
             el_center_seed + tos.reopt_retry_el_offset)
        else
            (el_center_seed,)
        end
        # Asked for under the turn authority the reply will be JUDGED with (depower_final's c1 from phase 5).
        opt_r_on && (st.opt_r_min =
            min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                    c1 = c1_at_phase(phase, st)))
        # The floor moves with the length: box rebuilt per request, `size_box_growth` x the previous install.
        st.opt_box_now = with_size_box(
            pattern_limits_from(tos;
                elevation_min = elevation_min_request(fcs, tos, opt_length(tos, l_now);
                                                      extra = st.el_min_extra),
                wind_speed = cap_wind),
            st.opt_paths_raw[end]..., tos.size_box_growth)
        for (attempt, el_seed) in enumerate(el_seeds)
            if isnothing(el_seed)
                # `min_turn_radius` is re-sent because it MOVES with the length; `nothing` means "keep".
                chain_step(opt_chain,
                           StepParams(; length = opt_length(tos, l_now), winch_params = winch_reopt,
                                      min_turn_radius = st.opt_r_min,
                                      pattern_limits = st.opt_box_now);
                           wait = false)
            else
                guess_az_r, guess_el_r =
                    figure_eight_path(tos.guess_a, tos.guess_b,
                                      0.0, el_seed,
                                      0.0, tos.guess_points)
                reopt_params = InitParams(; name = tos.name, length = opt_length(tos, l_now),
                                          winch_params = winch_reopt,
                                          inflow_conditions = inflow,
                                          trajectory = Trajectory(collect(guess_az_r),
                                                                  collect(guess_el_r)),
                                          input_depower = depower_seed(tos, inflow.wind_speed),
                                          reg_weight = tos.reg_weight,
                                          detect_simple_bounds = tos.detect_simple_bounds,
                                          min_turn_radius = st.opt_r_min,
                                          pattern_limits = st.opt_box_now)
                # A known failure is served by `opt_chain`, not skipped here: skipping would leave
                # the chain on the warm lineage, and every later step would miss the cache.
                reopt_reply = chain_init(opt_chain, reopt_params)
                chain_step(opt_chain,
                           StepParams(opt_length(tos, l_now), winch_reopt, reopt_reply.trajectory);
                           wait = false)
            end
            st.reopt_pending = true
            st.reopt_t_request = t
            st.reopt_lap = st.fig8_idx_progress / st.n_path
            st.reopt_next_poll = t + tos.reopt_poll_interval
            @info @sprintf("Re-optimizing for L = %.0f m at t = %.1f s \
                            (lap %.1f, request %d of %d, %s)%s%s.",
                           l_now, t, st.reopt_lap, st.reopt_n + 1, tos.max_reopt,
                           isnothing(el_seed) ? "warm start" :
                               @sprintf("guess el %.0f°", el_seed),
                           isnothing(st.opt_box_now) ? "" :
                               @sprintf(", box |az| <= %s, el %s..%s, half-span <= %s",
                                        isnothing(st.opt_box_now.azimuth_max) ? "-" :
                                            @sprintf("%.1f°", st.opt_box_now.azimuth_max),
                                        isnothing(st.opt_box_now.elevation_min) ? "-" :
                                            @sprintf("%.1f°", st.opt_box_now.elevation_min),
                                        isnothing(st.opt_box_now.elevation_max) ? "-" :
                                            @sprintf("%.1f°", st.opt_box_now.elevation_max),
                                        isnothing(st.opt_box_now.elevation_amplitude_max) ? "-" :
                                            @sprintf("%.1f°", st.opt_box_now.elevation_amplitude_max)),
                           tos.reopt_blocking ? " — holding the simulation" : "")
            # Freeze here, so the reply is anchored to `l_now` and not to a length the run drifted to.
            tos.reopt_blocking || break
            t_block = time()
            while (try
                       chain_status(opt_chain)["state"]
                   catch exc
                       @warn "Could not reach the optimizer while \
                              holding; will retry." exception = exc
                       "solving"
                   end) == "solving"
                sleep(tos.reopt_poll_interval)
            end
            st.reopt_last_solve_s = time() - t_block
            st.reopt_blocked_s += st.reopt_last_solve_s
            # Collect on THIS step: the reply is already on the server.
            st.reopt_next_poll = t
            # Retry only a solver failure, and only while a seed is left.
            failed = (try
                          chain_status(opt_chain)["state"]
                      catch; "failed"; end) == "failed"
            (failed && attempt < length(el_seeds)) || break
            @info @sprintf("  ... failed from %s; retrying from %s.",
                           isnothing(el_seed) ? "the warm start" :
                               @sprintf("guess el %.0f°", el_seed),
                           @sprintf("guess el %.0f°", el_seeds[attempt + 1]))
        end
    catch exc
        # A refused request must not take the run with it: the path in the air is still flyable.
        st.reopt_n += 1
        push!(st.reopt_events, (; t, l = l_now, status = "request failed",
                             detail = first(sprint(showerror, exc), 120)))
        push!(st.reopt_cycles, (; t, l = l_now, status = "request failed",
                             wall_s = time() - st.reopt_t_wall_request))
        @warn "Re-optimization request failed; flying on with the \
               current path." exception = exc
    end
    return nothing
end

"""
    collect_reopt!(st, setup, t, phase, l_now, el_target)

Poll the optimizer for the pending reply; once the solve is over, gate and install a converged one
([`gate_and_install!`](@ref)) and record the cycle in `st.reopt_events` and `st.reopt_cycles`.
"""
function collect_reopt!(st::RunState, setup, t, phase, l_now, el_target)
    (; tos, opt_chain) = setup
    st.reopt_next_poll = t + tos.reopt_poll_interval
    state = try
        chain_status(opt_chain)["state"]
    catch exc
        @warn "Could not reach the optimizer; will retry." exception = exc
        "solving"
    end
    state == "solving" && return nothing
    st.reopt_pending = false
    st.reopt_n += 1
    event = (; t, l = l_now, status = state, detail = "")
    if state == "converged"
        event = gate_and_install!(st, setup, t, phase, l_now, el_target, event)
    end
    push!(st.reopt_events, event)
    # Non-blocking: an upper bound on the solve, by at most one `reopt_poll_interval`.
    push!(st.reopt_cycles, (; t, l = l_now, status = event.status,
                         wall_s = time() - st.reopt_t_wall_request))
    # Blocking collects on the SAME step as the request, so the wall time is the figure that counts.
    @info @sprintf("Re-optimization %d: %s%s (%s).",
                   st.reopt_n, event.status,
                   isempty(event.detail) ? "" : " — " * event.detail,
                   tos.reopt_blocking ?
                       @sprintf("%.1f s of wall time, held", st.reopt_last_solve_s) :
                       @sprintf("%.1f s of sim after the request",
                                t - st.reopt_t_request))
    return nothing
end

"""
    gate_and_install!(st, setup, t, phase, l_now, el_target, event) -> event

The converged reply on the server through the accept gate ([`evaluate_candidate!`](@ref)): installed
when it passes, re-asked from a cold start ([`cold_retry!`](@ref)) when the gate says `:retry`, at
most `blend_max_retries` times, dropped when it says `:reject`. Returns the cycle's event; `event`
as given when the retries run out.
"""
function gate_and_install!(st::RunState, setup, t, phase, l_now, el_target, event)
    (; tos, opt_chain) = setup
    tab = chain_trajectory(opt_chain)
    # k_v and input_depower are applied only in the accept gate below, from the `tab` that passes it.
    # A reply whose blend folds is not flown: a fresh COLD reply is requested, `blend_max_retries` times at most.
    reject_reason = ""
    # A clearance/elevation rejection retries the FLOOR, not the guess; `el_min_extra` carries the shortfall.
    reject_low = false
    # Frozen for the retry chain; the first entry is the startup solve, which `min_power_frac_prev` skips.
    prev_install_pred = length(st.pred_timeline) > 1 ?
        st.pred_timeline[end].power : NaN
    for blend_attempt in 0:tos.blend_max_retries
        if blend_attempt > 0
            retry_state = cold_retry!(st, setup, l_now, blend_attempt, reject_low, reject_reason)
            if retry_state != "converged"
                @warn @sprintf("Blend-fold retry %d of %d for L = \
                                %.0f m did not converge (%s); giving \
                                up on this cycle.",
                               blend_attempt, tos.blend_max_retries,
                               l_now, retry_state)
                event = (; t, l = l_now, status = "rejected",
                         detail = @sprintf("blend-fold retry %d did \
                                            not converge (%s)",
                                           blend_attempt, retry_state))
                break
            end
            tab = chain_trajectory(opt_chain)
            # k_v and input_depower are applied only in the accept gate below, see above.
        end
        cand = evaluate_candidate!(st, setup, tab, phase, l_now, el_target, blend_attempt,
                                   prev_install_pred)
        gate = cand.gate
        if gate.verdict == :retry
            # A height shortfall is re-asked at a raised floor; any other retry gets a fresh reply.
            isnothing(gate.raise) || (st.el_min_extra += gate.raise)
            reject_reason = gate.reason
            reject_low = gate.low
            continue   # at the top
        elseif gate.verdict == :reject
            event = (; t, l = l_now, status = "rejected", detail = gate.detail)
            break
        else
            event = install_candidate!(st, setup, t, phase, l_now, el_target, tab, cand)
            break
        end
    end
    return event
end

"""
    cold_retry!(st, setup, l_now, blend_attempt, reject_low, reject_reason) -> state

A fresh COLD solve for `l_now` after a rejected candidate, from the guess moved by
`reopt_retry_el_offset` (up after a height shortfall, else alternating) and under the floor raised
by `el_min_extra`. Holds the simulation until the solve is over; returns the optimizer's state.
"""
function cold_retry!(st::RunState, setup, l_now, blend_attempt, reject_low, reject_reason)
    (; tos, fcs, opt_chain, el_center_seed, winch_reopt, inflow, cap_wind) = setup
    st.blend_retries_total += 1
    # Alternating +/- `reopt_retry_el_offset`, never scaled UP by `blend_attempt`.
    retry_el_seed = el_center_seed +
        (reject_low || isodd(blend_attempt) ? 1 : -1) *
        tos.reopt_retry_el_offset
    retry_el_min = elevation_min_request(fcs, tos, opt_length(tos, l_now);
                                         extra = st.el_min_extra)
    @info @sprintf("  ... candidate at L = %.0f m rejected \
                    (%s); cold-restarting from guess el \
                    %.0f°%s (retry %d of %d), holding the \
                    simulation.",
                   l_now, reject_reason, retry_el_seed,
                   isnothing(retry_el_min) ? "" :
                       @sprintf(", floor %.1f°%s", retry_el_min,
                                st.el_min_extra > 0 ?
                                    @sprintf(" (+%.1f° for the \
                                              shortfall)",
                                             st.el_min_extra) : ""),
                   blend_attempt, tos.blend_max_retries)
    retry_az, retry_el = figure_eight_path(tos.guess_a,
        tos.guess_b, 0.0,
        retry_el_seed, 0.0, tos.guess_points)
    retry_params = InitParams(; name = tos.name, length = opt_length(tos, l_now),
        winch_params = winch_reopt, inflow_conditions = inflow,
        trajectory = Trajectory(collect(retry_az),
                                collect(retry_el)),
        input_depower = depower_seed(tos, inflow.wind_speed),
        reg_weight = tos.reg_weight,
        detect_simple_bounds = tos.detect_simple_bounds,
        min_turn_radius = st.opt_r_min,
        pattern_limits = with_size_box(
            pattern_limits_from(tos;
                elevation_min = retry_el_min,
                wind_speed = cap_wind),
            st.opt_paths_raw[end]..., tos.size_box_growth))
    retry_reply = chain_init(opt_chain, retry_params)
    chain_step(opt_chain,
               StepParams(opt_length(tos, l_now), winch_reopt, retry_reply.trajectory);
               wait = false)
    t_retry = time()
    retry_state = "solving"
    while retry_state == "solving"
        sleep(tos.reopt_poll_interval)
        retry_state = try
            chain_status(opt_chain)["state"]
        catch exc
            @warn "Could not reach the optimizer while \
                   retrying a folded blend; will retry." exception = exc
            "solving"
        end
    end
    st.reopt_blocked_s += time() - t_retry
    return retry_state
end

"""
    evaluate_candidate!(st, setup, tab, phase, l_now, el_target, blend_attempt, prev_install_pred)
        -> (; gate, cand_raw, cand_az, cand_el, cand_from, chk_az, chk_el, margin, clearance,
              new_pred, cand_size, wing_frac)

The reply `tab` as the path it would be flown as (lifted by `el_target` and the lobe lift, rationed
to fit the curvature gate), checked at the reply's own resolution and at the current length, and
the verdict of `gate_candidate`. Also re-measures `st.opt_r_scale` off the reply and sets
`st.chk_points`.
"""
function evaluate_candidate!(st::RunState, setup, tab, phase, l_now, el_target, blend_attempt,
                             prev_install_pred)
    (; tos, fcs, fec, feas, el_floor, opt_r_on, wing_lift, c1_at_phase, power_gate_off) = setup
    # Re-measure the anchor SCALE off the reply; the radius itself is derived where the request goes out.
    opt_r_on && (st.opt_r_scale = reelout_anchor_ratio(tab) *
                                      tos.turn_radius_headroom)
    # What the optimizer measured, in the request's metres; the gate reads the same curve AT THE ANCHOR.
    opt_r_reply = opt_float(tab["metrics"], "turn_radius_min_m")
    r_span = extrema(Float64.(tab["table"]["distance_radial"]))
    # /trajectory is in RADIANS, unlike the degrees of the structs.
    new_az = rad2deg.(Float64.(tab["table"]["azimuth"]))
    new_el = rad2deg.(Float64.(tab["table"]["elevation"]))
    # Lifted BEFORE the gates, which must score the curve that will be flown.
    cand_raw = (copy(new_az), copy(new_el))
    # The rigid lift goes in whole; the lobe lift is rationed to fit the curvature gate.
    wing_delta = wing_lift(new_az, new_el)
    n_native = min(tos.resample_points, length(new_az) - 1)
    # The turn authority THIS reply will be flown with, read off `tab`.
    cand_c1 = c1_at_phase(phase, tos.fly_opt_depower ?
        awetrim_depower_to_v3kite(
            Float64(tab["optimized_parameters"]["input_depower"])) :
        fcs.course.depower_setpoint)
    lifted(fw) = new_el .+ el_target .+ fw .* wing_delta
    function lifted_margin(el_try)
        isnan(feas.c1) && return Inf
        az_chk, el_chk = prepare_path(new_az, el_try; resample = n_native,
                                      up_loops = fcs.pattern.up_loops)
        check_pattern_feasible(az_chk, el_chk, l_now, fcs.course.max_steering;
                               c1 = cand_c1, prn = false).margin
    end
    wing_frac = 1.0
    for fw in (1.0, 0.75, 0.5, 0.25, 0.0)
        wing_frac = fw
        lifted_margin(lifted(fw)) >= tos.min_feasibility_margin &&
            break
    end
    new_el = lifted(wing_frac)
    wing_frac < 1 &&
        @info @sprintf("Lobe lift held back on the path for L = %.0f m \
                        to fit the curvature gate: %.0f %% of %.2f°.",
                       l_now, 100 * wing_frac, fcs.reelout.el_offset_wing)
    # TWO resolutions: the CHECKS at the reply's own, what is FLOWN at `n_path` so the lap counter holds.
    chk_az, chk_el = prepare_path(new_az, new_el;
        resample = n_native, up_loops = fcs.pattern.up_loops)
    st.chk_points = n_native
    cand_az, cand_el = prepare_path(new_az, new_el;
        resample = st.n_path, up_loops = fcs.pattern.up_loops)
    # Canonicalized like `cand_az`/`cand_el`, so `blend_folds` and `blend_paths` pair the same points.
    cand_from = prepare_path(fec.az_path, fec.el_path;
        resample = st.n_path, up_loops = fcs.pattern.up_loops)
    # At the CURRENT length, which is what it will be flown at.
    margin = isnan(feas.c1) ? Inf :
        check_pattern_feasible(chk_az, chk_el, l_now,
            fcs.course.max_steering; c1 = cand_c1, prn = false).margin
    clearance = path_min_height(chk_az, chk_el, l_now)
    # Gated against BOTH the startup prediction and the previous install's (`min_power_frac*`).
    new_pred = Float64(tab["metrics"]["avg_power_W"])
    cand_folds = blend_folds(tos, cand_from..., cand_az, cand_el)
    # Raw against raw: the reply's curve against the previous install's, before either carries a lift.
    cand_size = pattern_size_growth(st.opt_paths_raw[end]..., cand_raw...)
    # The accept gate: turn margin, clearance, elevation floor, blend fold and power, size growth.
    gate = gate_candidate(tos, (; margin, clearance, l_now,
                                chk_el_min = minimum(chk_el), el_floor,
                                folds = cand_folds, new_pred, st.opt_power_pred,
                                prev_install_pred,
                                power_gate_off = power_gate_off(new_pred),
                                size = cand_size, blend_attempt, opt_r_reply,
                                r_span, st.opt_r_min))
    return (; gate, cand_raw, cand_az, cand_el, cand_from, chk_az, chk_el, margin, clearance,
            new_pred, cand_size, wing_frac)
end

"""
    apply_optimized_kv!(setup, tab, t, l_now)

Move the winch gain the optimizer chose out of a `/trajectory` reply and into the
`WCSettings` the run reads, so it flies the `k_v` the path was solved for. `wc`,
`rcs` and `rc.wcs` are one object, and every sub-controller of `rc` holds a
reference to it, so a single assignment reaches all of them. A reply that did not
optimize the gain carries no `k_v` under `optimized_parameters` and this is then a
no-op. A gain that ran into its own bracket is reported: the value is the edge of
the box, not an optimum.
"""
function apply_optimized_kv!(setup, tab, t, l_now)
    (; tos, wc, rc, opt_kv_log) = setup
    tos.optimize_k_v || return
    params = get(tab, "optimized_parameters", nothing)
    raw = params === nothing ? nothing : get(params, "k_v", nothing)
    raw === nothing && return
    k_v = Float64(raw)
    k_v > 0 || return
    at_bound = something(get(params, "k_v_at_bound", false), false)
    if isempty(opt_kv_log) || abs(k_v - last(opt_kv_log).k_v) > 1e-9
        @info @sprintf("  ... optimizer chose k_v = %.5f at L = %.0f m (was %.5f)%s",
                       k_v, l_now, wc.kv, at_bound ? " — AT ITS BRACKET EDGE" : "")
        at_bound && @warn "k_v hit the K_V_BRACKET_FACTOR bound: the optimizer wanted \
                           to retune further than it was allowed, so this is the edge \
                           of the box rather than an optimum."
    end
    wc.kv = k_v
    @assert rc.wcs === wc "the reel-out controller must read the WCSettings the gain is written to"
    # EVERY accepted install with a gain, repeats included; a rejected candidate never reaches this function.
    push!(opt_kv_log, (; t, l = l_now, k_v, at_bound))
    return
end

"""
    install_candidate!(st, setup, t, phase, l_now, el_target, tab, cand) -> event

Install a candidate that passed the gate: queue the blend from the aligned old path, move the scored
reference, the optimizer's depower and `k_v` with it, re-base the lap counter, and record the install
and its phase-5 margin. Returns the cycle's event.
"""
function install_candidate!(st::RunState, setup, t, phase, l_now, el_target, tab, cand)
    (; tos, fcs, fec, opt_chain, opt_depower_log, margin5, phase5_margin_at, power_gate_off,
       project_set) = setup
    (; cand_raw, cand_az, cand_el, cand_from, chk_az, chk_el, margin, clearance, new_pred,
       cand_size, wing_frac) = cand
    power_gate_off(new_pred) &&
        @info @sprintf("  ... power gate bypassed at L = %.0f m \
                        (%.0f W predicted, %.1f m/s < \
                        power_gate_wind_min %.1f): installing anyway.",
                       l_now, new_pred, project_set.v_wind,
                       tos.power_gate_wind_min)
    st.blend_from = cand_from
    st.blend_to = (cand_az, cand_el)
    # The scored reference follows the same ramp, unlifted curve to unlifted curve.
    st.raw_from = prepare_path(st.raw_az, st.raw_el;
        resample = st.n_path, up_loops = fcs.pattern.up_loops)
    st.raw_to = prepare_path(cand_raw[1], cand_raw[2];
        resample = st.n_path, up_loops = fcs.pattern.up_loops)
    st.raw_az, st.raw_el = st.raw_from
    # Here, not before the gates: a REJECTED reply is not a path the kite ever flies.
    push!(st.opt_paths_raw, cand_raw)
    push!(st.opt_paths_at, (t, phase))
    st.blend_t0 = t
    # k_v and input_depower move only for the `tab` that made it here.
    let l_dp = Float64(tab["optimized_parameters"]["input_depower"])
        st.depower_flown_opt = awetrim_depower_to_v3kite(l_dp)
        st.depower_blend_from = st.depower_flown
        st.depower_blend_to = st.depower_flown_opt
        st.depower_blend_t0 = t
        push!(opt_depower_log, (; t, l_dp, u_p_equiv = st.depower_flown_opt))
    end
    apply_optimized_kv!(setup, tab, t, l_now)
    record_opt_success!(opt_chain)
    abs(el_target - st.el_applied) > 1e-6 &&
        push!(st.el_shift_events,
              (; t, delta = el_target - st.el_applied, margin,
               status = "carried by an install"))
    st.el_applied = el_target
    # Arm the in-air warning again: one warning per SHIFT, not per run.
    st.el_shift_warned = false
    # Install the aligned OLD path (w = 0, new point indices) and re-base the lap counter on it.
    set_path!(fec, st.blend_from[1], st.blend_from[2];
              up_loops = fcs.pattern.up_loops)
    st.fig8_idx_prev = fec.last_idx
    # A no-op while paths are resampled to `n_path`; `fig8_idx_progress` counts POINTS.
    n_path_new = length(fec.az_path)
    st.fig8_idx_progress *= n_path_new / st.n_path
    st.n_path = n_path_new
    push!(st.pred_timeline, (t = t, power = new_pred))
    margin5.margin = phase5_margin_at(chk_az, chk_el)
    push!(st.p5_history, (t, az = copy(cand_az), el = copy(cand_el), raw = cand_raw,
                       margin = margin5.margin, st.el_applied))
    # Not a rejection reason: said once, so a phase 5 flown on the clamp is not a surprise.
    if !isnan(margin5.margin) &&
       margin5.margin < tos.min_feasibility_margin && !margin5.warned
        margin5.warned = true
        @warn @sprintf("The path installed at %.0f m has a \
                        curvature margin of %.2f at \
                        depower_final (%.2f here at \
                        depower_setpoint), below \
                        min_feasibility_margin = %.2f: phase 5 \
                        will fly it with less turn authority \
                        than any gate has checked.",
                       l_now, margin5.margin, margin,
                       tos.min_feasibility_margin)
    end
    return (; t, l = l_now, status = "installed",
            detail = @sprintf("margin %.2f%s%s, clearance %.1f m, \
                               size x%.2f, %.0f W predicted", margin,
                              wing_frac < 1 ?
                                  @sprintf(" (lobe lift at %.0f %%)",
                                           100 * wing_frac) : "",
                              isnan(margin5.margin) ? "" :
                                  @sprintf(" (phase 5: %.2f at %.0f m)",
                                           margin5.margin,
                                           fcs.reelout.reelout_l_max),
                              clearance, cand_size.growth, new_pred))
end

# ---- Path blends ------------------------------------------------------------------------------ #

"""
    advance_blend!(st, setup, t)

Phase 4 on: step the running path blend (`blend_from` -> `blend_to` over `path_blend_time`) and the
scored reference with it; clear both at the end. `blend_to` is guaranteed fold-free across all of
w by whoever queued it, so a plain linear ramp.
"""
function advance_blend!(st::RunState, setup, t)
    (; tos, fcs, fec) = setup
    isnothing(st.blend_to) && return nothing
    weight = clamp((t - st.blend_t0) / tos.path_blend_time, 0.0, 1.0)
    b_az, b_el = blend_paths(st.blend_from[1], st.blend_from[2],
                             st.blend_to[1], st.blend_to[2], weight)
    set_path!(fec, b_az, b_el; up_loops = fcs.pattern.up_loops)
    if !isnothing(st.raw_to)
        st.raw_az, st.raw_el = blend_paths(st.raw_from[1], st.raw_from[2],
                                            st.raw_to[1], st.raw_to[2], weight)
    end
    if weight >= 1
        st.blend_from = nothing
        st.blend_to = nothing
        st.raw_from = nothing
        st.raw_to = nothing
    end
    return nothing
end

"""
    deliver_lift_in_air!(st, setup, plant, t, phase, el_target)

Phase 4 on: when the path in the air lacks the elevation shift `el_target` and no install is due,
queue it as a blend onto the path in the air, rationed down to a quarter if the whole shift fails
the curvature margin or folds the blend, once per lap and target. Runs AFTER the re-optimizer,
which has first claim on `blend_to`.
"""
function deliver_lift_in_air!(st::RunState, setup, plant, t, phase, el_target)
    (; tos, fcs, fec, feas, c1_at_phase) = setup
    ss = plant.ss
    # The shift reaches the kite at an install or, when none is due, as a blend onto the path in the air.
    el_delta = el_target - st.el_applied
    (abs(el_delta) > 1e-6 && isnothing(st.blend_to) && !st.reopt_pending &&
     !(st.fig8_n == st.el_shift_lap && el_target == st.el_shift_target)) || return nothing
    st.el_shift_lap = st.fig8_n
    st.el_shift_target = el_target
    # Scored at `chk_points`, the resolution the path in the air came at; only the CHECK is downsampled.
    chk_n = min(st.chk_points, length(fec.az_path) - 1)
    function shift_margin(el_try)
        isnan(feas.c1) && return Inf
        az_chk, el_chk = prepare_path(fec.az_path, el_try; resample = chk_n,
                                      up_loops = fcs.pattern.up_loops)
        check_pattern_feasible(az_chk, el_chk, Float64(ss.l_tether[1]),
            fcs.course.max_steering; c1 = c1_at_phase(phase, st), prn = false).margin
    end
    # Rationed down to a quarter of the shift, never below.
    # A rung that clears the margin can still fold `blend_paths` in between, so that is checked too.
    hit = nothing
    margin = NaN
    for fm in (1.0, 0.75, 0.5, 0.25)
        el_rung = fec.el_path .+ fm * el_delta
        rung_margin = shift_margin(el_rung)
        isnan(margin) && (margin = rung_margin)   # the WHOLE shift's margin, reported
        if rung_margin >= tos.min_feasibility_margin &&
           !blend_folds(tos, fec.az_path, fec.el_path, fec.az_path, el_rung)
            hit = (fm, el_rung, rung_margin)
            break
        end
    end
    # A rung that moves the path by no more than a hundredth of a degree is not a delivery, and
    # nothing held it back either: no event, no warning.
    !isnothing(hit) && abs(hit[1] * el_delta) <= 0.01 && return nothing
    if !isnothing(hit)
        fm, shifted, hit_margin = hit
        st.blend_from = (copy(fec.az_path), copy(fec.el_path))
        st.blend_to = (copy(fec.az_path), shifted)
        st.blend_t0 = t
        went_in = fm * el_delta
        push!(st.el_shift_events, (; t, delta = went_in,
                                margin = hit_margin,
                                status = fm < 1 ?
                                    @sprintf("blended in (%.0f %%)", 100 * fm) :
                                    "blended in"))
        st.el_applied = st.el_applied + went_in
        st.el_shift_warned = false
        fm < 1 &&
            @info @sprintf("Elevation shift rationed to fit the curvature \
                            gate: %.0f %% of %+.2f°; the rest is retried \
                            next lap.", 100 * fm, el_delta)
    elseif !st.el_shift_warned
        push!(st.el_shift_events, (; t, delta = el_delta,
                                margin, status = "held back"))
        st.el_shift_warned = true
        @warn @sprintf("Elevation shift of %+.2f° held back: the curvature \
                        margin would be %.2f even rationed to a quarter. \
                        Retrying as the tether grows.", el_delta, margin)
    end
    return nothing
end

"""
    phase5_fallback!(st, setup, t, phase)

Phase-5 path: fall back to an install that phase 5 can fly (`final_margin_min`). From the stop
latch, once no other blend is running; checked once. Phase 5 makes no power, so the smaller, later
paths that saturate the steering there buy nothing.

Only as Q passes the crossing (its azimuth changes sign about the path centre): the startup path is
far taller at full length (attractor up to ~36° vs ~18°), and blended in mid-lobe the kite fell 14°
behind it and spun an extra loop (Maasvlakte 8.25 m/s, 2026-09-26).
"""
function phase5_fallback!(st::RunState, setup, t, phase)
    (; fcs, tos, fec) = setup
    (fcs.reelout.final_margin_min > 0 && !st.p5_fallback_done) || return nothing
    p5_crossing = false
    if !isnan(st.stop_start) || phase >= 5
        az_q = fec.az_path[fec.last_idx] - (minimum(fec.az_path) + maximum(fec.az_path)) / 2
        p5_crossing = !isnan(st.p5_q_az_prev) && signbit(az_q) != signbit(st.p5_q_az_prev)
        st.p5_q_az_prev = az_q
    end
    (p5_crossing && isnothing(st.blend_to) && !st.reopt_pending) || return nothing
    st.p5_fallback_done = true
    # Native margins, as each install computed them: on the 360-point resampled path a
    # 100-point reply reads about half its margin, an artefact of the resampling's kinks.
    m_now = st.p5_history[end].margin
    (!isnan(m_now) && m_now < fcs.reelout.final_margin_min) || return nothing
    i_ok = findlast(entry -> !isnan(entry.margin) && entry.margin >= fcs.reelout.final_margin_min, st.p5_history)
    if isnothing(i_ok)
        @warn @sprintf("Phase-5 margin %.2f < final_margin_min %.2f and no earlier \
                        install meets it: flying phase 5 on the current path.",
                       m_now, fcs.reelout.final_margin_min)
        return nothing
    end
    install = st.p5_history[i_ok]
    # The lift the kite carries now, not the one that install was made with.
    to = prepare_path(install.az, install.el .+ (st.el_applied - install.el_applied);
                      resample = st.n_path, up_loops = fcs.pattern.up_loops)
    from = prepare_path(fec.az_path, fec.el_path;
                        resample = st.n_path, up_loops = fcs.pattern.up_loops)
    m_to = install.margin
    if blend_folds(tos, from..., to...)
        @warn @sprintf("Phase-5 fallback to the path installed at t = %.1f s \
                        skipped: the blend would fold.", install.t)
        return nothing
    end
    st.blend_from = from
    st.blend_to = to
    st.blend_t0 = t
    st.raw_from = prepare_path(st.raw_az, st.raw_el; resample = st.n_path,
                                   up_loops = fcs.pattern.up_loops)
    st.raw_to = prepare_path(install.raw[1], install.raw[2]; resample = st.n_path,
                                 up_loops = fcs.pattern.up_loops)
    st.raw_az, st.raw_el = st.raw_from
    # Install the aligned current path (w = 0) and re-base the lap counter, as an install does.
    set_path!(fec, st.blend_from[1], st.blend_from[2]; up_loops = fcs.pattern.up_loops)
    st.fig8_idx_prev = fec.last_idx
    st.p5_fallback = (; t, from_margin = m_now, to_margin = m_to, to_t = install.t)
    @info @sprintf("Phase-5 fallback at t = %.1f s: the flown path has a \
                    phase-5 margin of %.2f, below final_margin_min = %.2f; \
                    blending to the path installed at t = %.1f s \
                    (margin %.2f).", t, m_now, fcs.reelout.final_margin_min,
                   install.t, m_to)
    return nothing
end

# ---- Winch ------------------------------------------------------------------------------------ #

"""
    winch_setpoint!(st, setup, plant, t, phase, rel_depower) -> v_set

The tether's speed setpoint [m/s], with `st.l_set` advanced by it: the reel-out once released
([`release_reelout!`](@ref), [`reelout_speed!`](@ref)), the force floor before it
([`entry_force_guard!`](@ref)), and the compliant hold of phase 5 ([`compliant_hold!`](@ref)).
"""
function winch_setpoint!(st::RunState, setup, plant, t, phase, rel_depower)
    v_set = 0.0
    phase >= 3 && !st.reelout_started && release_reelout!(st, setup, plant, t)
    if st.reelout_started && !st.reelout_done
        v_set = reelout_speed!(st, setup, plant, t, rel_depower)
    elseif phase < 3
        v_set = entry_force_guard!(st, setup, plant, v_set)
    end
    if !isnothing(setup.hold_compliance) && st.reelout_done && phase == 5
        v_set = compliant_hold!(st, setup, plant)
    end
    return v_set
end

"""
    release_reelout!(st, setup, plant, t)

`REEL_OUT` is released `reelout_delay` seconds after phase 3, or early by `reelout_f_trigger`. The gate
LATCHES: once open it never re-closes.
"""
function release_reelout!(st::RunState, setup, plant, t)
    (; fcs) = setup
    ss = plant.ss
    by_timer, by_force = reelout_release(fcs, t, st.transition_start, plant.force)
    if by_timer || by_force
        st.reelout_started = true
        st.reelout_start_t = t
        st.reelout_trigger_fired = by_force && !by_timer
        by_force && !by_timer &&
            @info @sprintf("  ... reel-out released EARLY at t = %.1f s by \
                            force %.0f N >= %.0f N (%.1f s before the \
                            %.1f s delay would have).",
                           t, plant.force, fcs.reelout.reelout_f_trigger,
                           st.transition_start + fcs.reelout.reelout_delay - t,
                           fcs.reelout.reelout_delay)
    end
    return nothing
end

"""
    reelout_speed!(st, setup, plant, t, rel_depower) -> v_set

The reel-out speed while reeling out, until `l_set` reaches `reelout_l_max` or the laps are flown:
the force law of the winch controller, soft-started, and the soft-stop that latches once
`reelout_softstop` seconds would cover the rest.
"""
function reelout_speed!(st::RunState, setup, plant, t, rel_depower)
    (; fcs, rc, rcs) = setup
    ss = plant.ss
    # The INSTANTANEOUS force: reeling out faster when the kite pulls harder is what regulates the force.
    v_raw = calc_v_set(rc, plant.v_reel, plant.force, rcs.f_low)
    # Ramps the COMMAND, not the law, from when the gate OPENED; `t_startup` does not do this,
    # but released in proportion to tether load, so the soft-start never overrides the force limiter.
    v_cmd = reelout_command(fcs, v_raw, t, st.reelout_start_t, plant.force,
                            rcs.f_low, rcs.f_high)

    remaining = fcs.reelout.reelout_l_max - st.l_set
    # Soft-stop: latch once `reelout_softstop` seconds would cover the rest, then decelerate linearly to 0.
    if isnan(st.stop_start) && fcs.reelout.reelout_softstop > 0 && v_cmd > 0 &&
       remaining <= v_cmd * fcs.reelout.reelout_softstop
        st.stop_start = t
        st.stop_v_entry = v_cmd
        st.stop_dp_entry = rel_depower
        st.stop_T = 2 * remaining / v_cmd
    end
    # Second stop criterion: N COMPLETE laps by `fig8_idx_progress` (`fig8_n` reads 1 during the first lap).
    if isnan(st.stop_start) && fcs.reelout.n_fig_eight > 0 &&
       st.fig8_idx_progress >= fcs.reelout.n_fig_eight * st.n_path
        st.stop_reason = "laps"
        if fcs.reelout.reelout_softstop > 0 && v_cmd > 0
            st.stop_start = t
            st.stop_v_entry = v_cmd
            st.stop_dp_entry = rel_depower
            # No remaining distance to solve T from: same nominal duration instead.
            st.stop_T = 2 * fcs.reelout.reelout_softstop
        else
            st.reelout_done = true   # hard stop, as reelout_l_max does today
        end
    end
    v_set = isnan(st.stop_start) ? v_cmd :
        soft_stop_speed(st.stop_v_entry, t, st.stop_start, st.stop_T)
    st.l_set = min(st.l_set + v_set * plant.dt, fcs.reelout.reelout_l_max)
    on_timer(rc)
    if st.l_set >= fcs.reelout.reelout_l_max
        st.reelout_done = true
        isempty(st.stop_reason) && (st.stop_reason = "length")
    elseif !isnan(st.stop_start) && st.stop_reason == "laps" && t - st.stop_start >= st.stop_T
        st.reelout_done = true   # the soft-stop ramp has run out
    end
    return v_set
end

"""
    entry_force_guard!(st, setup, plant, v_set) -> v_set

Force floor BEFORE reel-out: `guard_lfc` (NOT `rc`, see `build_winch`) stepped by hand through
`calc_v_set`'s setters. Reel-IN only; returns `v_set` unchanged while the guard is inactive.
"""
function entry_force_guard!(st::RunState, setup, plant, v_set)
    (; guard_lfc, fcs, rcs) = setup
    ss = plant.ss
    set_reset(guard_lfc, false)
    set_f_set(guard_lfc, fcs.winch.entry_f_min)
    set_v_sw(guard_lfc, calc_vro(rcs, fcs.winch.entry_f_min) * 1.05)
    set_v_act(guard_lfc, plant.v_reel)
    set_tracking(guard_lfc, 0.0)   # bumpless: l_set is otherwise flat here
    set_force(guard_lfc, plant.force)
    # Reel-IN only: before reel-out this guard exists to catch a force SAG, never to reel out.
    v_guard = min(get_v_set_out(guard_lfc), 0.0)
    on_timer(guard_lfc)
    if guard_lfc.active
        v_set = v_guard
        st.l_set = st.l_set + v_set * plant.dt
    end
    return v_set
end

"""
    compliant_hold!(st, setup, plant) -> v_set

The `hold_compliance` test input of phase 5: the drum gives with the force around its low-passed
value, `kv·√F`'s slope times `gain`, and is pulled back to the held length over `τpos`.
"""
function compliant_hold!(st::RunState, setup, plant)
    (; hold_compliance, rcs) = setup
    ss = plant.ss
    f_now_h = plant.force
    if isnan(st.hold_f_lp)
        st.hold_f_lp = f_now_h
        st.hold_l0 = st.l_set
    end
    st.hold_f_lp += plant.dt / hold_compliance.τF * (f_now_h - st.hold_f_lp)
    slope = rcs.kv / (2 * sqrt(max(st.hold_f_lp, 1.0)))     # [m/s per N], of v = kv·√F
    v_set = hold_compliance.gain * slope * (f_now_h - st.hold_f_lp) -
            (st.l_set - st.hold_l0) / hold_compliance.τpos
    st.l_set = st.l_set + v_set * plant.dt
    return v_set
end

# ---- Steering hooks and logging --------------------------------------------------------------- #

"""
    steering_hooks!(st, setup, t, rel_steering, u_ff) -> rel_steering

The test inputs on the steering: the `steer_disturbance` added to it, and the V1 stability hooks
(`steer_gain_factor`, `extra_steer_delay`, see `setup_run`) from `hook_settle` after phase 4.
"""
function steering_hooks!(st::RunState, setup, t, rel_steering, u_ff)
    (; steer_disturbance, extra_steer_delay, hook_settle, steer_gain_feedback_only,
       steer_gain_factor, fcs) = setup
    if !isnothing(steer_disturbance)
        du = steer_disturbance(t)
        rel_steering += du
        push!(st.dist_t, t); push!(st.dist_d, du); push!(st.dist_u, rel_steering)
    end
    push!(st.steer_delay_buf, rel_steering)
    push!(st.ff_delay_buf, u_ff)
    delayed_u = length(st.steer_delay_buf) > extra_steer_delay ?
        popfirst!(st.steer_delay_buf) : rel_steering
    delayed_ff = length(st.ff_delay_buf) > extra_steer_delay ?
        popfirst!(st.ff_delay_buf) : u_ff
    if !isnan(st.t_phase4) && t - st.t_phase4 >= hook_settle
        u_scaled = steer_gain_feedback_only ?
            delayed_ff + steer_gain_factor * (delayed_u - delayed_ff) :
            delayed_u * steer_gain_factor
        # calc_steering already clamped its own output to ±max_steering;
        # re-clamp here too, or a gain factor > 1 commands the tape angles
        # it was never calibrated for instead of just saturating earlier,
        # as scaling heading_p itself would.
        rel_steering = clamp(u_scaled, -fcs.course.max_steering, fcs.course.max_steering)
    end
    return rel_steering
end

"""
    check_overspeed(setup, plant) -> Bool

`v_app` above `v_app_abort`, reported: the run stops rather than wait for the opaque solver abort
it causes later.
"""
function check_overspeed(setup, plant)
    (; fcs) = setup
    ss = plant.ss
    Float64(ss.v_app) > fcs.run.v_app_abort || return false
    @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                     Stopping before the solver diverges.",
                    ss.time, ss.v_app, fcs.run.v_app_abort,
                    rad2deg(ss.elevation), rad2deg(ss.AoA))
    return true
end

"""
    record_step!(st, setup, plant, t, commands)

After `step!`, which overwrites parts of `sys_state`: the controller's view of the step in the
log's slots (`commands` from [`step_commands!`](@ref)), the path
geometry logs and the running `e_mech`.
"""
function record_step!(st::RunState, setup, plant, t, commands)
    (; fcs, fec, rc) = setup
    ss = plant.ss
    (; phase, v_set, cmd) = commands
    (; chi_set, az_attr, el_attr, dmin) = commands.guide
    ss.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 transition, 4 fig8, 5 final
    ss.bearing = cmd.chi_cmd      # the course actually tracked
    ss.attractor .= (deg2rad(az_attr), deg2rad(el_attr))
    ss.var_01 = dmin              # cross-track error [deg]
    ss.var_02 = az_attr           # attractor azimuth [deg]
    ss.var_03 = el_attr           # attractor elevation [deg]
    ss.var_04 = st.el_c_path      # pattern-centre elevation [deg]
    ss.var_05 = chi_set           # RAW guidance course [rad]
    ss.var_06 = rad2deg(cmd.err)  # REGULATED error [deg]
    # A weight, not a flag: a step here means entry_d_blend is too narrow.
    ss.var_07 = abs(chi_set) > deg2rad(fcs.course.entry_chi_max) ? cmd.w_lim : 0.0
    ss.var_08 = cmd.w_course      # course/heading blend weight [-]
    # Whole wing; sys_state.AoA is the centre panel only, which a turn twists away from.
    ss.var_09 = rad2deg(plant.aoa)
    az_lo_g, az_hi_g = extrema(fec.az_path)
    el_lo_g, el_hi_g = extrema(fec.el_path)
    push!(st.geom_t, t)
    push!(st.geom_az_c, 0.5 * (az_hi_g + az_lo_g))
    push!(st.geom_az_amp, 0.5 * (az_hi_g - az_lo_g))
    push!(st.geom_el_h, el_hi_g - el_lo_g)
    push!(st.geom_d_raw, path_distance(st.raw_az, st.raw_el,
                                    rad2deg(Float64(ss.azimuth)),
                                    rad2deg(Float64(ss.elevation))))
    ss.fig_8 = Int16(st.fig8_n)   # live lap count
    ss.var_10 = st.l_set          # tether length setpoint [m]
    ss.var_11 = v_set             # REEL_OUT speed setpoint [m/s]
    ss.var_12 = get_state(rc)     # WinchController state (0/1/2)
    ss.var_13 = get_f_err(rc)     # force error [N], NaN in speed control
    # Not filled anywhere in the model chain: without this the log and the viewer read 0.
    ss.v_wind_200m .= plant.wind_factor_200 .* ss.v_wind_gnd
    # Same for e_mech, which KiteViewers prints in Wh: the running integral of the viewer's p_mech.
    st.e_mech += ss.winch_force[1] * ss.v_reelout[1] *
                     plant.dt / 3600
    ss.e_mech = st.e_mech
    return nothing
end
