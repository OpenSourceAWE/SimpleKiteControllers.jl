# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The startup of one reel-out run of examples/simple_opt_reelout.jl: the startup solve, its
# corrected retries, the installed path and the state the loop starts from. Each function takes
# the run's `setup` (see `setup_run`) and writes into its `RunState` `st`; none touches the model
# beyond reading `setup.s.dt`, so the package still does not depend on it.

"""
    startup_params(setup, el_center) -> InitParams

The startup `/init` request seeded with the guess lemniscate centred at
`el_center` [deg]; everything else comes from `tos`, the inflow and the
first-lap winch.
"""
function startup_params(setup, el_center)
    (; tos, l_set, winch_first_lap, inflow, opt_r_min, opt_box) = setup
    az, el = figure_eight_path(tos.guess_a, tos.guess_b,
                               0.0, el_center, 0.0, tos.guess_points)
    InitParams(; name = tos.name, length = opt_length(tos, l_set),
               winch_params = winch_first_lap, inflow_conditions = inflow,
               trajectory = Trajectory(collect(az), collect(el)),
               input_depower = depower_seed(tos, inflow.wind_speed),
               reg_weight = tos.reg_weight,
               detect_simple_bounds = tos.detect_simple_bounds,
               min_turn_radius = opt_r_min, pattern_limits = opt_box)
end

"""
    startup_solve(setup, params) -> (result, seed_trajectory)

`/init` with `params`, the optional seeding solve at `opt_warm_start_awe_trim`,
then the `/step` under the first-lap winch, the one lap 1 flies. Throws the `HTTP.StatusError` of a
422 unchanged; the caller decides whether that ends the run.
"""
function startup_solve(setup, params)
    (; opt_chain, tos, winch, rcs, l_set, winch_first_lap) = setup
    reply = chain_init(opt_chain, params)
    # Seeding solve: the cold first request fails at low winch force, so it is warmed at `opt_warm_start_awe_trim`.
    seed_trajectory = reply.trajectory
    if tos.opt_warm_start_awe_trim > winch.use_awe_trim
        @info @sprintf("Seeding solve at use_awe_trim %.3f before the startup \
                        request at %.3f; see opt_warm_start_awe_trim.",
                       tos.opt_warm_start_awe_trim, winch.use_awe_trim)
        warm_winch = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v,
                                   use_awe_trim = tos.opt_warm_start_awe_trim)
        seed_trajectory = chain_step(opt_chain, StepParams(opt_length(tos, l_set), warm_winch,
                                                           reply.trajectory)).trajectory
    end
    result = chain_step(opt_chain, StepParams(opt_length(tos, l_set), winch_first_lap,
                                              seed_trajectory))
    return result, seed_trajectory
end

"""
    solve_startup_path!(setup, st) -> NamedTuple

The startup solve (`solve_startup`, a 422 retried from `startup_retry_el_offsets` in order); its reply
goes to `st.opt_result`. Returns what `setup` gains from it: the seed it converged from
(`el_center_seed`, `startup_seed_offset`, `guess_az`, `guess_el`) and its wall time,
`opt_startup_solve_s`. The startup solve holds the script; blocking re-optimizations hold the loop
(`reopt_blocked_s`).
"""
function solve_startup_path!(setup, st::RunState)
    (; tos, el_center_seed_base, l_set, winch, inflow) = setup
    t_solve_start = time()
    (; opt_result, el_center_seed, startup_seed_offset, guess_az, guess_el) =
        solve_startup(tos, el -> startup_params(setup, el), params -> startup_solve(setup, params),
                      startup_params(setup, setup.el_center_seed), el_center_seed_base, l_set, winch,
                      inflow)
    st.opt_result = opt_result
    startup_seed_offset == 0 ||
        @warn @sprintf("Startup path solved from a RETRY seed centred at %.0f° \
                        (%+.1f° off guess_el_center): a different optimum than the \
                        shipped guess would have given.", el_center_seed, startup_seed_offset)
    return (; el_center_seed, startup_seed_offset, guess_az, guess_el,
            opt_startup_solve_s = time() - t_solve_start)
end

"""
    log_startup_reply(fcs, opt_result, opt_r_min)

Say what the startup reply was optimized at: its depower converted to the V3Kite `rel_depower` flying at
the SAME power, against the flown setpoint, and the optimizer's own curvature diagnostic, physical and
comparable with `min_feasibility_margin`.
"""
function log_startup_reply(fcs, opt_result, opt_r_min)
    if !isnothing(opt_result.depower)
        u_p_equiv = awetrim_depower_to_v3kite(opt_result.depower.value)
        @info @sprintf("Optimized at depower l_dp = %.3f m (mode %s) = rel_depower \
                        %.3f equivalent, against the flown depower_setpoint = %.3f — \
                        a %+.3f gap.",
                       opt_result.depower.value, opt_result.depower.mode, u_p_equiv,
                       fcs.course.depower_setpoint, u_p_equiv - fcs.course.depower_setpoint)
    end
    # The optimizer's own curvature diagnostic, physical and comparable with `min_feasibility_margin`.
    isnothing(opt_result.metrics.turn_radius_min_m) ||
        @info @sprintf("Tightest physical turn radius of the reply: %.2f m%s.",
                       opt_result.metrics.turn_radius_min_m,
                       isnothing(opt_r_min) ? "" :
                           @sprintf(" (asked for >= %.2f m)", opt_r_min))
end

"""
    log_lobe_lift(fcs, opt_result)

Say how the lobe lift reads on the startup pattern: in degrees of azimuth, or, for `azimuth_frac`, as
fractions of each path's own amplitude, so the degrees are the STARTUP pattern's.
"""
function log_lobe_lift(fcs, opt_result)
    if fcs.reelout.el_offset_wing != 0 && fcs.reelout.el_offset_wing_mode == "azimuth"
        @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.1f°, ramped over %.1f°, \
                        zero inside %.1f°.",
                       fcs.reelout.el_offset_wing, fcs.reelout.el_offset_wing_az, fcs.reelout.el_offset_wing_blend,
                       fcs.reelout.el_offset_wing_az - fcs.reelout.el_offset_wing_blend)
    elseif fcs.reelout.el_offset_wing != 0 && fcs.reelout.el_offset_wing_mode == "azimuth_frac"
        # Fractions of each path's own amplitude, so the degrees below are the STARTUP pattern's.
        amp0 = 0.5 * (maximum(opt_result.trajectory.azimuth) -
                      minimum(opt_result.trajectory.azimuth))
        @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.2f of the pattern's own \
                        amplitude, ramped over %.2f of it — %.1f° and %.1f° on the \
                        startup path's ±%.1f°.",
                       fcs.reelout.el_offset_wing, fcs.reelout.el_offset_wing_az, fcs.reelout.el_offset_wing_blend,
                       fcs.reelout.el_offset_wing_az * amp0, fcs.reelout.el_offset_wing_blend * amp0, amp0)
    end
end

# Resample but never upsample; the lobe lift is rationed to fit the curvature gate.
function install_optimized_path!(setup, st::RunState, reply)
    (; tos, fcs, fec, l_tether, wing_lift, c1_at_depower, pattern_depower) = setup
    az = collect(Float64.(reply.trajectory.azimuth))
    el = collect(Float64.(reply.trajectory.elevation))
    lift = wing_lift(az, el)
    resample = min(tos.resample_points, length(az) - 1)
    st.c1_startup = c1_at_depower(pattern_depower(reply))
    st.startup_wing_frac = 1.0
    if !isnan(st.c1_startup) && tos.min_feasibility_margin > 0 && any(!=(0), lift)
        for fw in (1.0, 0.75, 0.5, 0.25, 0.0)
            st.startup_wing_frac = fw
            set_path!(fec, az, el .+ fw .* lift; resample)
            check_pattern_feasible(fec, l_tether, fcs.course.max_steering;
                                   c1 = st.c1_startup, prn = false).margin >=
                tos.min_feasibility_margin && break
        end
        st.startup_wing_frac < 1 &&
            @info @sprintf("Lobe lift held back on the startup path to fit the \
                            curvature gate: %.0f %% of %.2f°.",
                           100 * st.startup_wing_frac, fcs.reelout.el_offset_wing)
    else
        set_path!(fec, az, el .+ lift; resample)
    end
    return (az, el)
end
"""
    adopt_startup_path!(setup, st)

Take the startup reply as the path of the run: install it (every optimizer answer is kept as it arrived,
before any lift, with where and when it was installed), record its success for a rerun, apply the winch
gain it chose, and re-measure the anchor ratio and the turn-radius request off it.
"""
function adopt_startup_path!(setup, st::RunState)
    (; opt_chain, l_tether, opt_r_on, tos, fcs) = setup
    st.opt_paths_raw = [install_optimized_path!(setup, st, st.opt_result)]
    # Where each of those was installed: (sim time [s], phase); the startup path goes in before the run.
    st.opt_paths_at = [(0.0, 0)]

    # set_path! REVERSES a path that does not match up_loops, so a mismatch must be caught here.
    st.opt_table = chain_trajectory(opt_chain)
    # Installed above, so applied: stored for a rerun that sends the same requests.
    record_opt_success!(opt_chain)
    apply_optimized_kv!(setup, st.opt_table, 0.0, l_tether)
    st.opt_downloops = st.opt_table["spline"]["downloops"]
    st.opt_power_pred = Float64(st.opt_table["metrics"]["avg_power_W"])
    # The anchor ratio, now measured off the reply; guarded so a request that is off stays off.
    if opt_r_on
        st.opt_r_scale = reelout_anchor_ratio(st.opt_table) * tos.turn_radius_headroom
        st.opt_r_min = min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                            c1 = st.c1_startup)
    end
    isnothing(st.opt_r_min) ||
        @info @sprintf("Turn-radius request for the re-optimizations: %.2f m — the \
                        gate's %.2f m at margin %.2f, x %.3f for the lap's reel-out \
                        (%.1f -> %.1f m) and x %.2f of headroom.",
                       st.opt_r_min, st.opt_r_min / st.opt_r_scale, tos.min_feasibility_margin,
                       reelout_anchor_ratio(st.opt_table),
                       minimum(Float64.(st.opt_table["table"]["distance_radial"])),
                       maximum(Float64.(st.opt_table["table"]["distance_radial"])),
                       tos.turn_radius_headroom)
end

# ---- Corrected retries of the STARTUP solve: one lever per attempt (ceiling, width, radius) ---- #
# The decisions (which lever, what the answers imply) are `next_lever` & co. of src/startup_retry.jl.

# All three startup gates on the path installed in `fec`, shared by the retries and the incumbent's record after them.
function score_installed(setup, st::RunState)
    (; fec, l_tether, fcs, tos, el_floor) = setup
    margin = check_pattern_feasible(fec, l_tether, fcs.course.max_steering;
                                    c1 = st.c1_startup, prn = false).margin
    el_ok = minimum(fec.el_path) >= el_floor
    height = NaN
    clr_ok = true
    if tos.min_height > 0
        clr = check_pattern_height(fec, l_tether, tos.min_height; prn = false)
        height = clr.height
        clr_ok = clr.ok
    end
    (; margin, el_ok, clr_ok, height,
       ok = margin >= tos.min_feasibility_margin && el_ok && clr_ok)
end
"""
Where [`save_failed_trajectory`](@ref) writes: the package's `trajectories/`. Set it only around a
block that must not write there, e.g. a unit test, and reset it in a `finally`.
"""
const FAILED_TRAJECTORY_DIR = Ref(joinpath(@__DIR__, "..", "trajectories"))

# Saves a rejected curve for examples/plot_trajectory.jl.
function save_failed_trajectory(setup, name, az, el; margin = NaN, power = NaN)
    dir = FAILED_TRAJECTORY_DIR[]
    mkpath(dir)
    stamp = replace(string(now()), r"[:.]" => "", "T" => "_")[1:15]
    file = joinpath(dir, "$(name)_$stamp.yaml")
    YAML.write_file(file, Dict(
        "name" => name,
        "date" => string(now()),
        "l_tether" => setup.l_tether,
        "min_feasibility_margin" => setup.tos.min_feasibility_margin,
        "margin" => margin,
        "predicted_power_W" => power,
        "azimuth_deg" => collect(Float64.(az)),
        "elevation_deg" => collect(Float64.(el)),
    ))
    @info "Saved failed trajectory to $file (margin $margin)."
end

"""
    retry_startup!(setup, st)

Corrected retries of the STARTUP solve, for a startup path whose turn margin is below
`min_feasibility_margin`: one lever per attempt (`next_lever`), the best path so far installed
in `fec` and in the `opt_*` fields of `st` the rest of the run reads, `incumbent_score` and `inc_*` (the
incumbent, which `startup_incumbent` records afterwards) included.
"""
function retry_startup!(setup, st::RunState)
    (; tos, fcs, opt_chain, opt_r_sent, opt_box, el_floor, winch_first_lap) = setup
    st.incumbent_score = score_installed(setup, st)
    st.inc_result, st.inc_table, st.inc_raw = st.opt_result, st.opt_table, st.opt_paths_raw[1]
    ladder = RetryLadder(; m_reply = st.incumbent_score.margin)  # what the answers so far imply
    t_retries = time()
    for attempt in 1:max(Int(tos.startup_retries_max), 0)
        ask = next_lever(ladder, tos, st.inc_raw[1], st.inc_raw[2], opt_r_sent,
                         isnothing(opt_box) ? nothing : opt_box.elevation_min, el_floor,
                         margin -> min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                                           margin, c1 = st.c1_startup))
        if isnothing(ask.lever)
            @info @sprintf("Startup retries stop at %d/%d: no radius under the \
                            %.2f m that 422'd can reach past margin %.3f (the \
                            gate wants %.2f), and the ceiling and width levers \
                            are spent.",
                           attempt, Int(tos.startup_retries_max), ladder.bisect_hi,
                           ladder.m_reply * ladder.bisect_hi / ask.prev_ask,
                           tos.min_feasibility_margin)
            break
        end
        (; lever, r_ask, el_cap, az_min, target, prev_ask, inc_top, inc_height, inc_amp,
           el_min_box, bisect_room) = ask
        # `nothing` keeps the session's limits; only a changed side builds a box.
        box_ask = nothing
        isnothing(el_cap) || (box_ask = with_elevation_max(opt_box, el_cap))
        isnothing(az_min) ||
            (box_ask = with_azimuth_amplitude_min(isnothing(box_ask) ? opt_box : box_ask,
                                                  az_min))
        @info @sprintf("The startup path is at margin %.3f, below \
                        min_feasibility_margin = %.2f: retry %d/%d (%s) at \
                        L = %.1f m, turn radius %.2f m (was %.2f m)%s, targeting \
                        margin %.3f%s.",
                       st.incumbent_score.margin, tos.min_feasibility_margin,
                       attempt, Int(tos.startup_retries_max), lever, st.l_set,
                       r_ask, prev_ask,
                       (isnothing(el_cap) ? ", no elevation ceiling" :
                           @sprintf(", elevation ceiling %.1f° (the incumbent \
                                    spans %.1f-%.1f° over a %.1f° floor)",
                                    el_cap, inc_top - inc_height, inc_top,
                                    el_min_box)) *
                       (isnothing(az_min) ? "" :
                           @sprintf(", azimuth half-width >= %.1f° (the \
                                    incumbent's is %.1f°)", az_min, inc_amp)),
                       target,
                       isnan(ladder.bisect_hi) ? "" :
                           @sprintf(" (%s the %.2f m that 422'd)",
                                    bisect_room ? "bisecting below" :
                                        "the radius lever is spent under",
                                    ladder.bisect_hi))
        t_attempt = time()
        local att_result, att_table, att_raw, att_score
        try
            att_result = chain_step(opt_chain,
                                    StepParams(; length = opt_length(tos, st.l_set),
                                               winch_params = winch_first_lap,
                                               min_turn_radius = r_ask,
                                               pattern_limits = box_ask))
            att_table = chain_trajectory(opt_chain)
            att_raw = install_optimized_path!(setup, st, att_result)
            att_score = score_installed(setup, st)
        catch exc
            exc isa HTTP.StatusError && exc.status == 422 || rethrow()
            kind = record_422!(ladder, ask)
            if kind == :ceiling
                # The ceiling moved and failed: never send it (or lower) again; the radius steps next.
                @info @sprintf("Startup retry %d (%s) could not converge (HTTP \
                                422) at %.2f m under a ceiling of %s; the \
                                ceiling lever is spent, %s under the last \
                                converged ceiling (%s).",
                               attempt, lever, r_ask,
                               isnothing(el_cap) ? "none" : @sprintf("%.1f°", el_cap),
                               isnan(ladder.r_asked) ? "re-asking the same radius" :
                                                       "the radius steps next",
                               isnothing(ladder.cap_ok) ? "none" : @sprintf("%.1f°", ladder.cap_ok))
            elseif kind == :width
                # Only the width floor moved and failed: never ask for it (or wider) again.
                @info @sprintf("Startup retry %d (%s) could not converge (HTTP \
                                422) at %.2f m with an azimuth half-width >= \
                                %.1f°; the width lever is spent, the radius \
                                steps next at the last converged floor (%s).",
                               attempt, lever, r_ask, az_min,
                               isnothing(ladder.width_ok) ? "none" :
                                   @sprintf("%.1f°", ladder.width_ok))
            else
                @info @sprintf("Startup retry %d could not converge (HTTP 422) at \
                                %.2f m; bisecting toward the last converged ask \
                                of %.2f m.", attempt, r_ask, prev_ask)
            end
            continue
        end
        @info @sprintf("Startup retry %d measured: margin %.3f%s, lowest point \
                        %.1f m (floor %.0f m), elevation %s, predicted power \
                        %.0f W, %.1f s.",
                       attempt, att_score.margin,
                       att_score.ok ? " clearing all gates" : "",
                       att_score.height, tos.min_height,
                       att_score.el_ok ? "ok" : "BELOW FLOOR",
                       Float64(att_table["metrics"]["avg_power_W"]),
                       time() - t_attempt)
        takes_over = att_score.ok ||
                     (att_score.el_ok && att_score.clr_ok &&
                      att_score.margin > st.incumbent_score.margin)
        if !takes_over && att_score.margin > st.incumbent_score.margin &&
           (!att_score.el_ok || !att_score.clr_ok)
            install_optimized_path!(setup, st, st.inc_result)     # incumbent stays flown
            @warn @sprintf("Startup retry %d reached margin %.3f but dropped \
                            below a floor (clearance %s, elevation %s); wider \
                            cannot recover clearance — retries stop here.",
                           attempt, att_score.margin,
                           att_score.clr_ok ? "ok" : "MISSED",
                           att_score.el_ok ? "ok" : "MISSED")
            break
        elseif takes_over
            record_opt_success!(opt_chain)
            st.incumbent_score = att_score
            st.inc_result, st.inc_table, st.inc_raw = att_result, att_table, att_raw
            apply_optimized_kv!(setup, st.inc_table, 0.0, st.l_set)
            # Only adoption moves these: a discarded retry leaves the `opt_*` state untouched.
            st.opt_result = st.inc_result
            st.opt_table = st.inc_table
            st.opt_downloops = st.inc_table["spline"]["downloops"]
            st.opt_power_pred = Float64(st.inc_table["metrics"]["avg_power_W"])
            st.opt_paths_raw = [st.inc_raw]
            st.opt_paths_at = [(0.0, 0)]
            st.opt_r_scale = reelout_anchor_ratio(st.inc_table) *
                          tos.turn_radius_headroom
            st.opt_r_min = min_turn_radius_request(fcs, tos; scale = st.opt_r_scale,
                                                c1 = st.c1_startup)
            if att_score.ok
                @info @sprintf("Startup path clears the gates at margin %.3f \
                                after %d solves (%.1f s of wall time).",
                               st.incumbent_score.margin, attempt, time() - t_retries)
                break
            end
            @info "Kept retry $attempt as the best-so-far; trying again."
        else
            install_optimized_path!(setup, st, st.inc_result)     # put the incumbent back
            save_failed_trajectory(setup, "startup_retry$attempt", att_raw[1],
                                   att_raw[2]; margin = att_score.margin,
                                   power = Float64(att_table["metrics"]["avg_power_W"]))
            @info @sprintf("Startup retry %d gave margin %.3f, no better than \
                            %.3f — keeping the incumbent.",
                           attempt, att_score.margin, st.incumbent_score.margin)
        end
        record_converged!(ladder, ask, att_score.margin)
    end
end

"""
    finish_startup!(setup, st)

The corrected retries (`retry_startup!`) for a startup path whose turn margin is below
`min_feasibility_margin`, the record of the incumbent the gates will then refuse, and the depower the
optimizer asked for, logged for the summary.
"""
function finish_startup!(setup, st::RunState)
    (; fec, l_tether, fcs, tos, opt_r_on, opt_depower_log) = setup
    margin_startup = check_pattern_feasible(fec, l_tether, fcs.course.max_steering;
                                            c1 = st.c1_startup, prn = false).margin
    if opt_r_on && !isnan(st.c1_startup) && margin_startup < tos.min_feasibility_margin
        retry_startup!(setup, st)
    end

    if margin_startup < tos.min_feasibility_margin
        # The incumbent is what the gates will refuse; `incumbent_score` exists exactly when this fires.
        save_failed_trajectory(setup, "startup_incumbent", st.inc_raw[1], st.inc_raw[2];
                               margin = st.incumbent_score.margin,
                               power = st.opt_power_pred)
    end

    if !isnothing(st.opt_result.depower)
        st.depower_flown_opt = awetrim_depower_to_v3kite(st.opt_result.depower.value)
        push!(opt_depower_log,
              (; t = 0.0, l_dp = st.opt_result.depower.value, u_p_equiv = st.depower_flown_opt))
    end
end

"""
    capture_startup_geometry!(setup, st)

The startup pattern's own geometry, captured now (with `reopt_enabled`, `fec` holds another path at the
end), and the prediction timeline that says which path was flown when, so the run is scored against the
path in the air. Refuses a path that flies against `up_loops`.
"""
function capture_startup_geometry!(setup, st::RunState)
    (; fec, l_tether, fcs) = setup
    st.n_path_initial = length(fec.az_path)
    st.path_min_h_start = path_min_height(fec, l_tether)
    st.az_c_path = 0.5 * (maximum(fec.az_path) + minimum(fec.az_path))
    st.el_c_path = 0.5 * (maximum(fec.el_path) + minimum(fec.el_path))
    st.az_amp_path = 0.5 * (maximum(fec.az_path) - minimum(fec.az_path))
    st.el_height_path = maximum(fec.el_path) - minimum(fec.el_path)

    # Which path was flown when, so the run is scored against the prediction of the path in the air.
    st.pred_timeline = [(t = 0.0, power = st.opt_power_pred)]
    st.opt_downloops == !fcs.pattern.up_loops ||
        error("The optimizer returned a downloops = $(st.opt_downloops) path while this run \
               flies up_loops = $(fcs.pattern.up_loops). Change fcs.pattern.up_loops or the guess; do \
               not fly it reversed.")

    @info @sprintf("Optimized path: %d points, azimuth %.1f°…%.1f°, elevation \
                    %.1f°…%.1f° (centre %.1f°), predicted mean reel-out power %.0f W.",
                   length(fec.az_path), minimum(fec.az_path), maximum(fec.az_path),
                   minimum(fec.el_path), maximum(fec.el_path), st.el_c_path, st.opt_power_pred)
end

"""
    startup_feasibility(setup, st) -> (; feas, margin5, c1_at_phase, phase5_margin_at)

The gates that refuse the run (`check_startup_path`) on the installed startup path, at
the depower the pattern is FLOWN at (`pattern_depower`): with `fly_opt_depower` the kite
flies the optimizer's `u_d` from phase 3 on. Returns the verdicts `feas`, `margin5`, the
`Phase5MarginState` of the in-air phase-5 check, and the two laws the loop reads off
`feas`: `c1_at_phase(phase, depower | st)`, the c1 to check a path against at time t,
and `phase5_margin_at(az, el)`, what phase 5 will fly a candidate path with.
"""
function startup_feasibility(setup, st::RunState)
    (; fec, fcs, tos, l_tether, c1_at_depower, pattern_depower) = setup
    feas = check_startup_path(fec, fcs, tos; l_tether, depower = pattern_depower(st.opt_result))
    # A cell the table cannot serve falls back to the startup law, see `c1_at`; `st` reads it
    # at the depower currently flown (`st.depower_flown_opt`), when sizing a request.
    c1_at_phase(phase::Integer, depower::Real) =
        c1_at(feas, phase, phase >= 5 ? NaN : c1_at_depower(depower))
    c1_at_phase(phase::Integer, st::RunState) =
        c1_at_phase(phase, tos.fly_opt_depower ? st.depower_flown_opt : fcs.course.depower_setpoint)
    # NaN when the table could not serve depower_final. See phase5_margin's docstring for
    # why this is NOT comparable to the install's own margin early in the reel-out.
    phase5_margin_at(az, el) = phase5_margin(feas, az, el, fcs.reelout.reelout_l_max, fcs.course.max_steering)
    return (; feas, margin5 = Phase5MarginState(), c1_at_phase, phase5_margin_at)
end

"""
    init_phase5_and_controller!(setup, st)

The record of every path flown, for the phase-5 fallback (`final_margin_min`), and the course controller,
whose dive aims at the pattern centre, which is the OPTIMIZED path's now.
"""
function init_phase5_and_controller!(setup, st::RunState)
    (; fec, fcs, s, phase5_margin_at) = setup
    st.p5_history = [(t = 0.0, az = copy(fec.az_path), el = copy(fec.el_path), raw = st.opt_paths_raw[end],
                   margin = phase5_margin_at(fec.az_path, fec.el_path), el_applied = 0.0)]
    st.p5_fallback_done = false    # checked once, from the stop latch on, at the next crossing
    st.p5_q_az_prev = NaN          # [deg] Q's azimuth from the path centre, last step; arms the crossing gate
    st.p5_fallback = nothing       # (; t, from_margin, to_margin, to_t) when a fallback was blended in

    @info @sprintf("Elevation lift: el_offset_final = %+.2f°, el_offset_lead = %.1f s \
                    (%s), reelout_softstop = %.1f s.",
                   fcs.reelout.el_offset_final, fcs.reelout.el_offset_lead,
                   fcs.reelout.el_offset_lead > 0 ? "anticipates the end of reel-out" :
                                            "starts at the stop latch / phase 5",
                   fcs.reelout.reelout_softstop)

    # The dive aims at the pattern centre, which is the OPTIMIZED path's now.
    st.ccs = CourseControllerSettings(fcs; dt = s.dt)
    st.ccs.el_center = st.el_c_path
    st.cc = CourseController(st.ccs)
end

"""
    init_loop_state!(setup, st)

The loop's state that cannot be a `RunState` default because it depends on the run: the lap counter's
starting index, the scored reference, the path resolution, the last commanded depower and the depower
ramp's start. The rest starts at the defaults of `RunState`.
"""
function init_loop_state!(setup, st::RunState)
    (; fcs, fec, tos) = setup
    st.rel_depower_prev = fcs.course.depower_setpoint  # the gain reads c1 there
    # fig_8 live lap count: 0 before phase 4, 1 at first entry, +1 per traversal; the post-run `fig8` is another thing.
    st.fig8_idx_prev = fec.last_idx
    st.n_path = length(fec.az_path)
    # The reference TRACKING is scored against: the optimizer's curve, canonicalized and blended like the flown one, never lifted.
    st.raw_az, st.raw_el = prepare_path(st.opt_paths_raw[1]...;
                                        resample = min(tos.resample_points, length(st.opt_paths_raw[1][1]) - 1),
                                        up_loops = fcs.pattern.up_loops)
    length(st.raw_az) == st.n_path ||
        error("scored reference has $(length(st.raw_az)) points, the flown path $(st.n_path)")
    # Resolution the path in the air is worth checking at (the reply's own, not `n_path`); updated per install.
    st.chk_points = st.n_path
    # Same mechanism as the path blend, scalar, for the optimizer's rel_depower override.
    st.depower_flown = st.depower_flown_opt    # current blended output
    st.depower_blend_from = st.depower_flown
end
