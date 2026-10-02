# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    reelout_results.jl — scoring, summary, archive and plots of an optimized reel-out run.

`include`d by `simple_opt_reelout.jl` before its last step, which calls
[`reelout_results`](@ref) once the log has been saved. It reads the run's
`setup` (`fcs`, `tos`, ...) and its `RunState` `st` (`st.reopt_events`,
`st.el_applied`, `st.opt_power_pred`, ...), reloads the log, prints the results
block, writes the summary YAML next to it, copies both plus every input YAML
into a timestamped archive folder, draws the plots and writes the finished-run
marker. Nothing here touches the plant, so a change to it is checked by
re-scoring an existing log rather than by flying again.

Every step is a function of its own; `reelout_results` runs them in order and
leaves no globals behind but `REF_PATH` and `LOG_NAME`, the plots' hand-over.
"""

# ==================== RESULTS ==================== #

# The log helpers (`lap_durations`, `on_log`, `weighted_prediction`), the summary writer
# (`write_yaml_commented`, `time_keyed`) and `free_speed_reference` are the package's.

"""
    score_log(setup, st) -> NamedTuple

Reload the saved log and score it with `print_fig8_metrics` against the pattern
actually commanded, lap by lap (`st.geom_*`); `st.fig8m` keeps the verdict for the
finished-run marker. Also prints the lift budget: what every lift costs on the
pattern-size criteria. Returns the log `sl`, `fig8m`, the laps flown and the
numbers the summary's `lift_budget` reports.
"""
function score_log(setup, st::RunState)
    (; fcs, project_set, output_path, log_name) = setup
    sl = load_log(log_name; path = output_path).syslog
    laps_flown = lap_durations(sl)
    # The geometry is passed in too: without it the criteria are blind to pattern SIZE.
    # require_final: this script's own phase 5, unlike simple_fig8.jl's sys_state
    # (which never goes past 4) — checks reel-out actually finished within the run.
    # From the flown path, not from fcs.f8_*. `az_center` is in RADIANS here (it is
    # compared against the logged azimuth), the two extents in degrees.

    # The size criteria against the pattern actually commanded, lap by lap. The
    # startup path is only the FIRST of them, and it is the widest the run ever flies:
    # scored against it, the last laps are asked for a reach nothing ever commanded.
    t_log = Float64.(sl.time)
    have_geom = !isempty(st.geom_t)
    az_c_log = have_geom ? deg2rad.(on_log(t_log, st.geom_t, st.geom_az_c)) : deg2rad(st.az_c_path)
    az_amp_log = have_geom ? on_log(t_log, st.geom_t, st.geom_az_amp) : st.az_amp_path
    el_h_log = have_geom ? on_log(t_log, st.geom_t, st.geom_el_h) : st.el_height_path
    # The cross-track error scored is the one to the optimizer's UNLIFTED curve
    # (`raw_az`/`raw_el` in the run script), not `var_01`, the guidance's own error
    # to the corrected path it steers for — see the comment at `raw_az`.
    d_raw_log = have_geom ? on_log(t_log, st.geom_t, st.geom_d_raw) : nothing
    fig8m = print_fig8_metrics(sl; t_start = fcs.course.park_time, settle_time = fcs.run.entry_time,
                       min_elevation = fcs.run.min_elevation, az_center = az_c_log,
                       az_amplitude = az_amp_log, el_height = el_h_log,
                       min_span_frac = fcs.run.min_span_frac, require_final = true,
                       max_force = project_set.max_force, cross_track = d_raw_log)
    st.fig8m = fig8m
    # A run that stopped before the metrics window scores nothing, and every line
    # below dereferences `fig8m`. Say so, instead of a `FieldError` on `Nothing`.
    isnothing(fig8m) && error("No settled samples: the run ended at t = ", round(t_log[end], digits = 1),
                              " s, before the metrics window opens at park_time + entry_time = ",
                              fcs.course.park_time + fcs.run.entry_time, " s. Nothing to score; the log is ",
                              joinpath(output_path, log_name * ".arrow"), ".")

    # What every lift costs on the OTHER axis. A pattern raised is a pattern flown
    # narrower, and the three size criteria are the first thing a lift breaks — the
    # 2.5 s el_offset_lead failed the azimuth reach by hundredths of a degree while
    # passing everything else. Reported next to the elevation that lift bought, so a
    # run says both halves of the trade rather than one.
    az_amp_mean = have_geom ? mean(az_amp_log) : st.az_amp_path
    el_h_mean = have_geom ? mean(el_h_log) : st.el_height_path
    # In FILL fractions, since that is what the criteria are scored on once the
    # commanded pattern moves; the degrees are the same margin read against the mean
    # geometry, for a number that can be compared with a lift.
    span_checks = [("azimuth_reach_pos", fig8m.az_fill_pos, az_amp_mean),
                   ("azimuth_reach_neg", fig8m.az_fill_neg, az_amp_mean),
                   ("elevation_span", fig8m.el_fill, el_h_mean)]
    span_margins = [(; name, fill, flown = fill * ref,
                     required = fcs.run.min_span_frac * ref,
                     margin = (fill - fcs.run.min_span_frac) * ref,
                     pct = 100 * (fill / fcs.run.min_span_frac - 1))
                    for (name, fill, ref) in span_checks]
    span_worst = argmin(m -> m.margin, span_margins)
    i_final = findall(==(5), Int.(sl.sys_state))
    el_min_final = isempty(i_final) ? NaN : rad2deg(minimum(Float64.(sl.elevation[i_final])))
    # What the path in the air actually carries, not what was asked for: the lift only
    # reaches the kite through an install or an in-air blend that the curvature gate
    # can refuse.
    lift_mean = st.el_applied
    @info @sprintf("Lift budget: %+.2f° of lift delivered (lobes up to %+.2f°); the \
                    tightest size criterion is %s with %+.2f° (%+.0f %%) to spare; min \
                    elevation %.1f° over the run, %.1f° in phase 5.",
                   lift_mean, fcs.reelout.el_offset_wing,
                   replace(span_worst.name, "_" => " "), span_worst.margin,
                   span_worst.pct, fig8m.min_elevation_all, el_min_final)
    return (; sl, fig8m, laps_flown, az_amp_mean, el_h_mean, span_margins, span_worst,
            el_min_final, lift_mean)
end

# The verdict, the `simulation`, `fig8_metrics` and `reelout` sections and the performance
# figures are the package's (`success_verdict`, `simulation_block`, `fig8_metrics_block`,
# `reelout_block`, `performance_block`, `opt_cycle_max`), shared with simple_reelout.jl.

# Only meaningful where k_v's soft floor bites (low force) AND the run either fell
# short of its own prediction or beat it by more than FREE_SPEED_RATIO_MIN_HIGH;
# a ratio in between needs no upper bound.
const FREE_SPEED_RATIO_MAX = 0.9
const FREE_SPEED_RATIO_MIN_HIGH = 1.1

"""
    power_comparison(setup, st, sl, rp, p4) -> NamedTuple

The comparison this script exists for: what the optimizer promised against what
reeling out delivered, as the `traj_opt.power` block. `rp` is `nothing` when the
tether never reeled out, and then there is nothing to compare.

The prediction is a WEIGHTED one whenever a re-optimization installed a path. Each
path carries its own `avg_power_W`, and the run flies each for part of the reeling
window, so scoring the whole window against the FIRST path's number compares the
measurement to a path that was not in the air for some of it. `rp.idx` is that
window's samples, so every share (`weighted_prediction`) is measured over exactly
the samples `measured_W` averages. Adds the free-speed upper bound (`free_speed_reference`)
where the ratio is notable and the force low. The measured power goes to
`st.opt_power_meas` for the finished-run marker.
"""
function power_comparison(setup, st::RunState, sl, rp, p4)
    (; tos) = setup
    opt_power_meas = isnothing(rp) ? nothing : rp.mean_power
    st.opt_power_meas = opt_power_meas
    pred_shares = NamedTuple[]
    opt_power_pred_eff = st.opt_power_pred
    if !isnothing(rp)
        (; shares, power) = weighted_prediction(st.pred_timeline, Float64.(sl.time[rp.idx]))
        pred_shares, opt_power_pred_eff = shares, power
    end
    power_summary = nothing
    if !isnothing(opt_power_meas)
        # Built once and repeated as an `@info` after the archive line: this is the
        # comparison the script exists for, and here it is buried in the results block.
        power_summary = @sprintf("Optimizer predicted %.0f W of mean reel-out \
                                  power%s; measured %.0f W (%.2f x).",
            opt_power_pred_eff,
            length(pred_shares) > 1 ?
                @sprintf(" (weighted over %d paths: %s)", length(pred_shares),
                         join((@sprintf("%.0f W for %.0f%%", p.power, 100 * p.share)
                               for p in pred_shares), ", ")) : " for this path",
            opt_power_meas, opt_power_meas / opt_power_pred_eff)
        println("  ", power_summary)
    end
    # A key is omitted rather than written empty when its measurement does not exist:
    # `string(nothing)` would put the bare word `nothing` into the file, which YAML
    # reads back as a string.
    power_block = OrderedDict{String, Any}(
        "predicted_W" => (round(Int, opt_power_pred_eff),
            "predicted mean reel-out power of the paths actually flown, weighted by \
             their share of the reeling window [W]"),
        "predicted_initial_W" => (round(Int, st.opt_power_pred),
            "predicted mean reel-out power of the path installed before the run [W]"))
    if length(pred_shares) > 1
        power_block["predicted_paths"] = OrderedDict(
            @sprintf("t_%05.1f_s", p.from_s) =>
                (round(Int, p.power), @sprintf("%.0f%% of the reeling window", 100 * p.share))
            for p in pred_shares)
    end
    if !isnothing(opt_power_meas)
        power_block["measured_W"] = (round(Int, opt_power_meas),
            "mean reel-out power the run harvested [W]")
        power_block["ratio"] = (round(opt_power_meas / opt_power_pred_eff; digits = 2),
            "measured / predicted, against the weighted prediction")
    end

    fs_ref = nothing
    power_ratio_notable = !isnothing(opt_power_meas) &&
                          (opt_power_meas / opt_power_pred_eff <= FREE_SPEED_RATIO_MAX ||
                           opt_power_meas / opt_power_pred_eff >= FREE_SPEED_RATIO_MIN_HIGH)
    if !isnothing(rp) && tos.free_speed_reference_points >= 2 && !isnothing(p4) &&
       p4.force.min < 1000 && power_ratio_notable
        lengths_ro = Float64.(sl.var_10[rp.idx])
        fs_ref = free_speed_reference(tos, setup.rcs, setup.inflow, setup.guess_az, setup.guess_el,
                                      lengths_ro; min_turn_radius = st.opt_r_min,
                                      pattern_limits = setup.opt_box)
        if isnothing(fs_ref)
            @warn "free_speed reference: no solve succeeded, omitting it from the summary."
        else
            @printf("  free_speed reference: %.0f W over %.0f-%.0f m (%d of %d solved)%s\n",
                    fs_ref.weighted, minimum(lengths_ro), maximum(lengths_ro),
                    length(fs_ref.points), tos.free_speed_reference_points,
                    isnothing(opt_power_meas) ? "" :
                        @sprintf("; measured %.0f W is %.2f x it",
                                 opt_power_meas, opt_power_meas / fs_ref.weighted))
            power_block["free_speed_reference_W"] = (round(Int, fs_ref.weighted),
                "mean reel-out power an OPTIMAL path could harvest with ANY winch in \
                 [f_min, f_max], solved after the run at $(length(fs_ref.points)) \
                 lengths and weighted by time spent at each. An UPPER BOUND, not a \
                 prediction of this k_v law, and never flown [W]")
            power_block["free_speed_points"] = OrderedDict(
                @sprintf("L_%05.1f_m", p.l) => (round(Int, p.power), "free_speed solve [W]")
                for p in fs_ref.points)
            isnothing(opt_power_meas) || (power_block["free_speed_ratio"] =
                (round(opt_power_meas / fs_ref.weighted; digits = 2),
                 "measured / free_speed_reference_W; above 1 means the run harvests more \
                  than the optimizer's own upper bound, which no winch law can explain"))
        end
    end
    return (; block = power_block, opt_power_meas, opt_power_pred_eff, fs_ref, power_summary)
end

"The summary's `traj_opt.feasibility` section: the curvature gates and what phase 5 inherited"
function feasibility_block(setup, st::RunState)
    (; tos, feas, margin5, c1_setpoint) = setup
    block = OrderedDict{String, Any}(
        "min_required" => (tos.min_feasibility_margin,
            "min_feasibility_margin of data/traj_opt.yaml [-]"),
        "turn_radius_headroom" => (tos.turn_radius_headroom,
            "factor on the gate's number for what the GATE adds: its \
             finite-difference curvature estimate and the elevation lift [-]"),
        "turn_radius_request_m" => (isnothing(st.opt_r_min) ? "unset" :
                                    round(st.opt_r_min; digits = 2),
            "minimum turn radius asked of the optimizer for the LAST request; the \
             gate's margin/(c1*max_steering) times the headroom and the lap's \
             reel-out ratio, since the optimizer measures the radius up the reel-out \
             while the run flies the curve at the anchor [m]"),
        "turn_radius_scale" => (round(st.opt_r_scale; digits = 3),
            "the two corrections together, as sent [-]"),
        "turn_radius_lap_reelout_m" => (tos.turn_radius_lap_reelout_m,
            "reel-out per lap ASSUMED for the startup request, the only one with no \
             reply to measure it off [m]"))
    if !isnan(feas.c1)
        block["c1_pattern"] = (round(feas.c1; digits = 4),
            "turn-rate gain the startup gate and requests read the path at: at the \
             depower the pattern is FLOWN at (the optimizer's own under \
             fly_opt_depower), not depower_setpoint's [1/m]")
        block["gain_scale_flown"] = (round(c1_setpoint / feas.c1; digits = 3),
            "heading_p factor phases 3-4 flew with, c1(depower_setpoint)/c1(flown) \
             for the startup reply; 1.0 when the optimizer's depower is the setpoint [-]")
        block["margin_start"] = (round(feas.feas_start.margin; digits = 2),
            "curvature margin at the starting length; the worst case only when one \
             path is flown throughout — see the docstring [-]")
        block["margin_end"] = (round(feas.feas_end.margin; digits = 2),
            "curvature margin at reelout_l_max [-]")
    end
    if !isnothing(feas.feas_final)
        block["c1_final"] = (round(feas.c1_final; digits = 4),
            "turn-rate gain at depower_final [1/m]")
        block["margin_final"] = (round(feas.feas_final.margin; digits = 2),
            "curvature margin phase 5 flies with: depower_final's c1, at \
             reelout_l_max, on the STARTING path lifted by el_offset_final [-]")
        isnan(margin5.margin) ||
            (block["margin_final_flown"] =
                (round(margin5.margin; digits = 2),
                 "the same, for the last path the re-optimizer installed — the one \
                  phase 5 actually inherited [-]"))
        if !isnothing(st.p5_fallback)
            block["final_fallback_t"] = (round(st.p5_fallback.t; digits = 1),
                "when phase 5's path fell back to an earlier install (final_margin_min) [s]")
            block["final_fallback_to_t"] = (round(st.p5_fallback.to_t; digits = 1),
                "install time of the path it fell back to; 0 = the startup path [s]")
            block["margin_final_fallback"] = (round(st.p5_fallback.to_margin; digits = 2),
                "phase-5 margin of that path, carrying the current lift: the one phase 5 flies [-]")
        end
    end
    return block
end

"""
    traj_opt_block(setup, st, power_block, feasibility, scored) -> OrderedDict

The summary's `traj_opt` section: the power comparison, the guess and what the
optimizer moved, the installed path, the feasibility gates, every
re-optimization, the elevation lift and what it cost (`scored`, from
`score_log`), the droop profile, clearance, inflow and winch.
"""
function traj_opt_block(setup, st::RunState, power_block, feasibility, scored)
    (; fcs, tos, inflow, winch, rc, fec, opt_chain, opt_box, opt_kv_log, opt_depower_log,
       replay_paths, el_center_seed, startup_seed_offset) = setup
    (; sl, fig8m, az_amp_mean, el_h_mean, span_margins, span_worst, el_min_final, lift_mean) = scored
    droop_mean = [st.droop_n[b] > 0 ? st.droop_flown[b] / st.droop_n[b] : NaN
                  for b in 1:st.n_droop_bins]
    OrderedDict{String, Any}(
        "power" => power_block,
        "guess" => OrderedDict(
            "a_deg" => (tos.guess_a, "width of the guess lemniscate; azimuth spans ±a [deg]"),
            "b_deg" => (tos.guess_b, "height of the guess, peak to peak [deg]"),
            "el_center_deg" => (el_center_seed,
                "centre elevation of the guess the startup path was solved from; \
                 guess_el_center_high at and above guess_el_center_wind_ref, plus \
                 the retry offset below [deg]"),
            "el_center_retry_offset_deg" => (startup_seed_offset,
                "offset of the seed the startup solve converged from after a 422 — a \
                 startup_retry_el_offsets entry, or a whole degree walked outward past \
                 them when the failure cache rejected the listed ones; 0 means the \
                 shipped guess converged [deg]"),
            "points" => (tos.guess_points, "points the guess was sent with"),
            "depower_seed_m" => (round(depower_seed(tos, inflow.wind_speed); digits = 3),
                "power-tape length the solve started from, input_depower ramped with \
                 the wind above input_depower_wind_ref; only a seed, the server \
                 optimizes it [m]"),
            "depower_optimized" => OrderedDict(time_keyed(opt_depower_log) do e
                (e.t, (round(e.l_dp; digits = 3),
                       @sprintf("optimizer's l_dp [m]; rel_depower equivalent %.3f \
                                 (awetrim_depower_to_v3kite), against the flown \
                                 depower_setpoint = %.3f",
                                e.u_p_equiv, fcs.course.depower_setpoint)))
            end),
            "depower_optimized_rel" => OrderedDict(time_keyed(opt_depower_log) do e
                (e.t, (round(e.u_p_equiv; digits = 4),
                       "the same reply as V3Kite rel_depower, converted with the \
                        AWETRIM_V3KITE_DEPOWER_OFFSET in force at run time; what a \
                        replot's u_d panel draws [-]"))
            end),
            "k_v_optimized" => OrderedDict(time_keyed(opt_kv_log) do e
                (e.t, (round(e.k_v; digits = 5),
                       @sprintf("optimizer's k_v at L = %.0f m, %+.1f %% of the %.5f \
                                 seed kv of wc_settings%s",
                                e.l, 100 * (e.k_v / winch.k_v - 1), winch.k_v,
                                e.at_bound ? "; AT ITS BRACKET EDGE" : "")))
            end)),
        "path" => OrderedDict(
            "points" => (st.n_path_initial, "points of the path installed before the run"),
            "points_final" => (length(fec.az_path),
                "points of the path at the end; differs when reopt installed one"),
            "az_center_deg" => (round(st.az_c_path; digits = 1), "centre azimuth of the path [deg]"),
            "az_amplitude_deg" => (round(st.az_amp_path; digits = 1),
                "half-width of the STARTUP path; the criteria are scored against the \
                 pattern commanded at each lap, whose mean is lift_budget's reference [deg]"),
            "el_center_deg" => (round(st.el_c_path; digits = 1), "centre elevation of the path [deg]"),
            "el_height_deg" => (round(st.el_height_path; digits = 1),
                "elevation span of the path, peak to peak [deg]"),
            "lobe_lift_pct" => (round(Int, 100 * st.startup_wing_frac),
                "share of el_offset_wing the STARTUP install carried; held back on the \
                 same rungs as a mid-run install when the full lift fails the curvature gate"),
            "downloops" => (st.opt_downloops, "traversal direction the optimizer solved for")),
        "feasibility" => feasibility,
        "reopt" => OrderedDict(
            "enabled" => (tos.reopt_enabled, "re-optimization during the run"),
            "warm_start" => (tos.use_step,
                "/step alone, seeded from the previous optimum; false re-inits from \
                 the parametric guess at every length"),
            "requests" => (st.reopt_n, "solves that completed, accepted or rejected"),
            "blocking" => (tos.reopt_blocking,
                "simulation held while a solve ran"),
            "blocked" => (round(st.reopt_blocked_s; digits = 1),
                "wall time the simulation was frozen waiting for replies [s]"),
            "cache_hits" => (opt_chain.hits,
                "optimizer steps, startup included, served from the solution or failure \
                 cache (OptChain) without asking the server"),
            "cache_misses" => (opt_chain.misses, "optimizer steps sent to the server"),
            "cache_rebuilds" => (opt_chain.rebuilds,
                "server sessions rebuilt from the cache before a miss"),
            "replayed_from" => (isnothing(replay_paths) ? "" : String(replay_paths),
                "scenario whose optimizer results were flown instead of asking the \
                 optimizer (`replay_paths`); empty for a normal run"),
            "installed" => (count(e -> e.status == "installed", st.reopt_events),
                "new paths actually flown"),
            "cycle_wall" => OrderedDict(time_keyed(st.reopt_cycles) do c
                (c.t, (round(c.wall_s; digits = 1),
                       @sprintf("wall time from the first request to the verdict (%s), \
                                 every retry included, at L = %.0f m [s]", c.status, c.l)))
            end),
            "blend_retries" => (st.blend_retries_total,
                "cold-restart attempts spent on a rejected reply — a folded blend, a \
                 collapsed prediction, or a clearance/elevation shortfall re-asked at \
                 a raised floor — across every install this run"),
            "events" => OrderedDict(time_keyed(st.reopt_events) do e
                (e.t, (string(e.status, isempty(e.detail) ? "" : " — " * e.detail),
                       @sprintf("at L = %.0f m", e.l)))
            end)),
        "el_lift" => OrderedDict(
            "lift_deg" => (fcs.reelout.el_offset_final,
                "el_offset_final, the fixed lift of the path once reel-out ends [deg]"),
            "lift_lead" => (fcs.reelout.el_offset_lead,
                "el_offset_lead, how early the lift is allowed to latch; 0 = at the end [s]"),
            "wing_deg" => (fcs.reelout.el_offset_wing,
                "el_offset_wing, extra lift at the lobes, baked into every installed path [deg]"),
            "wing_mode" => (fcs.reelout.el_offset_wing_mode,
                "el_offset_wing_mode, the units el_offset_wing_az/_blend are read in"),
            "wing_az_deg" => (fcs.reelout.el_offset_wing_az,
                "azimuth beyond which that lift is full; it ramps over \
                 el_offset_wing_blend below it [deg or fraction of A]"),
            "lift_t" => (isnan(st.lift_t) ? "never" : round(st.lift_t; digits = 1),
                "when the lift actually latched [s]"),
            "lift_remaining_m" => (isnan(st.lift_remaining) ? "n/a" :
                                   round(st.lift_remaining; digits = 1),
                "reel-out left at that moment; > 0 means the lead fired, 0 means \
                 phase 5 did [m]"),
            "shift_delivery" => OrderedDict(
                @sprintf("t_%05.1f_s", e.t) =>
                    (e.status,
                     @sprintf("%+.2f° of shift, curvature margin %.2f vs %.2f required",
                              e.delta, e.margin, tos.min_feasibility_margin))
                for e in st.el_shift_events)),
        "droop_profile" => OrderedDict(
            vcat(
                [@sprintf("az_%02d_%02d_pct", 100 * (b - 1) ÷ st.n_droop_bins,
                          100 * b ÷ st.n_droop_bins) =>
                     (st.droop_n[b] == 0 ? "n/a" : round(droop_mean[b]; digits = 2),
                      st.droop_n[b] == 0 ? "no samples in this azimuth band" :
                      @sprintf("kite below the path's elevation centre [deg], over %d \
                                samples; the path itself sits %.2f half-spans down there \
                                and the kite %.2f° under it",
                               st.droop_n[b], st.droop_ref[b] / st.droop_n[b],
                               st.droop_sag[b] / st.droop_n[b]))
                 for b in 1:st.n_droop_bins],
                ["centre_to_lobe_deg" =>
                     (all(isnan, droop_mean) || isnan(first(droop_mean)) ? "n/a" :
                      round(maximum(filter(!isnan, droop_mean)) - first(droop_mean);
                            digits = 2),
                      "how much deeper the kite flies in its worst azimuth band than in \
                       the crossing — what a SHAPED lift is aimed at, where \
                       el_offset_final can only move the pattern rigidly [deg]")])),
        "lift_budget" => OrderedDict(
            vcat(
                ["delivered_deg" => (round(lift_mean; digits = 2),
                     "rigid lift the path in the air actually carries — el_offset_final \
                      once it has been delivered [deg]"),
                 "commanded_az_amp_deg" => (round(az_amp_mean; digits = 2),
                     "mean half-width the run was COMMANDED to fly, against the startup \
                      path's in traj_opt.path [deg]"),
                 "commanded_el_height_deg" => (round(el_h_mean; digits = 2),
                     "mean commanded pattern height [deg]"),
                 "wing_deg" => (fcs.reelout.el_offset_wing,
                     "el_offset_wing, the fixed lobe lift baked into every installed path [deg]"),
                 "el_min_run_deg" => (round(fig8m.min_elevation_all; digits = 1),
                     "lowest elevation over the whole run — usually set in PHASE 5 at 4 m/s \
                      and up, so el_offset_final does move it; the entry sets it below that [deg]"),
                 "el_min_final_deg" => (isnan(el_min_final) ? "n/a" :
                                        round(el_min_final; digits = 2),
                     "lowest elevation in phase 5, which is what el_offset_final buys [deg]")],
                [string(m.name, "_deg") =>
                     (round(m.margin; digits = 2),
                      @sprintf("reach margin: flew %.2f° against the %.2f° required by \
                                min_span_frac = %.2f, %+.0f %%", m.flown, m.required,
                               fcs.run.min_span_frac, m.pct))
                 for m in span_margins],
                ["tightest" => (replace(span_worst.name, "_" => " "),
                     @sprintf("the size criterion with the least room, %+.2f° (%+.0f %%) — \
                               what the NEXT lift has to spend", span_worst.margin,
                              span_worst.pct))])),
        "clearance" => OrderedDict(
            "path_min_m" => (round(st.path_min_h_start; digits = 1),
                "lowest point of the PRE-FLIGHT path at the starting length [m]"),
            "flown_min_m" => (round(minimum(first(lt) * sin(el)
                                            for (lt, el) in zip(sl.l_tether, sl.elevation));
                                    digits = 1),
                "lowest point the kite actually reached over the run [m]"),
            "min_required_m" => (tos.min_height, "min_height of data/traj_opt.yaml [m]"),
            "elevation_min_asked_deg" => (
                let b = !isnothing(st.opt_box_now) ? st.opt_box_now : opt_box
                    isnothing(b) || isnothing(b.elevation_min) ? "unset" :
                        round(b.elevation_min; digits = 2)
                end,
                "elevation floor the LAST request carried, the higher of \
                 asind(min_height/L) and min_elevation + candidate_elevation_margin at \
                 the length it was made for; the optimizer constrains HEIGHT, and only \
                 at the far end of the lap's reel-out [deg]"),
            "elevation_min_extra_deg" => (round(st.el_min_extra; digits = 2),
                "raise a rejected reply's shortfall added to that floor, carried \
                 forward once measured; 0 means no reply was gated out low [deg]")),
        "inflow" => OrderedDict(
            "wind_speed_m_s" => (inflow.wind_speed, "wind speed at 6 m sent to the optimizer [m/s]"),
            "wind_direction_deg" => (inflow.wind_direction, "direction the wind comes from [deg]"),
            "profile_law" => (inflow.profile_law, "0=CONST, 1=EXP, 2=LOG, 3=EXPLOG, 4-6=CUSTOM_*")),
        "winch" => OrderedDict(
            "k_v" => (winch.k_v, "v_set = k_v * sqrt(force) sent to the optimizer [-]"),
            "optimize_k_v" => (winch.optimize_k_v,
                "k_v sent as a design variable; the reply's value is then the one flown"),
            # `rc.wcs`: the gain the RUN flew is the one in the object the reel-out
            # law actually read, not the one the summary would like it to be.
            "k_v_flown" => (round(rc.wcs.kv; digits = 5),
                isempty(opt_kv_log) ?
                    "k_v the run actually flew; == k_v, the optimizer did not move it" :
                    @sprintf("k_v the run actually flew, last of %d optimizer change(s)%s",
                             length(opt_kv_log),
                             any(e -> e.at_bound, opt_kv_log) ?
                                 "; one hit the bracket edge, widen it to chase further" : "")),
            "f_min_N" => (winch.f_min, "minimum winch force sent [N]"),
            "f_max_N" => (winch.f_max, "maximum winch force sent [N]"),
            "force_limit" => (rc.wcs.force_limit,
                rc.wcs.force_limit == "soft" ?
                    @sprintf("the reel-out law itself limits the force, inverting the \
                              optimizer's own saturating tension curve (beta %.0e/%.0e, \
                              force filtered at tau = %.2f s); the UpperForceController is \
                              held in reset, so upper_force_pct is 0 by construction",
                             rc.wcs.softminus_beta, rc.wcs.softplus_beta, rc.wcs.force_limit_tau) :
                    "bare kv*sqrt(force) with the two force controllers switching in at the limits")))
end

"""
    opt_performance(setup, st, timing, cycle) -> (; block, t_total)

The summary's `performance` section (`performance_block`), with the rates excluding
the time the loop was frozen for re-optimization and three entries only this run has:
the whole script's wall time `t_total` (from `timing.t_script_start`), the time spent
waiting for the optimizer and the longest optimization (`cycle`, from `opt_cycle_max`).
`block` is `nothing` when no simulated time elapsed.
"""
function opt_performance(setup, st::RunState, timing, cycle)
    (; s, fcs, opt_chain, opt_startup_solve_s) = setup
    (; t_script_start, t_wall, t_sim) = timing
    t_total = time() - t_script_start
    # Everything the script spent waiting for the solver, wherever it fell: the
    # startup solve holds the script before the loop exists, the blocking
    # re-optimizations freeze the loop itself.
    t_opt = opt_startup_solve_s + st.reopt_blocked_s
    block = performance_block(t_sim, t_wall, s.dt, fcs.run.vsm_interval;
        blocked_s = st.reopt_blocked_s,
        extra = Pair{String, Any}[
            "total_wall_time" => (round(t_total; digits = 1),
                "the whole script: package loading, init, settling, the startup solve \
                 and the run, up to this summary — the archive copy and any plots \
                 follow it [s]"),
            "optimization_time" => (round(t_opt; digits = 1),
                "wall time waiting for the optimizer: the startup solve \
                 ($(round(opt_startup_solve_s; digits = 1)) s) plus every blocking \
                 re-optimization (traj_opt.reopt.blocked) [s]"),
            "max_optimization_time" => (round(cycle.s; digits = 1), cycle.comment)])
    if !isnothing(block)
        @printf("               %.0f s from the first line of the script, %.0f s of it \
                 waiting for the optimizer (%.0f s at startup).\n",
                t_total, t_opt, opt_startup_solve_s)
        @printf("               Optimizer steps: %d served from the cache, %d sent, %d \
                 session rebuilds.\n", opt_chain.hits, opt_chain.misses, opt_chain.rebuilds)
    end
    return (; block, t_total)
end

"""
    recap_block(setup, st, timing, run_time, scored, p4, power, cycle, t_total) -> OrderedDict

A recap of the numbers scattered above, the summary's `summary` section, written
last so it reads as the file's TL;DR without displacing `success_criteria` as the
first key. Also printed, coloured, as the run's last word.
"""
function recap_block(setup, st::RunState, timing, run_time, scored, p4, power, cycle, t_total)
    (; fcs, inflow, c1_setpoint, c1_depower_max, c1_at_depower) = setup
    (; t_wall, t_sim) = timing
    (; fig8m, laps_flown) = scored
    (; opt_power_meas, opt_power_pred_eff, fs_ref) = power
    summary_block = OrderedDict{String, Any}(
        "date" => (Dates.format(run_time, "yyyy-mm-dd"), "wall-clock date the run finished"),
        "time" => (Dates.format(run_time, "HH:MM:SS"), "wall-clock time the run finished"),
        "wind_speed_gnd" => (inflow.wind_speed, "wind speed at 6 m sent to the optimizer [m/s]"))
    if !isnothing(opt_power_meas)
        summary_block["power_ratio"] = (round(opt_power_meas / opt_power_pred_eff; digits = 2),
            "measured / predicted reel-out power [-]")
        # The one to read at low wind: force_law's prediction is against the k_v law
        # including its soft floor, which at 3 m/s stands the winch still and has come
        # back NEGATIVE. This is against an upper bound over any winch in
        # [f_min, f_max], so above 1 is a real disagreement about the physics.
        # Absent when power_ratio is already above FREE_SPEED_RATIO_MAX (or the
        # force never dropped low): the reference solves are skipped there.
        isnothing(fs_ref) || (summary_block["power_ratio_free_speed"] =
            (round(opt_power_meas / fs_ref.weighted; digits = 2),
             "measured / free_speed reference reel-out power ($(round(Int, fs_ref.weighted)) W, \
              an UPPER BOUND over any winch in [f_min, f_max]); above 1 cannot be \
              explained by any winch law [-]"))
    end
    summary_block["success_criteria"] = success_verdict(fig8m)
    fig8m === nothing || (summary_block["cross_track_rms_deg"] = (round(fig8m.rms_d; digits = 2),
        "RMS cross-track error, settled window [deg]"))
    if t_sim > 0
        summary_block["total_wall_time"] = (round(t_total; digits = 1), "the whole script, start to this summary [s]")
        summary_block["realtime_factor"] = (round(t_sim / max(t_wall - st.reopt_blocked_s, eps()); digits = 2),
            "sim_time / wall_time, excluding time frozen for re-optimization")
    end
    summary_block["optimization_requests"] = (st.reopt_n, "solves that completed, accepted or rejected")
    summary_block["optimizations_installed"] = (count(e -> e.status == "installed", st.reopt_events),
        "new paths actually flown")
    isempty(laps_flown.dt) || (summary_block["fastest_fig8"] = (round(minimum(laps_flown.dt); digits = 1),
        "shortest time for flying one full figure of eight [s]"))
    summary_block["max_optimization_time"] = (round(cycle.s; digits = 1),
        "longest wall time to compute a new figure of eight, retries included [s]")
    if !isnothing(p4)
        summary_block["av_power_ro"] = (round(Int, p4.power.av), "mean reel-out power over phase four [W]")
        summary_block["min_power_ro"] = (round(Int, p4.power.min), "min reel-out power over phase four, last 2 s excluded [W]")
        summary_block["max_power_ro"] = (round(Int, p4.power.max), "max reel-out power over phase four [W]")
        summary_block["min_force_ro"] = (round(Int, p4.force.min), "min tether force over phase four, last 2 s excluded [N]")
        summary_block["av_force_ro"] = (round(Int, p4.force.av), "mean tether force over phase four [N]")
        summary_block["max_force_ro"] = (round(Int, p4.force.max), "max tether force over phase four [N]")
        summary_block["v_ro_min"] = (round(p4.v_ro.min; digits = 2), "min reel-out speed over phase four, last 2 s excluded [m/s]")
        summary_block["v_ro_av"] = (round(p4.v_ro.av; digits = 2), "mean reel-out speed over phase four [m/s]")
        summary_block["v_ro_max"] = (round(p4.v_ro.max; digits = 2), "max reel-out speed over phase four [m/s]")
        summary_block["av_depower_ro"] = (round(p4.depower_av; digits = 3), "mean KCU depower over phase four [-]")
        # The floor the limiter integrates above: depower_final, or the depower the
        # stop latched at when that is higher (simple_opt_reelout.jl's stop ramp).
        dp_final_floor = isnan(st.stop_dp_entry) ? fcs.reelout.depower_final : max(fcs.reelout.depower_final, st.stop_dp_entry)
        summary_block["max_depower_final"] = (round(min(dp_final_floor + st.dp_final_extra_peak,
                                                        max(fcs.reelout.depower_final_max, dp_final_floor)); digits = 3),
            "highest depower the force limiter asked for, from the stop latch through phase 5; \
             the phase-5 floor itself when it never engaged or is off (depower_final_max == depower_final) [-]")
        # The phase-5 counterpart of feasibility.gain_scale_flown, at the limiter's
        # peak: what simple_opt_reelout.jl scaled heading_p by there, saturated at
        # the table's usable edge exactly as the run was.
        if isfinite(c1_setpoint) && isfinite(c1_depower_max)
            dp5_peak = round(min(fcs.reelout.depower_final + st.dp_final_extra_peak,
                                 fcs.reelout.depower_final_max, c1_depower_max); digits = 3)
            c1_5 = c1_at_depower(dp5_peak)
            summary_block["gain_scale_final_peak"] = (round(isfinite(c1_5) ? c1_setpoint / c1_5 : 1.0; digits = 3),
                "heading_p factor phase 5 flew with at the limiter's peak, \
                 c1(depower_setpoint)/c1(flown), read at $(dp5_peak) [-]")
        end
    end
    return summary_block
end

"""
    write_summary_files(setup, st, summary) -> opt_paths_file

Write the summary YAML next to the log, and `<log>_opt_paths.yaml`: every path
the optimizer returned that the run went on to fly, AS IT ARRIVED — before
`el_offset_wing` and `el_offset_final` were added. The logged attractor walks the
CORRECTED path, so it rises with the correction and cannot show what the
correction did; these are the curves the kite is meant to land on. Its own file
next to the log, archived with it, so `plot_pattern_scenario` can draw them for
an archived run as well as a live one. Each carries the sim time and phase it
was installed at, so the plot can leave out what only phase 5 flew.
"""
function write_summary_files(setup, st::RunState, summary)
    (; output_path, log_name) = setup
    open(joinpath(output_path, log_name * ".yaml"), "w") do io
        println(io, "# Run summary for output/", log_name,
                ".arrow, written by examples/simple_reelout.jl at the end of the run.")
        write_yaml_commented(io, 0, summary)
    end
    opt_paths_file = joinpath(output_path, log_name * "_opt_paths.yaml")
    if !isempty(st.opt_paths_raw)
        YAML.write_file(opt_paths_file, Dict(
            "paths" => [Dict("installed_t" => round(t_at; digits = 2),
                             "installed_phase" => ph_at,
                             "azimuth" => round.(Float64.(paz); digits = 3),
                             "elevation" => round.(Float64.(pel); digits = 3))
                        for ((paz, pel), (t_at, ph_at)) in zip(st.opt_paths_raw, st.opt_paths_at)]))
    elseif isfile(opt_paths_file)
        rm(opt_paths_file)   # a stale one from an earlier run would be drawn as this run's
    end
    return opt_paths_file
end

# ==================== ARCHIVE ==================== #

"""
    archive_run(setup, run_time, opt_paths_file) -> archive_dir | "none"

One timestamped folder per run under output/archives/, so the exact config
that produced a log survives even after the next run overwrites output/*.
The input `run_archive = false` skips it: a sweep writes
one folder per grid point otherwise, each with a copy of the 40 MB arrow log,
and its own results table already records what distinguished the runs.
"""
function archive_run(setup, run_time, opt_paths_file)
    (; inputs, project, project_set, output_path, log_name) = setup
    if !inputs.run_archive
        @info "Archiving suppressed by run_archive = false."
        return "none"
    end
    input_files = [
        run_input_files(project, project_set);
        joinpath(skc_data_path(), winch_table_file(project)), # f_low/force_limit(v_wind) table
        # The identified c1/c2/delay: they set the steering response and the
        # curvature gate, and a re-identification replaces the rows in place.
        joinpath(skc_data_path(), turn_rate_coeffs_file(project)), # turn-rate law
        # Without this the archive cannot reproduce its own run: the guess decides
        # WHICH optimum the solve converges to, and reopt_*/min_feasibility_margin
        # decide what is re-anchored and what is flown.
        joinpath(skc_data_path(), traj_opt_settings_file(project)), # optimizer guess and knobs
    ]
    output_files = [
        joinpath(output_path, log_name * ".arrow"),
        joinpath(output_path, log_name * ".yaml"),
        opt_paths_file,                                       # optimizer's uncorrected curves
    ]
    return archive_run_files(output_path, run_time, input_files, output_files)
end

"""
    draw_plots(setup) -> plots_failed

`simple_reelout_plots.jl`, handed the flown curve and this run's log name in
`REF_PATH` and `LOG_NAME`, so the plots draw the optimized path and load the
`_opt` log instead of the lemniscate run's; the optimizer's raw curves reach them
through `<log>_opt_paths.yaml`. Returns the exception the plots threw, or
`nothing`: they are cosmetic and the run is already scored, logged and archived
by here, so a GLMakie failure (needs the main thread, so an eval off it throws)
must not cost the marker that tells a watcher the run is over.
"""
function draw_plots(setup)
    if !setup.show_plots
        @info "Plots suppressed by show_plots = false."
        return nothing
    end
    global REF_PATH = (setup.fec.az_path, setup.fec.el_path)
    global LOG_NAME = setup.log_name
    try
        include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
    catch exc
        @error "Plots failed; the run itself completed and is archived." exception =
            (exc, catch_backtrace())
        return exc
    end
    return nothing
end

"""
    reelout_results(setup, st, timing) -> (; summary, fig8m, archive_dir, plots_failed)

Score the saved log, print the results block, write the summary YAML and the
optimizer's paths next to the log, archive the run, draw the plots and write the
finished-run marker, in that order. `timing` holds the script's `t_script_start`,
`run_script`, `t_wall` and `t_sim`. The marker's fields are kept in `st` as soon
as they are known, so a throw on the way still leaves them to the `FAILED` marker
the script writes.
"""
function reelout_results(setup, st::RunState, timing)
    scored = score_log(setup, st)
    summary = OrderedDict{String, Any}()
    # The FIRST key of the file, the same words the console logs as "Success criteria:
    # …". It is the one line a reader looks for, so it is not buried at the end of
    # `fig8_metrics:` — that section keeps the numbers the verdict was computed from.
    # A failure names the criteria that broke, exactly as the console does.
    # `examples/wind_scan.jl`, which reads these summaries out of the archives, falls
    # back to the old nested position so runs flown before this still parse.
    summary["success_criteria"] = success_verdict(scored.fig8m)
    run_time = Dates.now()
    summary["simulation"] = simulation_block(timing.run_script, setup.project_name,
                                             setup.turbulence, setup.project_set.v_wind, run_time)
    # The verdict these numbers were scored into is the file's first key now.
    scored.fig8m === nothing ||
        (summary["fig8_metrics"] = fig8_metrics_block(scored.fig8m, scored.laps_flown;
                                                      cross_track_ref = "the unlifted optimizer path"))
    reelout = reelout_block(scored.sl, setup.fcs, setup.l_tether; stop_reason = st.stop_reason,
                            laps_reeled = st.fig8_idx_progress / st.n_path)
    summary["reelout"] = reelout.block
    power = power_comparison(setup, st, scored.sl, reelout.rp, reelout.p4)
    summary["traj_opt"] = traj_opt_block(setup, st, power.block, feasibility_block(setup, st), scored)
    cycle = opt_cycle_max(st.reopt_cycles, setup.opt_startup_solve_s)
    performance = opt_performance(setup, st, timing, cycle)
    isnothing(performance.block) || (summary["performance"] = performance.block)
    summary_block = recap_block(setup, st, timing, run_time, scored, reelout.p4, power, cycle,
                                performance.t_total)
    summary["summary"] = summary_block

    opt_paths_file = write_summary_files(setup, st, summary)
    st.archive_dir = archive_run(setup, run_time, opt_paths_file)
    plots_failed = draw_plots(setup)

    isnothing(power.power_summary) || @info power.power_summary
    printstyled("\nSummary:\n"; bold = true)
    write_yaml_commented(stdout, 1, summary_block; color = true)

    # Defined in simple_opt_reelout.jl before anything that can throw, and called
    # from its `catch` too — see the docstring there.
    write_run_done(setup, st, isnothing(plots_failed) ? "ok" : "ok (plots failed)")
    return (; summary, scored.fig8m, st.archive_dir, plots_failed)
end
