# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    reelout_results.jl — scoring, summary, archive and plots of an optimized reel-out run.

`include`d by `simple_opt_reelout.jl` as its last step, after the log has been
saved. It reads the run's globals (`fcs`, `tos`, `sl`, `reopt_events`,
`el_applied`, `opt_power_pred`, `pred_timeline`, ...), reloads the log, prints
the results block, writes the summary YAML next to it, copies both plus every
input YAML into a timestamped archive folder, draws the plots and writes the
finished-run marker. Nothing here touches the plant, so a change to it is
checked by re-scoring an existing log rather than by flying again.
"""

# ==================== RESULTS ==================== #

"""
    write_yaml_commented(io, indent, node)

Serialize a nested `OrderedDict` as YAML, recursing into `OrderedDict` values
and appending a trailing `# comment` for leaves given as `(value, comment)`
pairs, aligned to `comment_col` where the line is short enough.
`YAML.write_file` has no concept of comments, hence this by hand.
"""
function write_yaml_commented(io, indent, node; comment_col = 36)
    pad = "  "^indent
    for (k, v) in node
        if v isa AbstractDict
            println(io, pad, k, ":")
            write_yaml_commented(io, indent + 1, v; comment_col)
        else
            value, comment = v isa Tuple ? v : (v, "")
            val = value isa AbstractString ? "\"$value\"" : string(value)
            prefix = string(pad, k, ": ", val)
            if isempty(comment)
                println(io, prefix)
            else
                println(io, prefix, " "^max(1, comment_col - length(prefix)), "# ", comment)
            end
        end
    end
end

syslog = load_log(log_name; path = output_path)
sl = syslog.syslog
# The geometry is passed in too: without it the criteria are blind to pattern SIZE.
# require_final: this script's own phase 5, unlike simple_fig8.jl's sys_state
# (which never goes past 4) — checks reel-out actually finished within the run.
# From the flown path, not from fcs.f8_*. `az_center` is in RADIANS here (it is
# compared against the logged azimuth), the two extents in degrees.
"""
    on_log(t_log, t_src, v_src) -> Vector{Float64}

Per-log-sample view of a per-step series, taking the last value recorded at or
before each log timestamp. Both time vectors must be ascending. Used to hand
`fig8_metrics` the geometry that was COMMANDED at each sample rather than one
number for the run, without depending on the logger writing exactly one row per
step.
"""
function on_log(t_log, t_src, v_src)
    out = zeros(Float64, length(t_log))
    j = 1
    for (k, t) in enumerate(t_log)
        while j < length(t_src) && t_src[j + 1] <= t
            j += 1
        end
        out[k] = v_src[j]
    end
    return out
end

# The size criteria against the pattern actually commanded, lap by lap. The
# startup path is only the FIRST of them, and it is the widest the run ever flies:
# scored against it, the last laps are asked for a reach nothing ever commanded.
t_log = Float64.(sl.time)
have_geom = !isempty(geom_t)
az_c_log = have_geom ? deg2rad.(on_log(t_log, geom_t, geom_az_c)) : deg2rad(az_c_path)
az_amp_log = have_geom ? on_log(t_log, geom_t, geom_az_amp) : az_amp_path
el_h_log = have_geom ? on_log(t_log, geom_t, geom_el_h) : el_height_path
fig8m = print_fig8_metrics(sl; t_start = fcs.park_time, settle_time = fcs.entry_time,
                   min_elevation = fcs.min_elevation, az_center = az_c_log,
                   az_amplitude = az_amp_log, el_height = el_h_log,
                   min_span_frac = fcs.min_span_frac, require_final = true)

# What every lift costs on the OTHER axis. A pattern raised is a pattern flown
# narrower, and the three size criteria are the first thing a lift breaks — the
# 2.5 s el_offset_lead failed the azimuth reach by hundredths of a degree while
# passing everything else. Reported next to the elevation that lift bought, so a
# run says both halves of the trade rather than one.
az_amp_mean = have_geom ? mean(az_amp_log) : az_amp_path
el_h_mean = have_geom ? mean(el_h_log) : el_height_path
# In FILL fractions, since that is what the criteria are scored on once the
# commanded pattern moves; the degrees are the same margin read against the mean
# geometry, for a number that can be compared with a lift.
span_checks = [("azimuth_reach_pos", fig8m.az_fill_pos, az_amp_mean),
               ("azimuth_reach_neg", fig8m.az_fill_neg, az_amp_mean),
               ("elevation_span", fig8m.el_fill, el_h_mean)]
span_margins = [(; name, fill, flown = fill * ref,
                 required = fcs.min_span_frac * ref,
                 margin = (fill - fcs.min_span_frac) * ref,
                 pct = 100 * (fill / fcs.min_span_frac - 1))
                for (name, fill, ref) in span_checks]
span_worst = argmin(m -> m.margin, span_margins)
i_final = findall(==(5), Int.(sl.sys_state))
el_min_final = isempty(i_final) ? NaN : rad2deg(minimum(Float64.(sl.elevation[i_final])))
# What the path in the air actually carries, not what was asked for: the bias only
# reaches the kite through an install or an in-air blend that the curvature gate
# can refuse.
lift_mean = mean(el_applied)
@info @sprintf("Lift budget: %+.2f° of correction delivered (learnt %s, lobes up \
                to %+.2f°); the tightest size criterion is %s with %+.2f° (%+.0f %%) \
                to spare; min elevation %.1f° over the run, %.1f° in phase 5.",
               lift_mean, prof_str(el_applied), fcs.el_offset_wing,
               replace(span_worst.name, "_" => " "), span_worst.margin,
               span_worst.pct, fig8m.min_elevation_all, el_min_final)

summary = OrderedDict{String, Any}()
# The FIRST key of the file, the same words the console logs as "Success criteria:
# …". It is the one line a reader looks for, so it is not buried at the end of
# `fig8_metrics:` — that section keeps the numbers the verdict was computed from.
# A failure names the criteria that broke, exactly as the console does.
# `examples/wind_scan.jl`, which reads these summaries out of the archives, falls
# back to the old nested position so runs flown before this still parse.
summary["success_criteria"] = (fig8m === nothing ? "not scored — no settled samples" :
    isempty(fig8m.criteria_failed) ? "all $(fig8m.criteria) passed" :
    "FAILED: " * join(fig8m.criteria_failed, ", "),
    "pass/fail verdict vs V3Kite's success criteria")
run_time = Dates.now()
# The package repo, not `examples/` — this reports the controller code, not the script's own project.
pkg_dir = pkgdir(SimpleKiteControllers)
git_hash, git_status = try
    hash = strip(read(`git -C $pkg_dir rev-parse --short HEAD`, String))
    dirty = !isempty(strip(read(`git -C $pkg_dir status --porcelain`, String)))
    (hash, dirty ? "dirty" : "clean")
catch
    ("unknown", "unknown")
end
summary["simulation"] = OrderedDict{String, Any}(
    "script" => (run_script, "script that produced this run"),
    "project" => (PROJECT, "system project flown"),
    "rel_turbulence" => (TURBULENCE, "turbulence level in [0, 1] passed to init"),
    "wind_speed" => (project_set.v_wind, "mean wind speed in m/s passed to init"),
    "time" => (Dates.format(run_time, "HH:MM:SS"), "wall-clock time the run finished"),
    "date" => (Dates.format(run_time, "yyyy-mm-dd"), "wall-clock date the run finished"),
    "hostname" => (gethostname(), "machine the run executed on"),
    "git_hash" => (git_hash, "SimpleKiteControllers.jl commit hash"),
    "git_status" => (git_status, "SimpleKiteControllers.jl working tree: clean or dirty"))
if fig8m !== nothing
    summary["fig8_metrics"] = OrderedDict{String, Any}(
        "settled_from_s" => (round(fig8m.stats_start; digits = 1), "sim time the scoring window begins [s]"),
        "settle_time_s" => (round(fig8m.settle_time_used; digits = 1), "time after t_start to converge [s]"),
        "laps" => (fig8m.laps, "figure-eight laps completed"),
        "cross_track_deg" => OrderedDict(
            "rms" => (round(fig8m.rms_d; digits = 2), "RMS cross-track error [deg]"),
            "mean" => (round(fig8m.mean_d; digits = 2), "mean cross-track error [deg]"),
            "max" => (round(fig8m.max_d; digits = 2), "max cross-track error [deg]")),
        "elevation_deg" => OrderedDict(
            "min_settled" => (round(fig8m.min_elevation_settled; digits = 1), "min elevation, settled window [deg]"),
            "min_whole_run" => (round(fig8m.min_elevation_all; digits = 1), "min elevation, whole run [deg]")),
        "peak_turn_rate_deg_s" => (round(Int, fig8m.max_turn_rate), "peak heading rate [deg/s]"),
        "extent" => OrderedDict(
            "azimuth_deg" => OrderedDict(
                "min" => (round(-fig8m.az_reach_neg; digits = 1), "flown azimuth reach, negative side [deg]"),
                "max" => (round(fig8m.az_reach_pos; digits = 1), "flown azimuth reach, positive side [deg]")),
            "azimuth_pct_of_amplitude" => OrderedDict(
                "min" => (round(Int, 100 * fig8m.az_fill_neg), "negative reach as % of the pattern's ±A"),
                "max" => (round(Int, 100 * fig8m.az_fill_pos), "positive reach as % of the pattern's ±A")),
            "worst_lobe_deg" => OrderedDict(
                "min" => (round(-fig8m.az_reach_neg_worst; digits = 1), "weakest lobe, negative side [deg]"),
                "max" => (round(fig8m.az_reach_pos_worst; digits = 1), "weakest lobe, positive side [deg]")),
            "elevation_span_deg" => (round(fig8m.el_span; digits = 1),
                "flown elevation span over the whole settled window — the pattern \
                 DESCENDS as the tether grows, so this is mostly that descent [deg]"),
            "elevation_span_lap_deg" => (round(fig8m.el_span_lap; digits = 1),
                "flown elevation span per lobe, which is the pattern's own height [deg]"),
            "elevation_span_pct_of_b" => (round(Int, 100 * fig8m.el_fill),
                "flown span as % of the B commanded where it was flown")),
        "tether_force_N" => OrderedDict(
            "mean" => (round(Int, fig8m.mean_force), "mean tether force, settled window [N]"),
            "std" => (round(Int, fig8m.std_force), "std tether force, settled window [N]"),
            "cv_pct" => (round(100 * fig8m.cv_force; digits = 1), "coefficient of variation [%]")),
        "steering" => OrderedDict(
            "peak_abs_u_s" => (round(fig8m.max_steering_used; digits = 3), "peak |rel_steering| commanded"),
            "pct_time_within_2pct_of_peak" => (round(Int, 100 * fig8m.steering_sat_frac),
                "% of time within 2% of peak (saturation)"),
            "hf_std_steering" => (round(fig8m.steering_hf_std; digits = 4), "high-frequency std of steering (chatter)"),
            "hf_std_turnrate_deg_s" => (round(fig8m.turnrate_hf_std; digits = 2),
                "high-frequency std of heading rate [deg/s]")),
        "tape" => OrderedDict(
            "delivered_peak_abs_u_s" => (round(fig8m.max_steering_delivered; digits = 3),
                "peak steering the KCU tape delivered"),
            "commanded_peak_abs_u_s" => (round(fig8m.max_steering_used; digits = 3), "peak steering commanded"),
            "rate_limited_pct_time" => (round(Int, 100 * fig8m.tape_rate_frac),
                "% of time the tape's rate limit was hit"),
            "rate_limited_peak_per_s" => (round(fig8m.max_tape_rate; digits = 3), "peak tape rate reached [1/s]"),
            "rate_limit_per_s" => (fig8m.v_steering, "KCU's configured rate limit [1/s]")))
    # The verdict these numbers were scored into is the file's first key now.
end

reelout_summary = OrderedDict{String, Any}()
# On the LOGGED PHASE, not a time window; the mean is what v_app_ref should be.
fig8 = findall(x -> Int(x) == 4, sl.sys_state)
if isempty(fig8)
    # Reel-out runs from phase 3, so this is about the anchor only.
    @warn "Phase 4 never reached — no fig8 apparent wind speed."
else
    va = Float64.(sl.v_app[fig8])
    duration = sl.time[fig8[end]] - sl.time[fig8[1]]
    @printf("  v_app over phase 4 (%.1f s): mean %.2f m/s, range %.2f … %.2f m/s \
             | v_app_ref = %.1f (%+.1f%%)\n",
            duration, mean(va), minimum(va), maximum(va),
            fcs.v_app_ref, 100 * (mean(va) / fcs.v_app_ref - 1))
    reelout_summary["v_app_phase4"] = OrderedDict(
        "duration_s" => (round(duration; digits = 1), "phase-4 window length [s]"),
        "mean_m_s" => (round(mean(va); digits = 2), "mean apparent wind speed [m/s]"),
        "min_m_s" => (round(minimum(va); digits = 2), "min apparent wind speed [m/s]"),
        "max_m_s" => (round(maximum(va); digits = 2), "max apparent wind speed [m/s]"),
        "v_app_ref_m_s" => (fcs.v_app_ref, "reference apparent wind speed [m/s]"),
        "deviation_pct" => (round(100 * (mean(va) / fcs.v_app_ref - 1); digits = 1),
            "mean v_app deviation from v_app_ref [%]"))
end
# Unconditional: the winch is gated on phase 3, which phase 4 may never follow.
# stop_reason is "" when the run ended with reel-out still going.
reelout_stop_reason = isempty(stop_reason) ? "none" : stop_reason
@printf("  Tether: %.1f m -> %.1f m (target %.1f m, stopped by: %s).\n",
        l_tether, sl.var_10[end], fcs.reelout_l_max, reelout_stop_reason)
reelout_summary["tether"] = OrderedDict(
    "start_m" => (l_tether, "tether length at run start [m]"),
    "end_m" => (round(Float64(sl.var_10[end]); digits = 1), "tether length at run end [m]"),
    "target_m" => (fcs.reelout_l_max, "reelout_l_max target [m]"),
    "stop_reason" => (reelout_stop_reason, "criterion that ended reel-out: length, laps, or none"),
    "laps_reeled" => (round(fig8_idx_progress / n_path; digits = 2),
        "figure-eight laps completed by the time reel-out ended"))

rp = reelout_power(sl)
if isnothing(rp)
    @warn "Tether never reeled out — no reel-out power to report."
else
    @printf("  Reel-out force: mean %.0f N, peak %.0f N (cf=%.2f).\n",
            rp.mean_force, rp.peak_force, rp.cf_force_ro)
    # Both totals: an earlier engagement trades mean power for a longer window.
    @printf("  Reel-out power: mean %.0f W, peak %.0f W (cf=%.2f) over %.1f s (%d samples), \
             E = %.1f kJ; whole run E = %.1f kJ.\n",
            rp.mean_power, rp.peak_power, rp.cf_power_ro, rp.duration, rp.n,
            rp.energy / 1000, rp.energy_run / 1000)
    reelout_summary["force"] = OrderedDict(
        "mean_N" => (round(Int, rp.mean_force), "mean tether force over the reeling window [N]"),
        "peak_N" => (round(Int, rp.peak_force), "peak tether force over the reeling window [N]"),
        "cf_force_ro" => (round(rp.cf_force_ro; digits = 2), "crest factor: peak_N / mean_N"))
    reelout_summary["power"] = OrderedDict(
        "mean_W" => (round(Int, rp.mean_power), "mean reel-out power over the reeling window [W]"),
        "peak_W" => (round(Int, rp.peak_power), "peak reel-out power over the reeling window [W]"),
        "cf_power_ro" => (round(rp.cf_power_ro; digits = 2), "crest factor: peak_W / mean_W"),
        "duration_s" => (round(rp.duration; digits = 1), "reeling window length [s]"),
        "n_samples" => (rp.n, "sample count in the reeling window"),
        "energy_kJ" => (round(rp.energy / 1000; digits = 1), "energy over the reeling window [kJ]"),
        "energy_run_kJ" => (round(rp.energy_run / 1000; digits = 1), "energy over the whole run [kJ]"))
end

ws = winch_state_pct(sl)
if isnothing(ws)
    @warn "Tether never reeled out — no winch controller states to report."
else
    @printf("  Winch states over the reeling window: speed %.1f %%, lower force %.1f %%, \
             upper force %.1f %%.\n",
            ws.speed_pct, ws.lower_force_pct, ws.upper_force_pct)
    reelout_summary["winch_state"] = OrderedDict(
        "speed_pct" => (round(ws.speed_pct; digits = 1),
            "% of the reeling window in speed control (state 1)"),
        "lower_force_pct" => (round(ws.lower_force_pct; digits = 1),
            "% in the LowerForceController, reeling in (state 0)"),
        "upper_force_pct" => (round(ws.upper_force_pct; digits = 1),
            "% in the UpperForceController, force capped at f_high (state 2)"),
        "n_samples" => (ws.n, "sample count in the reeling window"))
end

rr = reelout_ringing(sl)
if isnothing(rr)
    @warn "Tether never reeled out — no ringing to report."
elseif rr.n_peaks == 0
    @printf("  Reel-out ring: none detected (peak %.2f m/s vs steady %.2f m/s).\n",
            rr.peak_v_reelout_m_s, rr.steady_v_reelout_m_s)
    reelout_summary["ringing"] = OrderedDict(
        "n_peaks" => (0, "ring peaks detected above peak_floor"),
        "peak_v_reelout_m_s" => (round(rr.peak_v_reelout_m_s; digits = 2), "raw v_reelout max within ring_span [m/s]"),
        "steady_v_reelout_m_s" => (round(rr.steady_v_reelout_m_s; digits = 2), "mean v_reelout after ring_span [m/s]"))
else
    @printf("  Reel-out ring: period %.2f s, zeta %.2f, overshoot %.2f m/s, decays in %.1f s \
             (peak %.2f m/s vs steady %.2f m/s).\n",
            rr.period_s, rr.zeta, rr.overshoot_m_s, rr.duration_s,
            rr.peak_v_reelout_m_s, rr.steady_v_reelout_m_s)
    reelout_summary["ringing"] = OrderedDict(
        "n_peaks" => (rr.n_peaks, "ring peaks detected above peak_floor"),
        "period_s" => (round(rr.period_s; digits = 2), "mean peak-to-peak ring period [s]"),
        "zeta" => (round(rr.zeta; digits = 3), "damping ratio from the peak log decrement"),
        "overshoot_m_s" => (round(rr.overshoot_m_s; digits = 2), "first ring peak's amplitude above the local trend [m/s]"),
        "duration_s" => (round(rr.duration_s; digits = 1), "time until the ring decays below settle_frac of overshoot_m_s [s]"),
        "peak_v_reelout_m_s" => (round(rr.peak_v_reelout_m_s; digits = 2), "raw v_reelout max within ring_span [m/s]"),
        "steady_v_reelout_m_s" => (round(rr.steady_v_reelout_m_s; digits = 2), "mean v_reelout after ring_span [m/s]"))
end
summary["reelout"] = reelout_summary

# The comparison this script exists for: what the optimizer promised against what
# reeling out delivered. `rp` is `nothing` when the tether never reeled out, and
# then there is nothing to compare.
#
# The prediction is a WEIGHTED one whenever a re-optimization installed a path. Each
# path carries its own `avg_power_W`, and the run flies each for part of the reeling
# window, so scoring the whole window against the FIRST path's number compares the
# measurement to a path that was not in the air for some of it. `rp.idx` is that
# window's samples, so every share below is measured over exactly the samples
# `measured_W` averages.
opt_power_meas = isnothing(rp) ? nothing : rp.mean_power
pred_shares = NamedTuple[]
opt_power_pred_eff = opt_power_pred
if !isnothing(rp)
    t_ro = Float64.(sl.time[rp.idx])
    # Which timeline entry was current at each reeling sample.
    which = [findlast(e -> e.t <= tq, pred_timeline) for tq in t_ro]
    for (k, e) in enumerate(pred_timeline)
        share = count(==(k), which) / length(which)
        share > 0 && push!(pred_shares, (; from_s = e.t, power = e.power, share))
    end
    opt_power_pred_eff = sum(p.power * p.share for p in pred_shares)
end
if !isnothing(opt_power_meas)
    # Built once and repeated as an `@info` after the archive line: this is the
    # comparison the script exists for, and here it is buried in the results block.
    global power_summary = @sprintf("Optimizer predicted %.0f W of mean reel-out \
                                     power%s; the run measured %.0f W (%.2f x).",
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
    "predicted_initial_W" => (round(Int, opt_power_pred),
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
feasibility_block = OrderedDict{String, Any}(
    "min_required" => (tos.min_feasibility_margin,
        "min_feasibility_margin of data/traj_opt.yaml [-]"),
    "turn_radius_headroom" => (tos.turn_radius_headroom,
        "factor on the gate's number for what the GATE adds: its \
         finite-difference curvature estimate and the elevation lift [-]"),
    "turn_radius_request_m" => (isnothing(opt_r_min) ? "unset" :
                                round(opt_r_min; digits = 2),
        "minimum turn radius asked of the optimizer for the LAST request; the \
         gate's margin/(c1*max_steering) times the headroom and the lap's \
         reel-out ratio, since the optimizer measures the radius up the reel-out \
         while the run flies the curve at the anchor [m]"),
    "turn_radius_scale" => (round(opt_r_scale; digits = 3),
        "the two corrections together, as sent [-]"),
    "turn_radius_lap_reelout_m" => (tos.turn_radius_lap_reelout_m,
        "reel-out per lap ASSUMED for the startup request, the only one with no \
         reply to measure it off [m]"))
if !isnothing(coeffs)
    feasibility_block["margin_start"] = (round(feas_start.margin; digits = 2),
        "curvature margin at the starting length; the worst case only when one \
         path is flown throughout — see the docstring [-]")
    feasibility_block["margin_end"] = (round(feas_end.margin; digits = 2),
        "curvature margin at reelout_l_max [-]")
end
if !isnothing(feas_final)
    feasibility_block["c1_final"] = (round(c1_final; digits = 4),
        "turn-rate gain at depower_final [1/m]")
    feasibility_block["margin_final"] = (round(feas_final.margin; digits = 2),
        "curvature margin phase 5 flies with: depower_final's c1, at \
         reelout_l_max, on the STARTING path lifted by el_offset_final [-]")
    isnan(margin5_flown) ||
        (feasibility_block["margin_final_flown"] =
            (round(margin5_flown; digits = 2),
             "the same, for the last path the re-optimizer installed — the one \
              phase 5 actually inherited [-]"))
end
droop_mean = [droop_n[b] > 0 ? droop_flown[b] / droop_n[b] : NaN
              for b in 1:n_droop_bins]

summary["traj_opt"] = OrderedDict{String, Any}(
    "power" => power_block,
    "guess" => OrderedDict(
        "a_deg" => (tos.guess_a, "width of the guess lemniscate; azimuth spans ±a [deg]"),
        "b_deg" => (tos.guess_b, "height of the guess, peak to peak [deg]"),
        "el_center_deg" => (tos.guess_el_center, "centre elevation of the guess [deg]"),
        "points" => (tos.guess_points, "points the guess was sent with"),
        "depower_seed_m" => (round(depower_seed(tos, inflow.wind_speed); digits = 3),
            "power-tape length the solve started from, input_depower ramped with \
             the wind above input_depower_wind_ref; only a seed, the server \
             optimizes it [m]"),
        "depower_optimized" => let seen = Dict{String, Int}(), dp = OrderedDict{String, Any}()
            for e in opt_depower_log
                k = @sprintf("t_%05.1f_s", e.t)
                n = get(seen, k, 0) + 1
                seen[k] = n
                dp[n == 1 ? k : "$(k)_$n"] =
                    (round(e.l_dp; digits = 3),
                     @sprintf("optimizer's l_dp [m]; rel_depower equivalent %.3f \
                               (awetrim_depower_to_v3kite), against the flown \
                               depower_setpoint = %.3f",
                              e.u_p_equiv, fcs.depower_setpoint))
            end
            dp
        end),
    "path" => OrderedDict(
        "points" => (n_path_initial, "points of the path installed before the run"),
        "points_final" => (length(fec.az_path),
            "points of the path at the end; differs when reopt installed one"),
        "az_center_deg" => (round(az_c_path; digits = 1), "centre azimuth of the path [deg]"),
        "az_amplitude_deg" => (round(az_amp_path; digits = 1),
            "half-width of the STARTUP path; the criteria are scored against the \
             pattern commanded at each lap, whose mean is lift_budget's reference [deg]"),
        "el_center_deg" => (round(el_c_path; digits = 1), "centre elevation of the path [deg]"),
        "el_height_deg" => (round(el_height_path; digits = 1),
            "elevation span of the path, peak to peak [deg]"),
        "downloops" => (opt_downloops, "traversal direction the optimizer solved for")),
    "feasibility" => feasibility_block,
    "reopt" => OrderedDict(
        "enabled" => (tos.reopt_enabled, "re-optimization during the run"),
        "warm_start" => (tos.use_step,
            "/step alone, seeded from the previous optimum; false re-inits from \
             the parametric guess at every length"),
        "requests" => (reopt_n, "solves that completed, accepted or rejected"),
        "blocking" => (tos.reopt_blocking,
            "simulation held while a solve ran"),
        "blocked_s" => (round(reopt_blocked_s; digits = 1),
            "wall time the simulation was frozen waiting for replies [s]"),
        "installed" => (count(e -> e.status == "installed", reopt_events),
            "new paths actually flown"),
        "blend_retries" => (@isdefined(blend_retries_total) ? blend_retries_total : 0,
            "cold-restart attempts spent on a rejected reply — a folded blend, a \
             collapsed prediction, or a clearance/elevation shortfall re-asked at \
             a raised floor — across every install this run"),
        # Keyed by time, but two events CAN share one: a blocking solve is
        # requested and collected on the same step, so a seed skipped from the
        # failure cache carries the same `t` as the install that follows it. A
        # plain comprehension silently keeps the last of them — which hid the
        # cache's own skips the first time it ran — so collisions get a suffix.
        "events" => let seen = Dict{String, Int}(), ev = OrderedDict{String, Any}()
            for e in reopt_events
                k = @sprintf("t_%05.1f_s", e.t)
                n = get(seen, k, 0) + 1
                seen[k] = n
                ev[n == 1 ? k : "$(k)_$n"] =
                    (string(e.status, isempty(e.detail) ? "" : " — " * e.detail),
                     @sprintf("at L = %.0f m", e.l))
            end
            ev
        end),
    "el_bias" => OrderedDict(
        "gain" => (fcs.el_bias_gain,
            "per-lap learning gain for the elevation bias; 0 = off"),
        "final_deg" => (round(mean(el_bias); digits = 2),
            "mean elevation correction added to the last installed path [deg]"),
        "bins" => (n_el_bins,
            "el_bias_bins, azimuth bands the correction is learnt over; 1 = one \
             rigid shift"),
        "smooth" => (fcs.el_bias_smooth,
            "el_bias_smooth, how far each band is pulled towards its neighbours \
             after every lap [-]"),
        "profile_deg" => (n_el_bins == 1 ? "n/a" : round.(el_bias; digits = 2),
            "the learnt correction per band, crossing first [deg]"),
        "lift_deg" => (fcs.el_offset_final,
            "el_offset_final, the fixed lift added on top of the learnt bias [deg]"),
        "lift_lead_s" => (fcs.el_offset_lead,
            "el_offset_lead, how early the lift is allowed to latch; 0 = at the end [s]"),
        "wing_deg" => (fcs.el_offset_wing,
            "el_offset_wing, extra lift at the lobes, baked into every installed path [deg]"),
        "wing_mode" => (fcs.el_offset_wing_mode,
            "el_offset_wing_mode, the coordinate that lift is shaped over"),
        "wing_az_deg" => (fcs.el_offset_wing_az,
            "azimuth mode: azimuth beyond which that lift is full; it ramps over \
             el_offset_wing_blend below it [deg]"),
        "wing_depth" => (fcs.el_offset_wing_depth,
            "elevation mode: depth below the path's elevation centre where the lift \
             starts, in half-spans; full at its lowest point [-]"),
        "lift_t_s" => (isnan(lift_t) ? "never" : round(lift_t; digits = 1),
            "when the lift actually latched [s]"),
        "lift_remaining_m" => (isnan(lift_remaining) ? "n/a" :
                               round(lift_remaining; digits = 1),
            "reel-out left at that moment; > 0 means the lead fired, 0 means \
             phase 5 did [m]"),
        "shift_delivery" => OrderedDict(
            @sprintf("t_%05.1f_s", e.t) =>
                (e.status,
                 @sprintf("%+.2f° of shift (mean over the bands), curvature margin \
                           %.2f vs %.2f required",
                          e.delta, e.margin, tos.min_feasibility_margin))
            for e in el_shift_events),
        "laps" => OrderedDict(
            @sprintf("lap_%d", e.lap) =>
                (round(e.err_opt; digits = 2),
                 @sprintf("mean elevation error vs the OPTIMIZER's curve; sag under \
                           the flown path %.2f°, correction became %s",
                          e.err_el, prof_str(e.bias)))
            for e in el_bias_events),
        "lap_profiles" => (n_el_bins == 1 ? OrderedDict("n/a" =>
                ("one band", "el_bias_bins is 1, so traj_opt.el_bias.laps says it all")) :
            OrderedDict(
                @sprintf("lap_%d", e.lap) =>
                    (join((isnan(v) ? "  -  " : @sprintf("%+.2f", v) for v in e.err_bin),
                          " "),
                     @sprintf("error vs the optimizer's curve per band, crossing \
                               first; %s samples, correction now %s",
                              join(e.n, "/"), prof_str(e.bias)))
                for e in el_bias_events))),
    "droop_profile" => OrderedDict(
        vcat(
            [@sprintf("az_%02d_%02d_pct", 100 * (b - 1) ÷ n_droop_bins,
                      100 * b ÷ n_droop_bins) =>
                 (droop_n[b] == 0 ? "n/a" : round(droop_mean[b]; digits = 2),
                  droop_n[b] == 0 ? "no samples in this azimuth band" :
                  @sprintf("kite below the path's elevation centre [deg], over %d \
                            samples; the path itself sits %.2f half-spans down there \
                            and the kite %.2f° under it",
                           droop_n[b], droop_ref[b] / droop_n[b],
                           droop_sag[b] / droop_n[b]))
             for b in 1:n_droop_bins],
            ["centre_to_lobe_deg" =>
                 (all(isnan, droop_mean) || isnan(first(droop_mean)) ? "n/a" :
                  round(maximum(filter(!isnan, droop_mean)) - first(droop_mean);
                        digits = 2),
                  "how much deeper the kite flies in its worst azimuth band than in \
                   the crossing — what a SHAPED lift is aimed at, where el_bias and \
                   el_offset_final can only move the pattern rigidly [deg]")])),
    "lift_budget" => OrderedDict(
        vcat(
            ["delivered_deg" => (round(lift_mean; digits = 2),
                 "mean correction the path in the air actually carries — el_bias plus \
                  el_offset_final once it has been delivered [deg]"),
             "delivered_profile_deg" => (n_el_bins == 1 ? "n/a" :
                                         round.(el_applied; digits = 2),
                 "the same, per azimuth band, crossing first [deg]"),
             "commanded_az_amp_deg" => (round(az_amp_mean; digits = 2),
                 "mean half-width the run was COMMANDED to fly, against the startup \
                  path's in traj_opt.path [deg]"),
             "commanded_el_height_deg" => (round(el_h_mean; digits = 2),
                 "mean commanded pattern height [deg]"),
             "wing_deg" => (fcs.el_offset_wing,
                 "el_offset_wing, the fixed lobe lift baked into every installed path [deg]"),
             "el_min_run_deg" => (round(fig8m.min_elevation_all; digits = 1),
                 "lowest elevation over the whole run — set in phase 4, so a phase-5 \
                  lift does not move it [deg]"),
             "el_min_final_deg" => (isnan(el_min_final) ? "n/a" :
                                    round(el_min_final; digits = 2),
                 "lowest elevation in phase 5, which is what el_offset_final buys [deg]")],
            [string(m.name, "_deg") =>
                 (round(m.margin; digits = 2),
                  @sprintf("reach margin: flew %.2f° against the %.2f° required by \
                            min_span_frac = %.2f, %+.0f %%", m.flown, m.required,
                           fcs.min_span_frac, m.pct))
             for m in span_margins],
            ["tightest" => (replace(span_worst.name, "_" => " "),
                 @sprintf("the size criterion with the least room, %+.2f° (%+.0f %%) — \
                           what the NEXT lift has to spend", span_worst.margin,
                          span_worst.pct))])),
    "clearance" => OrderedDict(
        "path_min_m" => (round(path_min_h_start; digits = 1),
            "lowest point of the PRE-FLIGHT path at the starting length [m]"),
        "flown_min_m" => (round(minimum(first(lt) * sin(el)
                                        for (lt, el) in zip(sl.l_tether, sl.elevation));
                                digits = 1),
            "lowest point the kite actually reached over the run [m]"),
        "min_required_m" => (tos.min_height, "min_height of data/traj_opt.yaml [m]"),
        "elevation_min_asked_deg" => (
            let b = @isdefined(opt_box_now) && !isnothing(opt_box_now) ?
                    opt_box_now : opt_box
                isnothing(b) || isnothing(b.elevation_min) ? "unset" :
                    round(b.elevation_min; digits = 2)
            end,
            "elevation floor the LAST request carried, the higher of \
             asind(min_height/L) and min_elevation + candidate_elevation_margin at \
             the length it was made for; the optimizer constrains HEIGHT, and only \
             at the far end of the lap's reel-out [deg]"),
        "elevation_min_extra_deg" => (round(el_min_extra; digits = 2),
            "raise a rejected reply's shortfall added to that floor, carried \
             forward once measured; 0 means no reply was gated out low [deg]")),
    "inflow" => OrderedDict(
        "wind_speed_m_s" => (inflow.wind_speed, "wind speed at 6 m sent to the optimizer [m/s]"),
        "wind_direction_deg" => (inflow.wind_direction, "direction the wind comes from [deg]"),
        "profile_law" => (inflow.profile_law, "0=CONST, 1=EXP, 2=LOG, 3=EXPLOG, 4-6=CUSTOM_*")),
    "winch" => OrderedDict(
        "k_v" => (winch.k_v, "v_set = k_v * sqrt(force) sent to the optimizer [-]"),
        "f_min_N" => (winch.f_min, "minimum winch force sent [N]"),
        "f_max_N" => (winch.f_max, "maximum winch force sent [N]")))

# Speed of the SIMULATED time against the wall clock; > 1 is faster than realtime.
if t_sim > 0
    steps = round(Int, t_sim / s.dt)
    # Rates exclude the frozen time, as in the summary; the raw wall time is still
    # shown, and the gap between the two is `reopt_blocked_s`.
    t_run = max(t_wall - reopt_blocked_s, eps())
    # Everything the script spent waiting for the solver, wherever it fell: the
    # startup solve holds the script before the loop exists, the blocking
    # re-optimizations freeze the loop itself.
    t_opt = opt_startup_solve_s + reopt_blocked_s
    t_total = time() - t_script_start
    @printf("  Performance: %.1f s sim in %.1f s wall%s = %.2f x realtime \
             (%.1f ms/step over %d steps at dt = %.4f s, vsm_interval = %d)\n",
            t_sim, t_wall,
            reopt_blocked_s > 0 ? @sprintf(" (%.1f s held for re-optimization)",
                                           reopt_blocked_s) : "",
            t_sim / t_run, 1000 * t_run / steps,
            steps, s.dt, fcs.vsm_interval)
    @printf("               %.0f s from the first line of the script, %.0f s of it \
             waiting for the optimizer (%.0f s at startup).\n",
            t_total, t_opt, opt_startup_solve_s)
    summary["performance"] = OrderedDict(
        "sim_time_s" => (round(t_sim; digits = 1), "simulated time [s]"),
        "wall_time_s" => (round(t_wall; digits = 1), "wall-clock time [s]"),
        "total_wall_time_s" => (round(t_total; digits = 1),
            "the whole script: package loading, init, settling, the startup solve \
             and the run, up to this summary — the archive copy and any plots \
             follow it [s]"),
        "optimization_time_s" => (round(t_opt; digits = 1),
            "wall time waiting for the optimizer: the startup solve \
             ($(round(opt_startup_solve_s; digits = 1)) s) plus every blocking \
             re-optimization (traj_opt.reopt.blocked_s) [s]"),
        "realtime_factor" => (round(t_sim / max(t_wall - reopt_blocked_s, eps()); digits = 2),
            "sim_time / wall_time, excluding time frozen for re-optimization"),
        "ms_per_step" => (round(1000 * (t_wall - reopt_blocked_s) / steps; digits = 1),
            "wall time per step, excluding time frozen for re-optimization [ms]"),
        "steps" => (steps, "step count"),
        "dt_s" => (s.dt, "simulation timestep [s]"),
        "vsm_interval" => (fcs.vsm_interval, "VSM aerodynamic update interval [steps]"))
else
    @warn "No simulated time elapsed — no performance figure."
end

# A recap of the numbers scattered above, written last so it reads as the file's
# TL;DR without displacing `success_criteria` as the first key.
summary_block = OrderedDict{String, Any}(
    "date" => (Dates.format(run_time, "yyyy-mm-dd"), "wall-clock date the run finished"),
    "time" => (Dates.format(run_time, "HH:MM:SS"), "wall-clock time the run finished"),
    "wind_speed_m_s" => (inflow.wind_speed, "wind speed at 6 m sent to the optimizer [m/s]"))
if !isnothing(opt_power_meas)
    summary_block["mean_power_W"] = (round(Int, opt_power_meas), "mean reel-out power the run harvested [W]")
    summary_block["power_ratio"] = (round(opt_power_meas / opt_power_pred_eff; digits = 2),
        "measured / predicted mean reel-out power, against the weighted prediction")
end
if t_sim > 0
    summary_block["total_wall_time_s"] = (round(t_total; digits = 1), "the whole script, start to this summary [s]")
    summary_block["realtime_factor"] = (round(t_sim / max(t_wall - reopt_blocked_s, eps()); digits = 2),
        "sim_time / wall_time, excluding time frozen for re-optimization")
end
summary_block["optimization_requests"] = (reopt_n, "solves that completed, accepted or rejected")
summary_block["optimizations_installed"] = (count(e -> e.status == "installed", reopt_events),
    "new paths actually flown")
summary["summary"] = summary_block

open(joinpath(output_path, log_name * ".yaml"), "w") do io
    println(io, "# Run summary for output/", log_name,
            ".arrow, written by examples/simple_reelout.jl at the end of the run.")
    write_yaml_commented(io, 0, summary)
end

# ==================== ARCHIVE ==================== #

# Repeated here, as the last thing the run says: the results block above is a wall
# of printf and this is the comparison the script exists for. `@info`, not `printf`,
# so it lands on the same stream as the archive line it precedes.
isnothing(opt_power_meas) || @info power_summary

# One timestamped folder per run under output/archives/, so the exact config
# that produced a log survives even after the next run overwrites output/*.
# RUN_ARCHIVE = false skips it, read and cleared like SHOW_PLOTS: a sweep writes
# one folder per grid point otherwise, each with a copy of the 40 MB arrow log,
# and its own results table already records what distinguished the runs.
run_archive = @isdefined(RUN_ARCHIVE) ? RUN_ARCHIVE : true
RUN_ARCHIVE = true
if run_archive
    archive_dir = joinpath(output_path, "archives",
                           Dates.format(run_time, "yyyy-mm-dd_HHMMSS"))
    mkpath(archive_dir)
    input_yaml_files = [
        project,                                              # system project
        joinpath(dirname(project), project_set.sim_settings), # plant/solver settings
        joinpath(skc_data_path(), wc_settings(project)),      # winch gains
        joinpath(skc_data_path(), fc_settings(project)),      # flight-controller tuning
        joinpath(skc_data_path(), winch_kv_table_file(project)), # kv(v_wind) table
        joinpath(skc_data_path(), "gui.yaml"),                # project/sim_time/turbulence choice
        # Without this the archive cannot reproduce its own run: the guess decides
        # WHICH optimum the solve converges to, and reopt_*/min_feasibility_margin
        # decide what is re-anchored and what is flown.
        joinpath(skc_data_path(), traj_opt_settings_file(project)), # optimizer guess and knobs
    ]
    output_files = [
        joinpath(output_path, log_name * ".arrow"),
        joinpath(output_path, log_name * ".yaml"),
    ]
    for f in unique(vcat(input_yaml_files, output_files))
        isfile(f) && cp(f, joinpath(archive_dir, basename(f)); force = true)
    end
    @info "Archived run inputs and outputs to $archive_dir"
else
    @info "Archiving suppressed by RUN_ARCHIVE = false; it is back to true for the next run."
end

plots_failed = nothing
if show_plots
    # The flown curve and this run's log name, so the plots draw the optimized
    # path and load the _opt log instead of the lemniscate run's.
    REF_PATH = (fec.az_path, fec.el_path)
    OPT_PATHS_RAW = opt_paths_raw
    LOG_NAME = log_name
    # The plots are cosmetic and the run is already scored, logged and archived by
    # here; a GLMakie failure (needs the main thread, so an eval off it throws)
    # must not cost the marker that tells a watcher the run is over.
    try
        include(joinpath(@__DIR__, "simple_reelout_plots.jl"))
    catch exc
        global plots_failed = exc
        @error "Plots failed; the run itself completed and is archived." exception =
            (exc, catch_backtrace())
    end
else
    @info "Plots suppressed by SHOW_PLOTS = false; it is back to true for the next run."
end

# Defined in simple_opt_reelout.jl before anything that can throw, and called
# from its `catch` too — see the docstring there.
write_run_done(isnothing(plots_failed) ? "ok" : "ok (plots failed)")

nothing
