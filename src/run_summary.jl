# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The sections the two reel-out scripts share in their run summary YAML
# (examples/simple_reelout.jl and examples/simple_opt_reelout.jl): which code and
# conditions flew, the verdict and its figure-of-eight metrics, the reel-out window
# and the speed of the run. Each builds an `OrderedDict` of `(value, comment)`
# leaves for [`write_yaml_commented`](@ref), printing the lines the console shows.

"""
    package_git_state() -> (; hash, status)

The commit this package's code is at (`git rev-parse --short HEAD`) and whether
its working tree is `"clean"` or `"dirty"`; both `"unknown"` without git or
outside a checkout. The package repo, not the caller's: a run summary reports the
controller code that flew.
"""
function package_git_state()
    pkg_dir = pkgdir(@__MODULE__)
    try
        hash = strip(read(`git -C $pkg_dir rev-parse --short HEAD`, String))
        dirty = !isempty(strip(read(`git -C $pkg_dir status --porcelain`, String)))
        return (; hash, status = dirty ? "dirty" : "clean")
    catch
        return (; hash = "unknown", status = "unknown")
    end
end

"""
    success_verdict(fig8m) -> (verdict, comment)

The pass/fail verdict of [`print_fig8_metrics`](@ref)'s result as the console
logs it, `"all N passed"` or `"FAILED: "` and the criteria that broke, as a
summary leaf. `fig8m === nothing` (no settled samples) is `"not scored"`.
"""
success_verdict(fig8m) = (fig8m === nothing ? "not scored — no settled samples" :
    isempty(fig8m.criteria_failed) ? "all $(fig8m.criteria) passed" :
    "FAILED: " * join(fig8m.criteria_failed, ", "),
    "pass/fail verdict vs V3Kite's success criteria")

"""
    simulation_block(script, project, turbulence, wind_speed, run_time) -> OrderedDict

The summary's `simulation` section: the script and system project flown, the
turbulence level and mean wind speed passed to `init`, when (`run_time`, a
`DateTime`) and where the run finished, and [`package_git_state`](@ref).
"""
function simulation_block(script, project, turbulence, wind_speed, run_time)
    git = package_git_state()
    OrderedDict{String, Any}(
        "script" => (script, "script that produced this run"),
        "project" => (project, "system project flown"),
        "rel_turbulence" => (turbulence, "turbulence level in [0, 1] passed to init"),
        "wind_speed" => (wind_speed, "mean wind speed in m/s passed to init"),
        "time" => (format(run_time, "HH:MM:SS"), "wall-clock time the run finished"),
        "date" => (format(run_time, "yyyy-mm-dd"), "wall-clock date the run finished"),
        "hostname" => (gethostname(), "machine the run executed on"),
        "git_hash" => (git.hash, "SimpleKiteControllers.jl commit hash"),
        "git_status" => (git.status, "SimpleKiteControllers.jl working tree: clean or dirty"))
end

"""
    fig8_metrics_block(fig8m, laps_flown; cross_track_ref = nothing,
                       nested_verdict = false) -> OrderedDict

The summary's `fig8_metrics` section: the numbers the verdict of
[`print_fig8_metrics`](@ref) was computed from, and the lap times of
[`lap_durations`](@ref) (`laps_flown`). `cross_track_ref` names what the
cross-track error is measured against, for the comments. `nested_verdict = true`
repeats [`success_verdict`](@ref) as the section's last key, where
[`run_metrics`](@ref) reads a sweep's runs.
"""
function fig8_metrics_block(fig8m, laps_flown; cross_track_ref = nothing,
                            nested_verdict = false)
    vs = isnothing(cross_track_ref) ? "" : " vs $cross_track_ref"
    block = OrderedDict{String, Any}(
        "settled_from" => (round(fig8m.stats_start; digits = 1), "sim time the scoring window begins [s]"),
        "settle_time" => (round(fig8m.settle_time_used; digits = 1), "time after t_start to converge [s]"),
        "laps" => (fig8m.laps, "figure-eight laps completed"),
        "lap_time" => isempty(laps_flown.dt) ?
            ("none", "no full figure of eight was completed") :
            OrderedDict(
                "fastest" => (round(minimum(laps_flown.dt); digits = 1),
                    @sprintf("shortest time for one full figure of eight — lap %d, \
                              starting at t = %.1f s [s]",
                             argmin(laps_flown.dt), laps_flown.t_start[argmin(laps_flown.dt)])),
                "mean" => (round(mean(laps_flown.dt); digits = 1),
                    "mean time per full figure of eight, over $(length(laps_flown.dt)) laps [s]"),
                "slowest" => (round(maximum(laps_flown.dt); digits = 1),
                    @sprintf("longest time for one full figure of eight — lap %d [s]",
                             argmax(laps_flown.dt)))),
        "cross_track_deg" => OrderedDict(
            "rms" => (round(fig8m.rms_d; digits = 2), "RMS cross-track error$vs [deg]"),
            "mean" => (round(fig8m.mean_d; digits = 2), "mean cross-track error$vs [deg]"),
            "max" => (round(fig8m.max_d; digits = 2), "max cross-track error$vs [deg]")),
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
    nested_verdict &&
        (block["success_criteria"] = (first(success_verdict(fig8m)), "same verdict as the top-level key"))
    return block
end

"""
    reelout_block(sl, fcs, l_tether; stop_reason, laps_reeled, window_means = false)
        -> (; block, rp, p4)

The summary's `reelout` section of the log `sl`, printed as it is built: the
apparent wind over phase 4 (against `fcs.v_app_ref`), the tether's reel-out from
`l_tether` and why it stopped (`stop_reason`, `""` when reel-out never stopped;
`laps_reeled`, the laps completed by then), and the force, power, winch states and
ringing over the reeling window. `window_means = true` adds the window's mean and
peak force and power, which [`run_metrics`](@ref) reads.

Also returns `rp` ([`reelout_power`](@ref), `nothing` when the tether never reeled
out) and `p4`, the phase-4 power, force, reel-out speed and depower as
`(; power, force, v_ro, depower_av)`, each of the first three `(; av, min, max)`,
the minima without the last 2 s of phase 4; `nothing` when phase 4 was never reached.
"""
function reelout_block(sl, fcs::FC_Settings, l_tether; stop_reason::AbstractString,
                       laps_reeled, window_means::Bool = false)
    reelout_summary = OrderedDict{String, Any}()
    # On the LOGGED PHASE, not a time window; the mean is what v_app_ref should be.
    fig8 = findall(x -> Int(x) == 4, sl.sys_state)
    p4 = nothing
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
            "duration" => (round(duration; digits = 1), "phase-4 window length [s]"),
            "mean_m_s" => (round(mean(va); digits = 2), "mean apparent wind speed [m/s]"),
            "min_m_s" => (round(minimum(va); digits = 2), "min apparent wind speed [m/s]"),
            "max_m_s" => (round(maximum(va); digits = 2), "max apparent wind speed [m/s]"),
            "v_app_ref_m_s" => (fcs.v_app_ref, "reference apparent wind speed [m/s]"),
            "deviation_pct" => (round(100 * (mean(va) / fcs.v_app_ref - 1); digits = 1),
                "mean v_app deviation from v_app_ref [%]"))

        # Same phase-4 window, for a recap's power/force/reel-out-speed/depower —
        # distinct from `reelout.force`/`reelout.power` below, which are scored over
        # the REELING window (length setpoint growing), not phase 4.
        f4 = Float64.(getindex.(sl.winch_force, 1))[fig8]
        vro4 = Float64.(getindex.(sl.v_reelout, 1))[fig8]
        pw4 = f4 .* vro4
        dp4 = Float64.(sl.depower[fig8])
        # The power and speed minima skip the last 2 s of phase 4: the soft-stop
        # ramp winds the speed (and with it the power) down before the length stop
        # flips to phase 5, and that ramp is a setpoint move, not a dip. Falls back
        # to the whole window if phase 4 is shorter.
        t4 = Float64.(sl.time[fig8])
        i_min = findall(<=(t4[end] - 2.0), t4)
        isempty(i_min) && (i_min = eachindex(t4))
        p4 = (power = (av = mean(pw4), min = minimum(pw4[i_min]), max = maximum(pw4)),
              force = (av = mean(f4), min = minimum(f4[i_min]), max = maximum(f4)),
              v_ro = (av = mean(vro4), min = minimum(vro4[i_min]), max = maximum(vro4)),
              depower_av = mean(dp4))
        @printf("  Phase 4: power av %.0f W (min %.0f, max %.0f); force av %.0f N \
                 (min %.0f, max %.0f); v_reelout av %.2f m/s (min %.2f, max %.2f); \
                 depower av %.3f.\n",
                p4.power.av, p4.power.min, p4.power.max,
                p4.force.av, p4.force.min, p4.force.max,
                p4.v_ro.av, p4.v_ro.min, p4.v_ro.max, p4.depower_av)
    end
    # Unconditional: the winch is gated on phase 3, which phase 4 may never follow.
    reelout_stop_reason = isempty(stop_reason) ? "none" : stop_reason
    @printf("  Tether: %.1f m -> %.1f m (target %.1f m, stopped by: %s).\n",
            l_tether, sl.var_10[end], fcs.reelout_l_max, reelout_stop_reason)
    reelout_summary["tether"] = OrderedDict(
        "start_m" => (l_tether, "tether length at run start [m]"),
        "end_m" => (round(Float64(sl.var_10[end]); digits = 1), "tether length at run end [m]"),
        "target_m" => (fcs.reelout_l_max, "reelout_l_max target [m]"),
        "stop_reason" => (reelout_stop_reason, "criterion that ended reel-out: length, laps, or none"),
        "laps_reeled" => (round(laps_reeled; digits = 2),
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
        force = OrderedDict{String, Any}()
        power = OrderedDict{String, Any}()
        if window_means
            force["mean_N"] = (round(Int, rp.mean_force), "mean tether force over the reeling window [N]")
            force["peak_N"] = (round(Int, rp.peak_force), "peak tether force over the reeling window [N]")
            power["mean_W"] = (round(Int, rp.mean_power), "mean reel-out power over the reeling window [W]")
            power["peak_W"] = (round(Int, rp.peak_power), "peak reel-out power over the reeling window [W]")
        end
        force["cf_force_ro"] = (round(rp.cf_force_ro; digits = 2),
            "crest factor: peak / mean tether force over the reeling window")
        power["cf_power_ro"] = (round(rp.cf_power_ro; digits = 2),
            "crest factor: peak / mean reel-out power over the reeling window")
        power["duration"] = (round(rp.duration; digits = 1), "reeling window length [s]")
        power["n_samples"] = (rp.n, "sample count in the reeling window")
        power["energy_kJ"] = (round(rp.energy / 1000; digits = 1), "energy over the reeling window [kJ]")
        power["energy_run_kJ"] = (round(rp.energy_run / 1000; digits = 1), "energy over the whole run [kJ]")
        reelout_summary["force"] = force
        reelout_summary["power"] = power
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
            "period" => (round(rr.period_s; digits = 2), "mean peak-to-peak ring period [s]"),
            "zeta" => (round(rr.zeta; digits = 3), "damping ratio from the peak log decrement"),
            "overshoot_m_s" => (round(rr.overshoot_m_s; digits = 2), "first ring peak's amplitude above the local trend [m/s]"),
            "duration" => (round(rr.duration_s; digits = 1), "time until the ring decays below settle_frac of overshoot_m_s [s]"),
            "peak_v_reelout_m_s" => (round(rr.peak_v_reelout_m_s; digits = 2), "raw v_reelout max within ring_span [m/s]"),
            "steady_v_reelout_m_s" => (round(rr.steady_v_reelout_m_s; digits = 2), "mean v_reelout after ring_span [m/s]"))
    end
    return (; block = reelout_summary, rp, p4)
end

"""
    performance_block(t_sim, t_wall, dt, vsm_interval; blocked_s = nothing,
                      extra = Pair{String, Any}[]) -> OrderedDict | nothing

Speed of the SIMULATED time `t_sim` [s] against the wall clock `t_wall` [s] of the
simulation loop (> 1 is faster than realtime), printed and as the summary's
`performance` section; `nothing`, with a warning, when no simulated time elapsed.
`dt` is the timestep, `vsm_interval` the VSM update interval in steps. With
`blocked_s`, the wall time the loop was frozen waiting for an optimizer, the rates
exclude it and the printed line says how much it was. `extra` are further entries,
placed after `wall_time`.
"""
function performance_block(t_sim, t_wall, dt, vsm_interval; blocked_s = nothing,
                           extra = Pair{String, Any}[])
    if t_sim <= 0
        @warn "No simulated time elapsed — no performance figure."
        return nothing
    end
    steps = round(Int, t_sim / dt)
    blocked = something(blocked_s, 0.0)
    # Rates exclude the frozen time; the raw wall time is still shown.
    t_run = max(t_wall - blocked, eps())
    excluding = isnothing(blocked_s) ? "" : ", excluding time frozen for re-optimization"
    @printf("  Performance: %.1f s sim in %.1f s wall%s = %.2f x realtime \
             (%.1f ms/step over %d steps at dt = %.4f s, vsm_interval = %d)\n",
            t_sim, t_wall,
            blocked > 0 ? @sprintf(" (%.1f s held for re-optimization)", blocked) : "",
            t_sim / t_run, 1000 * t_run / steps, steps, dt, vsm_interval)
    block = OrderedDict{String, Any}(
        "sim_time" => (round(t_sim; digits = 1), "simulated time [s]"),
        "wall_time" => (round(t_wall; digits = 1), "wall-clock time [s]"))
    for (k, v) in extra
        block[k] = v
    end
    block["realtime_factor"] = (round(t_sim / t_run; digits = 2), "sim_time / wall_time$excluding")
    block["ms_per_step"] = (round(1000 * (t_wall - blocked) / steps; digits = 1),
                            "wall time per step$excluding [ms]")
    block["steps"] = (steps, "step count")
    block["dt"] = (dt, "simulation timestep [s]")
    block["vsm_interval"] = (vsm_interval, "VSM aerodynamic update interval [steps]")
    return block
end

"""
    opt_cycle_max(reopt_cycles, startup_solve_s) -> (; s, comment)

The longest a new figure of eight took to compute, retries included: the startup
solve of `startup_solve_s` [s] (which already contains its own retry seed) against
every re-optimization cycle in `reopt_cycles`, each `(; t, l, status, wall_s)` from
its first request to the verdict. `comment` says which one it was.
"""
function opt_cycle_max(reopt_cycles, startup_solve_s)
    s, which = if isempty(reopt_cycles)
        startup_solve_s, "the startup solve, the only solve of the run"
    else
        c = reopt_cycles[argmax(getfield.(reopt_cycles, :wall_s))]
        c.wall_s > startup_solve_s ?
            (c.wall_s, @sprintf("the re-optimization at t = %.1f s (%s, L = %.0f m); \
                                 the startup solve took %.1f s",
                                c.t, c.status, c.l, startup_solve_s)) :
            (startup_solve_s,
             @sprintf("the startup solve; the slowest re-optimization took %.1f s", c.wall_s))
    end
    return (; s, comment = "longest wall time to compute a new figure of eight, retries \
                            included: $which [s]")
end

"""
    run_input_files(project, project_set) -> Vector{String}

The input files every reel-out run is configured by: the system project, the
plant/solver settings it names, the winch gains, the flight-controller tuning and
`gui.yaml` (the `project`/`sim_time`/`turbulence` choice). A caller with more inputs
appends them before handing the list to [`archive_run_files`](@ref).
"""
function run_input_files(project, project_set)
    return [
        project,                                                  # system project
        joinpath(dirname(project), project_set.sim_settings),     # plant/solver settings
        joinpath(skc_data_path(), KiteUtils.wc_settings(project)), # winch gains
        joinpath(skc_data_path(), fc_settings(project)),          # flight-controller tuning
        joinpath(skc_data_path(), "gui.yaml"),                    # project/sim_time/turbulence choice
    ]
end

"""
    archive_run_files(output_path, run_time, input_files, output_files) -> archive_dir

Copy a run's `input_files` and `output_files` into one timestamped folder,
`<output_path>/archives/yyyy-mm-dd_HHMMSS` of `run_time`, so the exact config that
produced a log survives even after the next run overwrites `output/*`. The inputs
are copied back to `output_path` too, for the plotting script to find them: they
get overwritten on the next run, but that is the point — each run's plots use the
settings that run actually flew. Files that do not exist are skipped.
"""
function archive_run_files(output_path, run_time, input_files, output_files)
    archive_dir = joinpath(output_path, "archives", format(run_time, "yyyy-mm-dd_HHMMSS"))
    mkpath(archive_dir)
    for f in unique(vcat(input_files, output_files))
        isfile(f) && cp(f, joinpath(archive_dir, basename(f)); force = true)
    end
    for f in input_files
        isfile(f) && cp(f, joinpath(output_path, basename(f)); force = true)
    end
    @info "Archived run inputs and outputs to $archive_dir"
    return archive_dir
end
