# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The setup of one reel-out run of examples/simple_opt_reelout.jl, and its finished-run marker.

"""
    setup_run(inputs; init_model) -> NamedTuple

Everything the run READS and never rebinds, built in order: the settings and their overrides, the
plant `s` (built by the caller's `init_model(project, project_set, fcs, wpc, sim_time; turbulence,
set_overrides)`, since the package does not depend on the model), the winch and its controllers, the optimizer's conditions and session, the turn-rate laws
(`c1_at_depower`, `c1_ctrl_at`) and the lobe lift. The script keeps the result in the one global
`setup` and hands it to every function of the run; the loop destructures it (see `run_loop!`).
Mutable members (`fcs`, `wc`, `s`, the `opt_*_log` vectors, ...) are still changed in place. The
startup solve and `startup_feasibility` add their results with `merge`.
"""
function setup_run(inputs; init_model)
    (; show_plots, steer_disturbance, xtrack_offset, xtrack_phase, hold_compliance, steer_gain_factor,
       steer_gain_feedback_only, extra_steer_delay, hook_settle, replay_paths) = inputs
    project_name = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
    @assert project_name in ("system_reelout_cabauw.yaml", "system_reelout_maasvlakte.yaml") "simple_opt_reelout.jl \
        supports only system_reelout_cabauw.yaml and system_reelout_maasvlakte.yaml, got $project_name"
    sim_time = selected_sim_time() # seconds, or `nothing` for the project's own default
    turbulence = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
    wind_speed = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
    @info "simple_opt_reelout.jl: project = $project_name, sim_time = $(isnothing(sim_time) ? "default" : "$sim_time s"), \
           turbulence = $turbulence, wind_speed = $(isnothing(wind_speed) ? "default" : "$wind_speed m/s")."
    project = project_file(project_name)
    fcs = FC_Settings(fc_settings(project))
    # The turn-rate table the PROJECT names (its `turn_rate_coeffs`), not the one `__init__` loaded
    # through system_fig8_200m.yaml. Two consumers: the CONTROLLER (gain schedule, curvature
    # feed-forward) reads `ctrl_tr_table`; the PATH side (turn-radius requests to the planner, the
    # startup gates, the feasibility checks) reads the session's table. Both are the project's unless
    # the input `path_tr_project` names another system project, whose table
    # then sizes the path: an A/B of the controller's table with the planned path held fixed.
    # Session-wide: a later script that does not reload keeps the path side's table.
    ctrl_tr_table = _load_turn_rate_table(project)
    reload_turn_rate_table!(isnothing(inputs.path_tr_project) ? project : project_file(inputs.path_tr_project))
    isnothing(inputs.path_tr_project) ||
        @info "Turn-rate tables: controller $(turn_rate_coeffs_file(project)), path side \
               $(turn_rate_coeffs_file(project_file(inputs.path_tr_project))) (path_tr_project)."
    # The optimizer's own settings: server, initial guess, solver knobs, margin.
    tos = TrajOptSettings(traj_opt_settings_file(project))

    # Sweep overrides (the input `fcs_overrides`), and the same for the optimizer's settings
    # (`tos_overrides`), e.g. `reopt_enabled = false` for a test run.
    apply_overrides!(fcs, inputs.fcs_overrides, "fcs_overrides", "FC_Settings", "fcs")
    apply_overrides!(tos, inputs.tos_overrides, "tos_overrides", "TrajOptSettings", "tos")
    # Test input: a steering disturbance `t -> Δu` added after the controller (`steer_disturbance`);
    # `stability_opt_reelout.jl`'s model is validated against the loop's response to it.
    isnothing(steer_disturbance) || @info "Steering disturbance in force (test input)."
    # Test input: a cross-track offset `τ -> δ` [deg], τ the time since phase `xtrack_phase` (default 5)
    # began (`xtrack_offset`, `xtrack_phase`). The attractor is moved δ along the path's right-hand normal, so the pursuit
    # aims at the parallel curve δ to the right: a reference step for the guided loop alone. The run
    # keeps (in `RunState`) `xt_t`, `xt_delta`, `xt_d`, the signed cross-track error to the UNSHIFTED path, and `xt_q`,
    # the index of its closest point Q: a reference run's d as a function of Q removes the lap forcing.
    # Test input: a COMPLIANT hold in phase 5, `(gain, τF, τpos)` (`hold_compliance`). Instead
    # of freezing l_set, the length setpoint moves at gain·kv/(2√F̄)·(F − F̄) − (l_set − l_hold)/τpos: the
    # reel-out law's force slope around the force F̄ low-passed over τF [s], zero mean speed, and a slow
    # pull back to the length the hold began at. `nothing` holds the length rigidly, as flown.
    isnothing(hold_compliance) || @info "Compliant hold in phase 5 (test input): $hold_compliance"
    # V1 model-validation test inputs (oldplans/Plan_model_validation.md, V1, point C):
    # `steer_gain_factor` multiplies rel_steering, and
    # `extra_steer_delay` adds a FIFO delay to it, in samples. Both act only from
    # `hook_settle` seconds after phase 4 is first reached, so entry, phase 3 and the
    # early part of phase 4 fly identically in every run of a sweep.
    # `steer_gain_feedback_only` true: scale only the feedback part,
    # rel_steering - u_ff. The feed-forward lies outside the loop, so this scales the loop gain
    # alone; it differs from the default only with ff_gain > 0.
    (steer_gain_factor == 1.0 && extra_steer_delay == 0) ||
        @info @sprintf("V1 stability hook in force: gain factor %.3g%s, extra delay %d \
                        samples, active %.1f s after phase 4 begins.",
                       steer_gain_factor, steer_gain_feedback_only ? " (feedback only)" : "",
                       extra_steer_delay, hook_settle)
    isnothing(xtrack_offset) || @info "Cross-track offset in force (test input)."

    project_set = Settings(project)
    default_v_wind = project_set.v_wind
    apply_windspeed_override!(project_set, wind_speed)
    l_tether = project_set.l_tether

    # Whether `min_power_frac`/`min_power_frac_prev` are bypassed for a candidate predicting `pred`
    # watts: only below `tos.power_gate_wind_min` mean wind AND only for a NEGATIVE prediction, where
    # the number reports the optimizer's winch model leaving its own domain rather than a bad path.
    # Defined after the wind override, so it gates on the speed actually flown.
    power_gate_off(pred) = pred < 0 && project_set.v_wind < tos.power_gate_wind_min

    # Simulated time to ask `init` for: the reel-out budget under a wind-speed override, see `sim_budget`.
    effective_sim_time = sim_budget(project, project_set, fcs, sim_time, wind_speed, default_v_wind)

    # Arrow log files named after the project's `log_file`; `output_path` redirects them for parallel sweep runs.
    # The default is the package's output/, next to src/ and examples/.
    output_path = something(inputs.output_path, normpath(joinpath(@__DIR__, "..", "output")))
    mkpath(output_path)

    # Finished-run marker for outside watchers: removed here, written last, so its presence means "this run is over".
    run_done_file = joinpath(output_path, "last_run_done.txt")
    rm(run_done_file; force = true)
    # `_opt`: never overwrite the lemniscate run's log and summary, the two are each other's baseline.
    log_name = basename(project_set.log_file) * "_opt"

    # ======================== INIT =========================== #

    # ONE WCSettings for BOTH winch loops: the POSITION-mode torque gains (`wpc`) and the speed-controller tuning (`rc`).
    (; wc, wpc, dt0) = build_winch(project, project_set, fcs)
    # Winch overrides for a test run, e.g. `v_sat` or `kv` (`inputs.wc_overrides`): applied
    # just before the simulation loop, so the optimizer plans the path with the unchanged winch.
    rcs = wc                                 # same object, two controllers read it

    # Plant overrides for a diagnostic run (`inputs.set_overrides`) are applied inside.
    s = init_model(project, project_set, fcs, wpc, effective_sim_time; turbulence,
                   set_overrides = inputs.set_overrides)

    # The controllers, built after `init` so the soft-start ramp begins when reel-out starts; see `build_controllers`.
    (; rc, f_high_nominal, guard_lfc, l_set, fec) = build_controllers(fcs, rcs, s)

    # ================= OPTIMIZED REFERENCE PATH ================== #

    # The conditions of THIS run and the winches the optimizer is sent, see `optimizer_conditions`.
    (; inflow, cap_wind, winch, winch_first_lap, winch_reopt) =
        optimizer_conditions(tos, fcs, project_set, rcs, f_high_nominal)
    # Every reply's optimized gain, so the summary reports what was flown, not only what was sent.
    opt_kv_log = NamedTuple{(:t, :l, :k_v, :at_bound), Tuple{Float64, Float64, Float64, Bool}}[]

    # The seed of the startup solve and the connection to the optimizer, see `optimizer_session`; anchored to the
    # STARTING length, re-optimizing during the run is `reoptimize!`. `solve_startup_path!` replaces the seed.
    (; el_center_seed_base, el_center_seed, opt_chain) =
        optimizer_session(tos, inflow, replay_paths, log_name)
    # Constraints the solve must respect; the turn radius carries the anchor ratio `L/r` and the gate's headroom.
    (; opt_r_scale, opt_r_min, opt_r_on, opt_r_sent, opt_box) =
        request_constraints(tos, fcs, inflow, cap_wind, opt_length(tos, l_set))

    # One row per depower value the optimizer reports back (startup, each ACCEPTED reopt), for summary and plot.
    opt_depower_log = NamedTuple[]

    # The LOBE lift, applied to every installed path on the reply AS IT ARRIVED, before `el_offset_final`.
    wing_lift(az, el) = lobe_lift(az, el; lift = fcs.el_offset_wing,
                                  mode = fcs.el_offset_wing_mode,
                                  az_full = fcs.el_offset_wing_az,
                                  az_blend = fcs.el_offset_wing_blend)

    # The turn-rate gain at a depower, NaN off the table; memoized because a blend asks every step.
    c1_memo = Dict{Float64, Float64}()
    c1_at_depower(depower) = get!(c1_memo, Float64(depower)) do
        try
            turn_rate_coeffs(fcs.body_damping, depower).c1
        catch exc
            exc isa ArgumentError || rethrow()
            # Once per run: a blend ramping off the grid would repeat it every step.
            any(isnan, values(c1_memo)) ||
                @warn @sprintf("No turn-rate coefficients at depower %.3f (body_damping \
                                %s): gates and gain fall back to the startup law there.",
                               depower, fcs.body_damping)
            NaN
        end
    end
    # The controller's turn-rate gain at a depower, from `ctrl_tr_table`; NaN off it, memoized like c1_at_depower.
    c1_ctrl_memo = Dict{Float64, Float64}()
    c1_ctrl_at(depower) = get!(c1_ctrl_memo, Float64(depower)) do
        try
            turn_rate_coeffs(fcs.body_damping, depower; table = ctrl_tr_table).c1
        catch exc
            exc isa ArgumentError || rethrow()
            NaN
        end
    end
    # The turn authority the loop was TUNED at; the sim loop rescales heading_p by c1_setpoint/c1(u_d) in every phase.
    c1_setpoint = c1_ctrl_at(fcs.depower_setpoint)
    # Phase 4 must fly with the curvature feed-forward, which silently drops out on either of these.
    fcs.ff_gain > 0 || @warn "simple_opt_reelout.jl needs the curvature feed-forward in phase 4, \
        but ff_gain = $(fcs.ff_gain)"
    @assert isfinite(c1_setpoint) && c1_setpoint > 0 "the curvature feed-forward needs the turn-rate \
        coefficient c1 at depower_setpoint = $(fcs.depower_setpoint), got $c1_setpoint"
    c1_depower_max = try
        last(turn_rate_depower_range(fcs.body_damping; table = ctrl_tr_table))
    catch exc
        exc isa ArgumentError || rethrow()
        NaN
    end
    # The depower a reply is FLOWN at, which is what every gate and request must read c1 at.
    pattern_depower(reply) =
        tos.fly_opt_depower && !isnothing(reply.depower) ?
            awetrim_depower_to_v3kite(reply.depower.value) : fcs.depower_setpoint
    # The elevation floor of every candidate path, the startup gates' and `check_startup_path`'s.
    el_floor = fcs.min_elevation + tos.candidate_elevation_margin

    return (; inputs, show_plots, steer_disturbance, xtrack_offset, xtrack_phase, hold_compliance,
            steer_gain_factor, steer_gain_feedback_only, extra_steer_delay, hook_settle, replay_paths,
            project_name, turbulence, project, fcs, tos, project_set, l_tether, effective_sim_time,
            output_path, run_done_file, log_name, wc, wpc, dt0, rcs, s, rc, f_high_nominal, guard_lfc,
            l_set, fec, inflow, cap_wind, winch, winch_first_lap, winch_reopt, el_center_seed_base,
            el_center_seed, opt_chain, opt_r_scale, opt_r_min, opt_r_on, opt_r_sent, opt_box,
            opt_kv_log, opt_depower_log, power_gate_off, wing_lift, c1_at_depower, c1_ctrl_at,
            c1_setpoint, c1_depower_max, pattern_depower, el_floor)
end

"""
    write_run_done(setup, st, status; err = nothing)

Write the finished-run marker. Called from the script's `catch` as well as from the
normal tail of `reelout_results`, so
the marker's absence means "still running" and never "it died" — a watcher that
only ever sees the success path waits forever on a run that crashed (measured: a
GLMakie main-thread error in the plots, long after the simulation had finished
and the log was safely written).

`status` is `ok`, `ok (plots failed)` or `FAILED`; on failure the exception's
first line follows on an `error:` line. The other fields come from `st`, where
`reelout_results` keeps them as soon as they are known: a crash early enough leaves
them at their defaults, and the marker still has to be writable.
"""
function write_run_done(setup, st, status::AbstractString; err = nothing)
    open(setup.run_done_file, "w") do io
        println(io, format(now(), "yyyy-mm-dd HH:MM:SS"))
        println(io, "status: ", status)
        isnothing(err) || println(io, "error: ", first(split(sprint(showerror, err), "\n")))
        println(io, "archive: ", st.archive_dir)
        println(io, "log: ", joinpath(setup.output_path, setup.log_name * ".yaml"))
        fig8m = st.fig8m
        println(io, "criteria: ", isnothing(fig8m) ? "n/a" :
                                  isempty(fig8m.criteria_failed) ?
                                  "all $(fig8m.criteria) passed" :
                                  "FAILED: " * join(fig8m.criteria_failed, ", "))
        power = st.opt_power_meas
        println(io, "power: ", isnothing(power) ? "n/a" :
                               @sprintf("%.0f W measured", power))
    end
end
