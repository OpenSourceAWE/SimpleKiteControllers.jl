# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The setup of one reel-out run of examples/simple_opt_reelout.jl, and its finished-run marker.

"""
    RunSetup

Everything one reel-out run of `examples/simple_opt_reelout.jl` READS, built once by
[`setup_run`](@ref): the settings, the plant `s`, the winch and its controllers, the optimizer's
conditions and session, and the laws the gates and the loop read. Every function of the run takes it
as `setup` and reads `setup.<field>` or destructures it. The script never rebinds it; mutable members
(`fcs`, `wc`, `s`, `fec`, the `opt_*_log` vectors, ...) are changed in place.

The last block of fields is filled later, by the startup solve ([`solve_startup_path!`](@ref)) and
the startup gates ([`startup_feasibility`](@ref)), through [`merge_into!`](@ref); until then they
hold their defaults. The type parameters are the plant's type `S`, which the package does not know,
and the two turn-rate laws, read every step, so that the loop compiles against their concrete types.

The block functions of the run read their fields by name only, so a `NamedTuple` with the fields a
function reads serves as well (the unit tests build them that way).

# Fields

$(TYPEDFIELDS)
"""
Base.@kwdef mutable struct RunSetup{S, CD, CC}
    # ---- the inputs of the run (`run_input_defaults`, `script_inputs`) ----
    "The caller's inputs, merged over the defaults"
    inputs::NamedTuple
    "Whether the run shows its plots at the end"
    show_plots::Bool
    "Test input: t -> Δu added to the steering"
    steer_disturbance::Union{Nothing, Function}
    "Test input: τ -> δ [deg], the attractor moved along the path normal"
    xtrack_offset::Union{Nothing, Function}
    "The phase `xtrack_offset` starts in"
    xtrack_phase::Int
    "Test input: (; gain, τF, τpos) of a compliant hold in phase 5"
    hold_compliance::Union{Nothing, NamedTuple}
    "V1 hook: factor on the steering"
    steer_gain_factor::Float64
    "The factor on the feedback part only"
    steer_gain_feedback_only::Bool
    "V1 hook: extra steering delay [samples]"
    extra_steer_delay::Int
    "[s] after phase 4 began, when the V1 hooks start"
    hook_settle::Float64
    "Scenario folder whose optimizer answers are replayed"
    replay_paths::Union{Nothing, String}
    # ---- project and settings ----
    "System_reelout_*.yaml"
    project_name::String
    "Level in [0, 1], or \"default\""
    turbulence::Union{Float64, String}
    "Its file"
    project::String
    "The controller's settings, overrides applied"
    fcs::FC_Settings
    "The optimizer's settings, overrides applied"
    tos::TrajOptSettings
    "The kite's settings, the wind override applied"
    project_set::Settings
    "[m] the starting tether length of the settings"
    l_tether::Float64
    "[s] asked of `init`, see `sim_budget`"
    effective_sim_time::Float64
    "Where the log, the summary and the marker go"
    output_path::String
    "The finished-run marker, see `write_run_done`"
    run_done_file::String
    "The log's name, `<log_file>_opt`"
    log_name::String
    # ---- the plant, the winch and the controllers ----
    "The ONE winch settings of both winch loops"
    wc::WCSettings
    "The length loop of the plant's winch"
    wpc::WinchPosController
    "[s] 1 / sample_freq"
    dt0::Float64
    "The same object as `wc`, as the reel-out controller reads it"
    rcs::WCSettings
    "The plant, built by the caller's `init_model`"
    s::S
    "The reel-out winch controller"
    rc::WinchController
    "[N] the force ceiling before the first-lap reduction"
    f_high_nominal::Float64
    "The force floor before the reel-out"
    guard_lfc::LowerForceController
    "[m] the settled length, as the plant reports it (Float32 for V3Kite); the startup request sends it unconverted"
    l_set::Real
    "The path in the air and the guidance on it"
    fec::FigureEightController
    # ---- the optimizer ----
    "The wind sent with every request"
    inflow::InflowConditions
    "[m/s] the wind the pattern box is sized at"
    cap_wind::Float64
    "The winch sent with the startup solve"
    winch::WinchParams
    "The same under the first-lap force limit"
    winch_first_lap::WinchParams
    "The winch sent with the re-optimizations"
    winch_reopt::WinchParams
    "[deg] centre elevation of the shipped guess"
    el_center_seed_base::Float64
    "[deg] the seed's centre, after `solve_startup_path!` the one it converged from"
    el_center_seed::Float64
    "The session with the optimizer, and its caches"
    opt_chain::OptChain
    "[-] anchor ratio and headroom of the turn-radius request"
    opt_r_scale::Float64
    "[m] the startup turn-radius request; nothing when off"
    opt_r_min::Union{Nothing, Float64}
    "Whether a turn radius is requested"
    opt_r_on::Bool
    "[m] the radius the startup solve was sent"
    opt_r_sent::Union{Nothing, Float64}
    "The pattern box sent; nothing when off"
    opt_box::Union{Nothing, PatternLimits}
    "Every reply's depower"
    opt_depower_log::Vector{NamedTuple}
    # ---- the laws the gates and the loop read ----
    "Pred -> whether the power gates are bypassed for a prediction [W]"
    power_gate_off::Function
    "(az, el) -> the lobe lift [deg] of an installed path"
    wing_lift::Function
    "Depower -> turn-rate gain c1 [1/m] of the path side; NaN off the table"
    c1_at_depower::CD
    "Depower -> c1 [1/m] of the controller's table; NaN off it"
    c1_ctrl_at::CC
    "[1/m] c1 at the depower setpoint, the loop's tuning point"
    c1_setpoint::Float64
    "[-] the highest depower of the controller's table"
    c1_depower_max::Float64
    "Reply -> the depower its path is flown at"
    pattern_depower::Function
    "[deg] the elevation floor of every candidate path"
    el_floor::Float64
    # ---- filled by `solve_startup_path!` (via `merge_into!`) ----
    "[deg] offset of the seed the startup solve converged from"
    startup_seed_offset::Float64 = NaN
    "That seed's guess, azimuth [deg]"
    guess_az::Vector{Float64} = Float64[]
    "That seed's guess, elevation [deg]"
    guess_el::Vector{Float64} = Float64[]
    "[s] wall time of the startup solve"
    opt_startup_solve_s::Float64 = NaN
    # ---- filled by `startup_feasibility` (via `merge_into!`) ----
    "The startup gates' verdict"
    feas::Union{Nothing, ReeloutFeasibility} = nothing
    "The in-air phase-5 check"
    margin5::Union{Nothing, Phase5MarginState} = nothing
    "(phase, depower | st) -> c1 [1/m] to check a path against"
    c1_at_phase::Union{Nothing, Function} = nothing
    "(az, el) -> the margin phase 5 flies a path with"
    phase5_margin_at::Union{Nothing, Function} = nothing
end

"""
    merge_into!(setup::RunSetup, results::NamedTuple) -> setup

Store `results` in the fields of the same names, e.g. what [`solve_startup_path!`](@ref) and
[`startup_feasibility`](@ref) return. A name that is not a field of `RunSetup` is an error.
"""
function merge_into!(setup::RunSetup, results::NamedTuple)
    for (name, value) in pairs(results)
        setproperty!(setup, name, value)
    end
    return setup
end

"""
    setup_run(inputs; init_model) -> RunSetup

Everything the run READS and never rebinds, built in order: the settings and their overrides, the
plant `s` (built by the caller's `init_model(project, project_set, fcs, wpc, sim_time; turbulence,
set_overrides)`, since the package does not depend on the model), the winch and its controllers, the optimizer's conditions and session, the turn-rate laws
(`c1_at_depower`, `c1_ctrl_at`) and the lobe lift. The script keeps the result in the one global
`setup` and hands it to every function of the run; the loop destructures it (see `run_loop!`).
The fields are documented at [`RunSetup`](@ref).
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
    # (`tos_overrides`), e.g. `max_reopt = 0` for a test run.
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
    # The low-wind schedule (`fcs.low_wind`) at the wind flown: starting length, startup guess,
    # gain-schedule floor and phase-5 lift. Before `sim_budget`, which reels out from `l_tether`;
    # an override of the same setting wins.
    apply_low_wind_schedule!(fcs, tos, project_set;
                             keep = (keys(inputs.fcs_overrides)..., keys(inputs.tos_overrides)...,
                                     keys(inputs.set_overrides)...))
    l_tether = project_set.l_tether

    # Whether `min_power_frac`/`min_power_frac_prev` are bypassed for a candidate predicting `pred`
    # watts: only below `tos.power_gate_wind_min` mean wind AND only for a NEGATIVE prediction, where
    # the number reports the optimizer's winch model leaving its own domain rather than a bad path.
    # Defined after the wind override, so it gates on the speed actually flown.
    power_gate_off(pred) = pred < 0 && project_set.v_wind < tos.power_gate_wind_min

    # Simulated time to ask `init` for: the reel-out budget under a wind-speed override, see `sim_budget`.
    # Without a sim_time and a wind override that is `nothing`: the project's own, as `init` would take it.
    effective_sim_time = Float64(something(sim_budget(project, project_set, fcs, sim_time, wind_speed,
                                                      default_v_wind), project_set.sim_time))

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

    # The seed of the startup solve and the connection to the optimizer, see `optimizer_session`; anchored to the
    # STARTING length, re-optimizing during the run is `reoptimize!`. `solve_startup_path!` replaces the seed.
    (; el_center_seed_base, el_center_seed, opt_chain) =
        optimizer_session(tos, inflow, replay_paths, log_name)
    # Constraints the solve must respect; the turn radius carries the anchor ratio `L/r` and the gate's headroom.
    (; opt_r_scale, opt_r_min, opt_r_on, opt_r_sent, opt_box) =
        request_constraints(tos, fcs, inflow, cap_wind, opt_length(tos, l_set))
    # `startup_params` fits the guess into that box; a guess outside it ends in local infeasibility.
    let (guess_a, guess_b, guess_el) = guess_in_box(tos.guess_a, tos.guess_b, el_center_seed, opt_box)
        (guess_a, guess_b, guess_el) == (tos.guess_a, tos.guess_b, el_center_seed) ||
            @info @sprintf("  ... startup guess fitted into the box: %.1f° x %.1f° at %.1f°.",
                           guess_a, guess_b, guess_el)
    end

    # One row per depower value the optimizer reports back (startup, each ACCEPTED reopt), for summary and plot.
    opt_depower_log = NamedTuple[]

    # The LOBE lift, applied to every installed path on the reply AS IT ARRIVED, before `el_offset_final`.
    wing_lift(az, el) = lobe_lift(az, el; lift = fcs.reelout.el_offset_wing,
                                  mode = fcs.reelout.el_offset_wing_mode,
                                  az_full = fcs.reelout.el_offset_wing_az,
                                  az_blend = fcs.reelout.el_offset_wing_blend)

    # The turn-rate gain at a depower, NaN off the table; memoized because a blend asks every step.
    c1_memo = Dict{Float64, Float64}()
    c1_at_depower(depower) = get!(c1_memo, Float64(depower)) do
        try
            turn_rate_coeffs(fcs.run.body_damping, depower).c1
        catch exc
            exc isa ArgumentError || rethrow()
            # Once per run: a blend ramping off the grid would repeat it every step.
            any(isnan, values(c1_memo)) ||
                @warn @sprintf("No turn-rate coefficients at depower %.3f (body_damping \
                                %s): gates and gain fall back to the startup law there.",
                               depower, fcs.run.body_damping)
            NaN
        end
    end
    # The controller's turn-rate gain at a depower, from `ctrl_tr_table`; NaN off it, memoized like c1_at_depower.
    c1_ctrl_memo = Dict{Float64, Float64}()
    c1_ctrl_at(depower) = get!(c1_ctrl_memo, Float64(depower)) do
        try
            turn_rate_coeffs(fcs.run.body_damping, depower; table = ctrl_tr_table).c1
        catch exc
            exc isa ArgumentError || rethrow()
            NaN
        end
    end
    # The turn authority the loop was TUNED at; the sim loop rescales heading_p by c1_setpoint/c1(u_d) in every phase.
    c1_setpoint = c1_ctrl_at(fcs.course.depower_setpoint)
    # Phase 4 must fly with the curvature feed-forward, which silently drops out on either of these.
    fcs.feedforward.ff_gain > 0 || @warn "simple_opt_reelout.jl needs the curvature feed-forward in phase 4, \
        but ff_gain = $(fcs.feedforward.ff_gain)"
    @assert isfinite(c1_setpoint) && c1_setpoint > 0 "the curvature feed-forward needs the turn-rate \
        coefficient c1 at depower_setpoint = $(fcs.course.depower_setpoint), got $c1_setpoint"
    c1_depower_max = try
        last(turn_rate_depower_range(fcs.run.body_damping; table = ctrl_tr_table))
    catch exc
        exc isa ArgumentError || rethrow()
        NaN
    end
    # The depower a reply is FLOWN at, which is what every gate and request must read c1 at.
    pattern_depower(reply) =
        !isnothing(reply.depower) ?
            awetrim_depower_to_v3kite(reply.depower.value) : fcs.course.depower_setpoint
    # The elevation floor of every candidate path, the startup gates' and `check_startup_path`'s.
    el_floor = fcs.run.min_elevation + tos.candidate_elevation_margin

    return RunSetup(; inputs, show_plots, steer_disturbance, xtrack_offset, xtrack_phase, hold_compliance,
            steer_gain_factor, steer_gain_feedback_only, extra_steer_delay, hook_settle, replay_paths,
            project_name, turbulence, project, fcs, tos, project_set, l_tether, effective_sim_time,
            output_path, run_done_file, log_name, wc, wpc, dt0, rcs, s, rc, f_high_nominal, guard_lfc,
            l_set, fec, inflow, cap_wind, winch, winch_first_lap, winch_reopt, el_center_seed_base,
            el_center_seed, opt_chain, opt_r_scale, opt_r_min, opt_r_on, opt_r_sent, opt_box,
            opt_depower_log, power_gate_off, wing_lift, c1_at_depower, c1_ctrl_at,
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
