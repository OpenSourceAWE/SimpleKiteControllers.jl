# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The state of one reel-out run of examples/simple_opt_reelout.jl.

# The value types of `RunState`'s fields, named where a field's type would otherwise hide what it holds.
"An (azimuth, elevation) path [deg], as the optimizer sent it or as it is flown"
const AzElPath = Tuple{Vector{Float64}, Vector{Float64}}
"`score_installed`'s verdict on a startup path"
const StartupScore = @NamedTuple{margin::Float64, el_ok::Bool, clr_ok::Bool, height::Float64, ok::Bool}
"One entry of `pred_timeline`: the predicted power [W] of the path flown from time `t` [s] on"
const PowerMark = @NamedTuple{t::Float64, power::Float64}
"One entry of `p5_history`: a path flown from `t` [s], its raw form and its phase-5 margin"
const P5Record = @NamedTuple{t::Float64, az::Vector{Float64}, el::Vector{Float64}, raw::AzElPath,
                             margin::Float64, el_applied::Float64}
"The phase-5 fallback that was blended in"
const P5Fallback = @NamedTuple{t::Float64, from_margin::Float64, to_margin::Float64, to_t::Float64}

"""
    RunState

Everything the startup functions and the simulation loop of `examples/simple_opt_reelout.jl`
WRITE, in one place, so they take it as an argument (`st`) instead of rebinding script globals.
Comments give the meaning and the unit; the loop-only bookkeeping is grouped as in the loop.
`startup_feasibility` and `reelout_results.jl` read the fields as `st.<field>`, and so does
`DelayedInjection` in `validate_margins.jl`, during the loop: `st` is the one global of the run's
state. Plain data: no model type, so the package can define it without depending on the model.
"""
Base.@kwdef mutable struct RunState
    # ---- the optimizer's answer and what the retries make of it ----
    opt_result::Union{Nothing, StepReply} = nothing  # the reply the run flies (startup, or the retry that took over)
    opt_table::Union{Nothing, Dict{String, Any}} = nothing  # its /trajectory table
    opt_downloops::Union{Nothing, Bool} = nothing
    opt_power_pred::Float64 = NaN           # [W] predicted mean reel-out power of the installed path
    opt_paths_raw::Vector{AzElPath} = AzElPath[]  # every optimizer answer as it arrived, before any lift
    opt_paths_at::Vector{Tuple{Float64, Int}} = Tuple{Float64, Int}[]  # (sim time [s], phase) each of those was installed at
    opt_r_scale::Union{Nothing, Float64} = nothing  # anchor ratio x headroom of the turn-radius request
    opt_r_min::Union{Nothing, Float64} = nothing  # [m] turn-radius request, or nothing
    opt_box_now::Union{Nothing, PatternLimits} = nothing  # pattern limits sent with the last re-optimization request
    incumbent_score::Union{Nothing, StartupScore} = nothing  # score of the best startup path so far
    inc_result::Union{Nothing, StepReply} = nothing
    inc_table::Union{Nothing, Dict{String, Any}} = nothing
    inc_raw::Union{Nothing, AzElPath} = nothing
    startup_wing_frac::Float64 = 1.0        # share of the lobe lift the startup path could carry
    c1_startup::Float64 = NaN               # [-] turn-rate gain the startup path is checked against
    depower_flown_opt::Float64 = NaN        # [-] rel_depower the optimizer asked for
    # ---- the startup pattern's geometry ----
    n_path_initial::Int = 0
    path_min_h_start::Float64 = NaN
    az_c_path::Float64 = NaN
    el_c_path::Float64 = NaN
    az_amp_path::Float64 = NaN
    el_height_path::Float64 = NaN
    pred_timeline::Vector{PowerMark} = PowerMark[]  # (; t, power): which path was flown when
    p5_history::Vector{P5Record} = P5Record[]  # every path flown, for the phase-5 fallback
    p5_fallback_done::Bool = false          # checked once, from the stop latch on, at the next crossing
    p5_q_az_prev::Float64 = NaN             # [deg] Q's azimuth from the path centre, last step
    p5_fallback::Union{Nothing, P5Fallback} = nothing  # (; t, from_margin, to_margin, to_t) when a fallback was blended in
    ccs::Union{Nothing, CourseControllerSettings} = nothing  # course controller settings
    cc::Union{Nothing, CourseController} = nothing  # course controller
    # ---- winch and reel-out ----
    l_set::Float64 = NaN                    # [m] tether length setpoint
    transition_start::Float64 = NaN         # [s] time phase 3 began; `reelout_delay` counts from it
    stop_start::Float64 = NaN               # [s] time the soft-stop deceleration latched; NaN = not yet
    stop_v_entry::Float64 = NaN             # [m/s] v_set at the moment it latched
    stop_dp_entry::Float64 = NaN            # [-] rel_depower at the moment it latched
    stop_T::Float64 = NaN                   # [s] duration of the linear decel to reach 0 at reelout_l_max
    reelout_started::Bool = false           # true once the gate has opened; LATCHED, never re-closes
    reelout_start_t::Float64 = NaN          # [s] time it opened; the soft-start ramp counts from here
    reelout_trigger_fired::Bool = false     # true if the FORCE trigger opened it, not the timer
    reelout_done::Bool = false              # true once either stop criterion has ended reel-out
    stop_reason::String = ""                # "length", "laps", or "" if reel-out never stopped
    final_start::Float64 = NaN              # [s] time phase 5 began; the run ends `fcs.final_time` after it
    e_mech::Float64 = 0.0                   # [Wh] running mechanical energy, logged for the viewer
    first_lap_f_high_applied::Bool = false
    # ---- feed-forward, depower ----
    ff_log::Vector{Float64} = Float64[]     # [-] feed-forward steering per step
    ff_chi_log::Vector{Float64} = Float64[] # [rad] chord correction per step
    ff_u_filt::Float64 = 0.0                # [-] low-passed feed-forward steering
    ff_chi_filt::Float64 = 0.0              # [rad] low-passed chord correction
    dp_final_extra::Float64 = 0.0           # [-] phase-5 force limiter's depower above depower_final
    dp_final_extra_peak::Float64 = 0.0      # [-] the most it asked for, for the summary
    rel_depower_prev::Float64 = NaN         # [-] depower commanded last step; the gain reads c1 there
    depower_flown::Float64 = NaN            # [-] current blended output
    depower_blend_from::Float64 = NaN
    depower_blend_to::Union{Nothing, Float64} = nothing
    depower_blend_t0::Float64 = NaN
    # ---- lap counter and scored reference ----
    fig8_n::Int = 0                         # live lap count: 0 before phase 4, 1 at first entry, +1 per traversal
    fig8_idx_prev::Int = 0
    fig8_idx_progress::Float64 = 0.0
    n_path::Int = 0
    raw_az::Union{Nothing, Vector{Float64}} = nothing  # the reference TRACKING is scored against
    raw_el::Union{Nothing, Vector{Float64}} = nothing
    chk_points::Int = 0                     # resolution the path in the air is checked at
    # ---- elevation lift ----
    el_applied::Float64 = 0.0               # [deg] lift the path in the air actually carries
    lift_on::Bool = false                   # `el_offset_final` latched in; never cleared once set
    el_shift_events::Vector{NamedTuple} = NamedTuple[]  # in-air shift attempts, one per outcome CHANGE
    lift_t::Float64 = NaN                   # [s] when it latched; NaN = never
    lift_remaining::Float64 = NaN           # [m] of reel-out left at that moment
    el_shift_warned::Bool = false           # a held-back shift warns once
    el_shift_lap::Int = 0                   # lap and target of the last in-air shift attempt
    el_shift_target::Float64 = NaN
    # ---- per-step logs of the pattern asked for ----
    geom_t::Vector{Float64} = Float64[]
    geom_az_c::Vector{Float64} = Float64[]
    geom_az_amp::Vector{Float64} = Float64[]
    geom_el_h::Vector{Float64} = Float64[]
    geom_d_raw::Vector{Float64} = Float64[] # cross-track error to the scored reference
    n_droop_bins::Int = 5                   # where in the pattern the kite ends up low, binned on |azimuth|
    droop_n::Vector{Int} = zeros(Int, 5)
    droop_flown::Vector{Float64} = zeros(5) # [deg] kite below the path's elevation centre
    droop_ref::Vector{Float64} = zeros(5)   # [-] depth of the path at Q, in half-spans
    droop_sag::Vector{Float64} = zeros(5)   # [deg] kite below the path at Q
    # ---- re-optimization (stage 4) and blend ----
    reopt_pending::Bool = false             # a solve is queued on the server
    reopt_n::Int = 0                        # solves completed, accepted or rejected
    reopt_lap::Float64 = 0.0                # lap count at which the last request went out
    reopt_next_poll::Float64 = 0.0          # [s] next /status poll
    reopt_t_request::Float64 = NaN          # [s] when the pending request went out
    reopt_blocked_s::Float64 = 0.0          # [s] wall time spent frozen waiting for a reply
    reopt_last_solve_s::Float64 = NaN       # [s] wall time the last blocking wait took
    reopt_events::Vector{NamedTuple} = NamedTuple[]  # one row per solve, for the run summary
    reopt_t_wall_request::Float64 = NaN     # [s] time() when the cycle's first request went out
    reopt_cycles::Vector{NamedTuple} = NamedTuple[]  # (; t, l, status, wall_s) per completed cycle
    blend_retries_total::Int = 0            # cold-restart attempts spent on a rejected reply
    el_min_extra::Float64 = 0.0             # [deg] shortfall of the last reply gated out; carried across cycles
    blend_from::Union{Nothing, AzElPath} = nothing  # the blend in progress; fold-free across w in [0, 1]
    blend_to::Union{Nothing, AzElPath} = nothing
    blend_t0::Float64 = NaN
    raw_from::Union{Nothing, AzElPath} = nothing  # the scored reference's endpoints of the SAME blend
    raw_to::Union{Nothing, AzElPath} = nothing
    # ---- test inputs ----
    t_phase4::Float64 = NaN                 # [s] time phase 4 was first reached this run; NaN before that
    xt_start::Float64 = NaN                 # [s] first step of phase `xtrack_phase`; τ counts from here
    hold_f_lp::Float64 = NaN                # [N] low-passed force of the compliant hold
    hold_l0::Float64 = NaN                  # [m] length the compliant hold began at
    dist_t::Vector{Float64} = Float64[]     # [s] time of each disturbed step
    dist_d::Vector{Float64} = Float64[]     # [-] disturbance added
    dist_u::Vector{Float64} = Float64[]     # [-] steering sent to the model, controller plus disturbance
    # Kept full of the last `extra_steer_delay` raw commands from the start of the run,
    # so it is already primed with real history by the time the hook switches on. The
    # feed-forward goes through a FIFO of its own, so the two stay aligned.
    steer_delay_buf::Vector{Float64} = Float64[]
    ff_delay_buf::Vector{Float64} = Float64[]
    xt_t::Vector{Float64} = Float64[]       # [s] time of each phase-5 step
    xt_delta::Vector{Float64} = Float64[]   # [deg] offset commanded
    xt_d::Vector{Float64} = Float64[]       # [deg] signed cross-track error to the unshifted path, right of travel > 0
    xt_q::Vector{Int} = Int[]               # [-] index of the closest path point Q
    xt_phase::Vector{Int} = Int[]           # [-] flight phase, and the operating point for the model:
    xt_L::Vector{Float64} = Float64[]       # [m] tether length
    xt_va::Vector{Float64} = Float64[]      # [m/s] apparent wind speed
    xt_vk::Vector{Float64} = Float64[]      # [m/s] kite speed normal to the tether
    xt_dp::Vector{Float64} = Float64[]      # [-] depower
    # ---- results, for the finished-run marker (`write_run_done`) ----
    fig8m::Union{Nothing, NamedTuple} = nothing  # the scored verdict, once `reelout_results` has it
    opt_power_meas::Union{Nothing, Float64} = nothing  # [W] measured mean reel-out power, or nothing
    archive_dir::String = "none"            # the run's archive folder, "none" until (or unless) it exists
end
