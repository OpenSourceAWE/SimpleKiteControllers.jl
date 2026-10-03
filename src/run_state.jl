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
The loop-only bookkeeping is grouped as in the loop.
`startup_feasibility` and `reelout_results.jl` read the fields as `st.<field>`, and so does
`DelayedInjection` in `validate_margins.jl`, during the loop: `st` is the one global of the run's
state. Plain data: no model type, so the package can define it without depending on the model.

# Fields

$(TYPEDFIELDS)
"""
Base.@kwdef mutable struct RunState
    # ---- the optimizer's answer and what the retries make of it ----
    "The reply the run flies (startup, or the retry that took over)"
    opt_result::Union{Nothing, StepReply} = nothing
    "Its /trajectory table"
    opt_table::Union{Nothing, Dict{String, Any}} = nothing
    "Whether the flown path turns downwards in the loops, from its /trajectory table"
    opt_downloops::Union{Nothing, Bool} = nothing
    "[W] predicted mean reel-out power of the installed path"
    opt_power_pred::Float64 = NaN
    "Every optimizer answer as it arrived, before any lift"
    opt_paths_raw::Vector{AzElPath} = AzElPath[]
    "(sim time [s], phase) each of those was installed at"
    opt_paths_at::Vector{Tuple{Float64, Int}} = Tuple{Float64, Int}[]
    "Anchor ratio x headroom of the turn-radius request"
    opt_r_scale::Union{Nothing, Float64} = nothing
    "[m] turn-radius request, or nothing"
    opt_r_min::Union{Nothing, Float64} = nothing
    "Pattern limits sent with the last re-optimization request"
    opt_box_now::Union{Nothing, PatternLimits} = nothing
    "Score of the best startup path so far"
    incumbent_score::Union{Nothing, StartupScore} = nothing
    "The reply of the best startup path so far (the incumbent)"
    inc_result::Union{Nothing, StepReply} = nothing
    "The incumbent's /trajectory table"
    inc_table::Union{Nothing, Dict{String, Any}} = nothing
    "The incumbent's path as the optimizer sent it"
    inc_raw::Union{Nothing, AzElPath} = nothing
    "Share of the lobe lift the startup path could carry"
    startup_wing_frac::Float64 = 1.0
    "[-] turn-rate gain the startup path is checked against"
    c1_startup::Float64 = NaN
    "[-] rel_depower the optimizer asked for"
    depower_flown_opt::Float64 = NaN
    # ---- the startup pattern's geometry ----
    "Number of points of the path installed before the run"
    n_path_initial::Int = 0
    "Lowest height of the startup path at the starting tether length [m]"
    path_min_h_start::Float64 = NaN
    "Azimuth centre of the startup path [deg]"
    az_c_path::Float64 = NaN
    "Elevation centre of the startup path [deg]"
    el_c_path::Float64 = NaN
    "Azimuth half-width of the startup path [deg]"
    az_amp_path::Float64 = NaN
    "Elevation span (peak to peak) of the startup path [deg]"
    el_height_path::Float64 = NaN
    "(; t, power): which path was flown when"
    pred_timeline::Vector{PowerMark} = PowerMark[]
    "Every path flown, for the phase-5 fallback"
    p5_history::Vector{P5Record} = P5Record[]
    "Checked once, from the stop latch on, at the next crossing"
    p5_fallback_done::Bool = false
    "[deg] Q's azimuth from the path centre, last step"
    p5_q_az_prev::Float64 = NaN
    "(; t, from_margin, to_margin, to_t) when a fallback was blended in"
    p5_fallback::Union{Nothing, P5Fallback} = nothing
    "Course controller settings"
    ccs::Union{Nothing, CourseControllerSettings} = nothing
    "Course controller"
    cc::Union{Nothing, CourseController} = nothing
    # ---- winch and reel-out ----
    "[m] tether length setpoint"
    l_set::Float64 = NaN
    "[s] time phase 3 began; `reelout_delay` counts from it"
    transition_start::Float64 = NaN
    "[s] time the soft-stop deceleration latched; NaN = not yet"
    stop_start::Float64 = NaN
    "[m/s] v_set at the moment it latched"
    stop_v_entry::Float64 = NaN
    "[-] rel_depower at the moment it latched"
    stop_dp_entry::Float64 = NaN
    "[s] duration of the linear decel to reach 0 at reelout_l_max"
    stop_T::Float64 = NaN
    "True once the gate has opened; LATCHED, never re-closes"
    reelout_started::Bool = false
    "[s] time it opened; the soft-start ramp counts from here"
    reelout_start_t::Float64 = NaN
    "True if the FORCE trigger opened it, not the timer"
    reelout_trigger_fired::Bool = false
    "True once either stop criterion has ended reel-out"
    reelout_done::Bool = false
    "\"length\", \"laps\", or \"\" if reel-out never stopped"
    stop_reason::String = ""
    "[s] time phase 5 began; the run ends `fcs.reelout.final_time` after it"
    final_start::Float64 = NaN
    "[Wh] running mechanical energy, logged for the viewer"
    e_mech::Float64 = 0.0
    "Whether the first-lap reduction of the upper force limit is in force"
    first_lap_f_high_applied::Bool = false
    # ---- feed-forward, depower ----
    "[-] feed-forward steering per step"
    ff_log::Vector{Float64} = Float64[]
    "[rad] chord correction per step"
    ff_chi_log::Vector{Float64} = Float64[]
    "[-] low-passed feed-forward steering"
    ff_u_filt::Float64 = 0.0
    "[rad] low-passed chord correction"
    ff_chi_filt::Float64 = 0.0
    "[-] phase-5 force limiter's depower above depower_final"
    dp_final_extra::Float64 = 0.0
    "[-] the most it asked for, for the summary"
    dp_final_extra_peak::Float64 = 0.0
    "[-] depower commanded last step; the gain reads c1 there"
    rel_depower_prev::Float64 = NaN
    "[-] current blended output"
    depower_flown::Float64 = NaN
    "Depower the current blend started from [-]"
    depower_blend_from::Float64 = NaN
    "Depower the current blend goes to [-]; `nothing` when none is in progress"
    depower_blend_to::Union{Nothing, Float64} = nothing
    "Time the current depower blend started [s]"
    depower_blend_t0::Float64 = NaN
    # ---- lap counter and scored reference ----
    "Live lap count: 0 before phase 4, 1 at first entry, +1 per traversal"
    fig8_n::Int = 0
    "Index of Q on the path at the previous step"
    fig8_idx_prev::Int = 0
    "Path points Q has advanced since phase 4 began, for the lap count"
    fig8_idx_progress::Float64 = 0.0
    "Number of points of the installed path"
    n_path::Int = 0
    "Azimuth of the reference TRACKING is scored against [deg]"
    raw_az::Union{Nothing, Vector{Float64}} = nothing
    "Elevation of the reference TRACKING is scored against [deg]"
    raw_el::Union{Nothing, Vector{Float64}} = nothing
    "Resolution the path in the air is checked at"
    chk_points::Int = 0
    # ---- elevation lift ----
    "[deg] lift the path in the air actually carries"
    el_applied::Float64 = 0.0
    "`el_offset_final` latched in; never cleared once set"
    lift_on::Bool = false
    "In-air shift attempts, one per outcome CHANGE"
    el_shift_events::Vector{NamedTuple} = NamedTuple[]
    "[s] when it latched; NaN = never"
    lift_t::Float64 = NaN
    "[m] of reel-out left at that moment"
    lift_remaining::Float64 = NaN
    "A held-back shift warns once"
    el_shift_warned::Bool = false
    "Lap of the last in-air shift attempt"
    el_shift_lap::Int = 0
    "Elevation target of the last in-air shift attempt [deg]"
    el_shift_target::Float64 = NaN
    # ---- per-step logs of the pattern asked for ----
    "Time of each logged step [s]"
    geom_t::Vector{Float64} = Float64[]
    "Azimuth centre of the pattern asked for at each step [deg]"
    geom_az_c::Vector{Float64} = Float64[]
    "Azimuth half-width of the pattern asked for at each step [deg]"
    geom_az_amp::Vector{Float64} = Float64[]
    "Elevation span of the pattern asked for at each step [deg]"
    geom_el_h::Vector{Float64} = Float64[]
    "Cross-track error to the scored reference"
    geom_d_raw::Vector{Float64} = Float64[]
    "Number of |azimuth| bins of the droop statistics: where in the pattern the kite ends up low"
    n_droop_bins::Int = 5
    "Number of samples in each |azimuth| bin"
    droop_n::Vector{Int} = zeros(Int, 5)
    "[deg] kite below the path's elevation centre"
    droop_flown::Vector{Float64} = zeros(5)
    "[-] depth of the path at Q, in half-spans"
    droop_ref::Vector{Float64} = zeros(5)
    "[deg] kite below the path at Q"
    droop_sag::Vector{Float64} = zeros(5)
    # ---- re-optimization (stage 4) and blend ----
    "A solve is queued on the server"
    reopt_pending::Bool = false
    "Solves completed, accepted or rejected"
    reopt_n::Int = 0
    "Lap count at which the last request went out"
    reopt_lap::Float64 = 0.0
    "[s] next /status poll"
    reopt_next_poll::Float64 = 0.0
    "[s] when the pending request went out"
    reopt_t_request::Float64 = NaN
    "[s] wall time spent frozen waiting for a reply"
    reopt_blocked_s::Float64 = 0.0
    "[s] wall time the last blocking wait took"
    reopt_last_solve_s::Float64 = NaN
    "One row per solve, for the run summary"
    reopt_events::Vector{NamedTuple} = NamedTuple[]
    "[s] time() when the cycle's first request went out"
    reopt_t_wall_request::Float64 = NaN
    "(; t, l, status, wall_s) per completed cycle"
    reopt_cycles::Vector{NamedTuple} = NamedTuple[]
    "Cold-restart attempts spent on a rejected reply"
    blend_retries_total::Int = 0
    "Cold challenger solves run against an accepted reply (`challenge_growth`)"
    challenges_total::Int = 0
    "Challenger solves that were installed instead of the reply they challenged"
    challenges_won::Int = 0
    "[deg] shortfall of the last reply gated out; carried across cycles"
    el_min_extra::Float64 = 0.0
    "The path the blend in progress starts from; fold-free across w in [0, 1]"
    blend_from::Union{Nothing, AzElPath} = nothing
    "The path the blend in progress goes to"
    blend_to::Union{Nothing, AzElPath} = nothing
    "Time the blend in progress started [s]"
    blend_t0::Float64 = NaN
    "The scored reference's endpoint the SAME blend starts from"
    raw_from::Union{Nothing, AzElPath} = nothing
    "The scored reference's endpoint the blend goes to"
    raw_to::Union{Nothing, AzElPath} = nothing
    # ---- test inputs ----
    "[s] time phase 4 was first reached this run; NaN before that"
    t_phase4::Float64 = NaN
    "[s] first step of phase `xtrack_phase`; τ counts from here"
    xt_start::Float64 = NaN
    "[N] low-passed force of the compliant hold"
    hold_f_lp::Float64 = NaN
    "[m] length the compliant hold began at"
    hold_l0::Float64 = NaN
    "[s] time of each disturbed step"
    dist_t::Vector{Float64} = Float64[]
    "[-] disturbance added"
    dist_d::Vector{Float64} = Float64[]
    "[-] steering sent to the model, controller plus disturbance"
    dist_u::Vector{Float64} = Float64[]
    """
    FIFO of the raw steering commands, kept full of the last `extra_steer_delay` of them
    from the start of the run, so it is already primed with real history by the time the
    hook switches on. The feed-forward goes through a FIFO of its own, so the two stay aligned.
    """
    steer_delay_buf::Vector{Float64} = Float64[]
    "FIFO of the feed-forward steering, delayed like `steer_delay_buf`"
    ff_delay_buf::Vector{Float64} = Float64[]
    "[s] time of each phase-5 step"
    xt_t::Vector{Float64} = Float64[]
    "[deg] offset commanded"
    xt_delta::Vector{Float64} = Float64[]
    "[deg] signed cross-track error to the unshifted path, right of travel > 0"
    xt_d::Vector{Float64} = Float64[]
    "[-] index of the closest path point Q"
    xt_q::Vector{Int} = Int[]
    "[-] flight phase, and the operating point for the model:"
    xt_phase::Vector{Int} = Int[]
    "[m] tether length"
    xt_L::Vector{Float64} = Float64[]
    "[m/s] apparent wind speed"
    xt_va::Vector{Float64} = Float64[]
    "[m/s] kite speed normal to the tether"
    xt_vk::Vector{Float64} = Float64[]
    "[-] depower"
    xt_dp::Vector{Float64} = Float64[]
    # ---- results, for the finished-run marker (`write_run_done`) ----
    "The scored verdict, once `reelout_results` has it"
    fig8m::Union{Nothing, NamedTuple} = nothing
    "[W] measured mean reel-out power, or nothing"
    opt_power_meas::Union{Nothing, Float64} = nothing
    "The run's archive folder, \"none\" until (or unless) it exists"
    archive_dir::String = "none"
end
