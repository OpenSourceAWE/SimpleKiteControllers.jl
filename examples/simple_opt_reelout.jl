# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Reel out along a path an EXTERNAL OPTIMIZER produced, instead of along a
lemniscate.

`simple_reelout.jl` with one block added — read that file's docstring for the
winch, the entry state machine, the log slots and the stop criteria, which all
apply unchanged. What differs is the reference path: the AWETrim optimizer is
asked for the power-optimal reel-out path under THIS run's wind and winch, and
`set_path!` installs it. [`simple_opt_fig8.jl`](simple_opt_fig8.jl) was the
rehearsal, flying the same path at constant length.

# The number this run exists to produce

The optimizer predicts a mean reel-out power for its path; reeling out along it
measures one. Both land in the `traj_opt:` section of the run summary with their
ratio. A large gap is the finding, not a bug to hide.

# Re-optimizing while the tether grows

`reopt_enabled` in `data/traj_opt.yaml` (off by default, so a run is
reproducible without a server) re-anchors the path to the length actually flown:
a request every `reopt_every_n_laps` laps, at most `max_reopt` times, polled via
`/status` and collected from `opt_trajectory` (its table is in RADIANS). While a
solve runs or after one fails, the server keeps serving the previous path.

`use_step: false` repeats the cold STARTUP solve (`/init` from the parametric
guess, then `/step`) at each length. `use_step: true` (shipped) sends `/init`
once and re-optimizes with `/step` alone, WARM-STARTING from the previous optimum
re-anchored to the new length — cheaper, but it follows one branch of a
multi-modal problem and the failure cache cannot key it; a failed warm step falls
back to one cold `/init`. The FLOWN path is never fed back as a seed: optimized
for a much shorter radius, it degrades as the run walks out and the solve escapes
to near-zenith (measured 2026-08-18: repeated failures from ~210 m, while the
guess converges at every length tried).

`reopt_blocking = true` (default) freezes the loop for the 7-13 s solve so the
reply matches the length asked for; the frozen time is reported as
`traj_opt.reopt.blocked` and excluded from `performance.realtime_factor`.
`false` flies on, anchoring the reply ~24 m behind at 2.4 m/s.

A candidate is checked for curvature, clearance and elevation AT THE CURRENT
LENGTH, at its OWN resolution (the flown path is resampled to `n_path`; upsampling
a coarse polyline makes the curvature check read too tight). A rejected candidate
is dropped and the run keeps flying what it has. One that passes is aligned with
[`prepare_path`](@ref) and blended in over `path_blend_time`
([`blend_paths`](@ref)), so the reference never steps and the lap counter is
re-based in the same move. Every solve is recorded in `traj_opt.reopt`.

# Lifting the path

The kite tracks BELOW the path it is given (1-2 deg, 3.5 deg at 380 m).
`fcs.el_offset_final` adds a fixed lift once reel-out ends — a setpoint move for
clearance. It latches at the STOP LATCH (the run's lowest point falls between
latch and phase 4 -> 5), or, with `reelout_softstop` at 0, once the length left
is under `v_reelout * el_offset_lead`. It reaches the kite through the next path
install or, when none is due, as a blend onto the path in the air, gated on the
curvature margin and rationed down to a quarter of it if the whole lift is
refused; the remainder waits in `el_target - el_applied` and is retried next lap.

`fcs.el_offset_wing` lifts the LOBES only, because the sag is deeper there than
at the crossing; it is baked into every installed path and rationed the same way.

Every lift is flown narrower, so the pattern-SIZE criteria break first;
`traj_opt.lift_budget` in the summary puts the lift bought against the room each
size criterion has left.

# Why the curvature margin is checked where it is

For ONE path flown all the way out, the start is the worst case: the angular
turn radius `1/(L*c1*u_s)` only shrinks with length, so the startup check runs
at the starting length. With `reopt_enabled` every re-optimization is the worst
case instead: the PHYSICAL radius `1/(c1*u_s)` = 11.35 m is length-independent,
and the optimizer's steering bounds put its tightest loop at 11.0-11.5 m at
every length, so the margin sits at ~1.0 and rejections are the normal case.
The real fix is to teach the optimizer the V3's turn-rate law, not to move the
gate.

# What comes from where

Optimizer settings — server, initial guess, solver knobs, resampling, margin —
are `data/traj_opt.yaml` ([`TrajOptSettings`](@ref)). The CONDITIONS come from
the system project's settings file (wind) and its `wc_settings` (winch law) via
`inflow_from_settings` and `winch_from_wc`. The guess decides whether and where
the solve converges, so this script never retries with a different one; see
`simple_opt_fig8.jl` for the measurements. `fcs.f8_a`/`f8_b`/`el_center` are NOT
flown here; the pattern's centre and extent are measured off the installed path.

`reelout_feasibility.jl` (abort/warn policy on the gates of
[`check_reelout_feasibility`](@ref), defining `feas`, `c1_at`, `phase5_margin`)
and `reelout_results.jl` (scoring, summary YAML, archive, plots, finished-run
marker) are `include`d at top level and share this script's globals.

Logs to `output/<log_file>_opt.arrow` and `_opt.yaml`, leaving the lemniscate
run's files intact for comparison. `REF_PATH` and `LOG_NAME` carry the flown
curve and that name to `simple_reelout_plots.jl`.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

# Wall clock of the whole script, packages included; `tic`/`toc` time the phases inside it.
t_script_start = time()
# Named here, not in `reelout_results.jl` where the summary is built and `@__FILE__` is that file.
run_script = basename(@__FILE__)

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
using WinchControllers: WCSettings, WinchController, calc_v_set, on_timer,
    get_state, get_f_err, wcsLowerForceLimit,
    LowerForceController, set_f_set, set_reset, set_v_sw, set_v_act,
    set_tracking, set_force, get_v_set_out, calc_vro
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: AtmosphericModel, calc_wind_factor
using LinearAlgebra: norm
using Statistics: mean
using Printf
import Dates
using OrderedCollections: OrderedDict

@info "simple_opt_reelout.jl: reeling out along an externally optimized path."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
# The optimizer client: opt_init/opt_step/opt_trajectory and ensure_server.
include(joinpath(@__DIR__, "awetrim_client.jl"))
# Read and cleared HERE, so a `SHOW_PLOTS = false` never survives into the next run.
show_plots = @isdefined(SHOW_PLOTS) ? SHOW_PLOTS : true
SHOW_PLOTS = true
# Reference curve and log name for simple_reelout_plots.jl; set below, cleared here like SHOW_PLOTS.
REF_PATH = nothing
LOG_NAME = nothing
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Tether/bridle structural damping as a ratio of stiffness [s]; see simple_fig8.jl's docstring.
DAMPING_PER_STIFFNESS = 0.001
PROJECT = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
@assert PROJECT in ("system_reelout_cabauw.yaml", "system_reelout_maasvlakte.yaml") "simple_opt_reelout.jl \
    supports only system_reelout_cabauw.yaml and system_reelout_maasvlakte.yaml, got $PROJECT"
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
WIND_SPEED = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
@info "simple_opt_reelout.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE, wind_speed = $(isnothing(WIND_SPEED) ? "default" : "$WIND_SPEED m/s")."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
# The optimizer's own settings: server, initial guess, solver knobs, margin.
tos = TrajOptSettings(traj_opt_settings_file(project))

# Sweep overrides (examples/optimize_fig8.jl), read and cleared here like SHOW_PLOTS.
fcs_overrides = @isdefined(FCS_OVERRIDES) ? FCS_OVERRIDES : Dict{Symbol, Any}()
FCS_OVERRIDES = Dict{Symbol, Any}()
for (key, value) in fcs_overrides
    hasfield(FC_Settings, key) ||
        error("FCS_OVERRIDES: \"$key\" is not a field of FC_Settings.")
    setfield!(fcs, key, convert(fieldtype(FC_Settings, key), value))
end
isempty(fcs_overrides) ||
    @info "fcs overrides in force: " * join(("$k = $v" for (k, v) in fcs_overrides), ", ")
# Test input: a steering disturbance `t -> Δu` added after the controller, read and cleared like
# SHOW_PLOTS; `stability_opt_reelout.jl`'s model is validated against the loop's response to it.
steer_disturbance = @isdefined(STEER_DISTURBANCE) ? STEER_DISTURBANCE : nothing
STEER_DISTURBANCE = nothing
isnothing(steer_disturbance) || @info "Steering disturbance in force (test input)."
dist_t = Float64[]      # [s] time of each disturbed step
dist_d = Float64[]      # [-] disturbance added
dist_u = Float64[]      # [-] steering sent to the model, controller plus disturbance

project_set = Settings(project)
default_v_wind = project_set.v_wind
apply_windspeed_override!(project_set, WIND_SPEED)
l_tether = project_set.l_tether

"""
    power_gate_off(pred) -> Bool

Whether `min_power_frac`/`min_power_frac_prev` are bypassed for a candidate
predicting `pred` watts: only below `tos.power_gate_wind_min` mean wind AND only
for a NEGATIVE prediction, where the number reports the optimizer's winch model
leaving its own domain rather than a bad path. Defined after the wind override,
so it gates on the speed actually flown.
"""
power_gate_off(pred) = pred < 0 && project_set.v_wind < tos.power_gate_wind_min

# Reel-out budget: the winch's sqrt-law at/above V_BUDGET_KNOT (wind at BUDGET_HEIGHT_M), ratio scaling below.
BELOW_DEFAULT_EXPONENT = 1.6  # exponent for scaling sim_time below the knot
BUDGET_HEIGHT_M = 100.0       # [m] height the budget's wind is taken at
V_BUDGET_KNOT = 7.7           # [m/s at BUDGET_HEIGHT_M] sqrt-law valid at/above; legacy scaling below
F_BUDGET_COEF = 48.0          # [N/(m/s)²] low-side fit of reeling-mean force ~ w_100², keeps the budget generous
REEL_MARGIN = 0.9             # achievable fraction of nominal speed (rings, soft-start)
BUDGET_ENTRY_S = 25.0         # park + dive + hold + reelout_delay [s]
BUDGET_TAIL_S = 10.0          # soft-stop ramp + phase-5 hold after length stop [s]
# The drum's own v_sat, read from the file so the budget follows a retune.
v_budget_cap =
    load_wc_settings(wc_settings(project); dt = 1 / project_set.sample_freq).v_sat
# Ratio of the wind at BUDGET_HEIGHT_M to the one at h_ref, from the project's own profile law.
budget_wind_factor = calc_wind_factor(AtmosphericModel(project_set; nowindfield = true),
                                      BUDGET_HEIGHT_M)
"Reel-out speed the budget assumes [m/s] at mean ground wind `w`: the winch's
own law at a conservative tension estimate from the wind at `BUDGET_HEIGHT_M`,
capped by the drum's speed limit. `winch_kv` stays keyed by the ground wind,
which is what its table lists."
v_reel_nominal(w) = min(winch_kv(w; project) * sqrt(F_BUDGET_COEF) * w * budget_wind_factor,
                        v_budget_cap)
EFFECTIVE_SIM_TIME = if isnothing(WIND_SPEED)
    SIM_TIME
elseif WIND_SPEED * budget_wind_factor < V_BUDGET_KNOT
    wind_ratio = default_v_wind / WIND_SPEED
    scale = wind_ratio <= 1 ? wind_ratio : wind_ratio^BELOW_DEFAULT_EXPONENT
    something(SIM_TIME, project_set.sim_time) * scale
else
    l_reel = fcs.reelout_l_max - l_tether
    BUDGET_ENTRY_S + l_reel / (REEL_MARGIN * v_reel_nominal(project_set.v_wind)) +
        BUDGET_TAIL_S
end
isnothing(WIND_SPEED) || @info @sprintf("simple_opt_reelout.jl: wind-speed override active, \
                                        %s",
    WIND_SPEED * budget_wind_factor < V_BUDGET_KNOT ?
    @sprintf("sim_time scaled to %.1f s (%.1f m/s at %.0f m, below the %.1f m/s knot).",
             EFFECTIVE_SIM_TIME, WIND_SPEED * budget_wind_factor, BUDGET_HEIGHT_M,
             V_BUDGET_KNOT) :
    @sprintf("reel-out budget %.1f s (%.0f s entry + %.0f m at %.2f m/s of %.2f nominal \
              + %.0f s tail; %.1f m/s at %.0f m)",
             EFFECTIVE_SIM_TIME, BUDGET_ENTRY_S, fcs.reelout_l_max - l_tether,
             v_reel_nominal(project_set.v_wind) * REEL_MARGIN,
             v_reel_nominal(project_set.v_wind), BUDGET_TAIL_S,
             WIND_SPEED * budget_wind_factor, BUDGET_HEIGHT_M))

# Arrow log files named after the project's `log_file`; OUTPUT_PATH redirects them for parallel sweep runs.
output_path = (@isdefined(OUTPUT_PATH) && !isnothing(OUTPUT_PATH)) ? OUTPUT_PATH :
              normpath(joinpath(@__DIR__, "..", "output"))
OUTPUT_PATH = nothing
mkpath(output_path)

# Finished-run marker for outside watchers: removed here, written last, so its presence means "this run is over".
const RUN_DONE_FILE = joinpath(output_path, "last_run_done.txt")
rm(RUN_DONE_FILE; force = true)

"""
    write_run_done(status; err = nothing)

Write the finished-run marker. Defined HERE, before anything that can throw, and
called from a `catch` as well as from the normal tail of `reelout_results.jl`, so
the marker's absence means "still running" and never "it died" — a watcher that
only ever sees the success path waits forever on a run that crashed (measured: a
GLMakie main-thread error in the plots, long after the simulation had finished
and the log was safely written).

`status` is `ok`, `ok (plots failed)` or `FAILED`; on failure the exception's
first line follows on an `error:` line. Every other field is read defensively —
a crash early enough leaves `archive_dir`/`fig8m` undefined, and the marker still
has to be writable.
"""
function write_run_done(status::AbstractString; err = nothing)
    get_global(name, default) = isdefined(Main, name) ? getfield(Main, name) : default
    open(RUN_DONE_FILE, "w") do io
        println(io, Dates.format(Dates.now(), "yyyy-mm-dd HH:MM:SS"))
        println(io, "status: ", status)
        isnothing(err) || println(io, "error: ", first(split(sprint(showerror, err), "\n")))
        println(io, "archive: ", get_global(:archive_dir, "none"))
        println(io, "log: ", joinpath(output_path, log_name * ".yaml"))
        fig8m = get_global(:fig8m, nothing)
        println(io, "criteria: ", isnothing(fig8m) ? "n/a" :
                                  isempty(fig8m.criteria_failed) ?
                                  "all $(fig8m.criteria) passed" :
                                  "FAILED: " * join(fig8m.criteria_failed, ", "))
        power = get_global(:opt_power_meas, nothing)
        println(io, "power: ", isnothing(power) ? "n/a" :
                               @sprintf("%.0f W measured", power))
    end
end
# `_opt`: never overwrite the lemniscate run's log and summary, the two are each other's baseline.
log_name = basename(project_set.log_file) * "_opt"

# ======================== INIT =========================== #

fcs.compliance >= 0 ||
    error("compliance must be >= 0, got $(fcs.compliance)")
fcs.compliance == 0 ||
    error("REEL_OUT needs compliance = 0 (POSITION mode) — REEL_OUT and V3Kite's own \
           FORCE mode both drive the winch and only one can hold the drum at a time.")
# ONE WCSettings for BOTH winch loops: the POSITION-mode torque gains (`wpc`) and the speed-controller tuning (`rc`).
dt0 = 1 / project_set.sample_freq
wc = load_wc_settings(wc_settings(project); dt = dt0)
wc.kv = winch_kv(project_set.v_wind; project) # overrides the file's flat kv, see data/winch_kv_table.yaml
# Wind-dependent floor too; NOT the entry guard's floor, that is fcs.entry_f_min.
wc.f_low = winch_f_low(project_set.v_wind; project)
# The soft law's floor cannot go below ~700 N, so it is off at low wind; see `winch_force_limit`'s docstring.
wc.force_limit = winch_force_limit(project_set.v_wind; project)
rcs = wc                                 # same object, two controllers read it
wpc = WinchPosController(wc; dt = dt0)   # the length loop `step!` used to own

# dt, sim_time and wind come from the project settings (overridden above); the default cache_path avoids a re-JIT.
s = init(project_set.v_wind, l_tether; body_start_damping = fcs.body_damping,
    body_sim_damping = 0.8 .* fcs.body_damping,
    damping_per_stiffness = DAMPING_PER_STIFFNESS,
    elevation = fcs.elevation, depower_setpoint = fcs.depower_setpoint,
    system_yaml = project, use_turbulence = TURBULENCE, aero_mode = AERO_MODE,
    sim_time = EFFECTIVE_SIM_TIME, warmup_time = fcs.warmup_time,
    # The warm-up relaxes at constant length, against the same loop the run uses.
    warmup_torque = (m, l) -> winch_torque!(wpc, m, l), remake_model = false)
@info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)

# Built here so the soft-start ramp begins when reel-out starts; `rcs` is `wc`, one file for both winches.
rcs.dt = s.dt
rc = WinchController(rcs)
# The nominal ceiling, captured before the first-lap reduction; `winch_from_wc` sends this one to the optimizer.
const F_HIGH_NOMINAL = rcs.f_high
first_lap_f_high_applied = false
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

# Standalone force-floor guard for phases 0-2: `rc`'s own SpeedController would wind up while its output is ignored.
guard_lfc = LowerForceController(rcs)

# Length setpoint: the settled length, growing from phase 3 until it reaches `reelout_l_max`.
l_set = s.sys_state.l_tether[1]

fec = FigureEightController(FigureEightSettings(;
    dt = s.dt, A = fcs.f8_a, B = fcs.f8_b,
    az_center = 0.0, el_center = fcs.el_center,
    attractor_distance = fcs.attractor_dist, up_loops = fcs.up_loops,
    reacquire_margin = fcs.reacquire_margin))

# ================= OPTIMIZED REFERENCE PATH ================== #

# The conditions of THIS run; `rcs` is the WCSettings the reel-out controller actually flies.
inflow = inflow_from_settings(project_set)
# The wind the elevation-cap step reads, at the height the step is keyed on.
cap_wind = cap_wind_speed(tos, project_set, inflow.wind_speed)
# What AWETrim is SENT, decoupled from the local law: the solve must converge at a force the kite can pull.
opt_awe_trim = tos.opt_awe_trim >= 0 ? tos.opt_awe_trim : rcs.use_awe_trim
opt_winch_mode = isempty(tos.opt_winch_mode) ? nothing : tos.opt_winch_mode
# STARTUP solve: the plain runtime ceiling, never `rcs.f_high_awe_trim`; that de-rating is confined to `winch_reopt`.
winch = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                      winch_mode = opt_winch_mode, f_max = F_HIGH_NOMINAL)
# Lap 1 flies under `first_lap_force_frac`, so the STARTUP path is solved against that same ceiling.
winch_first_lap = fcs.first_lap_force_frac < 1 ?
    winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                  winch_mode = opt_winch_mode,
                  f_max = F_HIGH_NOMINAL * fcs.first_lap_force_frac) : winch
# RE-OPTIMIZATION solves (lap 2 on): the one caller that may fly `rcs.f_high_awe_trim`.
winch_reopt = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                            winch_mode = opt_winch_mode)
# Every reply's optimized gain, so the summary reports what was flown, not only what was sent.
opt_kv_log = NamedTuple{(:t, :l, :k_v, :at_bound), Tuple{Float64, Float64, Float64, Bool}}[]

"""
    apply_optimized_kv!(tab, t, l)

Move the winch gain the optimizer chose out of a `/trajectory` reply and into the
`WCSettings` the run reads, so it flies the `k_v` the path was solved for. `wc`,
`rcs` and `rc.wcs` are one object, and every sub-controller of `rc` holds a
reference to it, so a single assignment reaches all of them. A reply that did not
optimize the gain carries no `k_v` under `optimized_parameters` and this is then a
no-op. A gain that ran into its own bracket is reported: the value is the edge of
the box, not an optimum.
"""
function apply_optimized_kv!(tab, t, l)
    tos.optimize_k_v || return
    params = get(tab, "optimized_parameters", nothing)
    raw = params === nothing ? nothing : get(params, "k_v", nothing)
    raw === nothing && return
    k_v = Float64(raw)
    k_v > 0 || return
    at_bound = something(get(params, "k_v_at_bound", false), false)
    if isempty(opt_kv_log) || abs(k_v - last(opt_kv_log).k_v) > 1e-9
        @info @sprintf("  ... optimizer chose k_v = %.5f at L = %.0f m (was %.5f)%s",
                       k_v, l, wc.kv, at_bound ? " — AT ITS BRACKET EDGE" : "")
        at_bound && @warn "k_v hit the K_V_BRACKET_FACTOR bound: the optimizer wanted \
                           to retune further than it was allowed, so this is the edge \
                           of the box rather than an optimum."
    end
    wc.kv = k_v
    @assert rc.wcs === wc "the reel-out controller must read the WCSettings the gain is written to"
    # EVERY accepted install with a gain, repeats included; a rejected candidate never reaches this function.
    push!(opt_kv_log, (; t, l, k_v, at_bound))
    return
end
@info @sprintf("Optimizer conditions: %.1f m/s at 6 m from %.0f°, profile_law %d, \
                z0 = %g m | winch kv = %.4f, i.e. %.1f m/s at f_high = %.0f N | \
                depower seed %.3f m%s.",
               inflow.wind_speed, inflow.wind_direction, inflow.profile_law, inflow.z0,
               winch.k_v, winch.k_v * sqrt(winch.f_max), winch.f_max,
               depower_seed(tos, inflow.wind_speed),
               inflow.wind_speed > tos.input_depower_wind_ref ?
                   @sprintf(" (%.2f + %.3f per m/s above %.1f m/s)", tos.input_depower,
                            tos.input_depower_per_wind, tos.input_depower_wind_ref) : "")

# The seed from data/traj_opt.yaml; `el_center_seed` is where the STARTUP solve finally converged from.
el_center_seed_base = guess_el_center_seed(tos, inflow.wind_speed)
el_center_seed = el_center_seed_base
startup_seed_offset = 0.0
guess_az, guess_el = figure_eight_path(tos.guess_a, tos.guess_b,
                                       0.0, el_center_seed,
                                       0.0, tos.guess_points)
@info @sprintf("Initial guess: %.0f° x %.0f° at %.0f°, %d points.",
               tos.guess_a, tos.guess_b, el_center_seed, tos.guess_points)

# Anchored to the STARTING length; re-optimizing during the run is stage 4, below.
ensure_server(tos.base_url; autostart = tos.autostart_server)
# Every request of the run goes through this chain, which replays applied results and known failures (see `OptChain`).
opt_chain = OptChain(tos.base_url; successes = tos.opt_success_cache,
                     failures = tos.opt_failure_cache)
# Constraints the solve must respect; the turn radius carries the anchor ratio `L/r` and the gate's headroom.
turn_radius_reel = turn_radius_lap_reelout(tos, inflow.wind_speed)
opt_r_scale = (1 + turn_radius_reel / l_set) * tos.turn_radius_headroom
# The depower the reply will be FLOWN at, which is the c1 the request must be sized
# at — see min_turn_radius_request. Under fly_opt_depower that is the optimizer's
# own and so unknown before the solve; the seed it starts from is the only estimate
# there is, and the setpoint (the default) is NOT one: it is the depower the loop is
# tuned at, typically far more powered, and a request sized there comes back a third
# too tight and is then gated out for a curvature the kite never had.
depower_request = tos.fly_opt_depower ?
                  awetrim_depower_to_v3kite(depower_seed(tos, inflow.wind_speed)) :
                  fcs.depower_setpoint
c1_request = try
    turn_rate_coeffs(fcs.body_damping, depower_request).c1
catch exc
    exc isa ArgumentError || rethrow()
    nothing       # off the grid: the request falls back to the setpoint and warns
end
opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale, c1 = c1_request)
opt_r_on = !isnothing(opt_r_min)   # off for margin 0, or an off-grid turn-rate cell
# The radius the startup solve actually CONVERGED at; the retry ladder bisects toward it.
opt_r_sent = opt_r_min
opt_box = pattern_limits_from(tos;
                              elevation_min = elevation_min_request(fcs, tos, l_set),
                              wind_speed = cap_wind)
isnothing(opt_r_min) && isnothing(opt_box) ||
    @info @sprintf("Constraints sent with the request: min_turn_radius %s, \
                    pattern box %s.",
                   isnothing(opt_r_min) ? "unset" :
                       @sprintf("%.2f m (min_feasibility_margin %.2f x the kite's \
                                own at depower %.3f%s, x %.3f for %.0f m of assumed \
                                reel-out per lap and %.2f of headroom)",
                                opt_r_min, tos.min_feasibility_margin,
                                depower_request,
                                tos.fly_opt_depower ? " — the seed's, not the \
                                    setpoint's, because the reply is flown at its own" : "",
                                opt_r_scale, turn_radius_reel,
                                tos.turn_radius_headroom),
                   isnothing(opt_box) ? "unset" : string(opt_box))

# One row per depower value the optimizer reports back (startup, each ACCEPTED reopt), for summary and plot.
opt_depower_log = NamedTuple[]
# What phases 3+ fly under `fly_opt_depower`; the fixed setpoint until the first optimizer answer.
depower_flown_opt = fcs.depower_setpoint

"""
    opt_length(l)

Tether length to SEND to the optimizer, rounded to `tos.opt_length_round` metres
(`0.0` sends `l` unchanged). The flown `l_set` is never rounded — see the setting's
docstring for why the request is.
"""
opt_length(l) = tos.opt_length_round > 0 ?
    round(l / tos.opt_length_round) * tos.opt_length_round : l

"""
    startup_seed_offsets(listed; max_abs = 10.0) -> Vector{Float64}

The centre-elevation offsets [deg] the startup solve may be seeded from, in
order: 0 (the shipped guess), then `listed` (`startup_retry_el_offsets`), then
every whole degree not yet in the list by growing magnitude, negative first, out
to `max_abs`. The tail only exists so a seed the failure cache rejects can be
replaced by one that has not been tried; how many of them are SENT is the
caller's budget, not this list. An empty `listed` never retries, so it gets no
tail either.
"""
function startup_seed_offsets(listed; max_abs = 10.0)
    offsets = [0.0; listed]
    isempty(listed) && return offsets
    for k in 1.0:max_abs, o in (-k, k)
        o in offsets || push!(offsets, o)
    end
    return offsets
end

"""
    startup_params(el_center) -> InitParams

The startup `/init` request seeded with the guess lemniscate centred at
`el_center` [deg]; everything else comes from `tos`, the inflow and the
first-lap winch.
"""
function startup_params(el_center)
    az, el = figure_eight_path(tos.guess_a, tos.guess_b,
                               0.0, el_center, 0.0, tos.guess_points)
    InitParams(; name = tos.name, length = opt_length(l_set),
               winch_params = winch_first_lap, inflow_conditions = inflow,
               trajectory = Trajectory(collect(az), collect(el)),
               input_depower = depower_seed(tos, inflow.wind_speed),
               reg_weight = tos.reg_weight,
               detect_simple_bounds = tos.detect_simple_bounds,
               min_turn_radius = opt_r_min, pattern_limits = opt_box)
end

"""
    startup_solve(params) -> (result, seed_trajectory)

`/init` with `params`, the optional seeding solve at `opt_warm_start_awe_trim`,
then the `/step` under the first-lap winch, the one lap 1 flies. Throws the `HTTP.StatusError` of a
422 unchanged; the caller decides whether that ends the run.
"""
function startup_solve(params)
    reply = chain_init(opt_chain, params)
    # Seeding solve: the cold first request fails at low winch force, so it is warmed at `opt_warm_start_awe_trim`.
    seed_trajectory = reply.trajectory
    if tos.opt_warm_start_awe_trim > winch.use_awe_trim
        @info @sprintf("Seeding solve at use_awe_trim %.3f before the startup \
                        request at %.3f; see opt_warm_start_awe_trim.",
                       tos.opt_warm_start_awe_trim, winch.use_awe_trim)
        warm_winch = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v,
                                   use_awe_trim = tos.opt_warm_start_awe_trim)
        seed_trajectory = chain_step(opt_chain, StepParams(opt_length(l_set), warm_winch,
                                                           reply.trajectory)).trajectory
    end
    result = chain_step(opt_chain, StepParams(opt_length(l_set), winch_first_lap,
                                              seed_trajectory))
    return result, seed_trajectory
end

start_params = startup_params(el_center_seed)
t_solve_start = time()
# A 422 is retried from `startup_retry_el_offsets` in order; cached failures are skipped and cost no retry.
opt_result = nothing
opt_seed_trajectory = nothing
let last_422 = nothing, cached_msg = nothing, sent = 0,
    budget = 1 + length(tos.startup_retry_el_offsets), sent_offsets = Float64[]
    for offset in startup_seed_offsets(tos.startup_retry_el_offsets)
        sent < budget || break
        el_center = el_center_seed_base + offset
        params = offset == 0 ? start_params : startup_params(el_center)
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
            global opt_result, opt_seed_trajectory = startup_solve(params)
            global start_params = params
            global el_center_seed = el_center
            global startup_seed_offset = offset
            global guess_az, guess_el = params.trajectory.azimuth,
                                        params.trajectory.elevation
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
end
startup_seed_offset == 0 ||
    @warn @sprintf("Startup path solved from a RETRY seed centred at %.0f° \
                    (%+.1f° off guess_el_center): a different optimum than the \
                    shipped guess would have given.", el_center_seed, startup_seed_offset)
# The startup solve holds the script; blocking re-optimizations hold the loop (`reopt_blocked_s`).
opt_startup_solve_s = time() - t_solve_start
toc("Received the optimized path in: ")

# What the reply was optimized AT, converted to the V3Kite rel_depower flying at the SAME power.
if !isnothing(opt_result.depower)
    u_p_equiv = awetrim_depower_to_v3kite(opt_result.depower.value)
    @info @sprintf("Optimized at depower l_dp = %.3f m (mode %s) = rel_depower \
                    %.3f equivalent, against the flown depower_setpoint = %.3f — \
                    a %+.3f gap.",
                   opt_result.depower.value, opt_result.depower.mode, u_p_equiv,
                   fcs.depower_setpoint, u_p_equiv - fcs.depower_setpoint)
end
# The optimizer's own curvature diagnostic, physical and comparable with `min_feasibility_margin`.
isnothing(opt_result.metrics.turn_radius_min_m) ||
    @info @sprintf("Tightest physical turn radius of the reply: %.2f m%s.",
                   opt_result.metrics.turn_radius_min_m,
                   isnothing(opt_r_min) ? "" :
                       @sprintf(" (asked for >= %.2f m)", opt_r_min))

# The LOBE lift, applied to every installed path on the reply AS IT ARRIVED, before `el_offset_final`.
wing_lift(az, el) = lobe_lift(az, el; lift = fcs.el_offset_wing,
                              mode = fcs.el_offset_wing_mode,
                              az_full = fcs.el_offset_wing_az,
                              az_blend = fcs.el_offset_wing_blend)
if fcs.el_offset_wing != 0 && fcs.el_offset_wing_mode == "azimuth"
    @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.1f°, ramped over %.1f°, \
                    zero inside %.1f°.",
                   fcs.el_offset_wing, fcs.el_offset_wing_az, fcs.el_offset_wing_blend,
                   fcs.el_offset_wing_az - fcs.el_offset_wing_blend)
elseif fcs.el_offset_wing != 0 && fcs.el_offset_wing_mode == "azimuth_frac"
    # Fractions of each path's own amplitude, so the degrees below are the STARTUP pattern's.
    amp0 = 0.5 * (maximum(opt_result.trajectory.azimuth) -
                  minimum(opt_result.trajectory.azimuth))
    @info @sprintf("Lobe lift: %+.2f° beyond |azimuth| = %.2f of the pattern's own \
                    amplitude, ramped over %.2f of it — %.1f° and %.1f° on the \
                    startup path's ±%.1f°.",
                   fcs.el_offset_wing, fcs.el_offset_wing_az, fcs.el_offset_wing_blend,
                   fcs.el_offset_wing_az * amp0, fcs.el_offset_wing_blend * amp0, amp0)
end

# The turn-rate gain at a depower, NaN off the table; memoized because a blend asks every step.
const c1_memo = Dict{Float64, Float64}()
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
# The turn authority the loop was TUNED at; the sim loop rescales heading_p by c1_setpoint/c1(u_d) in every phase.
c1_setpoint = c1_at_depower(fcs.depower_setpoint)
# Phase 4 must fly with the curvature feed-forward, which silently drops out on either of these.
@assert fcs.ff_gain > 0 "simple_opt_reelout.jl needs the curvature feed-forward in phase 4, \
    but ff_gain = $(fcs.ff_gain)"
@assert isfinite(c1_setpoint) && c1_setpoint > 0 "the curvature feed-forward needs the turn-rate \
    coefficient c1 at depower_setpoint = $(fcs.depower_setpoint), got $c1_setpoint"
c1_depower_max = try
    last(turn_rate_depower_range(fcs.body_damping))
catch exc
    exc isa ArgumentError || rethrow()
    NaN
end
# The depower a reply is FLOWN at, which is what every gate and request must read c1 at.
pattern_depower(reply) =
    tos.fly_opt_depower && !isnothing(reply.depower) ?
        awetrim_depower_to_v3kite(reply.depower.value) : fcs.depower_setpoint
# The turn-rate law the retry reads a path against; `reelout_feasibility.jl` looks it up again later.
c1_startup = c1_setpoint
# Resample but never upsample; the lobe lift is rationed to fit the curvature gate.
startup_wing_frac = 1.0
install_optimized_path!(reply) = begin
    global startup_wing_frac, c1_startup
    az = collect(Float64.(reply.trajectory.azimuth))
    el = collect(Float64.(reply.trajectory.elevation))
    lift = wing_lift(az, el)
    resample = min(tos.resample_points, length(az) - 1)
    c1_startup = c1_at_depower(pattern_depower(reply))
    startup_wing_frac = 1.0
    if !isnan(c1_startup) && tos.min_feasibility_margin > 0 && any(!=(0), lift)
        for fw in (1.0, 0.75, 0.5, 0.25, 0.0)
            startup_wing_frac = fw
            set_path!(fec, az, el .+ fw .* lift; resample)
            check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                   c1 = c1_startup, prn = false).margin >=
                tos.min_feasibility_margin && break
        end
        startup_wing_frac < 1 &&
            @info @sprintf("Lobe lift held back on the startup path to fit the \
                            curvature gate: %.0f %% of %.2f°.",
                           100 * startup_wing_frac, fcs.el_offset_wing)
    else
        set_path!(fec, az, el .+ lift; resample)
    end
    return (az, el)
end
# Every optimizer answer as it arrived, before any lift; installed paths shrink as the tether grows.
opt_paths_raw = [install_optimized_path!(opt_result)]
# Where each of those was installed: (sim time [s], phase); the startup path goes in before the run.
opt_paths_at = [(0.0, 0)]

# set_path! REVERSES a path that does not match up_loops, so a mismatch must be caught here.
opt_table = chain_trajectory(opt_chain)
# Installed above, so applied: stored for a rerun that sends the same requests.
record_opt_success!(opt_chain)
apply_optimized_kv!(opt_table, 0.0, l_tether)
opt_downloops = opt_table["spline"]["downloops"]
opt_power_pred = Float64(opt_table["metrics"]["avg_power_W"])
# The anchor ratio, now measured off the reply; guarded so a request that is off stays off.
if opt_r_on
    opt_r_scale = reelout_anchor_ratio(opt_table) * tos.turn_radius_headroom
    opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale,
                                        c1 = c1_startup)
end
isnothing(opt_r_min) ||
    @info @sprintf("Turn-radius request for the re-optimizations: %.2f m — the \
                    gate's %.2f m at margin %.2f, x %.3f for the lap's reel-out \
                    (%.1f -> %.1f m) and x %.2f of headroom.",
                   opt_r_min, opt_r_min / opt_r_scale, tos.min_feasibility_margin,
                   reelout_anchor_ratio(opt_table),
                   minimum(Float64.(opt_table["table"]["distance_radial"])),
                   maximum(Float64.(opt_table["table"]["distance_radial"])),
                   tos.turn_radius_headroom)

# ---- Corrected retries of the STARTUP solve: one lever per attempt (ceiling, width, radius) ---- #
const RETRY_GAIN_MAX = 1.15   # largest per-attempt scaling of the turn-radius ask
const RETRY_CAP_SLACK = 0.5     # box height kept above the incumbent's own    [deg]
const RETRY_CAP_MIN_STEP = 0.5  # smallest ceiling step still worth a solve    [deg]

"""
    with_elevation_max(box, el_max) -> PatternLimits

`box` (a `PatternLimits` or `nothing`) with its `elevation_max` replaced by
`el_max` [deg]; every other side is kept.
"""
with_elevation_max(box, el_max) = isnothing(box) ?
    PatternLimits(; elevation_max = el_max) :
    PatternLimits(; azimuth_max = box.azimuth_max, elevation_min = box.elevation_min,
                  elevation_max = el_max,
                  azimuth_amplitude_min = box.azimuth_amplitude_min,
                  elevation_amplitude_max = box.elevation_amplitude_max,
                  symmetric = box.symmetric)

"""
    with_azimuth_amplitude_min(box, a_min) -> PatternLimits

`box` (a `PatternLimits` or `nothing`) with its `azimuth_amplitude_min` replaced
by `a_min` [deg]; every other side is kept.
"""
with_azimuth_amplitude_min(box, a_min) = isnothing(box) ?
    PatternLimits(; azimuth_amplitude_min = a_min) :
    PatternLimits(; azimuth_max = box.azimuth_max, elevation_min = box.elevation_min,
                  elevation_max = box.elevation_max,
                  azimuth_amplitude_min = a_min,
                  elevation_amplitude_max = box.elevation_amplitude_max,
                  symmetric = box.symmetric)

"The server's amplitude measure of a path's azimuth [deg]: the RMS-based
half-width its `azimuth_amplitude_min` row bounds, `sqrt(2 * mean((az - mean(az))^2))`."
azimuth_amplitude(az) = sqrt(2 * mean((az .- mean(az)) .^ 2))

"The server's amplitude measure of a path's elevation [deg]: the RMS-based
half-span its `elevation_amplitude_max` row caps, same formula as [`azimuth_amplitude`](@ref)."
elevation_amplitude(el) = sqrt(2 * mean((el .- mean(el)) .^ 2))

"""
    with_size_box(box, az_prev, el_prev, growth) -> Union{PatternLimits, Nothing}

`box` (a `PatternLimits` or `nothing`) tightened to `growth` times the size of
the path `(az_prev, el_prev)` [deg]: `azimuth_max` to `growth * max|az_prev|`,
`elevation_amplitude_max` to `growth * elevation_amplitude(el_prev)`, and the
elevation range `[elevation_min, elevation_max]` to the path's own, each end let
out by `(growth - 1)/2` of its span — every side only where that is TIGHTER
than what the box already holds, the rest kept. `growth <= 0` returns `box`
unchanged. See `TrajOptSettings.size_box_growth`.
"""
function with_size_box(box, az_prev, el_prev, growth)
    growth > 0 || return box
    az_lim = growth * maximum(abs, az_prev)
    el_lim = growth * elevation_amplitude(el_prev)
    # The RMS half-span does not bound the peak-to-peak span the gate reads, so the elevation RANGE is boxed too.
    el_lo, el_hi = extrema(el_prev)
    slack = 0.5 * (growth - 1) * (el_hi - el_lo)
    tighter(old, new) = isnothing(old) ? new : min(old, new)
    higher(old, new) = isnothing(old) ? new : max(old, new)
    isnothing(box) && return PatternLimits(; azimuth_max = az_lim,
                                           elevation_min = el_lo - slack,
                                           elevation_max = el_hi + slack,
                                           elevation_amplitude_max = el_lim)
    return PatternLimits(; azimuth_max = tighter(box.azimuth_max, az_lim),
                         elevation_min = higher(box.elevation_min, el_lo - slack),
                         elevation_max = tighter(box.elevation_max, el_hi + slack),
                         azimuth_amplitude_min = box.azimuth_amplitude_min,
                         elevation_amplitude_max = tighter(box.elevation_amplitude_max, el_lim),
                         symmetric = box.symmetric)
end

# Read at TOP level: the retry block defines `incumbent_score` only when it runs.
margin_startup = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                        c1 = c1_startup, prn = false).margin

if opt_r_on && !isnan(c1_startup)
    if margin_startup < tos.min_feasibility_margin
        # All three startup gates, so a widening retry cannot trade clearance for turn margin unnoticed.
        el_floor_start = fcs.min_elevation + tos.candidate_elevation_margin
        score_installed() = begin
            margin = check_pattern_feasible(fec, l_tether, fcs.max_steering;
                                            c1 = c1_startup, prn = false).margin
            el_ok = minimum(fec.el_path) >= el_floor_start
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
        # Defined BEFORE the loop (soft scope); saves rejected curves for examples/plot_trajectory.jl.
        save_failed_trajectory(name, az, el; margin = NaN, power = NaN) = begin
            dir = joinpath(@__DIR__, "..", "trajectories")
            mkpath(dir)
            stamp = replace(string(now()), r"[:.]" => "", "T" => "_")[1:15]
            file = joinpath(dir, "$(name)_$stamp.yaml")
            YAML.write_file(file, Dict(
                "name" => name,
                "date" => string(now()),
                "l_tether" => l_tether,
                "min_feasibility_margin" => tos.min_feasibility_margin,
                "margin" => margin,
                "predicted_power_W" => power,
                "azimuth_deg" => collect(Float64.(az)),
                "elevation_deg" => collect(Float64.(el)),
            ))
            @info "Saved failed trajectory to $file (margin $margin)."
        end
        incumbent_score = score_installed()
        inc_result, inc_table, inc_raw = opt_result, opt_table, opt_paths_raw[1]
        r_asked = NaN                        # turn radius whose reply was asked last
        m_reply = incumbent_score.margin     # measured margin of that reply
        bisect_hi = NaN                      # narrowest ask KNOWN to 422; NaN = none yet
        cap_ok = nothing                     # elevation ceiling of the last CONVERGED ask
        cap_bad = NaN                        # highest ceiling KNOWN to 422; NaN = none yet
        relax_cap = false                    # retry under cap_ok instead of the ratchet
        width_ok = nothing                   # azimuth half-width floor of the last CONVERGED ask
        width_bad = NaN                      # lowest width floor KNOWN to 422; NaN = none yet
        relax_width = false                  # no width step until the next converged solve
        t_retries = time()
        for attempt in 1:max(Int(tos.startup_retries_max), 0)
            # Script globals written inside the loop; without the declaration each becomes a fresh local.
            global opt_result, opt_table, opt_downloops, opt_power_pred
            global opt_paths_raw, opt_paths_at, opt_r_scale, opt_r_min
            global incumbent_score, inc_result, inc_table, inc_raw
            global r_asked, m_reply, bisect_hi, cap_ok, cap_bad, relax_cap
            global width_ok, width_bad, relax_width
            target = max(tos.startup_retry_step * m_reply,
                         tos.startup_retry_slack * tos.min_feasibility_margin)
            # The last CONVERGED ask; before any retry converged (`r_asked` NaN) the
            # radius the startup solve was SENT. It must be one that converged, or the
            # bisection walks an interval with no solution at either end — `opt_r_min`
            # is re-measured off the reply by then and is NOT that number.
            prev_ask = isnan(r_asked) ? opt_r_sent : r_asked
            # The ceiling the ratchet would send next, clamped so the incumbent still fits above the box floor.
            inc_top = maximum(inc_raw[2])
            inc_height = inc_top - minimum(inc_raw[2])
            el_min_box = something(isnothing(opt_box) ? nothing :
                                   opt_box.elevation_min, el_floor_start)
            cap_from = isnothing(cap_ok) ? inc_top : min(inc_top, cap_ok)
            cap_next = max(cap_from - tos.startup_retry_el_cap_step,
                           el_min_box + inc_height + RETRY_CAP_SLACK)
            cap_room = tos.startup_retry_el_cap_step > 0 &&
                       cap_next <= cap_from - RETRY_CAP_MIN_STEP &&
                       (isnan(cap_bad) || cap_next > cap_bad)
            # The width floor a width step would send, in the server's RMS measure, never at a floor that 422'd.
            inc_amp = azimuth_amplitude(inc_raw[1])
            width_next = max(inc_amp, something(width_ok, 0.0)) +
                         tos.startup_retry_az_widen_step
            width_room = tos.startup_retry_az_widen_step > 0 &&
                         (isnan(width_bad) || width_next < width_bad)
            # What a bisection can still REACH: the radius lever is proportional (the
            # radius step scales the ask by target/measured), so no radius below the
            # 422'd `bisect_hi` beats `m_reply * bisect_hi / prev_ask`. Once that
            # ceiling is under the gate, bisecting only walks back to the incumbent's
            # own margin and the attempts belong to the geometry levers instead.
            bisect_room = !isnan(bisect_hi) &&
                          m_reply * bisect_hi / prev_ask >= tos.min_feasibility_margin
            # One lever per attempt; every rung carries the last converged ceiling and width floor.
            az_min = width_ok
            if bisect_room
                # A radius ask 422'd: bisect toward the last converged one instead of repeating it.
                lever = "radius bisection"
                r_ask = (prev_ask + bisect_hi) / 2
                el_cap = cap_ok
            elseif isnan(bisect_hi) && isnan(r_asked)
                # Attempt 1 corrects the ASSUMED lap reel-out to the measured ratio, capping the elevation if there is room.
                lever = "radius correction"
                r_ask = min_turn_radius_request(fcs, tos; scale = opt_r_scale,
                                                margin = target, c1 = c1_startup)
                el_cap = cap_room ? cap_next : cap_ok
            elseif !relax_cap && cap_room
                lever = "ceiling step"
                r_ask = prev_ask
                el_cap = cap_next
            elseif !relax_width && width_room
                lever = "width step"
                r_ask = prev_ask
                el_cap = cap_ok
                az_min = width_next
            elseif isnan(bisect_hi)
                # Scale the previous REQUEST by target/measured, clamped to RETRY_GAIN_MAX per converged solve.
                lever = "radius step"
                r_ask = prev_ask * clamp(target / m_reply, 1.0, RETRY_GAIN_MAX)
                el_cap = cap_ok
            else
                @info @sprintf("Startup retries stop at %d/%d: no radius under the \
                                %.2f m that 422'd can reach past margin %.3f (the \
                                gate wants %.2f), and the ceiling and width levers \
                                are spent.",
                               attempt, Int(tos.startup_retries_max), bisect_hi,
                               m_reply * bisect_hi / prev_ask,
                               tos.min_feasibility_margin)
                break
            end
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
                           incumbent_score.margin, tos.min_feasibility_margin,
                           attempt, Int(tos.startup_retries_max), lever, l_set,
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
                           isnan(bisect_hi) ? "" :
                               @sprintf(" (%s the %.2f m that 422'd)",
                                        bisect_room ? "bisecting below" :
                                            "the radius lever is spent under",
                                        bisect_hi))
            t_attempt = time()
            local att_result, att_table, att_raw, att_score
            try
                att_result = chain_step(opt_chain,
                                        StepParams(; length = opt_length(l_set),
                                                   winch_params = winch_first_lap,
                                                   min_turn_radius = r_ask,
                                                   pattern_limits = box_ask))
                att_table = chain_trajectory(opt_chain)
                att_raw = install_optimized_path!(att_result)
                att_score = score_installed()
            catch exc
                exc isa HTTP.StatusError && exc.status == 422 || rethrow()
                if !isequal(el_cap, cap_ok)
                    # The ceiling moved and failed: never send it (or lower) again; the radius steps next.
                    relax_cap = true
                    isnothing(el_cap) ||
                        (cap_bad = isnan(cap_bad) ? el_cap : max(cap_bad, el_cap))
                    @info @sprintf("Startup retry %d (%s) could not converge (HTTP \
                                    422) at %.2f m under a ceiling of %s; the \
                                    ceiling lever is spent, %s under the last \
                                    converged ceiling (%s).",
                                   attempt, lever, r_ask,
                                   isnothing(el_cap) ? "none" : @sprintf("%.1f°", el_cap),
                                   isnan(r_asked) ? "re-asking the same radius" :
                                                    "the radius steps next",
                                   isnothing(cap_ok) ? "none" : @sprintf("%.1f°", cap_ok))
                elseif !isequal(az_min, width_ok)
                    # Only the width floor moved and failed: never ask for it (or wider) again.
                    relax_width = true
                    width_bad = isnan(width_bad) ? az_min : min(width_bad, az_min)
                    @info @sprintf("Startup retry %d (%s) could not converge (HTTP \
                                    422) at %.2f m with an azimuth half-width >= \
                                    %.1f°; the width lever is spent, the radius \
                                    steps next at the last converged floor (%s).",
                                   attempt, lever, r_ask, az_min,
                                   isnothing(width_ok) ? "none" :
                                       @sprintf("%.1f°", width_ok))
                else
                    bisect_hi = r_ask
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
                          att_score.margin > incumbent_score.margin)
            if !takes_over && att_score.margin > incumbent_score.margin &&
               (!att_score.el_ok || !att_score.clr_ok)
                install_optimized_path!(inc_result)     # incumbent stays flown
                @warn @sprintf("Startup retry %d reached margin %.3f but dropped \
                                below a floor (clearance %s, elevation %s); wider \
                                cannot recover clearance — retries stop here.",
                               attempt, att_score.margin,
                               att_score.clr_ok ? "ok" : "MISSED",
                               att_score.el_ok ? "ok" : "MISSED")
                break
            elseif takes_over
                record_opt_success!(opt_chain)
                incumbent_score = att_score
                inc_result, inc_table, inc_raw = att_result, att_table, att_raw
                apply_optimized_kv!(inc_table, 0.0, l_set)
                # Only adoption moves these: a discarded retry leaves the `opt_*` state untouched.
                opt_result = inc_result
                opt_table = inc_table
                opt_downloops = inc_table["spline"]["downloops"]
                opt_power_pred = Float64(inc_table["metrics"]["avg_power_W"])
                opt_paths_raw = [inc_raw]
                opt_paths_at = [(0.0, 0)]
                opt_r_scale = reelout_anchor_ratio(inc_table) *
                              tos.turn_radius_headroom
                opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale,
                                                    c1 = c1_startup)
                if att_score.ok
                    @info @sprintf("Startup path clears the gates at margin %.3f \
                                    after %d solves (%.1f s of wall time).",
                                   incumbent_score.margin, attempt, time() - t_retries)
                    break
                end
                @info "Kept retry $attempt as the best-so-far; trying again."
            else
                install_optimized_path!(inc_result)     # put the incumbent back
                save_failed_trajectory("startup_retry$attempt", att_raw[1],
                                       att_raw[2]; margin = att_score.margin,
                                       power = Float64(att_table["metrics"]["avg_power_W"]))
                @info @sprintf("Startup retry %d gave margin %.3f, no better than \
                                %.3f — keeping the incumbent.",
                               attempt, att_score.margin, incumbent_score.margin)
            end
            r_asked, m_reply = r_ask, att_score.margin
            bisect_hi = NaN   # this ask converged, so it is no longer an upper bound
            cap_ok, relax_cap = el_cap, false
            width_ok, relax_width = az_min, false
        end
    end
end

if margin_startup < tos.min_feasibility_margin
    # The incumbent is what the gates will refuse; `incumbent_score` exists exactly when this fires.
    save_failed_trajectory("startup_incumbent", inc_raw[1], inc_raw[2];
                           margin = incumbent_score.margin,
                           power = opt_power_pred)
end

if !isnothing(opt_result.depower)
    global depower_flown_opt = awetrim_depower_to_v3kite(opt_result.depower.value)
    push!(opt_depower_log,
          (; t = 0.0, l_dp = opt_result.depower.value, u_p_equiv = depower_flown_opt))
end

# The pattern's own geometry, captured now: with reopt_enabled `fec` holds another path at the end.
n_path_initial = length(fec.az_path)
path_min_h_start = path_min_height(fec, l_tether)
az_c_path = 0.5 * (maximum(fec.az_path) + minimum(fec.az_path))
el_c_path = 0.5 * (maximum(fec.el_path) + minimum(fec.el_path))
az_amp_path = 0.5 * (maximum(fec.az_path) - minimum(fec.az_path))
el_height_path = maximum(fec.el_path) - minimum(fec.el_path)

# Which path was flown when, so the run is scored against the prediction of the path in the air.
pred_timeline = [(t = 0.0, power = opt_power_pred)]
opt_downloops == !fcs.up_loops ||
    error("The optimizer returned a downloops = $opt_downloops path while this run \
           flies up_loops = $(fcs.up_loops). Change fcs.up_loops or the guess; do \
           not fly it reversed.")
@info @sprintf("Optimized path: %d points, azimuth %.1f°…%.1f°, elevation \
                %.1f°…%.1f° (centre %.1f°), predicted mean reel-out power %.0f W.",
               length(fec.az_path), minimum(fec.az_path), maximum(fec.az_path),
               minimum(fec.el_path), maximum(fec.el_path), el_c_path, opt_power_pred)

# The three gates on the installed path; defines `el_floor`, `c1_at` and `phase5_margin` for the loop.
include(joinpath(@__DIR__, "reelout_feasibility.jl"))

@info @sprintf("Elevation lift: el_offset_final = %+.2f°, el_offset_lead = %.1f s \
                (%s), reelout_softstop = %.1f s.",
               fcs.el_offset_final, fcs.el_offset_lead,
               fcs.el_offset_lead > 0 ? "anticipates the end of reel-out" :
                                        "starts at the stop latch / phase 5",
               fcs.reelout_softstop)

# The dive aims at the pattern centre, which is the OPTIMIZED path's now.
ccs = CourseControllerSettings(fcs; dt = s.dt)
ccs.el_center = el_c_path
cc = CourseController(ccs)

transition_start = NaN            # [s] time phase 3 began; `reelout_delay` counts from it
stop_start = NaN            # [s] time the soft-stop deceleration latched; NaN = not yet
stop_v_entry = NaN          # [m/s] v_set at the moment it latched
stop_dp_entry = NaN         # [-] rel_depower at the moment it latched
stop_T = NaN                # [s] duration of the linear decel to reach 0 at reelout_l_max
ff_log = Float64[]          # [-] feed-forward steering per step, for the analysis after the run
ff_chi_log = Float64[]      # [rad] chord correction per step
ff_u_filt = 0.0             # [-] low-passed feed-forward steering
ff_chi_filt = 0.0           # [rad] low-passed chord correction
dp_final_extra = 0.0        # [-] phase-5 force limiter's depower above depower_final
dp_final_extra_peak = 0.0   # [-] the most it asked for, for the summary
rel_depower_prev = fcs.depower_setpoint  # [-] depower commanded last step; the gain reads c1 there
reelout_started = false    # true once the gate below has opened; LATCHED, never re-closes
reelout_start_t = NaN       # [s] time it opened; the soft-start ramp counts from here
reelout_trigger_fired = false # true if the FORCE trigger opened it, not the timer
reelout_done = false        # true once either stop criterion has ended reel-out
stop_reason = ""            # "length", "laps", or "" if reel-out never stopped
final_start = NaN           # [s] time phase 5 began; the run ends `fcs.final_time` after it
e_mech = 0.0                # [Wh] running mechanical energy, logged for the viewer

# fig_8 live lap count: 0 before phase 4, 1 at first entry, +1 per traversal; the post-run `fig8` is another thing.
fig8_n = 0
fig8_idx_prev = fec.last_idx
fig8_idx_progress = 0.0
n_path = length(fec.az_path)
# The reference TRACKING is scored against: the optimizer's curve, canonicalized and blended like the flown one, never lifted.
raw_az, raw_el = prepare_path(opt_paths_raw[1]...;
                              resample = min(tos.resample_points, length(opt_paths_raw[1][1]) - 1),
                              up_loops = fcs.up_loops)
length(raw_az) == n_path ||
    error("scored reference has $(length(raw_az)) points, the flown path $n_path")
# Resolution the path in the air is worth checking at (the reply's own, not `n_path`); updated per install.
chk_points = n_path

# Elevation lift: `el_offset_final` once it latches, delivered through an install or an in-air blend.
el_applied = 0.0            # [deg] lift the path in the air actually carries
lift_on = false             # `el_offset_final` latched in; never cleared once set
el_shift_events = NamedTuple[]  # in-air shift attempts, one entry per outcome CHANGE
lift_t = NaN                # [s] when it latched; NaN = never
lift_remaining = NaN        # [m] of reel-out left at that moment
el_shift_warned = false     # a held-back shift warns once; re-armed by the next delivery
# The lap and target of the last in-air shift attempt: one attempt per lap and per target.
el_shift_lap = 0
el_shift_target = NaN

# The pattern the kite is ASKED to fly, per step; the size criteria are scored lap by lap against it.
geom_t = Float64[]
geom_az_c = Float64[]
geom_az_amp = Float64[]
geom_el_h = Float64[]
# Cross-track error to the scored reference `raw_az`/`raw_el`, per step, on the same clock.
geom_d_raw = Float64[]

# Where in the pattern the kite ends up low, binned on |azimuth| as a fraction of the path's own amplitude.
n_droop_bins = 5
droop_n = zeros(Int, n_droop_bins)
droop_flown = zeros(n_droop_bins)   # [deg] kite below the path's elevation centre
droop_ref = zeros(n_droop_bins)     # [-] depth of the path at Q, in half-spans
droop_sag = zeros(n_droop_bins)     # [deg] kite below the path at Q

# ---- Re-optimization (stage 4): re-anchor the path to the flown length; `reopt_blocking` freezes the loop meanwhile.
reopt_pending = false       # a solve is queued on the server
reopt_n = 0                 # solves completed, accepted or rejected
reopt_lap = 0.0             # lap count at which the last request went out
reopt_next_poll = 0.0       # [s] next /status poll
reopt_t_request = NaN       # [s] when the pending request went out
reopt_blocked_s = 0.0       # [s] wall time spent frozen waiting for a reply
reopt_last_solve_s = NaN    # [s] wall time the last blocking wait took
reopt_events = NamedTuple[] # one row per solve, for the run summary
# Wall clock of the current cycle's FIRST request, and one row per cycle with the time to the verdict.
reopt_t_wall_request = NaN  # [s] time() when the cycle's first request went out
reopt_cycles = NamedTuple[] # (; t, l, status, wall_s) per completed cycle
blend_retries_total = 0     # cold-restart attempts spent on a rejected reply
# [deg] shortfall of the last reply gated out for clearance or elevation; carried across cycles.
el_min_extra = 0.0
# The blend in progress; both endpoints are fold-free across w in [0, 1] (the accept gate's fold check).
blend_from = nothing
blend_to = nothing
blend_t0 = NaN
# The scored reference's endpoints of the SAME blend, set only by a reopt install.
raw_from = nothing
raw_to = nothing
# Same mechanism, scalar, for the optimizer's rel_depower override.
depower_flown = depower_flown_opt    # current blended output
depower_blend_from = depower_flown
depower_blend_to = nothing
depower_blend_t0 = NaN

"""
    blend_folds(az0, el0, az1, el1) -> Bool

Does `blend_paths` between these two closed curves collapse `path_min_radius`
anywhere across `w` in `[0, 1]`, relative to the smaller of the two endpoints'
own radius? Sampled at `tos.blend_probe_points` points; a fold shows up as a
near-zero radius against endpoints that are not, so a coarse sweep catches it —
see the tuning log entry on why this replaced a runtime hold/jump-cap instead.
"""
function blend_folds(az0, el0, az1, el1)
    r0 = min(path_min_radius(az0, el0), path_min_radius(az1, el1))
    r0 <= 0 && return false   # degenerate endpoint; not this check's job
    any(w -> path_min_radius(blend_paths(az0, el0, az1, el1, w)...) <
             tos.blend_fold_margin * r0,
        range(0.0, 1.0; length = tos.blend_probe_points))
end

"""
    retried(n) -> String

How a rejection reports the retries that preceded it, `""` for the first try.
"""
retried(n) = n > 0 ? @sprintf(", after %d retries", n) : ""

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

# Assigned OUTSIDE the try: the loop's wall time must survive an early break.
t_wall_start = time()
try
    for _ in 1:s.steps
        t = s.sys_state.time
        t - final_start >= fcs.final_time && break

        # L0 attractor guidance -> commanded course [rad]; the lead is re-read every step.
        fec.fes.attractor_distance = attractor_distance(fcs, Float64(s.sys_state.v_app),
                                                        Float64(s.sys_state.l_tether[1]))
        chi_set, az_attr, el_attr, dmin =
            navigate_fig8(fec, Float64(s.sys_state.azimuth),
                          Float64(s.sys_state.elevation))

        # Entry state machine, descent limiter, feedback fusion, PID and rel_depower: see CourseController.
        heading = Float64(s.sys_state.heading)
        local v_kite = norm(s.sys_state.vel_kite)
        phase_before = cc.phase
        # Loop gain is heading_p * c1, so every phase flies heading_p * c1(setpoint)/c1(u_d), u_d rounded for the memo.
        local gain_scale = 1.0
        if isfinite(c1_setpoint)
            local dp_prev = round(isfinite(c1_depower_max) ?
                                  min(rel_depower_prev, c1_depower_max) : rel_depower_prev;
                                  digits = 3)
            local c1_now = c1_at_depower(dp_prev)
            isfinite(c1_now) && c1_now > 0 && (gain_scale = c1_setpoint / c1_now)
        end
        # Curvature feed-forward plus chord correction, low-passed over ff_tau; see FC_Settings.ff_gain.
        local u_ff = 0.0
        local chi_ff = 0.0
        if fcs.ff_gain > 0 && cc.phase >= 4
            local c1_ff = c1_setpoint / gain_scale     # c1 at the flown depower
            local v_app_ff = max(Float64(s.sys_state.v_app), fcs.v_app_min)
            local speed_ff = rad2deg(v_kite / Float64(s.sys_state.l_tether[1]))  # [deg/s]
            if isfinite(c1_ff) && c1_ff > 0 && speed_ff > 0
                local psi_dot_ff = path_turn_rate(fec, fcs.ff_lead_time * speed_ff, speed_ff;
                                                  smooth = fcs.ff_smooth)
                # Faded out when the kite is not on this branch (a Q swap hands it the other lobe's curvature).
                local fade_d = clamp((fcs.ff_d_fade - dmin) / (0.5 * fcs.ff_d_fade), 0.0, 1.0)
                local fade_e = clamp((deg2rad(fcs.ff_err_fade) - abs(cc.err)) /
                                     (0.5 * deg2rad(fcs.ff_err_fade)), 0.0, 1.0)
                local g_ff = fcs.ff_gain * fade_d * fade_e
                local alpha_ff = fcs.ff_tau > 0 ? dt0 / (dt0 + fcs.ff_tau) : 1.0
                global ff_u_filt += alpha_ff * (g_ff * psi_dot_ff / (c1_ff * v_app_ff) - ff_u_filt)
                global ff_chi_filt += alpha_ff * (g_ff * path_chord_offset(fec) - ff_chi_filt)
                u_ff = ff_u_filt
                chi_ff = ff_chi_filt
            end
        end
        push!(ff_log, u_ff)
        push!(ff_chi_log, chi_ff)
        local rel_steering, rel_depower, phase = calc_steering(cc, chi_set, heading,
            Float64(s.sys_state.course);
            t, elevation = Float64(s.sys_state.elevation),
            v_kite, v_app = Float64(s.sys_state.v_app),
            dmin, tangent = path_tangent(fec), gain_scale, u_ff, chi_ff)
        phase_before == 2 && phase == 3 && (global transition_start = t)
        # The optimizer's depower from phase 3 on, ramped over path_blend_time; phase 5 below still wins.
        if tos.fly_opt_depower && phase in (3, 4)
            # The entry ladder's depower is the FROM endpoint the first time, so the 2->3 hand-over ramps too.
            if phase_before < 3 && isnothing(depower_blend_to)
                global depower_blend_from = rel_depower
                global depower_blend_to = depower_flown_opt
                global depower_blend_t0 = t
            end
            local w_dp = isnothing(depower_blend_to) ? 1.0 :
                clamp((t - depower_blend_t0) / tos.path_blend_time, 0.0, 1.0)
            global depower_flown = isnothing(depower_blend_to) ? depower_flown_opt :
                (1 - w_dp) * depower_blend_from + w_dp * depower_blend_to
            w_dp >= 1.0 && (global depower_blend_to = nothing)
            rel_depower = depower_flown
        end
        # Separate from calc_steering's ladder so it can fire the SAME step as a 3->4 transition.
        if phase in (3, 4) && reelout_done
            set_phase!(cc, 5)
            phase = 5
            isnan(final_start) && (global final_start = t)
        end
        # Ramps depower toward depower_final with the soft-stop, never BELOW the depower the stop latched at.
        if !isnan(stop_start)
            dp_stop_target = max(fcs.depower_final, stop_dp_entry)
            rel_depower = stop_dp_entry +
                (dp_stop_target - stop_dp_entry) * clamp((t - stop_start) / stop_T, 0.0, 1.0)
        elseif phase == 5
            rel_depower = fcs.depower_final
        end
        # Force limiter from the STOP LATCH on: integrates on the force the stopped drum is about to see.
        if fcs.depower_final_max > fcs.depower_final && (phase == 5 || !isnan(stop_start))
            f_now = winch_force(s)
            v_app_now = Float64(s.sys_state.v_app)
            v_ro_now = max(Float64(s.sys_state.v_reelout[1]), 0.0)
            f_stopped = v_app_now > 0 ? f_now * ((v_app_now + v_ro_now) / v_app_now)^2 : f_now
            ramping = !isnan(stop_start) && t - stop_start < stop_T
            f_gain = ramping ? fcs.depower_final_f_gain_stop : fcs.depower_final_f_gain
            global dp_final_extra = clamp(dp_final_extra + f_gain *
                                          (f_stopped - fcs.depower_final_f_target) * s.dt,
                                          0.0, fcs.depower_final_max - fcs.depower_final)
            rel_depower = min(rel_depower + dp_final_extra, fcs.depower_final_max)
            dp_final_extra > dp_final_extra_peak && (global dp_final_extra_peak = dp_final_extra)
        end
        chi_cmd = cc.chi_cmd
        w_lim = cc.w_lim
        w_course = cc.w_course
        err = cc.err

        # Elevation shift target: `el_offset_final`, latched at the stop latch (or phase 5).
        if !lift_on && phase >= 4
            v_ro_now = Float64(s.sys_state.v_reelout[1])
            if !isnan(stop_start) || phase >= 5 ||
               (fcs.el_offset_lead > 0 && v_ro_now > 0 &&
                fcs.reelout_l_max - l_set <= v_ro_now * fcs.el_offset_lead)
                global lift_on = true
                global lift_t = t
                global lift_remaining = fcs.reelout_l_max - l_set
                @info @sprintf("Elevation lift of %+.2f° starting at t = %.1f s \
                                (%.1f m of reel-out left, phase %d).",
                               fcs.el_offset_final, t, fcs.reelout_l_max - l_set, phase)
            end
        end
        el_target = lift_on ? fcs.el_offset_final : 0.0

        # fig_8: 1 the instant phase first reaches >= 4, then +1 per traversal, unwrapped across the `mod1` wrap.
        if phase >= 4
            if fig8_n == 0
                global fig8_n = 1
                global fig8_idx_prev = fec.last_idx
                if fcs.first_lap_force_frac < 1
                    rcs.f_high = F_HIGH_NOMINAL * fcs.first_lap_force_frac
                    global first_lap_f_high_applied = true
                    @info @sprintf("Lap 1: upper force limit held at %.0f N \
                                    (%.0f %% of %.0f N) for this lap.",
                                   rcs.f_high, 100 * fcs.first_lap_force_frac,
                                   F_HIGH_NOMINAL)
                end
            else
                delta = fec.last_idx - fig8_idx_prev
                delta < -(n_path ÷ 2) && (delta += n_path)
                delta > n_path ÷ 2 && (delta -= n_path)
                # A step moves Q by a fraction of a point; a jump is Q changing branch.
                abs(delta) > n_path ÷ 8 && (delta = 0)
                global fig8_idx_progress += delta
                global fig8_idx_prev = fec.last_idx
                # Never counted DOWN: Q can slip a fraction of a point backwards at an install.
                global fig8_n = max(fig8_n, 1 + floor(Int, fig8_idx_progress / n_path))
                # Lap 1 only: the upper force limit is held down; `F_HIGH_NOMINAL` goes back on lap 2.
                if fcs.first_lap_force_frac < 1 && fig8_n > 1 && first_lap_f_high_applied
                    rcs.f_high = F_HIGH_NOMINAL
                    global first_lap_f_high_applied = false
                    @info @sprintf("Lap %d: upper force limit back to %.0f N.",
                                   fig8_n, F_HIGH_NOMINAL)
                end
            end

            az_lo, az_hi = extrema(fec.az_path)
            el_lo, el_hi = extrema(fec.el_path)
            az_amp, el_half = 0.5 * (az_hi - az_lo), 0.5 * (el_hi - el_lo)
            el_kite = rad2deg(Float64(s.sys_state.elevation))
            if az_amp > 0 && el_half > 0
                el_c = 0.5 * (el_hi + el_lo)
                b = azimuth_bin(fec.az_path[fec.last_idx], az_lo, az_hi, n_droop_bins)
                droop_n[b] += 1
                droop_flown[b] += el_c - el_kite
                droop_ref[b] += (el_c - fec.el_path[fec.last_idx]) / el_half
                droop_sag[b] += el_kite - fec.el_path[fec.last_idx]
            end
        end

        # ---- Re-optimize the path for the length now being flown (phase 4 only) -------- #
        if tos.reopt_enabled && phase == 4
            l_now = Float64(s.sys_state.l_tether[1])

            # Queue on a lap boundary, never while a solve or a blend is running, never past max_reopt.
            if !reopt_pending && isnothing(blend_to) && reopt_n < tos.max_reopt &&
               fig8_idx_progress >= (reopt_lap + tos.reopt_every_n_laps) * n_path
                try
                    # Clocked from here, so a request that fails while being BUILT still has a start.
                    global reopt_t_wall_request = time()
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
                    opt_r_on && (global opt_r_min =
                        min_turn_radius_request(fcs, tos; scale = opt_r_scale,
                                                c1 = c1_at(phase)))
                    # The floor moves with the length: box rebuilt per request, `size_box_growth` x the previous install.
                    global opt_box_now = with_size_box(
                        pattern_limits_from(tos;
                            elevation_min = elevation_min_request(fcs, tos, l_now;
                                                                  extra = el_min_extra),
                            wind_speed = cap_wind),
                        opt_paths_raw[end]..., tos.size_box_growth)
                    for (attempt, el_seed) in enumerate(el_seeds)
                        if isnothing(el_seed)
                            # `min_turn_radius` is re-sent because it MOVES with the length; `nothing` means "keep".
                            chain_step(opt_chain,
                                       StepParams(; length = opt_length(l_now), winch_params = winch_reopt,
                                                  min_turn_radius = opt_r_min,
                                                  pattern_limits = opt_box_now);
                                       wait = false)
                        else
                            guess_az_r, guess_el_r =
                                figure_eight_path(tos.guess_a, tos.guess_b,
                                                  0.0, el_seed,
                                                  0.0, tos.guess_points)
                            reopt_params = InitParams(; name = tos.name, length = opt_length(l_now),
                                                      winch_params = winch_reopt,
                                                      inflow_conditions = inflow,
                                                      trajectory = Trajectory(collect(guess_az_r),
                                                                              collect(guess_el_r)),
                                                      input_depower = depower_seed(tos, inflow.wind_speed),
                                                      reg_weight = tos.reg_weight,
                                                      detect_simple_bounds = tos.detect_simple_bounds,
                                                      min_turn_radius = opt_r_min,
                                                      pattern_limits = opt_box_now)
                            # A known failure is served by `opt_chain`, not skipped here: skipping would leave
                            # the chain on the warm lineage, and every later step would miss the cache.
                            reopt_reply = chain_init(opt_chain, reopt_params)
                            chain_step(opt_chain,
                                       StepParams(opt_length(l_now), winch_reopt, reopt_reply.trajectory);
                                       wait = false)
                        end
                        global reopt_pending = true
                        global reopt_t_request = t
                        global reopt_lap = fig8_idx_progress / n_path
                        global reopt_next_poll = t + tos.reopt_poll_interval
                        @info @sprintf("Re-optimizing for L = %.0f m at t = %.1f s \
                                        (lap %.1f, request %d of %d, %s)%s%s.",
                                       l_now, t, reopt_lap, reopt_n + 1, tos.max_reopt,
                                       isnothing(el_seed) ? "warm start" :
                                           @sprintf("guess el %.0f°", el_seed),
                                       isnothing(opt_box_now) ? "" :
                                           @sprintf(", box |az| <= %s, el %s..%s, half-span <= %s",
                                                    isnothing(opt_box_now.azimuth_max) ? "-" :
                                                        @sprintf("%.1f°", opt_box_now.azimuth_max),
                                                    isnothing(opt_box_now.elevation_min) ? "-" :
                                                        @sprintf("%.1f°", opt_box_now.elevation_min),
                                                    isnothing(opt_box_now.elevation_max) ? "-" :
                                                        @sprintf("%.1f°", opt_box_now.elevation_max),
                                                    isnothing(opt_box_now.elevation_amplitude_max) ? "-" :
                                                        @sprintf("%.1f°", opt_box_now.elevation_amplitude_max)),
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
                        global reopt_last_solve_s = time() - t_block
                        global reopt_blocked_s += reopt_last_solve_s
                        # Collect on THIS step: the reply is already on the server.
                        global reopt_next_poll = t
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
                    global reopt_n += 1
                    push!(reopt_events, (; t, l = l_now, status = "request failed",
                                         detail = first(sprint(showerror, exc), 120)))
                    push!(reopt_cycles, (; t, l = l_now, status = "request failed",
                                         wall_s = time() - reopt_t_wall_request))
                    @warn "Re-optimization request failed; flying on with the \
                           current path." exception = exc
                end
            end

            # Collect: poll rather than block, and validate before installing.
            if reopt_pending && t >= reopt_next_poll
                global reopt_next_poll = t + tos.reopt_poll_interval
                local state = try
                    chain_status(opt_chain)["state"]
                catch exc
                    @warn "Could not reach the optimizer; will retry." exception = exc
                    "solving"
                end
                if state != "solving"
                    global reopt_pending = false
                    global reopt_n += 1
                    event = (; t, l = l_now, status = state, detail = "")
                    if state == "converged"
                        tab = chain_trajectory(opt_chain)
                        # k_v and input_depower are applied only in the accept gate below, from the `tab` that passes it.
                    # A reply whose blend folds is not flown: a fresh COLD reply is requested, `blend_max_retries` times at most.
                    reject_reason = ""
                    # A clearance/elevation rejection retries the FLOOR, not the guess; `el_min_extra` carries the shortfall.
                    reject_low = false
                    # Frozen for the retry chain; the first entry is the startup solve, which `min_power_frac_prev` skips.
                    prev_install_pred = length(pred_timeline) > 1 ?
                        pred_timeline[end].power : NaN
                    for blend_attempt in 0:tos.blend_max_retries
                        if blend_attempt > 0
                            global blend_retries_total += 1
                            # Alternating +/- `reopt_retry_el_offset`, never scaled UP by `blend_attempt`.
                            retry_el_seed = el_center_seed +
                                (reject_low || isodd(blend_attempt) ? 1 : -1) *
                                tos.reopt_retry_el_offset
                            retry_el_min = elevation_min_request(fcs, tos, l_now;
                                                                 extra = el_min_extra)
                            @info @sprintf("  ... candidate at L = %.0f m rejected \
                                            (%s); cold-restarting from guess el \
                                            %.0f°%s (retry %d of %d), holding the \
                                            simulation.",
                                           l_now, reject_reason, retry_el_seed,
                                           isnothing(retry_el_min) ? "" :
                                               @sprintf(", floor %.1f°%s", retry_el_min,
                                                        el_min_extra > 0 ?
                                                            @sprintf(" (+%.1f° for the \
                                                                      shortfall)",
                                                                     el_min_extra) : ""),
                                           blend_attempt, tos.blend_max_retries)
                            retry_az, retry_el = figure_eight_path(tos.guess_a,
                                tos.guess_b, 0.0,
                                retry_el_seed, 0.0, tos.guess_points)
                            retry_params = InitParams(; name = tos.name, length = opt_length(l_now),
                                winch_params = winch_reopt, inflow_conditions = inflow,
                                trajectory = Trajectory(collect(retry_az),
                                                        collect(retry_el)),
                                input_depower = depower_seed(tos, inflow.wind_speed),
                                reg_weight = tos.reg_weight,
                                detect_simple_bounds = tos.detect_simple_bounds,
                                min_turn_radius = opt_r_min,
                                pattern_limits = with_size_box(
                                    pattern_limits_from(tos;
                                        elevation_min = retry_el_min,
                                        wind_speed = cap_wind),
                                    opt_paths_raw[end]..., tos.size_box_growth))
                            retry_reply = chain_init(opt_chain, retry_params)
                            chain_step(opt_chain,
                                       StepParams(opt_length(l_now), winch_reopt, retry_reply.trajectory);
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
                            global reopt_blocked_s += time() - t_retry
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
                        # Re-measure the anchor SCALE off the reply; the radius itself is derived where the request goes out.
                        opt_r_on && (global opt_r_scale = reelout_anchor_ratio(tab) *
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
                        cand_c1 = c1_at(phase, tos.fly_opt_depower ?
                            awetrim_depower_to_v3kite(
                                Float64(tab["optimized_parameters"]["input_depower"])) :
                            fcs.depower_setpoint)
                        lifted(fw) = new_el .+ el_target .+ fw .* wing_delta
                        function lifted_margin(e)
                            isnan(feas.c1) && return Inf
                            a, b = prepare_path(new_az, e; resample = n_native,
                                                up_loops = fcs.up_loops)
                            check_pattern_feasible(a, b, l_now, fcs.max_steering;
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
                                           l_now, 100 * wing_frac, fcs.el_offset_wing)
                        # TWO resolutions: the CHECKS at the reply's own, what is FLOWN at `n_path` so the lap counter holds.
                        chk_az, chk_el = prepare_path(new_az, new_el;
                            resample = n_native, up_loops = fcs.up_loops)
                        global chk_points = n_native
                        cand_az, cand_el = prepare_path(new_az, new_el;
                            resample = n_path, up_loops = fcs.up_loops)
                        # Canonicalized like `cand_az`/`cand_el`, so `blend_folds` and `blend_paths` pair the same points.
                        cand_from = prepare_path(fec.az_path, fec.el_path;
                            resample = n_path, up_loops = fcs.up_loops)
                        # At the CURRENT length, which is what it will be flown at.
                        margin = isnan(feas.c1) ? Inf :
                            check_pattern_feasible(chk_az, chk_el, l_now,
                                fcs.max_steering; c1 = cand_c1, prn = false).margin
                        clearance = path_min_height(chk_az, chk_el, l_now)
                        # Gated against BOTH the startup prediction and the previous install's (`min_power_frac*`).
                        new_pred = Float64(tab["metrics"]["avg_power_W"])
                        cand_folds = blend_folds(cand_from..., cand_az, cand_el)
                        # Raw against raw: the reply's curve against the previous install's, before either carries a lift.
                        cand_size = pattern_size_growth(opt_paths_raw[end]..., cand_raw...)
                        if margin < tos.min_feasibility_margin
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("curvature margin %.2f%s",
                                                       margin,
                                                       isnothing(opt_r_reply) ? "" :
                                                           @sprintf(" (the optimizer \
                                                                     measured %.2f m at \
                                                                     r = %.0f-%.0f m, \
                                                                     asked for >= %.2f m)",
                                                                    opt_r_reply, r_span[1],
                                                                    r_span[2],
                                                                    something(opt_r_min, 0.0))))
                            break
                        elseif tos.min_height > 0 && clearance < tos.min_height
                            reason = @sprintf("clearance %.1f m", clearance)
                            # In DEGREES, the request's currency: how far the lowest point sits below the elevation demanded.
                            deficit = asind(min(1.0, tos.min_height / l_now)) -
                                      minimum(chk_el)
                            if tos.elevation_min_from_gates &&
                               blend_attempt < tos.blend_max_retries
                                global el_min_extra += deficit +
                                                       tos.elevation_min_retry_margin
                                reject_reason = reason
                                reject_low = true
                                continue   # re-asked at a raised floor, at the top
                            end
                            event = (; t, l = l_now, status = "rejected",
                                     detail = reason * retried(blend_attempt))
                            break
                        elseif minimum(chk_el) < el_floor
                            # The clearance floor does NOT imply this one: at 318 m, 50 m of height is 9° of elevation.
                            reason = @sprintf("descends to %.1f°, below \
                                               min_elevation + margin = %.1f°",
                                              minimum(chk_el), el_floor)
                            if tos.elevation_min_from_gates &&
                               blend_attempt < tos.blend_max_retries
                                global el_min_extra += el_floor - minimum(chk_el) +
                                                       tos.elevation_min_retry_margin
                                reject_reason = reason
                                reject_low = true
                                continue
                            end
                            event = (; t, l = l_now, status = "rejected",
                                     detail = reason * retried(blend_attempt))
                            break
                        elseif cand_folds ||
                               (!power_gate_off(new_pred) &&
                                (new_pred < tos.min_power_frac * opt_power_pred ||
                                 new_pred < tos.min_power_frac_prev * prev_install_pred))
                            reason = if cand_folds
                                "blend folds"
                            elseif new_pred < tos.min_power_frac * opt_power_pred
                                @sprintf("%.0f W predicted, below %.0f%% of the \
                                          startup prediction (%.0f W)",
                                         new_pred, 100 * tos.min_power_frac,
                                         opt_power_pred)
                            else
                                @sprintf("%.0f W predicted, below %.0f%% of the \
                                          previous install's (%.0f W)",
                                         new_pred, 100 * tos.min_power_frac_prev,
                                         prev_install_pred)
                            end
                            if blend_attempt < tos.blend_max_retries
                                reject_reason = reason
                                reject_low = false   # this one is not about height
                                continue   # a fresh reply is requested at the top
                            end
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("%s, after %d retries",
                                                       reason, tos.blend_max_retries))
                            break
                        elseif tos.max_size_growth > 0 && cand_size.growth > tos.max_size_growth
                            # The continuity gate: a reply from another basin passes every gate above BY BEING BIG.
                            reason = @sprintf("%.2fx the previous install's size \
                                               (azimuth half-width x%.2f, elevation \
                                               span x%.2f), above max_size_growth = %.2f",
                                              cand_size.growth, cand_size.az_ratio, cand_size.el_ratio,
                                              tos.max_size_growth)
                            if blend_attempt < tos.blend_max_retries
                                reject_reason = reason
                                reject_low = false
                                continue   # a fresh reply is requested at the top
                            end
                            event = (; t, l = l_now, status = "rejected",
                                     detail = @sprintf("%s, after %d retries",
                                                       reason, tos.blend_max_retries))
                            break
                        else
                            power_gate_off(new_pred) &&
                                @info @sprintf("  ... power gate bypassed at L = %.0f m \
                                                (%.0f W predicted, %.1f m/s < \
                                                power_gate_wind_min %.1f): installing anyway.",
                                               l_now, new_pred, project_set.v_wind,
                                               tos.power_gate_wind_min)
                            global blend_from = cand_from
                            global blend_to = (cand_az, cand_el)
                            # The scored reference follows the same ramp, unlifted curve to unlifted curve.
                            global raw_from = prepare_path(raw_az, raw_el;
                                resample = n_path, up_loops = fcs.up_loops)
                            global raw_to = prepare_path(cand_raw[1], cand_raw[2];
                                resample = n_path, up_loops = fcs.up_loops)
                            global raw_az, raw_el = raw_from
                            # Here, not before the gates: a REJECTED reply is not a path the kite ever flies.
                            push!(opt_paths_raw, cand_raw)
                            push!(opt_paths_at, (t, phase))
                            global blend_t0 = t
                            # k_v and input_depower move only for the `tab` that made it here.
                            let l_dp = Float64(tab["optimized_parameters"]["input_depower"])
                                global depower_flown_opt = awetrim_depower_to_v3kite(l_dp)
                                global depower_blend_from = depower_flown
                                global depower_blend_to = depower_flown_opt
                                global depower_blend_t0 = t
                                push!(opt_depower_log, (; t, l_dp, u_p_equiv = depower_flown_opt))
                            end
                            apply_optimized_kv!(tab, t, l_now)
                            record_opt_success!(opt_chain)
                            abs(el_target - el_applied) > 1e-6 &&
                                push!(el_shift_events,
                                      (; t, delta = el_target - el_applied, margin,
                                       status = "carried by an install"))
                            global el_applied = el_target
                            # Arm the in-air warning again: one warning per SHIFT, not per run.
                            global el_shift_warned = false
                            # Install the aligned OLD path (w = 0, new point indices) and re-base the lap counter on it.
                            set_path!(fec, blend_from[1], blend_from[2];
                                      up_loops = fcs.up_loops)
                            global fig8_idx_prev = fec.last_idx
                            # A no-op while paths are resampled to `n_path`; `fig8_idx_progress` counts POINTS.
                            n_path_new = length(fec.az_path)
                            global fig8_idx_progress *= n_path_new / n_path
                            global n_path = n_path_new
                            push!(pred_timeline, (t = t, power = new_pred))
                            global margin5.margin = phase5_margin(chk_az, chk_el)
                            # Not a rejection reason: said once, so a phase 5 flown on the clamp is not a surprise.
                            if !isnan(margin5.margin) &&
                               margin5.margin < tos.min_feasibility_margin && !margin5.warned
                                global margin5.warned = true
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
                            event = (; t, l = l_now, status = "installed",
                                     detail = @sprintf("margin %.2f%s%s, clearance %.1f m, \
                                                        size x%.2f, %.0f W predicted", margin,
                                                       wing_frac < 1 ?
                                                           @sprintf(" (lobe lift at %.0f %%)",
                                                                    100 * wing_frac) : "",
                                                       isnan(margin5.margin) ? "" :
                                                           @sprintf(" (phase 5: %.2f at %.0f m)",
                                                                    margin5.margin,
                                                                    fcs.reelout_l_max),
                                                       clearance, cand_size.growth, new_pred))
                            break
                        end
                    end
                        end
                    push!(reopt_events, event)
                    # Non-blocking: an upper bound on the solve, by at most one `reopt_poll_interval`.
                    push!(reopt_cycles, (; t, l = l_now, status = event.status,
                                         wall_s = time() - reopt_t_wall_request))
                    # Blocking collects on the SAME step as the request, so the wall time is the figure that counts.
                    @info @sprintf("Re-optimization %d: %s%s (%s).",
                                   reopt_n, event.status,
                                   isempty(event.detail) ? "" : " — " * event.detail,
                                   tos.reopt_blocking ?
                                       @sprintf("%.1f s of wall time, held", reopt_last_solve_s) :
                                       @sprintf("%.1f s of sim after the request",
                                                t - reopt_t_request))
                end
            end

            # Blend: `blend_to` is guaranteed fold-free across all of w by the accept gate, so a plain linear ramp.
            if !isnothing(blend_to)
                w = clamp((t - blend_t0) / tos.path_blend_time, 0.0, 1.0)
                b_az, b_el = blend_paths(blend_from[1], blend_from[2],
                                         blend_to[1], blend_to[2], w)
                set_path!(fec, b_az, b_el; up_loops = fcs.up_loops)
                if !isnothing(raw_to)
                    global raw_az, raw_el = blend_paths(raw_from[1], raw_from[2],
                                                        raw_to[1], raw_to[2], w)
                end
                if w >= 1
                    global blend_from = nothing
                    global blend_to = nothing
                    global raw_from = nothing
                    global raw_to = nothing
                end
            end
        end

        # ---- Deliver the elevation shift in the air, AFTER the re-optimizer, which has first claim on `blend_to` ---- #
        if phase >= 4
            # The shift reaches the kite at an install or, when none is due, as a blend onto the path in the air.
            el_delta = el_target - el_applied
            if abs(el_delta) > 1e-6 && isnothing(blend_to) && !reopt_pending &&
               !(fig8_n == el_shift_lap && el_target == el_shift_target)
                global el_shift_lap = fig8_n
                global el_shift_target = el_target
                # Scored at `chk_points`, the resolution the path in the air came at; only the CHECK is downsampled.
                chk_n = min(chk_points, length(fec.az_path) - 1)
                function shift_margin(e)
                    isnan(feas.c1) && return Inf
                    a, b = prepare_path(fec.az_path, e; resample = chk_n,
                                        up_loops = fcs.up_loops)
                    check_pattern_feasible(a, b, Float64(s.sys_state.l_tether[1]),
                        fcs.max_steering; c1 = c1_at(phase), prn = false).margin
                end
                # Rationed down to a quarter of the shift, never below.
                # A rung that clears the margin can still fold `blend_paths` in between, so that is checked too.
                hit = nothing
                margin = NaN
                for fm in (1.0, 0.75, 0.5, 0.25)
                    e = fec.el_path .+ fm * el_delta
                    m = shift_margin(e)
                    isnan(margin) && (margin = m)   # the WHOLE shift's margin, reported
                    if m >= tos.min_feasibility_margin &&
                       !blend_folds(fec.az_path, fec.el_path, fec.az_path, e)
                        hit = (fm, e, m)
                        break
                    end
                end
                # A rung that moves the path by no more than a hundredth of a degree is not a delivery.
                if !isnothing(hit) && abs(hit[1] * el_delta) <= 0.01
                    hit = nothing
                end
                if !isnothing(hit)
                    fm, shifted, hit_margin = hit
                    global blend_from = (copy(fec.az_path), copy(fec.el_path))
                    global blend_to = (copy(fec.az_path), shifted)
                    global blend_t0 = t
                    went_in = fm * el_delta
                    push!(el_shift_events, (; t, delta = went_in,
                                            margin = hit_margin,
                                            status = fm < 1 ?
                                                @sprintf("blended in (%.0f %%)", 100 * fm) :
                                                "blended in"))
                    global el_applied = el_applied + went_in
                    global el_shift_warned = false
                    fm < 1 &&
                        @info @sprintf("Elevation shift rationed to fit the curvature \
                                        gate: %.0f %% of %+.2f°; the rest is retried \
                                        next lap.", 100 * fm, el_delta)
                elseif !el_shift_warned
                    push!(el_shift_events, (; t, delta = el_delta,
                                            margin, status = "held back"))
                    global el_shift_warned = true
                    @warn @sprintf("Elevation shift of %+.2f° held back: the curvature \
                                    margin would be %.2f even rationed to a quarter. \
                                    Retrying as the tether grows.", el_delta, margin)
                end
            end
        end

        # REEL_OUT: `reelout_delay` seconds after phase 3, and only until l_set reaches reelout_l_max.
        local v_set = 0.0
        # The gate LATCHES: `reelout_f_trigger` opens it early, and once open it never re-closes.
        if phase >= 3 && !reelout_started
            by_timer = t - transition_start >= fcs.reelout_delay
            by_force = winch_force(s) >= fcs.reelout_f_trigger
            if by_timer || by_force
                global reelout_started = true
                global reelout_start_t = t
                global reelout_trigger_fired = by_force && !by_timer
                by_force && !by_timer &&
                    @info @sprintf("  ... reel-out released EARLY at t = %.1f s by \
                                    force %.0f N >= %.0f N (%.1f s before the \
                                    %.1f s delay would have).",
                                   t, winch_force(s), fcs.reelout_f_trigger,
                                   transition_start + fcs.reelout_delay - t,
                                   fcs.reelout_delay)
            end
        end
        if reelout_started && !reelout_done
            # The INSTANTANEOUS force: reeling out faster when the kite pulls harder is what regulates the force.
            v_raw = calc_v_set(rc, reel_out_speed(s), winch_force(s), rcs.f_low)
            # Ramps the COMMAND, not the law, from when the gate OPENED; `t_startup` does not do this.
            ramp = fcs.reelout_softstart > 0 ?
                clamp((t - reelout_start_t) / fcs.reelout_softstart, 0.0, 1.0) : 1.0
            # ...but released in proportion to tether load, so the soft-start never overrides the force limiter.
            force_release = clamp((winch_force(s) - rcs.f_low) /
                                  (rcs.f_high - rcs.f_low), 0.0, 1.0)
            v_cmd = max(ramp, force_release) * v_raw

            remaining = fcs.reelout_l_max - l_set
            # Soft-stop: latch once `reelout_softstop` seconds would cover the rest, then decelerate linearly to 0.
            if isnan(stop_start) && fcs.reelout_softstop > 0 && v_cmd > 0 &&
               remaining <= v_cmd * fcs.reelout_softstop
                global stop_start = t
                global stop_v_entry = v_cmd
                global stop_dp_entry = rel_depower
                global stop_T = 2 * remaining / v_cmd
            end
            # Second stop criterion: N COMPLETE laps by `fig8_idx_progress` (`fig8_n` reads 1 during the first lap).
            if isnan(stop_start) && fcs.n_fig_eight > 0 &&
               fig8_idx_progress >= fcs.n_fig_eight * n_path
                global stop_reason = "laps"
                if fcs.reelout_softstop > 0 && v_cmd > 0
                    global stop_start = t
                    global stop_v_entry = v_cmd
                    global stop_dp_entry = rel_depower
                    # No remaining distance to solve T from: same nominal duration instead.
                    global stop_T = 2 * fcs.reelout_softstop
                else
                    global reelout_done = true   # hard stop, as reelout_l_max does today
                end
            end
            v_set = isnan(stop_start) ? v_cmd :
                stop_v_entry * (1 - clamp((t - stop_start) / stop_T, 0.0, 1.0))
            global l_set = min(l_set + v_set * s.dt, fcs.reelout_l_max)
            on_timer(rc)
            if l_set >= fcs.reelout_l_max
                global reelout_done = true
                isempty(stop_reason) && (global stop_reason = "length")
            elseif !isnan(stop_start) && stop_reason == "laps" && t - stop_start >= stop_T
                global reelout_done = true   # the soft-stop ramp has run out
            end
        elseif phase < 3
            # Force floor BEFORE reel-out: `guard_lfc` (NOT `rc`, see above) stepped by hand through calc_v_set's setters.
            set_reset(guard_lfc, false)
            set_f_set(guard_lfc, fcs.entry_f_min)
            set_v_sw(guard_lfc, calc_vro(rcs, fcs.entry_f_min) * 1.05)
            set_v_act(guard_lfc, reel_out_speed(s))
            set_tracking(guard_lfc, 0.0)   # bumpless: l_set is otherwise flat here
            set_force(guard_lfc, winch_force(s))
            # Reel-IN only: before reel-out this guard exists to catch a force SAG, never to reel out.
            v_guard = min(get_v_set_out(guard_lfc), 0.0)
            on_timer(guard_lfc)
            if guard_lfc.active
                v_set = v_guard
                global l_set = l_set + v_set * s.dt
            end
        end

        if !isnothing(steer_disturbance)
            local du = steer_disturbance(t)
            rel_steering += du
            push!(dist_t, t); push!(dist_d, du); push!(dist_u, rel_steering)
        end
        # `v_ff = v_set` removes the position loop's 2 s lag; `acceleration_limit` is `rcs.max_acc`, not the plant's own.
        step!(s; rel_depower, rel_steering, vsm_interval = fcs.vsm_interval,
              set_torque = winch_torque!(wpc, s, l_set; v_ff = v_set,
                                         speed_limit = rcs.v_sat,
                                         acceleration_limit = rcs.max_acc))
        global rel_depower_prev = rel_depower

        # Report the overspeed rather than the opaque solver abort it causes later.
        if Float64(s.sys_state.v_app) > fcs.v_app_abort
            @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                             Stopping before the solver diverges.",
                            s.sys_state.time, s.sys_state.v_app, fcs.v_app_abort,
                            rad2deg(s.sys_state.elevation), rad2deg(s.sys_state.AoA))
            break
        end

        # After step!, which overwrites parts of sys_state.
        s.sys_state.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 transition, 4 fig8, 5 final
        s.sys_state.bearing = chi_cmd          # the course actually tracked
        s.sys_state.attractor .= (deg2rad(az_attr), deg2rad(el_attr))
        s.sys_state.var_01 = dmin              # cross-track error [deg]
        s.sys_state.var_02 = az_attr           # attractor azimuth [deg]
        s.sys_state.var_03 = el_attr           # attractor elevation [deg]
        s.sys_state.var_04 = el_c_path         # pattern-centre elevation [deg]
        s.sys_state.var_05 = chi_set           # RAW guidance course [rad]
        s.sys_state.var_06 = rad2deg(err)      # REGULATED error [deg]
        # A weight, not a flag: a step here means entry_d_blend is too narrow.
        s.sys_state.var_07 = abs(chi_set) > deg2rad(fcs.entry_chi_max) ? w_lim : 0.0
        s.sys_state.var_08 = w_course          # course/heading blend weight [-]
        # Whole wing; sys_state.AoA is the centre panel only, which a turn twists away from.
        s.sys_state.var_09 = rad2deg(span_mean_aoa(s.sys))
        az_lo_g, az_hi_g = extrema(fec.az_path)
        el_lo_g, el_hi_g = extrema(fec.el_path)
        push!(geom_t, t)
        push!(geom_az_c, 0.5 * (az_hi_g + az_lo_g))
        push!(geom_az_amp, 0.5 * (az_hi_g - az_lo_g))
        push!(geom_el_h, el_hi_g - el_lo_g)
        push!(geom_d_raw, path_distance(raw_az, raw_el,
                                        rad2deg(Float64(s.sys_state.azimuth)),
                                        rad2deg(Float64(s.sys_state.elevation))))
        s.sys_state.fig_8 = Int16(fig8_n)      # live lap count
        s.sys_state.var_10 = l_set             # tether length setpoint [m]
        s.sys_state.var_11 = v_set             # REEL_OUT speed setpoint [m/s]
        s.sys_state.var_12 = get_state(rc)     # WinchController state (0/1/2)
        s.sys_state.var_13 = get_f_err(rc)     # force error [N], NaN in speed control
        # Not filled anywhere in the model chain: without this the log and the viewer read 0.
        s.sys_state.v_wind_200m .= calc_wind_factor(s.am, 200.0) .* s.sys_state.v_wind_gnd
        # Same for e_mech, which KiteViewers prints in Wh: the running integral of the viewer's p_mech.
        global e_mech += s.sys_state.winch_force[1] * s.sys_state.v_reelout[1] *
                         s.dt / 3600
        s.sys_state.e_mech = e_mech
    end
catch exc
    # `exc`, not `e`: a stray global `e` in the REPL makes the catch binding warn.
    @error "Simulation stopped early at t≈$(round(s.sys_state.time, digits=2))s" exception=(exc, catch_backtrace())
end
# The loop ALONE: saving the log and scoring it below are not simulation.
t_wall = time() - t_wall_start
t_sim = Float64(s.sys_state.time)

@info "Save the log"
save_log(s.logger, log_name; path = output_path, colmeta = timestamp_colmeta())

# Scoring, summary, archive, plots and marker; wrapped so a throw still leaves a FAILED marker, then rethrown.
try
    include(joinpath(@__DIR__, "reelout_results.jl"))
catch exc
    write_run_done("FAILED"; err = exc)
    rethrow()
end
