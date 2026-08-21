"""
Fly the V3 kite along a path an EXTERNAL OPTIMIZER produced, at constant tether
length.

Same plant, entry and inner loop as `simple_fig8.jl` — read that file first, its
docstring covers the log slots, the parameters, the sign conventions and why the
pattern must be flown low and wide, all of which apply here unchanged. What
differs is where the reference path comes from: instead of the lemniscate
`fcs.f8_a`/`f8_b`/`f8_c`/`f8_d` describe, the AWETrim optimizer is asked for the
power-optimal reel-out path under THIS run's wind and winch, and `set_path!`
installs it. The lemniscate is still built — as the initial guess the optimizer
starts from, and as the reference the result is plotted against.

# What this stage does and does not test

The optimized path is a REEL-OUT path: the optimizer maximizes reel-out power
and assumes the winch law `v_set = kv*sqrt(force)` of `data/wc_settings.yaml`.
This script flies it at CONSTANT length, so the winch coupling is not exercised
and the power the optimizer predicts is not the power this run harvests. What it
does establish, for the price of one short run, is that the guidance tracks an
arbitrary closed curve as well as it tracks its own lemniscate — everything else
is built on that. `simple_reelout.jl`'s successor closes the loop
(PlanOptFig8.md, stage 3).

# The server

`ensure_server()` starts one (`bin/run_server start`) if none answers, so a run
does not fail because a terminal was closed; `base_url` and `autostart_server`
are in `data/traj_opt.yaml` with the rest of the optimizer's settings. The
request is built by
`inflow_from_settings(project_set)` and `winch_from_wc(wcs)`, from the same files
the plant is built from: a path optimized for another wind, or for a winch three
times softer than the real one, is not the path for this run.

# The initial guess is not a formality

The guess lemniscate seeds the solve and nothing else — it does not shape the
flown path — and it has its own `guess_*` fields in `data/traj_opt.yaml` rather
than reusing `fcs.f8_a`/`f8_b`/`el_center`, which size the pattern the OTHER runs
actually fly. It decides two things, both measured on 2026-08-18 at 150 m and
6 m/s:
**whether the solve converges at all** (the reel-out settings' 20°/11° at 18°
give an IPOPT failure, while 30°/12°, or the same eight centred at 26°,
converge), and **which optimum it converges to** (those three all reach 6080 W,
while the server's own parametric lemniscate guess converges to a different,
much flatter 1431 W solution). The problem is multi-modal, so a guess is a
choice about the answer, and this script therefore never retries with a
different one behind your back: a 422 stops the run and says what to try.

Two things are checked before anything is flown, both of which cost a run
otherwise. The optimizer knows nothing of the V3's turn-rate law, so
`check_pattern_feasible` is evaluated on the path it RETURNED; and a winch too
stiff to reel out at the optimum makes the problem infeasible, which arrives as
HTTP 422 and is translated into the speed ceiling `kv*sqrt(f_high)` that caused
it. The traversal direction is asserted too: `set_path!` reverses a path to match
`up_loops`, so a `downloops` mismatch would otherwise silently fly the optimizer's
curve backwards — the same curve, but not the one that was optimized.

# Geometry that no longer comes from FC_Settings

`fcs.f8_a`/`f8_b` and `fcs.el_center` describe the guess, not what is flown, so
the pattern's centre (the dive target, and `var_04`) and its extent (the size
criteria of `fig8_metrics`) are measured off the installed path instead.

Logs to `output/<log_file>.arrow` like `simple_fig8.jl`, and to the same name:
this is the same run under the same project, and the last one flown is the one
plotted. `REF_PATH` carries the flown curve to `simple_fig8_plots.jl`, which
would otherwise redraw the lemniscate as the reference.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using KiteUtils: wc_settings   # resolves the wc-settings file named in the project
using AtmosphericModels: calc_wind_factor
using LinearAlgebra: norm
using Statistics: mean
using Printf

@info "simple_opt_fig8.jl: flying an externally optimized path with the V3 kite."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
# V3Kite is torque-only; the winch loops are ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
# The optimizer client: opt_init/opt_step/opt_trajectory and ensure_server.
include(joinpath(@__DIR__, "awetrim_client.jl"))
# Read and cleared HERE, so a `SHOW_PLOTS = false` never survives into the next run.
show_plots = @isdefined(SHOW_PLOTS) ? SHOW_PLOTS : true
SHOW_PLOTS = true
# The reference curve simple_fig8_plots.jl draws; set below, once it is known.
REF_PATH = nothing
AERO_MODE = ContinuousAero() # ContinuousAero() or AeroDirect()
# Structural damping of the tether and bridle segments, as a ratio of their
# stiffness: unit_damping = ratio * unit_stiffness [s]. See the docstring above.
DAMPING_PER_STIFFNESS = 0.001
PROJECT = selected_project() # system_fig8_{150,200,300}m.yaml, set via select_project()
SIM_TIME = selected_sim_time() # seconds, or `nothing` for the project's own default
TURBULENCE = selected_turbulence() # level in [0, 1], or "default" for the settings YAML value
WIND_SPEED = selected_windspeed() # m/s, or `nothing` for the project's own v_wind
@info "simple_opt_fig8.jl: project = $PROJECT, sim_time = $(isnothing(SIM_TIME) ? "default" : "$SIM_TIME s"), \
       turbulence = $TURBULENCE, wind_speed = $(isnothing(WIND_SPEED) ? "default" : "$WIND_SPEED m/s")."
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
# The optimizer's own settings: server, initial guess, solver knobs. Separate
# from fcs on purpose — see the file's header and the docstring above.
tos = TrajOptSettings(traj_opt_settings_file(project))

project_set = Settings(project)
apply_windspeed_override!(project_set, WIND_SPEED)
l_tether = project_set.l_tether

# Log files are arrow files, named after the project's `log_file`, kept out of git.
output_path = normpath(joinpath(@__DIR__, "..", "output"))
mkpath(output_path)
log_name = basename(project_set.log_file)

# ======================== INIT =========================== #

# Decided BEFORE init: the warm-up must relax against the winch the loop commands.
fcs.compliance >= 0 || error("compliance must be >= 0, got $(fcs.compliance)")
# Both winch loops are the CALLER's now: V3Kite's `step!` takes a torque.
wcs = load_wc_settings(wc_settings(project); dt = 1 / project_set.sample_freq)
wpc = nothing
wfc = nothing
if fcs.compliance > 0
    # winch_force_gains returns plain numbers; the controller object is V3Kite's.
    wfc = WinchForceController(; winch_force_gains(fcs)...)
    @info @sprintf("Winch: FORCE mode at compliance = %.2f — len_kp %.0f N/m, \
                    damp %.0f N·s/m, tau %.1f s.",
                   fcs.compliance, wfc.len_kp, wfc.damp, wfc.force_tau)
else
    # Perfectly stiff: the position feed-forward cancels the measured load exactly.
    wpc = WinchPosController(wcs; dt = 1 / project_set.sample_freq)
    @info "Winch: POSITION mode at compliance = 0 — constant unstretched length."
end

# No dt: init takes it from the project's settings (sample_freq). sim_time falls
# back to the project's own value when SIM_TIME is `nothing` (the `default` choice).
# The wind speed comes from the same file unless WIND_SPEED overrides it above: it is
# a plant condition, and project_set.v_wind keeps the mean wind, the turbulent field
# and the optimizer's inflow_from_settings query (below) at the same speed.
# No cache_path either: V3Kite's default is where its own precompile workload
# compiled the model, and a different model binary costs 40 s of re-JIT in init.
s = init(project_set.v_wind, l_tether; body_start_damping = fcs.body_damping,
    body_sim_damping = 0.8 .* fcs.body_damping,
    damping_per_stiffness = DAMPING_PER_STIFFNESS,
    elevation = fcs.elevation, depower_setpoint = fcs.depower_setpoint,
    system_yaml = project, use_turbulence = TURBULENCE, aero_mode = AERO_MODE,
    sim_time = SIM_TIME, warmup_time = fcs.warmup_time,
    # The warm-up must relax against the winch the loop will command.
    warmup_torque = isnothing(wfc) ? (m, l) -> winch_torque!(wpc, m, l) :
                                     (m, l) -> winch_force_hold!(wfc, m, l))
@info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)

# Constant-length setpoint: the tether length after settling and warm-up.
l0 = s.sys_state.l_tether[1]

fec = FigureEightController(FigureEightSettings(;
    dt = s.dt, A = fcs.f8_a, B = fcs.f8_b, C = fcs.f8_c, D = fcs.f8_d,
    az_center = 0.0, el_center = fcs.el_center,
    attractor_distance = fcs.attractor_dist, up_loops = fcs.up_loops))

# ================= OPTIMIZED REFERENCE PATH ================== #

# The conditions of THIS run, read off the files the plant was built from.
inflow = inflow_from_settings(project_set)
winch = winch_from_wc(wcs)
@info @sprintf("Optimizer conditions: %.1f m/s at 6 m from %.0f°, profile_law %d, \
                z0 = %g m | winch kv = %.4f, i.e. %.1f m/s at f_high = %.0f N | \
                depower seed %.3f m%s.",
               inflow.wind_speed, inflow.wind_direction, inflow.profile_law, inflow.z0,
               winch.k_v, winch.k_v * sqrt(winch.f_max), winch.f_max,
               depower_seed(tos, inflow.wind_speed),
               inflow.wind_speed > tos.input_depower_wind_ref ?
                   @sprintf(" (%.2f + %.3f per m/s above %.1f m/s)", tos.input_depower,
                            tos.input_depower_per_wind, tos.input_depower_wind_ref) : "")

# The seed, from data/traj_opt.yaml and NOT from fcs.f8_*: it decides whether the
# solve converges and which optimum it converges to, while fcs.f8_* size the
# lemniscate the OTHER runs fly. `figure_eight_path` closes the curve itself
# (last point == first), which is the shape the server expects.
el_center_seed = guess_el_center_seed(tos, inflow.wind_speed)
guess_az, guess_el = figure_eight_path(tos.guess_a, tos.guess_b, tos.guess_c,
                                       tos.guess_d, 0.0, el_center_seed,
                                       0.0, tos.guess_points)
@info @sprintf("Initial guess: %.0f° x %.0f° at %.0f°, %d points.",
               tos.guess_a, tos.guess_b, el_center_seed, tos.guess_points)

# What the solve must respect, as opposed to what it is scored against
# afterwards: both are off unless data/traj_opt.yaml turns them on.
#
# The turn radius is asked for with `turn_radius_headroom` on top, because the
# optimizer measures `R = r/|kappa|` at each node's own radius — which grows
# through its lap's reel-out — while this run installs the reply as an (azimuth,
# elevation) curve and flies it at ONE length. There is only this one solve here,
# so the geometric part cannot be measured off a previous reply the way
# `simple_opt_reelout.jl` does it — it is assumed instead, from
# `turn_radius_lap_reelout_m` over this run's length.
opt_r_scale = (1 + tos.turn_radius_lap_reelout_m / l0) * tos.turn_radius_headroom
opt_r_min = min_turn_radius_request(fcs, tos; scale = opt_r_scale)
opt_box = pattern_limits_from(tos)
isnothing(opt_r_min) && isnothing(opt_box) ||
    @info @sprintf("Constraints sent with the request: min_turn_radius %s, \
                    pattern box %s.",
                   isnothing(opt_r_min) ? "unset" :
                       @sprintf("%.2f m (min_feasibility_margin %.2f x the kite's \
                                own, x %.3f for %.0f m of assumed reel-out per lap \
                                and %.2f of headroom)",
                                opt_r_min, tos.min_feasibility_margin, opt_r_scale,
                                tos.turn_radius_lap_reelout_m, tos.turn_radius_headroom),
                   isnothing(opt_box) ? "unset" : string(opt_box))

ensure_server(tos.base_url; autostart = tos.autostart_server)
opt_reply = opt_init(InitParams(; name = tos.name, length = l0,
                                winch_params = winch, inflow_conditions = inflow,
                                trajectory = Trajectory(collect(guess_az), collect(guess_el)),
                                input_depower = depower_seed(tos, inflow.wind_speed),
                                reg_weight = tos.reg_weight,
                                detect_simple_bounds = tos.detect_simple_bounds,
                                min_turn_radius = opt_r_min,
                                pattern_limits = opt_box);
                     url = tos.base_url)
# No automatic retry with a different guess, deliberately: a guess that merely
# converges is not the same answer, and picking one silently would hide which
# optimum was flown. See the "initial guess" section of the docstring.
opt_result = try
    opt_step(StepParams(l0, winch, opt_reply.trajectory); url = tos.base_url)
catch exc
    exc isa HTTP.StatusError && exc.status == 422 || rethrow()
    error("""
          The optimizer returned no path: $(String(copy(exc.response.body)))

          Three candidates, most likely first:
            * the INITIAL GUESS is too far from the optimum for IPOPT to reach \
              it. Here that is guess_a = $(tos.guess_a)°, guess_b = \
              $(tos.guess_b)°, guess_el_center = $(el_center_seed)° of \
              data/traj_opt.yaml, which seeds the request and nothing else — \
              widening or raising it changes the guess, not the flown path. \
              Measured at 150 m and 6 m/s: 20°/11° at 18° does not converge, \
              30°/12° and 20°/11°-at-26° do.
            * the winch is too stiff to reel out at the optimum: \
              kv*sqrt(f_high) = $(round(winch.k_v * sqrt(winch.f_max); digits = 1)) \
              m/s against $(inflow.wind_speed) m/s of wind at 6 m.
            * these conditions genuinely have no solution.

          `bin/run_server log` carries the solver's own output.""")
end
toc("Received the optimized path in: ")

# What the reply was optimized AT, which is not what this run flies. The server
# varies `l_dp` unless it is pinned (`DepowerSpec("fixed", ...)`), and its tape
# scale is 0.4 m off V3Kite's: `l_dp = 0.6 + 5*u_p` there, `0.2 + 5*u_p` here. So
# the gap below is a real difference between the model that predicted the power
# and the model that flies it, and it belongs next to the prediction.
if !isnothing(opt_result.depower)
    flown_l_dp = 0.6 + 5 * fcs.depower_setpoint
    @info @sprintf("Optimized at depower l_dp = %.3f m (mode %s), against %.3f m \
                    for the flown depower_setpoint = %.2f — a %+.3f m gap on the \
                    optimizer's tape scale.",
                   opt_result.depower.value, opt_result.depower.mode, flown_l_dp,
                   fcs.depower_setpoint, opt_result.depower.value - flown_l_dp)
end
# The optimizer's own curvature diagnostic, in metres and physical: comparable
# with the kite's 1/(c1*u_s) and with `min_feasibility_margin`, and read
# before this repo's angular gates ever see the path.
isnothing(opt_result.metrics.turn_radius_min_m) ||
    @info @sprintf("Tightest physical turn radius of the reply: %.2f m%s.",
                   opt_result.metrics.turn_radius_min_m,
                   isnothing(opt_r_min) ? "" :
                       @sprintf(" (asked for >= %.2f m)", opt_r_min))

# Resample, but NEVER upsample: the reply is a polyline, and interpolating extra
# points onto it concentrates each vertex's turn into one short segment, which
# makes path_radius_profile report a far tighter pattern than the curve is.
n_opt = length(opt_result.trajectory.azimuth) - 1
set_path!(fec, opt_result.trajectory.azimuth, opt_result.trajectory.elevation;
          resample = min(tos.resample_points, n_opt))

# The pattern's own geometry: fcs.f8_* and fcs.el_center describe the GUESS now.
az_c_path = 0.5 * (maximum(fec.az_path) + minimum(fec.az_path))
el_c_path = 0.5 * (maximum(fec.el_path) + minimum(fec.el_path))
az_amp_path = 0.5 * (maximum(fec.az_path) - minimum(fec.az_path))
el_height_path = maximum(fec.el_path) - minimum(fec.el_path)

# set_path! REVERSES a path that does not match up_loops, so a mismatch here is
# not caught by anything downstream: the kite would fly the optimizer's curve,
# but backwards, which is not the trajectory that was optimized.
opt_table = opt_trajectory(; url = tos.base_url)
opt_downloops = opt_table["spline"]["downloops"]
opt_downloops == !fcs.up_loops ||
    error("The optimizer returned a downloops = $opt_downloops path while this run \
           flies up_loops = $(fcs.up_loops). Change fcs.up_loops or the optimizer's \
           initial guess; do not fly it reversed.")
@info @sprintf("Optimized path: %d points, azimuth %.1f°…%.1f°, elevation \
                %.1f°…%.1f° (centre %.1f°), predicted mean power %.0f W.",
               length(fec.az_path), minimum(fec.az_path), maximum(fec.az_path),
               minimum(fec.el_path), maximum(fec.el_path), el_c_path,
               opt_table["metrics"]["avg_power_W"])

# The ELEVATION floor, which is not the clearance floor and does not follow from
# it: `min_height` is satisfied at ever lower elevations as the tether grows, so a
# path that clears 50 m at 350 m of tether is at 8.2°. This repo's own criterion is
# an angle, and `fig8_metrics` fails a run that breaks it.
#
# Checked with `candidate_elevation_margin` on top, because this compares the
# REFERENCE path while the criterion scores the FLOWN one, and the kite flies below
# its reference near the lobe tips.
el_floor = fcs.min_elevation + tos.candidate_elevation_margin
minimum(fec.el_path) >= el_floor ||
    error(@sprintf("The optimized path descends to %.1f°, below min_elevation \
                    %.1f° + candidate_elevation_margin %.1f° = %.1f°. AWETrim \
                    constrains height and not elevation, so this is not something \
                    the solve avoids on its own.",
                   minimum(fec.el_path), fcs.min_elevation,
                   tos.candidate_elevation_margin, el_floor))

# The clearance floor, checked at the tether length this run flies. Independent of
# the turn-rate table above, so it runs whether or not `coeffs` was available.
if tos.min_height > 0
    clr = check_pattern_height(fec, l_tether, tos.min_height)
    clr.ok ||
        error(@sprintf("The optimized path's lowest point is %.1f m above ground at \
                        L = %.0f m (elevation %.1f°), below the min_height = %.0f m \
                        of data/traj_opt.yaml. The optimizer's own floor is a HEIGHT \
                        and it earns part of it by reeling out within the lap, which \
                        an installed (azimuth, elevation) curve does not. Raise the \
                        guess elevation, fly a longer tether, or lower min_height.",
                       clr.height, l_tether, clr.elevation, tos.min_height))
end

# Never hardcode these: both arguments move them a lot. The lookup key is
# `body_damping`, the value `init` was given: the damping the model FLIES the
# pattern with is the floor that decays out of it (0.8x by default), so the one
# value identifies both the settling transient and the flown damping c1 belongs
# to.
#
# The coefficients are DIAGNOSTIC here — they feed the feasibility check and the
# dead-time context below, no gain and no control law — so a damping/depower the
# table cannot serve costs the diagnosis, not the run. `turn_rate_coeffs` refuses
# to extrapolate (by design: c1 moves violently with both arguments), so catch
# that and fly on unadvised rather than aborting a deliberate off-grid run.
coeffs = try
    turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
catch e
    e isa ArgumentError || rethrow()
    @warn "No turn-rate coefficients for body_damping = $(fcs.body_damping), \
           depower = $(fcs.depower_setpoint) — flying WITHOUT the feasibility \
           check. Identify this cell with V3Kite.jl's steering_test_v3.jl to get \
           it back.\n$(e.msg)"
    nothing
end

if isnothing(coeffs)
    c1 = c2 = delay = NaN
else
    c1, c2, delay = coeffs.c1, coeffs.c2, coeffs.delay
    @info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                    c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
                   fcs.body_damping, fcs.depower_setpoint,
                   coeffs.interpolated ? " (INTERPOLATED)" : "",
                   c1, c2, delay)

    # On the OPTIMIZED path, at the length it is flown at, with the c1 of the
    # damping in use. The optimizer knows nothing of the V3's turn-rate law, so a
    # path the kite cannot turn along is a plausible thing for it to return.
    feas = check_pattern_feasible(fec, l_tether, fcs.max_steering; c1)
    feas.margin >= tos.min_feasibility_margin ||
        error(@sprintf("The optimized path asks for a turn radius of %.1f° where the \
                        kite manages %.1f° at L = %.0f m: margin %.2f, below \
                        min_feasibility_margin = %.2f. Lower body_damping, raise \
                        max_steering, or set min_feasibility_margin: 0.0 in \
                        data/traj_opt.yaml to fly it anyway.",
                       feas.path_radius, feas.kite_radius, l_tether, feas.margin,
                       tos.min_feasibility_margin))

    # Dead-time context for attractor_dist: how long the lead arc takes to fly.
    lead_time = deg2rad(fcs.attractor_dist) * l_tether / fcs.v_app_ref
    @info @sprintf("Attractor lead %.1f° ≈ %.1f s of flight at v_app %.1f m/s, \
                    vs %.2f s steering dead time (ratio %.1f).",
                   fcs.attractor_dist, lead_time, fcs.v_app_ref, delay, lead_time / delay)
end

# The dive aims at the pattern centre, which is the OPTIMIZED path's now.
ccs = CourseControllerSettings(fcs; dt = s.dt)
ccs.el_center = el_c_path
cc = CourseController(ccs)

# Live lap counter, logged to SysState's `fig_8`: 0 before the pattern is
# tracked, 1 at first entry into phase 4, then +1 each time `fec.last_idx` has
# advanced one full lemniscate (`n_path` points) since then — the guidance's
# own path parametrization, not a re-derived azimuth threshold. Refs, not plain
# variables: the top-level `for` below is a soft scope, so a plain variable
# reassigned only inside a conditional would rebind to a fresh, unassigned
# local every iteration instead of carrying its value forward.
fig8 = Ref(0)
idx_prev = Ref(fec.last_idx)
idx_progress = Ref(0.0)
n_path = length(fec.az_path)

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

# Assigned OUTSIDE the try: the loop's wall time must survive an early break.
t_wall_start = time()
try
    for _ in 1:s.steps
        t = s.sys_state.time

        # L0 attractor guidance -> commanded course [rad].
        chi_set, az_attr, el_attr, dmin =
            navigate_fig8(fec, Float64(s.sys_state.azimuth),
                          Float64(s.sys_state.elevation))

        # Entry state machine, descent limiter, open-loop entry override, feedback
        # fusion, PID and rel_depower: see CourseController.
        heading = Float64(s.sys_state.heading)
        local v_kite = norm(s.sys_state.vel_kite)
        local rel_steering, rel_depower, phase = calc_steering(cc, chi_set, heading,
            Float64(s.sys_state.course);
            t, elevation = Float64(s.sys_state.elevation),
            v_kite, v_app = Float64(s.sys_state.v_app),
            dmin, tangent = path_tangent(fec))
        chi_cmd = cc.chi_cmd
        w_lim = cc.w_lim
        w_course = cc.w_course
        err = cc.err

        # fig8: jumps to 1 the instant phase 4 is first reached, then +1 per
        # full traversal of the reference path, unwrapped so a single lap
        # never double-counts across the index's `mod1` wrap.
        if phase == 4
            if fig8[] == 0
                fig8[] = 1
                idx_prev[] = fec.last_idx
            else
                delta = fec.last_idx - idx_prev[]
                delta < -(n_path ÷ 2) && (delta += n_path)
                delta > n_path ÷ 2 && (delta -= n_path)
                idx_progress[] += delta
                idx_prev[] = fec.last_idx
                fig8[] = 1 + floor(Int, idx_progress[] / n_path)
            end
        end

        # Force mode reels out under load; compliance = 0 holds the length outright.
        if isnothing(wfc)
            step!(s; rel_depower, rel_steering,
                  set_torque = winch_torque!(wpc, s, l0),
                  vsm_interval = fcs.vsm_interval)
        else
            step!(s; rel_depower, rel_steering,
                  set_torque = winch_force_hold!(wfc, s, l0),
                  vsm_interval = fcs.vsm_interval)
        end

        # Report the overspeed rather than the opaque solver abort it causes later.
        if Float64(s.sys_state.v_app) > fcs.v_app_abort
            @error @sprintf("Overspeed at t=%.2fs: v_app=%.1f m/s > %.1f (elevation %.1f°, AoA %.1f°). \
                             Stopping before the solver diverges.",
                            s.sys_state.time, s.sys_state.v_app, fcs.v_app_abort,
                            rad2deg(s.sys_state.elevation), rad2deg(s.sys_state.AoA))
            break
        end

        # After step!, which overwrites parts of sys_state.
        s.sys_state.sys_state = Int16(phase)   # 0 park, 1 dive, 2 hold, 3 transition, 4 fig8
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
        s.sys_state.fig_8 = Int16(fig8[])      # live lap count
        # Not filled anywhere in the model chain: without this the log and the viewer read 0.
        s.sys_state.v_wind_200m .= calc_wind_factor(s.am, 200.0) .* s.sys_state.v_wind_gnd
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

# ==================== RESULTS ==================== #

syslog = load_log(log_name; path = output_path)
sl = syslog.syslog
# The geometry is passed in too: without it the criteria are blind to pattern SIZE.
# From the flown path, not from fcs.f8_*. `az_center` is in RADIANS here (it is
# compared against the logged azimuth), the two extents in degrees.
print_fig8_metrics(sl; t_start = fcs.park_time, settle_time = fcs.entry_time,
                   min_elevation = fcs.min_elevation, az_center = deg2rad(az_c_path),
                   az_amplitude = az_amp_path, el_height = el_height_path,
                   min_span_frac = fcs.min_span_frac)

# On the LOGGED PHASE, not a time window; the mean is what v_app_ref should be.
let fig8 = findall(x -> Int(x) == 4, sl.sys_state)
    if isempty(fig8)
        @warn "Phase 4 never reached — no fig8 apparent wind speed."
    else
        va = Float64.(sl.v_app[fig8])
        @printf("  v_app over phase 4 (%.1f s): mean %.2f m/s, range %.2f … %.2f m/s \
                 | v_app_ref = %.1f (%+.1f%%)\n",
                sl.time[fig8[end]] - sl.time[fig8[1]],
                mean(va), minimum(va), maximum(va),
                fcs.v_app_ref, 100 * (mean(va) / fcs.v_app_ref - 1))
    end
end

# Speed of the SIMULATED time against the wall clock; > 1 is faster than realtime.
if t_sim > 0
    @printf("  Performance: %.1f s sim in %.1f s wall = %.2f x realtime \
             (%.1f ms/step over %d steps at dt = %.4f s, vsm_interval = %d)\n",
            t_sim, t_wall, t_sim / t_wall, 1000 * t_wall / round(Int, t_sim / s.dt),
            round(Int, t_sim / s.dt), s.dt, fcs.vsm_interval)
else
    @warn "No simulated time elapsed — no performance figure."
end

# The flown curve, so the plots draw it instead of rebuilding the lemniscate.
REF_PATH = (fec.az_path, fec.el_path)
if show_plots
    include(joinpath(@__DIR__, "simple_fig8_plots.jl"))
else
    @info "Plots suppressed by SHOW_PLOTS = false; it is back to true for the next run."
end

nothing
