# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Disk-based stability analysis of the linearized course-control loop flown by
`simple_fig8.jl`: the gain-scheduled PD of `src/course_controller.jl` closed
around the identified turn-rate law of the V3.

Plant, linearized about a heading `ψ0`: the steering actuator, a first-order
lag from the commanded to the applied steering,

    T_act·u̇_s = u_cmd - u_s

then the turn-rate law with `c1` and `c2` of the low crosswind pattern
(`data/turn_rate_coeffs.yaml`, [`turn_rate_coeffs`](@ref)),

    ψ̇ = c1·v_a·u_k + c2/v_a·cos(ψ0)·cos(β)·δψ,   T_kite·u̇_k = u_s(t - τ_kite) - u_k

`T_act = 1/steering_gain` of the project's settings (`TAPE_LAG`, 0.1 s at
`steering_gain` 10): the tape's small-signal lag, which injected multisines
confirm up to 4 Hz. It leaves out the tape's rate limit (`v_steering`), which
the pattern hits part of the time; `step_response` below checks large errors
with it.

The kite responds to the applied steering with a dead time `τ_kite` and a lag
`T_kite` of its own, both scaling with the apparent wind speed, see
[`kite_dead_time`](@ref) and [`kite_lag`](@ref): the table's `dead_time` and
`kite_lag` are identified by the low crosswind flights of `build_turn_rate_table.jl`
at the row's `v_app` (about 39 m/s), 0.042 + 0.083 s at depower 0.275. See
`docs/course_loop_stability.md`.

In the pattern (phase ≥ 3) two more factors, both validated against the
simulation at 200 and 300 m (`oldplans/Plan_model_validation.md`, V1 step 1):

- the attractor guidance, `guidance_tf(ω_g)` = `1 + ω_g/s`, `ω_g = v_k/(L·D)`,
  with `v_k = V_K_OVER_V_A · v_a` and `L` the project's tether length;
- `kite_correction`: from ~0.9 Hz up the kite turns less than the turn-rate
  law says, a lag-lead chosen at `v_a` ≈ 33 m/s and scaled with `v_a`;
- the kite's response time from the pattern law, `τ + T =
  pattern_delay_ref · (pattern_v_ref/v_a)^pattern_delay_exp`
  (`pattern_dead_time_lag`, the values in `data/course_loop_model.yaml`),
  identified on pattern logs from 12.8 to 40.6 m/s; at other depowers times
  the measured `exp(pattern_depower_exp·(depower − pattern_law_depower))`.

With them the model under-predicts the simulation's margins at every point
measured (150 – 300 m, `v_a` 22 – 40 m/s): the delay margin by 0 – 42 %, the gain
margin by 21 – 45 %. Below `pattern_v_floor` the pattern law holds its
value, as weak-wind reel-outs measured (0.28 s at 10.1 – 10.6 m/s). What
it still lacks is the dynamics of the fed-back course, which have no low-order
model; `frd_margins` evaluates measured data instead. The pattern tables print
this loop and, for comparison, the inner loop `C·P` alone. `pattern_frd_margins`
evaluates the pattern loop with the measured course correction at the airspeed
it was measured at; its disk margin agrees with the table's, its delay and gain
margins are the realistic ones.

The gravity term only adds a slow real pole at `±c2/v_a·cos(β)`; both signs are
checked and the worse one is reported. The controller is the exact discrete
transfer function of `DiscretePIDs.DiscretePID` (backward-Euler filtered
derivative, forward-Euler integral), with the gain schedule
`K = heading_p · v_app_ref / max(v_a, v_app_min)` (times `entry_gain` below
phase 3, and from phase 3 on floored at `v_app_min_pattern` as well). The loop is discretized at the project's `1/sample_freq`, so the
kite's dead time is an exact number of samples.

A fourth check leaves the linear model: [`step_response`](@ref) simulates the
loop from a large course error with the real tape (proportional, rate-limited)
and the `max_steering` clamp, to find what the rate limit does to large turns.

Above `v_app_min` the schedule cancels `v_a` from the loop gain `K·c1·v_a`, but
not from the kite's dead time, which grows as `v_a` falls. The depower moves
`c1` and, through the table, the dead time too. Three sweeps are therefore
reported: the pattern (`depower_setpoint`, full gain) over `v_a`, the entry
(`entry_depower`, `entry_gain`) over `v_a`, and the full gain over the identified
depower range of the flown `body_damping`.

The disk margin `α` (skew 0) is the radius of the largest disk of simultaneous
gain and phase variations the loop tolerates; `α ≥ 0.5` is considered robust,
as in WinchControllers.jl's `stability_lfc.jl`.

    include("stability_fig8.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using SimpleKiteControllers: project_file
using KiteUtils: Settings, set_data_path
using ControlSystemsBase, RobustAndOptimalControl, MakieControlPlots
using Printf

set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# The caller's input, `run_example("stability_fig8.jl"; show_plots = false)`; a plain `include` plots.
(; show_plots) = script_inputs(@__FILE__, (; show_plots = true))

PROJECT = selected_fig8_project() # system_fig8_*.yaml; a reel-out selection falls back to the default
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
reload_turn_rate_table!(project)
reload_course_loop_model!(project)
SET = Settings(project)
Ts = 1 / SET.sample_freq

"The tape's small-signal lag [s], `1/steering_gain` of the project's settings"
const TAPE_LAG = 1 / SET.steering_gain
"Kite speed over apparent wind speed in the pattern, for the guidance corner: 0.96 at 200 and 300 m, 7 m/s"
const V_K_OVER_V_A = 0.96

"Corner [rad/s] of the attractor guidance at `v_app` [m/s] and the project's tether length"
guidance_corner(v_app) = guidance_rate(fcs, v_app, SET.l_tether, V_K_OVER_V_A * v_app)

"""
    loop_margins(depower, K_phase, v_app; v_min = fcs.course.v_app_min, pattern = false) -> NamedTuple

Disk margin, its gain/phase margins and the delay margin of the loop transfer
`L = C·P` at one operating point, worst case over the sign of the gravity pole
`±c2/v_a·cos(β)` (`c2` of [`turn_rate_coeffs`](@ref)).
`v_min` [m/s] is the floor of the gain schedule, see [`V_MIN_PATTERN`](@ref).
`pattern = true` multiplies in the guidance and the kite correction, and takes
the kite's dead time and lag from the pattern law (`pattern_dead_time_lag`).
"""
function loop_margins(depower, K_phase, v_app; v_min = fcs.course.v_app_min, pattern = false)
    tc = turn_rate_coeffs(fcs.run.body_damping, depower)
    K = K_phase * fcs.course.v_app_ref / max(v_app, v_min)
    C = course_pid(K, fcs.course.heading_i, fcs.course.heading_d, fcs.course.heading_d_n, Ts)
    cos_beta = cosd(fcs.pattern.el_center)
    # In the pattern the kite's response time follows the pattern law (V4), elsewhere the table.
    τ, T_kite = pattern ?
        pattern_dead_time_lag(tc, v_app, depower) :
        (kite_dead_time(tc, v_app), kite_lag(tc, v_app))
    results = map((-cos_beta, cos_beta)) do gravity
        L = C * turn_rate_plant(tc.c1, tc.c2, τ, v_app, gravity, Ts; lag = TAPE_LAG,
                                kite_lag = T_kite)
        pattern && (L = L * kite_correction(Ts, v_app) * guidance_tf(guidance_corner(v_app), Ts))
        dm = try
            diskmargin(L)
        catch
            nothing
        end
        α = isnothing(dm) ? 0.0 : dm.margin
        (; L, dm, α, delay_margin = delay_margin(L))
    end
    worst = argmin(r -> r.α, results)
    return (; worst..., c1 = tc.c1, delay = τ, kite_lag = T_kite, K)
end

"""
    step_response(err0_deg, v_app; depower, K_phase, v_min, t_end = 40.0) -> NamedTuple

Nonlinear simulation of the course loop from a course error of `err0_deg` [deg]
with constant command and constant `v_app` [m/s]: the PD of `CourseController`
(the `DiscretePID` update, clamped to `max_steering`), the KCU tape as
KitePodModels steps it, `u̇ = clamp(steering_gain·(u_cmd - u), ±v_steering)`,
then the kite's dead time and lag and `ψ̇ = c1·v_a·u` (no gravity). Returns the
overshoot [deg], the largest error [deg] over the last 10 s, and the fraction
of time the tape was rate-limited.

The turn-rate law is identified up to `|u| = 0.175`; in the fig8 log the turn
rate at the `max_steering` clamp was only 0.6 – 0.75 of what it predicts, so the
overshoot of a large turn is overstated here.
"""
function step_response(err0_deg, v_app; depower = fcs.course.depower_setpoint, K_phase = fcs.course.heading_p,
                       v_min = V_MIN_PATTERN, t_end = 40.0)
    tc = turn_rate_coeffs(fcs.run.body_damping, depower)
    n = round(Int, kite_dead_time(tc, v_app) / Ts)
    a_kite = exp(-Ts / kite_lag(tc, v_app))
    K = K_phase * fcs.course.v_app_ref / max(v_app, v_min)
    Td, N = fcs.course.heading_d, fcs.course.heading_d_n
    ad = Td / (Td + N * Ts)
    bd = K * N * ad
    gain, v_s = SET.steering_gain, SET.v_steering
    err = deg2rad(err0_deg)
    D, yold, u = 0.0, err, 0.0      # engaged on the error, as set_K! leaves it
    buffer = zeros(n)
    u_lag = 0.0
    steps = round(Int, t_end / Ts)
    errs = zeros(steps)
    limited = 0
    for k in 1:steps
        # calc_steering calls pid(0, err, 0): P = -K·err, D filters -err
        D = ad * D - bd * (err - yold)
        yold = err
        u_cmd = clamp(-K * err + D, -fcs.course.max_steering, fcs.course.max_steering)
        du = gain * (u_cmd - u)
        abs(du) > v_s && (limited += 1)
        u += clamp(du, -v_s, v_s) * Ts
        u_kite = n == 0 ? u : (pushfirst!(buffer, u); pop!(buffer))
        u_lag = a_kite * u_lag + (1 - a_kite) * u_kite
        err += tc.c1 * v_app * u_lag * Ts
        errs[k] = rad2deg(err)
    end
    cross = findfirst(e -> sign(e) != sign(err0_deg), errs)
    overshoot = isnothing(cross) ? 0.0 : maximum(abs, errs[cross:end])
    tail = maximum(abs, errs[end - round(Int, 10 / Ts):end])
    return (; overshoot, tail, limited = limited / steps)
end

function print_row(label, x, r)
    gm = isnothing(r.dm) ? (NaN, NaN) : r.dm.gainmargin
    pm = isnothing(r.dm) ? NaN : r.dm.phasemargin
    f0 = isnothing(r.dm) ? NaN : r.dm.ω0 / 2π
    @printf("  %-9s %6.3f   c1=%.4f  dead=%.3f s  lag=%.3f s  K=%.3f   α=%5.3f at %4.2f Hz  GM=[%.2f, %.2f]  PM=%5.1f°  DM=%5.3f s\n",
            label, x, r.c1, r.delay, r.kite_lag, r.K, r.α, f0, gm[1], gm[2], pm, r.delay_margin)
end

const CLM = course_loop_model()
@info @sprintf("Course-controller stability, project %s, body_damping = %s, dt = %.4f s, \
                heading_p = %.3f, heading_d = %.3f s, heading_d_n = %.1f, heading_i = %s, \
                v_app_min = %.1f m/s, v_app_min_pattern = %.1f m/s, \
                tape lag = %.2f s, kite dead time and lag = table's · (sweep v_app / v_app)^%.2f and ^%.2f; \
                pattern: guidance corner %.2f rad/s at v_app_ref (L = %.0f m), kite correction %.2f/%.2f Hz at %.1f m/s (scaled with v_a), \
                response time %.2f s·(%.0f/v_app)^%.2f.",
               PROJECT, fcs.run.body_damping, Ts, fcs.course.heading_p, fcs.course.heading_d,
               fcs.course.heading_d_n, fcs.course.heading_i, fcs.course.v_app_min, fcs.course.v_app_min_pattern,
               TAPE_LAG, CLM.kite_dead_time_exp, CLM.kite_lag_exp,
               guidance_corner(fcs.course.v_app_ref), SET.l_tether, CLM.kite_corr_zero, CLM.kite_corr_pole, CLM.kite_corr_v_ref,
               CLM.pattern_delay_ref, CLM.pattern_v_ref, CLM.pattern_delay_exp)

v_apps = [5.0, 10.0, 15.0, 20.0, 27.0, 35.0, 45.0]
"Floor of the gain schedule from phase 3 on, as `calc_steering` applies it"
const V_MIN_PATTERN = max(fcs.course.v_app_min, fcs.course.v_app_min_pattern)

println("Pattern (phase ≥ 3), depower = $(fcs.course.depower_setpoint), full gain, over v_app [m/s], \
         with guidance and kite correction:")
pattern = [loop_margins(fcs.course.depower_setpoint, fcs.course.heading_p, v; v_min = V_MIN_PATTERN, pattern = true)
           for v in v_apps]
foreach((v, r) -> print_row("v_app", v, r), v_apps, pattern)
rate_disk_margin("Pattern", [r.α for r in pattern])
println("  the same, inner loop C·P alone:")
inner = [loop_margins(fcs.course.depower_setpoint, fcs.course.heading_p, v; v_min = V_MIN_PATTERN) for v in v_apps]
foreach((v, r) -> print_row("v_app", v, r), v_apps, inner)

println("Entry (phases 1-2), depower = $(fcs.course.entry_depower), entry_gain = $(fcs.course.entry_gain), over v_app [m/s]:")
entry = [loop_margins(fcs.course.entry_depower, fcs.course.entry_gain * fcs.course.heading_p, v) for v in v_apps]
foreach((v, r) -> print_row("v_app", v, r), v_apps, entry)
rate_disk_margin("Entry", [r.α for r in entry])

dp_lo, dp_hi = turn_rate_depower_range(fcs.run.body_damping)
depowers = collect(range(dp_lo, dp_hi; length = 13))
println("Full gain at v_app = v_app_ref = $(fcs.course.v_app_ref) m/s, over depower [-], \
         with guidance and kite correction:")
sweep = [loop_margins(dp, fcs.course.heading_p, fcs.course.v_app_ref; v_min = V_MIN_PATTERN, pattern = true)
         for dp in depowers]
foreach((dp, r) -> print_row("depower", dp, r), depowers, sweep)
rate_disk_margin("Depower sweep", [r.α for r in sweep])

"""
    pattern_frd_margins(v_app; fs = 0.25:0.005:3.9) -> NamedTuple

Delay margin, gain margin and disk margin of the pattern loop with the
MEASURED course correction (`load_course_correction`) in place of
`kite_correction`: `C · tape · turn-rate law · M · guidance`, evaluated on a
frequency grid (`frd_margins`, `frd_diskmargin`). `M` was measured at `v_a`
23.7, 34 and 40.1 m/s (200 – 300 m) and is interpolated between them
(`course_correction`); outside that range, and at other tether lengths, it is
extrapolated. Its delay margin matched the simulation at 150, 200 and 300 m
(within 3 %); its gain margin is optimistic at shorter tethers (+19 % at
200 m, +33 % at 150 m), and between the measured airspeeds both may be ~20 %
off. The `pattern = true` loop of the tables is the conservative one. The gravity pole, far below `fs`, is left out.
"""
function pattern_frd_margins(v_app; fs = 0.25:0.005:3.9)
    tabs = load_course_correction()
    tc = turn_rate_coeffs(fcs.run.body_damping, fcs.course.depower_setpoint)
    K = fcs.course.heading_p * fcs.course.v_app_ref / max(v_app, V_MIN_PATTERN)
    C = course_pid(K, fcs.course.heading_i, fcs.course.heading_d, fcs.course.heading_d_n, Ts)
    τ, T_kite, ω_g = kite_dead_time(tc, v_app), kite_lag(tc, v_app), guidance_corner(v_app)
    L = map(fs) do f
        ω = 2π * f
        plant = tc.c1 * v_app * cis(-ω * τ) / ((1 + im * ω * TAPE_LAG) * (1 + im * ω * T_kite) * (im * ω))
        evalfr(C, cis(ω * Ts))[1] * plant * course_correction(tabs, f, v_app) * (1 + ω_g / (im * ω))
    end
    return (; frd_margins(collect(fs), L)..., α = frd_diskmargin(L))
end

println("Pattern with the MEASURED course correction (realistic; measured at v_a 23.7 - 40.1 m/s, 200 - 300 m), \
         against the conservative loop of the tables:")
for v in (25.0, 34.0, 40.0)
    local m = pattern_frd_margins(v)
    local r = loop_margins(fcs.course.depower_setpoint, fcs.course.heading_p, v; v_min = V_MIN_PATTERN, pattern = true)
    @printf("  v_app %4.1f m/s: measured correction α = %.2f  DM = %.3f s  GM = %.2f at %.2f Hz | \
             kite_correction α = %.2f  DM = %.3f s\n", v, m.α, m.dm, m.gm, m.f_pc, r.α, r.delay_margin)
end

step_v_apps = [13.0, 15.0, 20.0, 27.0, 35.0]
step_errs = [5.0, 20.0, 45.0, 90.0, 135.0, 170.0]
println("Large errors, pattern gain, tape rate limit $(SET.v_steering) 1/s: overshoot [deg] \
         (* = still oscillating > 1° after 30 s), over v_app [m/s]:")
println("  error    ", join((@sprintf("%8.1f", v) for v in step_v_apps)))
tails = Float64[]
for e0 in step_errs
    rs = [step_response(e0, v) for v in step_v_apps]
    append!(tails, [r.tail for r in rs])
    println(@sprintf("  %5.0f°  ", e0),
            join((@sprintf("%7.1f%s", r.overshoot, r.tail > 1.0 ? "*" : " ") for r in rs)))
end
if maximum(tails) > 1.0
    @error "Large errors: the rate-limited loop does not settle for every error and v_app."
else
    @info "Large errors: the rate-limited loop settles for every error and v_app, no limit cycle."
end

# Nominal loop: the pattern at v_app_ref, for `diskmargin(L)` and the plots.
L = loop_margins(fcs.course.depower_setpoint, fcs.course.heading_p, fcs.course.v_app_ref; v_min = V_MIN_PATTERN,
                 pattern = true).L

if show_plots
    display(bode_plot(L; from = -2, to = log10(0.5 / Ts),
                      title = "Course loop L = C·P·K·G, depower = $(fcs.course.depower_setpoint), v_app = $(fcs.course.v_app_ref) m/s"))
    MakieControlPlots.plot(depowers, [r.α for r in sweep], [r.delay_margin for r in sweep];
         xlabel = "relative depower [-]",
         ylabels = ["disk margin α [-]", "delay margin [s]"],
         title = "Course loop margins, full gain", fig = "course_loop_margins", disp = true)
end
@info "Type 'diskmargin(L)' for details on the nominal pattern loop."
nothing
