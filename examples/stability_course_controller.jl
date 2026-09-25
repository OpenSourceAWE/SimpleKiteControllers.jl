# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Disk-based stability analysis of the linearized course-control loop flown by
`simple_fig8.jl`: the gain-scheduled PD of `src/course_controller.jl` closed
around the identified turn-rate law of the V3.

Plant, linearized about a heading `ψ0`: the steering actuator, a first-order
lag from the commanded to the applied steering,

    T_act·u̇_s = u_cmd - u_s

then the turn-rate law of `data/turn_rate_coeffs.yaml`,

    ψ̇ = c1·v_a·u_s(t - τ_kite) + c2/v_a·cos(ψ0)·cos(β)·δψ

The lag stands in for the KCU tape's rate limit (`v_steering`), which is
nonlinear: `ACTUATOR_LAG` is its equivalent at the amplitudes flown in the
pattern, where the tape is rate-limited a quarter of the time. It was
identified on a `simple_fig8.jl` log.

The kite's dead time scales with the apparent wind speed, see
[`kite_delay`](@ref): the table's `delay` is identified by the relay sweeps of
`build_turn_rate_table.jl` at the row's `v_app` (about 13 m/s), and the same
fit gives 0.22 s at 22.5 m/s and 0.12 s in the pattern at 36 m/s. See
`docs/course_loop_stability.md`.

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
reported: the pattern (depower_setpoint, full gain) over `v_a`, the entry
(entry_depower, entry_gain) over `v_a`, and the full gain over the identified
depower range of the flown `body_damping`.

The disk margin `α` (skew 0) is the radius of the largest disk of simultaneous
gain and phase variations the loop tolerates; `α ≥ 0.5` is considered robust,
as in WinchControllers.jl's `stability_lfc.jl`.

    include("stability_course_controller.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using SimpleKiteControllers: project_file
using KiteUtils: Settings, set_data_path
using ControlSystemsBase, RobustAndOptimalControl, MakieControlPlots
using LinearAlgebra: diagm
using Printf

set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
show_plots = @isdefined(SHOW_PLOTS) ? SHOW_PLOTS : true
SHOW_PLOTS = true

PROJECT = selected_fig8_project() # system_fig8_*.yaml; a reel-out selection falls back to the default
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
reload_turn_rate_table!(project)
SET = Settings(project)
Ts = 1 / SET.sample_freq

include(joinpath(@__DIR__, "course_loop_model.jl"))

"""
    loop_margins(depower, K_phase, v_app; v_min = fcs.v_app_min) -> NamedTuple

Disk margin, its gain/phase margins and the delay margin of the loop transfer
`L = C·P` at one operating point, worst case over the sign of the gravity pole.
`v_min` [m/s] is the floor of the gain schedule, see [`V_MIN_PATTERN`](@ref).
"""
function loop_margins(depower, K_phase, v_app; v_min = fcs.v_app_min)
    tc = turn_rate_coeffs(fcs.body_damping, depower)
    K = K_phase * fcs.v_app_ref / max(v_app, v_min)
    C = course_pid(K, fcs.heading_i, fcs.heading_d, fcs.heading_d_n, Ts)
    cos_beta = cosd(fcs.el_center)
    τ = kite_delay(tc, v_app)
    results = map((-cos_beta, cos_beta)) do gravity
        L = C * turn_rate_plant(tc.c1, tc.c2, τ, v_app, gravity, Ts)
        dm = try
            diskmargin(L)
        catch
            nothing
        end
        α = isnothing(dm) ? 0.0 : dm.margin
        (; L, dm, α, delay_margin = delay_margin(L))
    end
    worst = argmin(r -> r.α, results)
    return (; worst..., c1 = tc.c1, delay = τ, K)
end

"""
    step_response(err0_deg, v_app; depower, K_phase, v_min, t_end = 40.0) -> NamedTuple

Nonlinear simulation of the course loop from a course error of `err0_deg` [deg]
with constant command and constant `v_app` [m/s]: the PD of `CourseController`
(the `DiscretePID` update, clamped to `max_steering`), the KCU tape as
KitePodModels steps it, `u̇ = clamp(steering_gain·(u_cmd - u), ±v_steering)`,
then the kite's dead time and `ψ̇ = c1·v_a·u` (no gravity). Returns the
overshoot [deg], the largest error [deg] over the last 10 s, and the fraction
of time the tape was rate-limited.

The turn-rate law is identified up to `|u| = 0.175`; in the fig8 log the turn
rate at the `max_steering` clamp was only 0.6 – 0.75 of what it predicts, so the
overshoot of a large turn is overstated here.
"""
function step_response(err0_deg, v_app; depower = fcs.depower_setpoint, K_phase = fcs.heading_p,
                       v_min = V_MIN_PATTERN, t_end = 40.0)
    tc = turn_rate_coeffs(fcs.body_damping, depower)
    n = round(Int, kite_delay(tc, v_app) / Ts)
    K = K_phase * fcs.v_app_ref / max(v_app, v_min)
    Td, N = fcs.heading_d, fcs.heading_d_n
    ad = Td / (Td + N * Ts)
    bd = K * N * ad
    gain, v_s = SET.steering_gain, SET.v_steering
    err = deg2rad(err0_deg)
    D, yold, u = 0.0, err, 0.0      # engaged on the error, as set_K! leaves it
    buffer = zeros(n)
    steps = round(Int, t_end / Ts)
    errs = zeros(steps)
    limited = 0
    for k in 1:steps
        # calc_steering calls pid(0, err, 0): P = -K·err, D filters -err
        D = ad * D - bd * (err - yold)
        yold = err
        u_cmd = clamp(-K * err + D, -fcs.max_steering, fcs.max_steering)
        du = gain * (u_cmd - u)
        abs(du) > v_s && (limited += 1)
        u += clamp(du, -v_s, v_s) * Ts
        u_kite = n == 0 ? u : (pushfirst!(buffer, u); pop!(buffer))
        err += tc.c1 * v_app * u_kite * Ts
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
    @printf("  %-9s %6.3f   c1=%.4f  delay=%.3f s  K=%.3f   α=%5.3f at %4.2f Hz  GM=[%.2f, %.2f]  PM=%5.1f°  DM=%5.3f s\n",
            label, x, r.c1, r.delay, r.K, r.α, f0, gm[1], gm[2], pm, r.delay_margin)
end

@info @sprintf("Course-controller stability, project %s, body_damping = %s, dt = %.4f s, \
                heading_p = %.3f, heading_d = %.3f s, heading_d_n = %.1f, heading_i = %s, \
                v_app_min = %.1f m/s, v_app_min_pattern = %.1f m/s, \
                actuator lag = %.2f s, kite dead time = table delay · (sweep v_app / v_app)^%.2f.",
               PROJECT, fcs.body_damping, Ts, fcs.heading_p, fcs.heading_d,
               fcs.heading_d_n, fcs.heading_i, fcs.v_app_min, fcs.v_app_min_pattern,
               ACTUATOR_LAG, KITE_DELAY_EXP)

v_apps = [5.0, 10.0, 15.0, 20.0, 27.0, 35.0, 45.0]
"Floor of the gain schedule from phase 3 on, as `calc_steering` applies it"
const V_MIN_PATTERN = max(fcs.v_app_min, fcs.v_app_min_pattern)

println("Pattern (phase ≥ 3), depower = $(fcs.depower_setpoint), full gain, over v_app [m/s]:")
pattern = [loop_margins(fcs.depower_setpoint, fcs.heading_p, v; v_min = V_MIN_PATTERN) for v in v_apps]
foreach((v, r) -> print_row("v_app", v, r), v_apps, pattern)
rate("Pattern", [r.α for r in pattern])

println("Entry (phases 1-2), depower = $(fcs.entry_depower), entry_gain = $(fcs.entry_gain), over v_app [m/s]:")
entry = [loop_margins(fcs.entry_depower, fcs.entry_gain * fcs.heading_p, v) for v in v_apps]
foreach((v, r) -> print_row("v_app", v, r), v_apps, entry)
rate("Entry", [r.α for r in entry])

dp_lo, dp_hi = turn_rate_depower_range(fcs.body_damping)
depowers = collect(range(dp_lo, dp_hi; length = 13))
println("Full gain at v_app = v_app_ref = $(fcs.v_app_ref) m/s, over depower [-]:")
sweep = [loop_margins(dp, fcs.heading_p, fcs.v_app_ref; v_min = V_MIN_PATTERN) for dp in depowers]
foreach((dp, r) -> print_row("depower", dp, r), depowers, sweep)
rate("Depower sweep", [r.α for r in sweep])

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
L = loop_margins(fcs.depower_setpoint, fcs.heading_p, fcs.v_app_ref; v_min = V_MIN_PATTERN).L

if show_plots
    display(bode_plot(L; from = -2, to = log10(0.5 / Ts),
                      title = "Course loop L = C·P, depower = $(fcs.depower_setpoint), v_app = $(fcs.v_app_ref) m/s"))
    MakieControlPlots.plot(depowers, [r.α for r in sweep], [r.delay_margin for r in sweep];
         xlabel = "relative depower [-]",
         ylabels = ["disk margin α [-]", "delay margin [s]"],
         title = "Course loop margins, full gain", fig = "course_loop_margins", disp = true)
end
@info "Type 'diskmargin(L)' for details on the nominal pattern loop."
nothing
