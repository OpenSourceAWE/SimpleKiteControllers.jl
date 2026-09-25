# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Disk-based stability analysis of the linearized course-control loop flown by
`simple_fig8.jl`: the gain-scheduled PD of `src/course_controller.jl` closed
around the identified turn-rate law of the V3.

Plant, linearized about a heading `ψ0` (`data/turn_rate_coeffs.yaml`):

    ψ̇ = c1·v_a·u_s(t - delay) + c2/v_a·cos(ψ0)·cos(β)·δψ

The gravity term only adds a slow real pole at `±c2/v_a·cos(β)`; both signs are
checked and the worse one is reported. The controller is the exact discrete
transfer function of `DiscretePIDs.DiscretePID` (backward-Euler filtered
derivative, forward-Euler integral), with the gain schedule
`K = heading_p · v_app_ref / max(v_a, v_app_min)` (times `entry_gain` below
phase 3). The loop is discretized at the project's `1/sample_freq`, so the
steering dead time is an exact number of samples.

Above `v_app_min` the schedule cancels `v_a` from the loop gain `K·c1·v_a`, so
what moves the margins is the depower: `c1` falls and the dead time rises with
it. Three sweeps are therefore reported: the pattern (depower_setpoint, full
gain) over `v_a`, the entry (entry_depower, entry_gain) over `v_a`, and the full
gain over the identified depower range of the flown `body_damping`.

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

PROJECT = selected_project()
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
reload_turn_rate_table!(project)
Ts = 1 / Settings(project).sample_freq

"""
    course_pid(K, Ti, Td, N, Ts) -> TransferFunction

Discrete transfer function from the regulated error to `rel_steering` of the
`DiscretePID` built in `CourseController`: `K` + `K·Ts/Ti/(z-1)` +
`bd·(z-1)/(z-ad)`, with `ad = Td/(Td+N·Ts)` and `bd = K·N·ad`. `Ti = false` means
no integral action.
"""
function course_pid(K, Ti, Td, N, Ts)
    z = tf("z", Ts)
    ad = Td / (Td + N * Ts)
    bd = K * N * ad
    C = K + bd * (z - 1) / (z - ad)
    Ti isa Bool || (C += K * Ts / Ti / (z - 1))
    return C
end

"""
    turn_rate_plant(c1, c2, delay, v_app, gravity, Ts) -> StateSpace

`rel_steering` -> heading, ZOH-discretized, with the dead time rounded to whole
samples. `gravity = cos(ψ0)·cos(β)` in [-1, 1] selects the sign and size of the
gravity pole.
"""
function turn_rate_plant(c1, c2, delay, v_app, gravity, Ts)
    P = c2d(ss(c2 / v_app * gravity, c1 * v_app, 1.0, 0.0), Ts)
    n = round(Int, delay / Ts)
    n == 0 && return P
    # Dead time as an n-sample shift register; a z^-n transfer function is ill-conditioned.
    A = diagm(-1 => ones(n - 1))
    D = ss(A, [1.0; zeros(n - 1)], [zeros(1, n - 1) 1.0], 0.0, Ts)
    return P * D
end

"""
    delay_margin(L) -> Float64

Smallest extra dead time [s] that destabilizes `L`, over all its gain
crossovers. `ControlSystemsBase.delaymargin` takes the phase margin unwrapped
and so reports e.g. 374° instead of 14° for a loop with a long dead time.
"""
function delay_margin(L)
    _, _, wpm, pm = margin(L; allMargins = true)
    dms = [deg2rad(mod(p, 360)) / w for (w, p) in zip(wpm[1], pm[1]) if w > 0]
    return isempty(dms) ? Inf : minimum(dms)
end

"""
    loop_margins(depower, K_phase, v_app) -> NamedTuple

Disk margin, its gain/phase margins and the delay margin of the loop transfer
`L = C·P` at one operating point, worst case over the sign of the gravity pole.
"""
function loop_margins(depower, K_phase, v_app)
    tc = turn_rate_coeffs(fcs.body_damping, depower)
    K = K_phase * fcs.v_app_ref / max(v_app, fcs.v_app_min)
    C = course_pid(K, fcs.heading_i, fcs.heading_d, fcs.heading_d_n, Ts)
    cos_beta = cosd(fcs.el_center)
    results = map((-cos_beta, cos_beta)) do gravity
        L = C * turn_rate_plant(tc.c1, tc.c2, tc.delay, v_app, gravity, Ts)
        dm = try
            diskmargin(L)
        catch
            nothing
        end
        α = isnothing(dm) ? 0.0 : dm.margin
        (; L, dm, α, delay_margin = delay_margin(L))
    end
    worst = argmin(r -> r.α, results)
    return (; worst..., c1 = tc.c1, delay = tc.delay, K)
end

function print_row(label, x, r)
    gm = isnothing(r.dm) ? (NaN, NaN) : r.dm.gainmargin
    pm = isnothing(r.dm) ? NaN : r.dm.phasemargin
    f0 = isnothing(r.dm) ? NaN : r.dm.ω0 / 2π
    @printf("  %-9s %6.3f   c1=%.4f  delay=%.3f s  K=%.3f   α=%5.3f at %4.2f Hz  GM=[%.2f, %.2f]  PM=%5.1f°  DM=%5.3f s\n",
            label, x, r.c1, r.delay, r.K, r.α, f0, gm[1], gm[2], pm, r.delay_margin)
end

function rate(name, αs)
    α_min = minimum(αs)
    if α_min < 0.3
        @error "$name: unstable or fragile, minimum disk margin $(round(α_min, digits=3))."
    elseif α_min < 0.5
        @warn "$name: marginally stable, minimum disk margin $(round(α_min, digits=3))."
    else
        @info "$name: stable, minimum disk margin $(round(α_min, digits=2)). A value ≥ 0.5 is considered robust."
    end
    return α_min
end

@info @sprintf("Course-controller stability, project %s, body_damping = %s, dt = %.4f s, \
                heading_p = %.3f, heading_d = %.3f s, heading_d_n = %.1f, heading_i = %s.",
               PROJECT, fcs.body_damping, Ts, fcs.heading_p, fcs.heading_d,
               fcs.heading_d_n, fcs.heading_i)

v_apps = [5.0, 10.0, 15.0, 20.0, 27.0, 35.0, 45.0]

println("Pattern (phase ≥ 3), depower = $(fcs.depower_setpoint), full gain, over v_app [m/s]:")
pattern = [loop_margins(fcs.depower_setpoint, fcs.heading_p, v) for v in v_apps]
foreach((v, r) -> print_row("v_app", v, r), v_apps, pattern)
rate("Pattern", [r.α for r in pattern])

println("Entry (phases 1-2), depower = $(fcs.entry_depower), entry_gain = $(fcs.entry_gain), over v_app [m/s]:")
entry = [loop_margins(fcs.entry_depower, fcs.entry_gain * fcs.heading_p, v) for v in v_apps]
foreach((v, r) -> print_row("v_app", v, r), v_apps, entry)
rate("Entry", [r.α for r in entry])

dp_lo, dp_hi = turn_rate_depower_range(fcs.body_damping)
depowers = collect(range(dp_lo, dp_hi; length = 13))
println("Full gain at v_app = v_app_ref = $(fcs.v_app_ref) m/s, over depower [-]:")
sweep = [loop_margins(dp, fcs.heading_p, fcs.v_app_ref) for dp in depowers]
foreach((dp, r) -> print_row("depower", dp, r), depowers, sweep)
rate("Depower sweep", [r.α for r in sweep])

# Nominal loop: the pattern at v_app_ref, for `diskmargin(L)` and the plots.
L = loop_margins(fcs.depower_setpoint, fcs.heading_p, fcs.v_app_ref).L

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
