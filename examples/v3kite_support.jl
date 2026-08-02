# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
V3Kite glue for `simple_fig8.jl`: the three pieces the run needs that V3Kite's
released `init`/`step!` interface does not provide.

They live here, in `examples/`, rather than in `src/`, because each one reaches
into a `V3KITE` — and SimpleKiteControllers must not depend on a kite model. The
library half of the run (the guidance, the metrics, the settings and the
turn-rate table) is model-agnostic and stays in `src/`.

All three are written against the PUBLIC V3Kite API only (`step!`,
`winch_force`, `force_to_torque`, `unstretched_length`, `reel_out_speed`,
`create_logger`), so nothing here needs a patched V3Kite. If a later V3Kite
release grows equivalents, delete the corresponding function here and use
V3Kite's; the names are deliberately the same, except `create_fig8_pid` (V3Kite
exports a `create_heading_pid` without the derivative-filter argument this loop
needs, and two exported functions of one name cannot both be `using`-ed).
"""

using DiscretePIDs: DiscretePID
using Printf: @sprintf

"""
    span_mean_aoa(sys) -> Float64

Span-averaged geometric angle of attack of the wing [rad], the mean of the
VSM's per-panel `alpha_geometric_dist`.

Use this rather than `SysState.AoA` whenever the number is meant to describe
the *whole wing*. `update_sys_state!` fills `AoA` from the single centre panel
of `alpha_geometric_dist`, which is representative only while the wing is
loaded symmetrically: steering twists the two halves in opposite directions, so
in a turn the centre panel misses the spanwise spread entirely.

Panels whose local velocity is too small for an angle to be defined are logged
as `NaN` by the VSM; they are skipped here, and the result is `NaN` only if
every panel is. Returns `NaN` for a wing without a VSM solver (e.g. a
`PlateWing`), where no spanwise distribution exists.
"""
function span_mean_aoa(sys)
    wing = sys.wings[1]
    hasproperty(wing, :vsm_solver) || return NaN
    alphas = wing.vsm_solver.sol.alpha_geometric_dist
    # Same wrap as update_sys_state! applies to the centre panel: the VSM
    # returns some panels as `pi + atan(...)`, which would otherwise average
    # against the others as if it were a large positive angle.
    n = 0
    acc = 0.0
    for a in alphas
        isnan(a) && continue
        n += 1
        acc += mod(a + pi, 2pi) - pi
    end
    return n == 0 ? NaN : acc / n
end

"""
    WinchForceController

State and gains of the force-mode winch (see [`winch_force_torque!`](@ref)).
`f_lpf` is the low-passed reference force, `NaN` until the first step.

Built from an `FC_Settings` by [`WinchForceController`](@ref)`(fcs)`, which
applies the `compliance` scaling.
"""
Base.@kwdef mutable struct WinchForceController
    "Time constant of the reference-force low-pass [s]"
    force_tau::Float64 = 10.0
    "Length-trim gain, length error [m] → reference force [N/m]"
    len_kp::Float64 = 100.0
    "Viscous damping on the drum, reel-out speed [m/s] → reference force [N·s/m]"
    damp::Float64 = 500.0
    "Floor on the reference force, keeps the tether taut [N]"
    force_min::Float64 = 100.0
    "Low-passed reference force, `NaN` before the first step [N]"
    f_lpf::Float64 = NaN
end

"""
    WinchForceController(fcs::FC_Settings) -> WinchForceController

Build the force-mode winch from the run's settings, applying the `compliance`
scaling: `winch_len_kp` and `winch_damp` are both divided by `fcs.compliance`,
so the yield scales linearly with it while their ratio — the length loop's own
time constant — is unchanged. `winch_force_tau` is passed through untouched (it
sets WHICH frequencies the drum yields to, not by how much).

Errors at `compliance == 0`: that is position mode and must not be flown through
this controller (an infinitely stiff spring is not representable — see
`FC_Settings`).
"""
function WinchForceController(fcs::FC_Settings)
    fcs.compliance > 0 ||
        error("WinchForceController needs compliance > 0; at 0 use position mode.")
    return WinchForceController(
        force_tau = fcs.winch_force_tau,
        len_kp = fcs.winch_len_kp / fcs.compliance,
        damp = fcs.winch_damp / fcs.compliance,
        force_min = fcs.winch_force_min)
end

"""
    winch_force_torque!(wfc::WinchForceController, s, set_length) -> torque

Force-mode winch controller: the drum is commanded to hold a *force*, not a
length, so it pays out whenever the tether pulls harder than the reference and
hauls in when it pulls less. Returns a torque to be passed to `step!` as
`set_torque`, e.g.

    step!(s; rel_steering, set_torque = winch_force_torque!(wfc, s, l0))

The reference force is a first-order low-pass of the measured winch force with
time constant `force_tau`, plus a slow length trim `len_kp * (l - set_length)` —
a tether longer than the setpoint asks for more holding force, which hauls it
back in — plus viscous damping `damp * reel_out_speed(s)`, which opposes fast
payout. The damping is not optional: force control gives the drum no velocity
feedback, so trim alone leaves it a free mass on a spring. Because the reference
tracks only the *mean* force, the drum yields to everything faster than
`force_tau` (the lap, the dive, gusts) while the mean length is still held. Raise
`force_tau` for a softer winch, raise `len_kp` for tighter length keeping; they
trade against each other, and `len_kp` should stay small enough that the length
loop is far slower than a lap.

Contrast V3Kite's `winch_position_torque!`, which holds a length: its force
feed-forward cancels the measured load exactly, so the drum has nothing to
accelerate it whatever the PI gains are. Nothing here reads those gains — the
two modes share only the winch.

The reference force is initialised to the measured force on the first call, so
engaging force mode from a settled state produces no torque step.
"""
function winch_force_torque!(wfc::WinchForceController, s, set_length)
    f_now = winch_force(s)
    isnan(wfc.f_lpf) && (wfc.f_lpf = f_now)
    alpha = s.dt / (wfc.force_tau + s.dt)
    wfc.f_lpf += alpha * (f_now - wfc.f_lpf)
    # Length trim (a spring) plus viscous damping. The damping term is what keeps
    # the drum from free-wheeling: force control gives it no velocity feedback of
    # its own, so spring alone is an undamped oscillator.
    f_set = wfc.f_lpf + wfc.len_kp * (unstretched_length(s) - set_length) +
            wfc.damp * reel_out_speed(s)
    return force_to_torque(max(f_set, wfc.force_min), s.sys)
end

"""
    warmup!(s, warmup_time; depower, wfc=nothing) -> nothing

Relax `s` into an equilibrium of ITS OWN model, then throw the relaxation away:
step the model forward `warmup_time` seconds with zero steering, the depower held
at `depower` and the winch in the mode the run will use, and afterwards replace
the logger and `sys_state` so the run's first logged row is again `t = 0`.

WHY: `settle_wing` returns an equilibrium of the SETTLING model — `dt = 0.001`,
heavily damped, winch braked, so the tether length is held kinematically. The
run integrates a different model: the brake is off, the drum holds a torque
instead of a length, and the aero load is applied at the run's `dt`. The settled
state is therefore not a fixed point of the model that starts at `t = 0`, and the
difference shows up as a decaying transient over the first second or so of every
log — visible in tether force, AoA and (sharply, because it is a ratio of two
forces that both dip) in the logged L/D.

The park phase of a controller script already exists to let that decay before the
controller engages; the warm-up is the same idea moved to where it belongs, so
the transient is not part of the run's data at all. It is deliberately NOT free:
it costs `warmup_time / dt` full steps, and it is real integration, not a
re-settle — if the model has no equilibrium at this condition (a diverging run)
the warm-up diverges with it, which is a true result and not a warm-up failure.

`wfc` must match what the caller will command afterwards: a
[`WinchForceController`](@ref) engages force mode against the current length (and
leaves its reference-force low-pass initialised, so the run starts with force
mode already engaged), `nothing` holds the length via `set_length`. Warming up
against the wrong winch would hand the run exactly the discontinuity this is
meant to remove.

The integrator clock keeps running across the warm-up; only the logged time is
restarted, which is what `step!` advances (`t = s.sys_state.time + dt`). The
progress lines `step!` prints during the warm-up are counted against `s.steps`
and can be ignored.
"""
function warmup!(s, warmup_time; depower = 0.0, wfc = nothing)
    n = round(Int, warmup_time / s.dt)
    n < 1 && return nothing
    if n > s.steps
        # The warm-up logs through the run's logger before that log is thrown
        # away, and the logger holds `steps + 1` rows.
        @warn "warmup_time is longer than the whole run; clamping to sim_time."
        n = s.steps
    end
    @info @sprintf("warmup: %.2f s (%d steps) with the winch in %s mode...",
                   warmup_time, n, isnothing(wfc) ? "position" : "force")
    # Hold the length the model arrived at; the run re-references to wherever
    # the warm-up leaves it (`l0` is read from `sys_state` afterwards).
    l_hold = unstretched_length(s)
    for _ in 1:n
        if isnothing(wfc)
            step!(s; rel_depower = depower, rel_steering = 0.0,
                  set_length = l_hold)
        else
            step!(s; rel_depower = depower, rel_steering = 0.0,
                  set_torque = winch_force_torque!(wfc, s, l_hold))
        end
    end
    # Discard the warm-up: a fresh logger and a `sys_state` rebuilt from the
    # model, which also re-logs the (now relaxed) t = 0 row.
    logger, sys_state = create_logger(s.sam, s.steps)
    s.logger = logger
    s.sys_state = sys_state
    s.sys_state.time = 0.0
    s.last_step_time = NaN
    return nothing
end

"""
    create_fig8_pid(; K, Ti, Td, N, dt, umin, umax) -> DiscretePID

Heading/course PID for the figure-eight loop. Gains `Ti` and `Td` accept `false`
to disable integral/derivative.

The only difference from V3Kite's `create_heading_pid` is `N`, the derivative
filter's maximum gain: the D path is `K*Td*s / (1 + s*Td/N)`, so it amplifies
measurement noise by up to `N*K` above the corner `N/(2*pi*Td)` [Hz].
`DiscretePIDs` defaults to 10, which on a course loop closed at ~0.1 Hz is 10x
noise gain bought for a few degrees of phase lead; at `N = 10` the broadband
noise on the fed-back angles came out as a 7.95 Hz ripple on the command. Hence
`fcs.heading_d_n = 2`, and hence this function rather than V3Kite's.
"""
function create_fig8_pid(; K = 1.0, Ti = false, Td = false, N = 10.0,
                         dt, umin = -1.0, umax = 1.0)
    return DiscretePID(; K, Ti, Td, N, Ts = dt, umin, umax)
end
