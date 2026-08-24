# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Example script demonstrating an ATTITUDE-STABILIZED parking maneuver of the V3
kite using the high-level `init` + `step!` interface.

Same setup as `examples/simple_parking.jl` — the wing is settled at a fixed
depower setting (DEPOWER_SETPOINT) and parked at a constant tether length
(the caller's winch length loop holds the tether) — but instead of leaving the
steering at zero, a heading PID (as in `examples/simple_sinus.jl`) regulates
the heading to a constant setpoint of zero, so the kite does not drift away
from straight-up parking.

Gain scheduling: the kite's turn rate is roughly proportional to
`u_s * v_app`, so the controller gain is scheduled with `1/v_app`:

    K = HEADING_P * V_APP_REF / v_app

`HEADING_P` is therefore the gain that applies at `v_app == V_APP_REF`, and
the closed-loop response stays roughly invariant when the apparent wind speed
changes. Because `DiscretePID` is in standard form (`K*(e + ...)`), scaling `K`
scales the D action along with it.

Logs the run to "tmp_auto_parking" in the `output` folder
(`examples/../output`, created if missing). For verification, run
`include("examples/simple_auto_parking.jl")` and check the printed heading
regulation RMS error.

At the end it also prints the AoA ripple metrics (see `src/ripple_metrics.jl` and
PlanSuppressOscillations.md) together with the solver cost and the wall clock, as
`examples/simple_parking.jl` does, so a change to e.g. the body-frame damping can
be judged on both the oscillation and the simulation speed. Those numbers are
only comparable across runs that fix PROJECT, V_WIND, TETHER_LENGTH,
DEPOWER_SETPOINT, the heading gains, SIM_TIME and DT.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using V3Kite: init, step!
using DiscretePIDs: set_K!
using Statistics
using Printf
include(joinpath(@__DIR__, "winch_adapter.jl"))  # winch_torque!: V3Kite is torque-only

@info "simple_auto_parking.jl: heading-stabilized parking of the V3 kite."

# ==================== USER PARAMETERS ==================== #

PROJECT =        "system_reelout.yaml"  # System project to use (see data/system_*.yaml)
SIM_TIME         = 60.0     # Total simulation time [s]
DT               = 0.05/3     # Simulation timestep [s]
V_WIND           = 9.51     # Ground wind speed at reference height [m/s]
TETHER_LENGTH    = 150.0    # Initial tether length [m]
DEPOWER_SETPOINT = 0.25     # Depower setting held during parking [-]

# Heading PID gains (output is rel_steering, dimensionless, -1..1).
# Baseline taken from simple_sinus.jl, which was tuned at v_app ≈ V_APP_REF.
HEADING_SETPOINT = 0.0      # Constant heading setpoint [rad]
HEADING_P        = 1.1      # Gain at v_app == V_APP_REF [-]
HEADING_I        = false    # No integral action
HEADING_D        = 0.15     # Derivative time [s], damps the initial transient
V_APP_REF        = 13.1     # Reference apparent wind speed for the gain schedule [m/s]
V_APP_MIN        = 5.0      # Lower clamp on v_app, limits the gain boost [m/s]
MAX_STEERING     = 0.175    # Steering command limit [-]
AERO_MODE        = ContinuousAero() # ContinuousAero() or AeroDirect()
VSM_INTERVAL     = 1   # steps between VSM aero solves
# `INITIAL_BODY_DAMPING` shapes the settling transient; it decays to
# `FLOWN_BODY_DAMPING`, the floor the parked run actually flies with — and
# which the heading gains above were tuned at. Both are part of the settling
# cache key, so changing either re-settles instead of reusing the cached geometry.
INITIAL_BODY_DAMPING = [0.0, 0.0, 40.0]             # Damping settling starts from, per axis [1/s]
FLOWN_BODY_DAMPING   = 0.8 .* INITIAL_BODY_DAMPING  # Damping floor the parked run flies with, per axis [1/s]
# Tether/bridle damping-to-stiffness ratio, overriding the `dyneema` material
# default in `data/struc_geometry.yaml`. `init` floors it during settling
# (see `stabilization.jl`) then applies the raw value to the settled structure.
DAMPING_PER_STIFFNESS = 0.001  # Damping per stiffness of tether and bridles [s]

# ======================== INIT =========================== #

# `init` leaves the data path alone, so `save_log`/`load_log` below need it set here.
set_data_path(v3_data_path())

OUTPUT_DIR = joinpath(@__DIR__, "..", "output")
mkpath(OUTPUT_DIR)

s = init(V_WIND, TETHER_LENGTH; body_start_damping = INITIAL_BODY_DAMPING,
    body_sim_damping = FLOWN_BODY_DAMPING, damping_per_stiffness = DAMPING_PER_STIFFNESS,
    depower_setpoint = DEPOWER_SETPOINT, sim_time = SIM_TIME, dt = DT,
    system_yaml = PROJECT, aero_mode = AERO_MODE, remake_model = false)

# Constant-length setpoint: the tether length just after settling.
l0 = s.sys_state.l_tether[1]
# The winch length loop is the CALLER's now (V3Kite's `step!` takes a torque).
wcs = WCSettings(; dt = s.dt)
wpc = WinchPosController(wcs; dt = s.dt)

# ==================== HEADING CONTROLLER ================= #

heading_pid = create_heading_pid(;
    K = HEADING_P, Ti = HEADING_I, Td = HEADING_D, dt = s.dt,
    umin = -MAX_STEERING, umax = MAX_STEERING)

toc("Start simulation loop...")

# ==================== SIMULATION LOOP ==================== #

steps_done = 0
t_loop = @elapsed try
    for _ in 1:s.steps
        s.sys_state.bearing = HEADING_SETPOINT
        # Gain scheduling: turn rate ~ u_s * v_app, so K ~ 1/v_app.
        v_app = max(s.sys_state.v_app, V_APP_MIN)
        # `set_K!` takes reference and measurement too, for bumpless transfer.
        set_K!(heading_pid, HEADING_P * V_APP_REF / v_app,
               HEADING_SETPOINT, s.sys_state.heading)
        rel_steering = heading_pid(HEADING_SETPOINT, s.sys_state.heading, 0.0)
        # Position mode: the caller's length loop holds the mean tether length.
        step!(s; rel_depower = DEPOWER_SETPOINT, rel_steering,
              set_torque = winch_torque!(wpc, s, l0),
              vsm_interval = VSM_INTERVAL)
        global steps_done += 1
        # The current system state is available via `s.sys_state`.
    end
catch e
    @error "Simulation stopped early at t≈$(round(s.sys_state.time, digits=2))s" exception=(e, catch_backtrace())
end

@info "Save the log"
save_log(s.logger, "tmp_auto_parking"; path=OUTPUT_DIR, colmeta=timestamp_colmeta())

# Regulation error over the settled part (skip the initial transient).
syslog = load_log("tmp_auto_parking"; path=OUTPUT_DIR)
sl = syslog.syslog
settled = findall(t -> t >= 10.0, sl.time)
if !isempty(settled)
    track_err = rad2deg.(sl.heading[settled] .- sl.bearing[settled])
    @printf("Heading regulation RMS error (t ≥ %.0f s): %.2f°, max |e|: %.2f°\n",
            sl.time[settled[1]], sqrt(mean(track_err .^ 2)), maximum(abs.(track_err)))
    @printf("Apparent wind speed: mean %.2f m/s, range %.2f … %.2f m/s\n",
            mean(sl.v_app[settled]), minimum(sl.v_app[settled]), maximum(sl.v_app[settled]))
end

# ==================== RIPPLE METRICS ===================== #

# `sl` is the syslog table of the run just saved, so this measures the same run
# `aoa_ripple` would see via `KiteUtils.syslog(s.logger)` in simple_parking.jl.
ripple = aoa_ripple(sl)
print("\n", format_ripple_report(ripple; sl, stats = s.sam.integrator.stats,
                                 t_loop, n_steps = steps_done))

nothing
