# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Settings of the figure-of-eight FLIGHT CONTROLLER flown by
`examples/simple_fig8.jl`: the simulation conditions, the pattern geometry, the
entry state machine, the heading/course PID and the metrics window. Loaded from
a YAML file (`fc_settings.yaml`) rather than hard-coded in the example, so a
sweep can vary them without editing the script.

Everything here is a TUNING parameter of one run. The POSITION-mode winch gains
live in the kite model's own winch settings (V3Kite's `WC_Settings`, loaded from
its `data/wc_settings.yaml`) and the model/geometry in its `settings.yaml`; the
FORCE-mode winch parameters are here (`winch_*` below), because how compliant the
winch is during a figure-eight is a choice of the run rather than a property of
the model. [`winch_force_gains`](@ref) applies the `compliance` scaling to them;
the controller they feed is V3Kite's `WinchForceController`.

The dated record of how these values were arrived at — sweeps, reverted attempts
and the failures behind each closed lever — is in `docs/fig8_tuning_log.md`.
Add new findings there, not here.
"""
@with_kw mutable struct FC_Settings @deftype Float64
    """
    System project file, see `data/system_*.yaml`. Resolved by
    [`project_file`](@ref) against this package's `data/` first, so the
    simulation conditions of a run are the ones bundled here; a name only the
    kite model knows falls back to the model's data directory.
    """
    project::String = "system_reelout.yaml"
    """
    Total simulation time [s]; ~43 s per lap at v_app 13 m/s, plus the descent
    from the park. The metrics window opens at `park_time + entry_time`, so a
    short run cannot be judged on `laps`.
    """
    sim_time = 150.0
    """
    Simulation timestep [s]. NOT a tuning parameter: 0.05/3 was numerically
    unstable, because `step!` holds the VSM aero load frozen inside the DAE
    between updates and that explicit coupling develops a growing 2*dt
    oscillation at maximum dynamic pressure.
    """
    dt = 0.05/6
    """
    Steps between VSM aero updates, held frozen in between (0 disables them).
    1 is the tightest coupling available, so this can only be raised, trading
    aero lag for wall time. Passed straight to V3Kite's `step!`.
    """
    vsm_interval::Int64 = 1
    "Ground wind speed at reference height [m/s]"
    v_wind = 5.0
    """
    How soft the winch is [-]. Divides both `winch_len_kp` and `winch_damp`, so
    the steady-state yield scales linearly with it while the length loop's own
    time constant is unchanged: a softer winch of the same character, not a
    different one. Above 1.0 is meaningful (2.0 = twice as soft).

    AT EXACTLY 0 the code path changes: the run switches to POSITION mode and
    hands the winch to the kite model's own controller, which holds a constant
    unstretched length. Continuous in behaviour, not in code.
    """
    compliance = 0.5
    """
    Initial tether length [m], held more or less constant. Tested from 150 m to
    300 m. The minimum angular turn radius is `1/(L*c1*u_s)`, so a LONGER tether
    turns tighter in angular terms — the most effective lever on pattern
    feasibility after `c1` itself.
    """
    tether_length = 200.0
    "Depower held during the run [-]; sets the operating point of the turn-rate law"
    depower_setpoint = 0.26
    """
    Settling elevation [deg]. NOTE: currently has NO EFFECT on where the run
    starts — `settle_wing`'s cache key does not include it, so the existing 73°
    geometry is reused.
    """
    elevation = 73.0
    """
    Parking phase [s]: hold zero steering so the transients left by
    init/settling decay before the controller demands maneuvers. The guidance
    still runs (its course estimate needs warming up), but is not applied.
    """
    park_time = 2.0
    """
    Warm-up [s], run inside `init` and discarded (V3Kite's `warmup!`, which
    `init` calls when `warmup_time > 0`). The park lets the settling transients
    decay; this lets them decay BEFORE t = 0, so they are not in the log at all.
    Costs `warmup_time / dt` full steps of wall time. 0.0 disables it.
    """
    warmup_time = 2.0

    # ---- Force-mode winch, read only when `compliance > 0` and before it scales these #
    """
    Time constant of the low-pass turning measured winch force into reference
    force [s]. The drum yields to everything faster and holds everything slower,
    so this is the compliance TIMESCALE and should sit above the lap rate. Not
    scaled by `compliance`.
    """
    winch_force_tau = 10.0
    "Length-trim gain, length error [m] -> reference force [N/m], before `compliance` scaling"
    winch_len_kp = 100.0
    """
    Viscous damping on the drum, reel-out speed [m/s] -> reference force
    [N·s/m], before `compliance` scaling. Not optional: without it force mode
    has no velocity feedback and the drum speed runs away.
    """
    winch_damp = 500.0
    "Floor on the reference force, keeps the tether taut [N]"
    winch_force_min = 100.0

    # ---- Entry state machine: park -> dive -> hold -> fig8 ------------------ #
    """
    Course commanded during the dive [deg]. |chi| > 90 is descending, < 90
    climbing, 90 exactly horizontal. A POSITIVE commanded course drives the kite
    towards NEGATIVE azimuth, and the entry is flown at the pattern's rightmost
    point, so this is negative.
    """
    chi_dive = -85.0
    "Course commanded during the hold [deg]: horizontal, so the kite arrives flat"
    chi_hold = -90.0
    "Margin above `el_center` [deg] at which the dive ends and the hold begins"
    dive_el_margin = 7.0
    "Duration of the hold [s]"
    hold_time = 0.8

    """
    In-plane body damping. A FLIGHT parameter, not just a solver setting: it
    sets `c1` and hence the achievable turn radius. `init`'s default is the most
    agile and the only one that flies this pattern inside the identified
    steering range.
    """
    body_damping::Vector{Float64} = [0.0, 0.0, 40.0]

    # ---- Pattern geometry [deg]; a SMALLER lemniscate is a TIGHTER one ------ #
    "Width of the eight [deg] (azimuth spans +-`f8_a`)"
    f8_a = 40.0
    "Height of the eight [deg] (elevation spans +-`f8_b`/2)"
    f8_b = 15.0
    "Size of the right part [deg]"
    f8_c = 0.0
    "Asymmetry factor [-]"
    f8_d = 0.0
    """
    Pattern-centre elevation [deg]. Two forces pull opposite ways: a lower
    centre IMPROVES the curvature margin (less `cos(elevation)` compression of
    the azimuth axis) but pushes the pattern deeper into the power zone, and
    every failure at low centre has been an ENERGY failure.
    """
    el_center = 26.0
    "Arc distance Q -> attractor [deg]"
    attractor_dist = 10.0
    """
    Fly UP-loops instead of down-loops. Reverses the traversal direction of the
    reference path; the shape is unchanged, so the curvature margin is
    unaffected. Down-loops convert height into speed where up-loops shed energy
    through the turn.
    """
    up_loops::Bool = false

    # ---- Heading PID; output is rel_steering (-1..1), fed UNNEGATED --------- #
    """
    Gain at `v_app == v_app_ref`, i.e. the gain actually applied in phase 3.
    Only the product `heading_p * v_app_ref` is physical.

    For context, the plant `psi_dot = c1*v_a*u_s` is an INTEGRATOR of gain
    `c1*v_a` = 3.66 rad/s per unit u_s at flight speed, so the crossover is
    `omega_c = K*3.66`. Against the 0.72 s measured tape lag, `omega_c*T_d <~
    0.8 rad` gives `K <~ 0.46`. The flown value sits well inside that.
    """
    heading_p = 0.1941
    """
    Integral time [s], or `false` for no integral action (the default): a steady
    heading bias shows up as a steady cross-track error, which the guidance
    already corrects by pulling the attractor back onto the path. Try a finite
    Ti only if a persistent one-sided offset remains.
    """
    heading_i::Union{Bool, Float64} = false
    "Derivative time [s], damps the initial transient"
    heading_d = 0.12
    """
    Derivative filter: maximum gain of the D path, `K*Td*s/(1 + s*Td/N)`. 2
    rather than the `DiscretePIDs` default of 10 because the fed-back angles
    carry broadband noise, which at N = 10 was amplified into a ripple on the
    command. At the loop's own 0.1 Hz this is a filter change, not a gain change.
    """
    heading_d_n = 2.0
    """
    Apparent wind speed actually flown during phase 3 [m/s]. Anchors the 1/v_app
    gain schedule and sets the attractor-lead kinematics; both only hold while
    it matches the measured average.
    """
    v_app_ref = 27.0
    "Lower clamp on v_app, limits the gain boost [m/s]"
    v_app_min = 10.0
    """
    Factor on `heading_p` during the ENTRY phases (dive and hold); phase 3 flies
    at the full gain. The entry is turn-rate limited, so detuning it costs no
    tracking and takes the command off its clamp.
    """
    entry_gain = 0.25
    """
    Depower held during the ENTRY phases [-]; the park and phase 3 fly at
    `depower_setpoint`. Higher than the pattern's, it lowers `c1` and unloads the
    wing, bleeding energy the dive converts out of height. The park is excluded:
    changing the tape there would inject the transient the park exists to decay.
    """
    entry_depower = 0.34

    # ---- Feedback: heading when slow, course when fast, on |vel_kite| ------- #
    "[m/s] at/below: pure heading feedback"
    v_kite_heading = 5.0
    "[m/s] at/above: pure course feedback; linearly blended in between"
    v_kite_course = 10.0
    """
    In FIG8 mode (phase 3), feed back COURSE alone and ignore the `v_kite_*`
    schedule; the entry phases keep it. Path following is a course problem, and
    on the pattern the schedule asks for course anyway — it only dips into the
    band during the slow part of a turn, swapping the feedback signal
    mid-manoeuvre. `false` restores the pure speed schedule in every phase.
    """
    fig8_pure_course::Bool = false
    """
    Steering command limit [-]. Raising it to relieve clamp saturation is
    CLOSED: at 0.33 the loop goes violently unstable and at 0.375 the PLANT
    itself diverges in bang-bang oscillation with no controller at all. `c1` is
    linear over the range, so this is a real dynamic limit of the plant at this
    depower, not a modelling artefact.
    """
    max_steering = 0.32

    # ---- Entry descent limiter, active only while far off the path --------- #
    """
    Steepest commanded course while off-path [deg]. 90 is constant elevation,
    above 90 descending. Set to 180 to disable the limiter entirely.
    """
    entry_chi_max = 95.0
    "Cross-track error [deg] below which the limiter is bypassed"
    entry_d_gate = 12.0
    """
    Width of the band [deg] ABOVE `entry_d_gate` over which the limited and raw
    courses are blended. 0 restores a hard switch, which stepped the command by
    the full clamp violation in one timestep and had the PID's D path turn that
    step into a sign reversal.
    """
    entry_d_blend = 4.0
    """
    How close to ±180° [deg] `chi_set` must be before its sign is treated as
    degenerate and the latched tangent sign is used instead
    """
    entry_cut_margin = 30.0

    """
    Abort guard: stop the run above this apparent wind speed [m/s], so an
    overspeed is reported as itself rather than as an opaque solver abort.
    """
    v_app_abort = 45.0

    # ---- Metrics window ---------------------------------------------------- #
    "Settle time [s] after `park_time` before the tracking statistics start"
    entry_time = 52.0
    "Elevation floor criterion [deg], evaluated over the WHOLE run"
    min_elevation = 10.0
    """
    Pattern-SIZE criterion: the mean per-lobe azimuth reach must be at least
    this fraction of `f8_a` on EACH side, and the elevation span this fraction
    of `f8_b`. Every other criterion is measured against the CLOSEST POINT of
    the path, so all of them pass on a kite flying a small eight — it is on the
    path, it just is not going anywhere on it.
    """
    min_span_frac = 0.7
end
"""
    FC_Settings(filename::String; path=skc_data_path()) -> FC_Settings

Load figure-eight flight-controller settings from the YAML file `filename` under
`path`, which defaults to this package's own [`skc_data_path`](@ref) rather than
`KiteUtils.get_data_path()` — the latter points at the *kite model's* data
directory during a run, and these settings belong to the controller. Pass `path`
explicitly to load a variant from elsewhere; an absolute `filename` is used
as-is.

The file must have a top-level `fc_settings:` mapping whose keys are the field
names of `FC_Settings`; any missing key falls back to the struct default, and an
unknown key is an error.
"""
function FC_Settings(filename::String; path = skc_data_path())
    dict = YAML.load_file(isabspath(filename) ? filename :
                          joinpath(path, filename))["fc_settings"]
    fcs = FC_Settings()
    for (key, value) in dict
        sym = Symbol(key)
        hasfield(FC_Settings, sym) ||
            error("Unknown key \"$key\" in $filename — not a field of FC_Settings.")
        setfield!(fcs, sym, convert(fieldtype(FC_Settings, sym), value))
    end
    return fcs
end

"""
    project_file(fcs::FC_Settings) -> String
    project_file(project::String) -> String

Path of the system project to hand to the kite model, preferring this package's
own [`skc_data_path`](@ref) over the model's data directory: the conditions a
figure-eight is flown under — wind, tether, winch, KCU, solver — are a choice of
the RUN, so `data/system_reelout.yaml` and the `data/settings_reelout.yaml` it
names live here and can be varied without editing the kite model.

Returns an absolute path when the project is bundled with this package. KiteUtils
resolves the `sim_settings` file relative to the project file's own directory, so
that settings file is taken from here too, while a bare name such as
`wc_settings` stays a lookup under the model's active data path. A project this
package does not carry is returned unchanged, which resolves it against the
model's data as before.
"""
project_file(fcs::FC_Settings) = project_file(fcs.project)
function project_file(project::String)
    path = joinpath(skc_data_path(), project)
    return isfile(path) ? path : project
end

"""
    winch_force_gains(fcs::FC_Settings) -> NamedTuple

Force-mode winch gains with the `compliance` scaling applied, as a NamedTuple
keyed to match a force-mode winch controller's fields
(`force_tau`, `len_kp`, `damp`, `force_min`). Splat it into whichever winch the
kite model provides:

    wfc = WinchForceController(; winch_force_gains(fcs)...)

`winch_len_kp` and `winch_damp` are both divided by `fcs.compliance`, so the
yield scales linearly with it while their ratio — the length loop's own time
constant — is unchanged. `winch_force_tau` is passed through untouched: it sets
WHICH frequencies the drum yields to, not by how much.

Plain numbers on purpose. The scaling is the part worth keeping in this package;
the controller object it feeds belongs to the kite model, which this package
does not depend on.

Errors at `compliance == 0`: that is position mode and must not be flown through
a force-mode winch (an infinitely stiff spring is not representable — see
[`FC_Settings`](@ref)).
"""
function winch_force_gains(fcs::FC_Settings)
    fcs.compliance > 0 ||
        error("winch_force_gains needs compliance > 0; at 0 use position mode.")
    return (force_tau = fcs.winch_force_tau,
            len_kp = fcs.winch_len_kp / fcs.compliance,
            damp = fcs.winch_damp / fcs.compliance,
            force_min = fcs.winch_force_min)
end
