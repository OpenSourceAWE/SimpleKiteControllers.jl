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
FORCE-mode winch parameters are here (`winch_*` below), because the force-mode
controller itself is part of the run rather than of the model — see
`examples/v3kite_support.jl`.

The dated record of how these values were arrived at — sweeps, reverted attempts
and the failures behind each closed lever — is in V3Kite.jl's
`docs/fig8_tuning_log.md`. Add new findings there, not here.
"""
@with_kw mutable struct FC_Settings @deftype Float64
    "System project file, see `data/system_*.yaml`"
    project::String = "system_reelout.yaml"
    """
    Total simulation time [s]; ~43 s per lap at v_app 13 m/s, plus the descent
    from the park. The metrics window opens at `park_time + entry_time`, so a
    30 s run scores only its last few seconds and `laps` is meaningless — judge
    short runs on RMS d, the elevation floor and the tape metrics, and use 150
    for anything that has to be counted in laps.
    """
    sim_time = 150.0
    """
    Simulation timestep [s]. NOT a tuning parameter: 0.05/3 was numerically
    unstable here. `step!` holds the VSM aero load frozen inside the DAE between
    updates, and that explicit coupling develops a growing 2*dt (30 Hz)
    structural oscillation at maximum dynamic pressure — measured at the bottom
    of the right lobe, ~3.2 kN, which ended a run in one timestep. Halving dt
    halves the aero lag and gives ~4x margin on the mode, at 2x wall time.
    """
    dt = 0.05/6
    """
    Steps between VSM aero updates; the load is held frozen inside the DAE in
    between (0 disables the update entirely). 1 is the TIGHTEST coupling
    available, so this can only be raised — trading aero lag for wall time.
    Exposed to sweep the coupling mode described under `dt`, not as a lever that
    can stabilize it.

    NOTE: V3Kite's released `step!` hardwires 1 and takes no `vsm_interval`
    keyword, so `examples/simple_fig8.jl` can only honour the default and errors
    on any other value rather than silently ignoring it. The field is kept
    because `dt` above is argued against it.
    """
    vsm_interval::Int64 = 1
    "Ground wind speed at reference height [m/s]"
    v_wind = 5.0
    """
    How soft the winch is [-].

    WHAT IT SCALES. In force mode the drum holds a low-passed reference force,
    and a load above that reference stretches the length trim like a spring: the
    steady-state yield is dF / winch_len_kp, so the compliance IS 1/winch_len_kp
    [m/N]. The knob divides both `winch_len_kp` and `winch_damp` (below) by
    `compliance`, so yield scales linearly with it while their ratio — the length
    loop's own time constant — is left alone. The result is a softer or stiffer
    winch of the SAME character, not a different one. `winch_force_tau` is
    untouched: that sets WHICH frequencies the drum yields to, not by how much.
    Values above 1.0 are meaningful (2.0 = twice as soft); the useful range is
    bounded by the runaway recorded under `winch_damp`, where force mode with no
    damping at all lost the run at t = 19.8 s.

    AT EXACTLY 0 the controller changes, because an infinitely stiff spring is
    not representable: the run switches to POSITION mode (`set_length = l0`) and
    hands the winch to V3Kite's own `winch_position_torque!`, whose force
    feed-forward cancels the measured load exactly — the drum sees nothing to
    accelerate it and travels 0.009 m over a 30 s run, i.e. holds a constant
    unstretched tether length. The limit is continuous in BEHAVIOUR
    (`compliance` 0.01 already means winch_len_kp = 10 kN/m) but not in code
    path, so a sweep should not expect the two sides to agree to the millimeter.
    """
    compliance = 0.5
    """
    Initial tether length [m], held more or less constant. Tested in the range
    150 m to 300 m. The minimum angular turn radius is rho = 1/(L*c1*u_s), so a
    LONGER tether lets the kite turn tighter in angular terms — the most
    effective lever on pattern feasibility after c1 itself.
    """
    tether_length = 200.0
    """
    Depower setting held during the run [-]. Sets the operating point of the
    turn-rate law.
    """
    depower_setpoint = 0.26
    """
    Settling elevation [deg] = the natural parked equilibrium, so the kite
    starts where it wants to be.

    NOTE: this currently has NO EFFECT on where the run starts. `settle_wing`'s
    cache key does not include the settling elevation, so the existing 73°
    geometry is reused (verified: logged elevation at t = 0 is 73.0°). Forcing
    `remake=true` would overwrite a cache shared with simple_sinus.jl /
    simple_parking.jl. Fixing it means adding the elevation to the cache key in
    `stabilization.jl` — see V3Kite.jl's PlanFig8.md, Findings 4. Left in place because
    starting on the pattern is the right way to develop the pattern controller
    once the key is fixed.
    """
    elevation = 73.0
    """
    Parking phase [s]: hold zero steering so the transients left by
    init/settling decay before the controller starts demanding maneuvers.
    Without it the guidance engaged at t=0 and drove the steering straight to
    its clamp while the model was still relaxing. The guidance still runs during
    the park (its course estimate is low-passed and needs warming up), but its
    output is not applied.
    """
    park_time = 2.0
    """
    Warm-up [s], run right after `init` and discarded (see `warmup!` in
    `examples/v3kite_support.jl`). The park above lets the settling transients
    decay; this lets them decay BEFORE t = 0, so they are not in the log at all.
    They are not the run's data: `settle_wing` returns an equilibrium of the
    settling model (dt = 0.001, damped, winch braked) and the first second of the
    run is that state relaxing into an equilibrium of the model actually being
    integrated — the brake released, the drum taking up the load at its own
    torque, the aero applied at the run's dt. It showed up most sharply in the
    logged L/D, which is a ratio of two forces that both dip while the wing is
    unloaded. Costs `warmup_time / dt` full steps of wall time (240 at 2 s), and
    must be long enough to cover the decay — the transient under investigation
    peaked at t = 0.66 s. 0.0 disables it.
    """
    warmup_time = 2.0

    # ---- Force-mode winch (see `examples/v3kite_support.jl`) ---------------- #
    # Only used when `compliance > 0`; at 0 the run hands the winch to the kite
    # model's own POSITION controller and none of these are read. They are
    # BASE values: `winch_len_kp` and `winch_damp` are divided by `compliance`
    # before the controller is built.
    """
    Time constant of the low-pass that turns the measured winch force into the
    reference force [s]. The drum yields to everything faster than this and holds
    everything slower, so this is the winch's compliance TIMESCALE — it should
    sit above the lap rate to let the kite trade line length for speed within a
    lap. Not scaled by `compliance`: it sets which frequencies the drum yields
    to, not by how much.
    """
    winch_force_tau = 10.0
    "Length-trim gain, length error [m] → reference force [N/m], before `compliance` scaling"
    winch_len_kp = 100.0
    """
    Viscous damping on the drum, reel-out speed [m/s] → reference force [N·s/m],
    before `compliance` scaling. Without it force mode has no velocity feedback
    at all — the drum is a free mass on the `winch_len_kp` spring and its speed
    runs away (measured: 3.5 m/s of payout, `v_app` 49.6 m/s, run lost at
    t = 19.8 s). This is what makes the winch compliant rather than
    free-wheeling; it opposes fast reel-out without pinning the length the way
    the position cascade does.
    """
    winch_damp = 500.0
    "Floor on the reference force, keeps the tether taut [N]"
    winch_force_min = 100.0

    # ---- Entry state machine: park -> dive -> hold -> fig8 ------------------ #
    # Modelled on the working controller's log (SmallPlan.md, "Reference run").
    # That controller does NOT let the path guidance fly the descent: from the
    # park it commands a near-horizontal course open loop and lets the kite fall
    # along the sphere (no attractor at all — the logged attractor is NaN until
    # handover), flattens out for the last second, and hands over at the
    # pattern's RIGHTMOST point, at the centre elevation, already moving
    # downwards into the first turn.
    #
    # Reference timings (its park is 10 s, ours 5 s): dive 5.6 s covering
    # 71 -> 42° of elevation (~5.2°/s), hold 1.2 s covering the last 42 -> 27°
    # (the kite is fastest here, so this is the steepest part), handover at the
    # centre elevation.
    """
    Course commanded during the dive [deg]. |chi| > 90 is descending, |chi| < 90
    climbing, 90 exactly horizontal. SIGN, measured: a POSITIVE commanded course
    drives the kite towards NEGATIVE azimuth, and the reference enters at the
    pattern's rightmost point, so the command is negative here.
    """
    chi_dive = -85.0
    """
    Course commanded during the hold [deg]: exactly horizontal, i.e. stop
    descending and let the kite arrive at the pattern flat rather than diving
    into it.
    """
    chi_hold = -90.0
    """
    Margin above `el_center` [deg] at which the dive ends and the hold begins
    (reference: 42° vs a 26° centre = 16°)
    """
    dive_el_margin = 7.0
    "Duration of the hold [s], from the reference log"
    hold_time = 0.8

    """
    In-plane body damping. A FLIGHT parameter here, not just a solver setting:
    it sets c1 and hence the achievable turn radius (see the `simple_fig8.jl`
    docstring). `init`'s default [0,0,40] is the most agile and the only one that
    flies this pattern inside the identified steering range. Raising it costs
    turn authority; it buys a smaller parked AoA ripple and ~3.4x fewer solver
    steps (see `init`).
    """
    body_damping::Vector{Float64} = [0.0, 0.0, 40.0]

    # ---- Pattern geometry [deg] -------------------------------------------- #
    # Sized by the turn-radius argument in the simple_fig8.jl docstring; check
    # the feasibility margin printed at startup before changing these. Note a
    # SMALLER lemniscate is a TIGHTER one: the reference controller's 40/15 drops
    # the margin to 1.02 and does not fly here.
    "Width of the eight [deg] (azimuth spans +-`f8_a`)"
    f8_a = 40.0
    "Height of the eight [deg] (elevation spans +-`f8_b`/2)"
    f8_b = 15.0
    "Size of the right part [deg]"
    f8_c = 0.0
    "Asymmetry factor [-]"
    f8_d = 0.0
    """
    Pattern-centre elevation [deg]; spans 16-36° at `f8_b` = 20. The reference
    controller's centre, and the lowest one flown here. Two forces pull opposite
    ways: a lower centre IMPROVES the curvature margin (less cos(elevation)
    compression of the azimuth axis) but pushes the pattern deeper into the
    power zone, and every failure at low centre has been an ENERGY failure
    (v_app and force run away while the tracking still looks fine).
    """
    el_center = 26.0
    "Arc distance Q -> attractor [deg]"
    attractor_dist = 10.0
    """
    Fly UP-loops instead of down-loops. With `false` (down-loops) the kite
    passes the azimuth extreme moving downwards at large |azimuth|. The flag
    reverses the traversal direction of the reference path (`_build_path` in
    `src/figure_eight_controller.jl`); the path shape itself is unchanged, so the
    curvature feasibility margin is unaffected. Down-loops convert height into
    speed where up-loops shed energy through the turn, so they were unflyable on
    the old heading loop; on course feedback they are the only configuration
    that crosses the centre instead of circling one lobe.
    """
    up_loops::Bool = false

    # ---- Heading PID ------------------------------------------------------- #
    # Output is rel_steering (dimensionless, -1..1), fed UNNEGATED: positive
    # rel_steering produces a positive heading rate on this plant (measured,
    # r = +0.998 — see src/figure_eight_controller.jl).
    """
    Gain at v_app == `v_app_ref`, i.e. the gain actually applied in phase 3,
    since `v_app_ref` is the phase-3 flight speed. Was 0.1747 against a
    `v_app_ref` of 30, and 0.4 against 13.1 before that; all three are the SAME
    applied gain (0.1941*27 = 0.1747*30 = 0.4*13.1 = 5.24) — only the anchor
    moved, the flown loop is unchanged.

    Stability bound, for context: the plant psi_dot = c1*v_a*u_s is an INTEGRATOR
    of gain c1*v_a = 3.66 rad/s per unit u_s at flight speed, so the crossover is
    omega_c = K*3.66. Against the 0.72 s measured tape lag a delay needs
    omega_c*T_d <~ 0.8 rad, giving K <~ 0.46; the optimistic 0.383 s small-signal
    dead time gives 0.86. The flown 0.194 sits a factor of roughly 2.5 inside the
    tighter bound — it is what flies well, not what the bound permits.

    The earlier 4.5 (old `v_app_ref` = 13.1 anchor, so ~1.97 applied) was ~8x
    over gain, i.e. a relay: it clamped at 6.7° of course error, exceeded by 88%
    of phase-3 samples, and the kite turned at a median 43.5 deg/s against the
    8.3 deg/s the guidance asked for. Everything measured at that gain describes
    a self-oscillating loop, not tracking.
    """
    heading_p = 0.1941
    """
    Integral time [s], or `false` for no integral action (the default): a steady
    heading bias shows up as a steady cross-track error, which the guidance
    itself already corrects by pulling the attractor back onto the path. Try a
    finite Ti only if a persistent one-sided cross-track offset remains.
    """
    heading_i::Union{Bool, Float64} = false
    "Derivative time [s], damps the initial transient"
    heading_d = 0.12
    """
    Derivative filter: maximum gain of the D path, which is K*Td*s/(1 + s*Td/N).
    Flat at K below N/(2*pi*Td) Hz, rising to N*K above it. 2 rather than the
    DiscretePIDs default of 10: the fed-back angles carry broadband noise, and at
    N = 10 the rising D gain amplified it into a 7.95 Hz ripple on the command.
    At the loop's own 0.1 Hz the D path contributes a gain of 1.005 and 5.4° of
    phase lead either way, so this is a filter change, not a gain change: same
    flight, 33% less peak tape slew.
    """
    heading_d_n = 2.0
    """
    Apparent wind speed actually flown during phase 3 [m/s], the average at the
    conditions configured here. Serves two roles, and they agree only because
    this is the real speed: it anchors the 1/v_app gain schedule (so `heading_p`
    reads as the gain the kite really flies at), and it sets the kinematics (how
    long the kite needs to cover a given arc). Only the product
    `heading_p * v_app_ref` is physical, so the 30 -> 27 correction was paired
    with the inverse scaling of `heading_p` above: same flight, the anchor is now
    the measured average. Was 13.1 before that, a parking speed carried over from
    simple_auto_parking.jl that the kite never flies here; that anchor also
    overstated the attractor lead.
    """
    v_app_ref = 27.0
    "Lower clamp on v_app, limits the gain boost [m/s]"
    v_app_min = 10.0
    """
    Factor on `heading_p` during the ENTRY phases (dive and hold, phases 1-2);
    phase 3 flies at the full gain. The entry is turn-rate limited — the steering
    command sits on its clamp for the whole dive — so detuning the loop there
    costs nothing in tracking and takes the command off the clamp, which is the
    only way to shape the descent from the loop side.
    """
    entry_gain = 0.25
    """
    Depower held during the ENTRY phases (dive and hold, phases 1-2); the park
    and phase 3 fly at `depower_setpoint`. A higher depower than the pattern's is
    the second lever on the descent: it lowers c1 (less turn authority, which the
    entry does not need — it is clamp-limited anyway) and unloads the wing,
    bleeding some of the energy the dive from the 73° park converts out of
    height. The park is excluded on purpose: `init` settles at
    `depower_setpoint`, and changing the tape during the park would inject
    exactly the transient the park exists to let decay. Both transitions are rate
    limited by the KCU tape speed inside `step!`, so the change is a ramp, not a
    step.
    """
    entry_depower = 0.34

    # ---- WHAT THE LOOP REGULATES: heading at low KITE speed, course at high -- #
    # The guidance commands a COURSE, so course is the signal that actually
    # closes the path-following loop and is what must be regulated once the kite
    # is flying fast. When the kite is slow it is the wrong feedback: the
    # flight-path direction is undefined at zero velocity and noisy just above
    # it, while heading stays clean and still has the right sign of steering
    # authority. Regulating course throughout also asks the loop to chase the
    # drift angle, which the kite cannot change directly.
    #
    # Scheduled on |vel_kite|, NOT on v_app. A parked V3 already sees v_app ~ the
    # ambient wind, so apparent wind speed cannot tell "flying" from "hanging
    # still" — measured on this configuration:
    #
    #     signal          park          flying (t >= 15 s)
    #     v_app           9.1 m/s       21.1 m/s, never below 10
    #     |vel_kite|      4.2 m/s       15.5 m/s (8.3 .. 22.3)
    #
    # A 10 m/s threshold on v_app therefore puts the PARK at blend weight 0.82 —
    # nearly full course feedback on a kite that is barely moving, which is the
    # one case the rule exists to prevent. On |vel_kite| the park is
    # unambiguously heading, and the weight modulates in flight when the kite
    # slows through a turn, which is when the course estimate is worst.
    #
    # Below v_kite_heading the error is formed from heading alone, above
    # v_kite_course from course alone, and linearly blended in between. The band
    # exists to avoid a hard switch: heading and course differ by the drift angle
    # (~13-15° on the V3), so a step change of feedback signal at one speed would
    # step the steering command by heading_p * drift. Widen it if that shows.
    "[m/s] at/below: pure heading feedback"
    v_kite_heading = 5.0
    """
    [m/s] at/above: pure course feedback ("high" per the flight note in
    SmallPlan.md; the lower edge is a choice — 5 m/s is just above the 4.2 m/s
    the kite drifts at during the park). With `fig8_pure_course` on, this band
    governs the ENTRY only.
    """
    v_kite_course = 10.0
    """
    In FIG8 mode (phase 3), feed back COURSE alone and ignore the `v_kite_*`
    schedule; the entry phases keep it. This is SmallPlan.md's "gate on phase
    instead of speed" option. Rationale: path following is a course problem, and
    on the pattern the kite is fast enough that the schedule asks for course
    anyway — it only dips into the band during the slow part of a turn, swapping
    the feedback signal mid-turn for no benefit. `false` restores the pure speed
    schedule in every phase.
    """
    fig8_pure_course::Bool = false
    """
    Steering command limit [-]. Raising it to relieve the clamp saturation is
    CLOSED: at 0.33 the loop goes violently unstable (diverged at t = 30.9 s,
    peak turn rate 949 deg/s) and at 0.375 the PLANT itself diverges in bang-bang
    oscillation with no controller at all. c1 is linear over the range, so this
    is a real dynamic limit, not a modelling artefact — the usable authority
    ceiling is a property of the plant at this depower. The remaining levers
    change the operating point: reel-out or a 300 m tether. See V3Kite.jl's PlanFig8.md.
    """
    max_steering = 0.32

    # ---- Entry descent limiter --------------------------------------------- #
    # WHY: without it the kite dove straight from the 73° park to the pattern,
    # converting 40° of potential energy into a 3.3x overspeed — v_app 15.7 -> 51
    # m/s in 7 s, AoA driven negative as the wing unloaded, and the solver aborted
    # at t=7.35 s. The guidance was working (cross-track error 35° -> 1.7°); the
    # descent was simply flown far too steeply.
    #
    # The fix limits the COMMANDED course, not its rate: while the kite is far off
    # the path, never command a course steeper than entry_chi_max (90° = constant
    # elevation, >90° = descending), so the approach is a shallow glide that drag
    # can bleed instead of a plunge. It also cures a second defect seen in the
    # same run: with the attractor nearly straight below, chi_set hunted across
    # the ±180° branch cut (+154.8° -> -155.2° -> -153.3°) and the steering
    # chattered between its clamps. Picking whichever of ±entry_chi_max needs the
    # smaller heading change makes that choice continuous, with no latch or state
    # machine.
    #
    # The limiter is gated on the cross-track error and self-disables: the pattern
    # itself legitimately requires steep courses (chi_set = -118° on the path at
    # the lobe crossing), so once |d| < entry_d_gate the raw guidance course
    # passes through untouched. Set entry_chi_max = 180 to disable the limiter
    # entirely.
    #
    # The handover is BLENDED over entry_d_blend, not switched. As a hard gate it
    # stepped the commanded course by the full clamp violation in one timestep,
    # and the PID's D path turned that step into a spike that reversed the sign of
    # the command — see the tuning log, ENTRY_D_BLEND.
    """
    Steepest commanded course while off-path [deg]. At 105° the descent from the
    73° park still reached 45.6 m/s by elevation 40°; 95° is only 5° below the
    local horizontal, so the kite spirals down slowly enough for drag to bleed
    the energy it gains.
    """
    entry_chi_max = 95.0
    "Cross-track error [deg] below which the limiter is bypassed"
    entry_d_gate = 12.0
    """
    Width of the band [deg] ABOVE `entry_d_gate` over which the limited and raw
    courses are blended: fully limited at d >= gate + blend, fully raw at
    d <= gate. 0 restores the old hard switch. Sized against the rate d closes at
    (~3.4 deg/s here), so 4° is ~1.2 s of traversal — 16x slower than the 0.075 s
    derivative filter, hence tracked rather than differentiated (D contribution
    ~0.05 instead of the +0.73 the step produced). It also makes CHATTER on the
    gate harmless: d is not monotonic, and a hard switch re-fires the full step on
    every recrossing.
    """
    entry_d_blend = 4.0
    """
    How close to ±180° [deg] chi_set must be before its sign is treated as
    degenerate and the latched tangent sign is used instead
    """
    entry_cut_margin = 30.0

    """
    Abort guard: stop the run above this apparent wind speed [m/s]. The first
    run's failure showed up as an opaque solver `dt_epsilon` abort; catching the
    overspeed that causes it reports the actual problem instead.
    """
    v_app_abort = 45.0

    # ---- Metrics window ---------------------------------------------------- #
    # The park plus the time allowed to settle onto the pattern before the
    # tracking statistics start.
    "Settle time [s] after `park_time` before the tracking statistics start"
    entry_time = 52.0
    "Elevation floor criterion [deg], evaluated over the WHOLE run"
    min_elevation = 10.0
    """
    Pattern-SIZE criterion: the mean per-lobe azimuth reach must be at least this
    fraction of `f8_a` on EACH side, and the elevation span this fraction of
    `f8_b`. Every other criterion is measured against the CLOSEST POINT of the
    path, so all of them pass on a kite flying a small eight, or one lobe in half
    the wind window — it is on the path, it just is not going anywhere on it.
    Sized against the flown spans on record: the reference run holds azimuth
    -43.5..+42.2° against A = 40 (fill 1.06/1.09) and an elevation span of 19.9°
    against B = 15 (1.33), so 0.7 has real margin against a good run while a
    degenerate one lands far below it.
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
