# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Settings of the figure-of-eight FLIGHT CONTROLLER flown by
`examples/simple_fig8.jl`: the simulation conditions, the pattern geometry, the
entry state machine, the heading/course PID and the metrics window. Loaded from
a YAML file (`fc_settings.yaml`) rather than hard-coded in the example, so a
sweep can vary them without editing the script.

Everything here is a TUNING parameter of one run. The PLANT the run is flown
against — including the wind speed (`v_wind`), the length of the run
(`sim_time`) and the timestep it is integrated with (`sample_freq`) — is the
system project resolved by
[`project_file`](@ref), i.e. `data/settings_fig8_200m.yaml`. Both winches' gains
are `data/wc_settings.yaml` (one `WCSettings` struct since the 2026-08-16 merge:
V3Kite's POSITION-mode torque gains and WinchControllers.jl's speed-controller
tuning, this package's file) and the geometry is the model's own; the
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
    Steps between VSM aero updates, held frozen in between (0 disables them).
    1 is the tightest coupling available, so this can only be raised, trading
    aero lag for wall time. Passed straight to V3Kite's `step!`.
    """
    vsm_interval::Int64 = 1
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

    # ---- REEL_OUT winch; mutually exclusive with `compliance > 0` ----------- #
    "Tether length [m] at which reel-out stops and the run holds the final length"
    reelout_l_max = 250.0
    """
    Number of complete figures of eight after which reel-out stops [-], the second
    stop criterion beside [`reelout_l_max`](@ref); whichever is reached first ends
    reel-out and enters phase 5. `0` disables it, leaving `reelout_l_max` the only
    criterion (the pre-change behaviour). Counted from the same lap counter the log
    records as `SysState.fig_8`, which starts at phase 4, so the partial lap flown
    between reel-out engaging (phase 3 + `reelout_delay`) and the pattern settling
    is not counted, and a run that never reaches phase 4 never fires this criterion.
    NOTE that `depower_final` is tuned for the force at a full 150 -> 350 m reel-out;
    stopping short leaves a different length and a different force, so a lap-limited
    run may want its own `depower_final`.
    """
    n_fig_eight::Int64 = 0
    """
    Depower flown once `reelout_l_max` is reached (phase 5, "final") [-]. Reel-out
    used to bleed off part of a force excursion by reeling out line instead of
    holding it; once the length is frozen, `step!`'s POSITION loop can only
    resist a load spike with more torque against a now-static setpoint, so a
    depower left at `depower_setpoint` runs noticeably tenser than during
    reel-out — confirmed nonlinear and much larger than expected, `docs/fig8_tuning_log.md`
    ("`depower_final` sweep"). Tuned for `reelout_l_max = 350` at 150 -> 350 m,
    6 m/s wind: a different length, wind speed or `depower_setpoint` moves the
    target force and needs its own sweep, not a linear rescale of this value.
    """
    depower_final = 0.328
    """
    Soft-start time [s]: the commanded reel-out speed (both `v_ff` and the
    `l_set` integration, i.e. the actual reel-out rate) is ramped linearly from
    `0` to WinchControllers.jl's `v_set = kv * sqrt(force)` over this many
    seconds, counted from the moment reel-out engages (`reelout_delay` after
    phase 3). `0` disables the ramp (the command jumps straight to the computed
    value, the pre-2026-08-15 behaviour). Only the value USED is scaled — the
    winch controller's own internal state (integrators, force limiters) still
    sees the true, unscaled law throughout, so this shapes the ENGAGEMENT
    transient only and does not lag the steady-state force regulation the way
    filtering the law itself would (`docs/fig8_tuning_log.md`, "Lagging the
    square-root law is CLOSED"). Distinct from WinchControllers.jl's OWN
    `WCSettings.t_startup` (`data/wc_settings.yaml`), which only holds
    ITS force limiters in reset for a while and does not ramp anything — see
    that file's comment on the key.
    """
    reelout_softstart = 0.0
    """
    Soft-stop trigger [s], the mirror image of [`reelout_softstart`](@ref): once
    the remaining reel-out would finish within this many seconds AT THE CURRENT
    RATE, `v_set` decelerates LINEARLY from whatever it is at that instant to 0,
    landing exactly at `reelout_l_max` — continuous with the speed already being
    flown, so unlike a hard stop there is no discontinuity for the POSITION loop
    to correct with a reel-IN transient (a power undershoot,
    `docs/fig8_tuning_log.md`, "soft-stop"). A linear ramp's area is
    `v_entry*T/2`, so the ACTUAL deceleration time is solved exactly from the
    remaining distance and comes out around 2x this value, not equal to it. `0`
    disables it (the pre-2026-08-16 hard stop). Loosening `step!`'s
    `acceleration_limit` instead was tried and made the undershoot WORSE: it
    lets the drum coast further past `reelout_l_max` before turning around, so
    the position error the loop then corrects is bigger, not smaller.
    """
    reelout_softstop = 0.0
    """
    Delay [s] between the guidance engaging (phase 3) and reel-out starting.
    `0` reels out from the first step of phase 3.

    It buys energy per metre, not power: reel-out ends at `reelout_l_max`, so the
    window is set by the metres to reel out and not by when it opens, and the
    metres flown before the pattern settles are the cheap ones. Measured at 6
    m/s wind, 150 -> 350 m: engaging at phase 3 gave 2.32 kJ/m against 2.49 kJ/m
    when the winch waited for phase 4 (mean power 6.5 vs 7.2 kW over a window
    that barely moved, 71.4 vs ~69 s). Delaying past the settling recovers that;
    delaying further only shortens the run's harvest.
    """
    reelout_delay = 0.0
    """
    Force [N] at which reel-out engages REGARDLESS of `reelout_delay`, latching
    once and never re-closing. `Inf` (the default) disables it, leaving reel-out
    purely on the timer.

    `reelout_delay` trades harvest for settling, which is a fair trade at moderate
    load and a bad one when the entry swoop is loading the tether: the winch is
    commanded to exactly zero while the force builds, so no force limiter of any
    kind is running. Measured at 10 m/s with `reelout_delay` 4.0 s: force reached
    9811 N by the moment the timer released the drum and peaked at 12365 N 0.8 s
    later, 47 % over the plant's 8400 N rating. That peak is untouchable by winch
    tuning — `force_limit`, `force_limit_tau` and a fast attack were all measured
    and none of them moves it, because the controller has no output at all until
    this gate opens.

    Set it below the force the entry reaches but above anything a settled pattern
    produces, so a normal run still gets its full settling delay.
    """
    reelout_f_trigger = Inf

    # ---- Entry state machine: park -> dive -> hold -> transition ----------- #
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
    Cross-track error [deg] below which phase 3 (transition) advances to phase 4
    (fig8), the first time it is crossed. A milestone in the log only —
    it does not change how the kite is flown.
    """
    fig8_d_gate = 5.0

    """
    Body damping settling STARTS from, per axis. It shapes the settling
    transient and the settled geometry it converges to; V3Kite's `init` decays
    it over the settling run to its `min_damping` floor (0.8 x this by default),
    and that floor is the damping the run is actually FLOWN with. Both are fixed
    by this one value, so it is the key [`turn_rate_coeffs`](@ref) is looked up
    with, and what sets `c1` and hence the achievable turn radius.
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
    """
    Learning gain for the elevation bias, `0` to switch the correction off.

    The kite flies BELOW the path it is given — measured over every segment of
    every reel-out run, 1-2 deg and up to 3.5 deg at 380 m. Once per lap the mean
    signed elevation error against the OPTIMIZER's curve is taken (the measured
    sag plus the correction already in the flown path) and the correction is moved
    by this fraction of it, so a path installed afterwards is raised by the
    correction and the kite lands on the curve the optimizer solved for. The sag
    itself does not shrink: raising the reference raises the kite with it, which
    is why closing on the sag instead integrates to `el_bias_max`. `1` corrects the whole measured bias in one lap, which fights the
    re-optimization rewriting the reference in the same lap; 0.5 is a lap of
    settling per correction. Only `examples/simple_opt_reelout.jl` applies it:
    the correction rides on a path INSTALL, and that is the only run that has any.
    """
    el_bias_gain = 0.0
    """
    Learning gain from state 5 (final) on, where the sag deepens as the winch
    stops and only a lap or two is left to learn in. Defaults to `el_bias_gain`,
    i.e. no change unless it is set.
    """
    el_bias_gain_final = el_bias_gain
    "Clamp on the learnt elevation correction [deg], per bin; guards a diverging estimate."
    el_bias_max = 3.0
    """
    Azimuth bands the elevation correction is learnt over, `1` for one number for
    the whole pattern.

    The sag is not uniform — measured at 380 m, the bottom of the pattern tracks
    ~2.3 deg under its reference where the crossing tracks ~0.4 deg under — so a
    single number is the mean of a profile and lifts the crossing it does not need
    to lift. With more than one band the same per-lap update runs per band, over
    `|azimuth|` as a fraction of the path's own amplitude
    ([`azimuth_bin`](@ref)), and what is applied is the smooth interpolation of
    [`bias_lift`](@ref) rather than a staircase.

    This LEARNS what `el_offset_wing` fixes in advance, and the two compose: the
    lobe lift is baked into every installed path and the learner closes on the
    optimizer's curve plus it. Bands cost samples — one lap is ~2-3 s per band at
    5 bins — so raise `el_bias_bins` and `el_bias_smooth` together.
    """
    el_bias_bins::Int = 1
    """
    Neighbour coupling applied to the learnt profile after each lap, `0` leaving
    the bands independent and `1` replacing each by the mean of its neighbours.

    A shape constraint only — [`smooth_bins`](@ref) restores the profile's mean,
    so the rigid part of the correction is untouched. It is what keeps a band that
    saw few samples from stepping away from its neighbours: the reference the step
    lands in is scored by the curvature gate, which refuses a corner (see
    `el_offset_wing_blend`) and leaves the correction undelivered. Ignored when
    `el_bias_bins` is 1.

    It also flattens the profile the learner settles on, because it damps the
    shape every lap while the gain only closes a fraction of it. Measured offline
    against a noiseless quadratic sag (0.4 deg at the crossing, 2.3 deg at the
    lobe, 5 bands, `el_bias_gain` 0.5), as the RMS the profile still leaves:

        lambda   0     0.25  0.5   0.75
        RMS      0.10  0.21  0.32  0.40

    One band leaves 0.67 deg, so even the heaviest smoothing is worth more than
    the rigid shift. Tuning goes DOWN from here if the flown profile is steady and
    up if a band jumps.
    """
    el_bias_smooth = 0.25
    """
    Extra elevation [deg] the flown path is lifted by once reel-out ends, on top
    of the learnt correction. A setpoint move, not an error: the kite is asked to
    fly the pattern higher where height is clearance and there is no reel-out power
    left to trade for it. It cancels out of the learning, which keeps closing on
    the optimizer's curve plus this offset.

    It starts at the reel-out STOP LATCH, not at state 5: the run's lowest point
    falls between the two, and by state 5 the blend and the climb are a further
    few seconds away.
    """
    el_offset_final = 0.0
    """
    How long before reel-out ends the lift starts [s], `0` to start it at the end.

    Measured 2026-08-18: with `reelout_softstop` at 0 there IS no stop latch before
    state 5, and the run's lowest point lands 3.8 s into state 5, inside the blend
    that is still ramping the lift in. Reel-out has stopped by then, so the lift has
    to be anticipated instead: it goes in once the length left is under
    `v_reelout * el_offset_lead`. Once latched it stays, so a fluctuating reel-out
    speed near the threshold cannot chatter the reference. Costs a little reel-out
    power for those seconds, since the pattern rises while the tether still pays.

    It also has to be long enough to land the shift BEFORE state 5. From there on
    the in-air curvature gate scores with `depower_final`'s `c1`, 0.775x the
    pattern's on the V3, and refuses the same lift it passes a few seconds earlier —
    measured 2026-08-19, margin 0.83 at a lead of 8 s against 0.67 at a lead of 0,
    where every retry over the remaining 26 s is refused too and state 5 flies
    without the lift at all.
    """
    el_offset_lead = 0.0
    """
    Extra elevation added to the path's LOBES only [deg], `0` to add none.

    `el_bias` and `el_offset_final` shift the whole pattern rigidly, but the sag
    is not uniform: measured at 380 m, the bottom of the pattern tracks ~2.3 deg
    under its reference where the top is ~0.4 deg under. This adds a lift that is
    zero in the middle of the pattern and `el_offset_wing` where it sags, shaped
    by `el_offset_wing_mode` — over azimuth (`el_offset_wing_az` /
    `el_offset_wing_blend`) or over elevation (`el_offset_wing_depth`).

    The ramp width is not cosmetic, and it is not monotone — see
    `el_offset_wing_blend`. Both a narrow ramp and a very wide one are refused by
    the feasibility check; a ramp that starts just outside the crossing is free.

    Applied to every path the run installs, the startup one included, so it is
    part of the reference the kite is asked to fly rather than a correction the
    learner has to discover. Its lap mean is absorbed by `el_bias`'s fixed point
    (the learner converges on the optimizer's curve PLUS this profile's mean), so
    the two do not fight.
    """
    el_offset_wing = 0.0
    """
    Azimuth beyond which `el_offset_wing` is applied in full [deg].
    """
    el_offset_wing_az = 10.0
    """
    Azimuth over which `el_offset_wing` ramps in below `el_offset_wing_az` [deg].

    The lift is zero at `el_offset_wing_az - el_offset_wing_blend` and full at
    `el_offset_wing_az`, with a smoothstep between, so the reference has no
    corner.

    This is the parameter to get right, and it is NOT monotone. Measured
    2026-08-18 by sweeping it on the real 150 m optimized path (`el_offset_wing`
    1.5 deg, `el_offset_wing_az` 10 deg, plain margin 0.94):

        blend [deg]  2     4     6     7-10    11    12    14
        margin       0.16  0.43  0.87  0.94    0.70  0.52  0.45

    Both ends fail for their own reason. TOO NARROW and the ramp is a corner: at
    4 deg the tightest point of the whole pattern moves off the lobe (azimuth
    -18.4 deg, radius 4.1 deg) and onto the ramp itself (azimuth -6.4 deg, radius
    1.86 deg). TOO WIDE and the ramp reaches into the CROSSING, where the path
    already turns hardest — at 12 deg it starts at |azimuth| = 2 deg below the
    centre... i.e. inside it.

    The plateau is a ramp that starts just outside the crossing and is full by
    the lobes: with `el_offset_wing_az = 10`, a blend of 8 starts the lift at
    |azimuth| = 2 deg and costs nothing at all — 0.94 -> 0.94 at 150 m, 2.39 ->
    2.38 at 380 m, 1.86 -> 1.84 at `depower_final`. Keep
    `el_offset_wing_az - el_offset_wing_blend` in 0 … 3 deg when tuning either.
    """
    el_offset_wing_blend = 8.0
    """
    Which coordinate `el_offset_wing` is shaped over, and in what units
    `el_offset_wing_az`/`el_offset_wing_blend` are then read:

      * `"azimuth"` — over `|azimuth|` in degrees. Raises the whole lobe, top and
        bottom alike, i.e. translates the part of the path that turns hardest,
        which is why it costs no curvature margin.
      * `"azimuth_frac"` — the same, with both read as FRACTIONS of the path's own
        azimuth amplitude, so the profile keeps its place on a pattern that shrinks
        as the tether grows. Only worth having once a pattern shrinks past the
        threshold fixed in degrees; on the reel-out flown so far it does not, and
        the two are indistinguishable (tuning log, 2026-08-19).
      * `"elevation"` — over the depth below the path's own elevation centre, in
        half-spans (`el_offset_wing_depth`). Raises the BOTTOM only, which is
        where the sag is deepest — but it DEFORMS the pattern where the other two
        translate it, and what it squashes is the lobe. Measured and refused by
        the curvature gate at every useful lift; see the tuning log before
        spending a run on it.

    See [`lobe_lift`](@ref).
    """
    el_offset_wing_mode::String = "azimuth"
    """
    Depth at which the ELEVATION-shaped lift starts, in half-spans of the path's
    own elevation range: `0` = its centre, `1` = its lowest point, where the lift
    is always full. Ignored unless `el_offset_wing_mode` is `"elevation"`.

    Normalized rather than in degrees because the pattern shrinks as the tether
    grows — measured over one 150 -> 380 m run, an elevation half-span of 6.6 deg
    at 163 m against 3.2 deg at 380 m — so a fixed number of degrees below the
    centre covers a quarter of the pattern early in the run and three quarters of
    it at the end, which is the wrong way round.

    Raising it narrows the ramp, and the profile FOLDS the path at
    `1.5 * el_offset_wing >= (1 - el_offset_wing_depth) * half_span`: at 1.5 deg
    of lift and the 3.2 deg half-span above, that is `0.3`. The feasibility gate
    refuses what folds, but it refuses it at startup or at an install, which is a
    run lost — check the arithmetic before sweeping upwards.
    """
    el_offset_wing_depth = 0.0

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
    """
    Seconds over which `rel_depower` ramps to a new phase-ladder target
    (`entry_depower`, `depower_setpoint` or `depower_final`) instead of
    stepping to it, in [`calc_steering`](@ref). `0` restores the hard switch.
    """
    depower_blend_time = 4.0

    # ---- Feedback: heading when slow, course when fast, on |vel_kite| ------- #
    "[m/s] at/below: pure heading feedback"
    v_kite_heading = 5.0
    "[m/s] at/above: pure course feedback; linearly blended in between"
    v_kite_course = 10.0
    """
    From phase 3 on (transition and fig8), feed back COURSE alone and ignore the
    `v_kite_*` schedule; the entry phases keep it. Path following is a course problem, and
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
    Force floor [N] of the standalone entry guard (`guard_lfc`, phases 0-2 of
    the reel-out examples), which reels in to catch a tether sag during the
    depowered dive. Deliberately NOT `WCSettings.f_low`: that one is the
    reel-out limiter's floor and scales with the winch, while this one is
    bounded by what a DEPOWERED wing can pull and does not. A setpoint the
    entry cannot reach makes the guard's PID wind up and run the drum away —
    see `docs/fig8_tuning_log.md`, "`f_low` 350 -> 700 N".
    """
    entry_f_min = 350.0

    """
    Fraction of `WCSettings.f_high` the upper force limit is held at during the
    FIRST figure of eight (`fig_8 == 1`, from the first step at phase 4 until one
    full traversal of the reference path), restored to `f_high` for every lap
    after. The first lap is flown into an unsettled force state — the pattern is
    still converging onto the reference and the winch has just engaged — so it
    carries the run's worst force excursions; a lower ceiling there costs a little
    reel-out speed over one lap and takes the peak off the drum.

    Only bites under `force_limit = "soft"`, where `calc_vro_soft` reads
    `f_high` live, and it also shifts `force_release`, which blends over
    `f_low .. f_high`. Under `"hard"` the `UpperForceController` copied `f_high`
    into its own setpoint when it was built, so this does not move it. The
    STARTUP request's `f_max` moves with it too — that path is the one lap 1
    flies — while re-optimization replies, installed from lap 2 on, keep the
    nominal ceiling.

    `1.0` is OFF and is the default: the idea has not been measured, so a run
    gets the plain ceiling unless it asks for otherwise.
    """
    first_lap_force_frac = 1.0

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
unknown key is an error. Built on [`load_yaml_fields!`](@ref).
"""
function FC_Settings(filename::String; path = skc_data_path())
    load_yaml_fields!(FC_Settings(), filename, "fc_settings"; path)
end

"""
    load_yaml_fields!(obj, filename, section; path = skc_data_path()) -> obj

Set every field `section` (a top-level key in the YAML file `filename`) names
on the mutable struct `obj`, converting each value to the field's declared
type; `filename` is resolved under `path` unless already absolute. An unknown
key errors, a key the file omits leaves `obj`'s existing value (its struct
default, for a freshly constructed `obj`) untouched.

Purely reflective (`hasfield`/`setfield!`/`fieldtype` on `typeof(obj)`), so it
works on any mutable struct without `src/` depending on the struct's package.
[`FC_Settings`](@ref) is built on it. It also loaded WinchControllers.jl's
`WCSettings` in `examples/simple_reelout.jl` until the 2026-08-16 winch merge
gave that struct a single file reached through the project's `wc_settings:` key
(PlanWinchcontrol.md), which is V3Kite's `WC_Settings(filename)`'s job now.
"""
function load_yaml_fields!(obj, filename::AbstractString, section::AbstractString;
                            path = skc_data_path())
    dict = YAML.load_file(isabspath(filename) ? filename :
                          joinpath(path, filename))[section]
    T = typeof(obj)
    for (key, value) in dict
        sym = Symbol(key)
        hasfield(T, sym) ||
            error("Unknown key \"$key\" in $filename — not a field of $T.")
        setfield!(obj, sym, convert(fieldtype(T, sym), value))
    end
    return obj
end

"""
    project_file(project = "system_fig8_200m.yaml") -> String

Path of the system project to hand to the kite model, absolute when this package
carries it in [`skc_data_path`](@ref) and unchanged otherwise, which leaves it a
lookup under the active data path.

The absolute form is what makes `sim_settings` this package's file: KiteUtils
resolves it relative to the project. Bare names like `wc_settings` instead follow
the active data path, which `examples/simple_fig8.jl` also points here.

Not a field of [`FC_Settings`](@ref): which plant a run is flown against is not a
tuning parameter of the controller.
"""
function project_file(project::String = "system_fig8_200m.yaml")
    path = joinpath(skc_data_path(), project)
    return isfile(path) ? path : project
end

"""
    fc_settings(project = project_file()) -> String

Get the flight-controller (FC) settings filename from the system project,
analogous to `KiteUtils.wc_settings`. Returns the value of the `fc_settings`
field of the project's `system` section; `project` defaults to this package's
own [`project_file`](@ref) rather than `KiteUtils.PROJECT`.
"""
function fc_settings(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["fc_settings"]
end

"""
    turn_rate_coeffs_file(project = project_file()) -> String

Get the turn-rate table filename from the system project, the same way as
[`fc_settings`](@ref). Returns the value of the `turn_rate_coeffs` field of
the project's `system` section; present in every project.
"""
function turn_rate_coeffs_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["turn_rate_coeffs"]
end

"""
    winch_kv_table_file(project = project_file()) -> String

Get the winch kv table filename from the system project, the same way as
[`fc_settings`](@ref). Returns the value of the `winch_kv_table` field of the
project's `system` section; only reel-out projects carry it.
"""
function winch_kv_table_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["winch_kv_table"]
end

"""
    traj_opt_settings_file(project = project_file()) -> String

Get the trajectory-optimizer settings filename from the system project, the
same way as [`fc_settings`](@ref). Returns the value of the
`traj_opt_settings` field of the project's `system` section; present in every
project that flies against an externally optimized path (both fig8 and
reel-out).
"""
function traj_opt_settings_file(project = project_file())
    dict = YAML.load_file(project)
    dict["system"]["traj_opt_settings"]
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
