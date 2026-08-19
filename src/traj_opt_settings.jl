# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
    TrajOptSettings

Settings of a run flown along an EXTERNALLY optimized path
(`examples/simple_opt_fig8.jl`): where the AWETrim server is, the initial guess
the solve starts from, the solver knobs the API exposes, and what is done with
the path that comes back. Loaded from `data/traj_opt.yaml` the same way
[`FC_Settings`](@ref) and [`OptSettings`](@ref) are loaded from theirs — a run is
defined by a file, not by editing a script.

The conditions are NOT here: the wind comes from the system project's settings
file and the winch law from its `wc_settings`, both read off the same files the
plant is built from (`inflow_from_settings`, `winch_from_wc` in
`examples/awetrim_client.jl`). A path optimized for a wind the kite does not fly
in is not the path for the run.

The **initial guess is not a formality**, which is why it has its own fields here
rather than borrowing `FC_Settings`' `f8_a`/`f8_b`/`el_center`. Those size the
lemniscate that `simple_fig8.jl` and `simple_reelout.jl` actually FLY; here the
lemniscate is only a seed, and the two roles pull in different directions.
Measured 2026-08-18 at 150 m and 6 m/s: the reel-out pattern (20°/11° at 18°)
makes the solve fail to converge, while 30°/12°, or the same eight centred at
26°, converge — and to the same optimum, worth 6080 W, while the server's own
parametric guess converges to a different one worth 1431 W. The problem is
multi-modal, so the guess is a choice about the answer.
"""
@with_kw mutable struct TrajOptSettings @deftype Float64
    "Address of the AWETrim server"
    base_url::String = "http://127.0.0.1:8000"
    """
    Start a detached server with `bin/run_server start` when nothing answers
    `base_url`. `false` turns a missing server into an error, which is what a run
    against a server on another machine wants.
    """
    autostart_server::Bool = true
    "Name the optimization is registered under on the server"
    name::String = "simple_opt_fig8"

    # ---- The initial guess the solve starts from ------------------------- #
    "Width of the guess lemniscate; azimuth spans ±`guess_a` [deg]"
    guess_a = 30.0
    "Height of the guess lemniscate; elevation spans `guess_b` peak to peak [deg]"
    guess_b = 12.0
    "Size of the right part of the guess [deg]"
    guess_c = 0.0
    "Asymmetry factor of the guess [-]"
    guess_d = 0.0
    "Centre elevation of the guess [deg]"
    guess_el_center = 26.0
    """
    Points the guess is sent with. The reply comes back with as many points as
    were sent (measured; the server's own `n_points` does not enter), so this
    also sets the resolution of the optimized path before resampling.
    """
    guess_points::Int64 = 361

    # ---- Solver knobs the API exposes ------------------------------------ #
    """
    Power-tape length `l_dp` [m] the optimizer starts from, on ITS scale
    (`l_dp = 0.6 + 5*u_p`, 0.4 m longer than V3Kite's for the same `rel_depower`).
    Only a seed: the optimizer varies it, since it is in the server's default
    `optimization_params`. Pinning it to the flown depower makes the solve
    infeasible — see `docs/steering_depower.md`.
    """
    input_depower = 1.6
    "Regularization weight of the solve [-]"
    reg_weight = 1.0
    "Solver flag passed through to IPOPT"
    detect_simple_bounds::Bool = true

    # ---- What is done with the path that comes back ---------------------- #
    """
    Points the optimized path is resampled to before it is flown, equidistant in
    arc length ([`resample_path`](@ref)). Used as an UPPER bound: a reply with
    fewer points is left at its own resolution, because interpolating extra
    points onto a polyline concentrates each vertex's turn into one short segment
    and makes [`path_radius_profile`](@ref) report a far tighter pattern than the
    curve is.
    """
    resample_points::Int64 = 361
    """
    Skip optimizer requests that are recorded as having failed before.

    A failing solve is the expensive one — it runs to IPOPT's iteration cap,
    measured at 81 s against 1.5 s for a converged solve — and in blocking mode
    the simulation is held for all of it. The failures are reproducible and
    isolated in tether length, so `examples/awetrim_client.jl` records each one
    and refuses to send it again; see `OPT_FAILURE_CACHE` there for the file, and
    `clear_opt_failures()` to forget them.

    Set `false` to send every request regardless — after a server or AWETrim
    upgrade, say, when what used to fail may not any more.
    """
    opt_failure_cache::Bool = true
    """
    Smallest curvature margin ([`check_pattern_feasible`](@ref), path radius over
    the kite's minimum turn radius) the returned path must have at the tether
    length it is flown at. Below 1.0 the path is tighter than the kite can turn,
    and the run measures the steering clamp instead of the path. `0.0` disables
    the check and flies whatever came back.

    Since 2026-08-19 this is not only a gate. `min_turn_radius_request` converts
    it to metres — `margin/(c1*max_steering)`, 8.4 m at the shipped 0.74 — and
    sends it with the request, so the optimizer solves under the same limit the
    reply is about to be judged by and "PATTERN TOO TIGHT" is answered before the
    solve rather than after it. `0.0` therefore turns off the request and the
    gate together.

    What is sent is that number scaled UP, by [`turn_radius_headroom`](@ref) and by
    the lap's reel-out (10.1 m at the shipped numbers): the optimizer measures the
    path's radius up its own reel-out while the gate reads the same curve at the
    anchor, so an unscaled request is one the reply satisfies and the gate still
    refuses.
    """
    min_feasibility_margin = 1.0
    """
    Extra turn radius asked of the optimizer, as a factor on what
    [`min_feasibility_margin`](@ref) demands; `1.0` asks for exactly the gate's
    number, which is NOT enough.

    The request and the gate do not measure the same curve at the same radius, and
    two corrections are needed. The geometric one the run MEASURES per length
    (`reelout_anchor_ratio`: the optimizer constrains `R = r/|kappa|` at each node's
    own radius, which grows through the lap, while the run flies the curve at the
    anchor, tighter by `L/r` — 0.87 at 220 m, 0.92 at 380 m). This field is the
    rest, which is about the GATE and not the geometry:

    - `path_radius_profile` estimates the curvature by finite differences on the
      resampled ~99-point reply and reads ~5 % tighter than the exact value
      (10.81 m against 11.38 m on one reply, measured 2026-08-19);
    - the run adds `el_bias` and `el_offset_wing` before checking, and lifting the
      path compresses its azimuth axis by `cos(elevation)`.

    1.10 covers both with a little to spare. Raising it buys installs at the cost of
    a wider, less powerful pattern; lowering it towards 1.0 brings back replies that
    converge, satisfy their own constraint and are then rejected at margin ~0.63
    against a demanded 0.74 — which is what the first `use_step` run did.

    The startup `/init` gets this factor alone: there is no reply to measure the
    geometric one off yet. If the STARTUP path is refused for curvature, this is the
    number to raise.
    """
    turn_radius_headroom = 1.0
    """
    Ground clearance [m] the returned path must have at the tether length it is
    flown at ([`check_pattern_height`](@ref)); `0.0` disables the check.

    AWETrim constrains HEIGHT (50 m, `DEFAULT_LIMITS["height"]`) and does not
    constrain elevation at all — but it reaches that floor at the END of a lap's
    reel-out, so a curve installed as (azimuth, elevation) and flown at its anchor
    radius clears only `50 * r0/r_low` ≈ 42 m, whatever the anchor. This floor is
    therefore about the anchor radius, not about how low the kite gets: the run
    reels out and climbs through 50 m within the first lap. `data/traj_opt.yaml`
    ships 40.0 for that reason, and the flown minimum is reported separately.

    The shortest tether is the worst case, since a fixed elevation rises as the
    tether grows.
    """
    min_height = 50.0

    # ---- Constraints the optimizer solves UNDER -------------------------- #
    # `min_height` above is a gate: the reply is scored and thrown away if it
    # fails, after the solve has been paid for. What follows is sent WITH the
    # request, so the optimizer cannot offer a path that breaks it (AWETrim,
    # 2026-08-19).
    #
    # `min_feasibility_margin` is BOTH now: it still gates the reply, and
    # `min_turn_radius_request` converts it to metres and sends it, so the two
    # cannot disagree and "PATTERN TOO TIGHT" stops being something the solve
    # discovers only after it is paid for. The price is that a constrained cold
    # solve lands on the best branch less often than a free one (11/18 lengths
    # against 16/18, measured by AWETrim on the LEI-V3 reference), so a 422 where
    # there used to be a rejected reply is the expected new failure mode.
    """
    Elevation floor [deg] the optimized pattern must stay above, sent with the
    request; `0.0` keeps the optimizer's own 0.6°.

    Not the same thing as [`min_height`](@ref), and that is the point: AWETrim
    constrains HEIGHT, which a growing tether satisfies at ever lower angles (50 m
    at 350 m of tether is 8.2°), while this repo's criterion — and `el_floor` in
    `reelout_feasibility.jl` — is an ANGLE. Set it to `min_elevation +
    candidate_elevation_margin` to ask for what the gate will demand.
    """
    pattern_elevation_min = 0.0
    """
    Elevation ceiling [deg] of the optimized pattern; `0.0` keeps the optimizer's
    51.6°. Guards the run-away-to-zenith basin a failed re-optimization falls into.
    """
    pattern_elevation_max = 0.0
    """
    Half-width limit [deg] of the optimized pattern in azimuth; `0.0` keeps the
    optimizer's 45.8°.
    """
    pattern_azimuth_max = 0.0
    """
    Smallest azimuth half-width [deg] the optimized figure may have; `0.0` is off.
    Guards the OTHER bad basin: the pattern collapsing to zero amplitude.
    """
    pattern_azimuth_amplitude_min = 0.0

    # ---- Re-optimization while the tether grows (simple_opt_reelout.jl) --- #
    """
    Re-optimize during the run. The path is anchored to ONE radius, and a reel-out
    run walks 180 -> 350 m away from it; each re-optimization re-anchors it to the
    length actually being flown. `false` flies the path from `/init` all the way.
    """
    reopt_enabled::Bool = false
    """
    Re-optimize with `/step` ALONE, keeping the session `/init` built at the start
    of the run, so each solve WARM-STARTS from the previous optimum. `/step`
    re-anchors the pattern to the length it is given — the server moves `r0` and
    shifts its stored node-wise warm start by the same delta — so the seed is the
    last optimum at the radius now being flown. `false` repeats the STARTUP solve
    every time: a fresh `/init` from the parametric guess, i.e. a cold solve at
    every length.

    Not to be confused with feeding the FLOWN path back as the seed, which is what
    the docstring of `examples/simple_opt_reelout.jl` warns about and which failed
    repeatedly from ~210 m out. That refits an (azimuth, elevation) curve solved
    for a much shorter radius; this hands the optimizer its own previous iterate in
    its own variables and never leaves the server.

    What it costs. A warm request is not reproducible on its own — it depends on
    every solve before it — so the failure cache cannot key it and is skipped for
    warm steps (the cold fallback below is still cached). And the run then follows
    ONE branch of a multi-modal problem instead of re-drawing from the guess at
    every length: a good branch is kept, a mediocre one is kept too. What it buys
    is the cheap solve, which under `reopt_blocking` is wall time the simulation is
    frozen for.

    A warm step that comes back `"failed"` falls back to ONE cold `/init` from the
    parametric guess, under the same rule as [`reopt_retry_el_offset`](@ref):
    blocking mode only, since without the hold there is nothing to retry within.
    That `/init` also RESETS the session, so the warm starts that follow it
    continue from the new solve.
    """
    use_step::Bool = false
    """
    Laps between re-optimizations. Requests go out on a lap boundary because that
    is where a path swap is cheapest, and never while a solve is still running.
    """
    reopt_every_n_laps::Int64 = 2
    "Maximum re-optimizations per run; a bound on both wall time and surprise"
    max_reopt::Int64 = 4
    """
    Seconds over which a new path replaces the old one ([`blend_paths`](@ref)).
    Installing one in a single step is a step in the cross-track error, and the
    guidance answers a step with steering.
    """
    path_blend_time = 4.0
    """
    Seconds between `/status` polls while a solve is running. SIMULATED seconds
    when `reopt_blocking` is false, WALL-CLOCK seconds when it is true (nothing is
    simulated then). The solve takes 7-13 s of wall time (measured), so polling
    faster only adds HTTP round trips.
    """
    reopt_poll_interval = 0.5
    """
    Halt the simulation while a re-optimization runs, instead of flying on and
    collecting the reply when it lands.

    `false` keeps the run realtime-ish: the kite keeps flying through the 7-13 s
    solve, so the path arrives anchored to a radius the run has already left, by
    the reel-out speed times the solve time. `true` freezes the loop at the
    request, so the reply is anchored to the length it was asked for, at the cost
    of that wall time. Only the accounting differs, not the physics: the frozen
    time is reported as `traj_opt.reopt.blocked_s` and taken out of
    `performance.realtime_factor`.
    """
    reopt_blocking::Bool = true
    """
    Elevation offset [deg] added to `guess_el_center` for ONE retry when a
    re-optimization comes back `"failed"`; `0.0` never retries.

    The optimizer has isolated failure pockets in tether length — measured
    2026-08-18 at 6 m/s, 240 and 243 m fail from the shipped guess while 246 m
    solves to 8931 W — and it falls into two bad basins there: the pattern
    collapses to zero amplitude, or it runs away to the `C_beta` bound. A retry
    from a different seed usually lands elsewhere.

    This is NOT the startup solve, which never retries on purpose: there a guess
    that merely converges is a different optimum flown silently. A re-optimization
    is a candidate that still has to pass the curvature, clearance and elevation
    gates before it is installed, and a failure just means flying on with the
    current path — so a second attempt costs one solve and risks nothing.

    Only applies when `reopt_blocking` is true; without the hold there is nothing
    to retry within.
    """
    reopt_retry_el_offset = 2.0
    """
    Extra elevation [deg] a path must clear above `FC_Settings.min_elevation`
    before it is flown.

    The gate compares the REFERENCE path's lowest point; the criterion scores the
    FLOWN one, and the kite flies below its reference near the lobe tips where the
    steering is already clamped. Measured on 2026-08-18, twice: reference 10.0° ->
    flown 7.0°, and reference 10.0° -> flown 7.3°. Without an allowance the gate
    admits paths that break the criterion it is supposed to protect, which is what
    happened on the first stage-4 run. `0.0` gates on `min_elevation` alone.
    """
    candidate_elevation_margin = 3.0
end

"""
    TrajOptSettings(filename::String; path = skc_data_path())

Load the settings from the `traj_opt:` section of `filename`. An unknown key is
an error; a missing one keeps the struct default.
"""
function TrajOptSettings(filename::String; path = skc_data_path())
    tos = load_yaml_fields!(TrajOptSettings(), filename, "traj_opt"; path)
    tos.guess_points >= 8 ||
        error("guess_points must be >= 8, got $(tos.guess_points).")
    tos.resample_points >= 4 ||
        error("resample_points must be >= 4, got $(tos.resample_points).")
    tos.min_height >= 0 || error("min_height must be >= 0, got $(tos.min_height).")
    tos.reopt_every_n_laps >= 1 ||
        error("reopt_every_n_laps must be >= 1, got $(tos.reopt_every_n_laps).")
    for (name, value) in (("pattern_elevation_min", tos.pattern_elevation_min),
                          ("pattern_elevation_max", tos.pattern_elevation_max),
                          ("pattern_azimuth_max", tos.pattern_azimuth_max),
                          ("pattern_azimuth_amplitude_min",
                           tos.pattern_azimuth_amplitude_min))
        0 <= value <= 90 || error("$name must be in [0, 90], got $value.")
    end
    tos.pattern_elevation_max == 0 ||
        tos.pattern_elevation_max > tos.pattern_elevation_min ||
        error("pattern_elevation_max ($(tos.pattern_elevation_max)) must be greater "*
              "than pattern_elevation_min ($(tos.pattern_elevation_min)).")
    tos.turn_radius_headroom >= 1 ||
        error("turn_radius_headroom must be >= 1, got $(tos.turn_radius_headroom).")
    tos.candidate_elevation_margin >= 0 ||
        error("candidate_elevation_margin must be >= 0, got "*
              "$(tos.candidate_elevation_margin).")
    tos.path_blend_time > 0 ||
        error("path_blend_time must be > 0, got $(tos.path_blend_time).")
    tos.guess_a > 0 && tos.guess_b > 0 ||
        error("guess_a and guess_b must be > 0, got $(tos.guess_a) and $(tos.guess_b).")
    return tos
end
