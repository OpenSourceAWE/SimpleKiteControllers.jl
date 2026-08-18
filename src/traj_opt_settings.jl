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
    Depower setpoint `u_p` the optimizer starts from. NOT the run's
    `depower_setpoint`: the conversion between the two is unresolved (see
    `docs/TrajectoryOptimization.md`), and the optimizer varies this one anyway —
    it is in the server's default `optimization_params`.
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
    Smallest curvature margin ([`check_pattern_feasible`](@ref), path radius over
    the kite's minimum turn radius) the returned path must have at the tether
    length it is flown at. Below 1.0 the path is tighter than the kite can turn,
    and the run measures the steering clamp instead of the path. `0.0` disables
    the check and flies whatever came back.
    """
    min_feasibility_margin = 1.0
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

    # ---- Re-optimization while the tether grows (simple_opt_reelout.jl) --- #
    """
    Re-optimize during the run. The path is anchored to ONE radius, and a reel-out
    run walks 180 -> 350 m away from it; each re-optimization re-anchors it to the
    length actually being flown. `false` flies the path from `/init` all the way.
    """
    reopt_enabled::Bool = false
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
    Seconds of simulated time between `/status` polls while a solve is running.
    The solve takes 7-13 s of wall time (measured), so polling faster only adds
    HTTP round trips to the simulation loop.
    """
    reopt_poll_interval = 0.5
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
    tos.candidate_elevation_margin >= 0 ||
        error("candidate_elevation_margin must be >= 0, got "*
              "$(tos.candidate_elevation_margin).")
    tos.path_blend_time > 0 ||
        error("path_blend_time must be > 0, got $(tos.path_blend_time).")
    tos.guess_a > 0 && tos.guess_b > 0 ||
        error("guess_a and guess_b must be > 0, got $(tos.guess_a) and $(tos.guess_b).")
    return tos
end
