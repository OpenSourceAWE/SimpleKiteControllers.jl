# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The conditions of a run, as the AWETrim optimizer wants them: the wind (`InflowConditions`)
# and winch (`WinchParams`) of a request, read off the run's settings. Pure: settings in,
# structs out; their JSON form (`StructTypes`) and the HTTP calls are in `awetrim_client.jl`.

Base.@kwdef struct InflowConditions
    wind_speed::Float64      # in m/s at 6 m height
    wind_direction::Float64  # in degrees, 0 = North, 90 = East
    profile_law::Int64       # 0=CONST, 1=EXP, 2=LOG, 3=EXPLOG, 4=CUSTOM_LOG, 5=CUSTOM_EXP, 6=CUSTOM_JET
    # the custom profiles are fitted using the heights and speeds given in the heights and speeds fields
    # CUSTOM_JET: u(z) = u_bg(z) + U_J * exp(-(z - z_c)^2 / (2*sigma^2))
    # the following fields are optional; the defaults given below are the server defaults
    alpha::Float64 = 0.08163                # exponent of the wind profile law
    z0::Float64 = 0.0002                    # surface roughness                                     [m]
    turbulence::Float64 = 0.0               # in [0, 1], 0 = no turbulence, 1 = full turbulence
    heights::Vector{Float64} = [6.0]        # heights at which the wind speed is given
    speeds::Vector{Float64} = [wind_speed]  # wind speeds at the given heights
end

Base.@kwdef struct WinchParams
    mode::String      # "reelout" ("reelin" not supported yet)
    k_v::Float64      # v_set = k_v * sqrt(force)
    f_min::Float64    # minimum winch force [N]
    f_max::Float64    # maximum winch force [N]
    # Past `f_max` the controller holds the force while the reel-out speed keeps
    # rising, up to the winch's power limit — so the speed cap is a genuine input
    # and not derivable from `k_v`/`f_max`. Give either; both `nothing` leaves the
    # optimizer's own reel-speed bound (10 m/s) in force, which with this repo's
    # kv = 0.0408 and f_high = 8000 N is never reached (3.65 m/s).
    v_max::Union{Float64, Nothing} = nothing   # maximum reel-out speed [m/s]
    p_max::Union{Float64, Nothing} = nothing   # maximum winch power [W]; v_max = p_max/f_max
    # Make `k_v` a DESIGN VARIABLE instead of a constant: the server brackets it a
    # factor `K_V_BRACKET_FACTOR` (2.0) either side of the value sent and solves for
    # it under its own saturating tension curve, so the path and the gain that flies
    # it are optimized together. The reply's `k_v` is then the OPTIMIZED one and the
    # path is not flyable with the value sent in — see `apply_optimized_kv!`.
    optimize_k_v::Bool = false
    # Corner sharpness of the two soft saturations the server applies to the
    # tension curve [1/N]; larger is sharper, the transition spanning a band of
    # order 1/beta. Sent because `WCSettings.force_limit = "soft"` INVERTS that
    # curve to command a reel-out speed: the controller and the optimizer must use
    # one law, and before this they agreed only because both defaulted to 1e-3.
    # `nothing` leaves the server on its own default.
    softplus_beta::Union{Float64, Nothing} = nothing
    softminus_beta::Union{Float64, Nothing} = nothing
    # Blend factor in [0, 1] towards a reel-in-capable server winch law
    # (Winch.tension_curve's use_awe_trim, AWETrim/src/awetrim/system/winch.py):
    # 0 (default) is the server's plain quadratic law above, unchanged; 1
    # replaces it below f_min with a straight line through (0, f_min) and
    # (v_reel_in, 0), handed to the quadratic law by a smooth maximum. Mirrors
    # calc_vro_soft's own use_awe_trim (WinchControllers.jl), which blends the
    # OTHER direction (force -> speed; the server computes speed -> force).
    use_awe_trim::Float64 = 0.0
    # use_awe_trim only: reel-in speed [m/s] at zero force (< 0) and handover
    # sharpness [s/m]. `nothing` keeps the server's own defaults (-2.0, 20.0).
    # Unlike softminus_beta above, reel_in_beta has no matching sharpness
    # requirement against f_min/k_v to worry about on the server side -- see
    # WinchParams.reel_in_beta's docstring in AWETrim's schemas.py.
    v_reel_in::Union{Float64, Nothing} = nothing
    reel_in_beta::Union{Float64, Nothing} = nothing
    # "force_law" (the server default) ties the tension to the reel speed through
    # the curve above, as a per-node equality; "free_speed" drops that equality and
    # bounds the tension to [f_min, f_max] instead, the reel speed becoming a direct
    # acceleration-limited control. `nothing` leaves the server on its own default.
    # The reply is then the best path for ANY winch in that force band, so its
    # predicted power is an UPPER BOUND, not a prediction of this k_v law -- see
    # TrajOptSettings.opt_winch_mode.
    winch_mode::Union{String, Nothing} = nothing
    # Sharpness [s/m] of the soft reel-speed clamp at `v_max`, the server-side
    # mirror of `calc_vro_soft`'s `_clamp_v_sat` (`WCSettings.v_sat_beta`): the
    # tension then rises to `f_max` AT `v_max`, as the controller's curve does,
    # instead of ending at the `v_max` speed bound. Needs `v_max` or `p_max`;
    # `nothing` keeps the plain law, i.e. a hard clamp.
    v_sat_beta::Union{Float64, Nothing} = nothing
end

"The four-field winch of the original contract; `v_max`/`p_max` stay unset."
WinchParams(mode::AbstractString, k_v, f_min, f_max) =
    WinchParams(; mode = String(mode), k_v = Float64(k_v),
                f_min = Float64(f_min), f_max = Float64(f_max))

# Deliberately duck-typed rather than annotated with `KiteUtils.Settings` and
# `WinchControllers.WCSettings`: any settings object with these fields will do.

"""
    inflow_from_settings(set) -> InflowConditions

The wind of a run, from the `KiteUtils.Settings` of its system project — the
same file the plant is built from, so the path cannot be optimized for a wind
the kite does not fly in. Field for field: `v_wind`, `upwind_dir` (both are "the
direction the wind comes FROM", 0 = North, clockwise), `profile_law`, `alpha`,
`z0`, and `heights`/`speeds` for the fitted `CUSTOM_*` laws 4-6.

`turbulence` is left at 0 on purpose: the server accepts and echoes it but does
NOT use it — the optimizer is deterministic and works on the mean profile.
Passing a run's `use_turbulence` would imply otherwise.
"""
function inflow_from_settings(set)
    isapprox(set.h_ref, 6.0; atol = 1e-6) ||
        @warn "The optimizer reads wind_speed at 6 m, but this project's h_ref is \
               $(set.h_ref) m — the reference speed is not comparable."
    samples = set.profile_law >= 4 ?
              (; heights = Float64.(set.heights), speeds = Float64.(set.speeds)) : (;)
    return InflowConditions(; wind_speed = set.v_wind,
                            wind_direction = mod(set.upwind_dir, 360.0),
                            profile_law = set.profile_law,
                            alpha = set.alpha, z0 = set.z0,
                            turbulence = 0.0, samples...)
end

"""
The `softminus_beta` always sent to AWETrim, independent of the local winch's own
`wc.softminus_beta` — measured 2026-08-25: `2e-3`/`5e-3` both make the 3 m/s
solve FAIL (422, `Max_Iterations_Exceeded`), `1e-3` converges. Kept fixed here so a
local sharpening of the plain `force_limit = "soft"` law WITHOUT `soft_lfc`
(which needs `softminus_beta * f_low >= 8`, e.g. `0.03` at `f_low = 350`)
never reaches the server. With `soft_lfc = true` this value is not even read
locally any more — `calc_vro_soft` hard-clamps at `f_low` instead, see
`use_awe_trim` on `winch_from_wc` for what governs that curve's server-side
counterpart. See `data/wc_settings.yaml`.
"""
const AWETRIM_SOFTMINUS_BETA = 1e-3

"""
Whether [`winch_from_wc`](@ref) sends `v_sat_beta`. Needs an AWETrim whose
`Winch.radial_equation` blends in the speed form `v = V(F)` near `v_max`
(2026-09-23): with the plain equality `F = T(v)` the clamped curve is nearly
vertical there, and the 7 m/s Cabauw startup solve at 150 m failed in IPOPT.
With the blend it converges at 3, 4, 5, 7 and 10 m/s.
"""
const SEND_V_SAT_BETA = true

"""
    winch_from_wc(wc; v_max = wc.v_sat, p_max = nothing) -> WinchParams

The winch law of a run, from the `WinchControllers.WCSettings` its
`WinchController` is built from: `kv`, `f_low` and `f_high` of
`data/wc_settings.yaml`, plus the two `beta`s that set how softly the server
saturates its tension curve at those limits, and `use_awe_trim`/`v_reel_in`/
`reel_in_beta` for the reel-in-capable blend. The optimizer maps
`v_set = kv*sqrt(force)` onto its radial force model, so these numbers are what
makes the optimized path the path for THIS ground station.

The `beta`s matter more than they look. The server's effective force FLOOR is
`sp(beta*f_min)/beta`, not `f_min`: at the historical 1e-3 a 350 N `f_min` acts as
884 N, which is above the entire force range a 3 m/s run ever reaches. They also
have to match `calc_vro_soft`'s, which inverts this exact curve under
`force_limit = "soft"` — the two sides used to agree only by coincidence.

A winch too stiff to reach the optimal reel-out speed within `f_high` makes the
problem infeasible and the server answers 422 — `kv*sqrt(f_high)` is the speed
ceiling to compare against roughly a third of the wind speed.

`v_max`/`p_max` are the ground station's limit PAST the force bound, where the
controller holds `f_high` and the speed keeps rising. `v_max` defaults to `wc.v_sat`
— `data/wc_settings.yaml`'s own reel-out speed limit (8 m/s here), the same cap the
runtime `WinchController` enforces — so the optimizer is asked for a path THIS
ground station can actually fly. Pass `v_max = nothing` to fall back to the
optimizer's own 10 m/s bound instead, which with `kv = 0.0408` and
`f_high = 8000 N` the square-root law never reaches anyway (3.65 m/s), so this
only starts to matter on a softer winch.

`f_max` defaults to `wc.f_high_awe_trim` when that is set (a fixed de-rating,
independent of wind speed) and to `wc.f_high` otherwise, and is overridable so
a request can be solved against the ceiling that will actually be in force
while its path is flown — `fcs.first_lap_force_frac` holds the runtime limit
down for the first figure of eight, and the startup path is the one flown
there. Lowering it also lowers the speed ceiling `kv*sqrt(f_max)` referred to
above.

`softminus_beta` sent to the server is pinned to `AWETRIM_SOFTMINUS_BETA`, NOT
read off `wc`: `wc.softminus_beta` may be sharpened locally for the plain
`force_limit = "soft"` law WITHOUT `soft_lfc` (which needs `softminus_beta *
f_low >= 8`, far sharper than the server tolerates) — with `soft_lfc = true`,
`calc_vro_soft` hard-clamps at `f_low` locally instead and never reads
`softminus_beta` at all, but the pin stays unconditional so a run that flips
`soft_lfc` back off is still covered. See `AWETRIM_SOFTMINUS_BETA`'s
docstring.

`use_awe_trim` defaults to `wc`'s and is overridable so a seeding solve can be
sent at a value known to converge before the real request — see
`TrajOptSettings.opt_warm_start_awe_trim`. `v_reel_in`/`reel_in_beta` are read
straight off `wc` (unlike the `beta`s above, they need no pinning: the
server-side sharpness requirement on `reel_in_beta` is far more forgiving than
the local one, since `Winch.tension_curve` blends in the forward direction),
so a run that reels in locally optimizes against a server winch model that
can do the same. `wc.use_awe_trim` defaults to `0.0`, leaving the server's
plain law unchanged unless `data/wc_settings.yaml` opts in.

`v_sat_beta` is sent only while [`SEND_V_SAT_BETA`](@ref) is on, and then only when
the local law really soft-clamps at the same speed the
server is bounded by: `force_limit = "soft"` (only `calc_vro_soft` applies the
clamp), a finite `wc.v_sat_beta` (`Inf` is a hard clamp, the server's plain law)
and `v_max == wc.v_sat`. The server's tension curve then rises to `f_max` at
`v_max` like the controller's, instead of ending at about 6.9 kN there.
"""
winch_from_wc(wc; v_max = wc.v_sat, p_max = nothing, optimize_k_v = false,
              f_max = wc.f_high_awe_trim > 0 ? wc.f_high_awe_trim : wc.f_high,
              use_awe_trim = wc.use_awe_trim, winch_mode = nothing) =
    WinchParams(; mode = "reelout", k_v = wc.kv, f_min = wc.f_low,
                v_sat_beta = SEND_V_SAT_BETA && wc.force_limit == "soft" && isfinite(wc.v_sat_beta) &&
                             v_max !== nothing && v_max == wc.v_sat ?
                             Float64(wc.v_sat_beta) : nothing,
                f_max = Float64(f_max),
                v_max = v_max === nothing ? nothing : Float64(v_max),
                p_max = p_max === nothing ? nothing : Float64(p_max),
                optimize_k_v = Bool(optimize_k_v),
                use_awe_trim = Float64(use_awe_trim),
                v_reel_in = Float64(wc.v_reel_in),
                reel_in_beta = Float64(wc.reel_in_beta),
                # Sent unconditionally, not only under `force_limit = "soft"`: the
                # server's floor is `sp(beta*f_min)/beta`, so beta shapes the path
                # it plans whatever the runtime winch then does with it.
                softplus_beta = Float64(wc.softplus_beta),
                softminus_beta = Float64(AWETRIM_SOFTMINUS_BETA),
                winch_mode = winch_mode)

"""
    cap_wind_speed(tos, project_set, v_wind_gnd) -> Float64

The wind speed the elevation-cap step is keyed on: `v_wind_gnd` (the mean wind
at the project's `h_ref`, as passed to `init`) scaled to
`tos.pattern_elevation_amplitude_max_wind_height` by the project's own profile
law, or unscaled when that height is `0.0`.
"""
function cap_wind_speed(tos, project_set, v_wind_gnd)
    h = tos.pattern_elevation_amplitude_max_wind_height
    h > 0 || return Float64(v_wind_gnd)
    return calc_wind_factor(AtmosphericModel(project_set; nowindfield = true), h) *
           v_wind_gnd
end

"""
    optimizer_conditions(tos, fcs, project_set, rcs, f_high_nominal)
        -> (; inflow, cap_wind, opt_awe_trim, opt_winch_mode, winch, winch_first_lap, winch_reopt)

What the optimizer is SENT about THIS run, decoupled from the local winch law (the solve
must converge at a force the kite can pull): the `inflow`, the wind the elevation-cap step
reads (`cap_wind`), the AWETrim setting and mode, and the winch of the three kinds of solve:
`winch` for the STARTUP solve, the plain runtime ceiling `f_high_nominal`, never
`rcs.f_high_awe_trim`; `winch_first_lap` under `first_lap_force_frac`, so the startup path is
solved against the ceiling lap 1 flies under; and `winch_reopt` for the re-optimizations (lap 2
on), the one caller that may fly `rcs.f_high_awe_trim`.
"""
function optimizer_conditions(tos, fcs, project_set, rcs, f_high_nominal)
    inflow = inflow_from_settings(project_set)
    cap_wind = cap_wind_speed(tos, project_set, inflow.wind_speed)
    opt_awe_trim = tos.opt_awe_trim >= 0 ? tos.opt_awe_trim : rcs.use_awe_trim
    opt_winch_mode = isempty(tos.opt_winch_mode) ? nothing : tos.opt_winch_mode
    winch = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                          winch_mode = opt_winch_mode, f_max = f_high_nominal)
    winch_first_lap = fcs.first_lap_force_frac < 1 ?
        winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                      winch_mode = opt_winch_mode,
                      f_max = f_high_nominal * fcs.first_lap_force_frac) : winch
    winch_reopt = winch_from_wc(rcs; optimize_k_v = tos.optimize_k_v, use_awe_trim = opt_awe_trim,
                                winch_mode = opt_winch_mode)
    @info @sprintf("Optimizer conditions: %.1f m/s at 6 m from %.0f°, profile_law %d, \
                    z0 = %g m | winch kv = %.4f, i.e. %.1f m/s at f_high = %.0f N | \
                    depower seed %.3f m%s.",
                   inflow.wind_speed, inflow.wind_direction, inflow.profile_law, inflow.z0,
                   winch.k_v, winch.k_v * sqrt(winch.f_max), winch.f_max,
                   depower_seed(tos, inflow.wind_speed),
                   inflow.wind_speed > tos.input_depower_wind_ref ?
                       @sprintf(" (%.2f + %.3f per m/s above %.1f m/s)", tos.input_depower,
                                tos.input_depower_per_wind, tos.input_depower_wind_ref) : "")
    return (; inflow, cap_wind, opt_awe_trim, opt_winch_mode, winch, winch_first_lap, winch_reopt)
end
