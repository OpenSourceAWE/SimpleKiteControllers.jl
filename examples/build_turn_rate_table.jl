# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Fill `data/turn_rate_coeffs.yaml` with the depower cells this package needs, so
[`turn_rate_coeffs`](@ref) can interpolate instead of throwing. The turn-rate law is
identified LOW in the wind window, where the kite flies its patterns: per depower,
one relay flight per entry of `FLIGHT_SETTINGS`, each at a fixed steering amplitude
for `SWEEP_SIM_TIME` (`_fly_relay`), and one joint fit of the flights that stayed
airborne (`_fly_low_flights`, V3Kite's `joint_delay_lag_fit`): `c1`, `c2`, the
dead time and the kite's lag.

Every flight starts at `START_ELEVATION`, relays about a crosswind heading
(`±HEADING_CENTER`), reverses at `±az_reverse` of azimuth and tilts the band to
hold `EL_HOLD`: a lazy-eight-like pattern at `v_a` ≈ 20 – 50 m/s (depower 0.275,
2026-09-29). The relay sweeps at 73° this replaced hovered near the zenith at
`v_a` ≈ 11 – 16 m/s; their table and its fixed-`c3` variant were deleted on
2026-10-01.

It flies THIS package's project: the identification must see the same plant the
runs do — same wing mass, tether diameter and KCU rate limits — or it identifies a
kite nobody flies.

Writes incrementally: after every depower the whole file is re-read, that cell's
row inserted or replaced, and the file rewritten, so a diverged run costs one cell
and not the grid. `remake = false` (the default) skips a cell whose row already
passed at the table's current `conditions`, and `_write_turn_rate_entry!`
separately refuses to let a failed re-run demote a row that passed. Once this
script has written to the YAML its formatting is `YAML.write_file`'s, not the
hand-authored layout.

`include` only loads the definitions — this script is called repeatedly with
different arguments rather than once with fixed ones. Call it yourself:

    build_turn_rate_table()                       # the whole grid
    build_turn_rate_table(depowers = [0.30])      # one cell
    build_turn_rate_table(remake = true)          # re-identify every cell

Expect about 5 – 10 minutes per depower and a settling-cache miss on every new
depower. Expect some cells to fail too: at depower 0.40 no flight stayed airborne
(2026-09-29). Afterwards the table is reloaded into the running session, so
`test/test_fig8_controller.jl` can be re-run without restarting.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using V3Kite
using V3Kite: init, step!
using SimpleKiteControllers
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
using WinchControllers: WCSettings, WinchPosController
import KiteUtils   # for KiteUtils.syslog; V3Kite does not re-export it
using YAML
using Printf
using LinearAlgebra: norm
using Statistics: mean, std
using Base.CoreLogging: with_logger, NullLogger
import Dates

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))

# ============== FIXED CONDITIONS (data/turn_rate_coeffs.yaml) ============== #
# Only the depower varies across the grid. These must agree with the file's
# `conditions:` block, which `_check_conditions` enforces.
#
# `SWEEP_`-prefixed where every run script assigns the bare name as a plain
# global (`PROJECT`, `SIM_TIME`, `AERO_MODE`): this file is meant to be included
# into the SAME session those scripts run in (its docstring: the table is reloaded
# there afterwards), and a `const` of the same name makes their next `include`
# die on "invalid assignment to constant" (2026-09-18, simple_opt_reelout.jl).

const SWEEP_PROJECT    = project_file("system_reelout_maasvlakte.yaml")
const V_WIND           = 9.51
const TETHER_LENGTH    = 150.0
const DT               = 0.05 / 3
const SWEEP_SIM_TIME   = 200.0
const SWEEP_AERO_MODE  = ContinuousAero()
const VSM_INTERVAL     = 5

# Settling starts at the first and decays to the second, which is what the flights
# are FLOWN at — the same pair every run here uses (`0.8 .* fcs.run.body_damping`).
# The row is keyed by the START value, as `fcs.run.body_damping` is.
const BODY_START_DAMPING = [0.0, 0.0, 40.0]
const BODY_SIM_DAMPING   = 0.8 .* BODY_START_DAMPING

# Relay controller, as in V3Kite's steering_test_v3.jl, about a crosswind heading.
const T_START          = 10.0
const HEADING_OFFSET   = 10.0
const START_ELEVATION  = 30.0   # [°] elevation the flights start at; the table's `conditions`
const HEADING_CENTER   = 90.0   # [°] centre of the relay's heading band, crosswind; 0 climbs back to ~70°
const EL_HOLD          = 30.0   # [°] elevation the band's tilt holds the kite near
const ELEVATION_FLOOR  = 10.0   # [°] a flight stops below this
const MAX_ELEVATION    = 55.0   # [°] a flight's fit window starts at its first sample below this
# [m] a flight flown with `v_reelout` stops reeling out here, the reel-out runs' length.
const REELOUT_L_MAX    = 380.0

# One flight per fixed steering amplitude `a` [-], each with its own azimuth of reversal
# `az_reverse` [°] and tilt limit of the elevation hold `el_hold_tilt` [°], see `_fly_relay`.
# The range that flies steadily at depower 0.275 (2026-09-29): 0.05 turns too weakly and
# drifts to the edge of the wind window even reversing at ±10°; 0.15 turns ~90 °/s against
# a tape that needs ~1 s to swing, so the relay overshoots its band past heading 180° and
# loops into the ground.
const FLIGHT_SETTINGS = [(a = 0.075, az_reverse = 20.0, el_hold_tilt = 45.0),
                         (a = 0.100, az_reverse = 30.0, el_hold_tilt = 45.0),
                         (a = 0.125, az_reverse = 30.0, el_hold_tilt = 25.0)]
# `G` mask of `identify_turn_rate_law`, below the smallest amplitude.
const MIN_STEERING_FIT = 0.025

# Dead time + lag split of one flight (`_split_delay`): the kite's lag is searched on a
# grid of DT steps up to KITE_LAG_MAX [s]; the dead time over DELAY_BLOCK_TMAX [s].
const KITE_LAG_MAX     = 1.0
const DELAY_BLOCK_TMAX = 3.0
# [s] length of the blocks the standard errors of a row are taken over (`block_standard_errors`).
const BLOCK_LENGTH     = 20.0

const OUT_FILE = "turn_rate_coeffs.yaml"

"""
    _check_conditions(dict)

Throw unless the `conditions:` block of the loaded table agrees with this
script's constants. A row written under conditions the block does not describe
is unusable data that looks like data.
"""
function _check_conditions(dict)
    want = Dict{String, Any}("system" => basename(SWEEP_PROJECT), "v_wind" => V_WIND,
                "l_tether" => TETHER_LENGTH, "elevation" => START_ELEVATION, "dt" => DT)
    for (k, v) in want
        have = get(dict["conditions"], k, missing)
        isapprox_ok = have isa Real && v isa Real ? isapprox(have, v; rtol = 1e-4) : have == v
        isapprox_ok || error("build_turn_rate_table: conditions[$k] is $have, this script " *
                             "flies at $v. Update the conditions block of data/$OUT_FILE " *
                             "(or this script) before writing rows against it.")
    end
end

"""
    _fly_relay(depower, a; az_reverse, el_hold_tilt, v_wind=V_WIND, v_reelout=0.0,
               elevation_floor=ELEVATION_FLOOR, elevation=START_ELEVATION,
               heading_center=HEADING_CENTER, el_hold=EL_HOLD, el_hold_gain=3.0) -> NamedTuple

One relay flight at `depower` and the fixed steering amplitude `a`: settle at
`elevation` [°] and `TETHER_LENGTH`, hold the length (or reel out at `v_reelout`
[m/s] from `T_START` on, up to `REELOUT_L_MAX`), and from `T_START` on flip the
steering between `-a` and `+a` whenever the heading leaves the band
`±HEADING_OFFSET` about its centre, for `SWEEP_SIM_TIME`.

The band's centre is `side · heading_center` [°]: `side` flips when the azimuth
passes `±az_reverse` [°], so the kite reverses its direction of flight there. Its
magnitude is tilted by `el_hold_gain` [°/°] per degree above (down) or below (up)
`el_hold` [°], clamped to `heading_center ± el_hold_tilt`, which holds the kite
near that elevation. A fast turn (a large amplitude) overshoots the band, so it
needs a smaller `el_hold_tilt`. A flight stops below `elevation_floor` [°].

Returns `(; outcome, min_elevation, sl, band_time, band_center)`. `outcome` is
`:time_limit` (flew the full time), `:low_elevation` or `:error` (the simulation
diverged). `sl` is the log of the samples flown. `band_center` [°] is the signed
centre of the band at each step from the one the relay starts on, after the
reversal and the elevation hold; `band_time` [s] is the time of that step.
"""
function _fly_relay(depower, a; az_reverse::Real, el_hold_tilt::Real, v_wind::Real = V_WIND,
                    v_reelout::Real = 0.0, elevation_floor::Real = ELEVATION_FLOOR,
                    elevation::Real = START_ELEVATION, heading_center::Real = HEADING_CENTER,
                    el_hold::Real = EL_HOLD, el_hold_gain::Real = 3.0)
    @info @sprintf("build_turn_rate_table: depower = %.3f, amplitude = %.3f, wind %.2f m/s",
                   depower, a, v_wind)
    s = init(v_wind, TETHER_LENGTH; body_start_damping = BODY_START_DAMPING,
        body_sim_damping = BODY_SIM_DAMPING, elevation,
        depower_setpoint = depower, sim_time = SWEEP_SIM_TIME, dt = DT,
        system_yaml = SWEEP_PROJECT, aero_mode = SWEEP_AERO_MODE, remake_model = false)

    l_set = s.sys_state.l_tether[1]
    wpc = WinchPosController(WCSettings(true; dt = s.dt); dt = s.dt)

    side = 1.0              # sign of the band centre, flipped at ±az_reverse
    rel_steering = 0.0
    min_elevation = Inf
    outcome = :time_limit
    band_time = Float64[]
    band_center = Float64[]

    try
        for _ in 1:s.steps
            t = s.sys_state.time + s.dt
            if T_START <= t < T_START + s.dt
                rel_steering = -a
            end
            if t > T_START + s.dt
                az = rad2deg(s.sys_state.azimuth)
                side > 0 && az > az_reverse && (side = -1.0)
                side < 0 && az < -az_reverse && (side = 1.0)
                center = clamp(heading_center + el_hold_gain * (rad2deg(s.sys_state.elevation) - el_hold),
                               heading_center - el_hold_tilt, heading_center + el_hold_tilt)
                # Relative to the band's centre, so the relay logic is the same for any centre.
                # NOT wrapped: heading and centre both lie in (-180°, 180°], so the plain
                # difference never turns the kite through ±180° (straight down), and passes
                # through 0 (straight up) whenever a reversal flips the centre's sign.
                heading = wrap_to_pi(s.sys_state.heading) - deg2rad(side * center)
                push!(band_time, t)
                push!(band_center, side * center)
                if rad2deg(heading) < -HEADING_OFFSET
                    rel_steering = a
                elseif rad2deg(heading) > HEADING_OFFSET
                    rel_steering = -a
                end
            end

            v_set = t < T_START || l_set >= REELOUT_L_MAX ? 0.0 :
                    v_reelout * clamp((t - T_START) / 2.0, 0.0, 1.0)
            l_set = min(l_set + v_set * s.dt, REELOUT_L_MAX)
            step!(s; rel_depower = depower, rel_steering,
                  set_torque = winch_torque!(wpc, s, l_set; v_ff = v_set),
                  vsm_interval = VSM_INTERVAL)

            el = rad2deg(s.sys_state.elevation)
            min_elevation = min(min_elevation, el)
            if el < elevation_floor
                @warn @sprintf("  elevation %.2f° below floor %.1f° at t = %.2f s, stopping",
                               el, elevation_floor, t)
                outcome = :low_elevation
                break
            end
        end
    catch e
        outcome = :error
        @warn "build_turn_rate_table: run diverged" depower a exception = (e, catch_backtrace())
    end

    # The logger is preallocated for SWEEP_SIM_TIME; a flight that ends early leaves the rest
    # of the rows at zero (time 0 included), which folds the time axis back onto itself.
    sl = KiteUtils.syslog(s.logger)
    sl = sl[1:findlast(>(0), sl.time)]
    return (; outcome, min_elevation, sl, band_time, band_center)
end

"""
    _fit_window(sl; max_elevation=MAX_ELEVATION, label="") -> NamedTuple or nothing

`identify_turn_rate_law` on the fit window of the flight log `sl`: from its first
sample below `max_elevation` [°] after `T_START` to its end, contiguous, as the
backward-difference turn rate and the delay search need. Warns when part of the
window lies above `max_elevation`; `nothing` when the flight never came below it.
Returns `(; t_fit, fit)`, the start of the window [s] and the identification.
"""
function _fit_window(sl; max_elevation::Real = MAX_ELEVATION, label = "")
    el = rad2deg.(sl.elevation)
    k_below = findfirst(i -> sl.time[i] >= T_START && el[i] < max_elevation, eachindex(sl.time))
    if isnothing(k_below)
        @warn @sprintf("%s: never below max_elevation = %.1f° after T_START; skipped.", label, max_elevation)
        return nothing
    end
    t_fit = sl.time[k_below]
    in_window = sl.time .>= t_fit
    frac_above = count(in_window .& (el .> max_elevation)) / count(in_window)
    frac_above > 0 &&
        @warn @sprintf("%s: %.0f %% of the fit window above max_elevation = %.1f°.",
                       label, 100 * frac_above, max_elevation)
    fit = identify_turn_rate_law(sl; dt = DT, t_start = t_fit, min_steering = MIN_STEERING_FIT)
    return (; t_fit, fit)
end

"""
    _split_delay(fit) -> NamedTuple

[`fit_delay_lag`](@ref) at this script's sample time and search ranges.
"""
_split_delay(fit) = fit_delay_lag(fit, DT; lag_max = KITE_LAG_MAX, t_max = DELAY_BLOCK_TMAX)

"""
    _fly_low_flights(depower; v_wind=V_WIND, v_reelout=0.0) -> NamedTuple

One relay flight per entry of `FLIGHT_SETTINGS` at `depower` (`_fly_relay`), each
fitted on its own window (`_fit_window`, then `_split_delay`), and the joint fit of
the steady ones (`joint_delay_lag_fit`; of all of them if none flew the full time):
one dead time, lag, `c1` and `c2` for every flight, the steering of each flight
filtered and shifted separately so no shift crosses from one flight into the next.

Returns `(; flights, joint_flights, joint)`. Each flight is `(; a, outcome, sl, el,
vk, t_fit, fit, dl, v_ratio)`: the amplitude, the outcome, the log, the elevation
[°] and kite speed [m/s] of every sample, the start of the fit window [s], its
identification and delay-lag fit, and `v_τ/v_a` in the window. `joint` is
`nothing` when no flight came below `MAX_ELEVATION`.
"""
function _fly_low_flights(depower; v_wind::Real = V_WIND, v_reelout::Real = 0.0)
    flights = NamedTuple[]
    for (; a, az_reverse, el_hold_tilt) in FLIGHT_SETTINGS
        r = _fly_relay(depower, a; az_reverse, el_hold_tilt, v_wind, v_reelout)
        label = @sprintf("Amplitude %.3f", a)
        w = _fit_window(r.sl; label)
        isnothing(w) && continue
        sl = r.sl
        in_window = sl.time .>= w.t_fit
        vk = norm.(sl.vel_kite)
        v_tau = sqrt.(max.(vk .^ 2 .- Float64.(first.(sl.v_reelout)) .^ 2, 0.0))
        fit = merge(w.fit, (; c3 = nothing))
        push!(flights, (; a, r.outcome, sl, el = rad2deg.(sl.elevation), vk, w.t_fit, fit,
                        dl = _split_delay(fit), v_ratio = v_tau[in_window] ./ Float64.(sl.v_app[in_window])))
        @info @sprintf("%s: %s after %.0f s, fit window from %.1f s.", label, r.outcome,
                       last(sl.time), w.t_fit)
    end
    isempty(flights) && return (; flights, joint_flights = flights, joint = nothing)
    # The steady flights, if any: a flight that sank to the floor is a transient with a short window.
    joint_flights = let steady = filter(f -> f.outcome == :time_limit, flights)
        isempty(steady) ? flights : steady
    end
    joint = joint_delay_lag_fit([f.fit for f in joint_flights], DT)
    return (; flights, joint_flights, joint)
end

"""
    block_standard_errors(fits; block_t=BLOCK_LENGTH) -> NamedTuple

Standard errors of the dead time, lag, `c1` and `c2`: each fit window of `fits`
(from `identify_turn_rate_law`) is cut into blocks of `block_t` seconds, the
four are fitted on each block alone with `joint_delay_lag_fit`, and each standard
error is the scatter over the blocks divided by √(number of blocks). Not the
linear fit's own standard errors, which assume independent residuals: the
residuals of a flown path are strongly autocorrelated, so those come out far too
small. Returns `(; n, c1, c2, dead_time, lag)`, `NaN` for fewer than two blocks.
"""
function block_standard_errors(fits; block_t = BLOCK_LENGTH)
    nb = round(Int, block_t / DT)
    blocks = NamedTuple[]
    for f in fits, k in 1:nb:length(f.rate) - nb + 1
        r = k:k + nb - 1
        sub = (; us = f.us[r], rate = f.rate[r], v_app = f.v_app[r], psi = f.psi[r], beta = f.beta[r])
        # Muted: a short block's lag may hit its search limit, which is scatter, not news.
        push!(blocks, with_logger(() -> joint_delay_lag_fit([sub], DT), NullLogger()))
    end
    se(field) = length(blocks) > 1 ?
        std([getfield(b, field) for b in blocks]) / sqrt(length(blocks)) : NaN
    return (; n = length(blocks), c1 = se(:c1), c2 = se(:c2), dead_time = se(:dead_time), lag = se(:lag))
end

"""
    _entry_key(e) -> (Vector{Float64}, Float64)

`(body_damping, depower)` of a YAML entry dict, for matching against existing rows.
"""
_entry_key(e) = (Float64.(e["body_damping"]), Float64(e["depower"]))

"`true` if the YAML entry `e` passed: its steady flights flew the full time."
_entry_passed(e) = get(e, "outcome", "") == "time_limit"

"""
    _entry_is_legacy(e, conditions) -> Bool

`true` if entry `e` overrides any key of the table's `conditions` block. Such a
row is never treated as "already passing": the point of re-running its cell is to
replace it with one at the current conditions.
"""
function _entry_is_legacy(e, conditions)
    any(haskey(e, String(k)) && e[String(k)] != v for (k, v) in conditions)
end

"""
    _write_turn_rate_entry!(path, entry; remake=false) -> Bool

Insert or replace `entry`'s cell in the table at `path` and rewrite the file.
Returns whether it was written.

The existing row is KEPT, and nothing written, when it passed and either the new
one did not — a re-run never demotes a working value to a broken one — or it is
already at the table's current conditions. Otherwise it is replaced: a legacy row
is promoted once a re-run at current conditions passes, and a non-passing row
always yields to the latest attempt. `remake = true` overwrites unconditionally.
"""
function _write_turn_rate_entry!(path, entry::Dict; remake::Bool = false)
    dict = YAML.load_file(path)
    entries = dict["entries"]
    conditions = dict["conditions"]
    key = _entry_key(entry)
    idx = findfirst(e -> _entry_key(e) == key, entries)

    if isnothing(idx)
        push!(entries, entry)
    elseif remake
        entries[idx] = entry
    else
        old = entries[idx]
        if _entry_passed(old) && !_entry_passed(entry)
            @warn "build_turn_rate_table: keeping existing PASSING row for $key -- the " *
                  "re-run did not pass (outcome = $(get(entry, "outcome", missing))). Not " *
                  "overwriting a working value with a failed one; pass remake=true to force it in."
            return false
        elseif _entry_passed(old) && !_entry_is_legacy(old, conditions)
            @info "build_turn_rate_table: keeping existing passing row for $key (remake=false)"
            return false
        else
            entries[idx] = entry
        end
    end
    sort!(entries; by = e -> _entry_key(e)[2])
    YAML.write_file(path, dict)
    return true
end

"""
    build_turn_rate_table(; depowers, remake=false) -> Vector{NamedTuple}

Identify every depower in `depowers` that is not already a passing row in
`data/turn_rate_coeffs.yaml` (`_fly_low_flights`), writing each result as it
completes and reloading the table at the end. See this file's docstring for the
resume behaviour and wall-time.

`depowers` defaults to the grid the reel-out run needs: it brackets the flown
`depower_setpoint` and `depower_final` with margin, and keeps 0.25 a real row
because `reload_turn_rate_table!` anchors `V3_TURN_RATE_C1`/`C2` there.

A row passes (`outcome: time_limit`) when at least one flight flew the full time;
its coefficients are then the joint fit of those flights. Otherwise it records
the joint fit of the flights that sank (`outcome: low_elevation`), which
[`turn_rate_coeffs`](@ref) does not use, or no coefficients at all when no flight
came below `MAX_ELEVATION` (`outcome: error`). Each row carries `delay` =
`dead_time` + `kite_lag`, `v_app`, the mean apparent wind speed [m/s] of the fit
windows, the residual RMS [rad/s] with and without the lag, `n_runs` (the
flights fitted) and `n_flights`, and the standard errors `*_se` of
`block_standard_errors`.
"""
function build_turn_rate_table(; depowers = [0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40],
                               remake::Bool = false)
    path = joinpath(skc_data_path(), OUT_FILE)
    _check_conditions(YAML.load_file(path))

    results = NamedTuple[]
    for dp in depowers
        dict = YAML.load_file(path)
        idx = findfirst(e -> _entry_key(e) == (Float64.(BODY_START_DAMPING), dp), dict["entries"])
        if !remake && !isnothing(idx) && _entry_passed(dict["entries"][idx]) &&
           !_entry_is_legacy(dict["entries"][idx], dict["conditions"])
            @info "build_turn_rate_table: skipping depower=$dp (already passing at current conditions)"
            continue
        end

        (; flights, joint_flights, joint) = _fly_low_flights(dp)
        steady = count(f -> f.outcome == :time_limit, flights)
        outcome = isnothing(joint) ? "error" : steady > 0 ? "time_limit" : "low_elevation"
        entry = Dict{String, Any}(
            "body_damping" => Float64.(BODY_START_DAMPING),
            "body_sim_damping" => Float64.(BODY_SIM_DAMPING),
            "depower" => dp, "outcome" => outcome,
            "n_runs" => length(joint_flights), "n_flights" => length(flights),
            "date" => string(Dates.today()),
        )
        se = nothing
        if !isnothing(joint)
            se = block_standard_errors([f.fit for f in joint_flights])
            entry["c1"] = joint.c1
            entry["c2"] = joint.c2
            entry["delay"] = joint.dead_time + joint.lag
            entry["dead_time"] = joint.dead_time
            entry["kite_lag"] = joint.lag
            # The dead time scales with the airspeed (docs/course_loop_stability.md),
            # so a delay is only meaningful together with the v_app it was flown at.
            entry["v_app"] = mean(reduce(vcat, [f.fit.v_app for f in joint_flights]))
            entry["rms_lag"] = joint.rms_lag
            entry["rms_delay"] = joint.rms_delay
            entry["c1_se"], entry["c2_se"] = se.c1, se.c2
            entry["dead_time_se"], entry["kite_lag_se"] = se.dead_time, se.lag
        end
        _write_turn_rate_entry!(path, entry; remake)
        push!(results, (; depower = dp, outcome, steady, n_flights = length(flights), joint, se))

        @printf("  depower=%.3f  outcome=%-13s  %d of %d flights steady%s\n", dp, outcome, steady,
                length(flights), isnothing(joint) ? "  (no fit)" : @sprintf("  c1=%.4f", joint.c1))
    end

    println("\n depower   outcome        steady   c1      c1_se     c2      c2_se   dead_t   lag     rms gain")
    for r in results
        isnothing(r.joint) && continue
        j, se = r.joint, r.se
        @printf("  %.3f   %-13s  %d/%d   %7.4f  %7.4f  %6.3f  %6.3f  %6.3f  %6.3f  %6.1f %%\n",
                r.depower, r.outcome, r.steady, r.n_flights, j.c1, se.c1, j.c2, se.c2,
                j.dead_time, j.lag, 100 * (1 - j.rms_lag / j.rms_delay))
    end

    reload_turn_rate_table!()
    return results
end

@info "build_turn_rate_table.jl: definitions loaded -- call build_turn_rate_table() " *
      "yourself (see this file's docstring for the full-grid vs. single-cell forms)."
