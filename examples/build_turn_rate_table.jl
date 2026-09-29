# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Fill `data/turn_rate_coeffs.yaml` with the `(body_damping, depower)` cells this
package needs, so [`turn_rate_coeffs`](@ref) can interpolate instead of throwing.
One `steering_test_v3.jl`-style sweep per cell — settle, hold a constant tether
length, relay-oscillate the heading with a stepped steering amplitude, then fit
`identify_turn_rate_law` on the log — factored into `_run_turn_rate_sweep` and
driven over a grid by `build_turn_rate_table`.

Ported from V3Kite.jl's `examples/build_turn_rate_table.jl` (branch `fig8`,
commit 9f1be96), which was never merged to its main. It lives here now because
it writes THIS package's data file and flies THIS package's project: the sweep
must see the same plant the runs do — same wing mass, tether diameter and KCU
rate limits — or it identifies a kite nobody flies.

Writes incrementally: after every cell the whole file is re-read, that cell's row
inserted or replaced, and the file rewritten, so a diverged run costs one cell
and not the grid. `remake = false` (the default) skips a cell whose row already
passed at the table's current `conditions`, and `_write_turn_rate_entry!`
separately refuses to let a failed re-run demote a row that passed. Once this
script has written to the YAML its formatting is `YAML.write_file`'s, not the
hand-authored layout.

`include` only loads the definitions — this script is called repeatedly with
different arguments rather than once with fixed ones. Call it yourself:

    build_turn_rate_table()                                  # the whole grid
    build_turn_rate_table(depowers = [0.25],                 # one cell, capped lower
        max_steering_cap = 0.15)
    build_turn_rate_table(c3 = SWEEP_C3)                     # c3 fixed, Eq. (9)

With `c3` given, the sweep is fitted with the turn-rate law of Eq. (9) of the paper,
`ψ̇ = c1·v_a·u_s + c3·sin(ψ)·cos(β)`, with the gravity coefficient held at `c3`
(`SWEEP_C3` = 0.23 1/s, identified on the flown figures of eight by
`identify_c3.jl`), so only `c1` and the delay are fitted. The relay sweep cannot
identify the gravity term itself: a shorter delay trades against a larger free
`c2`. These rows go to their own table, `turn_rate_coeffs_c3.yaml` by default,
created from the conditions of `turn_rate_coeffs.yaml` plus `c3` on first use, and
store `c3` together with the table-form `c2 = c3·v_app`. The controllers read the
table named by the project, so this one is not loaded by `reload_turn_rate_table!`.

Expect roughly two minutes per cell (measured here 2026-09-22; the quarter of an
hour this said before was inherited from V3Kite.jl and never re-measured against
this plant, which made a cheap re-run look expensive) and a settling-cache miss
on every new `(body_damping, depower)` pair. Expect some cells to fail outright
too: the plant does not survive every depower at every amplitude. Afterwards
the table is reloaded into the running session, so
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
using Statistics: mean, std
import Dates

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))
include(joinpath(@__DIR__, "delay_lag_fit.jl"))

# ============== FIXED CONDITIONS (data/turn_rate_coeffs.yaml) ============== #
# Only body_damping and depower vary across the grid. These must agree with the
# file's `conditions:` block, which `_check_conditions` enforces.
#
# `SWEEP_`-prefixed where every run script assigns the bare name as a plain
# global (`PROJECT`, `SIM_TIME`, `AERO_MODE`): this file is meant to be included
# into the SAME session those scripts run in (its docstring: the table is reloaded
# there afterwards), and a `const` of the same name makes their next `include`
# die on "invalid assignment to constant" (2026-09-18, simple_opt_reelout.jl).

const SWEEP_PROJECT    = project_file("system_reelout_maasvlakte.yaml")
const V_WIND           = 9.51
const TETHER_LENGTH    = 150.0
const ELEVATION        = 73.0
const DT               = 0.05 / 3
const SWEEP_SIM_TIME   = 200.0
const SWEEP_AERO_MODE  = ContinuousAero()
const VSM_INTERVAL     = 5

# Settling starts at the first and decays to the second, which is what the sweep
# is FLOWN at — the same pair every run here uses (`0.8 .* fcs.body_damping`).
# The row is keyed by the START value, as `fcs.body_damping` is.
const BODY_START_DAMPING = [0.0, 0.0, 40.0]
const BODY_SIM_DAMPING   = 0.8 .* BODY_START_DAMPING

# Relay controller, as in V3Kite's steering_test_v3.jl.
const T_START          = 10.0
const HEADING_OFFSET   = 10.0
const START_STEERING   = 0.05
const STEERING_STEP    = 0.025
const CYCLES_PER_LEVEL = 2
const MAX_STEERING_CAP = 0.175

const MIN_ELEVATION    = 50.0
# [m] a sweep flown with `v_reelout` stops reeling out here, the reel-out runs' length.
const REELOUT_L_MAX    = 380.0
# Cells that cannot pass MIN_ELEVATION: the relay sweep at depower 0.40 sinks to
# 48.1° (2026-09-25) and stops 3.6 s into the excitation at the 50° floor
# (2026-09-27), so it is flown at a lower floor, recorded in its row.
const ELEVATION_FLOORS = Dict(0.40 => 40.0)

"""
    _elevation_floor(depower) -> Float64

The elevation floor the sweep at `depower` is flown with: its `ELEVATION_FLOORS`
entry, or `MIN_ELEVATION`.
"""
_elevation_floor(depower) = get(ELEVATION_FLOORS, Float64(depower), MIN_ELEVATION)
const MIN_STEERING_FIT = START_STEERING / 2

# Blockwise delay scatter (`_delay_std`). The delay is a single best-fit shift
# over the whole window, so it comes with no error bar of its own; re-estimating
# it per block is the cheapest honest substitute. The search range is narrowed from
# `identify_turn_rate_law`'s 10 s default because a short block can match the
# NEXT reversal with the current one and fit a full half-cycle late.
const DELAY_BLOCKS     = 4
const DELAY_BLOCK_TMAX = 3.0

# Dead time + lag split (`_split_delay`): the kite's lag is searched on a grid
# of DT steps up to KITE_LAG_MAX [s]; the dead time over DELAY_BLOCK_TMAX.
const KITE_LAG_MAX     = 1.0

const OUT_FILE = "turn_rate_coeffs.yaml"
const OUT_FILE_C3 = "turn_rate_coeffs_c3.yaml"

# Gravity coefficient [1/s] of Eq. (9), `c3·sin(ψ)·cos(β)`, identified separately on
# the 25 flown reel-out runs (`identify_c3.jl`, 2026-09-28); `C3` of
# `course_loop_model.jl`. Only used when a caller passes `c3 = SWEEP_C3`.
const SWEEP_C3 = 0.23

"`OUT_FILE`, or `OUT_FILE_C3` for a fit with `c3` fixed."
_out_file(c3) = isnothing(c3) ? OUT_FILE : OUT_FILE_C3

"""
    _check_conditions(dict; c3=nothing)

Throw unless the `conditions:` block of the loaded table agrees with this
script's constants, and with `c3`: a table fitted with `c3` fixed carries it in
its conditions, a free-`c2` table must not. A row written under conditions the
block does not describe is unusable data that looks like data.
"""
function _check_conditions(dict; c3 = nothing)
    want = Dict{String, Any}("system" => basename(SWEEP_PROJECT), "v_wind" => V_WIND,
                "l_tether" => TETHER_LENGTH, "elevation" => ELEVATION, "dt" => DT)
    if isnothing(c3)
        haskey(dict["conditions"], "c3") &&
            error("build_turn_rate_table: this table was fitted with c3 = " *
                  "$(dict["conditions"]["c3"]) fixed; pass that c3, or write to $OUT_FILE.")
    else
        want["c3"] = c3
    end
    for (k, v) in want
        have = get(dict["conditions"], k, missing)
        isapprox_ok = have isa Real && v isa Real ? isapprox(have, v; rtol = 1e-4) : have == v
        isapprox_ok || error("build_turn_rate_table: conditions[$k] is $have, this script " *
                             "sweeps at $v. Update the conditions block of data/$OUT_FILE " *
                             "(or this script) before writing rows against it.")
    end
end

"""
    _ensure_table(path, c3)

Create the table at `path` for a fit with `c3` fixed, if it does not exist yet:
the `conditions` of `OUT_FILE` plus `c3`, and no entries. Throws for a missing
free-`c2` table, which is never created from scratch.
"""
function _ensure_table(path, c3)
    isfile(path) && return nothing
    isnothing(c3) && error("build_turn_rate_table: $path not found")
    conditions = YAML.load_file(joinpath(skc_data_path(), OUT_FILE))["conditions"]
    conditions["c3"] = Float64(c3)
    YAML.write_file(path, Dict("conditions" => conditions, "entries" => Any[]))
    @info "build_turn_rate_table: created $path for c3 = $c3"
    return nothing
end

"""
    _identify(sl; c3=nothing) -> NamedTuple

`identify_turn_rate_law` on the sweep log `sl`. With `c3` given, the delay and `c1`
are re-fitted on its analysis window with the gravity term of Eq. (9) held fixed
(`estimate_delay_fit_c3`, `fit_c1_c3`), and every field that depends on them is
replaced; `c2` is then `c3·mean(v_app)` and `se2` 0. The result also carries `c3`
(`nothing` for the free-`c2` fit).
"""
function _identify(sl; c3 = nothing)
    fit = identify_turn_rate_law(sl; dt = DT, t_start = T_START, min_steering = MIN_STEERING_FIT)
    isnothing(c3) && return merge(fit, (; c3))
    n = length(fit.us)
    d, _, d_frac = estimate_delay_fit_c3(fit.us, fit.rate, fit.v_app, fit.psi, fit.beta, DT; c3)
    us_del = shift_delay(fit.us, d)
    gain = turn_rate_gain(us_del, fit.rate, fit.v_app; min_steering = MIN_STEERING_FIT)
    c = fit_c1_c3(fit.v_app, fit.psi, fit.beta, fit.rate, us_del; c3)
    us_est = (fit.rate .- c3 .* sin.(fit.psi) .* cos.(fit.beta)) ./ (c.c1 .* fit.v_app)
    delay_corr = V3Kite.corr(view(fit.us, 1:n-d), view(fit.rate ./ fit.v_app, 1+d:n))
    return merge(fit, (; us_del, G = gain.G, us_est, delay_samples = d,
                       delay_sec = max(d_frac - 0.5, 0.0) * DT, delay_corr,
                       G_mean = gain.mean, G_std = gain.std, G_rel_std = gain.rel_std,
                       n_gain = gain.n, c1 = c.c1, c2 = c.c2, se1 = c.se1, se2 = c.se2,
                       rms = c.rms, cond = c.cond, n_fit = c.n, c3))
end

"""
    _delay_fit(us, rate, v_app, psi, beta; c3=nothing, t_max) -> (d, rms, d_frac)

`estimate_delay_fit` at `DT`, or `estimate_delay_fit_c3` when `c3` is given.
"""
function _delay_fit(us, rate, v_app, psi, beta; c3 = nothing, t_max::Real)
    isnothing(c3) ? estimate_delay_fit(us, rate, v_app, psi, beta, DT; t_max) :
                    estimate_delay_fit_c3(us, rate, v_app, psi, beta, DT; c3, t_max)
end

"""
    _run_turn_rate_sweep(depower; max_steering_cap=MAX_STEERING_CAP,
                         elevation_floor=MIN_ELEVATION, v_wind=V_WIND,
                         c3=nothing, elevation=ELEVATION,
                         heading_center=0.0, start_steering=START_STEERING,
                         steering_step=STEERING_STEP, az_reverse=nothing,
                         el_hold=nothing, el_hold_gain=3.0,
                         el_hold_tilt=45.0, v_reelout=0.0) -> NamedTuple

One steering-amplitude sweep at the fixed conditions above, for `depower`.
`v_wind` [m/s] other than `V_WIND` flies the sweep at another airspeed, for the
scaling of the dead time and lag over `v_app`; such a run is not a table row.

`elevation_floor` is the elevation below which the sweep is abandoned as
`:low_elevation`. Relax it for a cell the wing cannot hold at 50° — `c1`/`c2`
are normalised by apparent wind, so the sag costs no coefficient quality, and
the pattern flies such a depower at ~30° anyway (Cabauw 10 m/s, 2026-09-18,
rel_depower 0.355-0.370 for the whole reel-out, off the table's 0.35 end).

The kite is settled and parked at constant tether length (the winch length loop
is the caller's), then a relay controller flips the steering between `-u_s` and
`+u_s` whenever the heading leaves the `±HEADING_OFFSET` band, stepping `u_s` up
by `STEERING_STEP` after `CYCLES_PER_LEVEL` upward crossings.

`elevation` [°] is the start elevation and `heading_center` [°] the centre of the
relay's heading band (0 = flying straight up). Both default to the table's sweep;
other values fly a sweep lower or crosswind (`plot_turn_rate_identification.jl`),
which is not a table row either. `start_steering` and `steering_step` set the
amplitude ladder; `steering_step = 0` with a `max_steering_cap` above
`start_steering` flies one amplitude until the time limit or the floor.

Crosswind, a fixed band centre flies the kite out of the wind window. With
`az_reverse` [°] set, the centre is `±heading_center` and flips sign when the
azimuth passes `±az_reverse` in the direction of travel (heading > 0 moves the
azimuth up); the turn between the two always goes through heading 0, upwards.
With `el_hold` [°] set, the centre's magnitude is tilted by `el_hold_gain` [°/°]
per degree above (down) or below (up) `el_hold`, clamped to `heading_center ±
el_hold_tilt`, which holds the kite near that elevation. A fast turn (a large
amplitude) overshoots the band, so it needs a smaller `el_hold_tilt`, or the
heading passes 180° (straight down) and the kite loops into the ground.

`v_reelout` [m/s] reels the tether out from `T_START` on, ramped in over 2 s and
fed forward to the length loop, until `REELOUT_L_MAX`; 0 (the table's) holds the
length.

Returns `(; outcome, u_s_max, min_elevation, fit, sl)`. `outcome` is `:sweep_done`
(reached `max_steering_cap`), `:time_limit`, `:low_elevation`, or `:error` (the
solver diverged — the fit still runs on whatever was logged). `fit` is the
`identify_turn_rate_law` result, with `c3` given re-fitted by `_identify` with the
gravity term of Eq. (9) fixed, or `nothing` when even that failed. `sl` is the
sweep's log, for plotting (`plot_turn_rate_identification.jl`).
"""
function _run_turn_rate_sweep(depower; max_steering_cap::Real = MAX_STEERING_CAP,
                              elevation_floor::Real = MIN_ELEVATION, v_wind::Real = V_WIND,
                              c3::Union{Nothing, Real} = nothing,
                              elevation::Real = ELEVATION, heading_center::Real = 0.0,
                              start_steering::Real = START_STEERING,
                              steering_step::Real = STEERING_STEP,
                              az_reverse::Union{Nothing, Real} = nothing,
                              el_hold::Union{Nothing, Real} = nothing, el_hold_gain::Real = 3.0,
                              el_hold_tilt::Real = 45.0, v_reelout::Real = 0.0)
    @info @sprintf("build_turn_rate_table: depower = %.3f, max_steering_cap = %.3f, \
                    elevation floor %.1f°, wind %.2f m/s", depower, max_steering_cap,
                   elevation_floor, v_wind)
    s = init(v_wind, TETHER_LENGTH; body_start_damping = BODY_START_DAMPING,
        body_sim_damping = BODY_SIM_DAMPING, elevation,
        depower_setpoint = depower, sim_time = SWEEP_SIM_TIME, dt = DT,
        system_yaml = SWEEP_PROJECT, aero_mode = SWEEP_AERO_MODE, remake_model = false)

    l0 = s.sys_state.l_tether[1]
    l_set = l0
    wpc = WinchPosController(WCSettings(true; dt = s.dt); dt = s.dt)

    steering = start_steering
    side = 1.0              # sign of the band centre, flipped by `az_reverse`
    rel_steering = 0.0
    heading = 0.0
    cycles = 0
    min_elevation = Inf
    outcome = :time_limit

    try
        for _ in 1:s.steps
            t = s.sys_state.time + s.dt
            if T_START <= t < T_START + s.dt
                rel_steering = -steering
            end
            last_heading = heading
            if t > T_START + s.dt
                if !isnothing(az_reverse)
                    az = rad2deg(s.sys_state.azimuth)
                    side > 0 && az > az_reverse && (side = -1.0)
                    side < 0 && az < -az_reverse && (side = 1.0)
                end
                center = heading_center
                if !isnothing(el_hold)
                    center = clamp(center + el_hold_gain * (rad2deg(s.sys_state.elevation) - el_hold),
                                   heading_center - el_hold_tilt, heading_center + el_hold_tilt)
                end
                # Relative to the band's centre, so the relay logic is the same for any centre.
                # NOT wrapped: heading and centre both lie in (-180°, 180°], so the plain
                # difference never turns the kite through ±180° (straight down), and passes
                # through 0 (straight up) whenever a reversal flips the centre's sign.
                heading = wrap_to_pi(s.sys_state.heading) - deg2rad(side * center)
                if rad2deg(heading) < -HEADING_OFFSET
                    rel_steering = steering
                elseif rad2deg(heading) > HEADING_OFFSET
                    rel_steering = -steering
                    # One increment per crossing, not one per timestep.
                    if rad2deg(last_heading) <= HEADING_OFFSET
                        cycles += 1
                        if cycles >= CYCLES_PER_LEVEL
                            if steering >= max_steering_cap - 1e-9
                                outcome = :sweep_done
                                break
                            end
                            cycles = 0
                            steering = min(steering + steering_step, max_steering_cap)
                            @info @sprintf("  t = %6.2f s: steering amplitude -> %.3f",
                                           t, steering)
                        end
                    end
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
        @warn "build_turn_rate_table: run diverged" depower exception = (e, catch_backtrace())
    end

    sl = KiteUtils.syslog(s.logger)
    fit = try
        _identify(sl; c3)
    catch e
        @warn "build_turn_rate_table: identification failed, no coefficients recorded" depower exception = (e, catch_backtrace())
        nothing
    end

    return (; outcome, u_s_max = steering, min_elevation, fit, sl)
end

"""
    _delay_std(fit; nblocks=DELAY_BLOCKS) -> Float64

Scatter [s] of the transport delay over `nblocks` equal-length blocks of the
analysis window of `fit`, each re-estimated with `_delay_fit` exactly as the
window-wide one was (with `c3` fixed if `fit.c3` is set). `NaN` if the window is
too short to split.

This is a spread across sub-runs, not a standard error of the window-wide
estimate: the delay is quantised to `dt` and rises with steering amplitude, so
expect it to be dominated by the sweep's own stepping rather than by noise. It
says how well ONE delay describes the whole sweep, which is what the table's
single `delay` value claims.
"""
function _delay_std(fit; nblocks::Int = DELAY_BLOCKS)
    n = length(fit.time)
    edges = round.(Int, range(1, n + 1; length = nblocks + 1))
    delays = Float64[]
    for b in 1:nblocks
        rng = edges[b]:(edges[b + 1] - 1)
        length(rng) < 4 && continue
        d, _ = _delay_fit(fit.us[rng], fit.rate[rng], fit.v_app[rng], fit.psi[rng],
                          fit.beta[rng]; fit.c3, t_max = DELAY_BLOCK_TMAX)
        push!(delays, d * DT)
    end
    return length(delays) > 1 ? std(delays) : NaN
end

"""
    _delay_over_v_app(fit) -> NamedTuple

The pure delay of `identify_turn_rate_law`, re-fitted on the first and the
second half of the analysis window of `fit` separately, each at its own mean
`v_app`, and its exponent over `v_app` as `delay ∝ v_app^-exp`:
`exp = ln(d1/d2)/ln(v2/v1)`. `NaN` when either delay is 0.

Returns `(; v_app, delay, delay_exp)`, the first two as `(first half, second
half)`. A check, not a measurement of the scaling: the steering amplitude steps
up through the sweep and `v_app` rises with it (12.8 → 14.4 m/s at 9.51 m/s of
wind, 20.5 → 27.4 m/s at 15 m/s, depower 0.275, 2026-09-26), so the halves
differ in amplitude as much as in airspeed. The dead time and the lag are not
split per half: the fit trades one against the other between the halves
(exponents −4 and +5 at 15 m/s) while their sum stays put. Their scaling comes
from sweeps at two wind speeds (`_run_turn_rate_sweep(...; v_wind)`).
"""
function _delay_over_v_app(fit)
    n = length(fit.us)
    halves = map((1:n ÷ 2, n ÷ 2 + 1:n)) do rng
        _, _, d_frac = _delay_fit(fit.us[rng], fit.rate[rng], fit.v_app[rng], fit.psi[rng],
                                  fit.beta[rng]; fit.c3, t_max = DELAY_BLOCK_TMAX)
        (; v_app = mean(fit.v_app[rng]), delay = max(d_frac - 0.5, 0.0) * DT)
    end
    v, delay = getfield.(halves, :v_app), getfield.(halves, :delay)
    delay_exp = all(>(0), delay) ? log(delay[1] / delay[2]) / log(v[2] / v[1]) : NaN
    return (; v_app = v, delay, delay_exp)
end

"""
    _split_delay(fit) -> NamedTuple

[`fit_delay_lag`](@ref) at this script's sample time and search ranges, with `c3`
fixed if `fit.c3` is set.
"""
_split_delay(fit) = fit_delay_lag(fit, DT; lag_max = KITE_LAG_MAX, t_max = DELAY_BLOCK_TMAX,
                                  fit.c3)

"""
    _entry_key(e) -> (Vector{Float64}, Float64)

`(body_damping, depower)` of a YAML entry dict, for matching against existing rows.
"""
_entry_key(e) = (Float64.(e["body_damping"]), Float64(e["depower"]))

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

The existing row is KEPT, and nothing written, when it passed
(`outcome ∈ (:sweep_done, :time_limit)`) and either the new one did not — a
re-run never demotes a working value to a broken one — or it is already at the
table's current conditions. Otherwise it is replaced: a legacy row is promoted
once a re-run at current conditions passes, and a non-passing row always yields
to the latest attempt. `remake = true` overwrites unconditionally.
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
        old_passing = Symbol(get(old, "outcome", "")) in (:sweep_done, :time_limit)
        new_passing = Symbol(get(entry, "outcome", "")) in (:sweep_done, :time_limit)
        if old_passing && !new_passing
            @warn "build_turn_rate_table: keeping existing PASSING row for $key -- the " *
                  "re-run did not pass (outcome = $(get(entry, "outcome", missing))). Not " *
                  "overwriting a working value with a failed one; pass remake=true to force it in."
            return false
        elseif old_passing && !_entry_is_legacy(old, conditions)
            @info "build_turn_rate_table: keeping existing passing row for $key (remake=false)"
            return false
        else
            entries[idx] = entry
        end
    end
    YAML.write_file(path, dict)
    return true
end

"""
    build_turn_rate_table(; depowers, c3=nothing, out, remake=false,
                          max_steering_cap=MAX_STEERING_CAP,
                          elevation_floor=nothing) -> Vector{NamedTuple}

Sweep every depower in `depowers` that is not already a passing row in
`data/out`, writing each result as it completes and reloading the table at the
end. See this file's docstring for the resume behaviour and wall-time.

`depowers` defaults to the grid the reel-out run needs: it brackets the flown
`depower_setpoint` and `depower_final` with margin, and keeps 0.25 a real row
because `reload_turn_rate_table!` anchors `V3_TURN_RATE_C1`/`C2` there.

`max_steering_cap` is the amplitude ceiling for every cell in this call — pass a
narrower `depowers` to retry one cell at a different cap instead of recomputing
the grid. `elevation_floor`, if given, likewise applies to every cell here;
by default each cell is flown at its own floor, `_elevation_floor(depower)`
(`ELEVATION_FLOORS`, else `MIN_ELEVATION`). A row swept under a floor other than
`MIN_ELEVATION` records it as `elevation_floor`.

`c3` [1/s], if given (`SWEEP_C3`), fits every cell with the gravity term of
Eq. (9) held at `c3`, and `out` then defaults to `turn_rate_coeffs_c3.yaml`
(created on first use) instead of `turn_rate_coeffs.yaml`; see this file's docstring.
"""
function build_turn_rate_table(;
        depowers = [0.25, 0.275, 0.30, 0.325, 0.35, 0.375, 0.40],
        c3::Union{Nothing, Real} = nothing,
        out::String = _out_file(c3),
        remake::Bool = false,
        max_steering_cap::Real = MAX_STEERING_CAP,
        elevation_floor::Union{Nothing, Real} = nothing)
    path = joinpath(skc_data_path(), out)
    _ensure_table(path, c3)
    _check_conditions(YAML.load_file(path); c3)

    results = NamedTuple[]
    for dp in depowers
        dict = YAML.load_file(path)
        idx = findfirst(e -> _entry_key(e) == (Float64.(BODY_START_DAMPING), dp),
                        dict["entries"])
        if !remake && !isnothing(idx) &&
           !_entry_is_legacy(dict["entries"][idx], dict["conditions"]) &&
           Symbol(get(dict["entries"][idx], "outcome", "")) in (:sweep_done, :time_limit)
            @info "build_turn_rate_table: skipping depower=$dp (already passing at current conditions)"
            continue
        end

        cell_floor = isnothing(elevation_floor) ? _elevation_floor(dp) : elevation_floor
        r = _run_turn_rate_sweep(dp; max_steering_cap, elevation_floor = cell_floor, c3)
        split = isnothing(r.fit) ? nothing : _split_delay(r.fit)
        halves = isnothing(r.fit) ? nothing : _delay_over_v_app(r.fit)
        entry = Dict{String, Any}(
            "body_damping" => Float64.(BODY_START_DAMPING),
            "body_sim_damping" => Float64.(BODY_SIM_DAMPING),
            "depower" => dp, "outcome" => String(r.outcome),
            "u_s_max" => r.u_s_max, "min_elevation" => r.min_elevation,
            "date" => string(Dates.today()),
        )
        cell_floor == MIN_ELEVATION ||
            (entry["elevation_floor"] = Float64(cell_floor))
        if !isnothing(r.fit)
            entry["c1"] = r.fit.c1
            entry["c2"] = r.fit.c2
            entry["delay"] = r.fit.delay_sec
            # The dead time scales with the airspeed (docs/course_loop_stability.md),
            # so a delay is only meaningful together with the v_app it was flown at.
            entry["v_app"] = mean(r.fit.v_app)
            entry["c1_rel_std"] = abs(r.fit.se1 / r.fit.c1)
            entry["g_rel_std"] = r.fit.G_rel_std
            # Absolute scatter alongside the relative one: se1/se2 are the fit's own
            # standard errors, delay_std the blockwise spread (`_delay_std`).
            entry["c1_std"] = r.fit.se1
            # c2 is fitted, or with c3 fixed only its table form c3·v_app.
            isnothing(c3) ? (entry["c2_std"] = r.fit.se2) : (entry["c3"] = Float64(c3))
            entry["delay_std"] = _delay_std(r.fit)
            # `delay` split into a dead time and a lag (`fit_delay_lag`); `delay` stays the
            # pure-delay equivalent.
            entry["dead_time"] = split.dead_time
            entry["kite_lag"] = split.lag
            entry["rms_delay"] = split.rms_delay
            entry["rms_lag"] = split.rms_lag
            # The pure delay per half of the window (`_delay_over_v_app`).
            entry["v_app_halves"] = collect(halves.v_app)
            entry["delay_halves"] = collect(halves.delay)
            entry["delay_exp"] = halves.delay_exp
        end
        _write_turn_rate_entry!(path, entry; remake)
        push!(results, (; depower = dp, r..., split, halves))

        @printf("  depower=%.2f  outcome=%-12s  u_s_max=%.3f  min_el=%.1f°%s\n",
                dp, r.outcome, r.u_s_max, r.min_elevation,
                isnothing(r.fit) ? "  (no fit)" : @sprintf("  c1=%.4f", r.fit.c1))
    end

    # The quality bar turn_rate_coeffs applies before a row may be a neighbour.
    println("\n depower   outcome        c1   c1_std        c2   c2_std   delay  del_std   " *
            "c1_rel_std  g_rel_std  usable   dead_t     lag  rms gain")
    for r in results
        isnothing(r.fit) && continue
        usable = r.outcome in (:sweep_done, :time_limit) &&
                 abs(r.fit.se1 / r.fit.c1) <= 0.01 && r.fit.G_rel_std <= 0.35
        @printf("  %.3f   %-12s  %7.4f  %7.4f  %8.4f  %7.4f  %6.3f  %7.3f  %9.4f  %9.4f  %-6s  %6.3f  %6.3f  %6.1f %%\n",
                r.depower, r.outcome, r.fit.c1, r.fit.se1, r.fit.c2, r.fit.se2,
                r.fit.delay_sec, _delay_std(r.fit),
                abs(r.fit.se1 / r.fit.c1), r.fit.G_rel_std, usable ? "yes" : "NO",
                r.split.dead_time, r.split.lag, 100 * (1 - r.split.rms_lag / r.split.rms_delay))
    end

    # Per half of the window: the exponent of delay ∝ v_app^-exp (`_delay_over_v_app`).
    println("\n depower   v_app halves [m/s]   delay halves [s]   exp")
    for r in results
        isnothing(r.halves) && continue
        h = r.halves
        @printf("  %.3f   %5.2f  %5.2f        %5.3f  %5.3f     %5.2f\n",
                r.depower, h.v_app..., h.delay..., h.delay_exp)
    end

    reload_turn_rate_table!()
    return results
end

"""
    add_delay_lag_split!(; depowers=nothing, c3=nothing, out) -> Vector{NamedTuple}

Re-fly the sweep of each passing row of `BODY_START_DAMPING` in `data/out`
(all of them, or those in `depowers`) and add the dead time + lag split
([`fit_delay_lag`](@ref)) to it: `dead_time`, `kite_lag`, `rms_delay`,
`rms_lag` and `split_date`. Nothing else in the row changes, so `c1`, `c2` and
`delay`, which the flight controllers and the feasibility check read, stay the
records they were. Each row is re-flown at its own `elevation_floor`, at the
table's conditions, so its `v_app` is the airspeed of the split too. `c3` and
`out` as for [`build_turn_rate_table`](@ref).
"""
function add_delay_lag_split!(; depowers = nothing, c3::Union{Nothing, Real} = nothing,
                              out::String = _out_file(c3))
    path = joinpath(skc_data_path(), out)
    _check_conditions(YAML.load_file(path); c3)
    rows = filter(e -> Float64.(e["body_damping"]) == BODY_START_DAMPING &&
                       Symbol(get(e, "outcome", "")) in (:sweep_done, :time_limit) &&
                       (isnothing(depowers) || Float64(e["depower"]) in depowers),
                  YAML.load_file(path)["entries"])
    results = NamedTuple[]
    for row in rows
        dp = Float64(row["depower"])
        r = _run_turn_rate_sweep(dp; max_steering_cap = Float64(row["u_s_max"]),
                                 elevation_floor = Float64(get(row, "elevation_floor", _elevation_floor(dp))),
                                 c3)
        if isnothing(r.fit)
            @warn "add_delay_lag_split!: no fit at depower = $dp; row left unchanged."
            continue
        end
        split = _split_delay(r.fit)
        dict = YAML.load_file(path)
        e = dict["entries"][findfirst(x -> _entry_key(x) == _entry_key(row), dict["entries"])]
        e["dead_time"], e["kite_lag"] = split.dead_time, split.lag
        e["rms_delay"], e["rms_lag"] = split.rms_delay, split.rms_lag
        e["split_date"] = string(Dates.today())
        YAML.write_file(path, dict)
        push!(results, (; depower = dp, v_app = mean(r.fit.v_app), row_v_app = Float64(row["v_app"]),
                        delay = r.fit.delay_sec, split...))
    end
    println("\n depower   v_app [m/s] (row)   delay   dead_t     lag  rms gain")
    for r in results
        @printf("  %.3f   %6.2f (%6.2f)     %6.3f  %6.3f  %6.3f  %6.1f %%\n", r.depower, r.v_app,
                r.row_v_app, r.delay, r.dead_time, r.lag, 100 * (1 - r.rms_lag / r.rms_delay))
    end
    reload_turn_rate_table!()
    return results
end

@info "build_turn_rate_table.jl: definitions loaded -- call build_turn_rate_table() " *
      "yourself (see this file's docstring for the full-grid vs. single-cell forms)."
