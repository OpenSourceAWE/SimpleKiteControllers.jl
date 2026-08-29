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

Expect roughly a quarter of an hour per cell and a settling-cache miss on every
new `(body_damping, depower)` pair, and expect some cells to fail outright: the
plant does not survive every depower at every amplitude. Afterwards the table is
reloaded into the running session, so `test/test_fig8_controller.jl` can be
re-run without restarting.
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
import Dates

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# V3Kite is torque-only; the winch length loop is ours (WinchControllers.jl).
include(joinpath(@__DIR__, "winch_adapter.jl"))

# ============== FIXED CONDITIONS (data/turn_rate_coeffs.yaml) ============== #
# Only body_damping and depower vary across the grid. These must agree with the
# file's `conditions:` block, which `_check_conditions` enforces.

const PROJECT          = project_file("system_reelout_150m.yaml")
const V_WIND           = 9.51
const TETHER_LENGTH    = 150.0
const ELEVATION        = 73.0
const DT               = 0.05 / 3
const SIM_TIME         = 200.0
const AERO_MODE        = ContinuousAero()
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
const MIN_STEERING_FIT = START_STEERING / 2

const OUT_FILE = "turn_rate_coeffs.yaml"

"""
    _check_conditions(dict)

Throw unless the `conditions:` block of the loaded table agrees with this
script's constants. A row written under conditions the block does not describe
is unusable data that looks like data.
"""
function _check_conditions(dict)
    want = Dict("system" => basename(PROJECT), "v_wind" => V_WIND,
                "l_tether" => TETHER_LENGTH, "elevation" => ELEVATION, "dt" => DT)
    for (k, v) in want
        have = get(dict["conditions"], k, missing)
        isapprox_ok = have isa Real && v isa Real ? isapprox(have, v; rtol = 1e-4) : have == v
        isapprox_ok || error("build_turn_rate_table: conditions[$k] is $have, this script " *
                             "sweeps at $v. Update the conditions block of data/$OUT_FILE " *
                             "(or this script) before writing rows against it.")
    end
end

"""
    _run_turn_rate_sweep(depower; max_steering_cap=MAX_STEERING_CAP) -> NamedTuple

One steering-amplitude sweep at the fixed conditions above, for `depower`.

The kite is settled and parked at constant tether length (the winch length loop
is the caller's), then a relay controller flips the steering between `-u_s` and
`+u_s` whenever the heading leaves the `±HEADING_OFFSET` band, stepping `u_s` up
by `STEERING_STEP` after `CYCLES_PER_LEVEL` upward crossings.

Returns `(; outcome, u_s_max, min_elevation, fit)`. `outcome` is `:sweep_done`
(reached `max_steering_cap`), `:time_limit`, `:low_elevation`, or `:error` (the
solver diverged — the fit still runs on whatever was logged). `fit` is the
`identify_turn_rate_law` result, or `nothing` when even that failed.
"""
function _run_turn_rate_sweep(depower; max_steering_cap::Real = MAX_STEERING_CAP)
    @info @sprintf("build_turn_rate_table: depower = %.3f, max_steering_cap = %.3f",
                   depower, max_steering_cap)
    s = init(V_WIND, TETHER_LENGTH; body_start_damping = BODY_START_DAMPING,
        body_sim_damping = BODY_SIM_DAMPING, elevation = ELEVATION,
        depower_setpoint = depower, sim_time = SIM_TIME, dt = DT,
        system_yaml = PROJECT, aero_mode = AERO_MODE, remake_model = false)

    l0 = s.sys_state.l_tether[1]
    wpc = WinchPosController(WCSettings(true; dt = s.dt); dt = s.dt)

    steering = START_STEERING
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
                heading = wrap_to_pi(s.sys_state.heading)
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
                            steering = min(steering + STEERING_STEP, max_steering_cap)
                            @info @sprintf("  t = %6.2f s: steering amplitude -> %.3f",
                                           t, steering)
                        end
                    end
                end
            end

            step!(s; rel_depower = depower, rel_steering,
                  set_torque = winch_torque!(wpc, s, l0), vsm_interval = VSM_INTERVAL)

            el = rad2deg(s.sys_state.elevation)
            min_elevation = min(min_elevation, el)
            if el < MIN_ELEVATION
                @warn @sprintf("  elevation %.2f° below floor %.1f° at t = %.2f s, stopping",
                               el, MIN_ELEVATION, t)
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
        identify_turn_rate_law(sl; dt = DT, t_start = T_START, min_steering = MIN_STEERING_FIT)
    catch e
        @warn "build_turn_rate_table: identification failed, no coefficients recorded" depower exception = (e, catch_backtrace())
        nothing
    end

    return (; outcome, u_s_max = steering, min_elevation, fit)
end

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
    build_turn_rate_table(; depowers, out="turn_rate_coeffs.yaml", remake=false,
                          max_steering_cap=MAX_STEERING_CAP) -> Vector{NamedTuple}

Sweep every depower in `depowers` that is not already a passing row in
`data/out`, writing each result as it completes and reloading the table at the
end. See this file's docstring for the resume behaviour and wall-time.

`depowers` defaults to the grid the reel-out run needs: it brackets the flown
`depower_setpoint` and `depower_final` with margin, and keeps 0.25 a real row
because `reload_turn_rate_table!` anchors `V3_TURN_RATE_C1`/`C2` there.

`max_steering_cap` is the amplitude ceiling for every cell in this call — pass a
narrower `depowers` to retry one cell at a different cap instead of recomputing
the grid.
"""
function build_turn_rate_table(;
        depowers = [0.25, 0.30, 0.35, 0.40],
        out::String = OUT_FILE,
        remake::Bool = false,
        max_steering_cap::Real = MAX_STEERING_CAP)
    path = joinpath(skc_data_path(), out)
    isfile(path) || error("build_turn_rate_table: $path not found")
    _check_conditions(YAML.load_file(path))

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

        r = _run_turn_rate_sweep(dp; max_steering_cap)
        entry = Dict{String, Any}(
            "body_damping" => Float64.(BODY_START_DAMPING),
            "body_sim_damping" => Float64.(BODY_SIM_DAMPING),
            "depower" => dp, "outcome" => String(r.outcome),
            "u_s_max" => r.u_s_max, "min_elevation" => r.min_elevation,
            "date" => string(Dates.today()),
        )
        if !isnothing(r.fit)
            entry["c1"] = r.fit.c1
            entry["c2"] = r.fit.c2
            entry["delay"] = r.fit.delay_sec
            entry["c1_rel_std"] = abs(r.fit.se1 / r.fit.c1)
            entry["g_rel_std"] = r.fit.G_rel_std
        end
        _write_turn_rate_entry!(path, entry; remake)
        push!(results, (; depower = dp, r...))

        @printf("  depower=%.2f  outcome=%-12s  u_s_max=%.3f  min_el=%.1f°%s\n",
                dp, r.outcome, r.u_s_max, r.min_elevation,
                isnothing(r.fit) ? "  (no fit)" : @sprintf("  c1=%.4f", r.fit.c1))
    end

    # The quality bar turn_rate_coeffs applies before a row may be a neighbour.
    println("\n depower   outcome        c1        c2      delay   c1_rel_std  g_rel_std  usable")
    for r in results
        isnothing(r.fit) && continue
        usable = r.outcome in (:sweep_done, :time_limit) &&
                 abs(r.fit.se1 / r.fit.c1) <= 0.01 && r.fit.G_rel_std <= 0.35
        @printf("  %.3f   %-12s  %7.4f  %8.4f  %6.3f  %9.4f  %9.4f  %s\n",
                r.depower, r.outcome, r.fit.c1, r.fit.c2, r.fit.delay_sec,
                abs(r.fit.se1 / r.fit.c1), r.fit.G_rel_std, usable ? "yes" : "NO")
    end

    reload_turn_rate_table!()
    return results
end

@info "build_turn_rate_table.jl: definitions loaded -- call build_turn_rate_table() " *
      "yourself (see this file's docstring for the full-grid vs. single-cell forms)."
