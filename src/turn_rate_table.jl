# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Interpolated lookup for the V3 turn-rate-law coefficients `c1`, `c2` and
steering `delay`, backed by `data/turn_rate_coeffs.yaml`. Kept in its own file,
included before `figure_eight_controller.jl`, because it owns state — the loaded
table and the active-run conditions stash — that the guidance does not need to
know about.

The table is read once, at package load (`__init__`);
[`reload_turn_rate_table!`](@ref) re-reads it after a table-building script
appends rows in the same session (`examples/build_turn_rate_table.jl`
identifies the V3's rows).

The rows are identified from simulated steering sweeps of a specific kite at
specific conditions, so this is data ABOUT a plant, not about this controller —
it lives here only because the guidance's feasibility check is the only
consumer. A second kite means a second file, not a second column.
"""

# Quality bar an interpolation neighbour must meet, per V3Kite's steering_test_v3.jl.
const TURN_RATE_MAX_C1_REL_STD = 0.01
const TURN_RATE_MAX_G_REL_STD = 0.35

mutable struct TurnRateTable
    conditions::Dict{Symbol, Any}
    entries::Vector{<:NamedTuple}
end

"""
    _is_usable_turn_rate_entry(e) -> Bool

`true` if entry `e` may be used as an interpolation neighbour: a completed
sweep (`outcome ∈ (:sweep_done, :time_limit)`), a tight `c1` fit, and low gain
scatter. Identification QUALITY is the only bar — a row is never disqualified
for the conditions it was identified at.
"""
function _is_usable_turn_rate_entry(e)
    e.outcome in (:sweep_done, :time_limit) &&
        e.c1_rel_std <= TURN_RATE_MAX_C1_REL_STD &&
        e.g_rel_std <= TURN_RATE_MAX_G_REL_STD
end

"""
    _parse_turn_rate_entry(edict) -> NamedTuple

Parse one `entries:` element of `turn_rate_coeffs.yaml` (a `Dict` with String
keys, as `YAML.load_file` returns it) into a NamedTuple.

An entry may carry its own `l_tether`, `dt` or `date` alongside the table's
`conditions` block. Those are PROVENANCE, not a disqualification: `c1` and `c2`
are normalised by apparent wind speed and identified from relay flights, so
neither the tether length nor the identification timestep changes them, and such
a row is used like any other.
"""
function _parse_turn_rate_entry(edict)
    return (
        body_damping = Float64.(edict["body_damping"]),
        depower = Float64(edict["depower"]),
        c1 = Float64(edict["c1"]),
        c2 = Float64(edict["c2"]),
        delay = Float64(edict["delay"]),
        v_app = Float64(get(edict, "v_app", NaN)),
        dead_time = Float64(get(edict, "dead_time", NaN)),
        kite_lag = Float64(get(edict, "kite_lag", NaN)),
        c1_rel_std = Float64(get(edict, "c1_rel_std", 0.0)),
        g_rel_std = Float64(get(edict, "g_rel_std", 0.0)),
        outcome = Symbol(get(edict, "outcome", "sweep_done")),
    )
end

"""
    _load_turn_rate_table(project = project_file()) -> TurnRateTable

Read and parse the system project's `turn_rate_coeffs` file
([`turn_rate_coeffs_file`](@ref)) fresh from disk.
"""
function _load_turn_rate_table(project = project_file())
    path = joinpath(skc_data_path(), turn_rate_coeffs_file(project))
    raw = YAML.load_file(path)
    conditions = Dict{Symbol, Any}(Symbol(k) => v for (k, v) in raw["conditions"])
    # NamedTuple[...] and not a bare comprehension: over an empty `entries: []` the
    # latter infers Vector{Any}, which does not convert to the field's
    # Vector{<:NamedTuple}, and a table cleared for a fresh identification run would
    # fail to load from __init__ (2026-09-22).
    entries = NamedTuple[_parse_turn_rate_entry(e) for e in raw["entries"]]
    return TurnRateTable(conditions, entries)
end

const _TURN_RATE_TABLE = Ref{TurnRateTable}()

"""
    reload_turn_rate_table!(project = project_file())

Re-read the system project's `turn_rate_coeffs` file ([`turn_rate_coeffs_file`](@ref))
and refresh [`turn_rate_coeffs`](@ref), [`V3_TURN_RATE_COEFFS`](@ref),
[`V3_TURN_RATE_C1`](@ref) and [`V3_TURN_RATE_C2`](@ref) from it. The table is
otherwise read only at package load, against the default `project` — call this
after `examples/build_turn_rate_table.jl` appends rows in the same
session, instead of restarting.

If the `[0,0,40]`/0.25 lookup behind `V3_TURN_RATE_C1`/`C2` throws, this function
warns and leaves them at their previous value (`NaN` before the first successful
load): a data-quality problem in one grid cell must never become a load-time
failure, since this runs from `__init__`. `turn_rate_coeffs` still throws
normally for any other caller asking for that combination.
"""
function reload_turn_rate_table!(project = project_file())
    table = _load_turn_rate_table(project)
    _TURN_RATE_TABLE[] = table
    global V3_TURN_RATE_COEFFS = Dict((e.body_damping, e.depower) => (c1 = e.c1, c2 = e.c2, delay = e.delay)
                                       for e in table.entries)
    try
        default = turn_rate_coeffs([0.0, 0.0, 40.0], 0.25)
        global V3_TURN_RATE_C1 = default.c1
        global V3_TURN_RATE_C2 = default.c2
    catch e
        @error "reload_turn_rate_table!: could not refresh V3_TURN_RATE_C1/C2 -- " *
               "the [0,0,40]/0.25 row in data/$(turn_rate_coeffs_file(project)) is unusable. " *
               "Leaving them at their previous value. Re-identify that cell with " *
               "examples/build_turn_rate_table.jl." exception=(e, catch_backtrace())
    end
    return nothing
end

# A table built in code (the tests) may leave the column out.
_row_v_app(e) = get(e, :v_app, NaN)
_row_dead_time(e) = get(e, :dead_time, NaN)
_row_kite_lag(e) = get(e, :kite_lag, NaN)

"""
    turn_rate_coeffs(body_damping, depower; interpolate=true, table) -> (; c1, c2, delay, v_app, dead_time, kite_lag, interpolated)

Look up the V3 turn-rate-law coefficients for a given `body_damping` and
`depower_setpoint`, from `data/turn_rate_coeffs.yaml`
([`V3_TURN_RATE_COEFFS`](@ref) shows its grid points).

- An exact `(body_damping, depower)` hit returns that row's values unchanged,
  `interpolated = false`. A row whose own identification did not pass (`outcome`
  other than `:sweep_done`/`:time_limit`, or too much scatter) throws instead of
  returning it — a failed sweep is recorded, never looked up as if it were data.
  Nothing else disqualifies a row: identification quality is the only bar, and
  a row identified at its own tether length or timestep counts like any other.
- Between two grid points *of the same `body_damping`*, `c1` is interpolated
  log-linearly (it decays close to exponentially with depower) and `c2`/`delay`
  linearly; `delay` is then rounded up to a multiple of the identification
  `dt`, so an interpolated dead time is never optimistic. Non-passing rows are
  never used as neighbours. `interpolate = false` disables this and throws
  instead.
- Outside the identified depower range for that damping, or for a
  `body_damping` with no rows at all, this **throws** rather than
  extrapolating or guessing — re-identify by running
  `examples/build_turn_rate_table.jl`, which flies the missing cells and appends
  the rows itself.

`v_app` [m/s] is the mean apparent wind speed of the flights the row was
identified at, interpolated linearly like `delay`; `NaN` for a row identified
before it was recorded. The dead time falls with the airspeed, so `delay` holds
at that `v_app` only (`docs/course_loop_stability.md`).

`dead_time` and `kite_lag` [s] split `delay` into a dead time and a first-order
lag of the kite, identified on the same flights at the same `v_app`
(`joint_delay_lag_fit`, in `examples/build_turn_rate_table.jl`); interpolated
linearly like `delay`, `NaN` for a row without them.

`table` defaults to the session's table (loaded by [`reload_turn_rate_table!`](@ref));
pass another `TurnRateTable` (`_load_turn_rate_table(project)`) to look up a second
table without replacing the session's, e.g. one for the controller and one for the
path planning in `examples/simple_opt_reelout.jl`.

**Both arguments matter.** Depowering 0.25 → 0.55 costs a factor 2.95 of
steering authority *and* raises the steering dead time from 0.03 s to 0.55 s.
Body damping is never interpolated across: it is a 3-vector with a violently
nonlinear effect on `c1`, so a `body_damping` with no identified rows throws
rather than guessing from a nearby one.
"""
function turn_rate_coeffs(body_damping, depower; interpolate::Bool = true,
                          table::TurnRateTable = _TURN_RATE_TABLE[])
    bd = collect(Float64.(body_damping))
    dp = Float64(depower)

    group = filter(e -> e.body_damping == bd, table.entries)
    if isempty(group)
        known = sort(unique(e.body_damping for e in table.entries); by = string)
        throw(ArgumentError(
            "No identified turn-rate coefficients for body_damping = $bd. " *
            "Known: $known. Re-identify by running examples/build_turn_rate_table.jl."))
    end

    exact = findfirst(e -> e.depower == dp, group)
    if !isnothing(exact)
        e = group[exact]
        if !_is_usable_turn_rate_entry(e)
            throw(ArgumentError(
                "The turn-rate entry for body_damping = $bd, depower = $dp did not " *
                "produce usable coefficients (outcome = $(e.outcome), " *
                "c1_rel_std = $(e.c1_rel_std), g_rel_std = $(e.g_rel_std)). " *
                "Re-identify with examples/build_turn_rate_table.jl."))
        end
        return (c1 = e.c1, c2 = e.c2, delay = e.delay, v_app = _row_v_app(e),
                dead_time = _row_dead_time(e), kite_lag = _row_kite_lag(e), interpolated = false)
    end

    interpolate || throw(ArgumentError(
        "No exact turn-rate entry for body_damping = $bd, depower = $dp " *
        "(interpolate = false)."))

    usable = sort(filter(_is_usable_turn_rate_entry, group); by = e -> e.depower)
    if length(usable) < 2
        throw(ArgumentError(
            "Not enough usable turn-rate entries to interpolate for " *
            "body_damping = $bd (need >= 2, have $(length(usable))). " *
            "Run examples/build_turn_rate_table.jl for more depower values."))
    end

    depowers = [e.depower for e in usable]
    if dp < first(depowers) || dp > last(depowers)
        throw(ArgumentError(
            "depower = $dp is outside the identified range " *
            "[$(first(depowers)), $(last(depowers))] for body_damping = $bd. " *
            "No extrapolation -- re-identify at this depower first."))
    end

    i = searchsortedlast(depowers, dp)
    lo, hi = usable[i], usable[i + 1]
    t = (dp - lo.depower) / (hi.depower - lo.depower)
    c1 = exp(log(lo.c1) + t * (log(hi.c1) - log(lo.c1)))
    c2 = lo.c2 + t * (hi.c2 - lo.c2)
    delay = lo.delay + t * (hi.delay - lo.delay)
    v_app = _row_v_app(lo) + t * (_row_v_app(hi) - _row_v_app(lo))
    dead_time = _row_dead_time(lo) + t * (_row_dead_time(hi) - _row_dead_time(lo))
    kite_lag = _row_kite_lag(lo) + t * (_row_kite_lag(hi) - _row_kite_lag(lo))
    dt = get(table.conditions, :dt, nothing)
    isnothing(dt) || (delay = ceil(delay / Float64(dt)) * Float64(dt))
    return (c1 = c1, c2 = c2, delay = delay, v_app = v_app, dead_time, kite_lag, interpolated = true)
end

"""
    stack_fits(fits, field::Symbol; skip = 0) -> Vector{Float64}

The `field` of every fit in `fits` (one identification window each), the first
`skip` samples of each dropped — the samples a delay shift of up to `skip` leaves
without input — concatenated into one series for a joint fit.
"""
stack_fits(fits, field::Symbol; skip = 0) =
    reduce(vcat, [Float64.(getfield(f, field))[skip + 1:end] for f in fits])

"""
    try_turn_rate_coeffs(fcs; consequence = "flying WITHOUT the feasibility check",
                         info = true) -> NamedTuple or nothing

[`turn_rate_coeffs`](@ref) at `fcs.body_damping` and `fcs.depower_setpoint`, or
`nothing`, with a warning ending in `consequence`, when the table cannot serve that
cell. `turn_rate_coeffs` refuses to extrapolate (by design: c1 moves violently with
both arguments), so a caller that can run on unadvised — a deliberate off-grid run —
uses this instead of aborting. Errors other than its `ArgumentError` are rethrown.
`info = true` logs the coefficients that were found.
"""
function try_turn_rate_coeffs(fcs; consequence = "flying WITHOUT the feasibility check",
                              info = true)
    coeffs = try
        turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint)
    catch exc
        exc isa ArgumentError || rethrow()
        @warn "No turn-rate coefficients for body_damping = $(fcs.body_damping), \
               depower = $(fcs.depower_setpoint) — $consequence. Identify this cell \
               with V3Kite.jl's steering_test_v3.jl to get it back.\n$(exc.msg)"
        return nothing
    end
    info && @info @sprintf("Turn-rate law at body_damping=%s, depower=%.2f%s: \
                            c1 = %.4f 1/m, c2 = %.4f m/s^2, delay = %.3f s",
                           fcs.body_damping, fcs.depower_setpoint,
                           coeffs.interpolated ? " (INTERPOLATED)" : "",
                           coeffs.c1, coeffs.c2, coeffs.delay)
    return coeffs
end

"""
    turn_rate_depower_range(body_damping; table) -> (lo, hi)

The depower interval [`turn_rate_coeffs`](@ref) can serve for `body_damping`
without throwing: the lowest and highest USABLE row (non-passing rows do not
count, exactly as they are never interpolation neighbours). For a caller whose
depower can leave the table — the phase-5 force limiter integrates up to
`depower_final_max`, above the identified grid — so it can saturate its lookup
at the edge instead of losing the coefficient altogether. Throws the same
`ArgumentError` as `turn_rate_coeffs` for a damping with fewer than two usable
rows, since nothing can be interpolated there either. `table` as in
[`turn_rate_coeffs`](@ref).
"""
function turn_rate_depower_range(body_damping; table::TurnRateTable = _TURN_RATE_TABLE[])
    bd = collect(Float64.(body_damping))
    usable = filter(e -> e.body_damping == bd && _is_usable_turn_rate_entry(e),
                    table.entries)
    length(usable) >= 2 || throw(ArgumentError(
        "Not enough usable turn-rate entries for body_damping = $bd " *
        "(need >= 2, have $(length(usable))). " *
        "Run examples/build_turn_rate_table.jl for more depower values."))
    return extrema(e.depower for e in usable)
end

"""
    V3_TURN_RATE_COEFFS

Snapshot of every row of `data/turn_rate_coeffs.yaml`, as parsed at package
load (or by the last [`reload_turn_rate_table!`](@ref)), keyed by
`(body_damping, depower)`. Includes non-passing rows; prefer
[`turn_rate_coeffs`](@ref), which applies the quality filtering and
interpolates between grid points.
"""
V3_TURN_RATE_COEFFS = Dict{Tuple{Vector{Float64}, Float64}, Any}()

"""
    V3_TURN_RATE_C1

`c1` for V3Kite `init`'s default `body_damping = [0.0, 0.0, 40.0]` at depower 0.25.
Prefer [`turn_rate_coeffs`](@ref) whenever the damping or depower differs.
"""
V3_TURN_RATE_C1 = NaN

"""
    V3_TURN_RATE_C2

`c2` for V3Kite `init`'s default `body_damping` at depower 0.25 — see
[`V3_TURN_RATE_C1`](@ref).
"""
V3_TURN_RATE_C2 = NaN
