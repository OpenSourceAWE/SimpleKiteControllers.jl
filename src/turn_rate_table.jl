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
appends rows in the same session (V3Kite.jl's
`examples/build_turn_rate_table.jl` identifies the V3's rows).

The rows are identified from simulated steering sweeps of a specific kite at
specific conditions, so this is data ABOUT a plant, not about this controller —
it lives here only because the guidance's feasibility check is the only
consumer. A second kite means a second file, not a second column.
"""

const TURN_RATE_TABLE_FILE = "turn_rate_coeffs.yaml"

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
scatter. Legacy entries are excluded by the caller, not here — that check needs
the table's `conditions`, which this function does not see.
"""
function _is_usable_turn_rate_entry(e)
    e.outcome in (:sweep_done, :time_limit) &&
        e.c1_rel_std <= TURN_RATE_MAX_C1_REL_STD &&
        e.g_rel_std <= TURN_RATE_MAX_G_REL_STD
end

"""
    _parse_turn_rate_entry(edict, conditions) -> NamedTuple

Parse one `entries:` element of `turn_rate_coeffs.yaml` (a `Dict` with String
keys, as `YAML.load_file` returns it) into a NamedTuple, and mark it `legacy`
if any of its own fields override the table's `conditions` (e.g. a per-entry
`l_tether` identified before the table settled on one tether length for
everything — see V3Kite.jl's PlanC1C2.md).
"""
function _parse_turn_rate_entry(edict, conditions::Dict{Symbol, Any})
    overrides = Dict{Symbol, Any}()
    for (k, v) in conditions
        ks = String(k)
        haskey(edict, ks) && edict[ks] != v && (overrides[k] = edict[ks])
    end
    return (
        body_damping = Float64.(edict["body_damping"]),
        depower = Float64(edict["depower"]),
        c1 = Float64(edict["c1"]),
        c2 = Float64(edict["c2"]),
        delay = Float64(edict["delay"]),
        c1_rel_std = Float64(get(edict, "c1_rel_std", 0.0)),
        g_rel_std = Float64(get(edict, "g_rel_std", 0.0)),
        outcome = Symbol(get(edict, "outcome", "sweep_done")),
        legacy = !isempty(overrides),
        overrides = overrides,
    )
end

"""
    _load_turn_rate_table() -> TurnRateTable

Read and parse `data/turn_rate_coeffs.yaml` fresh from disk.
"""
function _load_turn_rate_table()
    path = joinpath(skc_data_path(), TURN_RATE_TABLE_FILE)
    raw = YAML.load_file(path)
    conditions = Dict{Symbol, Any}(Symbol(k) => v for (k, v) in raw["conditions"])
    entries = [_parse_turn_rate_entry(e, conditions) for e in raw["entries"]]
    return TurnRateTable(conditions, entries)
end

const _TURN_RATE_TABLE = Ref{TurnRateTable}()

# Legacy rows already warned about; `maxlog` cannot be used, it dedups by call site.
const _TURN_RATE_WARNED_LEGACY = Ref(Set{Tuple{Vector{Float64}, Float64}}())

"""
    reload_turn_rate_table!()

Re-read `data/turn_rate_coeffs.yaml` and refresh [`turn_rate_coeffs`](@ref),
[`V3_TURN_RATE_COEFFS`](@ref), [`V3_TURN_RATE_C1`](@ref) and
[`V3_TURN_RATE_C2`](@ref) from it. The table is otherwise read only at package
load — call this after V3Kite.jl's `examples/build_turn_rate_table.jl` appends
rows in the same session, instead of restarting.

If the `[0,0,40]`/0.25 lookup behind `V3_TURN_RATE_C1`/`C2` throws, this function
warns and leaves them at their previous value (`NaN` before the first successful
load): a data-quality problem in one grid cell must never become a load-time
failure, since this runs from `__init__`. `turn_rate_coeffs` still throws
normally for any other caller asking for that combination.
"""
function reload_turn_rate_table!()
    table = _load_turn_rate_table()
    _TURN_RATE_TABLE[] = table
    empty!(_TURN_RATE_WARNED_LEGACY[])
    global V3_TURN_RATE_COEFFS = Dict((e.body_damping, e.depower) => (c1 = e.c1, c2 = e.c2, delay = e.delay)
                                       for e in table.entries)
    try
        default = turn_rate_coeffs([0.0, 0.0, 40.0], 0.25)
        global V3_TURN_RATE_C1 = default.c1
        global V3_TURN_RATE_C2 = default.c2
    catch e
        @error "reload_turn_rate_table!: could not refresh V3_TURN_RATE_C1/C2 -- " *
               "the [0,0,40]/0.25 row in data/$(TURN_RATE_TABLE_FILE) is unusable. " *
               "Leaving them at their previous value. Re-identify that cell with " *
               "V3Kite.jl/examples/build_turn_rate_table.jl." exception=(e, catch_backtrace())
    end
    return nothing
end

"""
    turn_rate_coeffs(body_damping, depower; interpolate=true) -> (; c1, c2, delay, interpolated)

Look up the V3 turn-rate-law coefficients for a given `body_damping` and
`depower_setpoint`, from `data/turn_rate_coeffs.yaml`
([`V3_TURN_RATE_COEFFS`](@ref) shows its grid points).

- An exact `(body_damping, depower)` hit returns that row's values unchanged,
  `interpolated = false` — including a *legacy* row (one identified at
  conditions that no longer match the table's, e.g. a different tether length),
  with a one-time warning naming the mismatch. A row whose own identification
  did not pass (`outcome` other than `:sweep_done`/`:time_limit`, or too much
  scatter) throws instead of returning it — a failed sweep is recorded, never
  looked up as if it were data.
- Between two grid points *of the same `body_damping`*, `c1` is interpolated
  log-linearly (it decays close to exponentially with depower) and `c2`/`delay`
  linearly; `delay` is then rounded up to a multiple of the identification
  `dt`, so an interpolated dead time is never optimistic. Legacy and
  non-passing rows are never used as neighbours. `interpolate = false` disables
  this and throws instead.
- Outside the identified depower range for that damping, or for a
  `body_damping` with no rows at all, this **throws** rather than
  extrapolating or guessing — re-identify with V3Kite.jl's `steering_test_v3.jl` +
  V3Kite.jl's `identify_turn_rate_law`, or run `V3Kite.jl/examples/build_turn_rate_table.jl`,
  and add the row.

**Both arguments matter.** Depowering 0.25 → 0.55 costs a factor 2.95 of
steering authority *and* raises the steering dead time from 0.03 s to 0.55 s.
Body damping is never interpolated across: it is a 3-vector with a violently
nonlinear effect on `c1`, so a `body_damping` with no identified rows throws
rather than guessing from a nearby one.
"""
function turn_rate_coeffs(body_damping, depower; interpolate::Bool = true)
    bd = collect(Float64.(body_damping))
    dp = Float64(depower)
    table = _TURN_RATE_TABLE[]

    group = filter(e -> e.body_damping == bd, table.entries)
    if isempty(group)
        known = sort(unique(e.body_damping for e in table.entries); by = string)
        throw(ArgumentError(
            "No identified turn-rate coefficients for body_damping = $bd. " *
            "Known: $known. Re-identify with V3Kite.jl's steering_test_v3.jl + " *
            "V3Kite.jl's identify_turn_rate_law, or run V3Kite.jl/examples/build_turn_rate_table.jl."))
    end

    exact = findfirst(e -> e.depower == dp, group)
    if !isnothing(exact)
        e = group[exact]
        if !_is_usable_turn_rate_entry(e)
            throw(ArgumentError(
                "The turn-rate entry for body_damping = $bd, depower = $dp did not " *
                "produce usable coefficients (outcome = $(e.outcome), " *
                "c1_rel_std = $(e.c1_rel_std), g_rel_std = $(e.g_rel_std)). " *
                "Re-identify with V3Kite.jl/examples/build_turn_rate_table.jl."))
        end
        if e.legacy && (bd, dp) ∉ _TURN_RATE_WARNED_LEGACY[]
            @warn "turn_rate_coeffs($bd, $dp): this row was identified at conditions " *
                  "overriding data/$(TURN_RATE_TABLE_FILE)'s conditions block " *
                  "($(e.overrides)); it is excluded from interpolation."
            push!(_TURN_RATE_WARNED_LEGACY[], (bd, dp))
        end
        return (c1 = e.c1, c2 = e.c2, delay = e.delay, interpolated = false)
    end

    interpolate || throw(ArgumentError(
        "No exact turn-rate entry for body_damping = $bd, depower = $dp " *
        "(interpolate = false)."))

    usable = sort(filter(e -> !e.legacy && _is_usable_turn_rate_entry(e), group);
                  by = e -> e.depower)
    if length(usable) < 2
        throw(ArgumentError(
            "Not enough usable turn-rate entries to interpolate for " *
            "body_damping = $bd (need >= 2, have $(length(usable))). " *
            "Run V3Kite.jl/examples/build_turn_rate_table.jl for more depower values."))
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
    dt = get(table.conditions, :dt, nothing)
    isnothing(dt) || (delay = ceil(delay / Float64(dt)) * Float64(dt))
    return (c1 = c1, c2 = c2, delay = delay, interpolated = true)
end

"""
    V3_TURN_RATE_COEFFS

Snapshot of every row of `data/turn_rate_coeffs.yaml`, as parsed at package
load (or by the last [`reload_turn_rate_table!`](@ref)), keyed by
`(body_damping, depower)`. Includes legacy and non-passing rows; prefer
[`turn_rate_coeffs`](@ref), which applies the quality and legacy filtering and
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
