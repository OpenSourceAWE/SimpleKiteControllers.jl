# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The file remembering the learnt elevation bias per inflow condition, shared by
every reel-out run on this machine. Delete it to start every condition from zero
again.
"""
const EL_BIAS_CACHE = joinpath(dirname(@__DIR__), "output", "el_bias_cache.yaml")

"""
    el_bias_key(project, wind_speed) -> String

Identity of an inflow condition: the system project (its file name without the
`.yaml`, so a path and a bare name agree) and the wind speed in m/s, rounded to
a hundredth. The bias is a property of the plant under that inflow — the kite
sags below whatever path it is given, by an amount that depends on the wind and
the kite — so two runs of the same project at the same wind speed share it, and
runs at different speeds do not.
"""
function el_bias_key(project::AbstractString, wind_speed::Real)
    name = replace(basename(String(project)), r"\.ya?ml$" => "")
    return @sprintf("%s@%.2f", name, wind_speed)
end

"""
    el_bias_entries(; file = EL_BIAS_CACHE) -> Dict{String, Any}

Every stored bias, keyed by [`el_bias_key`](@ref). An absent or unreadable file
is an empty cache: this must never be the reason a run stops.
"""
function el_bias_entries(; file::AbstractString = EL_BIAS_CACHE)
    isfile(file) || return Dict{String, Any}()
    try
        d = YAML.load_file(file)
        d isa Dict && haskey(d, "entries") && d["entries"] isa Dict ?
            d["entries"] : Dict{String, Any}()
    catch exc
        @warn "Ignoring an unreadable elevation-bias cache at $file." exception = exc
        Dict{String, Any}()
    end
end

"""
    el_bias_seed(project, wind_speed, bins; file = EL_BIAS_CACHE, fallback = true) -> Union{Nothing, Vector{Float64}}
    el_bias_seed_info(project, wind_speed, bins; file = EL_BIAS_CACHE, fallback = true) -> Union{Nothing, NamedTuple}

The stored elevation correction [deg] for this inflow condition as a profile of
`bins` azimuth bands, or `nothing` when none has been recorded. A profile stored
with a different number of bands is not resampled: its mean is returned as a
rigid shift, which is the part of the correction that does not depend on how the
bands are cut, and the learner refines the shape from there.

With `fallback` (the default) a condition nobody has flown yet is not started
from zero, because the cache says the sag hardly depends on the condition: every
profile recorded so far (Cabauw 4-5.75 m/s, Maasvlakte 6-9 m/s, 2026-09-21) is
0.3-0.5 deg at the crossing rising to 1.2-1.7 deg in the lobe, so any neighbour
is a far better first lap than zeros. The fallback is, in order:

1. the same project at other wind speeds: linearly interpolated between the two
   recorded speeds that bracket `wind_speed`, or the nearest one beyond the ends;
2. every project: the mean profile, whatever the wind speed.

Each candidate is resampled to `bins` by the rule above before it is combined.
`el_bias_seed_info` also returns where the seed came from — `source` is
`"exact"`, `"interpolated"`, `"nearest"` or `"mean"`, `keys` the entries used —
so a run can log and store it; `el_bias_seed` is the profile alone.
"""
function el_bias_seed(project::AbstractString, wind_speed::Real, bins::Integer;
                      file::AbstractString = EL_BIAS_CACHE, fallback::Bool = true)
    info = el_bias_seed_info(project, wind_speed, bins; file, fallback)
    return isnothing(info) ? nothing : info.profile
end

function el_bias_seed_info(project::AbstractString, wind_speed::Real, bins::Integer;
                           file::AbstractString = EL_BIAS_CACHE, fallback::Bool = true)
    bins >= 1 || throw(ArgumentError("bins must be positive, got $bins"))
    entries = el_bias_entries(; file)
    key = el_bias_key(project, wind_speed)
    exact = el_bias_profile(entries, key, bins)
    isnothing(exact) || return (profile = exact, source = "exact", keys = [key])
    fallback || return nothing
    name = replace(basename(String(project)), r"\.ya?ml$" => "")
    # Same project, other speeds: (wind speed, profile, key), valid ones only. The
    # exact key was resolved above (and warned about, if malformed): not twice.
    same = Tuple{Float64, Vector{Float64}, String}[]
    other = Tuple{Vector{Float64}, String}[]
    for (k, e) in entries
        k == key && continue
        p = el_bias_profile(entries, k, bins)
        isnothing(p) && continue
        if e isa Dict && get(e, "project", nothing) == name && get(e, "wind_speed", nothing) isa Real
            push!(same, (Float64(e["wind_speed"]), p, k))
        else
            push!(other, (p, k))
        end
    end
    if !isempty(same)
        sort!(same; by = first)
        v = Float64(wind_speed)
        if v <= same[1][1]
            return (profile = same[1][2], source = "nearest", keys = [same[1][3]])
        elseif v >= same[end][1]
            return (profile = same[end][2], source = "nearest", keys = [same[end][3]])
        end
        i = findlast(t -> t[1] <= v, same)
        (v0, p0, k0), (v1, p1, k1) = same[i], same[i + 1]
        w = (v - v0) / (v1 - v0)
        return (profile = (1 - w) .* p0 .+ w .* p1, source = "interpolated", keys = [k0, k1])
    end
    isempty(other) && return nothing
    return (profile = mean(first.(other)), source = "mean", keys = last.(other))
end

"""
    el_bias_profile(entries, key, bins) -> Union{Nothing, Vector{Float64}}

The profile stored under `key`, resampled to `bins` bands by the rule of
[`el_bias_seed`](@ref), or `nothing` when the entry is absent or malformed.
"""
function el_bias_profile(entries::Dict, key::AbstractString, bins::Integer)
    entry = get(entries, key, nothing)
    isnothing(entry) && return nothing
    profile = try
        Float64.(entry["profile_deg"])
    catch exc
        @warn "Ignoring a malformed elevation-bias entry for $key." exception = exc
        return nothing
    end
    isempty(profile) && return nothing
    length(profile) == bins && return profile
    return fill(mean(profile), bins)
end

"""
    record_el_bias_seed!(project, wind_speed, profile; laps, file = EL_BIAS_CACHE, kwargs...)

Store `profile` [deg], the correction as it stood after `laps` completed laps of
learning, as the seed for the next run at this inflow condition. Under
`with_file_lock`, so parallel sweep workers cannot lose each other's entries.
The entry counts the runs that wrote it and keeps the previous profile next to
the new one, so a reader can see whether the seed is still moving between runs.
Any `kwargs` (a log name, a turbulence level) are stored as plain text next to
the profile, because a cache nobody can read is a cache nobody trusts.
"""
function record_el_bias_seed!(project::AbstractString, wind_speed::Real,
                              profile::AbstractVector{<:Real}; laps::Integer,
                              file::AbstractString = EL_BIAS_CACHE, kwargs...)
    isempty(profile) && throw(ArgumentError("the profile must have at least one bin"))
    key = el_bias_key(project, wind_speed)
    mkpath(dirname(file))
    with_file_lock(file * ".lock") do
        entries = el_bias_entries(; file)
        previous = get(entries, key, nothing)
        entry = Dict{String, Any}(
            "project" => replace(basename(String(project)), r"\.ya?ml$" => ""),
            "wind_speed" => round(Float64(wind_speed); digits = 2),
            "profile_deg" => round.(Float64.(profile); digits = 3),
            "bins" => length(profile),
            "laps" => Int(laps),
            "runs" => 1 + (previous isa Dict ? Int(get(previous, "runs", 0)) : 0),
            "previous_profile_deg" => previous isa Dict ?
                get(previous, "profile_deg", "none") : "none",
            "updated" => Libc.strftime("%Y-%m-%d %H:%M:%S", time()))
        for (k, v) in kwargs
            entry[String(k)] = v
        end
        entries[key] = entry
        YAML.write_file(file, Dict("entries" => entries))
    end
    return nothing
end
