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
    el_bias_seed(project, wind_speed, bins; file = EL_BIAS_CACHE) -> Union{Nothing, Vector{Float64}}

The stored elevation correction [deg] for this inflow condition as a profile of
`bins` azimuth bands, or `nothing` when none has been recorded. A profile stored
with a different number of bands is not resampled: its mean is returned as a
rigid shift, which is the part of the correction that does not depend on how the
bands are cut, and the learner refines the shape from there.
"""
function el_bias_seed(project::AbstractString, wind_speed::Real, bins::Integer;
                      file::AbstractString = EL_BIAS_CACHE)
    bins >= 1 || throw(ArgumentError("bins must be positive, got $bins"))
    entry = get(el_bias_entries(; file), el_bias_key(project, wind_speed), nothing)
    isnothing(entry) && return nothing
    profile = try
        Float64.(entry["profile_deg"])
    catch exc
        @warn "Ignoring a malformed elevation-bias entry for $(el_bias_key(project, wind_speed))." exception = exc
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
