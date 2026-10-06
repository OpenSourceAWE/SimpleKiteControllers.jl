# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The provenance of the identified model: which kite the turn-rate table and the course-loop
# model were identified on, so that a model half re-identified after a change of the kite is
# refused before it is rated or retuned against (2026-10-05: the guided loop was retuned for
# the wing drag on a turn-rate law with the drag and a course-loop model without it).

"""
The values of a project's kite settings that make up the kite in [`kite_fingerprint`](@ref):
the physics of the wing, its bridle and its rigging. Left out are the solver's options
(`backend`, `analytic_jacobian`, `vsm_interval`), the start-up (`init_mode`, `init_state`,
`remake_*`, `brake`) and the keys no run of this package reads (`body_sim_damping`,
`wing_mass`, `wing_mass_le_frac`).
"""
const KITE_SETTINGS_KEYS = ("wing_type", "aero_mode", "world_sim_damping", "wing_drag_coeff",
                            "adapter_dir", "geom", "bridle")

"""
The identification steps of "Re-identifying after a change of the kite" (documentation page
"Examples - identification"), in order: the key each step's `kite_id` is recorded under and
the script that writes it. Step 1 records it in every row of the turn-rate table, steps 2 – 5
in the `provenance:` section of the course-loop model file.
"""
const IDENTIFICATION_STEPS = (
    (key = "turn_rate_law", script = "build_turn_rate_table.jl"),
    (key = "kite_delay_scaling", script = "identify_kite_delay_scaling.jl"),
    (key = "pattern_law", script = "identify_pattern_law.jl"),
    (key = "depower_factor", script = "identify_depower_factor.jl"),
    (key = "kite_correction", script = "identify_kite_correction.jl"),
)

"""
    kite_fingerprint(project = project_file()) -> OrderedDict{String, Any}

The kite `project` flies, as the values that the identified model depends on: the wing
`mass` and the `kcu_mass` of its `sim_settings`, the `body_damping` of its `fc_settings`
(`run.body_damping`), the geometry files it names (`structural_geometry`,
`aero_geometry`) and the [`KITE_SETTINGS_KEYS`](@ref) of its `kite_settings` file, the
nested ones as `geom.<key>` and `bridle.<key>`. Not in it: the tether, the winch, the
steering tape (its lag is `1/steering_gain`, read live) and the
`damping_per_stiffness` of `init_model`, which is a keyword of the examples, not a setting.

The `kite_settings` file must lie beside the project: V3Kite would fall back to its own
data directory, which this package does not read.
"""
function kite_fingerprint(project = project_file())
    path = project_file(project)
    dir = dirname(path)
    system = YAML.load_file(path)["system"]
    sim = YAML.load_file(joinpath(dir, system["sim_settings"]))
    kite_file = joinpath(dir, system["kite_settings"])
    isfile(kite_file) || error("kite_fingerprint: $(basename(path)) names the kite settings " *
                               "$(system["kite_settings"]), which is not beside it in $dir.")
    kite = YAML.load_file(kite_file)["kite_settings"]
    body_damping = FC_Settings(fc_settings(path)).run.body_damping
    fingerprint = OrderedDict{String, Any}(
        "mass" => sim["kite"]["mass"], "kcu_mass" => sim["kcu"]["kcu_mass"],
        "body_damping" => Float64.(collect(body_damping)),
        "structural_geometry" => system["structural_geometry"],
        "aero_geometry" => system["aero_geometry"])
    for key in KITE_SETTINGS_KEYS
        value = kite[key]
        if value isa AbstractDict
            for sub in sort!(collect(keys(value)))
                fingerprint["$key.$sub"] = value[sub]
            end
        else
            fingerprint[key] = value
        end
    end
    return fingerprint
end

"""
    kite_id(project = project_file()) -> String

A short hash (12 hex digits) of the [`kite_fingerprint`](@ref) of `project`: two projects
fly the same kite when their `kite_id`s agree. The identification scripts record it with
what they identify, and [`check_model_provenance`](@ref) compares it with the kite flown.
"""
function kite_id(project = project_file())
    fingerprint = kite_fingerprint(project)
    text = join(("$key=$(repr(fingerprint[key]))" for key in sort!(collect(keys(fingerprint)))), "\n")
    return bytes2hex(sha256(text))[1:12]
end

"""
    stale_identification_steps(project = project_file(); through = 5) -> Vector{NamedTuple}

The steps `1:through` of [`IDENTIFICATION_STEPS`](@ref) whose results in `project`'s files
were not identified on the kite `project` flies ([`kite_id`](@ref)), one
`(; step, script, reason)` each. Step 1 is stale when a usable row of the turn-rate table
([`turn_rate_coeffs_file`](@ref)) carries another `kite_id` or none, or when it has no
usable row; steps 2 – 5 when the `provenance:` section of the course-loop model file
([`course_loop_model_file`](@ref)) records another `kite_id` for them, or none.
"""
function stale_identification_steps(project = project_file(); through = length(IDENTIFICATION_STEPS))
    path = project_file(project)
    id = kite_id(path)
    stale = NamedTuple[]
    note(step, reason) = push!(stale, (; step, script = IDENTIFICATION_STEPS[step].script, reason))
    if through >= 1
        table_file = turn_rate_coeffs_file(path)
        rows = filter(e -> _is_usable_turn_rate_entry(_parse_turn_rate_entry(e)),
                      YAML.load_file(joinpath(skc_data_path(), table_file))["entries"])
        others = filter(e -> get(e, "kite_id", nothing) != id, rows)
        if isempty(rows)
            note(1, "$table_file has no usable row")
        elseif !isempty(others)
            depowers = join((@sprintf("%g", e["depower"]) for e in others), ", ")
            found = join(unique(string(get(e, "kite_id", "none")) for e in others), ", ")
            note(1, "the rows of $table_file at depower $depowers were identified on kite $found")
        end
    end
    if through >= 2
        model_file = course_loop_model_file(path)
        provenance = something(get(YAML.load_file(joinpath(skc_data_path(), model_file)), "provenance", nothing),
                               Dict{String, Any}())
        for step in 2:through
            found = get(provenance, IDENTIFICATION_STEPS[step].key, nothing)
            found == id || note(step, "$model_file records $(IDENTIFICATION_STEPS[step].key) " *
                                      "on kite $(something(found, "none"))")
        end
    end
    return stale
end

"""
    check_model_provenance(project = project_file(); through = 5, flown = ())

Throw unless steps `1:through` of the identification ([`stale_identification_steps`](@ref))
were all identified on the kite `project` flies, and unless every project of `flown` flies
that kite too. The rating and retuning scripts (`stability_opt_reelout.jl`,
`stability_global.jl`, `retune_guided.jl`) call it with every step before they rate a
margin; the identification script of step `k` calls it with `through = k - 1` and the
projects it flies before it flies them, so the steps run in order and on one kite.

A margin rated on a model whose parts belong to different kites is wrong without looking
wrong: after a change of the kite, finish the identification before rating or retuning.
"""
function check_model_provenance(project = project_file(); through = length(IDENTIFICATION_STEPS), flown = ())
    path = project_file(project)
    id = kite_id(path)
    problems = ["step $(s.step), $(s.script): $(s.reason)"
                for s in stale_identification_steps(path; through)]
    for other in flown
        other_id = kite_id(other)
        other_id == id || push!(problems, "$(basename(project_file(other))) flies kite $other_id")
    end
    isempty(problems) && return nothing
    error("The identified model of $(basename(path)) does not belong to the kite it flies " *
          "(kite_id $id):\n  " * join(problems, "\n  ") * "\nRe-identify from the first stale " *
          "step on, in order (\"Re-identifying after a change of the kite\", documentation " *
          "page \"Examples - identification\"), before rating or retuning the controller.")
end
