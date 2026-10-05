# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Identify the conversion of the optimizer's power-tape length `l_dp` into V3Kite's
`rel_depower` ([`awetrim_depower_to_v3kite`](@ref)) as a quadratic in `l_dp`, from the
tension the paths of the scenario runs predict and the tension the plant flies on them,
and write it into `data/depower_conversion.yaml`.

Each run folder of `SCENARIOS`, in SimulationResults, is replayed (`replay_paths`: its own
optimizer results from `<log>_opt_entries.json`, no optimizer) with the conversion in force
shifted by each of `shifts` [`rel_depower`]. A folder replays on any machine only with that
file; runs before 2026-10-03 do not have it and replay only where the solution cache still
holds their replies. For every
path the run installed in phase 4, the predicted tension is the time mean of the reply's
`tension_tether_ground` over the pattern, and the measured one the mean winch force over the
time that path was flown (from `T_SETTLE` after it was installed to the next install or the
end of phase 4). Per path, `ln(measured/predicted)` is fitted linearly over the shift; its
zero is the shift `δ*` at which the plant flies the predicted tension, so the target
`rel_depower` at that path's `l_dp` is the conversion in force plus `δ*`. The quadratic
`(pivot - 0.6)/5 + offset + slope*x + curvature*x^2`, `x = l_dp - pivot`, is fitted to all
paths by least squares weighted with their flown time; `pivot` is kept, and so is `offset`
unless `fit_offset`: it is the 6 m/s power-ratio calibration. `l_dp_min`/`l_dp_max` become the
range of the paths' tape lengths, outside which the conversion continues along its tangent.

The startup path is left out: lap 1 flies it at a reduced force limit. The logs are kept in
`output/depower_conversion/`, so `fly = false` refits them without flying. About 30 s per run,
`length(SCENARIOS) * length(shifts)` runs. `save = true` also writes the per-path table
(`points.csv`) and the fit (`fit.yaml`) to `RECORD`, next to the run folders. The inputs are passed with `run_example`
(`src/script_inputs.jl`):

    include("examples/identify_depower_conversion.jl")                       # fly, fit, print
    run_example("identify_depower_conversion.jl"; fly = false, save = true)  # refit, write
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using SimpleKiteControllers: run_example, script_inputs, read_gui_field, write_gui_field,
    set_selected_project, with_run_log, replay_entries, depower_conversion,
    with_depower_conversion, awetrim_depower_to_v3kite, skc_data_path, DEPOWER_CONVERSION_FILE,
    DEPOWER_CONVERSION
using KiteUtils: load_log
using Printf
import YAML
import Dates

"Project and log name of each site's runs"
const SITE_RUNS = Dict("cabauw" => ("system_reelout_cabauw.yaml", "reelout_cabauw_opt"),
                       "maasvlakte" => ("system_reelout_maasvlakte.yaml", "reelout_150m_opt"))
"The SimulationResults repository: `output/scenarios` links into it"
const RESULTS = dirname(realpath(joinpath(@__DIR__, "..", "output", "scenarios")))
"""
Folder (relative to `RESULTS`) of the identification record: its run folders, table and fit.
2026-10-05: the base runs flown with `wing_drag_coeff = 0.03` and the conversion iterated to
it (two rounds of base runs, see docs/power_ratio_findings.md); 2026-10-03 holds the runs
without wing drag.
"""
const RECORD = "depower_conversion/2026-10-05"
"""
Runs replayed: site, wind speed [m/s], run folder (relative to `RESULTS`). Maasvlakte 3.5 and
4 m/s are in the record but not fitted: below about 1.5 kN the optimizer's soft winch floor
predicts a much slower reel-out than the plant flies, so the predicted tension is not a target
for the depower (docs/power_ratio_findings.md).
"""
const SCENARIOS = [(site, wind, @sprintf("%s/runs/%s_v%04.1f", RECORD, site, wind))
                   for (site, wind) in (("cabauw", 4.0), ("cabauw", 5.0), ("cabauw", 6.0),
                                        ("cabauw", 7.0), ("cabauw", 8.0), ("cabauw", 9.0),
                                        ("cabauw", 10.0), ("maasvlakte", 8.0),
                                        ("maasvlakte", 10.0), ("maasvlakte", 11.0))]
"Where the replay logs are kept"
const LOG_DIR = normpath(joinpath(@__DIR__, "..", "output", "depower_conversion"))
"Time after an install, and before the end of phase 4, left out of the measured mean [s]"
const T_SETTLE = 3.0

(; shifts, fly, save, fit_offset) =
    script_inputs(@__FILE__, (; shifts = [-0.01, 0.0, 0.01], fly = true, save = false, fit_offset = false))

run_dir(site, wind, shift) = joinpath(LOG_DIR, @sprintf("%s_v%04.1f_d%+.3f", site, wind, shift))
shifted(conv, shift) = merge(conv, (; offset = conv.offset + shift))
first_value(x) = Float64(x isa AbstractVector ? x[1] : x)

"Fly `simple_opt_reelout.jl` for `site` at `wind` into `dir`, with `inputs` passed on"
function fly_run(site, wind, dir; inputs...)
    project, _ = SITE_RUNS[site]
    mkpath(dir)
    project0, wind0 = read_gui_field("project"), read_gui_field("wind_speed")
    try
        set_selected_project(project)
        write_gui_field("wind_speed", wind)
        with_run_log(joinpath(dir, "run.log")) do
            run_example("simple_opt_reelout.jl"; show_plots = false, run_archive = false,
                        output_path = dir, inputs...)
        end
    finally
        set_selected_project(project0)
        write_gui_field("wind_speed", something(wind0, "default") == "default" ? "default" :
                                      parse(Float64, wind0))
    end
end

"Time mean of the predicted tension [N] and the tape length [m] of a solution-cache entry"
function predicted(entry)
    table = entry["table"]["table"]
    t, f = Float64.(table["t"]), Float64.(table["tension_tether_ground"])
    f_mean = sum((f[i] + f[i + 1]) / 2 * (t[i + 1] - t[i]) for i in 1:(length(t) - 1)) / (t[end] - t[1])
    return f_mean, sum(Float64.(table["input_depower"])) / length(table["input_depower"])
end

"""
Per phase-4 path of the replay in `dir`: `(; t0, t1, f)`, the window it was flown [s] and the
mean winch force over it [N]
"""
function measured(dir, log_name)
    sl = load_log(log_name; path = dir).syslog
    paths = YAML.load_file(joinpath(dir, log_name * "_opt_paths.yaml"))["paths"]
    t_inst = [Float64(p["installed_t"]) for p in paths]
    phase4 = findall(==(4), sl.sys_state)
    t4_end = sl.time[last(phase4)] - T_SETTLE
    force = first_value.(sl.winch_force)
    return map(2:length(t_inst)) do k
        t0, t1 = t_inst[k] + T_SETTLE, k < length(t_inst) ? t_inst[k + 1] : t4_end
        in_window = [i for i in phase4 if t0 <= sl.time[i] < min(t1, t4_end)]
        (; t0, t1 = min(t1, t4_end), f = isempty(in_window) ? NaN : sum(force[in_window]) / length(in_window))
    end
end

"Replay `folder` of `site` at `wind` with the conversion in force shifted by `shift`"
replay(site, wind, folder, shift) =
    with_depower_conversion(shifted(depower_conversion(), shift)) do
        fly_run(site, wind, run_dir(site, wind, shift); replay_paths = joinpath(RESULTS, folder))
    end

fly && for (site, wind, folder) in SCENARIOS, shift in shifts
    @info @sprintf("Replaying %s %.1f m/s at a depower shift of %+.3f", site, wind, shift)
    replay(site, wind, folder, shift)
end

# One point per phase-4 path: its tape length, the shift that makes it fly the predicted
# tension, and the time it was flown.
conv = depower_conversion()
points = NamedTuple[]
for (site, wind, folder) in SCENARIOS
    _, run_log_name = SITE_RUNS[site]
    entries = replay_entries(joinpath(RESULTS, folder), run_log_name)
    runs = [measured(run_dir(site, wind, shift), run_log_name) for shift in shifts]
    for k in eachindex(runs[1])
        f_pred, l_dp = predicted(entries[k + 1])
        y = [log(run[k].f / f_pred) for run in runs]
        all(isfinite, y) && runs[1][k].t1 - runs[1][k].t0 > 2.0 || continue
        # ln(measured/predicted) = c + s*shift, least squares; its zero is δ*.
        n, sx, sy = length(shifts), sum(shifts), sum(y)
        s = (n * sum(shifts .* y) - sx * sy) / (n * sum(shifts .^ 2) - sx^2)
        c = (sy - s * sx) / n
        δ = -c / s
        push!(points, (; site, wind, k, l_dp, f_pred, f_meas = [run[k].f for run in runs],
                       ratio = exp(c), sens = s, δ,
                       target = awetrim_depower_to_v3kite(l_dp; conv) + δ,
                       weight = runs[1][k].t1 - runs[1][k].t0))
    end
end

# Weighted least squares of target = c0 + slope*x + curvature*x^2 about the kept pivot,
# c0 kept at the calibrated offset unless `fit_offset`.
x = [p.l_dp - conv.pivot for p in points]
W = [p.weight for p in points]
targets = [p.target for p in points]
c0 = (conv.pivot - 0.6) / 5 + conv.offset
coef = fit_offset ?
    (A -> (A' * (W .* A)) \ (A' * (W .* targets)))(hcat(ones(length(x)), x, x .^ 2)) :
    [c0; (A -> (A' * (W .* A)) \ (A' * (W .* (targets .- c0))))(hcat(x, x .^ 2))]
l_dp_range = extrema(p.l_dp for p in points)
fit = (; offset = coef[1] - (conv.pivot - 0.6) / 5, pivot = conv.pivot, slope = coef[2],
       curvature = coef[3], l_dp_min = floor(l_dp_range[1]; digits = 2),
       l_dp_max = ceil(l_dp_range[2]; digits = 2))

println("  site        wind  path  l_dp [m]  F pred [N]  meas/pred  dln/dδ   δ* [-]   target   fit")
for p in points
    println(@sprintf("  %-10s %5.1f  %4d  %8.3f  %10.0f  %9.3f  %6.1f  %+7.4f  %7.4f  %+7.4f",
                     p.site, p.wind, p.k + 1, p.l_dp, p.f_pred, p.ratio, p.sens, p.δ, p.target,
                     awetrim_depower_to_v3kite(p.l_dp; conv = fit) - p.target))
end
rms = sqrt(sum(W .* [(awetrim_depower_to_v3kite(p.l_dp; conv = fit) - p.target)^2 for p in points]) / sum(W))
println(@sprintf("Fit over %d paths of %d scenarios: offset %.4f%s, slope %.4f 1/m, curvature %.4f 1/m², \
                  l_dp %.2f-%.2f m, weighted RMS residual %.4f (in force: offset %.4f, slope %.4f, \
                  curvature %.4f).",
                 length(points), length(SCENARIOS), fit.offset, fit_offset ? "" : " (kept)", fit.slope,
                 fit.curvature, fit.l_dp_min, fit.l_dp_max, rms, conv.offset, conv.slope, conv.curvature))

"Write the values of `fit` into the conversion file, keeping its comments"
function write_fit(fit, file = joinpath(skc_data_path(), DEPOWER_CONVERSION_FILE))
    yaml_text = read(file, String)
    for key in (:offset, :slope, :curvature, :l_dp_min, :l_dp_max)
        yaml_text = replace(yaml_text, Regex("(\\n  $key: )[-0-9.eE+]+") =>
                                       SubstitutionString("\\g<1>" * @sprintf("%.4f", fit[key])))
    end
    write(file, yaml_text)
    DEPOWER_CONVERSION[] = nothing   # re-read on next use
    @info "Wrote the fit to $file ($(Dates.today())); add the identification record by hand."
end

"""
Write the per-path table and the fit to `RECORD` in SimulationResults: what the fit was made
of, without the replay logs (about 70 MB each)
"""
function write_record(points, fit, rms, dir = joinpath(RESULTS, RECORD))
    mkpath(dir)
    open(joinpath(dir, "points.csv"), "w") do io
        println(io, "site,wind,path,l_dp,f_pred,", join([@sprintf("f_meas_d%+.3f", d) for d in shifts], ","),
                ",weight,delta,target,residual")
        for p in points
            println(io, join([p.site; string(p.wind); string(p.k + 1);
                              [@sprintf("%.4f", p.l_dp), @sprintf("%.1f", p.f_pred)];
                              [@sprintf("%.1f", f) for f in p.f_meas];
                              [@sprintf("%.2f", p.weight), @sprintf("%.5f", p.δ), @sprintf("%.5f", p.target),
                               @sprintf("%.5f", awetrim_depower_to_v3kite(p.l_dp; conv = fit) - p.target)]], ","))
        end
    end
    YAML.write_file(joinpath(dir, "fit.yaml"), Dict(
        "date" => string(Dates.today()), "shifts" => shifts, "fit_offset" => fit_offset,
        "paths" => length(points), "runs" => [folder for (_, _, folder) in SCENARIOS],
        "weighted_rms_residual" => round(rms; digits = 5),
        "fit" => Dict(string(k) => round(v; digits = 4) for (k, v) in pairs(fit)),
        "conversion_in_force" => Dict(string(k) => v for (k, v) in pairs(conv))))
    @info "Wrote points.csv and fit.yaml to $dir."
end

save && (write_fit(fit); write_record(points, fit, rms))
nothing
