# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Measure the command → course response of the course loop by multisine injection at the three
tether lengths of the FRF validation, 300, 200 and 150 m (points `D`, `A` and `F` of
`validate_margins.jl`, 7 m/s, no turbulence), write it to `data/course_link_measured.csv`,
which `plot_frf_validation.jl` reads, and print the delay and gain margins of the loop on the
measured plant (`measured_loop`, `frd_margins`) next to the model's (`model_loops`, the
pattern model with `kite_correction` and the guidance). The model should be the more
conservative of the two at every point. Run it after the re-identification of the
documentation page "Examples - identification", whose model it checks.

Each point is flown twice with `run_v1`, both times with the tape's rate limit `v_steering`
raised to `CL_V_STEERING` in the point's settings file (restored afterwards, also after an
error), so the tape stays linear:

1. a baseline, whose steady lap period sets the injection lines halfway between the lap's
   harmonics in `CL_BAND` (`lap_period`, `mid_lines`), where the pattern itself has little
   content;
2. the same run with a `Multisine` of amplitude `CL_AMPLITUDE` per line added to the
   steering command. `frf_injection` gives the command → heading and heading → course
   responses at each line; their product is written, for the lines whose course spread over
   the injection periods is below `CL_MAX_COURSE_SD`.

The patterns are the flown ones. Before the wing drag, the 150 m point needed a larger
pattern (f8_a 42°, f8_b 17°) to keep the command off its clamp; the share of the window on
the clamp is printed per point, so check it after a change of the kite.

The logs are kept in `output/course_link/` with the lines in `lines_<point>.yaml`, so
`fly = false` evaluates them again without flying. Six runs of `CL_SIM_TIME` simulated
seconds, about 15 minutes in all. The inputs are passed with `run_example`
(`src/script_inputs.jl`):

    include("examples/measure_course_link.jl")                   # fly, write, compare
    run_example("measure_course_link.jl"; fly = false)           # the saved runs again
    run_example("measure_course_link.jl"; save = false)          # print only
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

# `run_v1`, `analyze`, `Multisine`, `frf_injection`, `lap_period`, `mid_lines`, `measured_loop`,
# `model_loops`, `tf_margins`, `V1_POINTS`.
include(joinpath(@__DIR__, "validate_margins.jl"))
# `update_yaml_values!`.
include(joinpath(@__DIR__, "identification_utils.jl"))
using SimpleKiteControllers: run_example, script_inputs, project_file, skc_data_path, frd_margins
using YAML
using Printf
import Dates

# `CL_`-prefixed: the run scripts this one includes assign plain globals such as `SIM_TIME` and
# `PROJECT`, and a `const` of the same name in `Main` makes their next `include` fail.
"The points measured: key of `V1_POINTS`, its name in the CSV (as `plot_frf_validation.jl` reads it), tether length [m]"
const CL_POINTS = [(:D, "D", 300), (:A, "A", 200), (:F, "F150", 150)]
"Tape rate limit [1/s] for the runs: high enough that the tape never limits"
const CL_V_STEERING = 1.0
"Simulated time [s] of each run: the entry, `HOOK_SETTLE_V1`, one period dropped and at least four kept at 300 m"
const CL_SIM_TIME = 330.0
"Amplitude of each line of the multisine [-]"
const CL_AMPLITUDE = 0.004
"Frequency band [Hz] of the injection lines"
const CL_BAND = (0.2, 2.2)
"Largest relative spread of the course response over the injection periods for a line to be kept"
const CL_MAX_COURSE_SD = 0.35
"Where the injection runs' logs and their lines are kept"
const CL_DIR = normpath(joinpath(@__DIR__, "..", "output", "course_link"))

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
(; fly, save) = script_inputs(@__FILE__, (; fly = true, save = true))

"""
    with_fast_tape_cl(f, point)

Run `f()` with `v_steering` of `point`'s settings file set to `CL_V_STEERING`, and the menu's
selections (project, wind speed, simulation time, turbulence), which `run_v1` changes,
restored afterwards, also after an error. As `with_fast_tape` of `identify_kite_correction.jl`,
which cannot be included here without flying its own runs.
"""
function with_fast_tape_cl(f, point)
    project = project_file(V1_POINTS[point].project)
    settings_file = joinpath(dirname(project), YAML.load_file(project)["system"]["sim_settings"])
    v_steering0 = @sprintf("%g", Settings(project).v_steering)
    project0, wind0, sim_time0 = selected_project(), selected_windspeed(), selected_sim_time()
    turbulence0 = selected_turbulence()
    update_yaml_values!(settings_file, ["v_steering" => @sprintf("%g", CL_V_STEERING)])
    try
        return f()
    finally
        update_yaml_values!(settings_file, ["v_steering" => v_steering0])
        set_selected_project(project0)
        set_selected_windspeed(wind0)
        set_selected_sim_time(sim_time0)
        V3Kite.set_default_turbulence(turbulence0; data_path = skc_data_path())
    end
end

"Share [%] of the injection window of run `r` in which the steering command is on its clamp, `max_steering`"
function clamp_share(r)
    max_steering = FC_Settings(fc_settings(project_file(V1_POINTS[r.point].project))).course.max_steering
    sl = load_log(basename(r.log_path); path = dirname(r.log_path)).syslog
    t, u, phase = Float64.(sl.time), abs.(Float64.(sl.set_steering)), Int.(sl.sys_state)
    window = (phase .== 4) .& (t .> r.t_phase4 + r.hook_settle)
    return 100 * count(u[window] .>= max_steering - 1e-6) / count(window)
end

mkpath(CL_DIR)
if fly
    for (point, key, _) in CL_POINTS
        with_fast_tape_cl(point) do
            baseline = run_v1(point; label = "cl_baseline", sim_time = CL_SIM_TIME)
            period, freqs = mid_lines(lap_period(baseline), CL_BAND...)
            YAML.write_file(joinpath(CL_DIR, "lines_$key.yaml"),
                            Dict("period" => period, "freqs" => freqs, "amp" => CL_AMPLITUDE))
            injected = run_v1(point; label = "cl_injection", injection = Multisine(; period, freqs, amp = CL_AMPLITUDE),
                              sim_time = CL_SIM_TIME)
            cp(injected.log_path * ".arrow", joinpath(CL_DIR, "injection_$key.arrow"); force = true)
        end
    end
end

results = map(CL_POINTS) do (point, key, tether)
    lines = YAML.load_file(joinpath(CL_DIR, "lines_$key.yaml"))
    multisine = Multisine(; period = lines["period"], freqs = Float64.(lines["freqs"]), amp = lines["amp"])
    run = analyze(point, joinpath(CL_DIR, "injection_$key"))
    frf = frf_injection(run, multisine)
    kept = filter(line -> line.course_sd < CL_MAX_COURSE_SD, frf)
    measured = frd_margins(measured_loop([(; r = run, frf = kept)])...)
    model = tf_margins(model_loops(run).corrected)
    (; point, key, tether, run, frf, kept, measured, model, clamp = clamp_share(run))
end

println("\n tether   v_a    lines   on clamp   delay margin: measured  model         gain margin: measured  model")
for x in results
    @printf("  %3d m  %4.1f   %2d/%2d    %4.1f %%             %.3f s  %.3f s (%+3.0f %%)       %5.2f  %5.2f (%+3.0f %%)\n",
            x.tether, x.run.v_a_mean, length(x.kept), length(x.frf), x.clamp, x.measured.dm, x.model.dm,
            100 * (x.model.dm / x.measured.dm - 1), x.measured.gm, x.model.gm, 100 * (x.model.gm / x.measured.gm - 1))
end
all(x -> x.model.dm <= x.measured.dm && x.model.gm <= x.measured.gm, results) ||
    @warn "measure_course_link: the model is not conservative at every point, see the table above."

if save
    file = joinpath(skc_data_path(), "course_link_measured.csv")
    open(file, "w") do io
        println(io, "# Command -> course, rel_steering -> course [rad per unit steering], measured by injected")
        println(io, "# multisines in simple_fig8.jl (V2 of oldplans/Plan_model_validation.md), steering_gain 10,")
        @printf(io, "# v_steering %g 1/s (no rate limit), 7 m/s wind, flown fc_settings and patterns. Lines halfway\n", CL_V_STEERING)
        @printf(io, "# between the lap's harmonics (`mid_lines` of a baseline's own lap), %g - %g Hz, amplitude %g per\n",
                CL_BAND..., CL_AMPLITUDE)
        @printf(io, "# line, one %.0f s run per point; course_sd = relative spread of course/heading over the injection\n",
                CL_SIM_TIME)
        @printf(io, "# periods; lines with course_sd < %g only. The command was on its clamp in %s %% of the window\n",
                CL_MAX_COURSE_SD, join([@sprintf("%.1f", x.clamp) for x in results], " / "))
        @printf(io, "# at %s m. measure_course_link.jl, %s. Use with frd_margins (examples/course_loop_model.jl).\n",
                join([x.tether for x in results], " / "), Dates.today())
        println(io, "point,tether_m,f_hz,re,im,amp,course_sd,v_a")
        for x in results, line in x.kept
            G = line.heading * line.course
            @printf(io, "%s,%d,%.5f,%.5f,%.5f,%.3f,%.3f,%.2f\n", x.key, x.tether, line.f, real(G), imag(G),
                    CL_AMPLITUDE, line.course_sd, x.run.v_a_mean)
        end
    end
    @info "measure_course_link: wrote the command → course response to data/$(basename(file))."
end
nothing
