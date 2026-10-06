# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Measure the kite correction, the ratio of the kite's steering → heading response, as an
injected multisine measures it, to the turn-rate law with the pattern law's dead time and
lag, and

1. write it as a table, gain and phase per frequency, to the project's kite-correction file
   ([`kite_correction_file`](@ref), `data/kite_correction_measured.csv`), which
   `stability_opt_reelout.jl` evaluates the guided loop's margins with
   ([`load_course_correction`](@ref), [`course_correction`](@ref));
2. check that the lag-lead [`kite_correction`](@ref), `(1 + s/ω_z)/(1 + s/ω_p)`, the causal
   stand-in for the table in the transfer-function models, stays conservative against it at
   the measured `v_a` (zero and pole scale with `v_a`, as the table does, so it then is at
   every `v_a`, and `kite_corr_v_ref` is set to the measured `v_a`):
   its gain not lower than measured below `F_SPLIT`, around the gain crossover, and its
   phase not less lagging than measured from `F_SPLIT` on, around the phase crossover. If
   the current `kite_corr_zero` and `kite_corr_pole` of the course-loop model file do, they
   are kept; otherwise the lag-lead that fits the table best (RMS of the complex log error
   over `BAND`) among the conservative ones is written.

A lag-lead is not fitted freely: the measured ratio loses gain with frequency without the
phase lag a causal correction has to have with it (Bode's gain-phase relation), so a free
fit trades the gain at the crossover for the phase and ends below the band. Step 5 of
"Re-identifying after a change of the kite" (documentation page "Examples -
identification"); run it after `identify_pattern_law.jl` and `identify_depower_factor.jl`,
whose dead time and lag the correction goes with.

Point D of `validate_margins.jl` (`system_fig8_300m.yaml`, 7 m/s, no turbulence) is flown
twice with `run_v1`, both times with the tape's rate limit `v_steering` raised to
`V_STEERING` in the project's settings file (restored afterwards, also after an error), so
the tape stays linear:

1. a baseline, whose steady lap period sets the injection lines halfway between the lap's
   harmonics (`lap_period`, `mid_lines`), where the pattern itself has little content;
2. the same run with a `Multisine` of amplitude `AMPLITUDE` per line added to the steering
   command. `frf_injection` gives the tape → heading response at each line.

The measured response is divided by the model's, `c1·v_a·e^(−sτ)/((1 + sT)·s)`, with `c1`
of the turn-rate table and the dead time `τ` and lag `T` of the pattern law
(`pattern_dead_time_lag`) at the run's `v_a` and depower; the gravity pole is left out,
it is far below the lines.

The logs are kept in `output/kite_correction/` with the lines in `lines.yaml`, so
`fly = false` evaluates them again without flying. Two runs of `KC_SIM_TIME` simulated
seconds, about 2 minutes each. The inputs are passed with `run_example`
(`src/script_inputs.jl`):

    include("examples/identify_kite_correction.jl")                  # fly, write table, check
    run_example("identify_kite_correction.jl"; fly = false)          # the saved run again
    run_example("identify_kite_correction.jl"; save = false)         # print only
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

# `run_v1`, `analyze`, `Multisine`, `frf_injection`, `lap_period`, `mid_lines`, `V1_POINTS`.
include(joinpath(@__DIR__, "validate_margins.jl"))
# `wrap_comment` and `update_yaml_values!`.
include(joinpath(@__DIR__, "identification_utils.jl"))
using SimpleKiteControllers: run_example, script_inputs, project_file, skc_data_path
using YAML
using Printf
using Statistics: mean
import Dates

# `KC_`-prefixed: the run scripts this one includes assign plain globals such as `SIM_TIME` and
# `PROJECT`, and a `const` of the same name in `Main` makes their next `include` fail.
"The operating point, a key of `V1_POINTS`"
const KC_POINT = :D
"Tape rate limit [1/s] for the runs: high enough that the tape never limits"
const V_STEERING = 1.0
"Simulated time [s] of each run: the entry, `HOOK_SETTLE_V1`, one period dropped and at least four kept"
const KC_SIM_TIME = 330.0
"Amplitude of each line of the multisine [-]"
const AMPLITUDE = 0.004
"Frequency band [Hz] of the injection lines and of the fit"
const BAND = (0.5, 2.1)
"Frequency [Hz] between the gain check (below) and the phase check (from here on), at the measured `v_a`"
const F_SPLIT = 0.8
"Upper end [Hz] of the phase check, around the phase crossover"
const F_PHASE_MAX = 1.4
"Where the injection run's log and its lines are kept"
const KC_DIR = normpath(joinpath(@__DIR__, "..", "output", "kite_correction"))

# The caller's inputs (`run_example`); a plain `include` flies with these defaults.
(; fly, save) = script_inputs(@__FILE__, (; fly = true, save = true))

"""
    with_fast_tape(f, point)

Run `f()` with `v_steering` of `point`'s settings file set to `V_STEERING`, and the menu's
selections (project, wind speed, simulation time, turbulence), which `run_v1` changes,
restored afterwards, also after an error.
"""
function with_fast_tape(f, point)
    project = project_file(V1_POINTS[point].project)
    settings_file = joinpath(dirname(project), YAML.load_file(project)["system"]["sim_settings"])
    v_steering0 = @sprintf("%g", Settings(project).v_steering)
    project0, wind0, sim_time0 = selected_project(), selected_windspeed(), selected_sim_time()
    turbulence0 = selected_turbulence()
    update_yaml_values!(settings_file, ["v_steering" => @sprintf("%g", V_STEERING)])
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

"Gain and phase of the lag-lead with `zero` and `pole` [Hz] at `freq` [Hz]"
lag_lead(freq, zero, pole) = (1 + im * freq / zero) / (1 + im * freq / pole)

"""
    conservative(freqs, ratio, zero, pole) -> Bool

Whether the lag-lead with `zero` and `pole` [Hz] is conservative against the measured
`ratio` at `freqs` [Hz]: its gain not lower than measured below `F_SPLIT`, and its phase not
less lagging than measured from `F_SPLIT` to `F_PHASE_MAX`.
"""
function conservative(freqs, ratio, zero, pole)
    for (freq, measured) in zip(freqs, ratio)
        model = lag_lead(freq, zero, pole)
        freq < F_SPLIT && abs(model) < abs(measured) && return false
        F_SPLIT <= freq <= F_PHASE_MAX && angle(model) > angle(measured) && return false
    end
    return true
end

"RMS of the complex log error of the lag-lead with `zero` and `pole` [Hz] against `ratio` at `freqs`"
log_rms(freqs, ratio, zero, pole) = sqrt(mean(abs2, log.(ratio ./ lag_lead.(freqs, zero, pole))))

"""
    fit_conservative(freqs, ratio) -> Union{NamedTuple, Nothing}

The `(; zero, pole, log_rms)` [Hz] of the lag-lead that fits `ratio` at `freqs` best among
the conservative ones, by a grid search over 0.1 – 4 Hz in steps of 0.01 Hz; `nothing` if
none is.
"""
function fit_conservative(freqs, ratio)
    best = nothing
    for zero in 0.1:0.01:4.0, pole in 0.1:0.01:4.0
        conservative(freqs, ratio, zero, pole) || continue
        err = log_rms(freqs, ratio, zero, pole)
        (isnothing(best) || err < best.log_rms) && (best = (; zero, pole, log_rms = err))
    end
    return best
end

# Step 5: steps 1 - 4 must be the kite's own, and point D must fly that kite.
check_model_provenance(project_file(selected_project()); through = 4,
                       flown = (V1_POINTS[KC_POINT].project,))
mkpath(KC_DIR)
log_file = joinpath(KC_DIR, "kite_correction_injection")
if fly
    with_fast_tape(KC_POINT) do
        baseline = run_v1(KC_POINT; label = "kc_baseline", sim_time = KC_SIM_TIME)
        period, freqs = mid_lines(lap_period(baseline), BAND...)
        multisine = Multisine(; period, freqs, amp = AMPLITUDE)
        YAML.write_file(joinpath(KC_DIR, "lines.yaml"),
                        Dict("period" => period, "freqs" => freqs, "amp" => AMPLITUDE))
        injected = run_v1(KC_POINT; label = "kc_injection", injection = multisine, sim_time = KC_SIM_TIME)
        cp(injected.log_path * ".arrow", log_file * ".arrow"; force = true)
        record_log_kite!(log_file, V1_POINTS[KC_POINT].project)
    end
end
check_log_kite(log_file, V1_POINTS[KC_POINT].project)
lines = YAML.load_file(joinpath(KC_DIR, "lines.yaml"))
multisine = Multisine(; period = lines["period"], freqs = Float64.(lines["freqs"]), amp = lines["amp"])
kc_run = analyze(KC_POINT, log_file)
frf = frf_injection(kc_run, multisine)

project = project_file(V1_POINTS[KC_POINT].project)
fcs_point = FC_Settings(fc_settings(project))
tc = turn_rate_coeffs(fcs_point.run.body_damping, kc_run.depower)
τ_pattern, T_pattern = pattern_dead_time_lag(tc, kc_run.v_a_mean, kc_run.depower)
kite_model(freq) = tc.c1 * kc_run.v_a_mean * cis(-2π * freq * τ_pattern) /
                   ((1 + im * 2π * freq * T_pattern) * (im * 2π * freq))
in_band = filter(line -> BAND[1] <= line.f <= BAND[2], frf)
freqs = [line.f for line in in_band]
ratio = [line.heading / line.tape / kite_model(line.f) for line in in_band]
clm = course_loop_model()
# The current zero and pole, moved to the measured v_a (they scale with v_a, see kite_correction).
scale = kc_run.v_a_mean / clm.kite_corr_v_ref
zero_now, pole_now = clm.kite_corr_zero * scale, clm.kite_corr_pole * scale
keep = conservative(freqs, ratio, zero_now, pole_now)
fit = keep ? (; zero = zero_now, pole = pole_now, log_rms = log_rms(freqs, ratio, zero_now, pole_now)) :
             fit_conservative(freqs, ratio)
isnothing(fit) && error("No lag-lead on the grid is conservative against the measured kite correction.")

println("\n f [Hz]   measured / model: gain   phase      lag-lead: gain   phase    spread")
for (line, measured) in zip(in_band, ratio)
    model = lag_lead(line.f, fit.zero, fit.pole)
    @printf("  %.3f                    %5.2f  %6.1f°               %5.2f  %6.1f°   %4.1f %%\n", line.f,
            abs(measured), rad2deg(angle(measured)), abs(model), rad2deg(angle(model)), 100 * line.heading_sd)
end
@printf("\n operating point: v_a %.1f m/s, depower %.3f, pattern-law dead time %.3f s + lag %.3f s\n",
        kc_run.v_a_mean, kc_run.depower, τ_pattern, T_pattern)
if keep
    @printf(" The lag-lead %.2f / %.2f Hz is conservative against the measurement; kept (log-RMS %.3f).\n",
            fit.zero, fit.pole, fit.log_rms)
else
    @printf(" The lag-lead %.2f / %.2f Hz (at %.1f m/s) is not conservative against the measurement.\n",
            zero_now, pole_now, kc_run.v_a_mean)
    @printf(" kite_corr_zero = %.2f Hz, kite_corr_pole = %.2f Hz: the best conservative one (log-RMS %.3f).\n",
            fit.zero, fit.pole, fit.log_rms)
end

if save
    project = project_file(selected_project())
    # The table, in the format of course_correction_measured.csv (`load_course_correction`).
    table_file = joinpath(skc_data_path(), kite_correction_file(project))
    open(table_file, "w") do io
        println(io, "# Kite correction M_k(f) = (tape -> heading, measured) / (turn-rate law c1 v_a e^(-s tau) / ((1 + s T) s)")
        println(io, "# with the pattern law's dead time tau and lag T, course_loop_model.jl), measured with an injected")
        @printf(io, "# multisine at point %s (%s, %.1f m/s, v_a %.1f m/s, depower %.3f), v_steering %g, amplitude %g\n",
                KC_POINT, splitext(V1_POINTS[KC_POINT].project)[1], V1_POINTS[KC_POINT].wind, kc_run.v_a_mean,
                kc_run.depower, V_STEERING, AMPLITUDE)
        @printf(io, "# per line; tau = %.3f s, T = %.3f s there. identify_kite_correction.jl, %s.\n",
                τ_pattern, T_pattern, Dates.today())
        println(io, "# Its features move in frequency with v_a (course_correction scales the table by v_a).")
        println(io, "v_a,f_hz,abs,phase_rad")
        for (freq, measured) in zip(freqs, ratio)
            @printf(io, "%.1f,%.4f,%.5f,%.5f\n", kc_run.v_a_mean, freq, abs(measured), angle(measured))
        end
    end
    @info "identify_kite_correction: wrote the measured kite correction to data/$(basename(table_file))."
    file = joinpath(skc_data_path(), course_loop_model_file(project))
    comment = wrap_comment(
        "Zero and pole of kite_correction, the causal stand-in for the measured kite correction " *
        "($(basename(table_file))) in the transfer-function models: " *
        (keep ? "checked" : "chosen as the best fit among the lag-leads") *
        " conservative against it at point $(KC_POINT) ($(splitext(V1_POINTS[KC_POINT].project)[1]), " *
        @sprintf("v_a %.1f m/s): gain not lower than measured below %.1f Hz, phase not less lagging ", kc_run.v_a_mean, F_SPLIT) *
        @sprintf("from %.1f to %.1f Hz; log-RMS against the table %.3f. ", F_SPLIT, F_PHASE_MAX, fit.log_rms) *
        "identify_kite_correction.jl, $(Dates.today()).")
    update_yaml_values!(file, ["kite_corr_zero" => @sprintf("%.2f", fit.zero),
                               "kite_corr_pole" => @sprintf("%.2f", fit.pole),
                               "kite_corr_v_ref" => @sprintf("%.1f", kc_run.v_a_mean),
                               "kite_correction" => "\"$(kite_id(project))\""];
                        comments = Dict("kite_corr_zero" => comment))
    reload_course_loop_model!(project)
    @info "identify_kite_correction: wrote kite_corr_zero and kite_corr_pole to data/$(basename(file))."
end
nothing
