# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
V1 of oldplans/Plan_model_validation.md: push the simulated course loop until it
rings, and compare the critical gain factor `k_crit` / extra delay `τ_crit`,
and their ringing frequencies, against `course_loop_model.jl`'s gain margin,
phase crossover, delay margin and gain crossover.

Needs the steer_gain_factor / extra_steer_delay / hook_settle inputs of
`simple_fig8.jl` and `simple_opt_reelout.jl` (added for V1, see either
script's own comments next to them, or oldplans/Plan_model_validation.md#tests).

# Before the first run of a point

Both scripts read `FC_Settings` fresh from the project's YAML file with no
REPL-side override (`simple_fig8.jl`'s own docstring: "There is no REPL-side
override"). V1 needs, in `data/fc_settings.yaml` (points A, B, D) or
`data/fc_settings_reelout.yaml` (point C):

    ff_gain:          0        # delay test; 1 for the feedback-only gain test
    fig8_pure_course: true     # pure course feedback

The flown values are `ff_gain` 1.0 (0.7 for the reel-out file) and
`fig8_pure_course` false: restore them afterwards. `run_v1` warns, but does
not refuse, if the settings do not match the run.

# Usage

Point D (300 m, 7 m/s) is the one with a baseline that is not rate-limited; A
(200 m) could not bracket a linear onset (oldplans/Plan_model_validation.md, V1).

    include("examples/validate_margins.jl")
    predict(:D)                         # the model's numbers, for reference
    base = run_v1(:D)                   # baseline: gain factor 1, no extra delay
    delay_runs = sweep_delay(:D, base)  # grid, then bisects τ_crit (ff_gain 0)
    report(:D, base, Any[], delay_runs)
    # gain test with ff_gain 1, scaling only the feedback part:
    base_ff = run_v1(:D)
    gain_runs = sweep_gain(:D, base_ff; feedback_only = true)

The sweeps call a run unstable once it is in a saturated limit cycle
(`limit_cycle`), which needs the tape's rate limit. Without it (the
`v_steering` 1.0 s⁻¹ runs at point D) judge the runs with `oscillating` instead
and bisect by hand; `command_mode` shows the growing mode and `loop_breakdown`
splits an oscillating run's loop into tape, kite and course/error links at its
frequency.

`base`'s window checks the mean `v_a` against the point's nominal value
(`oldplans/Plan_model_validation.md#operating-points`); adjust the wind with
`select_windspeed()` and re-run `run_v1` if it is off by more than 10 %.

Each `run_v1` call archives the run's log under
`output/archives/v1_<point>_<label>_<timestamp>/`, so the next run in a sweep
does not overwrite it (every point flies the same project, hence the same log
file name). A 120 s run at point D takes about 45 s; a sweep is 6 – 10 runs.

Point C's prediction is the inner course loop only; the reel-out project also
flies the attractor guidance, which lowers the margins (see `predict`'s
docstring).
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file
using ControlSystemsBase, RobustAndOptimalControl
using LinearAlgebra: diagm
using Statistics: mean, std
using Dates
using Printf

set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "course_loop_model.jl"))

# ==================== OPERATING POINTS ==================== #
# oldplans/Plan_model_validation.md#operating-points

"One operating point of V1: which project/script flies it, at what wind."
struct V1Point
    project::String
    script::String    # examples/*.jl, next to this file
    wind::Float64      # m/s, select_windspeed()
    sim_time::Float64  # s, select_sim_time()
    v_a_nominal::Float64  # m/s, expected mean v_a in the analysis window
end

const V1_POINTS = Dict(
    :A => V1Point("system_fig8_200m.yaml", "simple_fig8.jl", 7.0, 90.0, 35.0),
    :B => V1Point("system_fig8_200m.yaml", "simple_fig8.jl", 4.5, 120.0, 23.0),
    :C => V1Point("system_reelout_maasvlakte.yaml", "simple_opt_reelout.jl", 4.0, 120.0, 15.0),
    # Point A at 300 m: the same pattern in degrees, so the steering it needs is
    # 2/3 of A's and the tape rate about (2/3)². With steering_gain 10 its
    # baseline is rate-limited 7.7 % of the window, A's 28 % (2026-09-27).
    :D => V1Point("system_fig8_300m.yaml", "simple_fig8.jl", 7.0, 120.0, 34.2),
    # The fig8 pattern at 150 m, same wind: with A (200 m) and D (300 m) the
    # tether-length check of the model (oldplans/Plan_model_validation.md).
    :F => V1Point("system_fig8_150m.yaml", "simple_fig8.jl", 7.0, 120.0, 35.0),
)

"""
    DelayedInjection(m, settle)

`t -> Δu` for `simple_opt_reelout.jl`'s input `steer_disturbance`, which is called
with the absolute time: zero until `settle` [s] after phase 4 is first reached
(`st.t_phase4` of the script's `RunState`, read live during the loop), then `m(τ)`,
τ the time since then. The same start as `steer_injection` in `simple_fig8.jl`, so
`frf_injection` works on either script's runs.
"""
struct DelayedInjection{F}
    m::F
    settle::Float64
end
function (d::DelayedInjection)(t)
    tp = isdefined(Main, :st) && hasproperty(Main.st, :t_phase4) ? Main.st.t_phase4 : NaN
    return (isnan(tp) || t < tp + d.settle) ? 0.0 : d.m(t - tp - d.settle)
end

"Matches the scripts' own default; passed explicitly so a sweep cannot drift from it."
const HOOK_SETTLE_V1 = 15.0

"""
    v1_lag(point) -> Float64

The tape's small-signal lag [s], `rel_steering` -> `steering`, of `point`'s
project: `1/steering_gain` of its settings YAML (`kcu.steering_gain`, 10 in every
V1 project since 2026-09-27, i.e. 0.1 s; the KiteUtils default 3 gives 0.33 s).
It holds while the tape stays off its 0.2 s⁻¹ rate limit, which is what V1
needs from a baseline anyway. `ACTUATOR_LAG` (0.43 s) was the equivalent lag
of the gain-3 tape when rate-limited much of the time; it is not used here.
"""
v1_lag(point) = 1 / Settings(project_file(V1_POINTS[point].project)).steering_gain

# ==================== MODEL PREDICTIONS ==================== #

"""
    predict(point; v_a = V1_POINTS[point].v_a_nominal, depower = nothing,
            lag = v1_lag(point)) -> NamedTuple

The model's gain margin, phase crossover, delay margin and gain crossover at
`point`'s `v_a` [m/s] (oldplans/Plan_model_validation.md#predictions), from
`course_loop_model.jl` at the project's own `FC_Settings` and sample rate —
the worst of the two gravity-pole signs, as `stability_fig8.jl`'s
`loop_margins` does. `depower` defaults to the project's `depower_setpoint`,
`lag` [s] is the actuator lag, by default the project's small-signal tape lag
(`v1_lag`).

Point `:C` is the INNER course loop only (`heading_d`, no pattern floor): the
reel-out project also flies the attractor guidance on top, which lowers the
margins further (`stability_opt_reelout.jl`). Compare a point-C sweep against
the guided margins on its own baseline log, not against this prediction alone.
"""
function predict(point::Symbol; v_a = V1_POINTS[point].v_a_nominal, depower = nothing,
                  lag = v1_lag(point))
    p = V1_POINTS[point]
    project = project_file(p.project)
    fcs_p = FC_Settings(fc_settings(project))
    reload_turn_rate_table!(project)
    Ts = 1 / Settings(project).sample_freq
    dp = something(depower, fcs_p.depower_setpoint)
    tc = turn_rate_coeffs(fcs_p.body_damping, dp)
    v_min = point == :C ? fcs_p.v_app_min : max(fcs_p.v_app_min, fcs_p.v_app_min_pattern)
    K = fcs_p.heading_p * fcs_p.v_app_ref / max(v_a, v_min)
    C = course_pid(K, fcs_p.heading_i, fcs_p.heading_d, fcs_p.heading_d_n, Ts)
    cos_beta = cosd(fcs_p.el_center)
    τ, T_kite = kite_dead_time(tc, v_a), kite_lag(tc, v_a)
    # The plant's c1 and gravity term c2/v_a·sin(ψ)·cos(β) of the low pattern (course_loop_model.jl).
    pc = plant_coeffs(dp)
    c2 = pc.c2
    branches = vec(map((-cos_beta, cos_beta)) do g
        L = C * turn_rate_plant(pc.c1, c2, τ, v_a, g, Ts; lag, kite_lag = T_kite)
        (; g, c2, L, α = diskmargin(L).margin, open_loop_stable = isstable(L))
    end)
    alpha = minimum(b -> b.α, branches)   # disk margin is well-defined either way

    # The classical gain margin / crossover frequencies below are only
    # meaningful for an OPEN-LOOP stable branch: at el_center's other gravity
    # sign the plant itself has an unstable pole (loaded, not flown, that way),
    # and `margin`'s gain-margin/crossover numbers degenerate (a spurious
    # near-0 Hz "crossing"), even though `feedback(L)` and the disk margin are
    # both fine. `delay_margin` already guards the same case via
    # `isstable(feedback(L))`, which both branches pass — this is a stricter,
    # additional condition on the OPEN loop.
    stable_branches = filter(b -> b.open_loop_stable, branches)
    # delay_margin is guarded internally (isstable(feedback(L))) and needs no
    # open-loop filter, so it uses the overall worst branch, as
    # `stability_fig8.jl`'s `loop_margins` does.
    dm_s = delay_margin(argmin(b -> b.α, branches).L)
    if isempty(stable_branches)
        @warn "predict($point): no open-loop-stable branch — gain margin/crossovers are NaN; \
               only alpha and the delay margin are meaningful here."
        return (; point, v_a, depower = dp, lag, K, Ts, alpha,
                 gain_margin = NaN, phase_crossover_hz = NaN,
                 delay_margin_s = dm_s, delay_margin_samples = round(Int, dm_s / Ts),
                 gain_crossover_hz = NaN)
    end
    # Worst (smallest) disk margin among the open-loop-stable branches, so the
    # classical numbers come from the same branch a reader would check by hand.
    L = argmin(b -> b.α, stable_branches).L
    wgm, gm, wpm, pm = margin(L; allMargins = true)
    return (; point, v_a, depower = dp, lag, K, Ts, alpha,
             gain_margin = gm[1][1], phase_crossover_hz = wgm[1][1] / 2π,
             delay_margin_s = dm_s, delay_margin_samples = round(Int, dm_s / Ts),
             gain_crossover_hz = wpm[1][1] / 2π)
end

# ==================== RUNNING ONE POINT ==================== #

"""
    run_v1(point; gain_factor = 1.0, extra_delay = 0, feedback_only = false,
           label = "run", hook_settle = HOOK_SETTLE_V1, baseline = nothing,
           test::Symbol = :gain, injection = nothing, sim_time = nothing) -> NamedTuple

Select `point`'s project/wind/sim_time, no turbulence, and `run_example` its script
with `show_plots = false` and the V1 hooks; archive the log and `analyze` it.
`injection` (a function `τ -> Δu`, e.g. a `Multisine`) is added to the command
as the input `steer_injection` (V2); `sim_time` [s] overrides the point's.
Pass `baseline` (another `run_v1` result, with `gain_factor = 1`,
`extra_delay = 0`) to also get the stable/unstable/rate-limited verdict — see
`analyze`. `feedback_only = true` scales only the feedback part of the
command, `rel_steering - u_ff`, and leaves the feed-forward alone (the
`steer_gain_feedback_only` hook); it only differs from the default with
`ff_gain > 0`.
"""
function run_v1(point::Symbol; gain_factor = 1.0, extra_delay = 0, feedback_only = false,
                 label = "run", hook_settle = HOOK_SETTLE_V1, baseline = nothing,
                 test::Symbol = :gain, injection = nothing, sim_time = nothing)
    p = V1_POINTS[point]
    sim_time = something(sim_time, p.sim_time)
    set_selected_project(p.project)
    set_selected_sim_time(sim_time)
    set_selected_windspeed(p.wind)
    V3Kite.set_default_turbulence(0.0; data_path = skc_data_path())

    # The reel-out script takes a test input as `steer_disturbance`, called with the absolute time.
    test_input = p.script == "simple_opt_reelout.jl" ?
        (; steer_disturbance = isnothing(injection) ? nothing :
                                   DelayedInjection(injection, hook_settle)) :
        (; steer_injection = injection)
    @info @sprintf("V1 %s (%s): gain factor %.4g%s, extra delay %d samples%s, \
                    wind %.1f m/s, sim_time %.0f s.",
                   point, label, gain_factor, feedback_only ? " (feedback only)" : "",
                   extra_delay, isnothing(injection) ? "" : ", injection",
                   p.wind, sim_time)
    run_example(p.script; show_plots = false, steer_gain_factor = gain_factor,
                steer_gain_feedback_only = feedback_only, extra_steer_delay = extra_delay,
                hook_settle, test_input...)

    project = project_file(p.project)
    project_set = Settings(project)
    apply_windspeed_override!(project_set, p.wind)
    is_reelout = p.script == "simple_opt_reelout.jl"
    log_name = basename(project_set.log_file) * (is_reelout ? "_opt" : "")

    fcs_p = FC_Settings(fc_settings(project))
    # The feed-forward is fine for a feedback-only gain run (it lies outside the
    # loop), and for a baseline or delay run that belongs to one.
    ff_ok = fcs_p.ff_gain == 0 || feedback_only || (gain_factor == 1.0)
    (ff_ok && fcs_p.fig8_pure_course) ||
        @warn @sprintf("V1 %s: ff_gain = %.2g, fig8_pure_course = %s — this run does \
                        not match V1's settings, see oldplans/Plan_model_validation.md#settings.",
                       point, fcs_p.ff_gain, fcs_p.fig8_pure_course)

    src_output = normpath(joinpath(@__DIR__, "..", "output"))
    stamp = Dates.format(Dates.now(), "yyyymmdd_HHMMSS")
    archive_dir = joinpath(src_output, "archives", "v1_$(point)_$(label)_$stamp")
    mkpath(archive_dir)
    src_arrow = joinpath(src_output, log_name * ".arrow")
    isfile(src_arrow) ||
        error("run_v1: no $log_name.arrow in $src_output — the run above did not reach save_log.")
    cp(src_arrow, joinpath(archive_dir, log_name * ".arrow"); force = true)

    return analyze(point, joinpath(archive_dir, log_name); hook_settle, gain_factor,
                    extra_delay, label, baseline, test)
end

# ==================== ANALYSIS ==================== #

"""
    bandpass(x, dt; f_lo, f_hi) -> Vector{Float64}

Crude band-pass, in the style of `SimpleKiteControllers.fig8_metrics`'s
`hf_std`: highpass by subtracting a centred moving average over `1/f_lo`
seconds, then lowpass by moving-averaging the result over `1/f_hi` seconds.
Adequate for V1's onset/ringing check, not a substitute for V2's proper FRF.
"""
function bandpass(x::AbstractVector{<:Real}, dt::Real; f_lo, f_hi)
    function movavg(v, n)
        half = n ÷ 2
        return [mean(@view v[max(1, i - half):min(length(v), i + half)]) for i in eachindex(v)]
    end
    n_lo = max(1, round(Int, 1 / (f_lo * dt)))
    hp = x .- movavg(x, n_lo)
    n_hi = max(1, round(Int, 1 / (f_hi * dt)))
    return movavg(hp, n_hi)
end

"RMS of `x` over non-overlapping `seg`-second windows of `t`; one value per window, at its midpoint."
function segment_rms(x, t; seg = 5.0)
    mids = Float64[]; rms = Float64[]
    edges = t[1]:seg:t[end]
    for k in 1:(length(edges) - 1)
        idx = findall(τ -> edges[k] <= τ < edges[k + 1], t)
        isempty(idx) && continue
        push!(mids, (edges[k] + edges[k + 1]) / 2)
        push!(rms, sqrt(mean(x[idx] .^ 2)))
    end
    return mids, rms
end

"Slope [·/s] of a linear fit of `y` against `x`; NaN with fewer than 2 points."
function slope(x, y)
    length(x) < 2 && return NaN
    xm = mean(x)
    return sum((x .- xm) .* y) / sum((x .- xm) .^ 2)
end

"""
    zero_cross_freq(x, t) -> Float64

Dominant frequency [Hz] of `x`, as half its zero-crossing rate over `t`. No
FFT in `examples/Project.toml`, and V1 only needs the ringing's frequency, not
its full spectrum (that is V2's job).
"""
function zero_cross_freq(x, t)
    n = count(i -> sign(x[i]) != sign(x[i + 1]) && x[i] != 0, 1:(length(x) - 1))
    return n / 2 / (t[end] - t[1])
end

"""
    amp_spectrum(x, dt, fs) -> Vector{Float64}

Amplitude spectrum of `x` [same unit] at the frequencies `fs` [Hz], by a direct
DFT with a Hann window (no FFT in `examples/Project.toml`; a few hundred
frequencies over one window are cheap).
"""
function amp_spectrum(x, dt, fs)
    n = length(x)
    w = 0.5 .- 0.5 .* cos.(2π .* (0:(n - 1)) ./ (n - 1))
    xw = (x .- mean(x)) .* w
    tt = (0:(n - 1)) .* dt
    return [abs(sum(xw .* cis.(-2π * f .* tt))) * 2 / sum(w) for f in fs]
end

"""
    excess_peak(run, baseline; fs = 0.15:0.01:1.2) -> NamedTuple

Frequency [Hz] and size [deg] of the largest excess of `run`'s regulated-error
spectrum over `baseline`'s. Near a stability boundary the closed loop has a
lightly damped mode that the pattern keeps exciting, so this peak marks the
critical frequency even when the lap forcing hides it in the time domain
(point D, oldplans/Plan_model_validation.md). In a saturated limit cycle it falls
below the linear critical frequency.
"""
function excess_peak(run, baseline; fs = 0.15:0.01:1.2)
    ex = amp_spectrum(run.err, run.dt, fs) .- amp_spectrum(baseline.err, baseline.dt, fs)
    i = argmax(ex)
    return (; f_hz = fs[i], deg = ex[i])
end

"Unwrap an angle series [rad], so a DFT does not see the ±π jumps."
unwrap_angle(a) = first(a) .+ cumsum(vcat(0.0, rem2pi.(diff(a), RoundNearest)))

"""
    window_signals(r) -> NamedTuple

The logged signals of `r`'s analysis window: the command `u` (`set_steering`),
the tape `s` (`steering`), `heading` and `course` [rad, unwrapped], the
regulated error `err` [rad] (`var_06`), and the time step `dt` [s].
"""
function window_signals(r)
    sl = load_log(basename(r.log_path); path = dirname(r.log_path)).syslog
    t = Float64.(sl.time)
    w = findall(k -> r.window[1] <= t[k] <= r.window[2], eachindex(t))
    return (; u = Float64.(sl.set_steering[w]), s = Float64.(sl.steering[w]),
              heading = unwrap_angle(Float64.(sl.heading[w])),
              course = unwrap_angle(Float64.(sl.course[w])),
              err = deg2rad.(Float64.(sl.var_06[w])), dt = t[w[2]] - t[w[1]])
end

"""
    command_mode(r, baseline; band = (0.8, 3.0), fs = 0.3:0.02:3.0) -> NamedTuple

Frequency [Hz] and amplitude [-] of the largest excess of `r`'s steering
command spectrum over `baseline`'s, within `band`. The onset signature at
point D: a mode that grows in the command until the command reaches the clamp
(oldplans/Plan_model_validation.md, "point D without the rate limit"). The gain
test's mode is near 1.1 Hz, the delay test's near 0.5 Hz: use `band = (0.3, 1.5)`
for the delay test.
"""
function command_mode(r, baseline; band = (0.8, 3.0), fs = 0.3:0.02:3.0)
    c, cb = window_signals(r), window_signals(baseline)
    ex = amp_spectrum(c.u, c.dt, fs) .- amp_spectrum(cb.u, cb.dt, fs)
    idx = findall(f -> band[1] <= f <= band[2], fs)
    i = idx[argmax(ex[idx])]
    return (; f_hz = fs[i], amp = ex[i])
end

"""
    oscillating(r, baseline; band = (0.8, 3.0), min_amp = 0.02) -> Bool

The command at the clamp and a `command_mode` of at least `min_amp`: the loop
has gone unstable into a clamped oscillation. Works whether or not the tape is
on its rate limit; the thresholds used at point D were `band = (0.8, 3.0)`,
`min_amp = 0.02` (gain test) and `band = (0.3, 1.5)`, `min_amp = 0.05` (delay
test).
"""
oscillating(r, baseline; band = (0.8, 3.0), min_amp = 0.02) =
    r.peak_cmd_frac >= 0.99 && command_mode(r, baseline; band).amp > min_amp

"Fourier coefficient of `x` [same unit] at `f` [Hz], Hann window, amplitude-scaled."
function dft_at(x, dt, f)
    n = length(x)
    w = 0.5 .- 0.5 .* cos.(2π .* (0:(n - 1)) ./ (n - 1))
    return sum((x .- mean(x)) .* w .* cis.(-2π * f .* (0:(n - 1)) .* dt)) * 2 / sum(w)
end

"""
    loop_breakdown(r, f) -> NamedTuple

Split the loop of an oscillating run `r` into its links at the oscillation's
frequency `f` [Hz], where the mode dominates every logged signal: gain ratio
and phase difference [deg] of each link, measured over modelled —

- `tape`: command → tape, against `1/(1 + s·v1_lag)`;
- `kite`: tape → heading, against the turn-rate law with the kite's dead time
  and lag, without the gravity term;
- `heading_to_course` and `heading_to_err`: course and the regulated error
  (course − commanded course) over heading, which the model does not contain
  (it takes the heading as the fed-back angle), so they are shown as measured.

At point D their product reproduced the loop gain from the margins at both
onset frequencies (oldplans/Plan_model_validation.md).
"""
function loop_breakdown(r, f)
    c = window_signals(r)
    U, S = dft_at(c.u, c.dt, f), dft_at(c.s, c.dt, f)
    Ψ, Χ, E = dft_at(c.heading, c.dt, f), dft_at(c.course, c.dt, f), dft_at(c.err, c.dt, f)
    project = project_file(V1_POINTS[r.point].project)
    tc = turn_rate_coeffs(FC_Settings(fc_settings(project)).body_damping, r.depower)
    ω = 2π * f
    kite_model = tc.c1 * r.v_a_mean * cis(-ω * kite_dead_time(tc, r.v_a_mean)) /
                 ((1 + im * ω * kite_lag(tc, r.v_a_mean)) * (im * ω))
    tape_model = 1 / (1 + im * ω * v1_lag(r.point))
    link(x) = (; ratio = abs(x), dphase = rad2deg(angle(x)))
    return (; f, tape = link((S / U) / tape_model), kite = link((Ψ / S) / kite_model),
              heading_to_course = link(Χ / Ψ), heading_to_err = link(E / Ψ))
end

"""
    analyze(point, log_path; hook_settle, gain_factor, extra_delay, label,
            baseline = nothing, test = :gain) -> NamedTuple

Read `log_path.arrow` (`KiteUtils.load_log`) and, from `t_phase4 + hook_settle`
to the end of the run — restricted to samples still in phase 4, and, for point
`:C` only, cut short at the first excursion of `v_a` more than 10 % from its
own mean over that stretch (oldplans/Plan_model_validation.md#operating-points) —
report the window's mean/std `v_a`, mean depower, the tape's rate-limited
fraction (as `SimpleKiteControllers.print_fig8_metrics` does), and, if
`baseline` is given (another `analyze` result with `gain_factor = 1`,
`extra_delay = 0`), the stability verdict from `onset`.
"""
function analyze(point, log_path; hook_settle = HOOK_SETTLE_V1, gain_factor = 1.0,
                  extra_delay = 0, label = "run", baseline = nothing, test::Symbol = :gain)
    dir, name = dirname(log_path), basename(log_path)
    syslog = load_log(name; path = dir)
    sl = syslog.syslog

    phase = Int.(sl.sys_state)
    t = Float64.(sl.time)
    i4 = findfirst(==(4), phase)
    isnothing(i4) && error("analyze: phase 4 never reached in $log_path.")
    t_phase4 = t[i4]

    active = findall(k -> phase[k] == 4 && t[k] - t_phase4 >= hook_settle, eachindex(t))
    isempty(active) &&
        error("analyze: the run ended before t_phase4 + hook_settle ($(t_phase4 + hook_settle) s); \
               lengthen sim_time for $point.")

    # Point C only (oldplans/Plan_model_validation.md#operating-points): the
    # reel-out project's v_a DRIFTS as the tether pays out, so its window is
    # cut at the first excursion of v_a outside ±10 % of the window's own
    # mean, rather than dropping interior samples, so the remaining stretch
    # stays contiguous for the moving-average filter below. Points A/B fly a
    # closed pattern instead: v_a swings ±20 % or more WITHIN A SINGLE LAP as
    # the ordinary, periodic apparent-wind variation of the figure-eight, not
    # drift, so the same cut there would shred an otherwise-stable window down
    # to a fraction of a second the moment that swing exceeds 10 % — as it did
    # empirically (2026-09-27, point A's delay sweep) before this was scoped
    # to :C.
    if point == :C
        # Judged on 10 s means: within a lap v_a swings by more than 10 % on its own.
        v_a_all = Float64.(sl.v_app[active])
        tt = t[active]
        va0 = mean(v_a_all)
        h = max(1, round(Int, 5 / (tt[2] - tt[1])))
        cs = vcat(0.0, cumsum(v_a_all))
        mean10 = [(cs[min(k + h, end - 1) + 1] - cs[max(k - h, 1)]) / (min(k + h, length(tt)) - max(k - h, 1) + 1)
                  for k in eachindex(tt)]
        cut = findfirst(v -> abs(v - va0) / va0 > 0.10, mean10)
        isnothing(cut) || (active = active[1:(cut - 1)])
    end

    tw = t[active]
    dt = length(tw) > 1 ? tw[2] - tw[1] : NaN
    err = Float64.(sl.var_06[active])           # regulated error [deg]
    v_a = Float64.(sl.v_app[active])
    depower = Float64.(sl.depower[active])
    steering = Float64.(sl.steering[active])    # delivered by the tape
    v_steering = Settings(project_file(V1_POINTS[point].project)).v_steering
    tape_rate = length(steering) > 1 ? abs.(diff(steering)) ./ dt : Float64[]
    rate_limit_frac = isempty(tape_rate) ? 0.0 :
        count(>=(0.95 * v_steering), tape_rate) / length(tape_rate)

    # Commanded steering against the clamp: with the rate-limited fraction, the
    # gate for a baseline with small-signal headroom.
    max_steering = FC_Settings(fc_settings(project_file(V1_POINTS[point].project))).max_steering
    peak_cmd_frac = maximum(abs.(Float64.(sl.set_steering[active]))) / max_steering

    result = (; point, label, gain_factor, extra_delay, hook_settle, test,
              log_path, t_phase4, window = (tw[1], tw[end]),
              v_a_mean = mean(v_a), v_a_std = std(v_a), depower = mean(depower),
              rate_limit_frac, peak_cmd_frac, err, tw, dt)
    isnothing(baseline) && return result
    return merge(result, onset(result, baseline))
end

"""
    onset(run, baseline) -> NamedTuple

Judge `run` (an `analyze` result) against `baseline` (also `analyze`, with
`gain_factor = 1`, `extra_delay = 0`): band-pass the regulated error in
`run.test`'s band (0.4 – 2 Hz for `:gain`, 0.1 – 1 Hz for `:delay` —
oldplans/Plan_model_validation.md#procedure), fit the slope of its segment RMS
over time, net of the baseline's, and the ringing frequency net of the
baseline's own zero-crossing frequency in the same band.

`verdict` is `:indeterminate` when `run`'s window is too short to fit at least
2 segment-RMS points (fewer than 10 s, after any point-`:C` cut in `analyze`) —
NEVER treat this as `:stable`, it usually means the run was cut short by
something worth looking at, not that it settled; `:rate_limited` when `run`'s
tape spends more than 10 percentage points longer at its rate limit than the
baseline's (a limit cycle, not a linear onset — bisect only between runs that
are neither of these two); otherwise `:unstable` when the net growth is
positive while the band-passed RMS is still below 5°, and `:stable` when it
decays or does not exceed the baseline.
"""
function onset(run, baseline)
    f_lo, f_hi = run.test == :gain ? (0.4, 2.0) : (0.1, 1.0)
    e_run = bandpass(run.err, run.dt; f_lo, f_hi)
    e_base = bandpass(baseline.err, baseline.dt; f_lo, f_hi)
    mids_r, rms_r = segment_rms(e_run, run.tw)
    mids_b, rms_b = segment_rms(e_base, baseline.tw)
    g_run, g_base = slope(mids_r, rms_r), slope(mids_b, rms_b)
    net_growth = g_run - g_base
    net_rms_deg = isempty(rms_r) ? NaN : last(rms_r) - (isempty(rms_b) ? 0.0 : last(rms_b))
    f_ring = zero_cross_freq(e_run, run.tw)
    f_ring_net = f_ring - zero_cross_freq(e_base, baseline.tw)
    rate_limited = run.rate_limit_frac - baseline.rate_limit_frac > 0.10
    verdict = length(mids_r) < 2 ? :indeterminate :
        rate_limited ? :rate_limited :
        (net_growth > 0 && maximum(abs.(e_run)) < 5.0) ? :unstable : :stable
    return (; f_lo, f_hi, net_growth, net_rms_deg, f_ring_hz = f_ring,
              f_ring_net_hz = f_ring_net, verdict)
end

# ==================== V2: INJECTED MULTISINE ==================== #

"""
    Multisine(; period = 10.0, freqs = [0.1:0.1:1.0; 1.2:0.2:2.0], amp = 0.004)

A periodic test input `τ -> Δu` [-] for `run_v1(...; injection)`: sines at
`freqs` [Hz], each a multiple of `1/period` so every line completes whole
cycles in one period, each of amplitude `amp`, with Schroeder phases for a low
crest factor. V2 of oldplans/Plan_model_validation.md.
"""
struct Multisine
    period::Float64
    freqs::Vector{Float64}
    amp::Float64
    phases::Vector{Float64}
end

function Multisine(; period = 10.0, freqs = [0.1:0.1:1.0; 1.2:0.2:2.0], amp = 0.004)
    all(f -> abs(f * period - round(f * period)) < 1e-9, freqs) ||
        error("Multisine: every line must be a multiple of 1/period = $(1 / period) Hz.")
    n = length(freqs)
    return Multisine(period, collect(Float64, freqs), amp, [-π * k * (k - 1) / n for k in 1:n])
end

(m::Multisine)(τ) = m.amp * sum(sin(2π * f * τ + φ) for (f, φ) in zip(m.freqs, m.phases))

"""
    frf_injection(r, m::Multisine; skip = 1, t_end = Inf) -> Vector{NamedTuple}

Frequency responses of the links of the loop at every line of `m`, from a run
`r` flown with `injection = m`: command → tape (`tape`), command → heading
(`heading`), heading → course (`course`) and course → regulated error
(`err`). Each whole period after the injection started (the first `skip`
periods dropped as transient, only phase 4) is Fourier-transformed at the
lines, up to `t_end` [s] (for a drifting operating point, like the reel-out's);
the spectra are averaged over the periods, which keeps what is
periodic with the injection and averages out the pattern's own content, and
the links are ratios of the averages. `*_sd` is the standard deviation of the
per-period ratio's magnitude, relative, over the periods.
"""
function frf_injection(r, m::Multisine; skip = 1, t_end = Inf)
    sl = load_log(basename(r.log_path); path = dirname(r.log_path)).syslog
    t = Float64.(sl.time)
    phase = Int.(sl.sys_state)
    t0 = r.t_phase4 + r.hook_settle + skip * m.period
    last4 = findlast(k -> phase[k] == 4, eachindex(t))
    np = floor(Int, (min(t[last4], t_end) - t0) / m.period)
    np >= 2 || error("frf_injection: fewer than 2 whole periods after the transient; lengthen sim_time.")
    u, s = Float64.(sl.set_steering), Float64.(sl.steering)
    ψ, χ = unwrap_angle(Float64.(sl.heading)), unwrap_angle(Float64.(sl.course))
    e = deg2rad.(Float64.(sl.var_06))
    coeff(x, idx, f) = 2 / length(idx) * sum((x[idx] .- mean(x[idx])) .* cis.(-2π * f .* (t[idx] .- t[idx[1]])))
    rows = map(m.freqs) do f
        per = map(1:np) do k
            idx = findall(τ -> t0 + (k - 1) * m.period <= τ < t0 + k * m.period, t)
            (; U = coeff(u, idx, f), S = coeff(s, idx, f), Ψ = coeff(ψ, idx, f),
               X = coeff(χ, idx, f), E = coeff(e, idx, f))
        end
        avg(key) = mean(getfield.(per, key))
        rel_sd(num, den) = std([abs(getfield(p, num) / getfield(p, den)) for p in per]) /
                           abs(avg(num) / avg(den))
        (; f, n_periods = np, u_amp = abs(avg(:U)),
           tape = avg(:S) / avg(:U), heading = avg(:Ψ) / avg(:U),
           course = avg(:X) / avg(:Ψ), err = avg(:E) / avg(:X),
           heading_sd = rel_sd(:Ψ, :U), course_sd = rel_sd(:X, :Ψ), err_sd = rel_sd(:E, :X))
    end
    return rows
end

"""
    lap_period(r) -> Float64

Steady lap time [s] of run `r`: the median of the second half of its laps,
from `SysState`'s live lap count `fig_8`. The first laps after the entry are
shorter (at 200 m 12.5 – 12.9 s against a steady 13.15 s), and a lap period
2.5 % off already puts `mid_lines` onto the lap's harmonics by 0.45 Hz, so a
short baseline is not enough: use a run of 150 s or more.
"""
function lap_period(r)
    sl = load_log(basename(r.log_path); path = dirname(r.log_path)).syslog
    t, n = Float64.(sl.time), Int.(sl.fig_8)
    starts = [t[i] for i in 2:length(t) if n[i] > n[i-1] && n[i-1] >= 1]
    length(starts) >= 5 || error("lap_period: fewer than five laps in $(r.log_path).")
    d = diff(starts)
    d = sort(d[(length(d) ÷ 2 + 1):end])
    return d[(length(d) + 1) ÷ 2]
end

"""
    mid_lines(T_lap, f_lo, f_hi; every = 1) -> (period, freqs)

Injection lines halfway between the lap's harmonics, `(n + ½)/T_lap` [Hz],
from `f_lo` to `f_hi`, every `every`-th one, and the matching `Multisine`
period `2·T_lap`. A figure-eight's heading carries mainly the ODD harmonics of
the lap, so lines on a plain grid can land on them and pick up the pattern
instead of the injection (oldplans/Plan_model_validation.md, V1 step 1).
"""
function mid_lines(T_lap, f_lo, f_hi; every = 1)
    P = 2T_lap
    ns = ceil(Int, f_lo * P / 2 - 0.5):every:floor(Int, f_hi * P / 2 - 0.5)
    return P, [(2n + 1) / P for n in ns]
end

"""
    guidance_rate(r) -> NamedTuple

The guidance corner `ω_g = v_k/(L·D)` [rad/s] of run `r` over its analysis
window (`guidance_tf`), with the kite speed `v_k`, tether length `L` and
attractor distance `D` [deg] it comes from.
"""
function guidance_rate(r)
    sl = load_log(basename(r.log_path); path = dirname(r.log_path)).syslog
    t = Float64.(sl.time)
    w = findall(k -> r.window[1] <= t[k] <= r.window[2], eachindex(t))
    v_k = mean(sqrt(sum(abs2, v)) for v in sl.vel_kite[w])
    L = mean(Float64(x[1]) for x in sl.l_tether[w])
    f = FC_Settings(fc_settings(project_file(V1_POINTS[r.point].project)))
    D = attractor_distance(f, r.v_a_mean, L)
    return (; ω_g = v_k / (L * deg2rad(D)), v_k, L, D)
end

"""
    course_controller_tf(point, v_a; depower = nothing) -> (C, Ts)

Discrete course PD of `point` at `v_a` [m/s], with the pattern's gain floor,
and its sample time. For the reel-out script, `depower` [-] (the depower flown)
adds its gain scale `c1(depower_setpoint)/c1(depower)`, which keeps the loop
gain at `heading_p · c1(depower_setpoint)` whatever depower the optimizer flies.
"""
function course_controller_tf(point, v_a; depower = nothing)
    project = project_file(V1_POINTS[point].project)
    f = FC_Settings(fc_settings(project))
    Ts = 1 / Settings(project).sample_freq
    K = f.heading_p * f.v_app_ref / max(v_a, max(f.v_app_min, f.v_app_min_pattern))
    if V1_POINTS[point].script == "simple_opt_reelout.jl" && !isnothing(depower)
        K *= turn_rate_coeffs(f.body_damping, f.depower_setpoint).c1 /
             turn_rate_coeffs(f.body_damping, depower).c1
    end
    return course_pid(K, f.heading_i, f.heading_d, f.heading_d_n, Ts), Ts
end

"""
    measured_loop(runs; band = (0.0, Inf)) -> (f, L)

The loop `C · (command → course) · (1 + ω_g/s)` on the measured plant: for
each injection run in `runs` (results of `run_v1` + `frf_injection`, as
NamedTuples with fields `r` and `frf`), the command → course response at its
lines within `band` [Hz], times the course PD at the run's `v_a` and the
guidance at its `ω_g`. Lines within 0.006 Hz of each other are averaged. Feed
the result to `frd_margins`.
"""
function measured_loop(runs; band = (0.0, Inf))
    pts = Tuple{Float64, ComplexF64}[]
    for x in runs
        C, Ts = course_controller_tf(x.r.point, x.r.v_a_mean; depower = x.r.depower)
        ω_g = guidance_rate(x.r).ω_g
        for q in x.frf
            band[1] <= q.f <= band[2] || continue
            Cz = evalfr(C, cis(2π * q.f * Ts))[1]
            push!(pts, (q.f, Cz * q.heading * q.course * (1 + ω_g / (im * 2π * q.f))))
        end
    end
    sort!(pts; by = first)
    groups = Vector{Vector{Tuple{Float64, ComplexF64}}}()
    for pt in pts
        if !isempty(groups) && pt[1] - groups[end][1][1] < 0.006
            push!(groups[end], pt)
        else
            push!(groups, [pt])
        end
    end
    return [mean(first.(g)) for g in groups], [mean(last.(g)) for g in groups]
end

"""
    model_loops(r) -> NamedTuple

The model's loops at run `r`'s operating point (`v_a`, depower, the
project's gains and sample rate, the worst gravity sign as in `predict`):
`inner` (`C·P`), `guided` (× `guidance_tf` at the run's `ω_g`) and
`corrected` (the pattern model: the pattern law's dead time and lag,
`kite_correction` and the guidance).
"""
function model_loops(r)
    p = V1_POINTS[r.point]
    project = project_file(p.project)
    f = FC_Settings(fc_settings(project))
    C, Ts = course_controller_tf(r.point, r.v_a_mean; depower = r.depower)
    tc = turn_rate_coeffs(f.body_damping, r.depower)
    # One model to set against the measurement: the stable sign of the gravity pole, with the
    # plant's c1 and c2 of the low pattern (plant_coeffs, course_loop_model.jl).
    pc = plant_coeffs(r.depower)
    c2 = pc.c2
    P = turn_rate_plant(pc.c1, c2, kite_dead_time(tc, r.v_a_mean), r.v_a_mean,
                        -cosd(f.el_center), Ts; lag = v1_lag(r.point),
                        kite_lag = kite_lag(tc, r.v_a_mean))
    G = guidance_tf(guidance_rate(r).ω_g, Ts)
    # The corrected loop is the pattern model: the pattern law's dead time and lag, kite_correction.
    τp, Tp = pattern_dead_time_lag(tc, r.v_a_mean, r.depower)
    Pp = turn_rate_plant(pc.c1, c2, τp, r.v_a_mean, -cosd(f.el_center), Ts; lag = v1_lag(r.point),
                         kite_lag = Tp)
    return (; inner = C * P, guided = C * P * G, corrected = C * Pp * G * kite_correction(Ts))
end

"Delay margin, crossovers and gain margin of a model loop, in the same fields as `frd_margins`."
function tf_margins(L)
    wgm, gm, wpm, pm = margin(L; allMargins = true)
    f_gc = wpm[1][1] / 2π
    return (; f_gc, pm = mod(pm[1][1], 360), dm = delay_margin(L), f_pc = wgm[1][1] / 2π,
              gm = gm[1][1])
end

# ==================== SWEEPS (bisection) ==================== #

"Print one sweep step: what was set, what came back."
function _log_step(test, x, r)
    name = test == :gain ? "gain_factor" : "extra_delay"
    @info @sprintf("  %s = %-8.4g -> %-13s net growth %+7.3f deg/s, ring %.3f Hz",
                   name, x, r.verdict, r.net_growth, r.f_ring_hz)
end

"""
Largest baseline rate-limited fraction for which a saturated limit cycle
(`limit_cycle`) counts as the loop going unstable. From such a baseline the tape
is mostly off its limit, so a run that ends up pinned on it has gone into the
limit cycle an unstable loop becomes once the tape limits it. From a baseline
already on the limit (point A) that says nothing about stability.
"""
const CLEAN_BASELINE = 0.10

"""
    limit_cycle(r) -> Bool

The command at the clamp and the tape rate-limited over half the window. The
`:rate_limited` verdict alone ("+10 points over the baseline") is not enough:
in the feedback-only gain test the rate-limited fraction rises smoothly with
the gain and crosses that line long before any oscillation (point D,
oldplans/Plan_model_validation.md).
"""
limit_cycle(r) = r.peak_cmd_frac >= 0.99 && r.rate_limit_frac > 0.5

"Whether `r` is on the unstable side of the onset, judged against `baseline`."
is_unstable(r, baseline) = r.verdict == :unstable ||
    (baseline.rate_limit_frac < CLEAN_BASELINE && limit_cycle(r))

"""
    bracket(runs, baseline, key) -> Union{Nothing, Tuple}

The largest `key` (`:gain_factor` or `:extra_delay`) of a run that is not
unstable, below the smallest one of an unstable run (`is_unstable`), as
`(lo, hi)`, or `nothing` if `runs` do not bracket the onset.
"""
function bracket(runs, baseline, key)
    hi_runs = filter(r -> is_unstable(r, baseline), runs)
    isempty(hi_runs) && return nothing
    hi = minimum(r -> getfield(r, key), hi_runs)
    lo_runs = filter(r -> r.verdict in (:stable, :rate_limited) && !is_unstable(r, baseline) &&
                          getfield(r, key) < hi, runs)
    isempty(lo_runs) && return nothing
    return (maximum(r -> getfield(r, key), lo_runs), hi)
end

"""
    sweep_gain(point, baseline; factors = nothing, feedback_only = false,
               rel_tol = 0.02) -> Vector

Run `point` at a coarse grid around the model's predicted gain margin — 0.6,
0.8, 1.0, 1.2, 1.4 × it, at `baseline`'s measured `v_a`/depower, unless
`factors` overrides the grid — then bisect between the last `:stable` and the
first unstable run (`is_unstable`) until the bracket is narrower than
`rel_tol` of its upper end. Returns every run in order; `bracket(runs,
baseline, :gain_factor)` gives `k_crit`. `feedback_only` is passed to
`run_v1`; with `ff_gain > 0` it scales the loop gain without scaling the
feed-forward, which lies outside the loop.
"""
function sweep_gain(point::Symbol, baseline; factors = nothing, feedback_only = false,
                    rel_tol = 0.02)
    pred = predict(point; v_a = baseline.v_a_mean, depower = baseline.depower)
    grid = something(factors, pred.gain_margin .* (0.6, 0.8, 1.0, 1.2, 1.4))
    runs = Any[]
    for (i, k) in enumerate(grid)
        r = run_v1(point; gain_factor = k, extra_delay = 0, feedback_only,
                   label = "gain_$i", baseline, test = :gain)
        push!(runs, r)
        _log_step(:gain, k, r)
    end
    b = bracket(runs, baseline, :gain_factor)
    if isnothing(b)
        @warn "sweep_gain($point): the grid never bracketed the onset — widen `factors`."
        return runs
    end
    lo, hi = b
    j = 0
    while (hi - lo) > rel_tol * hi
        j += 1
        mid = (lo + hi) / 2
        r = run_v1(point; gain_factor = mid, extra_delay = 0, feedback_only,
                   label = "gain_bisect$j", baseline, test = :gain)
        push!(runs, r)
        _log_step(:gain, mid, r)
        if is_unstable(r, baseline)
            hi = mid
        elseif r.verdict in (:stable, :rate_limited)
            lo = mid
        else
            @warn "sweep_gain($point): $(r.verdict) at $mid — bisection stopped."
            break
        end
    end
    return runs
end

"""
    sweep_delay(point, baseline; delays = nothing) -> Vector

Same as `sweep_gain`, stepping `extra_steer_delay` [samples] around the
model's predicted delay margin instead of `steer_gain_factor`, and bisecting
down to one sample; `bracket(runs, baseline, :extra_delay)` gives `τ_crit`.
"""
function sweep_delay(point::Symbol, baseline; delays = nothing)
    pred = predict(point; v_a = baseline.v_a_mean, depower = baseline.depower)
    grid = something(delays, round.(Int, pred.delay_margin_samples .* (0.6, 0.8, 1.0, 1.2, 1.4)))
    runs = Any[]
    for (i, n) in enumerate(grid)
        r = run_v1(point; gain_factor = 1.0, extra_delay = n, label = "delay_$i",
                   baseline, test = :delay)
        push!(runs, r)
        _log_step(:delay, n, r)
    end
    b = bracket(runs, baseline, :extra_delay)
    if isnothing(b)
        @warn "sweep_delay($point): the grid never bracketed the onset — widen `delays`."
        return runs
    end
    lo, hi = b
    j = 0
    while hi - lo > 1
        j += 1
        mid = (lo + hi) ÷ 2
        r = run_v1(point; gain_factor = 1.0, extra_delay = mid,
                   label = "delay_bisect$j", baseline, test = :delay)
        push!(runs, r)
        _log_step(:delay, mid, r)
        if is_unstable(r, baseline)
            hi = mid
        elseif r.verdict in (:stable, :rate_limited)
            lo = mid
        else
            @warn "sweep_delay($point): $(r.verdict) at $mid — bisection stopped."
            break
        end
    end
    return runs
end

# ==================== REPORT ==================== #

"""
    report(point, baseline, gain_runs, delay_runs; io = stdout)

Print V1's pass/fail table: `k_crit` and `τ_crit` as the middle of each sweep's
`bracket`, and the critical frequency as the `excess_peak` of the last stable
run below the onset (the lightly damped mode the pattern excites there),
against the model's gain margin, phase crossover, delay margin and gain
crossover — and whether each is within tolerance (±20 % / ±25 % / ±0.1 Hz /
±0.1 Hz, oldplans/Plan_model_validation.md#procedure), to `io`. Either vector may
be empty.
"""
function report(point::Symbol, baseline, gain_runs, delay_runs; io::IO = stdout)
    pred = predict(point; v_a = baseline.v_a_mean, depower = baseline.depower)
    @printf(io, "V1 point %s — v_a %.1f m/s, depower %.3f, tape lag %.2f s, α %.2f\n",
            point, pred.v_a, pred.depower, pred.lag, pred.alpha)
    @printf(io, "  baseline: v_a %.2f ± %.2f m/s, rate-limited %.1f%%, peak command %.0f%% of max_steering\n",
            baseline.v_a_mean, baseline.v_a_std, 100 * baseline.rate_limit_frac,
            100 * baseline.peak_cmd_frac)
    verdict(ok) = ok ? "PASS" : "FAIL"
    for (name, runs, key, model, f_model, tol, scale) in
            (("gain ", gain_runs, :gain_factor, pred.gain_margin, pred.phase_crossover_hz, 0.20, 1.0),
             ("delay", delay_runs, :extra_delay, pred.delay_margin_s, pred.gain_crossover_hz, 0.25, pred.Ts))
        b = isempty(runs) ? nothing : bracket(runs, baseline, key)
        if isnothing(b)
            println(io, "  $name: no bracket.")
            continue
        end
        crit = (b[1] + b[2]) / 2 * scale
        below = filter(r -> !is_unstable(r, baseline) && getfield(r, key) == b[1], runs)[end]
        f = excess_peak(below, baseline).f_hz
        @printf(io, "  %s: critical %.3g (bracket %.3g – %.3g), model %.3g, %s | f %.2f Hz, model %.2f Hz, %s\n",
                name, crit, b[1] * scale, b[2] * scale, model,
                verdict(abs(crit / model - 1) <= tol), f, f_model, verdict(abs(f - f_model) <= 0.1))
    end
    nothing
end

@info "validate_margins.jl loaded. Try: predict(:D); base = run_v1(:D)."
