# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
V3 of `oldplans/Plan_model_validation.md`: k-step-ahead prediction of the course-loop
plant on held-out logs.

The logged steering command `set_steering` is passed through the plant of
`course_loop_model.jl`, in the time domain and with every parameter updated
each sample from the log:

    tape:      T_act·u̇_s = u_cmd - u_s,             T_act = 1/steering_gain
    kite:      u_k(t) = u_s(t - τ_kite) through a lag T_kite,  τ, T from v_a
    [optional] kite_correction, the lag-lead (1 + s/ω_z)/(1 + s/ω_p)
    turn rate: ψ̇ = c1(u_d)·v_a·u_k + C3·sin(ψ)·cos(β)

`c1` at the logged depower `u_d`, `β` the logged elevation; `C3` the gravity
coefficient identified on the flown figures of eight (`course_loop_model.jl`). The steering
chain depends on the input alone and runs over the whole log; only the heading
integrates, so it is re-initialized from the log every `H` seconds. Three
variants separate the error sources:

- `:model`: the whole chain from the command, as the stability analysis has it;
- `:kite`: the logged tape position `steering` as the input, i.e. the kite
  model alone, without the tape's rate limit;
- `:kite_corr`: as `:kite`, with `kite_correction`.

Per `v_a` bin (5 m/s): the variance accounted for (VAF) of the turn rate, and
the heading error after `H` = 1, 2, 3 s, relative to the heading change over
`H`. Residual check: the cross-correlation of the turn-rate residual with the
command; a peak at lag `k` means the dead time is off by about `k·Ts`.

    include("examples/replay_prediction.jl")
    v3_report()                      # the default logs, V3_LOGS
    v3_report(V3_LOGS[1:2])          # a subset

The logs are archived runs of `validate_margins.jl` (`output/archives/`), all
with `steering_gain` 10, none used to identify the turn-rate table (relay
sweeps) or to fit `kite_correction` (the 300 m injection runs).
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file
using ControlSystemsBase
using SimpleKiteControllers: C3, KITE_CORR_ZERO, KITE_CORR_POLE
using Statistics: mean, std, var
using Printf

set_data_path(normpath(joinpath(@__DIR__, "..", "data")))

const ARCHIVES = normpath(joinpath(@__DIR__, "..", "output", "archives"))

"""
    V3Log(dir, project, note)

An archived run: its folder under `output/archives/` (a unique prefix is
enough), the system project it flew and a short note on the conditions.
"""
struct V3Log
    dir::String
    project::String
    note::String
end

"The held-out logs of V3 (2026-09-27), all `steering_gain` 10."
const V3_LOGS = [
    V3Log("v1_A_baseline_g10", "system_fig8_200m.yaml", "200 m, 7 m/s, rate limit 0.2"),
    V3Log("v1_A_v2_200m_long", "system_fig8_200m.yaml", "200 m, 7 m/s, injection, no rate limit"),
    V3Log("v1_B_v2_baseline", "system_fig8_200m.yaml", "200 m, 4.5 m/s, no rate limit"),
    V3Log("v1_B_wind_inj", "system_fig8_200m.yaml", "200 m, 4.5 m/s, injection, no rate limit"),
    V3Log("v1_D_baseline_g10", "system_fig8_300m.yaml", "300 m, 7 m/s, rate limit 0.2"),
    V3Log("v1_D5_wind_baseline", "system_fig8_300m.yaml", "300 m, 5 m/s, no rate limit"),
    V3Log("v1_D5_wind_inj", "system_fig8_300m.yaml", "300 m, 5 m/s, injection, no rate limit"),
    V3Log("v1_D85_wind_baseline", "system_fig8_300m.yaml", "300 m, 8.5 m/s, no rate limit"),
    V3Log("v1_D85_wind_inj", "system_fig8_300m.yaml", "300 m, 8.5 m/s, injection, no rate limit"),
    V3Log("v1_F_big_baseline", "system_fig8_150m.yaml", "150 m, 7 m/s, pattern 42 x 17 deg, no rate limit"),
    V3Log("v1_F_big_inj", "system_fig8_150m.yaml", "150 m, 7 m/s, pattern 42 x 17 deg, injection, no rate limit"),
    V3Log("v1_C_v2_inj", "system_reelout_maasvlakte.yaml", "reel-out 150 -> 380 m, 4 m/s, injection, no rate limit"),
]

"Folder of `log` under `output/archives/`: the newest one starting with `log.dir`."
function log_folder(log::V3Log)
    ds = sort(filter(d -> startswith(d, log.dir), readdir(ARCHIVES)))
    isempty(ds) && error("replay: no archive starting with $(log.dir) in $ARCHIVES")
    return joinpath(ARCHIVES, last(ds))
end

"""
    replay(log; variant = :model, min_phase = 3) -> NamedTuple

Replay `log` (a `V3Log`) through the plant, variant `:model`, `:kite` or
`:kite_corr` (see the file's docstring). Returns, over the samples from the
first one in phase `min_phase` on (the last sample dropped): the time, `v_a`,
depower, the measured and modelled turn rate [rad/s], the unwrapped logged
heading, the command, the elevation and the per-sample `c1·v_a·u` and gravity
terms needed by [`heading_errors`](@ref).
"""
function replay(log::V3Log; variant::Symbol = :model, min_phase = 3)
    dir = log_folder(log)
    name = first(splitext(only(filter(f -> endswith(f, ".arrow"), readdir(dir)))))
    sl = load_log(name; path = dir).syslog
    project = project_file(log.project)
    set = Settings(project)
    fcs = FC_Settings(fc_settings(project))
    t = Float64.(sl.time)
    Ts = t[2] - t[1]
    T_act = 1 / set.steering_gain
    u_cmd = Float64.(sl.set_steering)
    u_tape_log = Float64.(sl.steering)
    v_a = max.(Float64.(sl.v_app), 1.0)
    dp = Float64.(sl.depower)
    β = Float64.(sl.elevation)
    ψ = unwrap_angle(Float64.(sl.heading))
    n = length(t)

    # Clamped to the table's range: the log starts at 0 before the first step, and only
    # samples from phase `min_phase` on are scored.
    dp_lo, dp_hi = turn_rate_depower_range(fcs.body_damping)
    coeffs = Dict{Float64, Any}()
    tc_at(d) = get!(coeffs, round(clamp(d, dp_lo, dp_hi), digits = 3)) do
        turn_rate_coeffs(fcs.body_damping, round(clamp(d, dp_lo, dp_hi), digits = 3))
    end
    ωz, ωp = 2π * KITE_CORR_ZERO, 2π * KITE_CORR_POLE

    x_tape = u_tape_log[1]
    hist = zeros(n)                   # the tape position, the dead time's input
    x_kite = u_tape_log[1]
    x_corr = u_tape_log[1]
    drive = zeros(n)                  # c1·v_a·u_k [rad/s]
    grav = zeros(n)                   # c2/v_a·cos(β) [rad/s], times sin(ψ)
    for k in 1:n
        x_tape = variant == :model ? x_tape + Ts / T_act * (u_cmd[k] - x_tape) : u_tape_log[k]
        hist[k] = x_tape
        tc = tc_at(dp[k])
        nd = round(Int, kite_dead_time(tc, v_a[k]) / Ts)
        u_del = hist[max(k - nd, 1)]
        T_k = kite_lag(tc, v_a[k])
        x_kite += Ts / max(T_k, Ts) * (u_del - x_kite)
        u_k = x_kite
        if variant == :kite_corr
            x_corr += Ts * ωp * (x_kite - x_corr)
            u_k = ωp / ωz * x_kite + (1 - ωp / ωz) * x_corr
        end
        drive[k] = tc.c1 * v_a[k] * u_k
        grav[k] = C3 * cos(β[k])
    end
    rate_log = vcat(diff(ψ) ./ Ts, NaN)          # forward difference, aligned to t[k]
    rate_model = drive .+ grav .* sin.(ψ)
    phase = Int.(sl.sys_state)
    k0 = findfirst(>=(min_phase), phase)
    isnothing(k0) && error("replay: phase $min_phase never reached in $dir")
    r = k0:(n - 1)
    return (; log, variant, Ts, t = t[r], v_a = v_a[r], depower = dp[r], rate_log = rate_log[r],
              rate_model = rate_model[r], ψ = ψ[r], u_cmd = u_cmd[r], drive = drive[r], grav = grav[r],
              phase = phase[r])
end

"""
    heading_errors(rp, H) -> (errors, changes)

For replay `rp`, start the model heading from the logged one at every
multiple of `H` [s] and integrate `ψ̇ = drive + grav·sin(ψ)` over `H`: the
heading error at the end of each horizon [rad] and the logged heading change
over it [rad], with the `v_a` at the horizon's start for binning.
"""
function heading_errors(rp, H)
    N = round(Int, H / rp.Ts)
    errs, chgs, vas = Float64[], Float64[], Float64[]
    for k0 in 1:N:(length(rp.t) - N)
        ψm = rp.ψ[k0]
        for k in k0:(k0 + N - 1)
            ψm += rp.Ts * (rp.drive[k] + rp.grav[k] * sin(ψm))
        end
        push!(errs, ψm - rp.ψ[k0 + N]); push!(chgs, rp.ψ[k0 + N] - rp.ψ[k0]); push!(vas, rp.v_a[k0])
    end
    return errs, chgs, vas
end

"Normalized cross-correlation of `a` and `b` at lags `lags` [samples], `b` lagging `a` for positive lags."
function xcorr(a, b, lags)
    a = a .- mean(a); b = b .- mean(b)
    s = sqrt(sum(abs2, a) * sum(abs2, b))
    return [sum(a[max(1, 1 - L):min(end, end - L)] .* b[max(1, 1 + L):min(end, end + L)]) / s for L in lags]
end

"""
    v3_report(logs = V3_LOGS; bins = 10:5:45, Hs = (1.0, 2.0, 3.0), min_samples = 2000, io = stdout)

Replay every log in all three variants and print, per `v_a` bin: the
turn-rate VAF of each variant, the relative heading error after each `H`
(RMS error over RMS logged change, `:model` variant), and the lag [s] and
height of the peak of the residual/command cross-correlation (`:model`).
Bins with fewer than `min_samples` samples are skipped. Returns the rows.
"""
function v3_report(logs = V3_LOGS; bins = 10:5:45, Hs = (1.0, 2.0, 3.0), min_samples = 2000, io = stdout)
    reps = Dict(v => [replay(l; variant = v) for l in logs] for v in (:model, :kite, :kite_corr))
    cat(v, f) = reduce(vcat, [getfield(r, f) for r in reps[v]])
    va = cat(:model, :v_a)
    rows = NamedTuple[]
    @printf(io, "V3: %d logs, %d samples from phase 3 on\n", length(logs), length(va))
    @printf(io, "  v_a bin     samples | VAF model / kite / kite_corr | heading err after %s s | xcorr peak\n",
            join(Int.(Hs), ", "))
    herr = Dict(H => [heading_errors(r, H) for r in reps[:model]] for H in Hs)
    for lo in bins[1:end-1]
        hi = lo + step(bins)
        sel = (va .>= lo) .& (va .< hi)
        count(sel) < min_samples && continue
        vafs = map((:model, :kite, :kite_corr)) do v
            rl, rm = cat(v, :rate_log)[sel], cat(v, :rate_model)[sel]
            ok = isfinite.(rl) .& isfinite.(rm)
            1 - var(rl[ok] .- rm[ok]) / var(rl[ok])
        end
        hes = map(Hs) do H
            e = reduce(vcat, [x[1][(x[3] .>= lo) .& (x[3] .< hi)] for x in herr[H]])
            c = reduce(vcat, [x[2][(x[3] .>= lo) .& (x[3] .< hi)] for x in herr[H]])
            isempty(e) ? NaN : sqrt(mean(abs2, e)) / sqrt(mean(abs2, c))
        end
        # residual/command cross-correlation, per log and bin, pooled over the logs by averaging
        lags = -60:60
        xcs = [begin
                   s = (r.v_a .>= lo) .& (r.v_a .< hi) .& isfinite.(r.rate_log)
                   count(s) < 500 ? nothing : xcorr(r.u_cmd[s], (r.rate_log .- r.rate_model)[s], lags)
               end for r in reps[:model]]
        xcs = filter(!isnothing, xcs)
        xc = isempty(xcs) ? fill(NaN, length(lags)) : mean(xcs)
        ipk = argmax(abs.(xc))
        Ts = reps[:model][1].Ts
        push!(rows, (; bin = (lo, hi), n = count(sel), vaf = vafs, heading = hes,
                       xcorr_lag_s = lags[ipk] * Ts, xcorr_peak = xc[ipk]))
        @printf(io, "  %2d – %2d m/s %8d | %5.2f / %5.2f / %5.2f | %s | %+.2f s (%.2f)\n",
                lo, hi, count(sel), vafs..., join((@sprintf("%.2f", h) for h in hes), "  "),
                lags[ipk] * Ts, xc[ipk])
    end
    return rows
end
