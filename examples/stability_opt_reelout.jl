# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Disk-based stability analysis of the course-control loop flown by
`simple_opt_reelout.jl`, over the full range of tether length, from the
project's `l_tether` to `reelout_l_max`.

The plant and the controller are those of `stability_fig8.jl`
(shared in `course_loop_model.jl`): the steering tape as a first-order lag, the
turn-rate law with the kite's dead time and lag scaled over `v_a`, and the exact
discrete PD of `CourseController`. The plant's `c1` and gravity term
`c2/v_a·sin(ψ)·cos(β)` are those of the low crosswind pattern
([`plant_coeffs`](@ref), since 2026-09-29; before, the table's `c1` and a constant
`C3`); the gain schedule keeps `data/turn_rate_coeffs.yaml`, as flown. Four things
differ in the reel-out:

- **The tape's lag.** `ACTUATOR_LAG` (0.43 s) is the tape's equivalent lag in
  the fig8 pattern, where it is rate-limited 20 % of the time. The reel-out
  steers less hard, and the lag is fitted on the log instead, once on every
  on-path sample off the rate limit ([`fit_actuator_lag`](@ref)); the column
  "bin lag" is each bin's own fit, for diagnosis only. Since
  the actuator model was sped up, the lag is much lower here: in phase 4 it
  is about 0.20 s at 6 m/s wind, the small-signal `1/steering_gain`, with the
  tape rate-limited ~4 % of the time; phase 5 steers harder (about 0.24 s,
  also ~4 % rate-limited), rising to 0.31 s in the last, largest-signal bin.

- **The kite's dead time and lag.** As in `stability_fig8.jl`: the loop with
  the guidance is the validated pattern model (oldplans/Plan_model_validation.md),
  with the kite's response time from the pattern law
  ([`pattern_dead_time_lag`](@ref), re-identified on pattern logs including
  this reel-out, 0.29 s at 12.8 m/s against the table's 0.43 s) and
  [`kite_correction`](@ref); since 2026-09-29 the inner loop `C·P·kite_correction`
  uses the same plant (before, the turn-rate table's dead time and lag). The
  log cannot split dead time from lag: closed-loop steering has no steps, and
  its own split read 0 s + 0.27 s. It checks their sum instead: the pure delay
  identified on settled phase 4 (`identify_turn_rate_law`) against the pattern
  law's and the table's at the same `v_a` and depower. Before V3Kite v1.4.1
  that delay was a cross-correlation, which ignored the gravity term and read
  0.067 s, too short. `kite_correction` was measured at `v_a` ≈ 34 m/s; at the
  reel-out's 11 – 20 m/s it is extrapolated.

- **The gain schedule.** `simple_opt_reelout.jl` rescales the gain by
  `gain_scale = c1(depower_setpoint)/c1(depower)` in every phase, so the loop
  gain `K·c1·v_a` is that of `depower_setpoint` at whatever depower is flown:

      K = gain_scale · heading_p · v_app_ref / max(v_a, v_app_min, v_app_min_pattern)

- **The tether length.** The inner loop does not see `L` directly (the
  turn-rate law is physical), but the guidance does: the attractor sits
  `D = attractor_distance(fcs, v_a, L)` of arc ahead of the closest point, so a
  cross-track error `d` (angular) commands `δχ_set = -d/D`, and the kite
  closes it at `ḋ = v_k/L · δψ`. Broken at the plant input, the loop with the
  guidance is

      L_g = C · (1 + ω_g/s) · P,   ω_g = v_k / (L · D)

  A lead time (`attractor_lead_time`) holds `L·D ≈ lead_time·v_a` and so
  `ω_g` about constant; once `D` hits its floor `attractor_dist`, `ω_g` falls
  as `1/L`, and once it hits the ceiling `2·attractor_dist`, it rises.

`L`, `v_a`, the kite speed, the depower and the pattern's centre elevation (the
gravity pole) are read from the last log of `simple_opt_reelout.jl` for the
selected project, `output/<log_file>_opt.arrow` (or the folder given as the input
`log_dir`, e.g. an archive): phases 3-5 while the kite is within `attractor_dist` of the path
(where `atan(d/D) ≈ d/D` holds; the approach from far off is not linear),
binned on tether length. Each bin is checked at its lowest, median and highest
`v_a`, each with the highest `ω_g` its samples within `WG_VA_BAND` of that
`v_a` fly (`ω_g ∝ v_k`, so the bin's highest `ω_g` is flown at its highest
`v_a`, not its lowest), and at its lowest and highest depower, both signs of
the gravity pole, and the worst case is reported. A bin the log does not cover is an error: the whole range must be
flown before it can be checked.

The curvature feed-forward (`u_ff`, `chi_ff`) acts outside the loop and does
not change its margins; its fades on the cross-track and course error are not
modelled. The disk margin `α` (skew 0) is the radius of the largest disk of
simultaneous gain and phase variations the loop tolerates; `α ≥ 0.5` is
considered robust.

    include("stability_opt_reelout.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using SimpleKiteControllers: project_file
using KiteUtils: Settings, set_data_path
using V3Kite: load_log, YAML, identify_turn_rate_law
using ControlSystemsBase, RobustAndOptimalControl, MakieControlPlots
using SimpleKiteControllers: ACTUATOR_LAG, C3, KITE_CORR_ZERO, KITE_CORR_POLE
using LinearAlgebra: norm
using Statistics: median
using Printf
using Base.CoreLogging: with_logger, NullLogger

set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
# The caller's inputs, `run_example("stability_opt_reelout.jl"; show_plots = false, ...)`; a plain
# `include` runs with these defaults. `project` replaces the menu's selection (used by
# `retune_guided.jl`), `log_dir` the folder the log is read from, and `gravity_scale` scales the
# plant's gravity term (0 leaves it out).
inputs = script_inputs(@__FILE__, (; show_plots = true, project = nothing, log_dir = nothing,
                                    gravity_scale = 1.0))
show_plots = inputs.show_plots

# system_reelout_*.yaml; a fig8 selection falls back to the default.
PROJECT = something(inputs.project, selected_reelout_project())
@assert PROJECT in ("system_reelout_cabauw.yaml", "system_reelout_maasvlakte.yaml") "stability_opt_reelout.jl \
    supports only system_reelout_cabauw.yaml and system_reelout_maasvlakte.yaml, got $PROJECT"
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
reload_turn_rate_table!(project)
SET = Settings(project)
Ts = 1 / SET.sample_freq

"Width of a tether-length bin [m]"
const BIN_M = 10.0
"""
Half-width [m/s] of the `v_a` band around a checked `v_a` corner whose samples
give that corner's `ω_g` (their highest). Pairing every corner with the bin's
highest `ω_g` instead combined the slowest `v_a` with the guidance corner of
the fastest part of the lap, which the kite never flies (2026-09-27: α 0.22
against 0.51 at 154 m, Maasvlakte 3.5 m/s).
"""
const WG_VA_BAND = 1.0
"""
Largest fraction of a bin's samples with the tape on its rate limit for the
bin to count as linear. Above it, the kite flies a large-signal manoeuvre
(the entry transient into lap 1), an equivalent lag is no longer a model of
the tape, and the bin is reported but left out of the rating.
"""
const MAX_RATE_LIMITED = 0.2
"Floor of the gain schedule from phase 3 on, as `calc_steering` applies it"
const V_MIN_PATTERN = max(fcs.v_app_min, fcs.v_app_min_pattern)
const DP_LO, DP_HI = turn_rate_depower_range(fcs.body_damping)
"The turn authority the loop was tuned at, as `simple_opt_reelout.jl` computes it"
const C1_SETPOINT = turn_rate_coeffs(fcs.body_damping, fcs.depower_setpoint).c1
# The plant's turn-rate law is `c1(u_d)·v_a·u_s + c2(u_d)/v_a·sin(ψ)·cos(β)`, identified in the low
# crosswind pattern (`plant_coeffs`, course_loop_model.jl); the controller keeps the table's c1, as
# flown. The input `gravity_scale` scales the gravity term, 0 leaves it out.
const GRAVITY_EVAL = Float64(inputs.gravity_scale)

# ---- The flown operating points ------------------------------------------ #
# The input `log_dir` (e.g. an `output/archives/<stamp>` folder) replaces `output/`.
output_path = something(inputs.log_dir, normpath(joinpath(@__DIR__, "..", "output")))
log_name = basename(SET.log_file) * "_opt"
isfile(joinpath(output_path, log_name * ".arrow")) ||
    error("No $log_name.arrow in $output_path; run simple_opt_reelout.jl for $PROJECT first.")
summary_file = joinpath(output_path, log_name * ".yaml")
if isfile(summary_file)
    sim = get(YAML.load_file(summary_file), "simulation", Dict())
    flown = get(sim, "project", PROJECT)
    flown == PROJECT ||
        @warn "$log_name.arrow was flown with $flown, not $PROJECT; its operating points may not match."
    @info "Operating points from $log_name.arrow: $(get(sim, "date", "?")) $(get(sim, "time", "?")), \
           wind $(get(sim, "wind_speed", "?")) m/s, git $(get(sim, "git_hash", "?"))."
end
sl = load_log(log_name; path = output_path).syslog
# On the path only: the guidance is linear for a cross-track error below the attractor's arc distance.
in_pattern = findall(i -> 3 <= sl.sys_state[i] <= 5 && sl.var_01[i] <= fcs.attractor_dist,
                     eachindex(sl.sys_state))
isempty(in_pattern) && error("$log_name.arrow never reached the path in phases 3-5.")
log_L = [Float64(sl.l_tether[i][1]) for i in in_pattern]
log_va = Float64.(sl.v_app[in_pattern])
# Tangential kite speed: the reel-out speed moves the kite radially, not across the sphere.
log_vk = [sqrt(max(norm(sl.vel_kite[i])^2 - Float64(sl.v_reelout[i][1])^2, 0.0)) for i in in_pattern]
log_dp = Float64.(sl.depower[in_pattern])
log_elc = Float64.(sl.var_04[in_pattern])

"""
    fit_actuator_lag(sl, idx) -> NamedTuple

Equivalent first-order lag [s] of the steering tape, `set_steering` ->
`steering`, least-squares fitted on the log samples `idx`: `ẏ = (u - y)/T`.
Also returns the fraction of those samples on the tape's rate limit and the
fit's unexplained variance of `ẏ`.
"""
function fit_actuator_lag(sl, idx)
    idx = filter(k -> k < length(sl.time), idx)
    u, y, t = Float64.(sl.set_steering), Float64.(sl.steering), sl.time
    dy = [(y[k + 1] - y[k]) / (t[k + 1] - t[k]) for k in idx]
    e = [u[k] - y[k] for k in idx]
    a = sum(dy .* e) / sum(abs2, e)
    return (; T = 1 / a, rate_limited = count(x -> abs(x) > 0.975 * SET.v_steering, dy) / length(dy),
            unexplained = sum(abs2, dy .- a .* e) / sum(abs2, dy))
end

# The lag is the tape's, not the operating point's: one fit on every on-path sample off the rate
# limit. A quiet bin's own fit explains little of ẏ and reads the lag high (2026-09-27, Cabauw
# 10 m/s at 355 m: 2.25 s at 99.8 % unexplained, against 0.09 s in the clean bins).
off_limit = filter(in_pattern) do k
    k < length(sl.time) || return false
    rate = (Float64(sl.steering[k + 1]) - Float64(sl.steering[k])) / (sl.time[k + 1] - sl.time[k])
    abs(rate) <= 0.975 * SET.v_steering
end
tape_lag = fit_actuator_lag(sl, off_limit)

# Cross-check of the table's dead time + lag: the pure delay identified on settled phase 4 (from 10 s
# after it starts), against the table's sum at the median v_a and depower there. Closed-loop steering
# has no steps, so the log gives the sum only; its own split read 0 s + 0.27 s on 2026-09-26.
# On the log's own sample time: a compressed scenario log keeps every 3rd row, and Ts would scale the delay by 1/3.
dt_log = median(diff(Float64.(sl.time)))
let p4 = findall(==(4), sl.sys_state)
    length(p4) * dt_log > 20 || error("$log_name.arrow flies less than 20 s of phase 4; too short to identify the kite's dead time.")
    i1, i2 = p4[1] + round(Int, 10 / dt_log), p4[end]
    local id = identify_turn_rate_law(sl[i1:i2]; dt = dt_log)
    global τ_log, τ_corr, v_log = id.delay_sec, id.delay_corr, median(Float64.(sl.v_app[i1:i2]))
    local tc = turn_rate_coeffs(fcs.body_damping, clamp(median(Float64.(sl.depower[i1:i2])), DP_LO, DP_HI))
    global τ_table, T_table = kite_dead_time(tc, v_log), kite_lag(tc, v_log)
    global τ_pat, T_pat = pattern_dead_time_lag(tc, v_log, clamp(median(Float64.(sl.depower[i1:i2])), DP_LO, DP_HI))
end

# Per sample: v_k/v_a changes along the lap, and a bin holds less than one lap.
log_ωg = guidance_rate.(Ref(fcs), log_va, log_L, log_vk)

"""
    reelout_margins(L, v_app, ω_g, depower, el_c, lag; f = fcs, inner = true) -> NamedTuple

Disk and delay margins of the inner loop `C·P·kite_correction` and of the pattern
loop `C·(1 + ω_g/s)·P·kite_correction` at one operating point, both with the same
plant `P`: the kite's dead time and lag of the pattern law
([`pattern_dead_time_lag`](@ref)); until 2026-09-29 the inner loop took the turn-rate
table's and no `kite_correction`: tether length `L` [m],
`v_app` [m/s], the guidance's corner `ω_g` [rad/s] (see [`guidance_rate`](@ref)),
`depower` [-] (clamped to the turn-rate table's
range, as the gain schedule is), the pattern's centre elevation `el_c` [deg]
and the tape's lag `lag` [s]. Worst case over the sign of the gravity pole, whose
size is `c2(u_d)/v_a·cos(el_c)` ([`plant_coeffs`](@ref)) times `GRAVITY_EVAL`. The controller comes from the
settings `f`; `inner = false` skips the inner loop (its field is then `nothing`).
"""
function reelout_margins(L, v_app, ω_g, depower, el_c, lag; f = fcs, inner = true)
    tc = turn_rate_coeffs(f.body_damping, clamp(depower, DP_LO, DP_HI))
    K = C1_SETPOINT / tc.c1 * f.heading_p * f.v_app_ref / max(v_app, V_MIN_PATTERN)
    C = course_pid(K, f.heading_i, f.heading_d, f.heading_d_n, Ts)
    G = guidance_tf(ω_g, Ts) * kite_correction(Ts)
    τp, Tp = pattern_dead_time_lag(tc, v_app, clamp(depower, DP_LO, DP_HI))
    function margins(Lp)
        dm = try
            diskmargin(Lp)
        catch
            nothing
        end
        # The guidance's pole at z = 1 makes `margin` evaluate L there, where |L| = Inf is right; mute its warning.
        dlm = with_logger(() -> delay_margin(Lp), NullLogger())
        (; L = Lp, dm, α = isnothing(dm) ? 0.0 : dm.margin, delay_margin = dlm)
    end
    # The plant's c1 and c2 from the low pattern; K above keeps the table's c1, as flown.
    pc = plant_coeffs(depower)
    c2 = GRAVITY_EVAL * pc.c2
    results = [begin
                   Pp = turn_rate_plant(pc.c1, c2, τp, v_app, gravity, Ts; lag, kite_lag = Tp)
                   # The same plant for both; the inner loop without the guidance.
                   (; inner = inner ? margins(C * kite_correction(Ts) * Pp) : nothing, guided = margins(C * G * Pp))
               end for gravity in (-cosd(el_c), cosd(el_c))]
    worst_inner = inner ? argmin(r -> r.α, [r.inner for r in results]) : nothing
    guided = argmin(r -> r.α, [r.guided for r in results])
    return (; inner = worst_inner, guided, K, delay = τp)
end

"""
    bin_margins(s, lag; f = fcs, inner = true) -> NamedTuple

Worst case of one tether-length bin with the log samples `s = (; L, va, vk, dp, elc)`
(vectors) and the tape's lag `lag` [s], for the settings `f`: checked at the bin's
lowest, median and highest `v_a`, each with the highest `ω_g` its samples within
`WG_VA_BAND` of that `v_a` fly, and at its lowest and highest depower. Returns the
bin's median length `L`, all corners `evals`, and the worst corner of the inner loop
`wi` (`nothing` if `inner = false`) and of the guided loop `wg`.
"""
function bin_margins(s, lag; f = fcs, inner = true)
    L_mid = median(s.L)
    ωg = guidance_rate.(Ref(f), s.va, s.L, s.vk)
    ωg_at(va) = maximum(ωg[abs.(s.va .- va) .<= WG_VA_BAND])
    el_c = median(s.elc)
    evals = [(; va, dp, ωg = ωg_at(va),
              m = reelout_margins(L_mid, va, ωg_at(va), dp, el_c, lag; f, inner))
             for va in unique([minimum(s.va), median(s.va), maximum(s.va)])
             for dp in unique(extrema(s.dp))]
    wi = inner ? argmin(e -> e.m.inner.α, evals) : nothing
    wg = argmin(e -> e.m.guided.α, evals)
    return (; L = L_mid, evals, wi, wg)
end

@info @sprintf("Reel-out course-loop stability, project %s, body_damping = %s, dt = %.4f s, \
                heading_p = %.4f, heading_d = %.3f s, heading_d_n = %.1f, heading_i = %s, \
                depower_setpoint = %.3f (c1 = %.4f), v_app_min = %.1f m/s, v_app_min_pattern = %.1f m/s, \
                attractor_dist = %.1f°, attractor_lead_time = %.2f s, actuator lag %.3f s fitted on the whole log \
                (unexplained %.0f %%), \
                guided loop: kite correction %.2f/%.2f Hz, kite dead time + lag from the pattern law \
                %.3f + %.3f = %.3f s at %.1f m/s (turn-rate table: %.3f + %.3f = %.3f s), \
                against the log's pure delay %.3f s there (correlation %.3f); plant c1, c2 from the low pattern, \
                c2 = %.2f at depower_setpoint (gravity scale %.1f).",
               PROJECT, fcs.body_damping, Ts, fcs.heading_p, fcs.heading_d, fcs.heading_d_n,
               fcs.heading_i, fcs.depower_setpoint, C1_SETPOINT, fcs.v_app_min,
               fcs.v_app_min_pattern, fcs.attractor_dist, fcs.attractor_lead_time, tape_lag.T,
               100 * tape_lag.unexplained, KITE_CORR_ZERO, KITE_CORR_POLE,
               τ_pat, T_pat, τ_pat + T_pat, v_log, τ_table, T_table, τ_table + T_table, τ_log, τ_corr,
               plant_coeffs(fcs.depower_setpoint).c2, GRAVITY_EVAL)

l_lo, l_hi = SET.l_tether, fcs.reelout_l_max
edges = collect(range(l_lo, l_hi; length = max(ceil(Int, (l_hi - l_lo) / BIN_M), 1) + 1))
println(@sprintf("Phases 3-5 over tether length, %.0f – %.0f m in %d bins; worst case per bin over \
                  v_a (min, median, max) and depower (min, max), each v_a at the highest ω_g flown near it:", l_lo, l_hi, length(edges) - 1))
println("  L [m]          n   v_a [m/s]    depower        D [°]  ω_g [1/s]    bin lag  K      ",
        "α inner          α guided         DM guided")
rows = NamedTuple[]
uncovered = Tuple{Float64, Float64}[]
for b in 1:length(edges) - 1
    local lo, hi = edges[b], edges[b + 1]
    local idx = findall(l -> lo <= l < hi || (b == length(edges) - 1 && l == hi), log_L)
    if isempty(idx)
        push!(uncovered, (lo, hi))
        println(@sprintf("  %5.0f-%-5.0f    0   not flown in the log", lo, hi))
        continue
    end
    local samples = (; L = log_L[idx], va = log_va[idx], vk = log_vk[idx], dp = log_dp[idx], elc = log_elc[idx])
    local vas = log_va[idx]
    local D = attractor_distance(fcs, median(vas), median(log_L[idx]))
    local tape = fit_actuator_lag(sl, in_pattern[idx])
    # A bin spanning more than one phase (e.g. the phase 4 -> 5 handover, where depower ramps
    # from depower_setpoint to depower_final within the same tether-length bin) is not one
    # operating point: fit_actuator_lag's single first-order lag can fit it very poorly (seen:
    # 55 % unexplained variance, T = 0.85 s against 0.09 - 0.28 s in every phase-pure bin) and
    # the fitted T is not a model of the tape, so such a bin is excluded like a rate-limited one.
    local phases = unique(sl.sys_state[in_pattern[idx]])
    local mixed_phases = length(phases) > 1
    local bm = bin_margins(samples, tape_lag.T)
    local L_mid, wi, wg = bm.L, bm.wi, bm.wg
    f0(r) = isnothing(r.dm) ? NaN : r.dm.ω0 / 2π
    local note = tape.rate_limited > MAX_RATE_LIMITED ?
                 @sprintf("  large signal: tape rate-limited %.0f %%", 100 * tape.rate_limited) :
                 mixed_phases ?
                 @sprintf("  phase handover: bin spans phases %s, actuator-lag fit unexplained %.0f %%",
                          join(sort(phases), "+"), 100 * tape.unexplained) : ""
    println(@sprintf("  %5.0f-%-5.0f %5d  %4.1f – %4.1f  %.3f – %.3f  %5.2f  %4.2f – %4.2f  %5.3f    %.3f  %5.3f at %4.2f Hz  %5.3f at %4.2f Hz  %5.3f s%s",
                     lo, hi, length(idx), extrema(vas)..., extrema(log_dp[idx])...,
                     D, extrema(log_ωg[idx])..., tape.T, wg.m.K, wi.m.inner.α, f0(wi.m.inner),
                     wg.m.guided.α, f0(wg.m.guided), wg.m.guided.delay_margin, note))
    push!(rows, (; L = L_mid, α_inner = wi.m.inner.α, α_guided = wg.m.guided.α,
                 dm_guided = wg.m.guided.delay_margin, ω_g = wg.ωg, lag = tape_lag.T,
                 loop = wg.m.guided.L, va = wg.va, dp = wg.dp,
                 rate_limited = tape.rate_limited, mixed_phases = mixed_phases, samples,
                 linear = tape.rate_limited <= MAX_RATE_LIMITED && !mixed_phases))
end
any(dp -> !(DP_LO <= dp <= DP_HI), log_dp) &&
    @warn @sprintf("The log flies depower %.3f – %.3f, outside the turn-rate table's %.3f – %.3f; \
                    clamped to it, as the gain schedule is.", extrema(log_dp)..., DP_LO, DP_HI)
isempty(rows) && error("No tether-length bin is covered by $log_name.arrow.")

# Rated on the linear bins only; a large-signal bin (rate-limited) or a bin spanning a phase
# handover is a transient, not one operating point, see MAX_RATE_LIMITED.
lin_rows = filter(r -> r.linear, rows)
isempty(lin_rows) && error("No tether-length bin of $log_name.arrow flies the tape in its linear range.")
large = filter(r -> !r.linear && r.rate_limited > MAX_RATE_LIMITED, rows)
isempty(large) || @warn @sprintf("Not rated: %d bin(s) at L = %s m fly the tape on its rate limit more than \
                                  %.0f %% of the time (a large-signal transient, not a linear loop).",
                                 length(large), join((@sprintf("%.0f", r.L) for r in large), ", "),
                                 100 * MAX_RATE_LIMITED)
handover = filter(r -> !r.linear && r.mixed_phases, rows)
isempty(handover) || @warn @sprintf("Not rated: %d bin(s) at L = %s m span a phase handover (e.g. phase 4 -> 5, \
                                     where depower ramps within the bin); the actuator-lag fit is not one \
                                     operating point, not a linear loop.",
                                    length(handover), join((@sprintf("%.0f", r.L) for r in handover), ", "))
rate_disk_margin("Inner loop", [r.α_inner for r in lin_rows])
α_min = rate_disk_margin("Loop with guidance", [r.α_guided for r in lin_rows])
if isempty(uncovered)
    @info @sprintf("Tether length: the log covers the full range %.0f – %.0f m.", l_lo, l_hi)
else
    @error "Tether length: not checked in $(length(uncovered)) of $(length(edges) - 1) bins, \
            $(join((@sprintf("%.0f-%.0f m", u...) for u in uncovered), ", ")); the log does not \
            fly the full range $(l_lo) – $(l_hi) m."
end

# The worst loop with the guidance, for `diskmargin(L)` and the plots.
worst = argmin(r -> r.α_guided, lin_rows)
L = worst.loop

if show_plots
    display(bode_plot(L; from = -2, to = log10(0.5 / Ts),
                      title = @sprintf("Guided course loop, worst case: L = %.0f m, v_app = %.1f m/s, depower = %.3f",
                                       worst.L, worst.va, worst.dp)))
    MakieControlPlots.plotx([r.L for r in rows], [r.α_inner for r in rows],
                            [r.α_guided for r in rows], [r.dm_guided for r in rows];
                            xlabel = "tether length [m]",
                            ylabels = ["α inner [-]", "α guided [-]", "delay margin [s]"],
                            title = "Reel-out course loop margins, worst case per length",
                            fig = "reelout_loop_margins", disp = true)
end
@info @sprintf("Type 'diskmargin(L)' for details on the worst guided loop (L = %.0f m).", worst.L)
nothing
