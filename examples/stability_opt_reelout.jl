# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Disk-based stability analysis of the course-control loop flown by
`simple_opt_reelout.jl`, over the full range of tether length, from the
project's `l_tether` to `reelout_l_max`.

The plant and the controller are those of `stability_course_controller.jl`
(shared in `course_loop_model.jl`): the steering tape as a first-order lag, the
turn-rate law of `data/turn_rate_coeffs.yaml` with the kite's dead time scaled
over `v_a`, and the exact discrete PD of `CourseController`. Four things
differ in the reel-out:

- **The tape's lag.** `ACTUATOR_LAG` (0.43 s) is the tape's equivalent lag in
  the fig8 pattern, where it is rate-limited 20 % of the time. The reel-out
  steers less hard, and the lag is fitted on the log instead, per
  tether-length bin on the same samples ([`fit_actuator_lag`](@ref)). In
  phase 4 it is 0.33 s at 5 and 10 m/s, the small-signal `1/steering_gain`,
  with the tape rate-limited 0 – 5 % of the time; phase 5 steers harder
  (0.45 s at 5 m/s).

- **The kite's dead time.** `kite_delay` extrapolates the relay sweeps of
  `build_turn_rate_table.jl` (13 – 22.5 m/s) and the fig8 point (36 m/s) over
  `v_a`. The reel-out's own dead time is shorter at low wind: 0.067 s against
  the extrapolated 0.19 – 0.20 s at 24 – 26 m/s. It is identified on the log
  instead (`identify_turn_rate_law`, settled phase 4) and scaled over `v_a`
  with the same exponent, `τ = τ_log · (v_log / v_a)^KITE_DELAY_EXP`.

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
selected project, `output/<log_file>_opt.arrow` (or the folder `LOG_DIR`, e.g.
an archive): phases 3-5 while the kite is within `attractor_dist` of the path
(where `atan(d/D) ≈ d/D` holds; the approach from far off is not linear),
binned on tether length. Each bin is checked at its lowest, median and highest
`v_a` and at its
lowest and highest depower, both signs of the gravity pole, and the worst case
is reported. A bin the log does not cover is an error: the whole range must be
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
using LinearAlgebra: diagm, norm
using Statistics: median
using Printf
using Base.CoreLogging: with_logger, NullLogger

set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
show_plots = @isdefined(SHOW_PLOTS) ? SHOW_PLOTS : true
SHOW_PLOTS = true

PROJECT = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
@assert PROJECT in ("system_reelout_cabauw.yaml", "system_reelout_maasvlakte.yaml") "stability_opt_reelout.jl \
    supports only system_reelout_cabauw.yaml and system_reelout_maasvlakte.yaml, got $PROJECT"
project = project_file(PROJECT)
fcs = FC_Settings(fc_settings(project))
reload_turn_rate_table!(project)
SET = Settings(project)
Ts = 1 / SET.sample_freq

include(joinpath(@__DIR__, "course_loop_model.jl"))

"Width of a tether-length bin [m]"
const BIN_M = 10.0
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

# ---- The flown operating points ------------------------------------------ #
# LOG_DIR (e.g. an `output/archives/<stamp>` folder) replaces `output/`, read and cleared like SHOW_PLOTS.
output_path = (@isdefined(LOG_DIR) && !isnothing(LOG_DIR)) ? LOG_DIR :
              normpath(joinpath(@__DIR__, "..", "output"))
LOG_DIR = nothing
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

# The kite's dead time, identified on settled phase 4 (from 10 s after it starts) at the median v_a there.
# On the log's own sample time: a compressed scenario log keeps every 3rd row, and Ts would scale the delay by 1/3.
dt_log = median(diff(Float64.(sl.time)))
let p4 = findall(==(4), sl.sys_state)
    length(p4) * dt_log > 20 || error("$log_name.arrow flies less than 20 s of phase 4; too short to identify the kite's dead time.")
    i1, i2 = p4[1] + round(Int, 10 / dt_log), p4[end]
    local id = identify_turn_rate_law(sl[i1:i2]; dt = dt_log)
    global τ_log, v_log = id.delay_sec, median(Float64.(sl.v_app[i1:i2]))
    global τ_corr = id.delay_corr
end
"Dead time [s] from the applied steering to the turn rate at `v_app` [m/s], scaled from the log's own"
log_delay(v_app) = τ_log * (v_log / v_app)^KITE_DELAY_EXP

"Corner frequency [rad/s] of the guidance at tether length `L` [m], `v_app` and `v_kite` [m/s]"
guidance_rate(L, v_app, v_kite) = v_kite / (L * deg2rad(attractor_distance(fcs, v_app, L)))

# Per sample: v_k/v_a changes along the lap, and a bin holds less than one lap.
log_ωg = guidance_rate.(log_L, log_va, log_vk)

"""
    reelout_margins(L, v_app, ω_g, depower, el_c, lag) -> NamedTuple

Disk and delay margins of the inner loop `C·P` and of the loop with the
guidance `C·(1 + ω_g/s)·P` at one operating point: tether length `L` [m],
`v_app` [m/s], the guidance's corner `ω_g` [rad/s] (see [`guidance_rate`](@ref)),
`depower` [-] (clamped to the turn-rate table's
range, as the gain schedule is), the pattern's centre elevation `el_c` [deg]
and the tape's lag `lag` [s]. Worst case over the sign of the gravity pole.
"""
function reelout_margins(L, v_app, ω_g, depower, el_c, lag)
    tc = turn_rate_coeffs(fcs.body_damping, clamp(depower, DP_LO, DP_HI))
    K = C1_SETPOINT / tc.c1 * fcs.heading_p * fcs.v_app_ref / max(v_app, V_MIN_PATTERN)
    C = course_pid(K, fcs.heading_i, fcs.heading_d, fcs.heading_d_n, Ts)
    G = 1 + ω_g * Ts / (tf("z", Ts) - 1)
    τ = log_delay(v_app)
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
    results = map((-cosd(el_c), cosd(el_c))) do gravity
        P = turn_rate_plant(tc.c1, tc.c2, τ, v_app, gravity, Ts; lag)
        (; inner = margins(C * P), guided = margins(C * G * P))
    end
    inner = argmin(r -> r.α, first.(results))
    guided = argmin(r -> r.α, last.(results))
    return (; inner, guided, K, delay = τ)
end

@info @sprintf("Reel-out course-loop stability, project %s, body_damping = %s, dt = %.4f s, \
                heading_p = %.4f, heading_d = %.3f s, heading_d_n = %.1f, heading_i = %s, \
                depower_setpoint = %.3f (c1 = %.4f), v_app_min = %.1f m/s, v_app_min_pattern = %.1f m/s, \
                attractor_dist = %.1f°, attractor_lead_time = %.2f s, actuator lag fitted per bin on the log, kite dead time \
                %.3f s at %.1f m/s identified on the log (correlation %.3f).",
               PROJECT, fcs.body_damping, Ts, fcs.heading_p, fcs.heading_d, fcs.heading_d_n,
               fcs.heading_i, fcs.depower_setpoint, C1_SETPOINT, fcs.v_app_min,
               fcs.v_app_min_pattern, fcs.attractor_dist, fcs.attractor_lead_time, τ_log, v_log, τ_corr)

l_lo, l_hi = SET.l_tether, fcs.reelout_l_max
edges = collect(range(l_lo, l_hi; length = max(ceil(Int, (l_hi - l_lo) / BIN_M), 1) + 1))
println(@sprintf("Phases 3-5 over tether length, %.0f – %.0f m in %d bins; worst case per bin over \
                  v_a (min, median, max) and depower (min, max), at the bin's highest ω_g:", l_lo, l_hi, length(edges) - 1))
println("  L [m]          n   v_a [m/s]    depower        D [°]  ω_g [1/s]    lag [s]  K      ",
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
    local L_mid = median(log_L[idx])
    local vas = log_va[idx]
    local va_pts = unique([minimum(vas), median(vas), maximum(vas)])
    local ωg = maximum(log_ωg[idx])
    local D = attractor_distance(fcs, median(vas), median(log_L[idx]))
    local dp_pts = unique(extrema(log_dp[idx]))
    local el_c = median(log_elc[idx])
    local tape = fit_actuator_lag(sl, in_pattern[idx])
    local evals = [(; va, dp, m = reelout_margins(L_mid, va, ωg, dp, el_c, tape.T))
             for va in va_pts for dp in dp_pts]
    local wi = argmin(e -> e.m.inner.α, evals)
    local wg = argmin(e -> e.m.guided.α, evals)
    f0(r) = isnothing(r.dm) ? NaN : r.dm.ω0 / 2π
    println(@sprintf("  %5.0f-%-5.0f %5d  %4.1f – %4.1f  %.3f – %.3f  %5.2f  %4.2f – %4.2f  %5.3f    %.3f  %5.3f at %4.2f Hz  %5.3f at %4.2f Hz  %5.3f s%s",
                     lo, hi, length(idx), extrema(vas)..., extrema(log_dp[idx])...,
                     D, extrema(log_ωg[idx])..., tape.T, wg.m.K, wi.m.inner.α, f0(wi.m.inner),
                     wg.m.guided.α, f0(wg.m.guided), wg.m.guided.delay_margin,
                     tape.rate_limited > MAX_RATE_LIMITED ?
                     @sprintf("  large signal: tape rate-limited %.0f %%", 100 * tape.rate_limited) : ""))
    push!(rows, (; L = L_mid, α_inner = wi.m.inner.α, α_guided = wg.m.guided.α,
                 dm_guided = wg.m.guided.delay_margin, ω_g = ωg, lag = tape.T,
                 loop = wg.m.guided.L, va = wg.va, dp = wg.dp,
                 rate_limited = tape.rate_limited, linear = tape.rate_limited <= MAX_RATE_LIMITED))
end
any(dp -> !(DP_LO <= dp <= DP_HI), log_dp) &&
    @warn @sprintf("The log flies depower %.3f – %.3f, outside the turn-rate table's %.3f – %.3f; \
                    clamped to it, as the gain schedule is.", extrema(log_dp)..., DP_LO, DP_HI)
isempty(rows) && error("No tether-length bin is covered by $log_name.arrow.")

# Rated on the linear bins only; a large-signal bin is a transient, see MAX_RATE_LIMITED.
lin_rows = filter(r -> r.linear, rows)
isempty(lin_rows) && error("No tether-length bin of $log_name.arrow flies the tape in its linear range.")
large = filter(r -> !r.linear, rows)
isempty(large) || @warn @sprintf("Not rated: %d bin(s) at L = %s m fly the tape on its rate limit more than \
                                  %.0f %% of the time (a large-signal transient, not a linear loop).",
                                 length(large), join((@sprintf("%.0f", r.L) for r in large), ", "),
                                 100 * MAX_RATE_LIMITED)
rate("Inner loop", [r.α_inner for r in lin_rows])
α_min = rate("Loop with guidance", [r.α_guided for r in lin_rows])
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
