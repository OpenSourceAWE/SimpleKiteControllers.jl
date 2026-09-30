# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Worst stability margins of the archived reel-out scenarios with the turn-rate
coefficients of the low crosswind pattern (`PlanIdentifyTurnRateLaw.md`), against
the current model:

- **A**: as Table 6 of the paper and `stability_overview.md`: the table's `c1`,
  the constant gravity coefficient `C3` = 0.23 1/s, the pattern-law delay.
- **B**: the plant's `c1(u_d)` and gravity term `c2(u_d)/v_a` from the low flights,
  the pattern-law delay.
- **C**: B with the low flights' dead time and lag, one pair per depower and
  independent of `v_a`, instead of the pattern law. Optimistic: the worst bins sit
  at lower `v_a` than most of the low flights.

The controller's gain schedule keeps the table's `c1` in all three: that is what
was flown. The low-flight coefficients are interpolated linearly over depower
from `output/turn_rate_low_flights.csv` (unpacked from
`data/turn_rate_low_flights.tar.gz` when missing).

The scenarios whose phase-4 `v_a` drops below `va_min` are left out; with the
scenarios of 2026-09-29 these are Cabauw 3 m/s and Maasvlakte 3.5 and 4 m/s, and
`run_example("stability_new_coeffs.jl"; include_low_va = true)` rates them too.

B is the model's own since 2026-09-29 (`plant_coeffs` in `course_loop_model.jl`),
so B equals what `stability_global.jl` reports. The bins are collected once with
`retune_guided.jl`'s `collect_scenarios`; all three variants redefine
`reelout_margins` in `Main`, which `bin_margins` calls. A later include of
`stability_opt_reelout.jl` restores the original. Writes
`output/stability_new_coeffs.csv`. About 3 minutes.

    include("stability_new_coeffs.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Printf
using DelimitedFiles: readdlm, writedlm

va_min = 12.0             # [m/s] scenarios whose phase-4 v_a drops below this are left out
# The input `include_low_va = true` rates them anyway; read before anything else runs a script.
include(joinpath(@__DIR__, "script_inputs.jl"))
(; include_low_va) = script_inputs(@__FILE__, (; include_low_va = false))

# collect_scenarios, scenario_margins, live_settings; and through it stability_opt_reelout.jl.
include(joinpath(@__DIR__, "retune_guided.jl"))

snc_output = normpath(joinpath(@__DIR__, "..", "output"))
snc_summary = joinpath(snc_output, "turn_rate_low_flights.csv")
if !isfile(snc_summary)
    archive = normpath(joinpath(@__DIR__, "..", "data", "turn_rate_low_flights.tar.gz"))
    isdir(joinpath(snc_output, "turn_rate_low_flights")) &&
        error("Only part of the low-flight results is in $snc_output; move it away to unpack $archive.")
    run(`tar -xzf $archive -C $snc_output`)
    @info "Unpacked $archive into $snc_output."
end
snc_low = let mh = readdlm(snc_summary, ','; header = true)
    m, h = mh
    c(name) = Float64.(m[:, findfirst(==(name), vec(h))])
    (; dp = c("depower"), c1 = c("c1"), c2 = c("c2"), dead = c("dead_time"), lag = c("lag"))
end

"Linear interpolation of `y` over the low-flight depowers, clamped at the ends"
function snc_interp(y, u)
    u = clamp(u, first(snc_low.dp), last(snc_low.dp))
    k = clamp(searchsortedlast(snc_low.dp, u), 1, length(snc_low.dp) - 1)
    w = (u - snc_low.dp[k]) / (snc_low.dp[k + 1] - snc_low.dp[k])
    return (1 - w) * y[k] + w * y[k + 1]
end

"Lowest `v_a` [m/s] of phase 4 in the scenario folder of `d`"
function phase4_va_min(d)
    dir = normpath(joinpath(@__DIR__, "..", "output", "scenarios", d.site, d.name))
    f = only(filter(endswith(".arrow"), readdir(dir)))
    sl = load_log(replace(f, ".arrow" => ""); path = dir).syslog
    return minimum(sl.v_app[sl.sys_state .== 4])
end

snc_all = collect_scenarios()
snc_va4 = [phase4_va_min(d) for d in snc_all]
snc_data = [d for (d, v) in zip(snc_all, snc_va4) if include_low_va || v >= va_min]
excluded = [@sprintf("%s/%s (%.1f m/s)", d.site, d.name, v) for (d, v) in zip(snc_all, snc_va4) if v < va_min]
isempty(excluded) || @info (include_low_va ? "Rated although phase-4 v_a < $va_min m/s: " :
                                             "Left out, phase-4 v_a < $va_min m/s: ") * join(excluded, ", ")

snc_settings = live_settings()

snc_variant = Ref(:A)
# `reelout_margins` of stability_opt_reelout.jl with the plant's coefficients chosen by
# `snc_variant`. B is the model's own since 2026-09-29 (`plant_coeffs`); A rebuilds the one before.
@eval Main function reelout_margins(L, v_app, ω_g, depower, el_c, lag; f = fcs, inner = true)
    dp = clamp(depower, DP_LO, DP_HI)
    tc = turn_rate_coeffs(f.body_damping, dp)
    K = C1_SETPOINT / tc.c1 * f.heading_p * f.v_app_ref / max(v_app, V_MIN_PATTERN)   # as flown: table c1
    C = course_pid(K, f.heading_i, f.heading_d, f.heading_d_n, Ts)
    G = guidance_tf(ω_g, Ts) * kite_correction(Ts)
    c1, c2 = snc_variant[] === :A ? (tc.c1, c2_at(v_app)) :   # table c1, C3
             values(plant_coeffs(depower))                     # the model's own, c2/v_a
    if snc_variant[] === :C
        τp = snc_interp(snc_low.dead, depower)
        Tp = snc_interp(snc_low.lag, depower)
    else
        τp, Tp = pattern_dead_time_lag(tc, v_app, dp)
    end
    function margins(Lp)
        dm = try
            diskmargin(Lp)
        catch
            nothing
        end
        dlm = with_logger(() -> delay_margin(Lp), NullLogger())
        (; L = Lp, dm, α = isnothing(dm) ? 0.0 : dm.margin, delay_margin = dlm)
    end
    results = [begin
                   Pp = turn_rate_plant(c1, c2, τp, v_app, gravity, Ts; lag, kite_lag = Tp)
                   # The same plant for both, as in stability_opt_reelout.jl; the inner loop without the guidance.
                   (; inner = inner ? margins(C * kite_correction(Ts) * Pp) : nothing, guided = margins(C * G * Pp))
               end for gravity in (-cosd(el_c), cosd(el_c))]
    worst_inner = inner ? argmin(r -> r.α, [r.inner for r in results]) : nothing
    guided = argmin(r -> r.α, [r.guided for r in results])
    return (; inner = worst_inner, guided, K, delay = τp)
end
snc_variant[] = :A
snc_A = [Base.invokelatest(scenario_margins, d, snc_settings; inner = true) for d in snc_data]
snc_variant[] = :B
snc_B = [Base.invokelatest(scenario_margins, d, snc_settings; inner = true) for d in snc_data]
snc_variant[] = :C
snc_C = [Base.invokelatest(scenario_margins, d, snc_settings; inner = true) for d in snc_data]

println("\nWorst margins per scenario: α inner, α guided, delay margin of the guided loop [s], L [m]")
@printf("%-18s %28s %28s %28s\n", "scenario", "A (Table 6)", "B (low c1, c2)", "C (B + low delay)")
rows = Vector{Vector{Any}}()
for (d, a, b, c) in zip(snc_data, snc_A, snc_B, snc_C)
    @printf("%-18s %6.3f %6.3f %6.3f %5.0f  %6.3f %6.3f %6.3f %5.0f  %6.3f %6.3f %6.3f %5.0f\n",
            "$(d.site)/$(d.name)", a.α_i, a.α_g, a.dm_g, a.L, b.α_i, b.α_g, b.dm_g, b.L,
            c.α_i, c.α_g, c.dm_g, c.L)
    push!(rows, [d.site, d.name, a.α_i, a.α_g, a.dm_g, a.L, b.α_i, b.α_g, b.dm_g, b.L,
                 c.α_i, c.α_g, c.dm_g, c.L])
end
open(joinpath(snc_output, "stability_new_coeffs.csv"), "w") do io
    writedlm(io, permutedims(["site", "scenario", "A_alpha_inner", "A_alpha_guided", "A_dm", "A_L",
                              "B_alpha_inner", "B_alpha_guided", "B_dm", "B_L",
                              "C_alpha_inner", "C_alpha_guided", "C_dm", "C_L"]), ',')
    writedlm(io, permutedims(reduce(hcat, rows)), ',')
end

nothing
