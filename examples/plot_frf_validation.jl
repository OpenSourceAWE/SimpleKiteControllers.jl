# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The V2 deliverable of docs/Plan_model_validation.md: the course loop's
frequency response measured by injection in `simple_fig8.jl`, over the Bode
plot of `course_loop_model.jl`'s loop at the same operating point, one column
per tether length (150 / 200 / 300 m, 7 m/s of wind, depower 0.27).

- **measured**: `C · (command → course) · (1 + ω_g/s)`, the command → course
  points of `data/course_link_measured.csv` times the course PD at the run's
  `v_a` and the guidance, as `measured_loop` in `validate_margins.jl` forms it;
- **model**: the pattern model `stability_fig8.jl` uses (the pattern law's dead
  time and lag, `kite_correction`, `guidance_tf`);
- **inner model**: `C · P` with the table's dead time and lag alone, the
  model before the validation.

The guidance corner is `ω_g = 0.96 · v_a / (L · D)` for both, `D` from
`attractor_distance`. Writes `docs/course_loop_frf.png`.

    include("examples/plot_frf_validation.jl")
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using SimpleKiteControllers: project_file, skc_data_path
using KiteUtils: Settings
using ControlSystemsBase
using LinearAlgebra: diagm
using Statistics: mean
using Printf
using GLMakie

include(joinpath(@__DIR__, "course_loop_model.jl"))

const FRF_POINTS = [
    (key = "F150_f8a42_f8b17", project = "system_fig8_150m.yaml", title = "150 m (pattern 42 × 17°)"),
    (key = "A", project = "system_fig8_200m.yaml", title = "200 m"),
    (key = "D", project = "system_fig8_300m.yaml", title = "300 m"),
]
const FRF_DEPOWER = 0.27
"Ratio of the kite's speed to its airspeed in the pattern, as in `stability_fig8.jl`"
const FRF_VK_OVER_VA = 0.96

"The measured command → course points of `key`: frequency [Hz], response, `v_a` [m/s]."
function read_course_link(key)
    rows = filter(l -> !startswith(l, "#") && !startswith(l, "point,"),
                  readlines(joinpath(skc_data_path(), "course_link_measured.csv")))
    rows = [split(l, ",") for l in rows if split(l, ",")[1] == key]
    f = [parse(Float64, r[3]) for r in rows]
    G = [complex(parse(Float64, r[4]), parse(Float64, r[5])) for r in rows]
    v_a = [parse(Float64, r[8]) for r in rows]
    return f, G, v_a
end

"""
    frf_point(p) -> NamedTuple

Measured and model loops of point `p` (an entry of `FRF_POINTS`), at the mean
`v_a` of its injection runs.
"""
function frf_point(p)
    f, G, v_as = read_course_link(p.key)
    project = project_file(p.project)
    fcs = FC_Settings(fc_settings(project))
    set = Settings(project)
    Ts = 1 / set.sample_freq
    L_t = set.l_tethers[1]
    v_a = mean(v_as)
    K = fcs.heading_p * fcs.v_app_ref / max(v_a, max(fcs.v_app_min, fcs.v_app_min_pattern))
    C = course_pid(K, fcs.heading_i, fcs.heading_d, fcs.heading_d_n, Ts)
    ω_g = FRF_VK_OVER_VA * v_a / (L_t * deg2rad(attractor_distance(fcs, v_a, L_t)))
    tc = turn_rate_coeffs(fcs.body_damping, FRF_DEPOWER)
    # c2 = 0 (the lower end of C2_BOUNDS): the table's gravity coefficient is not identified.
    c2 = first(C2_BOUNDS)
    gravity = -cosd(fcs.el_center)
    lag = 1 / set.steering_gain
    P = turn_rate_plant(tc.c1, c2, kite_dead_time(tc, v_a), v_a, gravity, Ts; lag,
                        kite_lag = kite_lag(tc, v_a))
    τp, Tp = pattern_dead_time_lag(tc, v_a, FRF_DEPOWER)
    Pp = turn_rate_plant(tc.c1, c2, τp, v_a, gravity, Ts; lag, kite_lag = Tp)
    model = C * Pp * guidance_tf(ω_g, Ts) * kite_correction(Ts)
    inner = C * P
    # Measured loop: the PD and the guidance applied to each measured line, as `measured_loop`.
    L_meas = [evalfr(C, cis(2π * fi * Ts))[1] * Gi * (1 + ω_g / (im * 2π * fi)) for (fi, Gi) in zip(f, G)]
    return (; f, L_meas, model, inner, v_a, Ts)
end

_resp(L, f, Ts) = [evalfr(L, cis(2π * fi * Ts))[1] for fi in f]
_db(x) = 20 .* log10.(abs.(x))
# Phase [deg], unwrapped along the frequency axis, its first point within (-360, 0].
function _phase(x)
    a = angle.(x)
    ph = rad2deg.(first(a) .+ cumsum(vcat(0.0, rem2pi.(diff(a), RoundNearest))))
    return ph .- 360 * ceil(ph[1] / 360)
end

"""
    plot_frf_validation(; file = "docs/course_loop_frf.png") -> Figure

Draw and save the figure (see the file's docstring).
"""
function plot_frf_validation(; file = joinpath(@__DIR__, "..", "docs", "course_loop_frf.png"))
    fig = Figure(size = (1500, 800), fontsize = 18)
    fgrid = exp10.(range(log10(0.15), log10(4.5); length = 400))
    for (j, p) in enumerate(FRF_POINTS)
        r = frf_point(p)
        ticks = ([0.2, 0.5, 1.0, 2.0, 4.0], ["0.2", "0.5", "1", "2", "4"])
        ax1 = Axis(fig[1, j]; xscale = log10, xticks = ticks, title = @sprintf("%s, v_a %.1f m/s", p.title, r.v_a),
                   ylabel = j == 1 ? "|L| [dB]" : "")
        ax2 = Axis(fig[2, j]; xscale = log10, xticks = ticks, xlabel = "frequency [Hz]",
                   ylabel = j == 1 ? "phase of L [deg]" : "", yticks = -540:90:0)
        linkxaxes!(ax1, ax2)
        for (L, label, style) in ((r.model, "model (pattern)", :solid), (r.inner, "inner model C·P (table)", :dash))
            x = _resp(L, fgrid, r.Ts)
            lines!(ax1, fgrid, _db(x); label, linestyle = style, linewidth = 2.5)
            lines!(ax2, fgrid, _phase(x); linestyle = style, linewidth = 2.5)
        end
        ph_model = _phase(_resp(r.model, r.f, r.Ts))
        # A measured phase is only known modulo 360°: put each point on the branch nearest the model's.
        ph_meas = rad2deg.(angle.(r.L_meas))
        ph_meas = [pm + 360 * round((pmod - pm) / 360) for (pm, pmod) in zip(ph_meas, ph_model)]
        scatter!(ax1, r.f, _db(r.L_meas); color = :black, markersize = 9, label = "measured (V2 injection)")
        scatter!(ax2, r.f, ph_meas; color = :black, markersize = 9)
        hlines!(ax1, [0.0]; color = :gray, linestyle = :dot)
        hlines!(ax2, [-180.0]; color = :gray, linestyle = :dot)
        xlims!(ax1, 0.15, 4.5)
        ylims!(ax1, -30, 25)
        ylims!(ax2, -450, 0)
        j == 3 && axislegend(ax1; position = :lb)
    end
    Label(fig[0, :], "Course loop L = C·P·(1 + ω_g/s): measured by injection (simple_fig8.jl) against course_loop_model.jl, 7 m/s, depower 0.27";
          fontsize = 20)
    GLMakie.save(file, fig)
    @info "plot_frf_validation: wrote $(normpath(file))"
    return fig
end

plot_frf_validation()
