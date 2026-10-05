# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The V2 deliverable of `oldplans/Plan_model_validation.md`: the course loop's
frequency response measured by injection in `simple_fig8.jl`, over the Bode
plot of `course_loop_model.jl`'s loop at the same operating point, one column
per tether length (150 / 200 / 300 m, 7 m/s of wind, depower 0.27).

- **measured**: `C · (command → course) · (1 + ω_g/s)`, the command → course
  points of `data/course_link_measured.csv` times the course PD at the run's
  `v_a` and the guidance, as `measured_loop` in `validate_margins.jl` forms it;
- **model**: the pattern model `stability_fig8.jl` uses (the pattern law's dead
  time and lag, `kite_correction`, `guidance_tf`).

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
using Statistics: mean
using Printf
using GLMakie

const FRF_POINTS = [
    (key = "F150", project = "system_fig8_150m.yaml", title = "150 m"),
    (key = "A", project = "system_fig8_200m.yaml", title = "200 m"),
    (key = "D", project = "system_fig8_300m.yaml", title = "300 m"),
]
const FRF_DEPOWER = 0.27
"Ratio of the kite's speed to its airspeed in the pattern, as in `stability_fig8.jl`"
const FRF_VK_OVER_VA = 0.96
"TeX Gyre Termes, the Times clone closest to the paper's font (the Copernicus class loads `times`)"
const FRF_FONTS = (; regular = "TeX Gyre Termes", bold = "TeX Gyre Termes Bold",
                   italic = "TeX Gyre Termes Italic")

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
    K = fcs.course.heading_p * fcs.course.v_app_ref / max(v_a, max(fcs.course.v_app_min, fcs.course.v_app_min_pattern))
    C = course_pid(K, fcs.course.heading_i, fcs.course.heading_d, fcs.course.heading_d_n, Ts)
    ω_g = guidance_rate(fcs, v_a, L_t, FRF_VK_OVER_VA * v_a)
    tc = turn_rate_coeffs(fcs.run.body_damping, FRF_DEPOWER)
    c2 = tc.c2
    gravity = -cosd(fcs.pattern.el_center)
    lag = 1 / set.steering_gain
    τp, Tp = pattern_dead_time_lag(tc, v_a, FRF_DEPOWER)
    Pp = turn_rate_plant(tc.c1, c2, τp, v_a, gravity, Ts; lag, kite_lag = Tp)
    model = C * Pp * guidance_tf(ω_g, Ts) * kite_correction(Ts, v_a)
    # Measured loop: the PD and the guidance applied to each measured line, as `measured_loop`.
    L_meas = [evalfr(C, cis(2π * fi * Ts))[1] * Gi * (1 + ω_g / (im * 2π * fi)) for (fi, Gi) in zip(f, G)]
    return (; f, L_meas, model, v_a, Ts)
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
    fig = Figure(size = (500, 300), fontsize = 9, fonts = FRF_FONTS, figure_padding = 4)
    fgrid = exp10.(range(log10(0.15), log10(4.5); length = 400))
    for (j, p) in enumerate(FRF_POINTS)
        r = frf_point(p)
        ticks = ([0.2, 0.5, 1.0, 2.0, 4.0], ["0.2", "0.5", "1", "2", "4"])
        title = rich(p.title, ", v", subscript("a"), @sprintf(" = %.1f m/s", r.v_a))
        ax1 = Axis(fig[1, j]; xscale = log10, xticks = ticks, title, titlefont = :regular,
                   ylabel = j == 1 ? rich("|L", subscript("g"), "| [dB]") : "")
        ax2 = Axis(fig[2, j]; xscale = log10, xticks = ticks, xlabel = "frequency [Hz]",
                   ylabel = j == 1 ? rich("phase of L", subscript("g"), " [deg]") : "", yticks = -540:90:0)
        linkxaxes!(ax1, ax2)
        x = _resp(r.model, fgrid, r.Ts)
        lines!(ax1, fgrid, _db(x); label = "model", linewidth = 1.2)
        lines!(ax2, fgrid, _phase(x); linewidth = 1.2)
        ph_model = _phase(_resp(r.model, r.f, r.Ts))
        # A measured phase is only known modulo 360°: put each point on the branch nearest the model's.
        ph_meas = rad2deg.(angle.(r.L_meas))
        ph_meas = [pm + 360 * round((pmod - pm) / 360) for (pm, pmod) in zip(ph_meas, ph_model)]
        scatter!(ax1, r.f, _db(r.L_meas); color = :black, markersize = 4, label = "measured (multisine injection)")
        scatter!(ax2, r.f, ph_meas; color = :black, markersize = 4)
        hlines!(ax1, [0.0]; color = :gray, linestyle = :dot)
        hlines!(ax2, [-180.0]; color = :gray, linestyle = :dot)
        xlims!(ax1, 0.15, 4.5)
        ylims!(ax1, -30, 25)
        ylims!(ax2, -450, 0)
    end
    Legend(fig[3, 1:length(FRF_POINTS)], content(fig[1, 1]); orientation = :horizontal, framevisible = false,
           padding = (0, 0, 0, 0), patchsize = (14, 8))
    rowgap!(fig.layout, 2, 4)
    GLMakie.save(file, fig; px_per_unit = 3)
    @info "plot_frf_validation: wrote $(normpath(file))"
    return fig
end

plot_frf_validation()
