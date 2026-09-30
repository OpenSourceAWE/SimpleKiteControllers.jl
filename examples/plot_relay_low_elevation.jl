# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The relay excitation of the LOW crosswind identification, as a figure for the
paper: the counterpart of Fig. 3 (`steering_response.pdf`, V3Kite's
`steering_test_v3_plots.jl`), which shows the sweep at 73°.

One flight of `plot_turn_rate_identification.jl`, at one depower and one fixed
amplitude, with the same relay (`_run_turn_rate_sweep`): the band centred on
`±heading_center`, reversed at `±az_reverse` of azimuth and tilted to hold
`el_hold`. Three panels over `plot_span` seconds from `plot_start` after the
start of the fit window:

- the heading with the edges of the band, which moves with the reversals and the
  elevation hold (`band_center ± HEADING_OFFSET`);
- the elevation with `el_hold`;
- the commanded steering, the actual steering of the slew-limited actuator, and
  the actual steering shifted by the delay `identify_turn_rate_law` finds for this
  flight, the input of its fit.

The channels are saved to `output/relay_low_elevation.csv` (time [s], heading,
band_center, elevation [°], set_steering, steering, us_delayed [-], `NaN` outside
the fit window), so the figure can be redrawn without flying (`from_csv`). The
PDF goes to LearningControl's `figures/`, as in `plot_c1_c2.jl`.

About two to three minutes of flying. The inputs `depower` (default 0.275) and
`from_csv` are passed with `run_example` (`examples/script_inputs.jl`):

    include("plot_relay_low_elevation.jl")                          # fly, save, plot
    run_example("plot_relay_low_elevation.jl"; from_csv = true)     # plot the saved flight
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using MakieControlPlots
using LaTeXStrings
using DelimitedFiles: readdlm, writedlm
include(joinpath(@__DIR__, "script_inputs.jl"))

# ==================== USER PARAMETERS ==================== #

(; depower, from_csv) = script_inputs(@__FILE__, (; depower = 0.275, from_csv = false))
depower = Float64(depower)

# The middle flight of `plot_turn_rate_identification.jl`'s `flight_settings`, which flies the
# full time at every depower from 0.25 to 0.325 (2026-09-29).
amplitude = 0.10         # [-] fixed steering amplitude
az_reverse = 30.0        # [°] azimuth at which the direction of flight reverses
el_hold_tilt = 45.0      # [°] limit of the band's tilt by the elevation hold
start_elevation = 30.0   # [°] as in plot_turn_rate_identification.jl
heading_center = 90.0    # [°]
el_hold = 30.0           # [°]
elevation_floor = 10.0   # [°]
max_elevation = 55.0     # [°] the fit window starts at the first sample below this

plot_start = 20.0        # [s] start of the plotted span, after the start of the fit window
plot_span = 40.0         # [s] length of the plotted span

csv_file = normpath(joinpath(@__DIR__, "..", "output", "relay_low_elevation.csv"))
fig_dir = normpath(joinpath(@__DIR__, "..", "..", "LearningControl", "figures"))
fig_name = "steering_response_low"
csv_header = ["time", "heading", "band_center", "elevation", "set_steering", "steering", "us_delayed"]

# ======================== FLIGHT ========================= #

if !from_csv
    # `_run_turn_rate_sweep`, `DT`, `T_START`, `HEADING_OFFSET`, `MIN_STEERING_FIT`.
    include(joinpath(@__DIR__, "build_turn_rate_table.jl"))
    r = _run_turn_rate_sweep(depower; max_steering_cap = 1.0, elevation_floor,
                             elevation = start_elevation, heading_center,
                             start_steering = amplitude, steering_step = 0.0, az_reverse, el_hold,
                             el_hold_tilt)
    r.outcome == :time_limit ||
        @warn "The flight ended with $(r.outcome), not after the full time; plotting what was flown."
    # The logger is preallocated; a flight that ends early leaves zero rows behind.
    sl = r.sl[1:findlast(>(0), r.sl.time)]
    el = rad2deg.(sl.elevation)
    k_below = findfirst(i -> sl.time[i] >= T_START && el[i] < max_elevation, eachindex(sl.time))
    isnothing(k_below) && error("The flight never came below max_elevation = $max_elevation° after T_START.")
    fit = identify_turn_rate_law(sl; dt = DT, t_start = sl.time[k_below], min_steering = MIN_STEERING_FIT)

    # The band centre of the step each sample was logged after, NaN before the relay starts.
    band = [(k = searchsortedlast(r.band_time, t + DT / 2); k == 0 ? NaN : r.band_center[k])
            for t in sl.time]
    us_delayed = fill(NaN, length(sl.time))
    us_delayed[[searchsortedfirst(sl.time, t - DT / 2) for t in fit.time]] .= fit.us_del

    mkpath(dirname(csv_file))
    open(csv_file, "w") do io
        writedlm(io, permutedims(csv_header), ',')
        writedlm(io, hcat(Float64.(sl.time), rad2deg.(wrap_to_pi.(sl.heading)), band, el,
                          Float64.(sl.set_steering), Float64.(sl.steering), us_delayed), ',')
    end
    @info "Saved the flight to $csv_file; fit window from $(round(sl.time[k_below]; digits = 1)) s, " *
          "delay $(round(fit.delay_sec; digits = 3)) s."
end

# ========================= PLOT ========================== #

isfile(csv_file) || error("$csv_file does not exist: fly first.")
data, header = readdlm(csv_file, ','; header = true)
col(name) = Float64.(data[:, findfirst(==(name), vec(header))])
time = col("time")
t_fit = time[findfirst(!isnan, col("us_delayed"))]
rng = findall(t -> t_fit + plot_start <= t <= t_fit + plot_start + plot_span, time)
isempty(rng) && error("Nothing to plot between $(t_fit + plot_start) and $(t_fit + plot_start + plot_span) s.")
# From zero, as in Fig. 3: the absolute simulation time says nothing here.
t = time[rng] .- time[first(rng)]
band = col("band_center")[rng]
# `HEADING_OFFSET` of build_turn_rate_table.jl, not loaded when replotting.
band_offset = 10.0

plotx(t,
      [col("heading")[rng], band .+ band_offset, band .- band_offset],
      [col("elevation")[rng], fill(el_hold, length(rng))],
      [100.0 .* col("set_steering")[rng], 100.0 .* col("steering")[rng], 100.0 .* col("us_delayed")[rng]];
      xlabel = L"\mathrm{time}~[\mathrm{s}]",
      ysize = 18,
      legendsize = 16,
      ylabels = [L"\psi~[°]", L"\beta~[°]", L"u_{\mathrm{s}}~[\%]"],
      labels = [
          [L"\psi", L"+\psi_\mathrm{band}", L"-\psi_\mathrm{band}"],
          [L"\beta", L"\beta_\mathrm{hold}"],
          [L"u_{\mathrm{s,set}}", L"u_{\mathrm{s}}", L"u_{\mathrm{s,delayed}}"],
      ],
      disp = true,
      fig = fig_name)
mkpath(fig_dir)
savefig(joinpath(fig_dir, fig_name * ".pdf"))

nothing
