# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Build docs/wind_scan_results.yaml from ARCHIVED reel-out runs, one per wind
# speed. It reads only archives — it flies nothing — so the runs themselves are
# made first, by editing `environment.v_wind` AND `environment.wind_vec` of
# data/settings_reelout_150m.yaml together (`use_wind_vec` is true, so v_wind
# alone does not move the plant's wind) and `include`ing
# examples/simple_opt_reelout.jl once per speed. Each run names its archive in
# output/last_run_done.txt.
#
#     include("examples/wind_scan.jl")
#     write_scan(scan_row.(WIND_SCAN_2026_08_19))
#
# The mean/peak/crest-factor triples for power, tether force and reel-out speed
# are all scored over the SAME window — the samples where the length setpoint was
# still growing (`reelout_power`), i.e. the actual reel-out phase. Force and power
# come from the run's own summary, the speed triple from its arrow log, because
# the summary has no speed triple.
using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using YAML, Statistics, Printf
using KiteUtils: load_log
using SimpleKiteControllers: reelout_power

const SKC_ROOT = normpath(joinpath(@__DIR__, ".."))

"The scan of 2026-08-19, flown on `8231f95`: (wind [m/s], archive, note)."
const WIND_SCAN_2026_08_19 = [
    (5.0, "2026-08-19_095737", ""),
    (5.5, "2026-08-19_094714", ""),
    (6.0, "2026-08-19_094153", ""),
    (6.5, "2026-08-19_094958", ""),
    (7.0, "2026-08-19_100025", ""),
    (6.5, "2026-08-19_095327", "control: el_offset_wing = 0, everything else as flown"),
]

"One row of the scan, from an archive directory name under output/archives/."
function scan_row((wind, archive, note)::Tuple)
    dir = joinpath(SKC_ROOT, "output", "archives", archive)
    y = YAML.load_file(joinpath(dir, "reelout_150m_opt.yaml"))
    m, ro, t = y["fig8_metrics"], y["reelout"], y["traj_opt"]
    sl = load_log("reelout_150m_opt"; path = dir).syslog
    rp = reelout_power(sl)
    v = Float64.(getindex.(sl.v_reelout, 1))[rp.idx]   # same window as force/power
    v_mean, v_peak = mean(v), maximum(v)
    return (; wind, archive, note,
            laps = m["laps"],
            rms = m["cross_track_deg"]["rms"], max_d = m["cross_track_deg"]["max"],
            min_el = m["elevation_deg"]["min_whole_run"],
            span_lap = m["extent"]["elevation_span_lap_deg"],
            span_pct = m["extent"]["elevation_span_pct_of_b"],
            sat = m["steering"]["pct_time_within_2pct_of_peak"],
            # Top-level since the verdict was promoted out of `fig8_metrics:`;
            # the nested position is where the 2026-08-19 archives have it.
            crit = get(y, "success_criteria", get(m, "success_criteria", "unknown")),
            p_mean = ro["power"]["mean_W"], p_peak = ro["power"]["peak_W"],
            p_cf = ro["power"]["cf_power_ro"],
            f_mean = ro["force"]["mean_N"], f_peak = ro["force"]["peak_N"],
            f_cf = ro["force"]["cf_force_ro"],
            v_mean = round(v_mean; digits = 3), v_peak = round(v_peak; digits = 3),
            v_cf = round(v_peak / v_mean; digits = 2),
            l_end = ro["tether"]["end_m"], stop = ro["tether"]["stop_reason"],
            e_run = ro["power"]["energy_run_kJ"],
            ratio = t["power"]["ratio"], p_pred = t["power"]["predicted_W"],
            profile = t["el_bias"]["profile_deg"],
            margin = t["feasibility"]["margin_final_flown"],
            installed = t["reopt"]["installed"], requests = t["reopt"]["requests"])
end

"Write the rows to `path` as YAML; returns the path."
function write_scan(rows, path = joinpath(SKC_ROOT, "docs", "wind_scan_results.yaml"))
    io = IOBuffer()
    println(io, """
    # Wind scan of the reel-out run flown along an externally optimized path
    # (examples/simple_opt_reelout.jl), 150 -> 380 m, no turbulence, sim_time 150 s.
    # WRITTEN BY examples/wind_scan.jl — regenerate it there, do not hand-edit.
    # Everything except environment.v_wind / wind_vec of data/settings_reelout_150m.yaml
    # is the flown configuration of 2026-08-19 (el_bias_bins 5, el_bias_smooth 0.25,
    # el_offset_final 1.5, el_offset_lead 8.0, el_offset_wing 1.5 azimuth 10/8,
    # attractor_dist 8.0, depower_final 0.328, min_feasibility_margin 0.74).
    #
    # Force, power and speed are scored over the SAME window: the samples where the
    # length setpoint was still growing (SimpleKiteControllers.reelout_power), i.e.
    # the actual reel-out phase. crest factor = peak / mean.
    #
    # Runs are deterministic: a repeat at identical settings reproduces every number
    # below, so these are single runs (see docs/fig8_tuning_log.md, "The runs DO
    # repeat, and the wind is what the settings were never tested against").
    wind_scan:
      date: "2026-08-19"
      git_hash: "8231f95"
      project: "system_reelout_150m.yaml"
      runs:""")
    for r in rows
        @printf(io, "    - v_wind_m_s: %.1f\n", r.wind)
        println(io, "      archive: \"output/archives/$(r.archive)\"")
        isempty(r.note) || println(io, "      note: \"$(r.note)\"")
        println(io, "      success_criteria: \"$(r.crit)\"")
        @printf(io, "      laps: %.1f\n", r.laps)
        @printf(io, "      cross_track_deg: {rms: %.2f, max: %.2f}\n", r.rms, r.max_d)
        @printf(io, "      power_W: {mean: %d, peak: %d, crest_factor: %.2f, predicted: %d, ratio: %.2f}\n",
                r.p_mean, r.p_peak, r.p_cf, r.p_pred, r.ratio)
        @printf(io, "      force_N: {mean: %d, peak: %d, crest_factor: %.2f}\n",
                r.f_mean, r.f_peak, r.f_cf)
        @printf(io, "      reelout_speed_m_s: {mean: %.3f, peak: %.3f, crest_factor: %.2f}\n",
                r.v_mean, r.v_peak, r.v_cf)
        @printf(io, "      energy_run_kJ: %.1f\n", r.e_run)
        @printf(io, "      tether: {end_m: %.1f, stop_reason: \"%s\"}\n", r.l_end, r.stop)
        @printf(io, "      elevation: {min_deg: %.1f, span_per_lap_deg: %.1f, pct_of_commanded_B: %d}\n",
                r.min_el, r.span_lap, r.span_pct)
        @printf(io, "      steering_saturation_pct: %d\n", r.sat)
        @printf(io, "      el_bias_profile_deg: %s\n", string(r.profile))
        @printf(io, "      reopt: {requests: %d, installed: %d, margin_final_flown: %.2f}\n",
                r.requests, r.installed, r.margin)
    end
    write(path, String(take!(io)))
    return path
end
