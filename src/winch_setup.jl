# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# The winch settings and length loop of a run: `WCSettings` loaded from the project's
# `wc_settings:` file and set to the wind-dependent tables of `winch_table.jl`; and the
# controllers of a run built on the settled model.

"""
    load_wc_settings(filename; dt) -> WCSettings

Load winch-controller settings from the YAML file `filename`, looked up under
the active data path (`joinpath(get_data_path(), filename)`) unless absolute.
The file must have a top-level `wc_settings:` mapping whose keys are fields of
`WCSettings`; a missing key keeps the struct default, an unknown key errors.

This used to be V3Kite's own `WC_Settings(filename)`. It lives in SimpleKiteControllers now because
the struct belongs to WinchControllers.jl and the *file* belongs to the run —
V3Kite itself no longer reads winch gains at all. `dt` always wins over the
file's placeholder value: it is the plant's timestep, not a tuning choice.
"""
function load_wc_settings(filename::AbstractString; dt)
    path = isabspath(filename) ? filename : joinpath(KiteUtils.get_data_path(), filename)
    dict = YAML.load_file(path)["wc_settings"]
    wcs = WCSettings(; dt)
    for (key, value) in dict
        sym = Symbol(key)
        hasfield(WCSettings, sym) ||
            error("Unknown key \"$key\" in $path — not a field of WCSettings.")
        setfield!(wcs, sym, convert(fieldtype(WCSettings, sym), value))
    end
    wcs.dt = dt
    return wcs
end

"""
    build_winch(project, project_set, fcs) -> (; wc, wpc, dt0)

The winch settings and the length loop of the run. ONE `WCSettings` (`wc`) serves BOTH
winch loops, the POSITION-mode torque gains (`wpc`) and the speed-controller tuning of the
reel-out controller, so the wind-dependent force floor and force-limit law of the
project's tables are set on it here. Refuses a `compliance` other than 0: `REEL_OUT` and
V3Kite's own FORCE mode both drive the winch, and only one can hold the drum at a time.
"""
function build_winch(project, project_set, fcs)
    fcs.compliance >= 0 ||
        error("compliance must be >= 0, got $(fcs.compliance)")
    fcs.compliance == 0 ||
        error("REEL_OUT needs compliance = 0 (POSITION mode) — REEL_OUT and V3Kite's own \
               FORCE mode both drive the winch and only one can hold the drum at a time.")
    dt0 = 1 / project_set.sample_freq
    wc = load_wc_settings(KiteUtils.wc_settings(project); dt = dt0)
    # Wind-dependent floor; NOT the entry guard's floor, that is fcs.entry_f_min.
    wc.f_low = winch_f_low(project_set.v_wind; project)
    # The soft law's floor cannot go below ~700 N, so it is off at low wind; see `winch_force_limit`'s docstring.
    wc.force_limit = winch_force_limit(project_set.v_wind; project)
    wpc = WinchPosController(wc; dt = dt0)   # the length loop `step!` used to own
    return (; wc, wpc, dt0)
end

"""
    build_controllers(fcs, rcs, s) -> (; rc, f_high_nominal, stop_criteria, guard_lfc, l_set, fec)

The controllers of the run, built on the settled model `s`: the reel-out winch controller
`rc` (built after `init`, so its soft-start ramp begins when reel-out starts; `rcs` is the
one `WCSettings` of both winches), the nominal force ceiling `f_high_nominal` captured
before the first-lap reduction (`winch_from_wc` sends this one to the optimizer), the
standalone force-floor guard for phases 0-2 (`rc`'s own `SpeedController` would wind up
while its output is ignored), the length setpoint `l_set` (the settled length, growing
from phase 3 until it reaches `reelout_l_max`) and the figure-of-eight controller `fec`.

Of the model it reads only `s.dt` and `s.sys_state.l_tether`, so it takes any plant.
"""
function build_controllers(fcs, rcs, s)
    rcs.dt = s.dt
    rc = WinchController(rcs)
    f_high_nominal = rcs.f_high
    stop_criteria = fcs.n_fig_eight > 0 ?
        @sprintf("%.0f m or after %d figures of eight", fcs.reelout_l_max, fcs.n_fig_eight) :
        @sprintf("%.0f m", fcs.reelout_l_max)
    @info @sprintf("Winch: REEL_OUT mode — %s, stopping at %s.",
                   rcs.force_limit == "soft" ?
                       @sprintf("soft force limit inverting kv = %.4f saturated at [%.0f, %.0f] N \
                                 (beta %.0e/%.0e, force filtered at tau = %.2f s); the \
                                 UpperForceController is held in reset",
                                rcs.kv, rcs.f_low, rcs.f_high, rcs.softminus_beta,
                                rcs.softplus_beta, rcs.force_limit_tau) :
                       @sprintf("v_set = %.3f * sqrt(force)", rcs.kv),
                   stop_criteria)
    guard_lfc = LowerForceController(rcs)
    l_set = s.sys_state.l_tether[1]
    fec = FigureEightController(fcs; dt = s.dt)
    return (; rc, f_high_nominal, stop_criteria, guard_lfc, l_set, fec)
end
