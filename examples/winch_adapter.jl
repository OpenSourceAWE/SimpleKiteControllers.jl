# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Plant/controller adapter. V3Kite is the PLANT: `step!` takes a winch TORQUE,
# because that is what the drum takes. Holding a LENGTH is a control problem —
# it needs a cascaded loop with its own gains, saturations and state — so that
# controller lives in WinchControllers.jl and is the CALLER's to own, exactly as
# `KiteModels.jl`'s `next_step!` takes only speed/torque/force.
#
# The glue between the two is this file: it reads the plant scalars off a
# `V3KITE` and hands them to WinchControllers' scalar-only functions. It belongs
# to the application, not to either package — V3Kite must not depend on a
# controller, and WinchControllers must not depend on a kite model.
#
# A copy of V3Kite's `examples/winch_adapter.jl` — the adapter belongs to the
# APPLICATION, and this repo is one. Keep the two in step if either changes.
#
# `include` it and construct a controller once per run:
#
#     wcs = WCSettings(dt = s.dt)          # or WC_Settings("wc_settings.yaml")
#     wpc = WinchPosController(wcs; dt = s.dt)
#     step!(s; rel_depower, set_torque = winch_torque!(wpc, s, l0))

using WinchControllers: WCSettings, WinchPosController, WinchForceController,
    winch_position_torque!, winch_force_torque!

"""
    load_wc_settings(filename; dt) -> WCSettings

Load winch-controller settings from the YAML file `filename`, looked up under
the active data path (`joinpath(get_data_path(), filename)`) unless absolute.
The file must have a top-level `wc_settings:` mapping whose keys are fields of
`WCSettings`; a missing key keeps the struct default, an unknown key errors.

This used to be V3Kite's own `WC_Settings(filename)`. It lives here now because
the struct belongs to WinchControllers.jl and the *file* belongs to the run —
V3Kite itself no longer reads winch gains at all. `dt` always wins over the
file's placeholder value: it is the plant's timestep, not a tuning choice.
"""
function load_wc_settings(filename::AbstractString; dt)
    path = isabspath(filename) ? filename : joinpath(get_data_path(), filename)
    dict = V3Kite.YAML.load_file(path)["wc_settings"]
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
    winch_torque!(wpc::WinchPosController, s::V3KITE, set_length;
                  v_ff=0.0, speed_limit=Inf,
                  acceleration_limit=winch_acc_limit(s.set)) -> torque

Run WinchControllers.jl's cascaded length loop against `s` and return the winch
torque [N·m] for `step!`'s `set_torque`. Replaces the `set_length` keyword
`step!` used to carry before the winch controllers moved out of V3Kite.

`acceleration_limit` comes from the plant's own `winch: max_acc:`
([`winch_acc_limit`](@ref)), which is what `step!` defaulted it to. `v_ff` [m/s]
is the speed feed-forward: a caller integrating a speed setpoint into
`set_length` should pass that same speed, or the outer P loop has to rediscover
it from a length error at a cost of `1/kp_pos` of lag.
"""
function winch_torque!(wpc::WinchPosController, s::V3KITE, set_length;
                       v_ff = 0.0, speed_limit = Inf,
                       acceleration_limit = winch_acc_limit(s.set))
    r, G, friction = drum_params(s)
    return winch_position_torque!(wpc, set_length, unstretched_length(s),
                                  reel_out_speed(s), winch_force(s),
                                  r, G, friction, s.dt,
                                  speed_limit, acceleration_limit; v_ff)
end

"""
    winch_force_hold!(wfc::WinchForceController, s::V3KITE, set_length) -> torque

The force-mode counterpart of [`winch_torque!`](@ref): the drum holds a FORCE,
paying out whenever the tether pulls harder than the reference, and only trims
towards `set_length`. Returns a torque for `step!`'s `set_torque`.
"""
function winch_force_hold!(wfc::WinchForceController, s::V3KITE, set_length)
    r, G, friction = drum_params(s)
    return winch_force_torque!(wfc, set_length, unstretched_length(s),
                               reel_out_speed(s), winch_force(s),
                               r, G, friction, s.dt)
end
