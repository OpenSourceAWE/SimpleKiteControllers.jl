# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The model side of `simple_opt_reelout.jl`, which the package cannot hold because it
does not depend on V3Kite (see `oldplans/Plan_refactor_opt_reelout.md`). Included at the top of
the script, into the same module; `init_model` is handed to `setup_run`.
"""

"""
    init_model(project, project_set, fcs, wpc, sim_time; turbulence, set_overrides,
               aero_mode = ContinuousAero(), damping_per_stiffness = 0.002)

The model, initialized and settled: `init` at the project's wind and tether length,
`sim_time` plus a full phase 5 (`fcs.final_time`) long, with the winch loop `wpc`
holding the length during the warm-up. `set_overrides` are applied to
the model's own `Settings` afterwards, e.g. `v_steering`, the tape's rate limit.
`aero_mode` is `ContinuousAero()` or `AeroDirect()`; `damping_per_stiffness` [s] is the
tether/bridle structural damping as a ratio of stiffness, see simple_fig8.jl's docstring.
"""
function init_model(project, project_set, fcs, wpc, sim_time; turbulence, set_overrides,
                    aero_mode = ContinuousAero(), damping_per_stiffness = 0.002)
    # dt, sim_time and wind come from the project settings (overridden above); the default cache_path avoids a re-JIT.
    s = init(project_set.v_wind, project_set.l_tether; body_start_damping = fcs.body_damping,
        body_sim_damping = 0.8 .* fcs.body_damping,
        damping_per_stiffness = damping_per_stiffness,
        elevation = fcs.elevation, depower_setpoint = fcs.depower_setpoint,
        system_yaml = project, use_turbulence = turbulence, aero_mode = aero_mode,
        # Room for a full phase 5 past the budget: the budget ends a run only BEFORE phase 5,
        # which then always flies `final_time` (see the loop). At 10 m/s the budget cut it at 19.7 s.
        sim_time = sim_time + (isfinite(fcs.final_time) ? fcs.final_time : 0.0),
        warmup_time = fcs.warmup_time,
        # The warm-up relaxes at constant length, against the same loop the run uses.
        warmup_torque = (m, l) -> winch_torque!(wpc, m, l), remake_model = false)
    @info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)
    apply_overrides!(s.kcu.set, set_overrides, "set_overrides", "Settings", "plant")
    return s
end

