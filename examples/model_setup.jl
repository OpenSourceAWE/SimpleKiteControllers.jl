# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
The model side of the example scripts, which the package cannot hold because it
does not depend on V3Kite (see `oldplans/Plan_refactor_opt_reelout.md`). Included at the
top of `simple_fig8.jl`, `simple_fig8_live.jl`, `simple_opt_fig8.jl`, `simple_reelout.jl`
and `simple_opt_reelout.jl`, into the same module; the last one hands `init_model` to
`setup_run`.
"""

"""
    init_model(project, project_set, fcs, wpc, sim_time; turbulence, set_overrides = (),
               aero_mode = ContinuousAero(), damping_per_stiffness = 0.002,
               warmup_torque = (m, l) -> winch_torque!(wpc, m, l), pad_final_time = true)

The model, initialized and settled: `init` at the project's wind and tether length,
`sim_time` long, with `warmup_torque` holding the length during the warm-up (by default
the winch loop `wpc`; pass the force hold when the run flies the winch in FORCE mode).
With `pad_final_time` the run gets room for a full phase 5 (`fcs.reelout.final_time`) past
`sim_time`; a `sim_time` of `nothing` falls back to the project's own value, unpadded.
`set_overrides` are applied to the model's own `Settings` afterwards, e.g. `v_steering`,
the tape's rate limit. `aero_mode` is `ContinuousAero()` or `AeroDirect()`;
`damping_per_stiffness` [s] is the tether/bridle structural damping as a ratio of
stiffness, see `simple_fig8.jl`'s docstring.

The wing drag of the project's kite settings is applied after settling and before
the warm-up, see [`apply_wing_drag!`](@ref).
"""
function init_model(project, project_set, fcs, wpc, sim_time; turbulence, set_overrides = (),
                    aero_mode = ContinuousAero(), damping_per_stiffness = 0.002,
                    warmup_torque = (m, l) -> winch_torque!(wpc, m, l), pad_final_time = true)
    # Room for a full phase 5 past the budget: the budget ends a run only BEFORE phase 5,
    # which then always flies `final_time` (see the loop). At 10 m/s the budget cut it at 19.7 s.
    if pad_final_time && !isnothing(sim_time)
        sim_time += isfinite(fcs.reelout.final_time) ? fcs.reelout.final_time : 0.0
    end
    # No dt: init takes it from the project's settings (sample_freq). project_set.v_wind
    # keeps the mean wind and the turbulent field (which init builds for it) at the same speed.
    # No cache_path either: V3Kite's default is where its own precompile workload
    # compiled the model, and a different model binary costs 40 s of re-JIT in init.
    s = init(project_set.v_wind, project_set.l_tether; body_start_damping = fcs.run.body_damping,
        body_sim_damping = 0.8 .* fcs.run.body_damping,
        damping_per_stiffness = damping_per_stiffness,
        elevation = fcs.run.elevation, depower_setpoint = fcs.course.depower_setpoint,
        system_yaml = project, use_turbulence = turbulence, aero_mode = aero_mode,
        # The warm-up follows below, once the wing drag is in place.
        sim_time = sim_time, warmup_time = 0.0, remake_model = false)
    apply_wing_drag!(s, project)
    # The warm-up must relax against the winch the loop will command.
    warmup!(s, fcs.run.warmup_time; depower = fcs.course.depower_setpoint,
            winch_torque = warmup_torque)
    @info @sprintf("Run: %.0f s at dt = %.4f s (%d steps).", s.steps * s.dt, s.dt, s.steps)
    apply_overrides!(s.kcu.set, set_overrides, "set_overrides", "Settings", "plant")
    return s
end

"""
    apply_wing_drag!(s, project)

Apply the `wing_drag_coeff` of `project`'s `kite_settings:` file to the model `s`,
which V3Kite's `init` reads but does not apply: `distribute_wing_drag!` splits the
projected wing area equally over the wing nodes, each with that drag coefficient.
`0` adds none. The kernel backend re-syncs point parameters every step, so the drag
acts from the next step on. Every model this package flies goes through it,
`init_model` and the relay flights of `build_turn_rate_table.jl` alike, so the turn-rate
law is identified on the plant the runs fly.
"""
function apply_wing_drag!(s, project)
    kite_set = load_kite(project; data_path = project_data_path(project, nothing))
    kite_set.wing_drag_coeff > 0 || return nothing
    sys = s.sam.sys_struct
    distribute_wing_drag!(sys, sys.wings[1].vsm_aero.projected_area, kite_set.wing_drag_coeff)
    return nothing
end
