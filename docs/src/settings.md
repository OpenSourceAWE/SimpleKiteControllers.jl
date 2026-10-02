```@meta
CurrentModule = SimpleKiteControllers
```

# Settings

All settings of a run are YAML files in the `data/` directory of this package. The
example scripts call `set_data_path` on that directory, so the kite model reads them
from here instead of from its own data directory. You can change the wind, the tether,
the winch or the controller tuning without editing V3Kite.

## The system project

A run starts from one **system project**, a file `data/system_*.yaml`. It holds no
settings of its own. Its `system:` section lists the YAML files that together
describe the complete system and its controllers. For example,
`data/system_reelout_180m.yaml`:

```yaml
system:
    sim_settings: "settings_reelout_180m.yaml"
    wc_settings: "wc_settings.yaml"
    fc_settings: "fc_settings_reelout.yaml"
    traj_opt_settings: "traj_opt.yaml"
    winch_table: "winch_table.yaml"
    turn_rate_coeffs: "turn_rate_coeffs.yaml"
    kite_settings: "kite_settings_psm.yaml"
    structural_geometry: "struc_geometry.yaml"
    aero_geometry: "cfd_aero_geometry.yaml"
    vsm_settings: "vsm_settings.yaml"
    settle_settings: "settle_settings_default.yaml"
    heading_settings: "heading_settings_psm.yaml"
```

[`project_file`](@ref) resolves a project name such as `"system_reelout_180m.yaml"` to
this package's `data/` directory. The functions [`fc_settings`](@ref),
[`turn_rate_coeffs_file`](@ref), [`winch_table_file`](@ref) and
[`traj_opt_settings_file`](@ref) each return one entry of the `system:` section, in
the same way as `KiteUtils.wc_settings`.

These projects are included (all names without the `.yaml` extension):

| Project                     | Run              | `sim_settings`            | `fc_settings`           |
|:----------------------------|:-----------------|:--------------------------|:------------------------|
| `system_fig8_150m`          | fig8, 150 m      | `settings_fig8_150m`      | `fc_settings_fig8_150m` |
| `system_fig8_200m`          | fig8, 200 m      | `settings_fig8_200m`      | `fc_settings`           |
| `system_fig8_300m`          | fig8, 300 m      | `settings_fig8_300m`      | `fc_settings`           |
| `system_reelout_maasvlakte` | reel-out, 150 m  | `settings_reelout_150m`   | `fc_settings_reelout`   |
| `system_reelout_180m`       | reel-out, 180 m  | `settings_reelout_180m`   | `fc_settings_reelout`   |
| `system_reelout_cabauw`     | reel-out, Cabauw | `settings_reelout_cabauw` | `fc_settings_reelout`   |

`system_fig8_200m.yaml` is the default project: [`project_file`](@ref) uses it when no
name is given, and the turn-rate table is loaded against it when the package loads.

## The files a project names

- `sim_settings` → `settings_*.yaml`, read into `KiteUtils.Settings`: the plant and the
  simulation. Wind speed and profile, tether length, mass, winch model, KCU, solver,
  `sim_time` and `sample_freq` (the time step).
- `wc_settings` → `wc_settings.yaml`, read into `WCSettings`: the winch controllers. The
  torque block (V3Kite's position and force modes) and the speed block (WinchControllers.jl's
  reel-out law and its force limiters).
- `fc_settings` → `fc_settings*.yaml`, read into [`FC_Settings`](@ref): the flight
  controller. Entry state machine, pattern geometry, heading/course PID, feed-forward,
  reel-out phases and the run's pass criteria.
- `traj_opt_settings` → `traj_opt.yaml`, read into [`TrajOptSettings`](@ref): the AWETrim
  client. Server, initial guess, solver settings and re-optimization; only used by
  `simple_opt_fig8.jl` and `simple_opt_reelout.jl`.
- `winch_table` → `winch_table.yaml`, read by [`winch_f_low`](@ref) and
  [`winch_force_limit`](@ref): wind-dependent winch-law parameters. Reel-out projects only.
- `turn_rate_coeffs` → `turn_rate_coeffs.yaml`, read by [`turn_rate_coeffs`](@ref): the
  identified turn-rate law (`c1`, `c2`, `delay`) per body damping and depower.
- `kite_settings` → `kite_settings_psm*.yaml` in this package, read into V3Kite's
  `V3KiteConfig`: wing model, aerodynamics mode, backend and in-flight damping.
- `structural_geometry`, `aero_geometry`, `vsm_settings`, `settle_settings`,
  `heading_settings` → V3Kite's own files: geometry, polars, VSM and settling settings,
  read from V3Kite's data directory.

V3Kite looks for each of these files beside the project first and in its own data
directory after. The figure-of-eight projects and `system_reelout_180m.yaml` name
`kite_settings_psm.yaml`, a local copy of V3Kite's file, so an upstream change does not
reach these runs unnoticed. `system_reelout_maasvlakte.yaml` and
`system_reelout_cabauw.yaml` name `kite_settings_psm_kernel.yaml`, a local variant whose only
difference in effect is `analytic_jacobian: true`. The `vsm_interval` in these files is not read by the runs of
this package: they pass `run.vsm_interval` of `fc_settings` to `step!`.

Not named by any project:
- `optimization.yaml`: the settings of the pattern-shape sweep of
  `examples/optimize_fig8.jl`, read into [`OptSettings`](@ref).
- `gui.yaml`: the choices of the example menu (project, simulation time, plots, wind
  speed), see [State of the example menu](@ref). It is created from `gui.yaml.default`
  on first use and is not under version control.

## Rules shared by the settings files

- Every key of `traj_opt.yaml` and `optimization.yaml` is a field of the struct it is
  loaded into. A missing key falls back to the struct default; an unknown key is an
  error (see [`load_yaml_fields!`](@ref)).
- `fc_settings*.yaml` has one section per part of [`FC_Settings`](@ref): `course`,
  `feedforward`, `pattern`, `wind_ramp`, `winch`, `reelout` and `run`. Each key of a
  section is a field of that part, with the same rules. A figure-of-eight run can leave
  out the `reelout` section. Files archived before this layout, with all keys directly
  under `fc_settings:`, still load.
- The struct docstrings (on the [API](api/index.md) pages) say in one line what a field
  is. The comments in the YAML files give the longer notes and the reason for the value
  chosen there. The history of the tuning is in
  [docs/fig8_tuning_log.md](https://github.com/OpenSourceAWE/SimpleKiteControllers.jl/blob/main/docs/fig8_tuning_log.md).
- `fc_settings_fig8_150m.yaml` and `fc_settings_reelout.yaml` started as copies of
  `fc_settings.yaml`. Keys they share must be kept in step by hand.
- An example script can override single fields for one run without editing a file, see
  [`apply_overrides!`](@ref).

## Example: `fc_settings_reelout.yaml`

The flight-controller settings of all reel-out projects, shown here as shipped:

```@raw html
<div class="small-code">
```

```@eval
using Markdown, SimpleKiteControllers
file = joinpath(pkgdir(SimpleKiteControllers), "data", "fc_settings_reelout.yaml")
Markdown.MD(Markdown.Code("yaml", read(file, String)))
```

```@raw html
</div>
```
