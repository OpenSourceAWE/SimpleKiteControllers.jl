# Calculating steering and depower

There is rel_depower and rel_steering. How are these relative values converted to meters of line length/ line length difference?

The short answer: **only on the V3 (particle-system) plant is there a conversion to
meters at all.** `rel_depower`/`rel_steering` are dimensionless actuator commands
(`0..1` and `-1..1`), and what happens to them depends on the plant:

| plant | `rel_steering` becomes | `rel_depower` becomes |
| --- | --- | --- |
| V3Kite (`SymbolicAWEModel`) | left/right tape lengths [m] | depower tape length [m] |
| KPS4 (four point) | a side-plane angle [rad] | an angle, via a tape length [m] |
| KPS3 (one point) | a steering force [N] | an angle, via a tape length [m] |

Both paths share the same front end.

## Step 1 (all plants): the KCU actuator lag

`rel_*` never reaches the geometry directly — it is a *setpoint* handed to the KCU
model (`KitePodModels.jl`), which moves the real tape at finite speed:

```julia
set_depower_steering(kcu, rel_depower, rel_steering)  # set_steering = steering * cs_4p
on_timer(kcu, dt)
dp = get_depower(kcu)     # actual, tape-lagged
st = get_steering(kcu)
```

`on_timer` is a proportional controller with a rate limit, per channel:

$$
v = \mathrm{clamp}\big(g \cdot (u_\mathrm{set} - u),\ -v_\mathrm{max},\ +v_\mathrm{max}\big),
\qquad u \mathrel{+}= v\,\Delta t
$$

with $g$ = `depower_gain`/`steering_gain` (both default `3.0`) and $v_\mathrm{max}$ =
`v_depower`/`v_steering`. Note the rate limits are in **relative units per second**,
not m/s — this repo sets `v_depower: 0.075`, `v_steering: 0.2` in
[data/settings_fig8_200m.yaml](../data/settings_fig8_200m.yaml). So the depower
channel needs ~13 s to cross its full range, the steering channel ~5 s.

Everything downstream uses the lagged `dp`/`st`, never the command.

## Step 2a: V3Kite — the actual meters

This is the plant this package flies. `V3Kite`'s `step!` calls `set_depower!` and
`set_steering!` (`src/calibration.jl`), which write **unstretched segment lengths
`l0`** straight into the system structure. The conversion is affine.

### Steering: a symmetric differential

$$
L_\mathrm{left} = L_0 - G_s\,u_s, \qquad
L_\mathrm{right} = L_0 + G_s\,u_s
$$

with $L_0$ = `V3_STEERING_L0_BASE` = **1.6 m**, $G_s$ = `V3_STEERING_GAIN` =
**1.4 m**, and $u_s$ = `rel_steering`. Written to `sys.segments[89].l0` (left) and
`sys.segments[87].l0` (right).

So each side moves up to $\pm 1.4$ m, and the **length difference** is

$$
\Delta L = L_\mathrm{right} - L_\mathrm{left} = 2 G_s u_s = 2.8\,u_s \ \ [\mathrm{m}]
$$

i.e. 2.8 m of differential at full deflection, and 2.8 cm per 0.01 of `rel_steering`.
Positive `rel_steering` shortens the left tape. The inverse
(`steering_length_to_percentage`) depends only on the gain, not on $L_0$:
$u_s = (L_\mathrm{right} - L_\mathrm{left}) / 2G_s$.

### Depower: a single tape

$$
L_\mathrm{depower} = L_{0,d} + G_d \cdot u_p, \qquad
u_p = u_\mathrm{depower} + \mathrm{offset} + k\,\lvert u_s \rvert
$$

with $L_{0,d}$ = `V3_DEPOWER_L0_BASE` = **0.2 m** and $G_d$ = `V3_DEPOWER_GAIN` =
**5.0 m**. Written to `sys.segments[88].l0`. Full travel `0 → 1` is therefore
0.2 m → 5.2 m, i.e. **5 m of tape**, or 5 cm per 0.01 of `rel_depower`.

The two extra terms come from the `V3GeomAdjustConfig` (`src/model_setup.jl`) and
are both `0.0` by default: `depower_offset` is a constant bias, and
`steering_dp_offset` adds depower proportional to `abs(steering)` — the physical
effect that steering also depowers a little.

That config also carries `reduce_steering`/`reduce_depower` (default `false`), which
subtract `0.2 m` from the respective $L_0$ base — they shift the neutral point, they
do not change either gain.

## Step 2b: KPS3/KPS4 — no steering length at all

For the older models in `KiteModels.jl` the picture is different, and this is the
part that most often surprises: **`rel_steering` is never a length.**
`set_depower_steering!` first removes an offset and divides out the
depower-dependent sensitivity,

$$
u_s' = \frac{u_s - c_0}{1 + k_{ds}\,\alpha_\mathrm{depower}/\alpha_{d,\max}}
$$

and then KPS4 uses $u_s'$ as an **angle**: `ks = deg2rad(max_steering)`, applied
antisymmetrically to the two side planes as $\pm u_s' \cdot k_s$ on their angle of
attack, plus a steering force $\propto u_s' k_s$. KPS3 skips the angle entirely and
turns $u_s'$ straight into a side force via `c_s` and `rel_side_area`.

Depower *does* go through a genuine tape length in these models, but only as an
intermediate to get an angle. `calc_delta_l` (`KitePodModels.jl`) models the tape
unwinding from a drum whose effective radius shrinks by `tape_thickness` every turn:

```julia
rotations = (rel_depower - 0.01*depower_offset) * 10.0 * 11.0/3.0 * (3918.8 - 230.8)/4096
while rotations > 0.0
    l_ro += u * π; rotations -= 1.0; u -= tape_thickness
end
```

so $\Delta l$ is mildly nonlinear in `rel_depower`. `calc_alpha_depower1` then feeds
it through a triangle (the factor `0.5` is the pulleys) to get the angle between kite
and last tether segment:

$$
b = b_0 + \tfrac{1}{2}\Delta l, \quad c = \sqrt{a^2 + b_0^2}, \quad
\alpha_\mathrm{depower} = \frac{\pi}{2} - \arccos\frac{a^2 + b^2 - c^2}{2ab}
$$

with $a$ = `power2steer_dist` and $b_0$ = `h_bridle + 0.5·height_k`. The `KCU2`
model bypasses the length entirely and is linear by construction:
$\alpha_\mathrm{depower} = 100\,(u_p - 0.01\,z)\cdot d$ in rad, with $z$ =
`depower_zero` and $d$ = `degrees_per_percent_power`.

## A third scale: the AWETrim optimizer

The trajectory optimizer ([docs/TrajectoryOptimization.md](TrajectoryOptimization.md),
driven by `examples/awetrim_client.jl`) does not take a relative command at all. Its
`input_depower` **is** $l_\mathrm{dp}$, an absolute power-tape length in metres,
bounded to `(1.1, 2.3)` m in AWETrim's `src/awetrim/utils/defaults.py`. So the number
in `data/traj_opt.yaml` is already in the unit the server wants — but it is on a
*different scale* from V3Kite's.

AWETrim calibrates against the 2019 V3 flight logs
(`src/awetrim/identification/controls.py`): kcu 22 % is powered at 1.7 m, kcu 30 % is
depowered at 2.1 m. That is the same 5 m of tape per unit of `rel_depower` as V3Kite,
with a different zero:

| scale | mapping | `rel_depower` = 0.282 → |
| --- | --- | --- |
| V3Kite (`V3_DEPOWER_L0_BASE`) | $l = 0.2 + 5\,u_p$ | **1.61 m** |
| AWETrim (`ROM_*_INPUT_DEPOWER`) | $l = 0.6 + 5\,u_p$ | **2.01 m** |

The gap is a constant **0.4 m**, because AWETrim's $l_\mathrm{dp}$ is measured off the
AS/QSM structural baseline of 1.5 m (`AS_POWER_TAPE_BASELINE_LDP_M`) while V3Kite's
0.2 m base is the tape segment alone. Both claim to be kcu percent, so they cannot
both be right about the same kite; 0.4 m is 8 kcu-%, roughly a third of the useful
range. Converting a flown depower into an `input_depower` on V3Kite's base is
therefore wrong by that much.

### It is not a rescaling that can be applied

Pinning `input_depower` at the flown depower does not simply move the prediction — it
removes the solution. Measured 2026-08-18 at 150 m and 6 m/s, on the same guess and
winch as `simple_opt_reelout.jl`, with `input_depower` taken out of
`optimization_params` so it stays fixed:

| `input_depower` [m] | predicted mean power [W] | mean tether force [N] |
| --- | --- | --- |
| 1.44 | 5740 | 2803 |
| 1.457 (free optimum) | 6080 | 3028 |
| 1.60 (the seed, pinned) | 4486 | 2463 |
| 1.80 | 1873 | 1511 |
| 2.01 (the flown depower) | infeasible, HTTP 422 | — |
| 2.20 | infeasible, HTTP 422 | — |

The fixed rows are lower bounds rather than a clean sensitivity curve — the problem is
multi-modal, and with the depower pinned the solve settles on a differently shaped
optimum (the 1.44 row spans azimuth $-27.5\ldots34.2°$, far wider than the free
optimum's $-19.5\ldots21.6°$). The trend is unambiguous all the same: the ROM's power
collapses as the tape is let out.

### The two models agree on sensitivity, and differ by an offset

Flying the same lever on the plant settles what that means. Sweeping
`depower_setpoint` on the 20°/11° lemniscate at 180 m (`simple_reelout.jl`, same wind
and winch as the ROM was given) against the ROM's own pinned sweep:

| | slope $\mathrm{d}\ln P / \mathrm{d}u_p$ | at equal power (6538 W) |
| --- | --- | --- |
| V3Kite (VSM) | **-15.2** per unit `rel_depower` | $u_p = 0.2915$ |
| AWETrim (ROM) | **-17.8** per unit `rel_depower` | $u_p = 0.1699$ |

The slopes are within 15 % of each other, so the earlier reading — that the models
disagree about depower *qualitatively* — was wrong. They respond to depower almost
identically; they are **offset along the depower axis by 0.12 of `rel_depower`, i.e.
0.61 m of tape**. The 0.4 m calibration offset above accounts for about two thirds of
that, leaving ~0.2 m of genuine aerodynamic disagreement.

That is why `input_depower` is left free and a predicted-vs-measured ratio is a
comparison at two different depower settings — but the fix is a shift, not a rebuild.

**Caveat, measured after the above:** both AWETrim columns here were solved with its
built-in tether of 10 mm / 970 kg/m³ against this repo's 4 mm / 724 kg/m³. Correcting
that raises the optimizer's power by 22-25 %, so the 0.61 m equal-power offset is an
**upper bound** on the depower discrepancy, not the residual. The slope comparison is
unaffected — it is a shape, not a level. See
[fig8_tuning_log.md](fig8_tuning_log.md).

## Why it matters here

This package commands `rel_steering` and only ever sees the plant respond as a turn
rate, $\dot\psi = c_1 v_a u_s$. The 2.8 m/unit figure above is what closes that loop
physically, and the KCU rate limit is why `turn_rate_coeffs` carries a `delay`
alongside `c1`/`c2` — the steering tape cannot follow a step command.
