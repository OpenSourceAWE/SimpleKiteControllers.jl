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

## Summary

- Every plant lags `rel_depower`/`rel_steering` through the same KCU rate-limited
  P controller ([Step 1](#step-1-all-plants-the-kcu-actuator-lag)) before using them
  for anything.
- **V3Kite**, the plant this repo flies, then turns the lagged commands into
  unstretched tape lengths, affinely
  ([Step 2a](#step-2a-v3kite--the-actual-meters)):
  $$
  L_\mathrm{left} = 1.6 - 1.4\,u_s,\quad L_\mathrm{right} = 1.6 + 1.4\,u_s,\quad
  L_\mathrm{depower} = 0.2 + 5.0\,u_p \ \ [\mathrm{m}]
  $$
  with $u_s$ = `rel_steering`, $u_p$ = `rel_depower` (plus config offsets, `0.0` by
  default).
- **KPS3/KPS4** never turn `rel_steering` into a length at all — it becomes an
  angle or a force directly — and `rel_depower` passes through a length only as an
  intermediate on the way to an angle
  ([Step 2b](#step-2b-kps3kps4--no-steering-length-at-all)).
- **AWETrim**'s optimizer works in its own absolute tape length $l_\mathrm{dp}$
  [m], on a different zero than V3Kite's ([A third scale](#a-third-scale-the-awetrim-optimizer)).
  Converting one flown depower into the other is a fixed shift, codified as
  `awetrim_depower_to_v3kite(l_dp) = (l_dp - 0.6)/5.0 + 0.107` — see
  [below](#the-two-models-agree-on-sensitivity-and-differ-by-an-offset) for the
  derivation and a worked example.

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
*different scale* from V3Kite's, and (since 2026-08-20) only the base of what is
actually sent.

### `input_depower` is a seed, not the value flown

By default the server **optimizes** `input_depower` itself and reports what it
landed on (`DepowerSpec(mode = "optimize")`, `examples/awetrim_client.jl`); the
YAML value is only where that search starts. `DepowerSpec` has two other modes:
`"fixed"` pins the solve to one value — the setting the run actually flies, at the
cost of a much smaller feasible set (§ below) — and `"profile"` optimizes one
value per path node, returning a depower schedule instead of a scalar.

The seed itself is no longer the raw `input_depower` above a reference wind speed.
`depower_seed(tos, wind_speed)` (`examples/awetrim_client.jl`) adds
`tos.input_depower_per_wind` metres per m/s of wind above
`tos.input_depower_wind_ref` (7 m/s), clamped to AWETrim's own `(1.1, 2.3)` m
bound and, below that, to `tos.input_depower_seed_max` when set — because a seed
landing exactly on the hard 2.3 m ceiling leaves the solver no room to move and
fails outright. It exists because the solve runs against a 14° angle-of-attack cap
that is already binding at 6 m/s: more wind needs more tape to stay under it, and
an un-ramped seed starts on the wrong side of the cap. `data/traj_opt.yaml` ships
the measured slope; `TrajOptSettings.input_depower_per_wind` defaults to `0.0`, which
reproduces the pre-ramp behaviour.

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
identically; measured 2026-08-18, they were **offset along the depower axis by 0.12
of `rel_depower`, i.e. 0.61 m of tape**. The 0.4 m calibration offset above accounts
for about two thirds of that, leaving ~0.2 m of genuine aerodynamic disagreement.

That is why `input_depower` is left free and a predicted-vs-measured ratio is a
comparison at two different depower settings — but the fix is a shift, not a rebuild.

That 0.12 offset became code, but the constant has since been lowered to **0.107**
(2026-08-21, commit `ee1ecbd`, "Lower depower offset"); that revision has not been
re-measured or re-derived against the 0.4 m / 0.2 m split above:

```julia
const AWETRIM_V3KITE_DEPOWER_OFFSET = 0.107   # rel_depower units

awetrim_depower_to_v3kite(l_dp) = (l_dp - 0.6) / 5.0 + AWETRIM_V3KITE_DEPOWER_OFFSET
```

(`examples/awetrim_client.jl`). With the numbers in:

$$
u_p = \frac{l_\mathrm{dp} - 0.6}{5.0} + 0.107 \ \ [\mathrm{rel\_depower}]
$$

At the originally measured 0.12, `0.6/5.0` is exactly `0.12`, so the map collapsed to
$u_p = l_\mathrm{dp}/5.0$ — the AWETrim zero-offset (0.4 m) and the genuine
aerodynamic offset (0.2 m) exactly cancelling the `0.6` intercept. That cancellation
was specific to 0.12 and does not hold at 0.107: the current map is
$u_p = l_\mathrm{dp}/5.0 - 0.013$, a small residual intercept rather than an exact
collapse. Two worked points at the current constant: the seed `l_dp = 1.6` m gives
$u_p = 0.307$; the equal-power point measured above, `l_dp = 2.01` m (AWETrim's scale
for $u_p = 0.282$ on *its own* affine map), gives $u_p = 0.389$ — 0.107 more than the
naive `(l_dp-0.6)/5 = 0.282` a reader would get by only undoing AWETrim's own zero
shift and ignoring the aerodynamic disagreement.

This is the FULL correction — do not also subtract the 0.4 m calibration term
separately. `TrajOptSettings.fly_opt_depower` (`false` by default) flies the
optimizer's own depower from the transition phase (3, where reel-out begins)
on, through this conversion, instead of the fixed
`fcs.depower_setpoint` — installed by `simple_opt_reelout.jl` at
startup and after every re-optimization; phase 5 always flies `fcs.depower_final`
regardless of the flag.

**Historical caveat, since fixed:** both AWETrim columns above were originally solved
with its built-in tether of 10 mm / 970 kg/m³ against this repo's 4 mm / 724 kg/m³,
which raised the optimizer's power by 22-25 % and made the 0.61 m equal-power offset
an upper bound rather than the residual. AWETrim's `data/LEI-V3-KITE/system.yaml` was
changed to 4 mm / 724 kg/m³ on 2026-08-18 (`1e27e66`, "Change tether diameter to
4mm"), so both models now solve against the same tether and the mismatch no longer
applies. See [fig8_tuning_log.md](fig8_tuning_log.md).

**The constant above is stale.** `0.107` was superseded on 2026-08-26 (8 mm
tether) and again on 2026-08-29 (`kite.mass` raised from 6.2 kg to 10.9926 kg to
match AWETrim's own wing), each time by re-measuring `power_ratio` at 6 m/s
rather than by re-deriving the worked numbers above. Current value: **0.1010**.
Treat the `AWETRIM_V3KITE_DEPOWER_OFFSET` docstring in `examples/awetrim_client.jl`
as the source of truth, not the number in this file.


