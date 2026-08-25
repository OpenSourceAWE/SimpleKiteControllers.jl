# Adapt winch speed calculation

## Why

The runtime `UpperForceController` is switched off (`DISABLE_UFC = true`,
`examples/simple_opt_reelout.jl`) because it oscillated. Consequences:

- The runtime winch has **no upper force limit at all**. Peak winch force reaches
  9418 N at 9 m/s and 11657 N at 10 m/s, against `f_high` 8000 N and a plant
  rated 8400 N.
- The optimizer still solves under the real `f_high`, so it plans against a
  tension curve that saturates at 8000 N while the runtime flies bare
  `kv*sqrt(F)`. The mismatch is by construction.

The goal is force limiting that is **continuous** — no threshold, no hand-over,
no integrator — and that matches the curve the optimizer already assumes.

## The current law

$$v_{set} = k_v \sqrt{F}$$

$F$ is the measured tether force at the winch; $k_v$ is interpolated against
mean wind speed from `data/winch_kv_table.yaml`.

## The proposed law

The optimizer models the winch as a quadratic tension curve with **soft
saturation at both ends** (see Background). The controller measures $F$ and must
command a speed, so it inverts that saturation. With
$\mathrm{sp}^{-1}(y) = \ln(e^{y}-1)$, undoing the lower limit first and then the
upper one:

$$T_1 = T_{\min} + \frac{1}{\beta^{-}} \mathrm{sp}^{-1}(\beta^{-}(F-T_{\min}))$$

$$T_{nom} = T_{\max} - \frac{1}{\beta^{+}} \mathrm{sp}^{-1}(\beta^{+}(T_{\max}-T_1))$$

$$v_{set} = k_v \sqrt{\max(T_{nom}, 0)}$$

This is an exact inverse of the server's forward law, verified by round-trip
against `awetrim/system/winch.py`:

| $v_r$ | 1.0 | 2.0 | 3.0 | 3.75 |
| :-- | :-- | :-- | :-- | :-- |
| forward $T$ [N] | 1176.0 | 2520.4 | 5341.3 | 7506.7 |
| inverse $v$ | 1.0 | 2.0 | 3.0 | 3.75 |

```julia
sp(x)     = max(x, 0) + log1p(exp(-abs(x)))   # ln(1 + eˣ),  overflow-safe
sp_inv(y) = y + log(-expm1(-y))               # ln(eʸ − 1),  accurate for small y

function reel_speed(F; k_v, T_min, T_max, β_lo, β_up, v_max)
    F <= T_min && return 0.0
    F >= T_max && return v_max          # outside the model's range
    T = T_min + sp_inv(β_lo * (F - T_min)) / β_lo   # undo the lower saturation
    T = T_max - sp_inv(β_up * (T_max - T)) / β_up   # undo the upper saturation
    return min(k_v * sqrt(max(T, 0.0)), v_max)
end
```

`v_max` is load-bearing, not a guard: the inverse diverges as $F \to T_{\max}$.
It must be `wc.v_sat`, and it has no safe default.

## Background: the server side

Source: `AWETrim/src/awetrim/system/winch.py` and `server/session.py`.

### From `k_v` to the server's slope

The client sends `WinchParams {mode, k_v, f_min, f_max, v_max | p_max}`. The
server inverts $v_{set} = k_v\sqrt{F}$ into its quadratic radial force model:

$$T(v_r) = k (v_r - v_0)^2, \qquad k = \frac{1}{k_v^2}, \qquad v_0 = 0$$

`session.py` sets `slope_winch_ro = 1/k_v²` and `offset_winch_ro = 0.0`, so the
two forms are exact inverses. Note $k$ and $k_v$ are different quantities in
different units: $k_v$ is (m/s)/√N, $k$ is N/(m/s)².

### The saturation

Write $\mathrm{sp}(x) = \ln(1+e^{x})$. Starting from the nominal
$T = (v_r/k_v)^2$, the two limits are applied in this order:

$$T \leftarrow T_{\max} - \frac{1}{\beta^{+}} \mathrm{sp}(\beta^{+}(T_{\max}-T))$$

$$T \leftarrow T_{\min} + \frac{1}{\beta^{-}} \mathrm{sp}(\beta^{-}(T-T_{\min}))$$

The result is always strictly below $T_{\max}$ and strictly above $T_{\min}$.
$\beta$ has units of 1/N and sets the corner sharpness — the transition spans a
tension band of order $1/\beta$, and **larger $\beta$ = sharper**. As
$\beta \to \infty$ this collapses to $\min(\max(T,T_{\min}),T_{\max})$. At the
speed where the nominal law equals a limit, the saturated law sits $\ln 2/\beta$
below $T_{\max}$ (resp. above $T_{\min}$).

`session.py` enables both (`softplus` and `softminus`) and applies
`setdefault(1e-3)` to each β. **β is not carried by `WinchParams`** — the two
sides agree today only because both happen to say 1e-3.

## What the model can represent

With $\beta = 10^{-3}$ ($1/\beta$ = 1000 N), `f_min` 350 N and `f_max` 8000 N,
the server's tension curve spans **[883, 8000] N**:

| $v_r$ [m/s] | 0 | 1 | 2 | 3 | 4 | 6 | 8 |
| :-- | :-- | :-- | :-- | :-- | :-- | :-- | :-- |
| $T$ [N] | 883 | 1176 | 2520 | 5341 | 7819 | 8000 | 8000 |

The floor is 883 N, not 350 N, because $1/\beta$ is **larger than `f_min`
itself** — the smoothing dominates the limit it smooths. Effective floor
$\mathrm{sp}(\beta f_{\min})/\beta$:

| `f_min` | β=1e-3 | β=2e-3 | β=5e-3 |
| :-- | :-- | :-- | :-- |
| 350 | 883 (+152%) | 552 (+58%) | 382 (+9%) |
| 700 | 1103 (+58%) | 810 (+16%) | **706 (+1%)** |
| 2000 | 2127 (+6%) | 2009 (+0%) | 2000 (+0%) |

`f_low = 350` is a leftover from a 4000 N winch (`WinchControllers`' own module
docstring still describes a "20 kW winch, 4000 N max force"). Scaled to this
winch it is ~700 N. Raising `f_min` alone is not enough: at β=1e-3 even 700 N
gives an 1103 N floor. **Both `f_min` and $\beta^{-}$ have to move.**

Measured operating points against that range (`output/scenarios/overview.md`):

| v_wind | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| :-- | :-- | :-- | :-- | :-- | :-- | :-- | :-- | :-- |
| av force [N] | 585 | 1350 | 2406 | 3628 | 5031 | 6627 | 8394 | 8150 |
| power_ratio | **1.44** | 1.02 | 1.01 | 1.00 | 0.98 | 0.98 | 0.97 | 0.98 |

3 m/s is the only wind speed whose entire force range lies below the 883 N
floor, and it is the only one where the optimizer's power prediction is badly
wrong. 9 and 10 m/s sit **above** `f_max`, which the model cannot represent
either.

## Why this alone will not stop the oscillation

The soft law removes the *switching* that made the `UpperForceController`
chatter, but it replaces it with high static gain in the same region. Loop gain,
using a crosswind plant estimate $|dF/dv_r| \approx 2F/(v_w - v_r)$:

| v_wind | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| :-- | :-- | :-- | :-- | :-- | :-- | :-- | :-- | :-- |
| nominal $k_v\sqrt{F}$ | 0.47 | 0.59 | 0.66 | 0.69 | 0.69 | 0.70 | 0.70 | 0.72 |
| soft law | 0.00 | **1.16** | 0.78 | 0.72 | 0.73 | **0.92** | railed | railed |

The nominal law sits at 0.5–0.7 everywhere, which is why it is stable. The soft
law exceeds 1 at 4 m/s (1350 N is just above the 883 N floor, where the inverse
is steep), approaches 1 at 8 m/s, and at 9–10 m/s the mean force is *above*
`f_max`, so the command is pinned at `v_sat` while the per-lap force ripple
(std 866 N at 9 m/s) drags it in and out of the near-vertical region.

The root cause of the original oscillation is therefore **not** the UFC's
switching structure — it is that the mean operating force at 9–10 m/s is at or
above `f_max`. No limiter can hold a force the path is continuously asking to
exceed; it can only fight it every lap. The soft law relocates that fight, it
does not remove it.

The soft law is still the better structure — static, continuous, no integrator,
no flip-flop, and consistent with the optimizer's own model — but it must be
built **on top of** an operating point that fits inside the limit, not instead
of one.

## Plan

### Stage 1 — right-size the lower limit — DONE (2026-08-25)

Shipped as a WIND-DEPENDENT `f_low`, not the flat 350 -> 700 N this plan first
proposed: 350 N at 3 m/s and 700 N from 4 m/s up, interpolated, in the `f_low`
column of `data/winch_kv_table.yaml` beside `kv`, read by `winch_f_low(v_wind)`.
One flat value cannot serve the range — the winch's rating does not move with
the wind and the kite's force goes with its square. A flat 700 N passed 4 m/s
(limiter share 1.1 %) and FAILED 3 m/s (63.2 %, mean force pinned at 725 N
against the floor, `max d` 8.02°). Both ends now pass; 5–10 m/s are inert, their
limiter share was already 0.0.

It also turned up a bug this plan had not anticipated: `f_low` was **two limits
sharing a name**. The entry guard (`guard_lfc`, phases 0-2) took the same
setpoint, and 700 N is unreachable during the depowered dive, so its PID wound
up and ran the drum to -6.3 m/s until the tether snapped taut at 7912 N and the
solver failed. Split into `FC_Settings.entry_f_min` (350 N), which is bounded by
what a DEPOWERED wing can pull and does not scale with the winch. Full
measurements in `docs/fig8_tuning_log.md`.

### Stage 2 — get the operating point under `f_max` — DONE (2026-08-25)

Target mean force at or below ~0.8·`f_max`, so a limiter is a backstop rather
than a continuous fight. This stage is what actually addresses the oscillation;
stages 3 and 4 are worth little without it.

**`optimize_k_v`, and that is the whole stage.** $k_v$ goes out as a design
variable, the server brackets it a factor `K_V_BRACKET_FACTOR` (2.0) either side
of the value sent and solves for it under its own saturating tension curve, and
the run flies the gain that comes back — a path solved for one gain is not
flyable with another. Reeling out faster sheds force, so at high wind the
optimizer raises $k_v$ until the path fits inside the `f_max` it was given.
`TrajOptSettings.optimize_k_v`, on in `data/traj_opt.yaml`, off in the struct
default.

The two hand-tuning levers this plan originally listed beside it are **CLOSED**,
because the optimizer now sets both values itself:

- ~~Extend `data/winch_kv_table.yaml`~~ — $k_v$ is the optimizer's to choose.
  The table is NOT dead and must not be deleted: its `kv` column still supplies
  the **seed** the server brackets around, and `simple_reelout.jl` (the
  lemniscate baseline, which never talks to the optimizer) still flies it flat.
  Its `f_low` column is fully live either way — that is Stage 1's, not this
  one's.
- ~~More depower at high wind~~ — already optimized and already read back:
  `depower_mode: "optimize"` in the request, and
  `optimized_parameters["input_depower"]` lands in `depower_flown_opt` on every
  converged reply. Hand-tuning `depower_setpoint` against it would fight the
  solver, not help it.

What is left undone here, deliberately: **`k_v_bounds` is not implemented.** The
server's own factor-2 bracket applies. A reply that ran into it is reported as
`k_v_at_bound` in the summary and warned about at runtime — that is the signal
to widen the bracket, and the reason to add the field is a measurement, not a
guess.

**NOT YET VALIDLY MEASURED.** The 9 m/s run of 2026-08-25 20:43 flew with a bug
and its numbers say nothing about this stage: `apply_optimized_kv!` wrote the
gain to `wc.kv`, but with `DISABLE_UFC` set the controller is built from a
DEEPCOPY (`rc_settings = deepcopy(rcs)`, to raise `f_high` on a copy) and
`calc_vro` reads that copy. `rc.wcs === wc` was false, so the optimized gain
never reached the winch. Power (30137 W against the baseline's 30260) and
reel-out speed (2.91 m/s against 2.92) were unchanged because **nothing was
applied**, not because the optimizer agreed with the seed. Fixed 2026-08-25:
the gain is now written to `rc.wcs` as well, and `k_v_flown` reports the object
`calc_vro` actually read.

What the run DOES show, from `opt_kv_log`, is that the premise above is the wrong
way round. The optimizer did not raise $k_v$ — it drove it to the **lower** edge
of its bracket and sat there:

| t [s] | L [m] | $k_v$ | at bound |
|--:|--:|--:|:--|
| 0.0 | 150.0 | 0.02265 | |
| 36.0 | 188.0 | 0.02159 | |
| 45.6 | 223.7 | 0.02043 | |
| 56.0 | 262.8 | **0.02041** | **yes** |
| 56.0 | 262.8 | 0.03985 | |
| 72.1 | 323.3 | 0.02059 | |
| 72.1 | 323.3 | 0.04078 | |
| 84.7 | 370.8 | 0.04078 | |

The bracket is 0.0204 .. 0.0816 around the 0.0408 seed. For the first half of the
run the optimizer wanted roughly HALF the seed gain — to reel out slower, not
faster — and was held off by the bound. So:

- **`k_v_bounds` IS wanted** after all. An earlier note here dismissed it by
  reading only the final value; the bracket demonstrably binds.
- The pairs sharing a timestamp (56.0, 72.1) are the two call sites firing inside
  one re-optimization — the first reply, then the blend-fold retry. The first of
  each pair is a candidate that was REJECTED and still moved the gain, because
  both read-backs sit before the gates, where `input_depower`'s already was. That
  is a bug for depower too, not just for $k_v$.

**Re-measured on the fixed code, and `optimize_k_v` is OFF again as a result.**
The gain reached the winch this time — reel-out speed fell from 3.66 to 2.30 m/s
over the window where the optimizer held $k_v$ near its lower bound — and the
result is worse on both axes that matter:

| | `optimize_k_v` (fixed) | v09, fixed $k_v$ |
|:--|--:|--:|
| mean force, settled | 9116 +- 1522 N | 8070 +- 866 N |
| mean force, t = 30 .. 43 s | **11680 N** | 8068 N |
| **peak force** | **12684 N** | 9418 N |
| mean `v_ro`, t = 30 .. 43 s | 2.30 m/s | 3.66 m/s |
| measured power | 29581 W | 30260 W |
| verdict | all 9 passed | all 9 passed |

12684 N against a plant rated 8400 N — 51 % over — and 2 % LESS power. Every
success criterion still passed, because they score tracking and pattern geometry
and nothing scores force against the winch's rating.

The mechanism is this plan's own subject, now with teeth. The optimizer's model
is $T = (v_r/k_v)^2$ **soft-capped at `f_max`**: with a low $k_v$ the nominal
force is enormous, the softplus clamps it to ~8000 N, and the solver concludes
the winch will hold 8000 N at low speed for free. The plant has NO upper force
limit at all — `DISABLE_UFC` — so nothing clamps anything and the force runs to
12.7 kN. Reeling out slower does not shed force, it builds it.

So `optimize_k_v` hands the optimizer a freedom that is only safe when something
actually caps the force, and it is set `false` in `data/traj_opt.yaml` until that
is true. The capability stays implemented and tested.

**This reorders the plan: stage 4 (`force_limit`) is a PREREQUISITE for stage 2,
not a follow-up.** Enabling `optimize_k_v` again is only defensible either after
a working limiter, or paired with an `f_max` low enough that even an unlimited
plant stays under 8400 N — a deliberate experiment, not a resting state.

Stage 4 is now implemented, shipped and measured, and it does NOT clear this gate.
At 9 m/s it cut time above the plant's 8400 N rating from 58 % of the reeling
window to 13 %, but peak force only fell 9418 -> 9166 N and it rings at +-1.1 m/s.
`optimize_k_v` needs a limiter that is STABLE, not merely present: a solver free to
lower `k_v` under a limiter that already oscillates has more room to build force,
not less. The gate stays a measured run with peak force under 8400 N.

### Stage 2b — candidate levers, once Stage 2 has a valid measurement

1. **Widen `k_v_bounds`.** The factor-2 bracket bound for half the run. Add the
   field to `WinchParams` and send an explicit bracket, so what the optimizer
   wants is visible rather than clipped.
2. **Move both read-backs behind the gates**, so a rejected candidate stops
   moving $k_v$ and `input_depower`.
3. **`f_max` as the operating-point lever**, if the fixed run still rides the
   ceiling. Send an `f_max` below the winch's real limit and let the optimizer
   re-solve beneath it — a direct trade against power, not a free fix. The
   argument for it is SAFETY rather than tracking: peak force is 9287 N at 9 m/s
   and 11657 N at 10, against a plant rated 8400 N, while every wind speed passes
   its criteria today. `f_high` in `wc_settings.yaml` feeds both the request and
   the runtime limiter through `winch_from_wc`, so the two move together by
   construction.

### Stage 3 — carry β on the wire

Add `softplus_beta` / `softminus_beta` to `WinchParams` in
`examples/awetrim_client.jl` and send them. Set $\beta^{-} = 5\cdot10^{-3}$ so
the optimizer's floor is 706 N rather than 1103 N. $\beta^{+}$ stays at
$10^{-3}$, where 8000 >> 1000 makes the smoothing harmless.

Without this the server's `setdefault(1e-3)` wins regardless of what is
configured locally, and the controller's inverse silently biases if the server's
default ever changes.

### Stage 4 — `force_limit = "soft" | "hard"` — DONE and MEASURED (2026-08-25)

New `WCSettings` key, separate from `mode`: `mode` selects the law's shape
(`"piecewise"` / `"reelout"`), `force_limit` selects how the limits are
enforced. `"hard"` is the struct default and reproduces today's behaviour
exactly; `data/wc_settings.yaml` ships `"soft"`.

Shipped in WinchControllers (local checkout, `0.6.0`):

- `calc_vro_soft(wcs, force, f_low)` in `wc_components.jl`, the exact inverse of
  the plan's forward law. Round-trips against an independently written forward
  curve in `test/test_soft_limit.jl` (49 assertions), and reproduces the plan's
  table to the digit: 1176.0 / 2520.4 / 5341.3 / 7506.7 N at 1 / 2 / 3 / 3.75 m/s,
  883.2 N floor.
- `calc_vro` is UNCHANGED and stays the nominal law. Both `v_sw` call sites read
  it under either setting, which is what the implementation note demanded.
- `WCSettings.force_limit`, `softplus_beta`, `softminus_beta`, `force_limit_tau`,
  all at the end of the block.
- `LowPass` in `components.jl`, the force filter, `tau = 0` a pass-through. It
  lives in `CalcVSetIn` and only runs on the soft path.
- The `UpperForceController` is held in reset by the `WinchController` constructor
  under `"soft"` and its `nlsolve` is skipped every timestep. The
  `LowerForceController` is untouched.
- `WinchController` errors on `force_limit = "soft"` with `mode = "piecewise"`:
  the inverse is of the reel-out law and means nothing against the other shape.

And in this repo: `DISABLE_UFC` is gone from `examples/simple_opt_reelout.jl`
(superseded — `rc` is now built from `rcs` itself, so the deepcopy that swallowed
the optimized `k_v` cannot come back), the summary carries `winch.force_limit`,
and `winch_state.upper_force_pct` is 0 by construction on this setting — noted at
`fig8_metrics.jl`'s `winch_state_pct` and at `optimize_fig8.jl`'s side condition,
which the setting makes vacuous.

**Measured at 9 m/s** (`output/archives/2026-08-25_221525` against
`output/scenarios/v09`; same wind, no turbulence either side, same `k_v`,
`optimize_k_v` off in both — the limiter is the only difference). Full numbers in
`docs/fig8_tuning_log.md`; the verdict is that it works on force and oscillates:

| mid-run window, 31 s | `"soft"` | v09 (`"hard"`, UFC off) |
|:--|--:|--:|
| mean force | **7459** +- 1076 N | 8521 +- 188 N |
| % over the plant's 8400 N | **13.0 %** | 73.7 % |
| `v_ro` | 4.21 +- **1.10** m/s | 3.77 +- **0.05** m/s |
| `v_ro` swing period | **2.63 s** | none |

Whole run: peak force 9166 N (was 9418), 13.3 % of the reeling window over 8400 N
(was 58.1), power 29279 W (was 30260, -3.2 %), ringing `zeta` 0.005 (was 0.072),
all 9 criteria passed both times.

So the section "Why this alone will not stop the oscillation" was right, and is now
measured. The soft law removed the sustained overload — that part is a big, real
win — and relocated the fight into a +-1.1 m/s limit cycle at a fixed 2.63 s
period, running the whole window rather than switching on and off. Peak force
barely moved, because the oscillation makes its own peaks.

The mechanism is the law's own local gain: `dv/dF` is 0.53 (m/s)/kN at the flown
7459 N mean against the nominal law's 0.24, 2.1 at 7900 N and 17.2 at 7990 N, so a
1076 N std sweeps through the near-vertical region every lap.

`softplus_beta` is NOT the lever — a softer corner raises `dv/dF` at this operating
point (0.53 -> 0.85 at 5e-4 -> 1.29 at 2.5e-4) because it starts bending earlier.

**`force_limit_tau: 0.2` was tried and is CLOSED** (2026-08-25,
`output/archives/2026-08-25_223245`). The idea was that the 1 s filter supplied the
phase closing the loop. It did the opposite: `v_ro` swing +-2.12 m/s (from +-1.10),
peak force 9900 N (from 9166, and worse than the baseline's 9418), `v_ro` peak
8.09 m/s hard against `v_sat`, power 27671 W (from 29279), `zeta` 0.001. The
diagnostic is that the ring got SLOWER (2.63 -> 3.55 s): the filter was attenuating
loop GAIN, not adding the destabilising phase. Reverted to 1.0.

Left, in order: **`force_limit_tau` LONGER** (2-3 s — the direction the data points,
bounded by the ~8 s lap period), then **stage 2b item 3, lowering `f_high`**. The
second is not self-evident either: the knee sits AT `f_high`, so lowering it moves
the knee down with the operating point, and it only helps if the optimizer re-solves
onto a path that pulls enough less to leave margin.

## Implementation notes

- **New fields go at the end of the `@with_kw WCSettings` block.** The positional
  `WCSettings(update; dt)` constructor that KiteControllers' tests use keeps
  working, and YAML falls back to struct defaults for missing keys.
  KiteControllers pins `WinchControllers = "0.5.4"` against this repo's `"0.6"`,
  so it will not see the change until that bound is widened.
- **`v_sw` must keep using the nominal `kv*sqrt(F)` under both settings.**
  `calc_v_set` evaluates the law at *exactly* `f_low` and `f_high` every timestep
  to set the force controllers' hand-over speeds
  (`winchcontroller.jl:99-100`, and `:58` in the constructor). The soft law
  returns **0** at `f_low` and **∞** at `f_high` — the two worst possible values.
  Same call at `examples/simple_reelout.jl:507`.
- **Under `"soft"` the `UpperForceController` is off by design** — the soft law
  is the upper limiter. `DISABLE_UFC` is then superseded and should be removed.
- **The `LowerForceController` stays.** The soft law never commands reel-in; only
  the `lfc` does, and 3 m/s needs it.
- **Filter the force before inverting.** `examples/simple_reelout.jl:428` passes
  `winch_force(s)` raw. `dv/dF` rises from 0.30 (m/s)/kN at 6 kN to
  17.5 (m/s)/kN at 7990 N, against a force std of 866 N at 9 m/s.
- **`T_min` must track `calc_v_set`'s `f_low` argument**, which is per-call, not
  `wcs.f_low`.
- β, `f_min`, `f_max` and `k_v` must come from the same place that fills
  `WinchParams`, so the two sides cannot drift apart.

## Validation

Compare against **UFC-enabled** runs, not the archived scenarios — those were all
flown with the upper limiter disabled and have no ceiling. 3, 4, 9 and 10 m/s are
the wind speeds that discriminate; 5–8 m/s will look identical either way.

## Open questions

- Is the 883 N floor worth reporting upstream? `min_tether_force = 350` with
  `softminus_beta = 1e-3` gives an effective floor 2.5x the requested value, and
  the optimizer plans against it. The in-repo AWETrim examples use
  `min_tether_force: 1500`, where the artifact is much smaller.
- Does raising $k_v$ at 9 m/s (stage 2) cost more power than the force headroom
  is worth? That trade has not been measured.
