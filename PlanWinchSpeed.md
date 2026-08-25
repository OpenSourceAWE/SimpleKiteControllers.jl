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

### Stage 1 — right-size the lower limit

`f_low` 350 -> 700 N in `data/wc_settings.yaml`. Independent of everything else.
Re-run 3 and 4 m/s; expect the `LowerForceController` share to rise at 3 m/s
(currently 20.6%) and the 4 m/s operating point to move off the model floor.

### Stage 2 — get the operating point under `f_max`

Target mean force at or below ~0.8·`f_max`, so a limiter is a backstop rather
than a continuous fight. Levers, in order of preference:

1. **`optimize_k_v`.** The server supports it (`K_V_BRACKET_FACTOR = 2.0`,
   `k_v_bounds`), and this repo does not use it anywhere. Letting the optimizer
   choose $k_v$ subject to its own saturating tension curve is the principled
   fix — it will raise $k_v$ at high wind until the force fits.
2. **Extend `data/winch_kv_table.yaml`.** $k_v$ already rises 0.0408 -> 0.0558
   between 9 and 10 m/s; the 9 m/s value looks too low given a mean force of
   8394 N. Costs power — reeling out faster moves off the optimal operating
   point — so quantify the trade.
3. **More depower at high wind.** `av_depower` is 0.269 at 9–10 m/s against
   0.294 at 3 m/s.

This stage is what actually addresses the oscillation. Stages 3 and 4 are worth
little without it.

### Stage 3 — carry β on the wire

Add `softplus_beta` / `softminus_beta` to `WinchParams` in
`examples/awetrim_client.jl` and send them. Set $\beta^{-} = 5\cdot10^{-3}$ so
the optimizer's floor is 706 N rather than 1103 N. $\beta^{+}$ stays at
$10^{-3}$, where 8000 >> 1000 makes the smoothing harmless.

Without this the server's `setdefault(1e-3)` wins regardless of what is
configured locally, and the controller's inverse silently biases if the server's
default ever changes.

### Stage 4 — `force_limit = "soft" | "hard"`

New `WCSettings` key, separate from `mode`: `mode` selects the law's shape
(`"piecewise"` / `"reelout"`), `force_limit` selects how the limits are
enforced. `"hard"` is the default and reproduces today's behaviour exactly.

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
