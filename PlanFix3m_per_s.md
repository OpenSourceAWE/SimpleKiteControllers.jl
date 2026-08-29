# Fix simulation at 3 m/s wind speed
Problem:
The dynamic simulation calculates 4 times the power than AWETrim.

Reason:
Different speed/force curve at low forces/ low speeds

## Root cause, traced

At 3 m/s the LOCAL runtime winch (`WinchController`, `rcs.force_limit = "hard"` per
`data/winch_kv_table.yaml`) is a two-limiter switching controller: `LowerForceController`
takes over below `f_low = 350 N` and reels in at its own tuned dynamics, unrelated in
shape to AWETrim's tension curve. AWETrim's optimizer, and its power *prediction*, is
built entirely on its own smooth `T(v) = (v/k_v)²` law (`Winch.tension_curve`,
soft-saturated at `f_min`/`f_max`) — see WinchControllers.jl's `PlanWinchCurve.md`. The
two laws diverge most exactly in the low-force/low-speed region 3 m/s spends most of its
time in, which is why the mismatch is worst there.

`WinchControllers.jl` gained a fix for the LOCAL side of this: `calc_vro_soft`'s new
`soft_lfc` mode (see `PlanWinchCurve.md` there) replaces the `LowerForceController`
entirely with a law built from the SAME tension curve AWETrim uses — a straight
reel-in line through `(0, v_reel_in)`/`(f_low, 0)`, smoothly handed over
(`soft_min`) to the same soft-saturated `√` curve above `f_low`. Enabling it makes the
local runtime curve a much closer match to AWETrim's own, instead of two structurally
different laws.

## Problem found while trying to enable it here

1. **`softminus_beta` can't just be sharpened.** `soft_lfc` requires
   `softminus_beta * f_low >= 8` (here: `>= 0.0229`) so its corner doesn't leave a dead
   band of zero speed above `f_low` — see `calc_vro_soft`'s docstring. But
   `data/winch_kv_table.yaml` in THIS repo documents that raising AWETrim's own
   `softminus_beta` past `1e-3` was already tried and MEASURED to break the 3 m/s
   optimizer solve (422, Max_Iterations_Exceeded, at both `2e-3` and `5e-3` — over 4x
   less sharp than `soft_lfc` needs). Since `winch_from_wc` used to forward
   `rcs.softplus_beta`/`rcs.softminus_beta` straight to AWETrim, sharpening `rcs` for
   `soft_lfc` would ALSO have sharpened what AWETrim solves against, at exactly the
   wind speed the sharpening already broke it.

## Fix: decouple the local law from what AWETrim is sent

The decoupling is now in the code permanently, with no per-run override mechanism.

- **`examples/awetrim_client.jl`** defines `const AWETRIM_SOFTMINUS_BETA = 1e-3` — the
  `softminus_beta` `winch_from_wc` ALWAYS sends to the server, hardcoded there instead of
  read off `wc`. It is used unconditionally by the single `WinchParams` construction, so
  every request (nominal and first-lap alike) is solved against `1e-3`, the value measured
  to converge at 3 m/s, no matter how sharp the local `wc.softminus_beta` is. `softplus_beta`
  is still forwarded from `wc`; only the lower-limit beta needed pinning.
- **`data/wc_settings.yaml`** carries the local `soft_lfc` law directly, so it is what
  every run flies with no REPL setup: `force_limit: "soft"`, `soft_lfc: true`,
  `softminus_beta: 0.03` (sharp enough for the `* f_low >= 8` requirement),
  `v_reel_in: -2.0`, `reel_in_beta: 20.0`. Note `f_low` in this file is now `700.0`.
  Because `force_limit` is set here to `"soft"`, it no longer picks up the `"hard"` entry
  `winch_kv_table.yaml` used to force at 3 m/s, which `soft_lfc` would have thrown on.

## Open issue: power_ratio is 5.1

With the above in place, a 3 m/s run currently reports a `power_ratio` (local/AWETrim
predicted power) of **5.1** — worse than the ~4x mismatch this plan set out to fix, not
better. This needs further investigation before the approach is judged; things to check:

- whether the local `soft_lfc` reel-in law is actually behaving as intended at 3 m/s, or
  whether the reel-in line is now dominating and inflating measured power;
- whether pinning only `softminus_beta` is enough — the server still sees
  `wc.softplus_beta`, and `f_low` differing between the two sides (700 N locally vs. what
  the server's floor `sp(beta*f_min)/beta` effectively makes of it at `1e-3`) may be the
  larger remaining discrepancy;
- whether the mismatch is in the winch law at all, versus the flown path or the aero side.

