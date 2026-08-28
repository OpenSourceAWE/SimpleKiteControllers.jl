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

- **`examples/awetrim_client.jl`**: `winch_from_wc` now takes `softplus_beta`/
  `softminus_beta` as separate, explicit keyword arguments (still defaulting to
  `wc.softplus_beta`/`wc.softminus_beta`, so any caller that doesn't pass them keeps
  today's behaviour exactly).
- **`examples/simple_opt_reelout.jl`**: right after `rcs = wc`, `AWETRIM_SOFTPLUS_BETA`/
  `AWETRIM_SOFTMINUS_BETA` capture the betas BEFORE any local override can touch them.
  Both `winch_from_wc(rcs; ...)` call sites (the nominal request and the first-lap one)
  now pass these captured constants explicitly, never `rcs.softplus_beta`/
  `rcs.softminus_beta` directly — so whatever the local law does, AWETrim always still
  sees `1e-3`/`1e-3`, the values already known to converge.
- A new `WCS_OVERRIDES` dict (mirroring the existing `FCS_OVERRIDES` pattern: read and
  cleared on every run, so nothing lingers into the next interactive `include`) lets a
  run set arbitrary `WCSettings` fields on `rcs` for the LOCAL simulation only:

  ```julia
  WCS_OVERRIDES = Dict(:force_limit => "soft", :soft_lfc => true,
                        :softminus_beta => 0.03, :v_reel_in => -2.0,
                        :reel_in_beta => 20.0)
  include("examples/simple_opt_reelout.jl")
  ```

  `:force_limit => "soft"` is required in the override at 3 m/s specifically, since
  `winch_kv_table.yaml` sets `"hard"` there and `soft_lfc` needs `force_limit == "soft"`
  (`WinchController` throws otherwise — a loud failure, not a silent no-op, if the
  override is incomplete).

## Verified so far / still open

Checked without running the full (multi-minute, AWETrim-calling) script: both edited
files parse cleanly, and the decoupling logic was verified directly against
WinchControllers.jl — the local `rcs` ends up with the sharpened `soft_lfc` settings
(`WinchController` constructs fine), while the captured AWETrim-facing betas stay at the
original `1e-3`/`1e-3` regardless.

**Not yet done**: an actual 3 m/s run with `WCS_OVERRIDES` set, to check whether the
local/AWETrim power mismatch is actually closed (or by how much) — that is the real test
of this fix and hasn't been run yet.

