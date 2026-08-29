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

With the above in place, a 3 m/s run reports a `power_ratio` (measured/predicted reel-out
power) of **5.1** — `measured_W: 399` against `predicted_W: 78`, and the path installed
before the run was predicted at `-85 W` (`output/scenarios/v03/reelout_150m_opt.yaml`).
Worse than the ~4x this plan set out to fix, not better.

## Investigated: the server is NOT flying the soft_lfc law

**Verified: no.** `use_awe_trim` is `0.0` — the struct default in
`examples/awetrim_client.jl` (`WinchParams`), the value in `data/wc_settings.yaml`, and it
is overridden nowhere: `simple_opt_reelout.jl` overrides only `wc.kv`, `wc.f_low` and
`wc.force_limit` per wind speed from `winch_kv_table.yaml`. `winch_from_wc` forwards that
`0.0` straight through, and in AWETrim's `Winch.tension_curve`
(`src/awetrim/system/winch.py:153`) the whole reel-in branch sits behind `if use_awe_trim:`.
The archived `wc_settings.yaml` of the 5.1 run confirms the asymmetry it was flown with:
`soft_lfc: true` locally, `use_awe_trim: 0.0` towards the server.

That also resolves the apparent contradiction. `soft_lfc` really does ignore
`softminus_beta` (`calc_vro_soft` passes `Inf` and hard-clamps at `f_low`) — but the server
is not running that law. `session.py` sets `"softminus": True` unconditionally on every
request, so `softminus_beta` is the server's *entire* lower-force shaping. Sharpening it
still breaks IPOPT because it still shapes the only lower corner the server has.

**The pin is the problem, not the cure.** The server's effective force floor is
`sp(beta*f_min)/beta`: at `beta = 1e-3` and the `f_min = 350 N` sent at 3 m/s that is
**883 N** — and it cannot go below `log(2)/beta = 693 N` for *any* `f_min`. The measured
3 m/s run spends its whole life under that: force `516 +- 180 N`, range `276..1111 N`.
So the optimizer's winch model commands a standstill across almost the entire force range
the run actually flies, which is exactly why it predicts ~78 W against 399 W harvested.
The two laws AS THEY WERE, at the 3 m/s parametrization (`kv = 0.0408`, `f_low = 350`,
`f_high = 8000`, `v_reel_in = -2.0`, `reel_in_beta = 20`) — the third column is the
reel-in law the old code reached only at `use_awe_trim = 1`:

| force [N] | local `soft_lfc` v_ro | server, `use_awe_trim = 0` | server, reel-in law |
| ---: | ---: | ---: | ---: |
| 350 | -0.04 | 0.00 | 0.00 |
| 500 | 0.84 | 0.00 | 0.86 |
| 700 | 1.08 | 0.00 | 1.08 |
| 900 | 1.22 | 0.26 | 1.23 |
| 1200 | 1.41 | 1.04 | 1.41 |

The floor cannot fall below `log(2)/beta = 693 N` for ANY
`f_min` — that is what makes it unfixable from the client side. The archived runs confirm
it is the whole story: the mismatch appears exactly where the flown force drops under the
floor, and nowhere else.

| run | min force [N] | server floor [N] | measured/predicted |
| ---: | ---: | ---: | ---: |
| v03 | 276 | 883 | **5.1** |
| v04 | 814 | 1103 | 1.07 |
| v05 | 1751 | 1103 | 1.01 |
| v06-v08 | 2436+ | 1103 | 1.00 |

## Fix applied: the server law IS the soft_lfc law

Setting `use_awe_trim: 1.0` would have worked, but it is the wrong shape of fix — it
leaves every request that forgets the flag planning against an 883 N floor. Instead the
reel-in law is now what a quadratic winch does, in `AWETrim/src/awetrim/system/winch.py`:

- `Winch.tension_curve` builds the reel-in line + smooth-maximum handover for every
  `force_model: "quadratic"` request with `min_tether_force > 0`, and **returns it
  instead of** the softminus-floored law. It is no longer behind `if use_awe_trim:`.
- `softminus` is not applied on that path — replaced, not stacked, so the 693 N floor
  cannot come back through it.
- `min_tether_force == 0` keeps the plain law: the reel-in line's two defining points
  collapse there, and `m = -v_reel_in / min_tf` would divide by zero.
- `use_awe_trim` is still accepted and range-validated so no client breaks, but it selects
  nothing. `schemas.py` says so, and `softminus_beta`'s description now carries the
  floor warning.

This is exactly why sharpening `softminus_beta` was never the answer: the corner at
`f_low` needs no smoothing in the forward direction, because the quadratic term has zero
(not infinite) slope at its own zero, so the smooth maximum is well conditioned at any
`reel_in_beta > 0`. `AWETRIM_SOFTMINUS_BETA = 1e-3` can stay pinned; it is now inert for
these requests.

Verified: AWETrim's suite is green (765 passed), and against the actual request
`session.py` builds at 3 m/s the server's force at zero speed is now **349.5 N** (was
883.2), with the server and local `calc_vro_soft` curves agreeing to **< 0.02 m/s** over
350-3000 N.

**Not yet done**: rerun 3 m/s against the rebuilt server and check `power_ratio`. Rerun
v04-v08 too — they were at ~1.0 with the old law, and v04 (min force 814 N, partly under
its 1103 N floor) should improve rather than regress, but that is a prediction, not a
measurement.

## Separate lead: v09-v11 sit at 0.84-0.92

Not this bug. Those runs are at ~6700 N mean force, near `f_high = 8000`, where the two
sides handle the softplus cap differently — the server's forward `T - sp(beta*(T-f_max))/beta`
against `calc_vro_soft`'s inverse `sp_inv`. At 7000 N that is a 0.19 m/s gap, against
0.001 m/s in the mid range. Pre-existing, unchanged by the fix above, worth its own look.

# Findings I
Not working with use_awe_trim = 0.0
[ Info: warmup: 2.00 s (180 steps) with the winch under the caller's controller...
[ Info: Run: 521 s at dt = 0.0111 s (46847 steps).
[ Info: Winch: REEL_OUT mode — soft force limit inverting kv = 0.0408 saturated at [350, 8000] N (beta 3e-02/1e-03, force filtered at tau = 2.48 s); the UpperForceController is held in reset, stopping at 380 m.
[ Info: Optimizer conditions: 3.0 m/s at 6 m from 270°, profile_law 3, z0 = 0.0002 m | winch kv = 0.0408, i.e. 3.6 m/s at f_high = 8000 N | depower seed 1.600 m.
[ Info: Initial guess: 30° x 12° at 29°, 361 points.
[ Info: Constraints sent with the request: min_turn_radius 13.14 m (min_feasibility_margin 0.82 x the kite's own, x 1.418 for 20 m of assumed reel-out per lap and 1.25 of headroom), pattern box PatternLimits(nothing, 15.466014791351698, nothing, nothing, 10.0).
[ Info: POST /step -> HTTP 422: optimization infeasible or not converged: RuntimeError: Phase residual solve failed at the first node; no valid trajectory states were generated
ERROR: LoadError: The optimizer returned no path: {"detail":"optimization infeasible or not converged: RuntimeError: Phase residual solve failed at the first node; no valid trajectory states were generated"}

Three candidates, most likely first:
  * the INITIAL GUESS is too far from the optimum for IPOPT to reach it. Here that is guess_a = 30.0°, guess_b = 12.0°, guess_el_center = 29.0° of data/traj_opt.yaml, which seeds the request and nothing else — widening or raising it changes the guess, not the flown path. Measured at 150 m and 6 m/s: 20°/11° at 18° does not converge, 30°/12° and 20°/11°-at-26° do.
  * the winch is too stiff to reel out at the optimum: kv*sqrt(f_high) = 3.6 m/s against 3.0 m/s of wind at 6 m.
  * these conditions genuinely have no solution.

`bin/run_server log` carries the solver's own output.

## Findings II
With use_awe_trim=1.0 it is not working:

[ Info: Constraints sent with the request: min_turn_radius 13.14 m (min_feasibility_margin 0.82 x the kite's own, x 1.418 for 20 m of assumed reel-out per lap and 1.25 of headroom), pattern box PatternLimits(nothing, 15.466014791351698, nothing, nothing, 10.0).
[ Info: POST /step -> HTTP 422: optimization infeasible or not converged: RuntimeError: Phase residual solve failed at the first node; no valid trajectory states were generated
ERROR: LoadError: The optimizer returned no path: {"detail":"optimization infeasible or not converged: RuntimeError: Phase residual solve failed at the first node; no valid trajectory states were generated"}

Three candidates, most likely first:
  * the INITIAL GUESS is too far from the optimum for IPOPT to reach it. Here that is guess_a = 30.0°, guess_b = 12.0°, guess_el_center = 29.0° of data/traj_opt.yaml, which seeds the request and nothing else — widening or raising it changes the guess, not the flown path. Measured at 150 m and 6 m/s: 20°/11° at 18° does not converge, 30°/12° and 20°/11°-at-26° do.
  * the winch is too stiff to reel out at the optimum: kv*sqrt(f_high) = 3.6 m/s against 3.0 m/s of wind at 6 m.
  * these conditions genuinely have no solution.

`bin/run_server log` carries the solver's own output.
Stacktrace:
  [1] error(s::String)
    @ Base ./error.jl:44
  [2] top-level scope
    @ ~/repos/SimpleKiteControllers.jl/examples/simple_opt_reelout.jl:660
  [3] include(mapexpr::Function, mod::Module, _path::String)
    @ Base ./Base.jl:307
  [4] IncludeInto
    @ ./Base.jl:308 [inlined]
  [5] example_menu()
    @ Main ~/repos/SimpleKiteControllers.jl/examples/menu.jl:79
  [6] top-level scope
    @ ~/repos/SimpleKiteControllers.jl/examples/menu.jl:83
  [7] include(mapexpr::Function, mod::Module, _path::String)
    @ Base ./Base.jl:307
  [8] IncludeInto
    @ ./Base.jl:308 [inlined]
  [9] menu()
    @ Main ./none:1
 [10] top-level scope
    @ REPL[2]:1
in expression starting at /home/ufechner/repos/SimpleKiteControllers.jl/examples/simple_opt_reelout.jl:655
in expression starting at /home/ufechner/repos/SimpleKiteControllers.jl/examples/menu.jl:83

caused by: http status error: 422 for POST http://127.0.0.1:8000/step
Stacktrace:
  [1] request(method::String, url::String, h::Vector{…}, b::Nothing; headers::Vector{…}, body::String, basicauth::Nothing, retry::Bool, retries::Int64, retry_non_idempotent::Bool, retry_if::Nothing, respect_retry_after::Bool, retry_bucket::Bool, status_exception::Bool, redirect::Bool, redirect_limit::Nothing, redirect_method::Nothing, forwardheaders::Bool, proxy::HTTP._UseTransportProxy, cookies::Bool, cookiejar::Nothing, query::Nothing, response_stream::Nothing, decompress::Nothing, max_decompressed_size::Int64, sse_callback::Nothing, max_sse_line_bytes::Int64, max_sse_event_bytes::Int64, client::Nothing, context::Nothing, connect_timeout::Nothing, request_timeout::Int64, response_header_timeout::Int64, read_idle_timeout::Int64, write_idle_timeout::Int64, expect_continue_timeout::Nothing, readtimeout::Nothing, copyheaders::Nothing, pool::Nothing, canonicalize_headers::Nothing, detect_content_type::Nothing, observelayers::Nothing, retry_delays::Nothing, retry_check::Nothing, sslconfig::Nothing, socket_type_tls::Nothing, logerrors::Nothing, logtag::Nothing, trace::Nothing, verbose::Nothing, require_ssl_verification::Bool, protocol::Symbol)
    @ HTTP ~/.julia/packages/HTTP/r4KEz/src/http_client.jl:2130
  [2] request
    @ ~/.julia/packages/HTTP/r4KEz/src/http_client.jl:1964 [inlined]
  [3] #post#130
    @ ~/.julia/packages/HTTP/r4KEz/src/http_client.jl:2411 [inlined]
  [4] post(path::String, payload::Dict{String, Any}; url::String, timeout::Int64, attempts::Int64)
    @ Main ~/repos/SimpleKiteControllers.jl/examples/awetrim_client.jl:560
  [5] post
    @ ~/repos/SimpleKiteControllers.jl/examples/awetrim_client.jl:540 [inlined]
  [6] opt_step(params::StepParams; url::String, inflow_conditions::Nothing, wait::Bool, max_iter::Nothing)
    @ Main ~/repos/SimpleKiteControllers.jl/examples/awetrim_client.jl:702
  [7] top-level scope
    @ ~/repos/SimpleKiteControllers.jl/examples/simple_opt_reelout.jl:656
  [8] include(mapexpr::Function, mod::Module, _path::String)
    @ Base ./Base.jl:307
  [9] IncludeInto
    @ ./Base.jl:308 [inlined]
 [10] example_menu()
    @ Main ~/repos/SimpleKiteControllers.jl/examples/menu.jl:79
 [11] top-level scope
    @ ~/repos/SimpleKiteControllers.jl/examples/menu.jl:83
 [12] include(mapexpr::Function, mod::Module, _path::String)
    @ Base ./Base.jl:307
 [13] IncludeInto
    @ ./Base.jl:308 [inlined]
 [14] menu()
    @ Main ./none:1
 [15] top-level scope
    @ REPL[2]:1
Some type information was truncated. Use `show(err)` to see complete types.

# Investigated: Findings I and II

**They are one data point, not two.** After the fix above `use_awe_trim` selects nothing,
so the `0.0` run and the `1.0` run exercised the same reel-in law and failed identically.
The server was running the new code for both (`winch.py` edited 13:01, server started
13:06).

Reproduced standalone by POSTing the same `/init` + `/step` at 3 m/s, 150 m, the same
30x12-at-29 361-point seed, `min_turn_radius 13.14`, `elevation_min 15.466`,
`elevation_amplitude_max 10.0`. The failure is exact and repeatable.

## It is governed by ONE number: the winch's force at zero reel speed

At 3 m/s, sweeping `f_min` under the reel-in law (where the zero-speed force IS `f_min`):

| f_min [N] | 350 | 700 | 750 | 800 | 820 | 850 | 900 | 1000 | 1200 | 1500 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| result | fail | fail | fail | fail | fail | **OK** | OK | OK | OK | IPOPT |

A hard threshold between **820 and 850 N**. Everything below it fails the node-0 residual
solve; 1500 N fails differently (IPOPT — genuinely too stiff).

Ruled out, all measured, all still failing at `f_min = 350`:

- **not the reel-in law's shape** — `v_reel_in = -0.5` and `-0.2` (line slope 700 and
  1750 N/(m/s), steeper than the 425 that converges at 850 N) and `reel_in_beta = 1.0`
  all fail. The line slope is not the variable; `f_min` is.
- **not the kite side** — `el_center` 20 and 15, `input_depower` 1.0, a wider `a = 40`
  pattern: no effect.
- **not reel-out speed** — `v_max` 1.0 and 0.5 change nothing. `v_max` bounds the NLP, not
  the node-0 solve, which uses `DEFAULT_BOUNDS["speed_radial"] = (-10, 10)`.
- **not the reel-in law at all** — the PLAIN law with a 693 N floor
  (`f_min = 0`, `softminus_beta = 1e-3`) also fails.

## The 883 N floor was accidentally load-bearing

This is the real finding. The old law converged at 3 m/s only because softminus inflated
`f_min = 350` into an **883 N** floor — just past the ~830 N threshold. Reproduced
exactly, plain law, `f_min = 0` with beta chosen to place the floor:

| floor [N] | 693 | 883 (= OLD law at f_min 350) | 1103 (= OLD law at f_min 700) |
| --- | --- | --- | --- |
| result | fail | **OK** | OK |

So the two symptoms are one fact. AWETrim can only solve 3 m/s by pretending the winch
holds >= ~830 N — and that is exactly why it predicted 78 W against 399 W measured, since
the run actually flies at `516 +- 180 N`. There is no `f_low` that both converges AND
describes the physics: the values that converge are above anything the kite pulls at
3 m/s.

## Other wind speeds are NOT affected

Under the new law, at the `f_min = 700` those runs really send: 4, 6 and 9 m/s all
converge. The winch-law fix is safe for v04-v11; only 3 m/s is blocked. (An 11 m/s
attempt failed at IPOPT, but with a depower and turn-radius seed I guessed rather than
took from the run — not evidence of a regression.)

## Where this leaves it

The winch-law fix stands: it is correct, it is what makes the server's law match the one
flown, and it does not regress any wind speed that was working. What it did was remove an
accidental floor that had been hiding a real limitation — AWETrim's node-0 quasi-steady
solve cannot find a state at 3 m/s when the winch's zero-speed force is below ~830 N.

Next step is therefore in AWETrim's `phase_parametrized.py` residual solve, not in the
winch law and not in `f_low`. `_solve_node` already retries a tiered seed list
(`solver_retry_speed_radial`, `solver_retry_tension_kite`); worth dumping what it
converges to at `f_min = 850` and why the same system has no root at 800.

## Checked: `pattern_elevation_amplitude_max` does not shift the boundary

Measured, at 3 m/s with `f_min = 350`: `0.0` (off), `10.0`, `15.0`, `20.0`, `30.0` — all
five fail at node 0, identically. Same at `f_min = 800`, just under the boundary. Turning
`elevation_min` off as well, and then `min_turn_radius` too, changes nothing: with NO box
and NO turn-radius constraint at all it still fails at node 0.

With every constraint off the node-0 boundary is where it was — 800 and 820 fail at node
0, 900 converges — so the box does not move it in either direction. (`f_min = 850` does
flip from OK to an IPOPT failure once the box is removed, but that is the unconstrained
NLP being harder to converge, a different failure at a different stage.)

This is what the code says should happen, and it is worth stating because it closes off a
whole class of candidate fix: `pattern_elevation_amplitude_max`, `pattern_elevation_max`
and `min_turn_radius` are all NLP rows, while the failure is in the warm-start
quasi-steady march (`phase_parametrized.py`'s `_solve_node`) that runs along the SEED path
before IPOPT sees a single constraint. Nothing that shapes the optimized path can reach
it. Neither can the seed's own shape — `el_center` 20 and 15, `input_depower` 1.0 and a
wider `a = 40` were all measured earlier and all fail too.

## Tried: warm continuation from a high `f_min` down

Converge at an `f_min` known to work, then re-`/step` on the SAME session with a lower
`f_min`, feeding the previous reply back as the guess. The session carries the previous
solve's `speed_radial` / `s_dot` / `input_steering` profiles into the next node-0 solve,
which is exactly the seed that was missing.

**It clears the node-0 barrier outright.** Cold at `f_min = 800`: `residual@node0`. Warm
at 800: converges. The ~830 N cold threshold simply disappears — nothing else tried moved
it at all.

Ladder 1200 -> 1000 -> 900 -> 850 -> 800 -> 790 -> 780 -> 770, `elevation_amplitude_max`
off:

| f_min [N] | 1200 | 1000 | 900 | 850 | 800 | 790 | 780 | 770 | 760 | 750 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| result | OK | OK | OK | OK | OK | OK | OK | **OK** | fail | fail |
| elevation span [deg] | 14.8 | 16.6 | 18.3 | 19.9 | 23.8 | 24.1 | 25.5 | 27.4 | - | - |

`elevation_amplitude_max` is a real but small secondary effect, worth ~1-2 rungs and
nothing more: `10` stalls at 850 (span 19.9 against its own 20 cap), `15` reaches 780, and
`25`/off reach 770. Below 800 the cap is slack and the wall is an IPOPT convergence
failure, not the box and not node 0.

**Net: the reachable floor goes from 883 N (the old law's accidental one) to 770 N.** Real
progress, and it proves the node-0 threshold is a seeding problem rather than a physical
one. But it does not fix this bug: the run flies `276..1111 N` with a **516 N mean**, so
770 N is still above the average force, and the prediction would still be made against a
winch that stands still for most of the run.

Two things worth noting about the paths the ladder reaches. They climb and grow as `f_min`
falls — elevation `19.4..34.3` at 1200 N becomes `19.7..47.1` at 770 N — so they are
drifting away from the geometry the run actually flies (`elevation_amplitude_max: 10`,
a 20 deg span) and would likely be rejected by the curvature gate anyway. And the wall at
~765 N is a different failure from the one this plan started on: node 0 is solved, IPOPT
just will not converge.

## Where it stands

The winch-law fix is right and is not the blocker. The blocker is that AWETrim cannot
converge a 3 m/s solve with a winch whose zero-speed force is under ~765 N, warm-started
or not, and the honest value there is 350 N. Until that is addressed the 3 m/s power
prediction cannot be trusted in either direction — the old 78 W was an artefact of an
883 N floor, and no reachable setting replaces it with a number computed at the forces
the run actually flies.

Next step is AWETrim's NLP convergence below ~765 N at low wind, not the winch law, not
`f_low`, and not the pattern box.

## Restored: `use_awe_trim` blends again

Making the reel-in law unconditional had removed the blend line
(`T = (1 - use_awe_trim)*T + use_awe_trim*T_own`) rather than re-pointing it. It is back,
with the endpoints swapped so `0` is the law the client actually asks for:

| use_awe_trim | 0.0 | 0.25 | 0.5 | 0.75 | 1.0 |
| --- | --- | --- | --- | --- | --- |
| force at zero speed [N] | **349.5** | 483.0 | 616.4 | 749.8 | **883.2** |

`0` = the reel-in / `soft_lfc` law, `1` = the softminus-floored law it replaced. That now
matches WinchControllers.jl's own `use_awe_trim`, where `0` is the winch's own curve and
`1` is AWETrim's floored one — the same pair, in the same order.

One difference to be aware of: for `v < 0` the server's `u = 1` endpoint is the plain
SYMMETRIC quadratic (2520 N at -2 m/s), while `calc_vro_soft`'s `u = 1` endpoint keeps the
reel-in line (0 N there). They agree for `v >= 0`. The plain law was chosen deliberately —
it is exactly what the server ran before, and therefore exactly the law known to converge
at 3 m/s, which is what makes it useful as a continuation endpoint.

**Why this matters beyond tidiness.** It restores the homotopy knob on the right axis. The
`f_min` ladder walks the force level, but the endpoint wanted is the reel-in law at the
REAL `f_min = 350`. A blend continuation holds `f_min` fixed at 350 and walks
`u = 1 -> 0.75 -> 0.5 -> 0.25 -> 0`, from the law that converges to the law wanted, without
ever touching the force level. Worth trying next; the server needs a restart to pick the
change up.

## Tried: continuation on the LAW axis instead of the force axis

With `f_min` held at the real 350 N, walk `use_awe_trim` from 1 (the softminus-floored law,
883 N at zero speed, which converges) down to 0 (the reel-in law, 349.5 N, which is what
the run flies). Adaptive: halve the step on a failure and retry from the last good point.

| use_awe_trim | 1.000 | 0.875 | 0.844 | 0.812 | 0.750 |
| --- | --- | --- | --- | --- | --- |
| force at zero speed [N] | 883.2 | **816.5** | 799.8 | 783.1 | 749.8 |
| result | OK | **OK** | fail | fail | fail |

It stalls at `u = 0.875`, i.e. an effective zero-speed force of **816 N** — no better than
the `f_min` ladder, slightly worse.

**That is the useful result.** Two independent continuations, on two different axes, stop
at the same place: 770 N walking the force level, 816 N walking the law shape. The barrier
is the zero-speed force ITSELF, around 770-820 N at 3 m/s, and it does not care how you
arrive there. Every failure is now IPOPT, never `residual@node0` — the warm start
genuinely fixed the node-0 problem on both axes, and what remains is NLP convergence.

So there is no seeding trick left to try at this level. Reaching the real operating point
(349.5 N at zero speed, against a 516 N mean flown force) needs the 3 m/s NLP itself to
converge at low winch force, which is work inside `phase_parametrized.py`.

## Applied: `use_awe_trim: 0.875` plus a seeding solve

`data/wc_settings.yaml` is now at `use_awe_trim: 0.875`, the lowest value the 3 m/s solve
reaches. That puts the server's force at zero reel speed at **816 N**, down from 883 N.

The seeding trick is in as `opt_warm_start_awe_trim` (`data/traj_opt.yaml`,
`TrajOptSettings`): the `use_awe_trim` of a throwaway solve sent BEFORE the startup
request, purely to leave its converged `speed_radial`/`s_dot`/`input_steering` profiles in
the session. Only the first request after `/init` is cold — re-optimizations are already
warm from the solve before them — so it costs one extra solve at startup and nothing
during the run. It is a deterministic pre-solve, not a retry: it runs every time it is
set, so the optimum flown stays reproducible, which keeps it clear of the rule under
`reopt_retry_el_offset` that the startup solve never retries. `winch_from_wc` gained a
`use_awe_trim` keyword so the seeding request can differ from the real one.

**It is set to `0.0` (off), because 0.875 converges COLD.** Measured after the change:
cold at `u = 0.875` with no warm-up converges (`el 19.4..39.3`), so paying for a seeding
solve every run would buy nothing. The mechanism is there for when it is needed — set it
to `1.0` to enable.

Not yet checked: `u = 0.875` at the OTHER wind speeds, which send `f_low = 700` rather
than 350. Only 3 m/s was measured at this blend value.

## Measured: `use_awe_trim: 0.875` is fragile in tether length

The startup solve failed on the first real run at 0.875 (IPOPT, not `residual@node0` — the
node-0 barrier really is gone). The cause is NOT the blend value as such but its basin:
0.875 converges only in a narrow pocket of tether LENGTH, and a reel-out run sweeps
150-380 m. Cold, at 3 m/s:

| L [m] | 149.95 | 150.0 | 150.0137 | 150.05 | 150.2 | 151.0 |
| --- | --- | --- | --- | --- | --- | --- |
| result | OK | OK | OK | OK | **fail** | **fail** |

The earlier "0.875 converges cold" result was measured at exactly 150.0 m, inside that
pocket. That was the wrong test: the run sends `l_set = s.sys_state.l_tether[1]`, the
SETTLED length, and re-optimizes at every length after it. This is the same isolated-pocket
behaviour `data/traj_opt.yaml`'s header already warns about for the guess.

`use_awe_trim = 1.0` has no such problem — it converges cold at 150.2, 151, 160 and 200 m.
The seeding solve helps but does not fix it:

| L [m] | 150.2 | 151.0 | 160.0 | 200.0 |
| --- | --- | --- | --- | --- |
| cold at 1.0 | OK | OK | OK | OK |
| warm 1.0 -> 0.875 | **fail** | OK | OK | OK |

So `opt_warm_start_awe_trim: 1.0` is now on, and it recovers three of the four lengths that
fail cold. It improves the odds; it does not remove the failure.

**The trade to decide.** 0.875 buys a floor of 816 N against 883 N — 7.5% — and costs
runs that die at startup on the wrong length. Against a target of 350 N and a mean flown
force of 516 N, that is a small gain for a real reliability cost, and 1.0 is the setting
for a run that must not fail. The gain only becomes worth having if AWETrim's NLP is made
to converge well below 800 N, which remains the actual fix.

## Tried: doubling IPOPT's iteration cap (1000 -> 2000)

No effect anywhere. The default is `max_iter: 1000` (`phase.py`); `/step` already accepts a
per-request override, so this needed no code change.

| case | max_iter 1000 | max_iter 2000 |
| --- | --- | --- |
| L = 150.2, u = 0.875 | fail (41 s) | fail (67 s) |
| L = 151.0, u = 0.875 | fail (43 s) | fail (43 s) |
| L = 150.0, u = 0.50 | fail (53 s) | fail (91 s) |
| L = 150.0, u = 0.00 | fail node0 (3 s) | fail node0 (2 s) |

The timings say why. At L = 151.0 the solve takes the SAME 43 s at both caps, so it is not
stopping at the cap at all — it terminates on its own (restoration failure / local
infeasibility), and a bigger budget is irrelevant. Where the time does grow (41 -> 67 s,
53 -> 91 s) the extra iterations are spent and the solve still fails. The `u = 0` rows fail
at node 0 in 2-3 s, before IPOPT runs at all.

So the wall is not an iteration budget. Raising the cap would only make the failures more
expensive — and in `reopt_blocking` mode a failing solve freezes the simulation for its
whole duration.

## Found while testing: `u = 0.75` converges COLD, and warm-starting HURT it

At L = 150.0 the cold solve at `use_awe_trim = 0.75` converges in 18 s
(`el 24.0..47.5`) — a zero-speed force of **750 N**, better than 0.875's 816 N. The blend
continuation had reported 0.75 as failing, but that attempt was WARM from 0.875/1.0.

That is worth recording on its own: the warm start is not uniformly good. It cleared the
node-0 barrier everywhere, but at `u = 0.75` seeding from a higher blend lands IPOPT in a
basin it cannot leave, while a cold start finds one it can.

Not yet trusted: 0.75 has only been measured at ONE length, which is exactly the mistake
made with 0.875. A length sweep is needed before it is worth anything.

## The run at `use_awe_trim: 0.875`, and the correction it forced

The startup solve CONVERGED (14.7 s) — `opt_length_round: 1.0` fixed the 422, and the
149.9999542236328 m request became 150.0 m. Everything after that was worse than v03:

| | v03 | this run |
| --- | --- | --- |
| local `use_awe_trim` | **0.0** | 0.875 |
| success criteria | all 10 passed | FAILED: max d < 8 deg, phase 5 reached |
| tether 150 m -> | **380.0 m** (target) | **188.7 m** |
| predicted / measured | 78 W / 399 W -> 5.1 | **-59 W** / 383 W -> **-6.53** |
| mean reel-out force | 516 N | 707 N |
| re-optimizations installed | 5 of 7 | **0 of 7** |
| wall time frozen on solves | 60.8 s | **383.1 s** |

**The error in the earlier reading of this file: v03 was flown at `use_awe_trim = 0.0`,
not 1.0.** The recommendation to "go back to 1.0" was therefore wrong in the worst
direction — `use_awe_trim` also sets the force at which the LOCAL winch starts reeling out
(350 N at 0.0, 883 N at 1.0), and at 3 m/s the kite cannot pull 883 N, so 1.0 would never
reel out at all. 0.875 already showed this: reel-out stalled at 188.7 m against a 380 m
target.

**One knob, two opposite requirements.** Local wants it LOW so the winch reels out at a
force the kite can produce; the server wants it HIGH so the solve converges. They cannot
share a number — the same conflict `AWETRIM_SOFTMINUS_BETA` already resolves for the other
beta.

So it is split, following that precedent:

- `data/wc_settings.yaml` `use_awe_trim: 0.0` — the LOCAL law, back to what reaches 380 m.
- `data/traj_opt.yaml` `opt_awe_trim: 0.875` — what is SENT to AWETrim; `< 0` follows
  `wc_settings.yaml` as before. `winch_from_wc`'s `use_awe_trim` keyword carries it, and
  both the nominal and first-lap winches use it.

This does NOT make the two laws agree, and the power ratio will stay wrong: the prediction
is still computed against a winch floored at 816 N while the run flies well below that.
What it buys is that each side is usable — the run completes, and the optimizer is asked a
question it can answer. Closing the ratio still needs AWETrim to converge at low winch
force.

## Tried and REVERTED: flooring the `speed_radial` box at `v_reel_in`

The hypothesis was the rank-loss the codebase itself names for the per-node
`T_i == tension_curve(v_r[i])` equality: under the reel-in law `dT/dv_r` is EXACTLY zero
for every `v_r < v_reel_in`, which at `f_min = 350`, `v_max = 3.5` is 8 of the 13.5 m/s
box — a null direction over 60% of the variable's range. `session.py` was changed to floor
the box at `v_reel_in` instead of the hardcoded -10 wherever the reel-in law applies.

**It did not work, and it regressed a case that used to converge.** Cold at L = 150.0:

| use_awe_trim | 0.875 | 0.75 | 0.5 | 0.25 | 0.0 |
| --- | --- | --- | --- | --- | --- |
| before | OK (12 s) | **OK (18 s)** | fail IPOPT | - | fail node0 |
| after | OK (26 s) | **fail IPOPT** | fail IPOPT | fail node0 | fail node0 |

The wall stayed at 0.875, 0.875 itself got slower, and 0.75 went from converging to
failing. Reverted, with a comment in `session.py` recording the attempt so it is not
repeated. AWETrim's suite is green either way (766 passed).

What this rules out: the flat region below `v_reel_in` is NOT what stops the solve. The
plateau is real and measurable, but removing it from the box does not help — so either the
binding degeneracy is elsewhere in the law, or the tighter box costs more than the plateau
did (the optimizer may legitimately pass through `v_r < v_reel_in` on its way to a
solution, even though the law is unphysical there).

Also confirmed: this bound has NO effect on the node-0 failures at `u <= 0.25`, which fail
in 2 s. Those come from the forward quasi-steady march, which uses
`DEFAULT_BOUNDS["speed_radial"]`, a different constant that the request cannot reach.

Still open, in order of promise: `winch_mode: free_speed`, which replaces the equality with
a force band and is the code's own answer to this degeneracy (not reachable from a request
today); and the `DEFAULT_BOUNDS` box used by the node-0 march.

## `winch_mode: free_speed` — what it fixed, and what it revealed

`free_speed` is now reachable from a request: `WinchParams.winch_mode`
(`"force_law"` default | `"free_speed"`), validated in `session.py` and threaded into
`sim_parameters`, where the NLP already read it. Unset changes nothing for existing
clients. AWETrim suite green, 767 passed.

It drops the per-node `T_i == tension_curve(v_r[i])` equality: the reel speed becomes a
direct, acceleration-limited control and the winch only bounds tension to
`[f_min, f_max]`.

**It moved the IPOPT wall, and the warm start covers the rest.** Cold at L = 150 m:

| use_awe_trim | 1.0 | 0.875 | 0.5 | 0.25 | 0.0 |
| --- | --- | --- | --- | --- | --- |
| force_law | OK | OK | fail IPOPT | fail node0 | fail node0 |
| free_speed | OK | OK | **OK** | fail node0 | fail node0 |

The two barriers are independent and each needs its own remedy: `free_speed` clears the
IPOPT wall (0.875 -> 0.5 alone), the warm start clears node 0. Together a ladder
1.0 -> 0.75 -> 0.5 -> 0.25 -> 0.0 converges at every rung in 4-5 s.

**But `use_awe_trim` does nothing under `free_speed`, so that ladder is not what it
looks like.** The NLP never evaluates the tension curve in this mode, and the predicted
power confirms it — 192.349, 192.349, 192.355, 192.364, 192.368 W across the five rungs,
identical to five significant figures. The blend still reaches the node-0 forward march,
which is the only reason the low rungs fail COLD. So "reached u = 0" means the solve got
through, NOT that the NLP solved against the honest law: in `free_speed` there is no law.

**The finding that matters: the winch law is not the whole story.** With the law removed
entirely and the optimizer free to pick any reel speed inside the honest 350-8000 N band:

| | avg_power_W | energy_J | window_s |
| --- | --- | --- | --- |
| force_law, u = 1 (the 883 N floor) | **-54.3** | -1985 | 36.5 |
| free_speed (no law, honest force band) | **+192.3** | 7282 | 37.9 |
| the run MEASURED | **465** | | |

Removing the floor turns the prediction from negative to +192 W — that part IS the winch
law, and it is what this plan set out to find. But 192 W is an UPPER BOUND over any winch
in that band, and the run still measures 465 W, a factor 2.4 above it. No winch law can
close that.

Caveat on the factor: 192 W is at L = 150 m, while the measured 465 W is a mean over a run
that swept 150-324 m, and power grows with length. The two are not exactly comparable, so
2.4 is an upper estimate of the residual gap — but the sign is unambiguous, since the
optimizer's own best case at the starting length is far below what the run harvests there.

So the remaining discrepancy is NOT the speed/force curve this plan opened with. It is in
the aerodynamics, the path, the depower, or how the two sides define mean reel-out power.
That is where to look next.

Still open: the hand-off. Re-solving in `force_law` at u = 0, warm-started from the
converged `free_speed` optimum, FAILS (IPOPT, 32 s) — so an honest `force_law` prediction
at the flown law is still out of reach.

## Applied: the run optimizes in `free_speed`, and scores against its power

`free_speed` is ROBUST across the reel-out sweep — the thing `use_awe_trim: 0.875` never
was. Cold, one `/init` + `/step` per length, 3 m/s:

| L [m] | 150 | 151 | 175 | 200 | 250 | 300 | 324 | 380 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| predicted [W] | 192 | 200 | 264 | 296 | 317 | 315 | 308 | 274 |

Eight of eight, 8-12 s each, no length pockets. Against `force_law`, which at 3 m/s needs
the winch floored near 883 N to converge at all and then predicts a standstill.

Wired end to end:

- `WinchParams.winch_mode` in `examples/awetrim_client.jl`, `nothing` = server default;
  `winch_from_wc` takes it as a keyword and both the nominal and first-lap winches carry it.
- `TrajOptSettings.opt_winch_mode` / `data/traj_opt.yaml` `opt_winch_mode: "free_speed"`.
  `""` leaves the server on `force_law`, so nothing changes for another project.
- `opt_awe_trim` moved 0.875 -> **1.0**. Under `free_speed` the NLP never evaluates the
  tension curve, so this no longer shapes the path or the power at all; it still reaches
  the node-0 forward march, which DOES use the force law, and 1.0 is what that march
  converges at cold. The 0.875 fragility becomes irrelevant rather than worked around.
- `wc_settings.yaml` `use_awe_trim: 0.0` is untouched — the LOCAL law still reels out at
  350 N, which is what got the last run to 324 m.

**What the ratio now means.** `measured / predicted` becomes measured against an upper
bound over any winch in the honest 350-8000 N band, so a value above 1 is a real
disagreement about the physics and not an artefact of a floor the kite cannot reach. On
the last run's numbers that is roughly 465 W against a length-weighted free-speed
prediction of ~270 W over 150-324 m, i.e. **~1.7**, against the 8.1 `force_law` reported.
It is NOT a prediction of this `k_v` controller and must not be read as one.

Untested as a whole run: this has only been measured request by request.

## MEASURED: `free_speed` is not safe as the PATH source

The run with `opt_winch_mode: "free_speed"` crashed at t = 51.3 s — `next_step!` failed
after the kite stalled (lift 101 N against drag 228 N). The startup solve itself was fine;
the path it returned is not flyable with this winch.

| run | path source | az half-width | margin_start | min force | el_min | tether end |
| --- | --- | --- | --- | --- | --- | --- |
| 11:44 | force_law | 20.5 deg | 0.95 | - | - | **380 m**, passed |
| 14:50 | force_law, u = 0.875 | 33.6 deg | 0.95 | 254 N | 16.1 deg | 188.7 m |
| 15:30 | **free_speed** | 24.6 deg | **0.83** | **109 N** | **11.6 deg** | **83.2 m**, CRASH |

**The mechanism is a runaway, and it is the winch that closes the loop.** The free-speed
path dips to 11.6 deg elevation where the kite cannot make force. At 109 N the LOCAL law —
zero crossing 350 N — reels IN at up to -2 m/s. A shorter tether puts the kite lower and
slower, so the force falls further. 150 m -> 83 m, then stall. `speed_pct: 100.0` and
`lower_force_pct: 0.0` confirm it was the soft law's own reel-in branch, not the
LowerForceController. Its curvature margin was also the tightest of any run, 0.83 against
the 0.82 gate.

This is exactly the caveat the mode carries: a path optimal for ANY winch in the force
band is not merely suboptimal for the `k_v` winch, it is unflyable, because it does not
sustain the force that winch needs to keep reeling out. `free_speed` gives the optimizer no
reason to care.

Reverted: `opt_winch_mode: ""` and `opt_awe_trim: 0.875`, back to the configuration that
reached 324 m.

**But the reference half of the idea still stands, and the crashed run measured it.** Its
summary reports `predicted_W: 193` against `measured_W: 432`, ratio **2.24** — the
free-speed prediction is the 192-193 W measured independently at L = 150 m. So the two
roles must be split:

- PATH from `force_law`, which produces a flyable pattern.
- POWER REFERENCE from `free_speed`, an upper bound over any winch in the honest band.

The reference does not have to be computed during the run at all. Solving `free_speed` at
the lengths actually flown, AFTER the run, and length-weighting them costs nothing and
cannot destabilise anything — the eight-length sweep above (192 W at 150 m rising to 317 W
at 250 m, 274 W at 380 m) is exactly that calculation. That is the shape the reference
should take.

## Wired: a post-run `free_speed` power reference

The two roles are now split — `force_law` supplies the PATH (flyable), `free_speed` supplies
the REFERENCE (an upper bound), and the reference is computed only after the run.

`TrajOptSettings.free_speed_reference_points` / `data/traj_opt.yaml`
`free_speed_reference_points: 6` (`0` = off). At the end of a run,
`examples/reelout_results.jl` solves `free_speed` at that many lengths spanning the reeling
window, then scores every reeling sample at its own length by linear interpolation between
the probes, so the number is TIME-weighted rather than a flat mean over lengths. It writes:

- `free_speed_reference_W` — the weighted upper bound.
- `free_speed_points` — the per-length solves, so the curve is visible (192 W at 150 m
  rising to ~317 W at 250 m and back to 274 W at 380 m, measured).
- `free_speed_ratio` — `measured / free_speed_reference_W`.

`use_awe_trim = 1.0` is sent on these requests: the `free_speed` NLP never reads the tension
curve, but the node-0 forward march does, and 1.0 is the value that march converges at cold.

It cannot disturb the run — it happens after the last simulation step, uses its own session
name (`<name>-fsref`), and a solve that fails is skipped rather than fatal, with the keys
omitted entirely if none succeed. Cost is about 10 s per point.

**Read `free_speed_ratio`, not `ratio`, at low wind.** `ratio` is against `force_law`,
which at 3 m/s prices the path against a winch floored above anything the kite pulls and has
come back NEGATIVE. `free_speed_ratio` is against a bound that no winch can beat, so above 1
is a real disagreement about the physics. On the last two runs that is roughly 2.2-2.4,
against 8.1 and -6.53 from `ratio`.

Verified: settings load, all three edited files parse. Not yet exercised by a full run.

## ROOT CAUSE of the residual gap: the two sides fly different kites

The masses do not match, and the difference is large:

| | ours | AWETrim |
| --- | --- | --- |
| wing | **6.2 kg** | **10.9926 kg** |
| bridle (lines + pulleys) | not counted separately | 2.5875 kg |
| KCU | 8.4 kg | 8.4 kg |
| total | ~14.6 kg | ~22.0 kg |

Ours is `wing_mass: 6.2` in `data/kite_settings_psm_kernel.yaml` — the file
`system_reelout_150m.yaml` names, carried by THIS repo rather than V3Kite — with
`settings_reelout_150m.yaml`'s `mass: 6.2` / `kcu_mass: 8.4` agreeing. AWETrim's is
`data/LEI-V3-KITE/system.yaml`, the default `LEI_V3_SYSTEM_CONFIG` its server loads. The
KCU matches exactly; the wing does not.

**Measured: mass is the whole residual factor.** One line changed in a copy of AWETrim's
system config (wing 10.9926 -> 6.2, bridle and KCU untouched), `free_speed` reference at
3 m/s, same request otherwise:

| L [m] | 150 | 200 | 250 | 300 | 380 | mean |
| --- | --- | --- | --- | --- | --- | --- |
| wing 10.99 kg | 192 | 297 | 318 | 315 | 274 | **279** |
| wing 6.20 kg | 580 | 646 | 634 | 637 | 574 | **614** |
| ratio | 3.02 | 2.18 | 2.00 | 2.02 | 2.09 | **2.20** |

4.8 kg off the wing roughly DOUBLES the predicted power, and 3x at the starting length.
That is the whole of the residual `power_ratio_free_speed` of 1.54.

**With the masses matched the paradox disappears.** The run measured 389 W against a
reference of 252 W at AWETrim's mass — harvesting 1.54x an upper bound, which is
impossible. At OUR mass the reference is ~610 W weighted, and 389 W is **0.64** of it:
below the bound, exactly as an upper bound should behave.

So the 3 m/s discrepancy was two things, and neither is the "different speed/force curve at
low forces" this plan opened with:

1. The winch law's softminus floor (883 N, above anything the kite pulls) made `force_law`
   predict a standstill and even negative power. Fixed by the reel-in law, and sidestepped
   for scoring by the `free_speed` reference.
2. **The wing mass, worth a factor ~2.2.** Everything left after (1).

**Which mass is right is now the question, and OUR side looks like the outlier.** V3Kite's
own `data/bridle_geometry_full_fem.yaml` records `mass_without_bridles: 11 #kg`, i.e.
AWETrim's 10.9926. If that is the real V3, the simulation has been flying a kite ~4.8 kg
too light and its low-wind power is optimistic — the 394 W at 3 m/s in `output/scenarios/`
included.

Not settled from YAML alone: whether our `wing_mass: 6.2` is meant to exclude a bridle mass
that the structural geometry carries as particles elsewhere. That decides whether the true
difference is 4.8 kg or the full 7.4 kg. Confirm against the model, not the settings files.

## Applied: wing mass 6.2 -> 10.9926 kg

`data/kite_settings_psm_kernel.yaml` `wing_mass` and `data/settings_reelout_150m.yaml`
`mass`, both to AWETrim's **10.9926 kg**. `wing_mass` EXCLUDES the bridle on both sides
(ours is carried by the structural model, AWETrim's is a separate 2.5875 kg) and the KCU
is 8.4 kg on both, so the wing was the only real difference and the mass test isolated
exactly it.

Scope is narrow: `kite_settings_psm_kernel.yaml` is referenced only by
`system_reelout_150m.yaml`. `data/settings_fig8_{150,200,300}m.yaml` and
`settings_reelout_180m.yaml` still carry `mass: 6.2` and use different kite settings —
left alone, since changing them moves results that are already measured.

**Expect every 3 m/s number to move, and not only the ratio.** The prediction was made
against this mass all along; it is the SIMULATION that is about to get 4.8 kg heavier. At
3 m/s the kite is near its minimum flying condition, so the measured power should fall
towards the ~250-280 W the optimizer has been predicting, rather than the prediction
rising to meet 389-465 W. `output/scenarios/`'s 394 W at 3 m/s was flown at 6.2 kg and is
not comparable to what comes next.

Also expect the run itself to be harder: a heavier kite at 3 m/s pulls less, and the local
law reels in below 350 N. The 150 -> 83 m reel-in that crashed the `free_speed` path run is
the failure mode to watch for.

Unverified: no run has been flown at the new mass yet.

## The 10.9926 kg wing does not fly, and the mass cannot be fixed alone

Run at the corrected mass, 3 m/s:

| | 6.2 kg (before) | 10.9926 kg |
| --- | --- | --- |
| laps | 8.3 | **0.0** |
| tether 150 m -> | 324 m | **49 m** (reeled IN) |
| lowest elevation | 16.1 deg | **-82.4 deg** (below the horizon) |
| measured power | 465 W | **-759 W** |

It never completed one figure of eight. The flown path is not a degraded pattern but a
sweep over +-150 deg of azimuth and down through the horizon — the kite fell.

**This does not settle which mass is right; it shows the mass cannot be changed on its
own.** Everything downstream was identified against 6.2 kg:

- `data/turn_rate_coeffs.yaml`'s `c1`/`c2`/`delay`, which set the whole steering response
  and the curvature feasibility gate. Turn rate is mass-dependent, and the entire
  identification is against the light kite.
- `fc_settings_reelout.yaml`'s `depower_setpoint` (0.274). A 77% heavier wing needs more
  power, not the same tape.
- The steering gains, attractor lead and lift budget, all tuned around the light kite.

So the failure is consistent with mistuning AND with the kite genuinely being unable to
fly at 3 m/s at 22 kg — this run cannot tell them apart. Note AWETrim itself, at 10.9926 kg,
still finds a solution worth 192-317 W, but it solves an OPTIMAL trajectory rather than
what a PID controller achieves from a standing start.

**Reverted to 6.2 kg** so the reel-out project runs, with the evidence recorded in
`data/kite_settings_psm_kernel.yaml` next to the value.

To do this properly, in order:
1. Settle which wing mass is physically right. V3Kite's own `bridle_geometry_full_fem.yaml`
   (`mass_without_bridles: 11`) and AWETrim agree on ~11 kg, against this repo's 6.2 kg —
   the burden of proof is on 6.2.
2. Check whether `wing_mass` is the wing's TOTAL mass or an addition on top of the
   structural particles' own. `distribute_wing_mass!` writes `point.extra_mass`, and
   V3Kite's stock `kite_settings_psm.yaml` ships `wing_mass: 0.0`, which would be a
   massless wing if it were the total. Answer this from the model, not the YAML.
3. Only then re-identify `turn_rate_coeffs` and re-tune `depower_setpoint` at the new mass,
   and expect the low-wind scenarios to move — `output/scenarios/`'s 394 W at 3 m/s was
   flown at 6.2 kg.

## Confirmed: `wing_mass: 0.0` means "use the mass the geometry carries"

V3Kite's own docstring (`src/simulation.jl`):

> Mass [kg] redistributed over the wing nodes by chord, `0` keeping what the geometry
> carries. A beam wing keeps its mass in the bodies, so setting this there counts gravity
> twice.

And the only call site of `distribute_wing_mass!` in the whole package is
`examples/flight_replay.jl`, guarded by `if kite_set.wing_mass > 0`. So a non-zero
`wing_mass` OVERRIDES the geometry's mass rather than adding to it — which answers the
doubt raised when the 10.9926 kg run failed: setting it did not stack 11 kg on top of an
existing wing mass.

**But it also means `wing_mass` is not what broke that run.** Diffing the archived settings
of the crash (16:01) against the last good run (15:46), the ONLY difference is

```
-  mass: 6.2
+  mass: 10.9926          # settings_reelout_150m.yaml
```

`kite_settings_psm_kernel.yaml` is not archived, but `wing_mass` has no consumer in the
library path at all — only `flight_replay.jl`, which this run never touches. So the kite
mass that actually reaches the model is `mass` in the settings file, and `wing_mass: 6.2`
beside it has been inert.

**Still unresolved, and it should be settled before anyone edits a mass again**: no consumer
of `set.mass` could be found in V3Kite's `src`, SymbolicAWEModels or KiteUtils by grep,
despite the diff proving it drives the model. Either it is read somewhere the search missed
or the installed package differs from the checkout that was read. Settle it from the MODEL,
not the files — load the run's `V3KITE` and read the total point mass back, once, rather
than reasoning from YAML as this section and the one above it both did.

That also decides the open question underneath all of it: whether our 6.2 kg is the WING
alone (comparable to AWETrim's 10.9926 kg) or the whole kite including the bridle
(comparable to 10.9926 + 2.5875 = 13.58 kg). The power gap is a factor ~2.2 either way, but
the correction is not.

## RETRACTED: the masses agree. The wing-mass finding was wrong

Read from the structural geometry, which is what the model actually carries
(`wing_mass` being inert, per the section above). Summing the `extra_mass` column of
V3Kite's `data/struc_geometry.yaml`:

| point | extra_mass [kg] |
| --- | --- |
| 1 (KCU node) | 23.25 |
| 34-38 (pulleys) | 0.1 each |
| **total** | **23.75** |

against AWETrim's **21.98** kg (wing 10.9926 + bridle 2.5875 + KCU 8.4).

**The two kites are within 8% of each other on total mass, and OURS IS THE HEAVIER.** The
"our wing is 6.2 kg against AWETrim's 10.99" finding was an artefact of reading
`wing_mass`, which has no consumer in the library path at all. The real mass is lumped at
the KCU node in our model rather than distributed over the wing — a difference in inertia
distribution, not in weight.

So **mass does not explain the residual 1.54 power ratio**, and the direction is now wrong
too: a slightly heavier kite should harvest slightly LESS, not 1.5x more.

What survives from that work: the sensitivity measurement itself. 4.8 kg off AWETrim's wing
doubles its predicted power at 3 m/s (279 -> 614 W mean), so the models ARE very sensitive
to mass near the minimum flying condition — which is worth knowing, and means any future
mass discrepancy is worth chasing. The premise that we HAD such a discrepancy was false.

Also unexplained, and worth resolving before anyone edits a mass again: setting
`settings_reelout_150m.yaml`'s `mass` from 6.2 to 10.9926 demonstrably destroyed the run
(0 laps, elevation -82 deg, tether reeled to 49 m) — the archived diff shows it was the
only change. Yet the geometry already carries 23.75 kg without it, and no consumer of
`set.mass` can be found in V3Kite, SymbolicAWEModels or KiteUtils. If it ADDS to the
structural mass, the run has been flying ~29.95 kg, not 23.75, and the true comparison with
AWETrim's 21.98 kg is a 36% overweight in OUR favour — which would deepen the residual gap
rather than close it. This needs answering from the model.

**The 1.54 residual is therefore still unexplained.** The winch law accounted for the
absurd `force_law` numbers (negative power, ratio 8.1); nothing yet accounts for the run
harvesting 1.5x an upper bound computed at essentially the same kite mass.

## Found: `set.mass` IS the wing mass, via a silent fallback

`SymbolicAWEModels/src/system_structure/system_structure_core.jl`,
`finalize_particle_wing_mass!`:

```julia
point_mass, body_mass = particle_wing_masses(wing, ...)   # wing's own points + bodies
total = point_mass + body_mass
if total > 0
    wing.mass = total
else                                        # <- our case
    set_mass = hasproperty(set, :mass) ? set.mass : 0.0
    set_mass > 0 && distribute_mass_over_points!(points, wing_point_idxs, wing, set_mass)
```

`particle_wing_masses` sums `extra_mass` over the wing's TWIST-SURFACE points — the LE/TE
pairs — and every one of them is `0.0` in `struc_geometry.yaml`. With no beam bodies either,
`total = 0` and the fallback fires: **`settings_reelout_150m.yaml`'s `mass: 6.2` is the wing
mass, distributed over the wing points.** The crashed run is the proof the path is live, since
this is the only route by which `set.mass` can reach the model.

So the mass accounting is:

| | ours | AWETrim |
| --- | --- | --- |
| wing | **6.2** (`set.mass`, via the fallback) | **10.9926** |
| KCU node / bridle | **23.25 + 0.5 pulleys** | 8.4 + 2.5875 |
| **total** | **29.95 kg** | **21.98 kg** |

Both of the earlier readings were wrong. The wing IS 4.8 kg lighter on our side — the
original comparison was numerically right but attributed to `wing_mass`, which is inert;
the live field is `set.mass`. And the retraction was wrong too: the geometry's 23.75 kg is
NOT the whole kite, it is the KCU node plus pulleys, and the wing's 6.2 kg sits on top.

**Mass still does not explain the 1.54 ratio**, and the direction is against it: our kite is
36% HEAVIER in total (29.95 vs 21.98 kg), which should harvest LESS, not 1.5x more. The two
sides distribute that mass very differently — ours 78% at the KCU node, AWETrim's half in
the wing — which changes inertia and pendulum behaviour, but not weight.

**Worth investigating on its own: the 6.2 kg may be spurious.** The fallback fires precisely
because the wing's points carry no mass, which happens because this geometry lumps mass at
the KCU node instead. If that node's 23.25 kg was meant to represent the whole kite, then
`set.mass` is silently adding a second, 6.2 kg wing on top and the model is ~6 kg overweight
by accident. The code warns when points AND bodies both carry mass ("gravity is counted
twice") but cannot detect this case. That is a question for whoever built
`struc_geometry.yaml`, and it is not the 3 m/s bug.

## The mass accounting, finally traced to the model

Both live masses come from `data/settings_reelout_150m.yaml`, and neither is the field it
looks like:

- `mass: 6.2` is the WING mass. `struc_geometry.yaml`'s wing points carry `extra_mass: 0.0`,
  so `finalize_particle_wing_mass!` (SymbolicAWEModels) falls through to
  `distribute_mass_over_points!(..., set.mass)`.
- `kcu_mass: 8.4` is the KCU mass. `stabilization.jl` does
  `sys.points[1].extra_mass = kcu_mass`, OVERWRITING the geometry's 23.25 kg, which is a
  heavy-KCU test value that never reaches a run configured this way.
- `kite_settings_psm_kernel.yaml`'s `wing_mass` is INERT — its only consumer is V3Kite's
  `examples/flight_replay.jl`.

| | ours | AWETrim | difference |
| --- | --- | --- | --- |
| wing | **6.2** | **10.9926** | ours **-4.79** |
| KCU | 8.4 | 8.4 | equal |
| bridle / pulleys | 0.5 | 2.5875 | ours -2.09 |
| **total** | **15.1 kg** | **21.98 kg** | ours **-6.88 (-31%)** |

**Our kite is 31% lighter, and the wing is where most of it sits — so the original finding
stands.** The intermediate retraction was wrong: it read the geometry's 23.25 kg as the live
KCU mass and concluded the totals agreed. They do not.

This makes the sensitivity test directly load-bearing rather than hypothetical: taking
4.79 kg off AWETrim's wing — exactly the difference above — DOUBLES its predicted power at
3 m/s, 279 -> 614 W mean over 150-380 m. The run measured 465 W. Against AWETrim's own mass
that is 1.54x an upper bound, which is impossible; against OUR mass it is about 0.76 of one,
which is what an upper bound should look like.

So the 3 m/s discrepancy resolves into two causes, and the winch curve this plan opened with
is neither of them:

1. **The winch law's softminus floor** (883 N, above anything the kite pulls) made
   `force_law` predict a standstill and even negative power — hence ratios of 8.1 and -6.53.
   Fixed by the reel-in law; sidestepped for scoring by the `free_speed` reference.
2. **The kite mass**, worth the remaining factor ~2. The two sides are simulating kites that
   differ by 6.9 kg, nearly all of it in the wing.

Which wing mass is right is now the open question, and 6.2 kg carries the burden of proof:
AWETrim's 10.9926 and V3Kite's own `bridle_geometry_full_fem.yaml` (`mass_without_bridles:
11`) agree, against this repo's 6.2. But correcting it is not a one-line edit — a run at
10.9926 kg does not fly at all (0 laps, elevation -82 deg, tether reeled to 49 m), because
`turn_rate_coeffs`, `depower_setpoint` and the steering gains were all identified at 6.2 kg.

## Why AWETrim does not converge at low winch force

Investigated by reading the code only — a run was in progress, so no requests were sent.

**The NLP ties tension to reel speed with a per-node EQUALITY.** In
`phase_parametrized.py` each node imposes

```python
T_model = winch_model.tension_curve(opti_vars["speed_radial"][i], ...)
opti.subject_to((T_i - T_model) / S["T"] == 0)
```

The codebase already names the failure mode this creates. The `winch_mode: free_speed`
alternative exists because the force law's *"soft-clamp plateau (dT/dv_r ~ 0 when riding
the force limit) destroys the Jacobian rank"*, and its branch replaces the equality with
two inequalities, commented *"no soft-clamp plateau, no rank loss."*

**The reel-in law has a far worse plateau than the law it replaced, and it sits inside the
variable's own box.** `session.py` bounds `speed_radial` to
`[DEFAULT_OPTI_LIMITS["speed_radial"][0], v_max]` = **[-10, 3.5]** — the upper bound comes
from the request, the lower is hardcoded at -10. Under the reel-in law
`T = fmax(soft_max(line, quad), 0)` is EXACTLY zero for every `v_r < v_reel_in`, so:

| use_awe_trim | 0.0 | 0.5 | 0.875 | 1.0 |
| --- | --- | --- | --- | --- |
| contiguous `dT/dv_r == 0` inside the box | **8.1 m/s** (-10..-2) | 3.1 | 2.6 | 2.7 (fragmented) |
| `dT/dv_r` at `v_r = 0` [N/(m/s)] | 174.9 | 87.5 | 21.9 | 0.0 |

At `use_awe_trim = 0` roughly **60% of the `speed_radial` box is a null direction**: `v_r`
can move anywhere in -10..-2 m/s without changing any constraint residual, so the KKT
system is singular over that whole region and IPOPT's restoration phase can wander into
it. The softminus law's flat spots are isolated points, not an 8 m/s interval.

Secondary, same direction: in `v_r` in (-2, +0.92) the reel-in law IS the straight line, so
`dT/dv_r` is a constant 175 N/(m/s) and the second derivative is exactly zero — the whole
low-speed band is linear, which is also where a 3 m/s run lives.

**Candidate fix, minimal and untested: bound `speed_radial` below at `v_reel_in`, not -10.**
The flat region would then lie entirely outside the box. It is physically the right bound
anyway — under this law the winch cannot reel in faster than `v_reel_in`, and `T = 0` below
it is not a winch behaviour but an artefact of the `fmax`. One line in `session.py`'s
`override["speed_radial"]`.

**Second candidate: `winch_mode: free_speed`**, which the code already provides for exactly
this degeneracy. Bigger change, and NOT currently reachable from a request — it is read
from `sim_parameters`, which the server builds itself.

Neither is tested: the server was busy with a run.

## Confirmed by direct measurement, and the mass corrected

AWETrim solved at OUR exact kite (wing 6.2 + KCU 8.4 + bridle 0.5 = 15.1 kg) against its
own (21.98 kg), `free_speed`, 3 m/s:

| L [m] | 150 | 200 | 250 | 300 | 380 | mean |
| --- | --- | --- | --- | --- | --- | --- |
| AWETrim 21.98 kg | 192 | 297 | 318 | 315 | 274 | **279** |
| matched 15.1 kg | 580 | 646 | 634 | 637 | 574 | **614** |

The run measured 465 W: **1.54x** an upper bound computed at AWETrim's mass — impossible —
and **0.76** of one computed at ours, which is what an upper bound should look like. The gap
is fully accounted for.

**`data/settings_reelout_150m.yaml` `mass` is now 10.9926 kg** (was 6.2), the field that
actually reaches the model. `kite_settings_psm_kernel.yaml`'s inert `wing_mass` is set to
`0.0` rather than left at a stale 6.2 beside it — that mismatch is what produced several
wrong conclusions in the sections above, and `0` also carries the documented meaning "keep
what the geometry carries".

Both files now state which field is live and which is not. Not yet flown: 3 m/s does not fly
at this mass, and the controller is still identified at 6.2 kg.

## VALIDATED at 4 m/s with the corrected mass

First run at `mass: 10.9926`, 4 m/s:

| | 3 m/s, 6.2 kg (before) | 4 m/s, 10.9926 kg |
| --- | --- | --- |
| `power_ratio` (force_law) | 8.1, and -6.53 | **1.10** |
| `power_ratio_free_speed` | 1.54 (impossible: above the bound) | **0.99** |
| re-optimizations installed | 0-5 of 7 | **7 of 7** |
| criteria failed | up to 6 | **1** (min elevation > 6.5 deg) |
| mean reel-out power | 465 W | 1561 W |

**Both diagnoses are confirmed by this one run.** `power_ratio` of 1.10 means the reel-in
law fixed `force_law` — the same quantity was 8.1 and then NEGATIVE while the softminus
floor stood the winch still. And `power_ratio_free_speed` of 0.99 means the run harvests 99%
of an upper bound computed at the SAME kite mass: the models now agree about the physics,
which they could not while one simulated a 15.1 kg kite and the other a 21.98 kg one.

7 of 7 re-optimizations installed, against 0 of 7 at `use_awe_trim: 0.875` and 5 of 7 before
that — the winch-law and length-rounding work holds up at this wind too.

Worth watching rather than celebrating: **0.99 is very tight.** A PID-flown path matching an
unconstrained optimum to 1% is either excellent or means the bound is loose at 4 m/s (the
force band binds less when the kite pulls 1195 N mean). A ratio that sat slightly ABOVE 1
would be a warning; 0.99 is on the right side of the line but leaves no margin, so it is
worth confirming at another wind speed before treating the reference as calibrated.

Still open:
- **3 m/s does not fly at 10.9926 kg** (0 laps, elevation -82 deg). The controller is still
  identified at 6.2 kg — `turn_rate_coeffs.yaml`'s `c1`/`c2`, `depower_setpoint`, the
  steering gains — so low wind needs re-identification at the new mass.
- `min elevation > 6.5 deg` still fails at 4 m/s.
- **Everything in `output/scenarios/` was flown at 6.2 kg** and is not comparable to runs
  from here on: 394 W at 3 m/s through 35 kW at 10 m/s. The series needs re-flying.
