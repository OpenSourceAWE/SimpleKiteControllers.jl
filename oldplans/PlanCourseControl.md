# Move the course controller to `src/`

The inner loop of the flight controller — §3 of [control_algorithm.md](docs/control_algorithm.md),
"steering set point from position and desired course" — lives in the example scripts.
Move it into `src/` behind a controller type, so it can be documented, unit tested and
re-used, the way the guidance (`FigureEightController`) already is.

This is the work the README TODO names: *"the entry state machine, the winch force mode
and the heading PID still live in `examples/simple_fig8.jl` rather than in a controller
type"*. The winch force mode is NOT part of this plan (it went to WinchControllers.jl,
see [PlanWinchcontrol.md](PlanWinchcontrol.md)); the state machine and the PID are.

## Why

**Three byte-identical copies.** The block is duplicated verbatim in
[simple_fig8.jl:328-377](examples/simple_fig8.jl#L328-L377),
[simple_reelout.jl:371-420](examples/simple_reelout.jl#L371-L420) and
[simple_fig8_live.jl:455-504](examples/simple_fig8_live.jl#L455-L504) — verified with
`diff`, not by eye: zero differing lines across all three. The PID construction
(`create_heading_pid`, 3 lines), the `entry_sign` latch, the phase ladder and the four
log assignments (`var_05`…`var_08`) are duplicated alongside it, and the phase ladder
differs between them only by reel-out's extra phase 5. Every tuning fix, every sign
correction and every new diagnostic has to be applied three times, and
[control_algorithm.md#4](docs/control_algorithm.md) already records what that costs: the
`winch_adapter.jl` copy had silently drifted and broke five call sites.

**Nothing here is tested.** `test/test_fig8_controller.jl` covers the guidance, the
metrics and the turn-rate table — 15 testsets, none of them downstream of `chi_set`. The
inner loop is only ever exercised by a minutes-long closed-loop run, so a regression in
it is found by a plot, not by `runtests.jl`.

**The traps are in comments, not in code.** Each of the three copies carries the same
warnings: `SysState.course` needs `+π` before it is blended (feeding it unshifted is
positive feedback that diverges the run), the PID output is fed to `rel_steering`
*unnegated*, the error must be wrapped because `DiscretePIDs` does not wrap, and the
limiter blend must use a wrapped difference or it sweeps the long way at ±180°. A
comment cannot fail a test.

**Layering.** The scripts build the PID with V3Kite's `create_heading_pid` and wrap with
V3Kite's `wrap_to_pi`. Both are trivial (`create_heading_pid` is a one-line
`DiscretePID(; K, Ti, Td, N, Ts=dt, umin, umax)` forwarder; `wrap_to_pi` is
byte-identical to `KiteUtils.wrap2pi`), but they make the controller code depend on a
kite model — the same inversion PlanWinchcontrol.md closed for the winch. Neither
survives the move.

## Step 1: design the interface

### The signal list, reconciled against the code

The four signals in the original draft (`azimuth`, `elevation`, `chi_set`, `psi_prime` →
`rel_steering`) do not match what the block actually reads. Corrected:

| signal | in the draft? | used for | source today |
|:--|:--|:--|:--|
| `chi_set` [rad] | yes | commanded course, before the limiter | `navigate_fig8` |
| `psi_prime` [rad] | yes, as an input | the regulated angle | **computed inside the block**, from `heading`, `course`, `vel_kite` |
| `azimuth` | yes | — | **not read at all** |
| `elevation` | yes | — | **not read** (only the phase ladder reads it, in degrees) |
| `dmin` [deg] | no | limiter gate/blend, and the 3→4 transition | `navigate_fig8` |
| `tangent` [rad] | no | latched sign at the ±180° cut | `path_tangent(fec)` |
| `heading` [rad] | no | ψ' fusion | `sys_state.heading` |
| `course` [rad] | no | ψ' fusion (after `+π`) | `sys_state.course` |
| `v_kite` [m/s] | no | ψ' blend weight `w_χ` | `norm(sys_state.vel_kite)` |
| `v_app` [m/s] | no | `K ∝ 1/v_app` gain schedule | `sys_state.v_app` |
| `t` [s] | no | phase ladder (`park_time`, `hold_time`) | `sys_state.time` |
| `elevation` [deg] | — | phase ladder (1→2 on `el_center + dive_el_margin`) | `sys_state.elevation` |

`azimuth` and `elevation` are inputs of `navigate`/`linearize` in
[parking_controller.jl](src/parking_controller.jl) — `elevation` enters the NDI `c2`
term — which is presumably where the draft's list came from. The fig8 inner loop has no
NDI block and reads neither (except the ladder's elevation). Keep them out; see
[Out of scope](#out-of-scope) for the NDI convergence question.

`psi_prime` as an *input* is a real design choice, not an oversight — see decision 3.

### Decision 1 — no kite-model dependency

`create_heading_pid` → `DiscretePID(; K, Ti, Td, N, Ts = dt, umin, umax)` directly
(`DiscretePIDs` is already a dependency of this package, and `set_K!` comes with it).
`wrap_to_pi` → `KiteUtils.wrap2pi`, already `using`-ed in
[SimpleKiteControllers.jl:6](src/SimpleKiteControllers.jl#L6). Both substitutions are
value-identical, so they cannot move a number in the regression runs; check that claim
against `V3Kite/src/coordinate_utils.jl:15` and `KiteUtils/src/transformations.jl:254`
once more before relying on it.

### Decision 2 — one type trio, matching the two that exist

```julia
CourseControllerSettings   # @with_kw mutable, @deftype Float64, like FigureEightSettings
CourseController           # mutable, holds the DiscretePID + the latched state
calc_steering(cc, ...)     # a METHOD of the name parking_controller.jl already exports
```

Naming: `CourseController` (the draft's title) rather than `HeadingController` — it
tracks a course; the *heading* only appears inside ψ'. Reusing `calc_steering` by
dispatch keeps one verb for "angles in, `rel_steering` out" across both controllers.

### Decision 3 — fuse ψ' inside the controller (decided: yes), not outside

Taking `psi_prime` ready-made, as the draft proposes, is the smaller interface, but it
leaves in the scripts exactly the four lines that carry the documented divergence bug:

```julia
w_course = fcs.fig8_pure_course && phase >= 3 ? 1.0 : clamp(…)
course   = wrap_to_pi(Float64(s.sys_state.course) + pi)   # ← omit this and the run diverges
fb       = heading + w_course * wrap_to_pi(course - heading)
```

`w_χ` is also phase-dependent (`fig8_pure_course && phase >= 3`), so with the fusion
outside, the phase has to be handed *out* of the controller and back *in*. Fuse inside:
inputs `heading`, `course`, `v_kite`; ψ' and `w_χ` become readable fields for the log.

The `+π` becomes a settings field `course_offset = π`, documented as "`SysState.course`
has its zero pointing away from zenith; 0 for a source already in the bearing
convention". That is one constant of a frame convention, not a kite-model dependency,
and it stops the correction being copy-pasted three times.

### Decision 4 — the entry state machine moves too (decided: yes)

The limiter and the gain schedule are phase-dependent, so a controller that maps only
`(chi_set, ψ') → rel_steering` cannot reproduce what is flown. Either the phase comes in
as an argument (machine stays in the scripts, ~15 duplicated lines remain, and the entry
parameters `park_time`/`hold_time`/`dive_el_margin`/`chi_dive`/`chi_hold`/`fig8_d_gate`
stay split across two homes), or the machine moves with it.

Move it, with the reel-out difference handled explicitly: phases 0-4 are the flight-side
ladder; **phase 5 is winch-triggered** (`l_set >= reelout_l_max`) and stays the
reel-out script's business, applied through a `set_phase!(cc, 5)` that keeps the ladder's
"never backwards" invariant. `simple_reelout.jl:365-369` already separates that rule from
the ladder for exactly this reason, and its comment ("so it can fire the SAME step as a
3→4 transition") is the behaviour to preserve.

Consequence: `rel_depower` is phase-driven too and becomes a second output
(`entry_depower` / `depower_setpoint` / `depower_final`), which removes the last
phase-dependent block from the scripts.

### Decision 5 — settings: own struct, plus an `FC_Settings` constructor

Follow the `FigureEightSettings` precedent — an own `@with_kw mutable struct` with plain
defaults, so tests can build one in a line the way `_make_test_controller` does, and add

```julia
CourseControllerSettings(fcs::FC_Settings; dt) = …
```

so the three scripts do not each copy ~20 fields. Fields to carry over, all already in
[fc_settings.jl](src/fc_settings.jl) and unchanged in value:

- PID: `heading_p`, `heading_i::Union{Bool,Float64}`, `heading_d`, `heading_d_n`,
  `max_steering`, `v_app_ref`, `v_app_min`, `entry_gain`
- ψ': `v_kite_heading`, `v_kite_course`, `fig8_pure_course::Bool`, `course_offset` (new)
- limiter: `entry_chi_max`, `entry_d_gate`, `entry_d_blend`, `entry_cut_margin`
- ladder: `park_time`, `hold_time`, `dive_el_margin`, `el_center`, `chi_dive`,
  `chi_hold`, `fig8_d_gate`
- depower: `depower_setpoint`, `entry_depower`, `depower_final`

`el_center` is shared with `FigureEightSettings` — it is the pattern's centre and the
ladder's 1→2 threshold. Keep `FC_Settings` the single YAML-backed source of truth and let
both settings structs read it; do not add a second YAML file.

Include order in [SimpleKiteControllers.jl](src/SimpleKiteControllers.jl): after
`fc_settings.jl` (the `FC_Settings` constructor is defined with the type it constructs),
before `optimization.jl`. Add the same kind of one-line "after X because Y" comment the
file already uses.

### The interface

```julia
"""
    calc_steering(cc::CourseController, chi_set, heading, course; kwargs...)

Inner loop of the figure-of-eight flight controller: commanded course in,
`rel_steering` out. Returns `(rel_steering, rel_depower, phase)`.
"""
function calc_steering(cc::CourseController, chi_set, heading, course;
                       t, elevation, v_kite, v_app, dmin, tangent)
```

All angles in radians, `elevation` included (the ladder converts internally — the scripts
should not have to know it compares degrees). State kept in `CourseController`:
`pid::DiscretePID`, `phase::Int`, `hold_start`, `entry_sign::Int`, plus the last-step
diagnostics the logs read: `chi_cmd`, `psi_prime`, `w_course`, `w_lim`, `err`, `K`. With
those as fields the four log lines in each script become field reads, and `var_05`…
`var_08` keep their present meaning exactly.

Exports to add: `CourseController`, `CourseControllerSettings`, `set_phase!`. (`calc_steering`
is exported already.)

## Step 2: migrate, in three verifiable stages

Each stage ends with `simple_fig8.jl` reproducing the baseline **bit-identically**; that
is the check that matters, and it is what caught the winch-refactor regressions.

0. **Baseline first.** Run all three scripts before touching anything and archive the
   printed metrics (`output/archives/<timestamp>/`). Without a pre-refactor number to
   compare against, "unchanged" is unverifiable.
1. **Core loop** — `src/course_controller.jl` with the PID, ψ' fusion, wrapped error,
   `1/v_app` schedule, `entry_gain`, park bypass and the `max_steering` clamp. Phase
   still passed in. Convert `simple_fig8.jl` only.
2. **Descent limiter** — `w_lim`, the `entry_chi_max` clamp and the `entry_sign` latch
   move in (`dmin` and `tangent` as scalars, so the course controller never sees a
   `FigureEightController`). Convert `simple_fig8.jl`.
3. **State machine + depower** — the ladder, `set_phase!`, `rel_depower`. Convert all
   three scripts and delete the duplicates.

## Step 3: tests — `test/test_course_controller.jl`

Pure arithmetic, no kite model, so it belongs in the same sub-second suite; include it
from `runtests.jl` next to `test_fig8_controller.jl`. Cover the traps, not the algebra:

1. **Bumpless park** — phase 0 returns exactly `0.0` while the PID is still stepped;
   the first phase-1 output is continuous (no step from a cold D path).
2. **Gain schedule** — halving `v_app` doubles the applied `K`; `v_app_min` clamps it;
   phase < 3 applies `entry_gain`.
3. **Clamp** — a 180° error saturates at exactly `±max_steering`, and with
   `heading_i = false` (the shipped value) there is nothing to wind up.
4. **Wrapped error** — `chi_cmd = 179°`, `ψ' = -179°` gives `+2°`, not `-358°`.
5. **ψ' endpoints** — `w_χ = 0` at `v_kite <= v_kite_heading` gives pure heading,
   `w_χ = 1` at `>= v_kite_course` pure course, monotone in between; `fig8_pure_course`
   forces 1 from phase 3; the `course_offset` is applied once and only once.
6. **Limiter** — `w_lim` is 0 below `entry_d_gate`, 1 above `+entry_d_blend`, linear
   between, and a hard switch at `entry_d_blend = 0`; `entry_chi_max = 180` disables it.
7. **Sign latch** — with `chi_set` within `entry_cut_margin` of ±180° the latched
   `tangent` sign decides, and the latch does not change afterwards; the blend takes the
   short way round the cut.
8. **Ladder** — advances 0→1→2→3→4 on the documented conditions, never backwards;
   `set_phase!(cc, 5)` can fire the same step as 3→4; `set_phase!` to a lower phase is
   rejected.
9. **Sign of the plant coupling** — a positive error yields a positive `rel_steering`
   (the unnegated convention), pinned as a regression test.

Numeric fixtures in the style of the existing `test_calc_steering`: a couple of
hand-checked `@test u ≈ …` values, so a refactor that changes a formula shows up as a
number and not just as a green suite.

## Step 4: docs

- [control_algorithm.md](docs/control_algorithm.md) §3 — links point at
  `examples/simple_fig8.jl` and `create_heading_pid` (V3Kite); repoint them at
  `src/course_controller.jl`. §"What is actually verified by running it" gains the new
  unit tests; the guidance bullet currently carries the whole claim.
- [thesis.md](docs/thesis.md) — the `w_course`/`fb` snippet is copied from the script;
  update it to the controller's code.
- [reelout_state_machine.md](docs/reelout_state_machine.md) — the phase table becomes the
  controller's documentation; say where the machine now lives, and that phase 5 is still
  the script's to set. The `.drawio` needs no change.
- The three script docstrings — the log-slot tables stay, the descriptions of the loop's
  internals shrink to a pointer.
- [README.md](README.md#L83-L86) TODO — strike the "heading PID / entry state machine"
  item once stage 3 lands.

## Verification

| check | criterion |
|:--|:--|
| `test/runtests.jl` | 120/120 still pass, plus the new file |
| `simple_fig8.jl` | every printed metric identical to the stage-0 baseline: RMS d, all 7 criteria, phase-4 `v_app` mean |
| `simple_reelout.jl` | all 8 criteria, and RMS d / force mean+CV / power mean / ring period / E identical |
| `simple_fig8_live.jl` | runs, and its GC/frame-deadline behaviour is unaffected (it is the only script that runs with `GC.enable(false)`) |
| `optimize_fig8.jl` | one short sweep still resumes and ranks — it drives `simple_reelout.jl` through `FCS_OVERRIDES`, so an `FC_Settings` field that stopped being read would show up here and nowhere else |

A metric that moves means the move changed behaviour; find out which of the substitutions
did it before continuing. Do not re-tune anything in this plan.

**"Bit-identical" in practice, confirmed at stage 1 (2026-08-17):** `wrap_to_pi`
(V3Kite) and `wrap2pi` (KiteUtils) agree to ~1e-16 per call, not exactly — different
boundary handling at the ±π edge — and that noise floor gets amplified over ~9000
closed-loop steps into ~1% differences in PEAK/EXTREME metrics (max cross-track
error, worst-lobe reach) while RMS/mean/laps/elevation/force and every pass/fail
criterion land identically. Confirmed as the expected bar for every later stage: same
criteria pass, RMS/mean/laps/elevation/force exact, peaks within ~1%. A LARGER move,
or one that changes RMS/mean/laps/criteria, is a real regression and must be run down.

## Out of scope

- **NDI convergence with `ParkingController`.** Both controllers solve the same problem
  differently: the parking controller inverts the plant (`linearize`, Eq. 6.12) and
  cascades an outer course loop onto a turn-rate loop; the fig8 controller is a single
  gain-scheduled PD. `azimuth`/`elevation` become real inputs only if the `c2` term is
  adopted. Worth a plan of its own, after this one has a test suite to protect it.
- **Any tuning change.** `data/fc_settings.yaml` is not touched.
- **The winch and the reel-out logic**, including phase 5's trigger and `l_set`.
- **`FigureEightController`**, which keeps its current interface; the course controller
  receives `dmin` and `tangent` as plain numbers, so the two stay independently testable.

## Open questions — resolved 2026-08-17

1. Decision 3 — fuse ψ' inside the controller. **Yes.**
2. Decision 4 — move the state machine into the controller. **Yes.**
3. `rel_depower` comes out of the course controller as a second return value. **Yes.**
