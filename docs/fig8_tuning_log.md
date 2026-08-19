# Figure-of-eight tuning log (`examples/simple_fig8.jl`)

Dated record of the parameter experiments behind `examples/simple_fig8.jl`.
It was extracted from that script's comments on 2026-08-02, so the script keeps
only the rationale for the values actually in use. Entries are newest first
within each section. **This is where new findings go** — not into the
`FC_Settings` docstrings, which state what a value is, not the history that
produced it.

**Where the references point.** The log was written in V3Kite.jl and moved here
on 2026-08-03, when the controller did (see `PlanMigrate.md`). Nothing in the
entries below was rewritten, so read them with this map:

- `PlanFig8.md`, `SmallPlan.md`, `PlanC1C2.md` and every `data/*.yaml`,
  `data/*.arrow` are **V3Kite.jl's**, on its `fig8` branch. Only the tuning log
  itself moved.
- The ALL-CAPS parameter names (`HEADING_P`, `PARK_TIME`, `MAX_STEERING`, …)
  were script globals at the time. They are now lowercase fields of
  `FC_Settings` (`heading_p`, `park_time`, `max_steering`, …) in
  `src/fc_settings.jl`, loaded from `data/fc_settings.yaml`.
- The guidance was `src/fig8_controller.jl` in V3Kite; here it is
  `src/figure_eight_controller.jl`.

Unless stated otherwise, metrics are measured over the settled window
(`t >= PARK_TIME + ENTRY_TIME = 25 s`), `d` is the cross-track error, "clamped"
is the fraction of samples with the steering command on `MAX_STEERING`, and
"criteria" are the ones printed by `print_fig8_metrics`. Entries dated before
2026-08-02 were scored against **four** criteria (laps, RMS d, max d, elevation
floor); the pattern-extent criteria (azimuth reach per side, elevation span)
were added on 2026-08-02, so "4 of 4" in an older entry is not a pass under the
current set — those runs were never scored on the size of the eight they flew.

**Reading the older entries.** Everything dated 2026-08-01 or earlier was
measured through the *relay* loop (`HEADING_P = 4.5`, steering clamped ~90-100 %
of the time). In that regime the loop was ~8x over gain and self-oscillating, so
its RMS/clamp numbers do not transfer to the current tuning. Its *energy*
observations (mean force rising as the pattern centre drops) do.

---

## SIM_TIME

`30 -> 150 -> 30` (2026-08-02). The 150 s runs were a deliberate one-off
exception to `SmallPlan.md`'s 30 s cap, taken after `HEADING_P 4.5 -> 0.6` made
the loop stable (RMS d 1.02°, tape 0 % rate-limited). The question the cap
cannot answer: at 30 s the kite is on the left lobe only (azimuth
-56.7..-6.5°) with `laps = 0`, and that is equally consistent with "it has not
got there yet" and "it has settled into a stable one-lobe orbit" — which need
opposite fixes. `laps >= 3.0` needs ~130 s of flight, and the first 120 s line
up with `data/fig8_reference.arrow` for a like-for-like read. Answered (4 laps,
4 of 4 criteria), so the value went back to 30.

The metrics window opens at `PARK_TIME + ENTRY_TIME = 25 s`, so a 30 s run
scores only its last 5 s and `laps` is meaningless — judge short runs on RMS d,
the elevation floor and the tape metrics, and go back to 150 for anything that
has to be counted in laps.

## DT — the explicit aero-structure coupling mode

`0.05/3 -> 0.05/6` (2026-08-02). Not a tuning parameter: a numerical stability
fix, costing 2x wall time per run.

The 150 s run at `HEADING_D_N = 2.0` stopped at `t = 46.07 s` on the overspeed
guard, but the guard reported a symptom one step late: `v_app` is FLAT at 27 m/s
and the tether force flat at ~3.1 kN until the final timestep, where `v_app`
jumps 26.6 -> 67.9 m/s in one `dt`. That is not the energy runaway that killed
depower 0.36 and `EL_CENTER` 32.8, which crept over seconds.

The real failure is a step-to-step (`2*dt` = 30 Hz) oscillation of the wing,
growing ~1.3-1.4x per sample from `t = 45.0`:

| t [s] | AoA zigzag | \|acc\| zigzag | v_app | force |
|------:|-----------:|---------------:|------:|------:|
| 45.0  | 0.14°      | 10             | 26.8  | 3142  |
| 45.2  | 1.6°       | 14             | 26.9  | 3165  |
| 45.9  | 6.5°       | 73             | 27.3  | 3108  |
| 46.0  | 47.7°      | 253            | 27.4  | 3323  |
| 46.05 | 39.3°      | 7621           | 26.6  | 3850  |

Centre-panel AoA and span-mean AoA alternate in ANTIPHASE each step and the kite
acceleration carries the same Nyquist content, so it is the structure shaking,
not a logging artefact.

Where it comes from: `step!` runs with `vsm_interval = 1`, i.e. the VSM aero load
is refreshed once per `dt` and held frozen inside the DAE in between. That is an
explicitly coupled aero-structure scheme, whose characteristic instability is
exactly a growing `2*dt` oscillation appearing first at maximum dynamic
pressure — which is where and when this one appears (bottom of the right lobe,
~3.2 kN, elevation ~21°). Halving `dt` halves the aero lag and gives ~4x margin
on the mode. `abs_tol`/`rel_tol = 0.01` in `data/settings.yaml` leave nothing to
absorb it either, but tightening them does not touch the coupling lag.

`HEADING_D_N` is not the cause and was not reverted: the `N = 2` and `N = 10`
trajectories agree to 0.3° in azimuth+elevation over the whole 46 s and to ~2 %
on per-lap peak force, and the surviving `N = 10` baseline carries the SAME
marginal mode for all 150 s (its Nyquist acceleration content is 10-30 m/s² in
every 5 s window, its AoA zigzag touches 0.99° at `t = 22 s` and recovers). The
model rides this mode permanently at this operating point; a roundoff-level
difference decides whether it trips.

Raising the in-plane `BODY_DAMPING` would also damp it, at the cost of the turn
authority the pattern needs, so that route stays shut.

This re-bases the trajectory: `data/fig8_run_N10_baseline.arrow` is no longer
bit-comparable, only comparable on per-lap metrics.

## DEPOWER_SETPOINT

**`0.36 -> 0.40` (2026-08-02)**, targeting energy, which is now the binding
constraint. With `HEADING_P 0.6` the loop is stable (RMS d 1.65°, tape 1 %
rate-limited) and the kite CROSSED THE CENTRE for the first time under a
non-saturated loop — azimuth -56.7..+45.0°, laps 0.5 — then diverged at
`t = 37.6 s` at the right-lobe tip, `v_app` 27 -> 42.7 m/s in one second.
`v_app` had crept from ~20 m/s in the first lobe to ~28 by the crossing: the
kite gains energy lap over lap and the right-lobe turn is where it cashes out.

This re-tested the 2026-08-01 verdict below, which was taken under the relay
loop — there the run never reached a second lobe at all, so "0.36 diverges" was
measured on a trajectory that had nothing to do with this one.

Side effect: depower moves the turn-rate law. `c1` 0.1830 -> 0.1513 (-17 %) and
the dead time 0.383 -> 0.420 s, so loop gain drops ~17 % and the delay grows
slightly. Both push towards more stability, so `HEADING_P 0.6` stays valid; if
tracking goes sluggish, that is the reason.

**`0.40 -> 0.36` (2026-08-01, -10 %): reverted.** The entry was turn-rate
limited (steering pinned at 0.300 for the whole dive), and depower is the only
lever left that changes the authority itself — `c1 = 0.1513` at 0.40 vs 0.3159
at 0.25, with the dead time falling 0.42 s -> 0.03 s. `PlanFig8.md`'s standing
note is that reel-out is what makes a low depower survivable, and the winch runs
in force mode, so the failure mode that closed this lever (overspeed in a
sustained turn) is the one the compliant winch relieves.

Result: NO. Diverged at `t = 21.7 s` on the overspeed guard, and violently:
`v_app` 93.6 m/s (the 0.40 run peaks at 26.6), tether force 6164 N vs 2434 N,
lift 11 kN with the drag going negative. The winch did its part — 13.4 m paid
out, more than the 10.6 m at 0.40 — and it made no difference, so reel-out at
this rate does NOT make a low depower survivable here. The entry was unchanged
too (steering still pinned at 0.300 through the dive), so the extra authority
never even reached the trajectory before the energy problem ended the run.
`PlanFig8.md`'s "reel-out unlocks low depower" note is not supported at 200 m
with this pattern.

**Earlier.** 0.40 is the middle of the two settings that each fail alone at
150 m: 0.25 is agile (`c1 = 0.3159`) but cannot survive a sustained turn
(open-loop divergence at 13 s, `v_app` 51 m/s); 0.55 survives but is far too
sluggish (`c1 = 0.1071`, dead time 0.55 s) and flew a circle instead of the
pattern. 0.40 survived 29 s open-loop at 150 m — better than 0.25, short of
0.55 — and is paired with the longer tether.

## TETHER_LENGTH

`150 -> 200` (2026-07-26): the minimum angular turn radius is
`rho = 1/(L*c1*u_s)`, so a longer tether lets the kite turn tighter in angular
terms — the single most effective lever on pattern feasibility after `c1`.

## WARMUP_TIME — the first second of every log was not flight

`0 -> 2.0 s` (2026-08-02). Symptom: a sharp peak in the logged L/D at
`t = 0.66 s`, i.e. 79 steps into the PARK, with the controller commanding
nothing.

Two candidates, and they need opposite fixes:

1. **A division artefact.** `compute_lift` is a norm and non-negative, while
   `compute_drag` is a SIGNED projection onto `v_a`. When the wing unloads that
   denominator passes through zero, and the old guard in `step!` was
   `wing_drag > 1e-6` N — a threshold that is strict at 40 m/s and meaningless
   at 8, because the force scale moves with `v_app^2`. A vanishing force
   divided into itself is an arbitrarily large L/D.
2. **A real transient.** `settle_wing` returns an equilibrium of the SETTLING
   model (`dt = 0.001`, damped, winch braked, so the length is held
   kinematically). The run integrates a different model: brake off, drum on a
   torque, aero at the run's `dt`. The settled state is not a fixed point of
   the model that starts at `t = 0`.

Tested in that order. (1) alone did NOT remove the peak: the gate was moved
onto the drag COEFFICIENT (`drag_floor`, `LD_CD_MIN = 0.01` — an order of
magnitude below the ~0.1 a loaded V3 wing carries) and below it `var_15`/
`var_16` are now `NaN`, a gap rather than a number. The peak survived, which
proves the drag never got that small and the ratio was finite and real.

(2) removed it. Two changes at the handover, both in `init`:

- `init_winch_torque!(sys)` immediately after `brake = false`. The function
  already existed and `init` never called it, so the drum was released holding
  whatever `set_value` the cached settled binary carried.
- `warmup!` (new, `interface.jl`): step the real model `WARMUP_TIME` seconds
  with zero steering, depower at the settled value and the winch in the mode
  the run will command, then replace the logger and `sys_state` so the run's
  first logged row is `t = 0` again. `warmup_force_mode` MUST match the winch
  the run will command (`COMPLIANCE > 0`; the flag was `WINCH_FORCE_MODE` when
  this was written) — warming up against the wrong winch hands the run the
  discontinuity the warm-up exists to absorb. The gains must therefore be
  scaled BEFORE `init`, since the warm-up runs inside it.

Confirmed rather than assumed: after the change `count(isnan, sl.var_15) == 0`,
so the peak is genuinely relaxed away and is not the new guard masking it. Had
that count been nonzero we would have shipped a hidden spike and believed it
fixed. The guard stays in as dormant protection for an unloaded wing mid-lap.

This is the same idea as `PARK_TIME`, moved before `t = 0`: the park lets the
settling transients decay before the controller engages, the warm-up lets them
decay before anything is logged. Cost is `WARMUP_TIME / DT` full steps (240 at
2 s) on every run. 2 s was chosen to cover the 0.66 s peak with margin, not
measured as the decay time.

## Entry state machine (`CHI_DIVE`, `CHI_HOLD`, `DIVE_EL_MARGIN`)

First results (2026-08-01, `EL_CENTER` 40.5, force mode, attractor 15):

- The phases fire as intended: park -> dive at 5.05 s, dive -> hold at 10.45 s
  (el 55.2°), hold -> fig8 at 11.65 s (az 17.4°, el 46.1°).
- With `CHI_DIVE = +100` the settled metrics came out IDENTICAL to the run
  without any state machine (RMS d 5.27°, span 1.3..42.6°, 0 crossings). Not a
  bug: `ENTRY_CHI_MAX` was already clamping the guidance course to ±95° during
  the descent, so the "new" entry commanded almost exactly what the limiter had
  been commanding. The state machine's value is that the entry is now explicit
  and steerable, not that it changed the flight.
- Flipping to `CHI_DIVE = -100` (entering from the RIGHT, as the reference does)
  is what actually moves the pattern: azimuth span 1.3..42.6° -> -4.7..+49.2°
  and 0 -> 1 centre crossing. Cost: RMS d 5.27 -> 5.99°, max 7.88 -> 9.17°,
  steering back to 100 % clamped (bang-bang, HF std 0.0000), elevation floor
  23.2 -> 21.3°.

Still not the reference geometry: it hands over at az 17.4° / el 46.1°, i.e.
5.6° ABOVE the pattern centre and nowhere near the rightmost point of a ±50°
eight. The reference hands over AT the centre elevation, at the far edge,
already turning down.

**`CHI_DIVE -100 -> -85` (2026-08-01).** The entry is turn-rate limited — the
kite never reaches the commanded course, so a descending command just adds to
the fall it is already taking while it turns out of the park. -85° asks it to
climb slightly, spending the turn on azimuth travel instead of altitude, which
is what the reference's ramp (98° -> 143°, shallow first) achieves.
Result: bit-identical to -100. Same phase times (5.05 / 10.45 / 11.65 s), same
handover (az 17.4°, el 46.1°), same settled span (-4.7..49.2°), same RMS d
5.99°. The steering command sits at exactly +0.300 for the WHOLE dive, so the
kite is turning as hard as the plant allows and the numeric value of the command
is irrelevant — only its SIGN reaches the plant. Within this authority the entry
cannot be shaped by `CHI_DIVE`, `CHI_HOLD` or `DIVE_EL_MARGIN` at all; the only
entry choice that exists is which side to enter from. A shapeable entry needs
more turn authority, i.e. a lower depower (`c1` 0.1513 -> 0.3159).

Sign, measured 2026-08-01: a POSITIVE commanded course drives the kite towards
NEGATIVE azimuth (+100° took it from az 0.1° to -18.6°). The reference enters at
the pattern's rightmost point, i.e. positive azimuth, so the command is negative
here. This is the opposite of the reference log's +98°, whose azimuth sign
convention is mirrored.

**`DIVE_EL_MARGIN 15 -> 5` (2026-08-01): worse, reverted.** At 15 the handover
landed at el 46.1°, 5.6° above the centre and well short of the pattern's right
edge; the reference hands over at the centre elevation and the hold itself costs
~1°, so 5 aimed the handover at ~centre + 4. Handover elevation came out right
(39.1° vs a 40.5° centre) but the azimuth went the wrong way: 17.4° -> 7.7°, and
the settled pattern lost its centre crossing (1 -> 0, span 0.5..45.4°, RMS d
5.99 -> 6.97°, min el 21.3 -> 20.2°). Why: during the hold the kite tracks
az 16.8 -> 7.7 while dropping 45.2 -> 39.1°, i.e. it flies down-LEFT, not
horizontally right as commanded. It never reaches the commanded course at all —
at `chi = -100°` the great-circle geometry gives ~5.7° of azimuth per degree of
elevation lost, which would be ~160° of azimuth over this descent; the kite
manages 17°. The entry is turn-rate limited, so lengthening the dive does not
carry it further right, it only drops it lower in the same place.

## Pattern size (`F8_A`, `F8_B`)

`50/20 -> 40/15` (2026-08-01): **reverted**. The premise was wrong and
`check_pattern_feasible` said so before the run even started: a SMALLER
lemniscate is a TIGHTER one. Tightest path radius 8.4° -> 6.4° against a kite
minimum of 6.3°, i.e. margin 1.33 -> 1.02, right at the limit. The run aborted
at `t = 19.1 s`, 7 s after handover, with the kite never getting past azimuth
19.2°. The reference controller flies 40/15 because its ram-air kite has several
times this kite's turn authority, not because a small pattern is easier. For the
V3 the margin improves with a BIGGER pattern or a longer tether
(`rho = 1/(L*c1*u_s)`).

The motivation for trying it: three separate levers — steering gain, entry
course, depower — had each failed to produce the lobe crossover, all because the
kite already turns at its physical maximum (steering pinned at 0.300 through the
whole entry) and still gains only ~17° of azimuth, with
`check_pattern_feasible` reporting a margin of just 1.19-1.30 throughout.

## EL_CENTER (pattern-centre elevation)

**`36.5 -> 26.0` (2026-08-02)**: the reference controller's centre, reached in
one step (the 10 %-ladder abandoned), and it SURVIVES the full 150 s. Taken
together with `VSM_INTERVAL 2 -> 1`, so the tighter aero coupling is part of why
this works where rung 2 at 32.8 died at 48.7 s.

| metric | 36.5° | 26.0° |
|:--|--:|--:|
| laps | 4.0 | 3.5 |
| RMS d | 2.90° | 4.09° (FAIL, < 3) |
| max d | 7.07° | 13.92° (FAIL, < 8) |
| min el, whole run | 23.7° | 10.88° (pass, > 10) |
| mean force | 2655 N | 2982 N |
| force CV | 15.7 % | 25.2 % |
| peak v_app | 27.9 m/s | 30.3 m/s |
| command clamped | 8 % | 2.2 % |

So 2 of 4 criteria, and the failure is TRACKING, not energy: the energy is flat
lap over lap (median `v_app` 28.5 / 27.1 / 27.8 / 27.4 m/s and peak force
3.9 / 3.6 / 3.6 / 3.6 kN over the four 25 s windows from `t = 50 s`), which is
exactly what every lower-centre attempt failed on before. The steering command
is off its clamp 98 % of the time and the tape is rate-limited only 1.4 % of the
time, so there is authority left to spend.

What binds now is the pattern's BOTTOM EDGE, as predicted in the 32.8 entry
below. At `B = 20` the bottom sits at 16°, and the kite undershoots it by ~5° on
EVERY lap: the 25 s window floors are 12.8 / 10.9 / 10.9 / 11.2 / 10.9°, i.e. a
repeatable structural undershoot, not a transient (lowest point `t = 98.3 s`).
That leaves only 0.9° of margin on `MIN_ELEVATION = 10°`, and it is also where
the 13.9° max cross-track error comes from. Next lever is `B` (and probably `A`)
coming down with the centre — the reference controller flies `A = 40, B = 15` at
this centre, i.e. a bottom edge at 18.5° rather than 16°.

**`36.5 -> 32.8` (2026-08-02, -10.1 %): rung 2 of the descent, reverted.**
Failed at `t = 48.7 s`, and the mechanism is worth having: coming round the
RIGHT lobe the kite undershot to elevation 17.9°, five degrees BELOW the
pattern's own bottom edge (22.8° at this centre). That put it deep in the power
zone and speed and force ran away together — 25.2 -> 32.6 m/s and 2404 ->
3811 N in the last five seconds. Same lobe and same signature as the
depower-0.36 failure at 37.6 s. Tracking was fine until it was not (RMS d 3.55°
includes the blowup). So the centre is not the binding parameter — the pattern's
bottom edge is.

**`40.5 -> 36.5` (2026-08-02, -9.9 %)**: step 1 of the descent towards 26°, the
reference controller's centre — a known-flyable operating point for this pattern
family, so the target was not arbitrary. Ladder: 40.5, 36.5, 32.8, 29.5, 26.6,
26.0. Result: 4 of 4 criteria, and BETTER — RMS d 2.90 -> 2.63°, max d
7.07 -> 6.22°, clamp 12 -> 8 %. The curvature-margin argument working as
predicted. Energy moved the wrong way but only slightly: mean force
2432 -> 2655 N (+9 %), peak `v_app` 27.0 -> 27.9 m/s, elevation floor
23.7 -> 20.5°. Crucially it does NOT accumulate — median `v_app` is 25.9 m/s
over the first half of the settled window and 25.4 over the second, i.e. flat to
falling. Azimuth span -51.6..+47.4°, so the pattern slightly OVER-flies the left
lobe.

What to watch on this sweep, and it is not RMS d: two forces pull opposite ways.
A lower centre IMPROVES the curvature margin (less `cos(elevation)` compression
of the azimuth axis) but pushes the pattern deeper into the power zone. Every
failure at low centre so far has been an ENERGY failure — `v_app` and tether
force run away, the tracking looks fine until it does not (depower 0.36 held
RMS d 1.65° right up to a 27 -> 42.7 m/s blowup at 37.6 s). So: peak `v_app`,
mean and peak force, and the elevation floor first; RMS d second. Run at
`SIM_TIME` 150, not 30.

**`45 -> 40.5` (2026-08-01, -10 %)**: continuing down once the floor was known
to be movable. Survives 30 s, so the old floor is gone for good. RMS d
6.79 -> 5.27°, max 10.02 -> 7.88°, force CV 15.2 -> 9.2 %, but the mean force
keeps climbing (2456 -> 2794 N) and the elevation floor is down to 23.2°.
Steering 96 % clamped (45° gave 92 %), and the pattern drifted FURTHER off
centre: azimuth 1.3..42.6°, ZERO centre crossings (45° had 2). Lower centres are
survivable but do not by themselves produce the crossover — the pattern is
offset to the right, which points at the entry/asymmetry, not at the centre
elevation.

**`50 -> 45` (2026-08-01, -10 %): re-test, and it works.** The failure recorded
below was on the fixed-length tether; the winch now runs in force mode (10.6 m
of travel, +2.1/-1.2 m/s), and that failure was an ENERGY failure — overspeed in
the power zone at 3494 N — which is exactly what a reeling-out winch relieves.
45° now survives the full 30 s where it aborted at 17.4 s on the fixed tether,
and the steering finally comes off the clamp (92 % vs 100 %). Cost: RMS d
4.93 -> 6.79°, max 7.77 -> 10.02°, force 1675 -> 2456 N (CV 15.2 %). The
elevation floor recorded below is therefore an artefact of the FIXED-LENGTH
winch, not a property of the kite.

**Earlier `45.0` attempt on the course-feedback loop (2026-08-01)**: 45° was the
lowest centre that FAILED the sweep below back when the loop regulated heading,
so it was a deliberate re-test of that limit. It still failed, in the same way:
solver abort at `t = 17.4 s`, at elevation 33° with `v_app` 27.5 m/s, tether
force 3494 N and AoA 45°. The better feedback signal does NOT buy a lower
centre, because the binding limit is energy, not tracking: cross-track error was
0.4-6.8° right up to the abort. The curvature margin even improves
(1.19 -> 1.30), as the sweep predicted.

**Original sweep in 10 % steps (2026-07-26, A=50 B=20, u_s=0.30, fixed tether):**

| centre | el span | margin | survived |
|--:|:--|--:|--:|
| 50.0° | 40-60° | 1.19 | 200 s (kept at the time) |
| 45.0° | 35-55° | 1.30 | 18.4 s |
| 40.5° | 30-50° | 1.33 | 13.9 s |
| 36.5° | 26-46° | 1.35 | 13.7 s |

Lowering the centre eases the `cos(elevation)` compression and so improves the
curvature margin, but pushes the pattern deeper into the power zone and the
energy limit binds first. At centre 50° the pattern is 224 m wide x 70 m tall,
564 m of path per lap, tightest radius 26 m.

## ATTRACTOR_DIST (attractor lead)

**`15.0 -> 12.1` (2026-08-02).** Swept 10..20 in 10 % steps, 150 s per run,
depower 0.40, `HEADING_P` 0.6, metrics from `t = 25 s`:

| attr | t_end | laps | RMS d | max d | az span | crossings | CV |
|--:|--:|--:|--:|--:|:--|--:|--:|
| 10.0 | 150 | 3.5 | 2.85 | 7.25 | -48.7..48.5 | 7 | 17.0 % |
| 11.0 | 150 | 3.5 | 2.87 | 7.18 | -48.1..47.9 | 8 | 16.4 % |
| 12.1 | 150 | 4.0 | 2.90 | 7.07 | -47.5..47.2 | 8 | 15.7 % (kept) |
| 13.31 | 150 | 4.0 | 2.97 | 6.92 | -46.8..46.5 | 8 | 15.5 % |
| 14.64 | 150 | 4.0 | 3.13 | 6.94 | -46.0..45.6 | 8 | 14.9 % |
| 16.11 | 150 | 4.0 | 3.36 | 6.89 | -45.2..44.8 | 8 | 14.1 % |
| 17.72 | 41.1 | 0.0 | 3.38 | 5.88 | -21.9..43.9 | 1 | 11.0 % |
| 19.49 | 40.4 | 0.0 | 3.58 | 6.42 | -19.9..43.0 | 1 | 10.7 % |

Three results. (1) RMS d rises MONOTONICALLY with the lead, 2.85 -> 3.58°, so
the minimum is at the bottom edge of the swept range and may lie below 10.
(2) There is a survival CLIFF between 16.11 and 17.72: both long leads die at
~41 s with one crossing and a one-sided span. (3) Force ripple trades the other
way, CV 17.0 -> 10.7 %, i.e. the gentlest force is at leads that no longer fly
the pattern. The flown azimuth span also SHRINKS with lead (±48.6 -> ±45°), so a
short lead flies a fuller eight, not merely a better-tracked one.

12.1 is kept over the 2.85° minimum at 10: it is the shortest lead giving the
full 4 laps and 8 crossings, passes RMS d < 3.0° with margin, has 1.3 points
less force ripple than 10, and sits clear of the cliff.

This INVERTS the 2026-08-01 sweep below, which found 35.1 optimal — that sweep
ran under the relay loop, where the plan itself recorded that guidance changes
made no difference; it was measuring a saturated controller, not the guidance.

**`35.1 -> 15.0` (2026-08-01).** A 35° lead is ~22 % of a lap, and part of its
advantage was the kite CUTTING CORNERS on a smoother, higher trajectory rather
than tracking the pattern. With the winch in force mode the cross-track error
was 12.5° RMS and the flown eight far too small, so the lead was shortened to
make the guidance follow the path instead of the chord. Result (force mode,
30 s): RMS d 12.46 -> 4.97°, max d 22.0 -> 7.9°, mean force 2621 -> 1673 N, and
the elevation floor holds at 35.5°. The shorter lead recovers all of the
tracking the moving tether cost. Two things it does not fix: steering is now
clamped 100 % of the time (was 93 %), and the settled pattern is one lobe,
azimuth -3.9..+47.8° against the ±50° reference.

**`19.8 -> 35.1` (2026-08-01)**, swept in 10 % steps to MINIMIZE the RMS COURSE
error (`course - chi_cmd`, both in the guidance convention), on the
course-feedback loop with down-loops, 30 s runs, depower 0.40 / 200 m /
centre 50°:

| attr | t_end | RMS chi (t>=15 s) | (t>=25 s) | RMS d (t>=25 s) | min el |
|--:|--:|--:|--:|--:|--:|
| 16.2 | 30.0 | 60.1 | 79.2 | 5.24 | 30.6 |
| 18.0 | 30.0 | 63.7 | 87.6 | 6.72 | 28.2 |
| 19.8 | 30.0 | 60.4 | 82.4 | 8.37 | 27.9 |
| 21.8 | 30.0 | 59.8 | 73.3 | 8.30 | 28.5 |
| 24.0 | 30.0 | 58.4 | 62.2 | 7.42 | 29.9 |
| 26.4 | 30.0 | 55.8 | 47.7 | 6.26 | 31.3 |
| 29.0 | 30.0 | 57.5 | 45.0 | 5.81 | 31.4 |
| 31.9 | 27.1 | 77.6 | 42.7 | 9.43 | 28.4 (DIVERGED) |
| 35.1 | 30.0 | 41.9 | 29.2 | 4.21 | 40.2 (min) |
| 38.6 | 30.0 | 45.7 | 36.7 | 6.51 | 40.2 |

35.1 is the minimum in both windows and also the best on cross-track error,
elevation floor (40.2° vs ~29°) and force ripple (CV 8.6 % vs 27 %). Three
caveats: (1) the landscape is NOT smooth — 31.9 diverges between two surviving
neighbours, so these are single 30 s runs of a marginally stable loop, not a
converged optimum. (2) RMS course error is ~42° even at the minimum, with
steering clamped 77 % of the time, so this picks the least-bad point inside a
saturated regime; the course oscillates ±60° about the command (measured, not a
wrap artefact — only 12 % of samples exceed 90°). (3) A 35° lead is ~22 % of the
~162° of arc in one lap, so part of the gain is corner-cutting.

**Earliest sweep, 200 s runs on the HEADING loop with up-loops** (kept because
it maps the low end):

| attr | survived | laps | RMS d | min el | saturation |
|--:|--:|--:|--:|--:|--:|
| 10.0 | 13.6 s | - | - | - | - |
| 14.6 | 200 s | 0 | 6.34° | 29.3° | 97 % |
| 16.2 | 200 s | 0 | 5.94° | 29.8° | 97 % |
| 18.0 | 200 s | 0 | 5.76° | 29.9° | 97 % |
| 19.8 | 200 s | 0 | 5.50° | 30.5° | 97 % |

The lead is NOT the lever for the lobe crossover: every surviving value circles
the left lobe (azimuth ~ -47..-25°) and never crosses the centre. It only trades
tracking quality, and monotonically the OTHER way than expected — longer lead
gives lower RMS and a higher floor. Below ~14° the command rotates faster than
the kite can follow given the 0.42 s steering dead time at this depower, and the
run diverges (10° died at 13.6 s after the course swung -154° -> -45° in 2.5 s).
The heavy steering saturation in every case is the real constraint: the
crossover is authority-limited. Interacts strongly with `HEADING_P`; tune
jointly.

## UP_LOOPS

Measured on the V3 (2026-07-26) on the HEADING loop: `true` -> survives 200 s;
`false` -> diverges at 17.8 s, all else equal. Up-loops shed energy through the
turn where down-loops convert height into speed, and the failure mode of that
configuration was overspeed.

Re-measured on the course-feedback loop (2026-08-01): down-loops now SURVIVE the
full 30 s at that centre, so the 17.8 s divergence was partly a feedback-signal
problem, not purely an energy one. At the then-current lead of 19.8 they tracked
worse (RMS d 8.37° vs 3.86° for up-loops); after retuning `ATTRACTOR_DIST` to
35.1 they were the first configuration recorded that actually CROSSES the
pattern centre (2 crossings in 18 s, azimuth -23..+33°) instead of circling one
lobe.

## HEADING_P

**`4.5 -> 0.6` (2026-08-02, factor 7.5)**, deliberately outside the
10 %-per-iteration rule, because 10 % steps provably cannot reach it. Derived,
not guessed:

- plant: `psi_dot = c1*v_a*u_s`, an INTEGRATOR with gain
  `c1*v_a = 0.183*20 = 3.66 rad/s` per unit `u_s` at flight speed
- loop: `omega_c = K * 3.66`, and at `K = 4.5*13.1/20 = 2.55` that is
  9.3 rad/s = 1.5 Hz
- delay: 0.72 s MEASURED (the steering tape's rate-limit lag, ~2x the 0.383 s
  small-signal dead time from `turn_rate_coeffs`)
- margin: a delay needs `omega_c*T_d <~ 0.8 rad`, so `omega_c <= 1.1 rad/s`,
  `K <= 0.30`, i.e. `HEADING_P ~ 0.46` at `v_app` 20. Against the optimistic
  0.383 s figure it is 0.86. 0.6 sits between the two.

The loop was running ~8x over gain, hence a relay: `K = 2.55` clamps at only
6.7° of course error, and 88.2 % of phase-3 samples exceeded that. Measured
consequence: the kite turned at a median 43.5 deg/s while the guidance asked for
a median 8.3 deg/s — a 5x overshoot, i.e. a self-sustained oscillation, NOT a
tracking deficit. The 40° median course error is the RESULT of that oscillation.

This also explains both entries below. `4.5 -> 2.0` was still ~4x over gain, so
the loop stayed a relay and the run "changed only how it saturates" — the
correct observation with the wrong cause attached to it ("authority-limited"; it
is RATE-limited, see `SmallPlan.md`).

**`4.5 -> 2.0` (2026-08-01): reverted.** The deliberate large cut (factor 2.25)
meant to pull the steering command off its clamp. It does NOT come off the
clamp — still 100 % saturated, but now as a pure BANG-BANG command (steering HF
std 0.0000, i.e. the command only ever sits at +0.30 or -0.30). RMS d
4.93 -> 4.49°, max d 7.77 -> 8.34°, but the force ripple triples (CV
10.4 -> 21.6 %) and the elevation floor drops 35.1 -> 31.8°. Pattern still one
lobe (-1.8..+46.9°). Bought 0.4° of RMS and cost a tripling of the force ripple
and 3.3° of elevation floor.

**`5.0 -> 4.5` (2026-08-01, -10 %)**: at `ATTRACTOR_DIST` 15 the steering command
was clamped 100 % of the time, so the loop was running open. Result: within
noise, as expected while saturated. RMS d 4.97 -> 4.93°, max 7.88 -> 7.77°,
min el 35.5 -> 35.1°, force 1673 -> 1675 N, still clamped 100 % of the time,
pattern still one lobe (-3.8..+47.8°). A gain the plant never applies cannot
change the flight: while `|u_s|` sits on its limit the loop is open.

## HEADING_D_N (derivative filter)

`10` (the `DiscretePIDs` default) `-> 2` (2026-08-02), after `u_s` was seen
carrying a visible high-frequency ripple. Measured on the 150 s 4-lap log,
settled window `t = 30..150 s`: the fed-back course and heading carry only
BROADBAND noise (~0.003°, no peak), while the command has 0.00505 RMS in
2..25 Hz, peaking at 7.95 Hz. That peak is not a plant mode — multiplying the
measured error-noise spectrum by the PID's own magnitude reproduces both the
shape and the 7.95 Hz peak, which sits where the rising D gain meets the falling
noise floor. With `N = 10` the corner is at 10.6 Hz and the HF gain `10*K`; at
`N = 2` it is 2.1 Hz and `2*K`. At the loop's own 0.1 Hz the D path contributes a
gain of 1.005 and 5.4° of phase lead either way, so this is a filter change, not
a gain change.

The delivered tape barely saw the ripple (HF RMS 0.00027, 19x smaller than the
command), so the cost was never flight quality — it was that command-side slew
(RMS 0.89 /s against the 0.2 /s tape limit) is mostly noise, which makes every
rate metric measured on `set_steering` unreadable.

Verified two ways. (a) Offline replay of the PID over the BASELINE log's own
error signal, so the only difference is the filter: HF (2..25 Hz) command RMS
0.00505 -> 0.00225 (-56 %), band ratios 0.77 / 0.54 / 0.43 / 0.39 over
2-4 / 4-8 / 8-15 / 15-25 Hz against 0.80 / 0.52 / 0.36 / 0.31 predicted, while
the 0.05..0.5 Hz content — everything the loop actually uses — moved
0.0549 -> 0.0551 (+0.4 %). (b) A 30 s run against the baseline over the same
25..30 s window: RMS d 3.14 -> 3.10°, min elevation 22.2 -> 22.3°, azimuth span
identical to 0.2°, and peak TAPE slew 0.171 -> 0.115 /s (-33 %) — same flight,
less actuator work.

Do NOT judge this on a 17..30 s window: there the command's own transients
(clamped 8 % of the time) dominate the HF tails through the P path and hide the
effect (band ratios only 0.92 / 0.79 / 0.71).

## V_KITE_HEADING / V_KITE_COURSE (feedback blend)

`5.0/10.0 -> 0.0/0.001` (2026-08-01) DISABLED the blend (`w_course == 1` at any
speed), i.e. pure course feedback. This reproduced the conditions the 2026-08-01
`ATTRACTOR_DIST` table was swept under, to test whether the blend was why that
table's 30 s survival at attr 35.1 no longer reproduced (three runs that day,
incl. two winch variants, all diverged at 17.6-18.0 s with min_el ~28° instead
of 40.2°). Restored to 5.0/10.0 (2026-08-02): with the phased entry the
transition looks good again, and the band now applies to the entry only, since
`FIG8_PURE_COURSE` bypasses it in phase 3.

## STEERING_TRACK_TAPE — closed

Tried 2026-08-02 at `STEERING_LEAD = 3`, and it is a disaster. Recorded so
nobody proposes it again:

| metric | 4-lap baseline | governor on |
|:--|--:|--:|
| RMS d | 2.90° | 58.10° |
| min elevation | 23.7° | -49.2° (!) |
| laps | 4.0 | 2.0 |
| peak tape slew | 0.200 /s | 0.030 /s |
| criteria passed | 4 of 4 | 0 of 4 |

The mechanism: pinning the command to within 0.010 of the measured position
makes THE CLAMP the binding rate limit instead of the tape, and the pair can
then only advance as fast as the gap allows — an effective actuator 6.7x SLOWER
than the real one. The kite lost the pattern and went below the horizon.

The premise was wrong anyway: since `HEADING_P 4.5 -> 0.6` the tape saturates
only 2 % of the time, so there was nothing to govern. Fixing the loop gain
already solved what this was meant to solve. A LARGER `STEERING_LEAD` is less
harmful, not more — but the whole idea is closed.

## MAX_STEERING — raising the authority is closed

Option 3 of `PlanFig8.md` (raise the authority to relieve the 97 % clamp
saturation) does not work on this plant:

- 0.30 -> survives the full 200 s
- 0.33 -> DIVERGED at `t = 30.9 s`, peak turn rate 949 deg/s, turn-rate HF
  45.6 deg/s (vs 0.45 at 0.30) — the loop goes violently unstable, not merely
  saturated
- 0.375 -> the PLANT itself diverges, in plain bang-bang oscillation with no
  controller (identification sweep, 2026-07-26)

So the usable authority ceiling is BELOW what the lobe crossover needs, and 97 %
saturation at 0.30 is not a tuning oversight but the plant's limit at this
depower. `c1` is linear over the range (0.1495 to `u_s` 0.374 vs 0.1513 to
0.300), so this is a real dynamic limit, not a modelling artefact. The remaining
levers change the operating point: reel-out (restores `c1 = 0.3159` and a 0.03 s
dead time by making a low depower survivable) or a 300 m tether.

## The lobe lift cannot be keyed on ELEVATION — but its azimuth key needs NORMALIZING (2026-08-19)

`el_offset_wing` raises the lobes over AZIMUTH. The obvious refinement — key it on
ELEVATION instead, since the kite sags deepest at the bottom of the pattern and
that is close to but not the same as the lobes — is implemented (`lobe_lift`,
`el_offset_wing_mode`) and **measured to be unaffordable**. The same measurement
found the azimuth version losing most of its delivery at a long tether, which is a
one-line fix and is where the residual droop comes from.

All of it is OFFLINE, on the two real optimizer paths and the archived log of
2026-08-19_011951 (depower 0.274, `el_offset_wing 1.5 / 10 / 8`). No run has been
flown with either mode yet.

**Where the kite actually flies low.** Every flown sample past 300 m of tether,
matched to the nearest point of the attractor trace and measured against the
pattern's own elevation centre, deepest tenth:

| | |
|---|---|
| how deep the kite gets | 5.6 deg below the pattern centre |
| where, in azimuth | \|az\| 5.6 … 10.3 deg (median 8.0) |
| the reference point it tracks | 0.73 … 0.96 of a half-span down (median 0.80) |
| its sag there | -2.9 deg, against -1.0 deg over the whole window |

So the low points DO sit in the bottom fifth of the pattern, which is what the
elevation key was meant to exploit — and they sit at the lobes at the same time.

**The paths.** `/init` + `/step` at 150 m (the run's own guess and conditions) and
the 380 m reply the last run left on the server:

| | 150 m | 380 m |
|---|---|---|
| azimuth amplitude A | 19.8 deg | **8.7 deg** |
| elevation half-span | 6.15 deg | **2.39 deg** |
| curvature margin, no lift | 0.95 | 1.02 |
| tightest point | \|az\| 18.4, 0.27 half-spans ABOVE the centre | \|az\| 8.6, 0.37 half-spans BELOW it |

**Scored with `check_pattern_feasible` at that length** (`c1` of depower 0.274),
"bottom" being how far the path's lowest point rose:

| profile | margin @150 m | bottom | margin @380 m | bottom |
|---|---|---|---|---|
| none | 0.95 | — | 1.02 | — |
| azimuth 1.5 deg, 10/8 deg (flown today) | 0.94 | +1.50 | **0.83** | **+0.68** |
| azimuth 1.5 deg, 0.50/0.40 of A | 0.94 | +1.50 | **1.01** | **+1.50** |
| azimuth 1.5 deg, 0.60/0.45 of A | — | — | 1.02 | +1.50 |
| elevation 1.5 deg, depth 0 | 0.50 | +1.50 | 0.05 | +1.50 |
| elevation 1.0 deg, depth 0 | 0.67 | +1.00 | 0.20 | +1.00 |
| elevation 0.2 deg, depth 0 | — | — | 0.79 | +0.20 |
| elevation 0.4 deg, depth 0.4 | — | — | 0.45 | +0.40 |

**Why elevation fails, in one line: the bottom of the pattern IS the lobe.** An
azimuth-keyed lift is full across the whole lobe, so it TRANSLATES the tightest
turn of the path vertically and changes no radius. An elevation-keyed one is on
its ramp exactly where the path is tightest — the 380 m path's minimum radius sits
0.37 half-spans below the centre — so it squashes the hairpin: 1.73 deg of radius
at that point becomes 1.35 at 0.2 deg of lift and 0.09 at 1.5 deg, in the same
place. Pushing the ramp below the tip (`depth` 0.4 … 0.7) does not escape it, and
narrowing the ramp FOLDS the path, elevation ceasing to rise monotonically along
it, once `1.5 * lift >= (1 - depth) * half_span` — at 380 m that bound is 1.6 deg
at `depth = 0` and 0.8 deg at `depth = 0.5`. Nothing in the grid buys clearance
more cheaply than the azimuth key: 0.2 deg of bottom rise for the same margin the
azimuth version spends on 0.68.

**The finding that IS worth a run.** `el_offset_wing_az = 10 deg` is a threshold
fixed in degrees against a pattern that shrinks to +-8.7 deg — so at 380 m the
lobes never reach the plateau, they sit ON the ramp. That costs both ways: only
0.68 deg of the 1.5 arrives at the bottom, and the pattern is deformed rather
than translated, which is where the 19 % of curvature margin goes (1.02 -> 0.83).
Reading the same two thresholds as FRACTIONS of each path's own amplitude
(`el_offset_wing_mode: "azimuth_frac"`, `0.5` and `0.4`, i.e. the flown 10/8 at
150 m) delivers the whole 1.5 deg at 380 m for 0.01 of margin. That is the
candidate for the residual droop, not the elevation key.

`traj_opt.droop_profile` in the run summary is new for measuring this on a flown
run: mean depth below the path's elevation centre in five bins of `|azimuth|/A`,
each with the reference's own depth and the sag there, and
`centre_to_lobe_deg` as the headline.

**FLOWN, and the fractional key buys nothing** (2026-08-19_072132 against
2026-08-19_072606, the only difference `azimuth` 10/8 deg vs `azimuth_frac`
0.5/0.4):

| | azimuth 10/8 | azimuth_frac 0.5/0.4 |
|---|---|---|
| laps / RMS d | 8.5 / 1.43 deg | 8.5 / 1.47 deg |
| min elevation, whole run | 10.1 deg | 10.1 deg |
| `centre_to_lobe_deg` | 0.70 deg | 0.81 deg |
| mean reel-out power | 8625 W (1.00 x) | 8629 W (1.00 x) |
| steering saturation / tape rate-limited | 7 % / 9 % | 7 % / 8 % |
| phase-5 margin of the last install | 0.83 | 0.84 |
| criteria | FAILED: azimuth reach +13.8 deg | the same |

**The offline sweep above was run on a path this configuration never flies.** The
380 m reply it used was left on the server by an earlier session and has an
azimuth amplitude of 8.7 deg; the paths these runs install measure A = 18.3 deg at
187 m, 13.9 at 298 m and 12.1 at 380 m. So `az_full = 10 deg` is 0.55 A at the
start and 0.83 A at the end — inside the pattern the whole way, never walking out
of it — and normalizing moves the delivered lift by ~0.2 deg over part of the
lobe, which is what the two runs show. The fractional key is kept because it costs
nothing and is the right form if a pattern ever does shrink past A ~ 10 deg, but
`azimuth` 10/8 stays the flown setting.

The lesson is the one this log already carries in another form: sweep on the path
the run WILL fly. `opt_trajectory` serves whatever the server last solved, which
is not that path unless the run just produced it.

## Caching failed optimizer requests: 88 s a run, on a run with no visible failures (2026-08-18)

A solve that FAILS costs far more than one that works — 2870 evaluations and 81 s
against 75 and 1.5 s, and in `reopt_blocking` mode the simulation is frozen for
every second of it. `examples/awetrim_client.jl` now records each failed request
in `output/opt_failure_cache.yaml` and refuses to send it again;
`clear_opt_failures()` forgets them, `opt_failure_cache: false` in
`data/traj_opt.yaml` disables the whole thing.

**The failures were already there and nothing reported them.** The 150 -> 380 m
run's summary showed 7 requests, 4 installed, no failures — because
`reopt_retry_el_offset` recovers from a second guess seed and only the final
outcome was recorded. The cache made them visible on its first run: the first
seed (guess el 22 deg) fails at **L = 247.4 m** and **L = 354.2 m**. The retry at
27 deg converges, so the run looked clean and simply took longer.

Measured over three otherwise identical runs:

| | blocked_s | note |
|---|---|---|
| failures attempted | 157.9 s | both seeds sent, first fails |
| both cached | 70.4 s | first seed never sent |
| both cached | 68.9 s | repeat |

**88 s of held simulation per run, on a run that reported no failures at all**,
with a bit-identical result (8519 W, all 8 criteria, same install margins).

Two things this exposed. The summary's `reopt.events` dict was keyed by simulated
time alone, and a blocking solve is requested and collected on the SAME step — so
a skip and the install that follows it collided and the install won silently. That
is why the first cached run showed no skips at all and looked like the cache had
not fired; colliding keys now get a `_2` suffix. And the key must stay exact: the
same evening's 422 entry has 180.0 m converging where 180.00027 m throws, so a
bucketed length would cache a failure against a length that works. The cost of
that choice is that a request only ever hits when the run repeats it exactly —
true here, since these runs are deterministic, but not after a change that moves
the lap boundaries.

## Lifting the LOBES flattens the sag a rigid shift cannot reach (2026-08-18)

`el_bias` and `el_offset_final` move the whole pattern up together, but the kite
does not sag uniformly: past 300 m of tether it flies its centre at 14.4 deg and
its lobes at 13.0 deg, a **1.36 deg droop** the rigid correction can only answer by
raising everything (which costs reel-out power for elevation the centre did not
need). `el_offset_wing` adds elevation as a smoothstep function of azimuth — zero
in the centre, full past `el_offset_wing_az` — baked into every path the run
installs, including the startup one, BEFORE the feasibility and height gates.

**The ramp width is the whole game, and it is not monotone.** Swept on the real
150 m optimized path at `el_offset_wing = 1.5`, `el_offset_wing_az = 10`
(plain margin 0.94):

| blend [deg] | 2 | 4 | 6 | 7-10 | 11 | 12 | 14 |
|---|---|---|---|---|---|---|---|
| margin | 0.16 | 0.43 | 0.87 | **0.94** | 0.70 | 0.52 | 0.45 |

A NARROW ramp is a corner, and the corner becomes the tightest point of the whole
pattern: at blend 4 the minimum radius moves off the lobe (azimuth -18.4 deg,
4.1 deg) and onto the ramp (azimuth -6.4 deg, **1.86 deg**) — the first attempt at
this was refused outright by the startup gate at margin 0.31. A WIDE ramp reaches
into the CROSSING, where the path already turns hardest: blend 12 starts the lift
at |azimuth| = -2 deg, i.e. inside the centre. The plateau is a ramp that starts
just outside the crossing and is full by the lobes — keep
`el_offset_wing_az - el_offset_wing_blend` in 0 … 3 deg.

An earlier version of this measurement, taken on a hand-built lemniscate standing
in for the flown path, put the plateau at blend 4 and was WRONG in the direction
that matters. Sweep on the path the run will actually fly; `opt_result` is left in
`Main` after a startup-gate failure, which is the cheapest way to get it.

**Measured at the plateau** (`el_offset_wing = 1.5`, `az = 10`, `blend = 8`, so the
lift is 0 inside 2 deg, 0.47 deg at 5, 1.27 deg at 8 and 1.5 deg past 10; against
the same run with the profile off, both with the in-air gate fixed):

| | off | on |
|---|---|---|
| centre-to-lobe droop past 300 m | 1.36 deg | **0.53 deg** |
| min elevation, whole run | 8.8 deg | **10.0 deg** |
| RMS d | 1.22 deg | 1.17 deg |
| steering saturation | 6 % | 4 % |
| tape rate-limited | 8 % | 6 % |
| tether force CV | 5.0 % | 5.3 % |
| mean reel-out power | 8524 W | 8502 W |
| vs predicted | 0.98 x | 0.99 x |
| criteria | all 8 | all 8 |

1.2 deg of clearance at the run's lowest point, and a third off the steering
saturation, for 0.26 % of the power. The curvature margin is untouched at every
length: 0.94 -> 0.94 at 150 m, 2.39 -> 2.38 at 380 m, 1.86 -> 1.84 at
`depower_final`.

Not tried: making the profile a function of ELEVATION rather than azimuth (the sag
is deepest at the bottom of the pattern, which is not exactly the lobes), or
learning its shape per azimuth bin instead of fixing it. The bins above are the
data either would start from.

## The in-air elevation-shift gate measured the SAMPLING, not the curve (2026-08-18)

`simple_opt_reelout.jl` has two ways to get a learnt elevation correction to the
kite: the re-optimizer's next reply can carry it (an "install"), or, when no solve
is due, it is blended onto the path already in the air. **The second one had never
once succeeded.** Recorded per attempt over a 150 -> 380 m run:

| t [s] | shift | in-air margin | install margin, same reply |
|---|---|---|---|
| 48.3 | +0.37° | 0.48 | 0.83 |
| 61.4 | +0.14° | 0.24 | 0.95 |
| 89.1 | +0.06° | 0.49 | 1.03 |
| 122.2 | +1.50° | 0.67 | 0.78 |
| 140.0 | +1.25° | 0.35 | — (none left) |

Every one below `min_feasibility_margin = 0.74`, while the install gate passed the
same curves at 0.83 … 1.03.

**The cause is resolution, not curvature.** `/trajectory` answers with a 99-point
polyline; what is FLOWN is that polyline resampled to `n_path` (360), and
interpolating extra points onto a polyline concentrates each vertex's turn into one
short segment — so `path_radius_profile` reads a far tighter pattern than the curve
is. The install gate already knew this and scores the reply at its own resolution
(`n_native`); the in-air gate scored `fec.az_path` as flown. The seven curves this
gate refused, re-scored offline:

| points | margins |
|---|---|
| 359 (as flown) | 0.25 … 0.70 |
| 180 | 0.48 … 1.10 |
| **98 (the reply's own)** | **0.92 … 1.10** |
| 60 | 0.84 … 1.12 |

Stable from 98 down; 359 is the outlier. The same effect on a clean lemniscate:
2.95 at 99 points, 2.87 at 360 sampled from the smooth curve, **0.83** once those 99
are interpolated up to 360.

The consequence was not cosmetic. It meant `el_bias` reached the kite ONLY at an
install, so once `max_reopt` was spent the phase-5 learner (`el_bias_gain_final`,
+1.25° in the run above) went nowhere, and `el_offset_lead` could do nothing at all
unless it happened to fire before an install — which is why a 2.5 s lead was
provably inert (a lead-2.5 run was BIT-IDENTICAL to a lead-0 run in every logged
channel) while the 8 s lead of the earlier session did change runs.

**Fixed** by scoring the in-air check at `chk_points` — the resolution of the reply
the path in the air came from, seeded from the startup path and updated at every
install. Only the CHECK is downsampled; the flown path keeps `n_path` points, which
the lap counter depends on. Measured, same settings, `el_offset_lead = 0`:

| | before | after |
|---|---|---|
| shifts delivered in air | 0 of 7 | 8 of 8 (margins 1.06 … 1.39) |
| laps | 7.0 | 8.0 |
| RMS d | 1.58° | **1.22°** |
| min elevation, whole run | 8.4° | **8.8°** |
| steering saturation | 8 % | 6 % |
| tape rate-limited | 10 % | 8 % |
| mean reel-out power | 8507 W | 8524 W |
| criteria | all 8 | all 8 |

Phase 5 is where it shows most: the run's lowest point used to fall 4.9 s into
phase 5 at 8.45°, inside the blend that was still ramping the lift in. It is now
10.57°, 23.5 s in — the dip at the phase 4 -> 5 handover is GONE, because the
correction arrives continuously instead of in one step at the last install.

Two cautions. A path flown higher is flown NARROWER: with `el_offset_lead = 2.5`
the mean per-lobe azimuth reach landed on 70 % of `A` and failed the size criterion
by hundredths of a degree (`min_span_frac = 0.7`), where the `lead = 0` run passed.
And the net unwrapped course over phase 5 moved from -0.04 to -0.53 turns — but
that number is a DRIFT metric, not a loop detector: the kite still crosses the
pattern centre 3 times in those 25 s and flies a WIDER phase-5 pattern than before
(azimuth -12.5…+12.9° against -10.9…+8.5°). Nothing circles.

## Phase 5 is flown with 22 % less turn authority than any gate checked (2026-08-18)

The feasibility gate of `simple_opt_reelout.jl` — at startup and at every
re-optimization install — is evaluated with the `c1` of `depower_setpoint`.
Phase 5 (final) flies `depower_final`, and `c1` falls steeply over that step:
`turn_rate_coeffs([0,0,40], 0.275)` gives **0.2752 1/m**, at 0.328 it is
**0.2133 1/m**, both interpolated — **-22 %**.

The margin is exactly linear in `c1` (`margin = path_radius / kite_radius` and
`kite_radius = rad2deg(1/(L*c1*u_s))`), so every margin the log quotes is
**0.775x** that in phase 5, at the same length and on the same path. That
matters because the flown path is not the startup one: the run of
2026-08-18_213634 installed its last path AT `reelout_l_max` with margin 1.00,
i.e. **0.78 in phase 5** — above `min_feasibility_margin = 0.74`, but by 5 %.

**Measured, run 2026-08-18_215919** (150 -> 380 m, 6 m/s, no turbulence, the same
settings as 2026-08-18_213634): all 8 criteria pass, 8507 W at 0.98 x predicted,
RMS d 1.58 deg, min elevation 8.4 deg — i.e. unchanged, the diagnostics cost the
run nothing. The phase-5 margin of the successively installed paths, all read at
`reelout_l_max` so only the path differs:

| install | L installed at | margin at `l_now` | phase-5 margin at 380 m |
|---|---|---|---|
| 1 | 184 m | 0.98 | 1.58 |
| 2 | 216 m | 0.83 | 1.14 |
| 3 | 250 m | 0.95 | 1.12 |
| 4 | 284 m | 1.03 | 1.07 |
| 5 | 318 m | 1.03 | 0.96 |
| 6 | 353 m | 1.02 | 0.85 |
| 7 | 380 m | 0.78 | 0.78 |

The two columns are NOT comparable install by install — the left one is read at
`l_now`, and early on the length dominates the depower. What the right column
shows is that the re-optimizer returns a **tighter path every time** as the tether
grows (and the learnt `el_bias`, +2.20 deg by the end, compresses the azimuth
axis further), so phase 5 inherits the worst path of the run at the lowest turn
authority of the run: **0.78, 5 % above the gate**.

Install 7 also shows the depower-aware gate biting for the first time: it landed
at t = 128.4 s, AFTER the lift latched at 124.7 s, so `phase >= 5` and the gate
itself used `depower_final`'s `c1` — 0.78, where the old gate scored the same path
1.00. It passed, but a slightly tighter reply would now be rejected where it used
to be installed.

What is in the code now (not a new refusal — the run has already made its power
by then, and rejecting the best-power curve over the last few laps costs more
than it buys):

- startup prints `c1` at `depower_final`, its change vs the pattern's, and the
  margin at `reelout_l_max` on the path lifted by `el_offset_final`; it warns if
  that is below `min_feasibility_margin`.
- every re-optimization install reports its phase-5 margin next to its own
  (`margin 0.78 (phase 5: 0.78 at 380 m)`), and warns once per run when it drops
  below the gate.
- the two IN-AIR curvature checks (the elevation-shift blend and the
  re-optimization gate) now use `depower_final`'s `c1` from phase 5 onward, so a
  shift installed in phase 5 is scored with the authority that will fly it.
- `traj_opt.feasibility.c1_final` / `margin_final` / `margin_final_flown` in the
  run summary.

Not addressed: the lift `el_offset_final` compresses the azimuth axis by
`cos(elevation)`, so it tightens the path a little exactly where the authority
is lowest; and `el_bias` (learnt, +2.2 deg in that run) is on top of it and is
unknown at startup, so the startup number is optimistic by whatever the run
learns.

## Learning the elevation bias: close on the OPTIMIZER's curve, not the sag (2026-08-18)

The kite tracks below the path it is given in every segment of every run: the
mean over a lap is 0.7 deg at 150 m growing to 1.9 deg at 380 m, and 2-3.5 deg at
the pattern's extremes. `fcs.el_bias_gain` learns a correction once per lap and
raises every path installed afterwards by it (`simple_opt_reelout.jl`).

**The first version did not converge.** It fed back the measured sag — the error
against the path in the air — and the correction integrated to the clamp in seven
laps while the error never moved:

    lap_1: -0.73 -> +0.36    lap_5: -0.98 -> +1.86
    lap_2: -0.69 -> +0.71    lap_6: -1.12 -> +2.42
    lap_3: -0.75 -> +1.08    lap_7: -1.19 -> +3.00  (clamp)
    lap_4: -0.56 -> +1.37    lap_8: -2.22 -> +3.00

Of course it does not: the kite sags below WHATEVER path it is given, so raising
the reference raises the kite with it and leaves the sag exactly where it was.
The quantity that has to converge is the error against the curve the optimizer
solved for, i.e. the measured sag PLUS the correction already in the flown path.
One line, and it converges — same run, `el_bias_gain: 0.5`:

    lap  err vs optimizer   sag    correction
    1        -0.73         -0.73     +0.36
    2        -0.33         -0.69     +0.53
    3        -0.22         -0.75     +0.64
    4        +0.12         -0.52     +0.58
    5        -0.35         -0.92     +0.75
    6        -0.34         -1.09     +0.92
    7        -0.23         -1.15     +1.04
    8        -0.88         -1.91     +1.48

The correction tracks the sag as the tether grows and the optimizer-relative
error stays inside ~0.4 deg until the last lap, where the sag jumps by 0.8 deg in
one lap and the 0.5 gain cannot keep up.

Result against the same run without it (150 -> 380 m, 6 m/s, no turbulence):

| | off | on |
|---|---|---|
| min elevation, whole run | 7.1 deg | 8.1 deg |
| flown vs the OPTIMIZER's curve, last segment | -3.5 / -2.0 deg | -2.3 / -0.4 deg |
| measured power | 8491 W (0.98 x) | 8501 W (0.99 x) |
| RMS / mean / max d | 1.52 / 1.28 / 4.99 deg | 1.52 / 1.28 / 4.99 deg |
| steering at max / rate-limited | 9 % / 10 % | 8 % / 10 % |

The cross-track numbers are unchanged to the digit, and that is the correct
outcome, not a null result: the sag stays in the error by construction, because
the error is measured against the path that was commanded. What the correction
moves is WHERE the pattern is flown. `el_bias_max` had to go to 4.0 for the same
reason — the correction converges TO the sag, so a 3.0 clamp binds at 380 m.

Worth 1 deg of ground clearance and ~0.1 % of power here. It is worth more where
the elevation floor is the binding criterion, which it was two runs earlier.

**State 5 needs its own gain, and its own way in.** The sag deepens once the winch
stops (1.0 -> 1.8 deg over the last lap) and phase 5 has one or two laps to learn
in, so `el_bias_gain_final: 1.0` against 0.5 for phase 4. That alone would have
done nothing: a correction only ever reached the kite through a path INSTALL, and
by phase 5 `max_reopt` is spent — in the run before this one lap 8 learnt +0.44 deg
and it went nowhere. So a lap whose correction moved now blends the DIFFERENCE
onto the path in the air, gated on the curvature margin at the current length.
A fresh candidate takes the whole correction, the path in the air what has changed
since it was installed.

**A fixed lift on top of it, from state 5 on** (`el_offset_final: 1.5`): once
reel-out has ended there is no reel-out power left to trade for height, and height
is clearance. It is a SETPOINT move, not an error — the path becomes
`optimizer + el_bias + offset` while the learning still closes `err_el + el_bias`
on the optimizer's curve, so the offset cancels out of the update and the two
compose instead of fighting. It arrives at the 4->5 transition, which is not a lap
boundary, so the loop now tracks `el_target` (what the path should carry) against
`el_applied` (what it does) and blends the difference in whenever they differ and
nothing else is installing.

Measured in phase 5, after the 4 s blend has landed: the flown pattern sits
9.8-16.8 deg where the run before it flew 8.1-15.6, i.e. its bottom is now ON the
optimizer's curve (-0.2 deg) and its top 2.3 deg above. Minimum elevation while
the lift is in force: 9.8 deg against 8.1. It costs tracking in phase 5 — RMS d
1.98 -> 2.44 deg there, 1.18 -> 1.25 in phase 4, 1.39 -> 1.58 over the run — part
transient (the blend plus the climb), part the `cos(elevation)` compression a
higher pattern pays. Power 8508 W, all 8 criteria pass.

The whole-run minimum elevation barely moves (8.1 -> 8.3 deg) because it is set in
phase 4, and the dip at the START of phase 5 happens before the lift has blended
in — measured at t = 128.6 s, 3.8 s INTO phase 5, inside the blend.

**The stop latch is not early enough — and with `reelout_softstop: 0` it does not
exist.** Triggering the lift on `stop_start` reproduced the previous run bit for
bit: no soft-stop, no latch, so it fell through to phase 5 exactly as before. With
reel-out ending abruptly there is nothing earlier to hook onto, so the lift has to
be ANTICIPATED: `el_offset_lead: 8.0` puts it in once the length left is under
`v_reelout * el_offset_lead`, ~19 m at 2.4 m/s. One-way latch, so a wobbling
reel-out speed near the threshold cannot chatter the reference.

**The lead makes the kite CIRCLE in phase 5, and is off again.** Net heading
rotation over phase 5, seven runs: -0.07 turns in each of the four flown with the
lift at the phase transition, +1.39, -0.63 and +1.39 in the three flown with
`el_offset_lead: 8.0`, with a one-sided steering bias (mean u_s +6.1 % against
0.0). It is NOT turn authority — the circling runs saturate LESS (5.4 % of phase 5
against 14.6 %), so the kite is being commanded around rather than failing to
turn. Mechanism unknown; `el_offset_lead` is back to 0 and the confirming run is
clean (-0.08 turns, mean u_s -0.1 %). The dip it was meant to fix is back with it:
min elevation 8.3 deg.

**These runs do not repeat.** Two runs with byte-identical settings and identical
code (20:28 and 20:33) gave RMS d 1.25 vs 1.61 deg and 8 vs 9 laps. The archived
re-optimization events show the cause: the server answers the same request with
slightly different paths (margin 1.00 vs 0.99, clearance 45.3 vs 44.8 m at the
same 216 m), and the run diverges from there. That scatter is the size of most
single-run effects in this section — the elevation-bias and lift results each held
over several runs, but any number quoted from ONE run here is indicative only.

**The lap the lift lands in also broke the learner, which is worth recording.** It
is a CLIMB towards the new reference, and averaging it reads the transient as
sag:

    lap_7:  err -0.35 -> correction 1.31
    lap_8:  err -1.40 -> correction 2.72   (the lift lands mid-lap)
    lap_9:  err +1.88 -> correction 0.83   (unwinds it)

It ended at 0.83 deg against an actual sag of ~1.3. Skipping the learning update
for the lap the lift landed in fixes it — accumulator reset, no update — and the
run settles: lap 8 reads -0.39 deg and the correction holds at 1.29.

The run with the skip in place read RMS d 1.25 deg, min elevation 8.6 deg, 8518 W
at 0.99 x predicted and 0.7-1.4 deg ABOVE the optimizer's curve in phase 5 — the
lift delivered. Read it against the scatter above, not as a step change: the next
run with the same code and settings gave 1.61 deg.

Flown pattern against the OPTIMIZER's curve at the end of the run (bottom / top of
the pattern), same conditions each time:

| | bottom | top |
|---|---|---|
| correction off | -3.5 deg | -2.0 deg |
| gain 0.5 throughout | -2.3 deg | -0.4 deg |
| 0.5, then 1.0 from state 5 | **-1.4 deg** | **+0.2 deg** |

That last run had its 7th re-optimization FAIL, so nothing was installed after
118.1 s and the whole phase-5 correction went in through the in-air blend — which
is the mechanism working, not a confound. RMS d 1.52 -> 1.39 deg, steering at max
8 % -> 7 %, power flat at 8502 W. One criterion flipped to FAILED: azimuth reach
+13.5 deg against a 13.8 deg threshold, 0.3 deg short — and that threshold is 69 %
of `f8_a`, i.e. referenced to the PARAMETRIC pattern the run does not fly, so it
is a weak signal. Still, the flown pattern IS ~0.5 deg narrower than the run
before it; whether that is the correction or the failed re-optimization needs
another run to separate.

## ATTRACTOR_DIST 10 -> 8 on the reel-out settings (2026-08-18)

The overrun below said the lead was too long for the pattern the reel-out ends
on. Flown at 8.0 deg (`fc_settings_reelout.yaml`), everything else equal, 150 ->
380 m, 6 m/s, no turbulence:

| | 10.0 deg | 8.0 deg |
|---|---|---|
| RMS d | 2.08 deg | **1.40 deg** |
| mean / max d | 1.80 / 5.43 deg | 1.18 / 4.99 deg |
| laps | 6.0 | 7.0 |
| min elevation, whole run | 5.3 deg | 7.1 deg |
| steering within 2 % of peak | 13 % | 8 % |
| tape rate-limited | 17 % | 10 % |
| chatter (`hf_std_steering`) | 0.0214 | **0.0056** |
| criteria | FAILED min elevation > 6.5 deg | **all 8 passed** |
| measured power | 8491 W (0.98 x) | 8508 W (0.98 x) |

The overrun is gone with it: the kite never comes closer to its attractor than
1.54 deg (was 0.20 deg, i.e. through it), no sample under 1 deg, the largest
single-step change in `chi_set` is 30 deg (was a 180 deg swing), and no
saturated sample anywhere in phase 4+ flips sign. Around 134 s `u_s` now walks
32 -> 29 -> 22 -> 16 -> 11 -> 7 % with the attractor 5.8-9.5 deg away.

Power is unchanged, so the 30 % of cross-track error this bought is free. Note
the lead was NOT the whole story of the entry below — 8 deg is still a fifth of
the 40.5 deg pattern the run ends on — but at 8 deg the tracking is tight enough
(1.4 deg RMS against a 4.8 deg tall pattern) that the reference and the position
no longer disagree at the scale of the pattern.

## `u_s` saturates at the crossing because the kite overruns its attractor (2026-08-18)

With Q finally stable (see the entry below), the 18:16 run still saturates `u_s`
for ~0.6 s at 134.1 s. It is NOT a branch flip any more: Q's index rises
monotonically 208 -> 272 across the event and the cross-track error stays at
3.3-3.6 deg. What collapses is the distance from the KITE to its attractor:
2.52 -> 1.50 -> 0.75 -> 0.21 deg at 134.17 s. The kite flies THROUGH the point it
is steering at, so the great-circle bearing to it swings 180 deg (`chi_set` -103
-> -178 -> +102), the regulated error reads ~180 deg and the command pins to
`max_steering`, then flips sign as the attractor passes.

The geometry behind it, at 381 m of tether:

- The path is 17.8 deg wide, 4.8 deg tall, 40.5 deg of arc. At 150 m it was
  +/-19.7 deg wide and 12.4 deg tall — the optimized pattern shrinks ~2.5x in
  angle as the tether triples, because it keeps its PHYSICAL size.
- `attractor_dist` is 10 deg, i.e. a QUARTER of the whole pattern. The path
  rounds the left lobe tip in less arc than that, so the point 10 deg ahead of Q
  sits spatially on top of the kite.
- The kite is 3.4 deg below its reference on a pattern 4.8 deg tall (2.08 deg RMS
  over the run). Its leftward pass along the upper branch therefore coincides
  with where the lower branch runs — the reference and the position are
  inconsistent at the scale of one pattern height, and no closest-point guidance
  is well behaved there.

Three law changes were tried and all made it worse, scored by total |chi_set|
rotation over 126-140 s of that run (current law: 594 deg):

| lead rule | rotation | worst single step |
|---|---|---|
| arc 10 deg ahead of Q (current) | 594 deg | 42 deg |
| arc capped at 15 % / 10 % / 6 % of the path | 684 / 861 / 1105 | 165 deg |
| L1 circle, first point 10 deg from the kite | 832 deg | 103 deg |
| blend to the tangent at Q below 2/3/5 deg | 678 / 639 / 590 | 107 deg |

So this is not a Q-selection bug and not fixable by another guard. It is the
pattern being small: the levers are a shorter `attractor_dist` at long tether,
a pattern that stays larger in angle (shorter reel-out, or a guess/objective that
does not shrink it), or tighter tracking. All three need a re-fly to judge — the
table above is an OPEN-LOOP replay, scoring alternatives on a trajectory the
current law produced.

## The Q continuity window was never active — `search_window` vs a small path (2026-08-18)

The `cycle` trace of the 150 m reel-out run rings at the end: 8 -> 9 -> 8 at
134.4 s and again 9 -> 8 at 136.7 s, held for 2.7 s. Not the kite. `fig8_n`
integrates the step-to-step change of `fec.last_idx`, and Q was hopping between
the two branches of the crossing — the logged `dmin` (`var_01`) alternates
between the two local minima, 0.05-0.6 deg on the branch the kite is on and
3.2-3.4 deg on the far one, and the index between ~230 and ~120. That is 110
points, under the `n_path/2` unwrap threshold, so the counter took it for flight
progress worth 0.3 of a lap.

The cause is that `search_window` is an ABSOLUTE arc, 45 deg, and
`calc_attractor` fell back to a global search whenever the half-width reached
`n/2`:

    half = round(Int, n * search_window / total_len)
    half >= n / 2 ? (1:n) : window

That triggers for every path shorter than 2 * 45 = 90 deg of arc. The optimized
pattern at 343 m measures 41.5 deg in total, the shipped lemniscate (A = 30,
B = 12) about 90, and the test controller (A = 10, B = 5) 37 — so the continuity
guard has been off in every configuration this repo flies, and only `branch_tol`
(3 deg, comparable to the branch separation on a pattern this small) was
deciding which branch Q sat on. The share of samples where the logged `dmin`
exceeds 2 deg climbs through the run as the pattern shrinks in angle: 9 % at
40-50 s, 38 % at 100-110 s, 69 % at 130-140 s.

Fixed in three places:

- `search_window_max_frac` (0.125) caps the half-width at that fraction of the
  path's own arc length, so the window scales with the pattern instead of
  switching off. The far branch is then not a candidate at all.
- `reacquire_margin` (3 deg) is the second exit from the window, needed once the
  window actually bites: Q leaves it when the best point on the whole path is
  that much closer than the best inside it. Without it a window that went stale
  at a path install, or after an excursion that ended within `reacquire_dist`,
  locks Q to the wrong stretch — three tests covered exactly that and had been
  passing only because the search was global.
- The lap counter in `simple_reelout.jl` / `simple_opt_reelout.jl` drops any
  `|delta| > n_path/8`: one step moves Q by ~0.3 points, so a jump is Q changing
  branch, never flight.

Replaying the flown trajectory of the last 30 s through both versions: branch
hops (index jump > 45) fall from 6 to 1, and the samples where Q sits more than
1.5 deg worse than the true nearest point from 5 % to 2 %. The mean true
cross-track error over that window is 1.88 deg, so on a 41.5 deg path the
branches really are near-equidistant there — the residual ambiguity is the
pattern being small, not the guard. `branch_tol` was left at 3.0: inside a
window it has nothing to disambiguate, and it still governs the genuinely global
searches.

Re-flown at 17:40 the same day: the counter runs 0 -> 8 monotonically, no drop
anywhere in the run, and all 7 re-optimizations installed (shares 17/13/13/14/14/
14/15 %, measured 8495 W against 9098 W predicted).

That run then showed what the margin rule costs when it fires: at 133.7 s and
138.6 s, `u_s` steps to `max_steering` and stays there. Q jumped 123 points to
the branch the kite was physically nearest but which is traversed the OTHER way,
so `chi_set` flipped by ~156 deg (-94 -> +62), the regulated error read -139 deg
and the command saturated. The kite had been tracking its own branch happily at
3.4 deg of cross-track error while the reverse branch lay 0.4 deg away — on a
41.5 deg path near the crossing, "nearest" and "the branch I am flying" are
different questions. `branch_tol` could not rescue it: at 3.40 vs 0.41 the old
candidate missed `dmin + branch_tol` by 0.01 deg.

So the reacquire is now gated on flight direction — only points whose tangent is
within 90 deg of the filtered course qualify, once `min_speed` says the course
estimate is worth trusting. Replaying that run's last 10 s: both 123-point flips
disappear. What is left at those two instants is Q sliding forward along its own
branch to the window edge, 45 points, worth 13-15 deg of commanded course instead
of 156.

Gating only the RE-ACQUIRE was not enough — the 18:00 run still saturated `u_s`
at 134.1 s. Two more things had to be direction-aware:

- The WINDOW walks. On the path installed at 125.3 s the two branches are ~65
  points apart at the crossing, so a +/-45-point window reaches the reverse
  branch in two steps of sliding to its own edge (Q went 148 -> 193 -> 213). An
  index window cannot separate branches that are close in index; their tangents
  are what differ, by 166 deg here. So the 90 deg alignment test is now a FILTER
  on the whole Q search — windowed and global alike — with the plain nearest
  point as the fallback when nothing qualifies.
- The `branch_tol` tie-break started its search at `best = Inf`, so ANY local
  minimum within tolerance replaced Q, however badly aligned: the reverse branch
  at 0.37 deg beat the aligned Q at 3.21 deg simply by being a local minimum
  (the aligned Q, sitting at the window edge, was not one). The bar now starts at
  Q's own steering effort, so a candidate must be better ALIGNED to take it.

With all three, replaying 126-140 s of that run: no branch flip, the largest
single-step course change falls from ~170 deg to 43 deg, and what remains is Q
sliding 27-30 points forward along its own branch with the cross-track error
continuous across it (3.65 -> 3.21 deg).

## IPOPT 422 at 180 m: isolated failure pockets in tether length (2026-08-18)

`simple_opt_fig8.jl` on the 180 m project threw `optimization infeasible or not
converged (IPOPT failure)` from `/step`. The solve was not flaky: two attempts from
the menu and one replay of the identical request all converged to a bit-identical
`input_depower = 1.6585713831168203`.

The server's constraint report says what went wrong. Against a successful 180 m solve:

| | converged | failed |
| --- | --- | --- |
| height [m] | 50-76 (binding on the 50 m floor) | 122.7-142.8 |
| tension [N] | 2543-3897 | 969-1746 |
| speed_radial [m/s] | 2.01-2.55 | -0.99-1.56 |
| angle of attack [deg] | binding at 14 | 22.3, VIOLATED |
| winch-law residual | 1.9e-12 ok | 1.1e-03 INFEASIBLE |
| solver | 75 evals, 1.5 s, Optimal | 2870 evals, 81 s, Max Iterations |

The solve climbs to 122-143 m of height, where tension collapses to ~1 kN and
`v = kv*sqrt(F)` cannot be satisfied. It exits on the iteration cap while still
infeasible, so it is stuck in a bad basin rather than short of budget.

**Two hypotheses were tested and falsified.** It is not a warm start: a 150 m solve
immediately before still left 180 m failing. It is not AWETrim's `r0` bound of
`(180, 300)` being active at exactly 180 m: 185 m is interior and converges.

**What it is: isolated pockets in tether length.** At the stock 26 deg guess, 150.0,
179.0, 180.0 and 185.0 m converge on one smooth branch (6080, 6528, 6538, 6584 W)
while 180.00027 and 181.0 m throw. The failures are holes in that branch, not a
region.

`l0 = s.sys_state.l_tether[1]` (`simple_opt_fig8.jl`) is a Float32, which is how
180.00027 reached the solver at all. Rounding it would have unblocked this run, but
it is NOT the fix - 181.0 is already a round number and fails.

**The fix is the guess.** `guess_el_center` 26 -> 22 in `data/traj_opt.yaml`:
22 converges at all six lengths above and returns the same optimum wherever 26 also
worked (6080 W at 150 m, 6527 vs 6528 at 179 m, 6583 vs 6584 at 185 m). The
converged patterns sit at 14-24 deg of elevation, so 26 was seeding above the answer.
Six lengths is a small sample - a new length that throws 422 should move this number
first.

Every optimizer figure recorded elsewhere in this log was measured at 150 m or at
exactly 180.0 m, where 26 and 22 agree, so none of them shift.

## The curvature margin does not improve with tether length under reopt (2026-08-18)

`simple_opt_reelout.jl` and `traj_opt.yaml` both said the shortest tether is the worst
case for the curvature margin, so the startup check at the starting length settles it
for the whole run. That is true for ONE path flown throughout - the 150 m path measures
0.93 at its anchor and 2.18 at 350 m - and **false once `reopt_enabled` is on**.

Cancel the `L` in the kite's minimum angular turn radius `1/(L*c1*u_s)` and its minimum
PHYSICAL turn radius is `1/(c1*u_s)`, **independent of tether length**: 11.35 m at
`c1 = 0.2752`, `u_s = 0.32`. Solving fresh from the guess at three lengths:

| L | margin | path radius | kite radius | path radius [m] |
| --- | --- | --- | --- | --- |
| 182 m | 0.97 | 3.47 deg | 3.57 deg | 11.0 |
| 200 m | 1.01 | 3.28 deg | 3.25 deg | 11.5 |
| 246 m | 1.01 | 2.66 deg | 2.64 deg | 11.4 |

The angular radius nearly halves; the physical one does not move. The optimizer returns
a pattern of 11.0-11.5 m physical turn radius at every length because its own
`input_steering` (+/-0.35) and `input_steering_rate` (+/-0.29) bounds are binding in
every constraint report, and maximum power wants the tightest loops they allow. Two
near-equal constants against each other, so the margin sits at ~1.0 whatever the
length.

Consequence: a re-optimization landing at 0.85 rather than 0.98 is which optimum the
seed reached, not a trend with tether length, and with `min_feasibility_margin: 0.9`
roughly half of them are rejected by design. The rejections and the occasional
degenerate reply (margin 0.00 at 246 m) are the optimizer working at a curvature limit
it does not know about - not bugs in the gate.

The lever is the optimizer's steering bound, not the margin: raising
`min_feasibility_margin` towards 1.0 only stops anything installing. Ground clearance
behaves the same way, for the same reason - the flown minimum measured 42.8, 43.1 and
44.0 m at those three lengths instead of rising with the tether.

## The optimizer's tether is 10 mm; ours is 4 mm (2026-08-18)

**This supersedes the depower-offset explanation in the entry below.** AWETrim's
built-in LEI-V3 system config (`data/LEI-V3-KITE/system.yaml`) declares the tether as

    diameter: 0.010      # m  (10 mm)
    density: 970.0       # kg/m3  (Dyneema SK78)

against `d_tether: 4.0` mm and `rho_tether: 724.0` in this repo's
`settings_reelout_*.yaml`. That is 2.5x the diameter - so ~2.5x the tether drag area -
and 8.4x the mass per metre.

Measured by re-solving with a copy of that config edited to 4 mm / 724 kg/m3, passed
through `/init`'s `system_config_path` (the copy must sit in `data/LEI-V3-KITE/`,
since `factory.py` resolves the sibling configs relative to the system file's parent):

| tether | 150 m | 180 m |
| --- | --- | --- |
| 10 mm / 970 (default) | 6080 W | 6537 W |
| 4 mm / 724 (ours) | 7456 W | 8191 W |
| correction | +22.6 % | +25.3 % |

**Confirmed by re-flying it.** `simple_opt_reelout.jl` at 150 m against the corrected
optimizer (`output/reelout_150m_opt.yaml`, 13:15): predicted 7457 W, measured 8194 W,
**ratio 1.10x**, down from 1.45x. The run is otherwise healthy - 5.5 laps, RMS d
1.22 deg, 150 -> 377.7 m, mean force 3455 N.

That re-run carries two changes, not one, and they split cleanly:

| | old (11:20) | new (13:15) | effect |
| --- | --- | --- | --- |
| predicted | 6080 W | 7457 W | +22.6 %, the tether |
| measured | 8786 W | 8194 W | -6.7 %, depower 0.27 -> 0.275 |
| ratio | 1.445 | 1.099 | |

In log terms the tether closes 0.204 of the 0.274 that was closed (74 %) and the
depower nudge the remaining 0.070. `guess_el_center` 26 -> 22 is neutral at 150 m
(both reach the same optimum) and `reopt_enabled` went false, having installed nothing
before. So the tether is the dominant term - larger than depower, larger than the aero
model - and 1.10x is what is left for the depower-axis offset and VSM-vs-quasi-steady
together.

What survives from the entry below: the depower scales really are offset (0.4 m of
calibration convention), and the two models really do have similar depower
sensitivity. What does NOT survive: attributing the bulk of the power gap to that
offset. The 0.61 m equal-power offset quoted there was measured with the optimizer on
a 2.5x oversized tether, so it is an upper bound, not the residual.

Reproducing it needs a server-side file, which makes it awkward as a permanent
setting - `system_config_path` is a path on the SERVER, and the corrected config has
to live beside AWETrim's own sibling configs. The clean fix is upstream: AWETrim's
LEI-V3 tether should match the V3 the co-simulation flies, or this repo should ship a
documented override.

## Predicted 6080 W vs measured 8786 W — not a depower unit bug (2026-08-18)

The 150 m stage-4 run (`output/reelout_150m_opt.yaml`) harvested 8786 W where the
optimizer predicted 6080 W, a ratio of 1.45. The first suspicion was that
`traj_opt.yaml`'s `input_depower: 1.6` was being sent in the wrong unit. It is not:
AWETrim reads that field as `l_dp` in metres (bounds 1.1-2.3 m,
`src/awetrim/utils/defaults.py`), and 1.6 arrives as 1.6 m. Three checks then closed
the depower explanation entirely.

**It never reaches the prediction.** `input_depower` is in the server's default
`optimization_params`, so it is a seed. Re-running the same request returns
`optimized_parameters.input_depower = 1.457` m and reproduces 6080 W to the watt, on
the same path (azimuth -19.5..21.6°, elevation 16.1..28.7°).

**The gap is already there at the anchor length.** Binning the flown log by tether
length gives 8945 W over 150-200 m, 9061 W over 250-300 m and 8826 W over 350-381 m —
flat. So it is not the kite climbing into stronger wind as the tether grows; the
mismatch exists at 150 m, where the 6080 W was computed.

**The remaining gap is force, amplified by the winch law.** Both sides obey
`v = kv*sqrt(F)` (run: `0.0408*sqrt(3623) = 2.456` against 2.46 m/s measured; ROM:
`0.0408*sqrt(2967) = 2.22` against 2.19). So `P = kv*F^1.5`, and a 1.45x power gap is
a **1.28x tether-force gap** — V3Kite's VSM makes about 28 % more force on this path
than AWETrim's quasi-steady ROM. The optimizer's `avg_power_W` is a correct
time-average, not a node-mean artefact (recomputed from its own table: 6574 W against
its reported 6571 W; the node-mean would have been 7386 W).

**And correcting the depower would widen the gap, not close it.** The two scales
differ by a constant 0.4 m ([steering_depower.md](steering_depower.md)), so the run's
0.27 is 1.95 m on AWETrim's scale, not 1.55. Pinning `input_depower` there is
infeasible (HTTP 422); pinned at 1.6 the prediction falls to 4486 W and at 1.8 to
1873 W. The ROM's optimum sits at the powered end and its power collapses as the tape
is let out, while V3Kite flies the depowered end at 8786 W. The models disagree about
depower qualitatively, so `input_depower` stays free and the predicted-vs-measured
ratio in every summary is a comparison at two different depower settings.

**Resolved the same day by a depower sweep on the plant.** Five runs of
`simple_reelout.jl` on the 20°/11° lemniscate at 180 m (same wind and winch the ROM
was given; `FCS_OVERRIDES = Dict(:depower_setpoint => d)`, `SHOW_PLOTS`/`RUN_ARCHIVE`
off, logs to a scratch `OUTPUT_PATH`):

| `depower_setpoint` | power [W] | force [N] | RMS d [deg] | steering sat | min elev [deg] |
| --- | --- | --- | --- | --- | --- |
| 0.275 | 8318 | 3489 | 1.21 | 0.1 % | 11.3 |
| 0.290 | 6684 | 3005 | 1.27 | 0.2 % | 11.0 |
| 0.300 | 5743 | 2711 | 1.30 | 0.2 % | 10.7 |
| 0.325 | 3909 | 2090 | 1.57 | 0.5 % | 10.0 |
| 0.350 | 2654 | 1612 | 2.09 | 8.4 % | 8.9 |

Only the last row is contaminated by the steering clamp; the rest fly clean, so the
curve is aerodynamic response, not tracking loss. Fitted against the ROM's pinned
sweep, converted to `rel_depower` on AWETrim's scale:

- V3Kite `dlnP/du_p` = **-15.2**, AWETrim **-17.8** — within 15 %.
- Equal power (6538 W, the ROM's 180 m optimum) at `u_p` **0.2915** on V3Kite against
  **0.1699** on the ROM.

So the models do NOT disagree about depower qualitatively, which is what the pinned
ROM sweep alone suggested. They are offset along the depower axis by 0.12 of
`rel_depower` = **0.61 m of tape**, of which the 0.4 m calibration offset explains
about two thirds; the remaining ~0.2 m is genuine VSM-vs-quasi-steady disagreement.
The 1.28x force gap is that offset, not a separate effect.

Practical consequence: `depower_setpoint` 0.2915 on this lemniscate reproduces the
optimizer's predicted power. It is NOT installed as the default — the shared
`fc_settings_reelout.yaml` also feeds `simple_opt_reelout.jl`, whose optimized path
has a curvature ceiling of 0.282 at 180 m, so anything above that fails the startup
feasibility gate. 0.275 is the value both scripts can fly; use `FCS_OVERRIDES` to fly
higher on the lemniscate.

## `fig_8` jumped 4 -> 13 at a path swap — a unit bug (2026-08-18)

In the stage-4 runs the logged lap counter steps by ~3.6x at the instant a
re-optimized path is installed (t = 90.1 s, 4 -> 13). Not the kite: `fig8_idx_progress`
counts PATH POINTS, and a path collected from `GET /trajectory` carries the
server's `n_points` (99) while the one from `/init` carries the guess length
(360). `fig8_n = 1 + floor(fig8_idx_progress / n_path)` then divided a distance
accumulated in 360-point units by 99. Fixed by rescaling the accumulator with the
path, `fig8_idx_progress *= n_path_new / n_path`, in the same step that re-bases
`fig8_idx_prev`.

Only `SysState.fig_8`, the summary's `laps_reeled` and the `n_fig_eight` stop
criterion read that accumulator, and `n_fig_eight` is 0, so no run was steered by
it. `fig8_metrics`' own `laps` counts azimuth crossings and was never affected —
which is why the 380 m run reported a sane 7.0 laps while the log showed 17.

Then fixed properly, by pinning the FLOWN path to one resolution: a candidate is
resampled to the run's `n_path` before it is installed, so the point count never
changes and the rescale above is a no-op guard. That means upsampling 99 -> 360,
which is why the CHECKS still run at the reply's own resolution — interpolating a
coarse polyline up concentrates each vertex's turn into one short segment, and a
curvature check on an upsampled path measures the sampling rather than the curve.
The guidance is indifferent to the extra points: it walks arc length, and they sit
on the same curve.

## Stage 4 with the elevation gate, 180 -> 380 m (2026-08-18)

Same run with `min_elevation: 6.5` (down from 10), `reelout_l_max: 380` and the
candidate elevation gate in place. **All 8 criteria passed**, 9088 W over an 83.7 s
reeling window, 761.0 kJ, stopped on length at 380 m.

| | this run | reopt at 350 m | no reopt, 350 m | lemniscate, 350 m |
|:--|--:|--:|--:|--:|
| reel-out to | 380 m | 350 m | 350 m | 350 m |
| mean power | 9088 W | 9052 W | 9002 W | 8993 W |
| energy | 761.0 kJ | 648.9 kJ | 646.4 kJ | 646.0 kJ |
| RMS d | 1.59° | 1.75° | 1.31° | 1.27° |
| peak steering (time at peak) | 0.320 (8 %) | 0.320 (9 %) | 0.320 (5 %) | 0.271 (0 %) |
| tape rate-limited | 10 % | 10 % | 7 % | 0 % |
| min elevation | 7.3° | 7.0° | 11.1° | 11.4° |
| criteria | all 8 | FAILED | all 8 | all 8 |

**Only the first two columns share a tether range**, and only the last three share
one with each other. `reelout_l_max` moved with this run, so its 9088 W and 761 kJ
are not comparable with anything above them; a lemniscate run at 180 -> 380 m is
needed before the power is worth reading.

**The `distance_radial` ceiling is confirmed.** One path installed at t = 90.1 s
and L = 316 m (margin 1.22, clearance 55.1 m); the second request, at L = 380 m,
failed — exactly as the (100, 360) m bound predicts once ~35 m of in-lap reel-out
is added. Re-optimization is usable up to ~325 m of anchor length on this
configuration and not beyond, so a longer reel-out window does not buy more
re-anchoring, only more wasted solves.

**The run passes because the floor was moved to meet it.** The installed path's own
minimum is 10.0° (55.1 m of height at 316 m of tether) and the kite flew 7.3° —
2.7° of undershoot, against 3.0° measured on the previous run. The gate compares
the REFERENCE minimum with `min_elevation`, so at 6.5 it will admit a path that
flies near 4°. The undershoot is the quantity that needs the margin, and it is
reproducible enough (3.0°, 2.7°) to be budgeted rather than discovered.

**Steering is still fully spent**: 8 % of the settled window on the clamp and 10 %
rate-limited, against 0 % and 0 % for the lemniscate. Three configurations of the
optimized path in a row have cost the entire actuator margin for a power
difference inside noise.

## Stage 4 re-optimization: FAILED a criterion, and why (2026-08-18)

`simple_opt_reelout.jl` on the 180 m project with `reopt_enabled: true`. It
harvested 9052 W against 9002 W without reopt and 8993 W for the lemniscate, and
it **failed a success criterion**: `min elevation > 10.0° (whole run)`, reaching
7.0°. The power number is the least interesting thing in the run.

| | reopt on | reopt off | lemniscate |
|:--|--:|--:|--:|
| mean reel-out power | 9052 W | 9002 W | 8993 W |
| RMS d | 1.75° | 1.31° | 1.27° |
| peak steering (time at peak) | 0.320 (**9 %**) | 0.320 (5 %) | 0.271 (0 %) |
| tape rate-limited | **10 %** | 7 % | 0 % |
| min elevation, whole run | **7.0°** | 11.1° | 11.4° |
| criteria | **FAILED** | all 8 | all 8 |

**The clearance floor does not imply the elevation floor, and this is where it
bit.** One path was installed, at t = 90.6 s and L = 318 m, and it passed every
gate: curvature margin 1.23, clearance 55.3 m against the 40 m floor. At 318 m of
tether, 55 m of height is 10.0° of elevation — the gate was satisfied by a path
that sits on this repo's own elevation limit, and tracking error took the run
below it. The two floors scale opposite ways and one cannot stand in for the
other. Fixed: candidates and the pre-flight path are now both checked against
`fcs.min_elevation`, which costs nothing and is the criterion the run is scored on
anyway.

**Re-anchoring did not buy back steering margin — it spent more.** 9 % of the
window on the clamp and 10 % rate-limited, against 5 % and 7 % without reopt, and
RMS d up from 1.31° to 1.75°. The re-optimized path is anchored to a longer tether
and is correspondingly lower and tighter in the part of the window where it was
installed.

**The second request failed, and the reason is a bound, not the weather.** At
t = 140.8 s the run asked for a path at L = 350 m. AWETrim's
`DEFAULT_LIMITS["distance_radial"]` is (100, 360) m and its pattern reels out
~35 m within a lap, so a 350 m anchor needs 385 m and is infeasible by
construction. Re-optimization has a usable ceiling around 325 m of anchor length
on this configuration; asking above it wastes a solve.

**A reply collected from `/trajectory` is coarser than one from `/step`.** The
installed path had 99 points against the initial 360: the dense table is served at
the server's `n_points` (default 100), while the blocking `/step` reply echoes the
guess length. Harmless for guidance at ~1° spacing, but it is why
`traj_opt.path.points` changes mid-run, and the summary now reports the initial and
final counts separately (as it does for the pre-flight clearance, which was being
recomputed from whatever path `fec` held at the end).

## Optimized path vs lemniscate, like for like (2026-08-18)

The comparison the three runs before this one could not make: same project
(`system_reelout_180m.yaml`), same 180 -> 350 m, same `length` stop criterion,
same reeling window to the tenth of a second. Only the reference path differs.
Both passed all 8 criteria. `output/reelout_180m_opt.yaml` and
`output/reelout_180m.yaml`.

| | optimized | lemniscate |
|:--|--:|--:|
| mean reel-out power | 9002 W | 8993 W |
| energy | 646.4 kJ | 646.0 kJ |
| reeling window | 71.8 s | 71.8 s |
| mean force | 3674 N | 3672 N |
| RMS d | 1.31° | 1.27° |
| peak steering (time at peak) | **0.320 (5 %)** | 0.271 (0 %) |
| tape rate-limited | **7 %** | 0 % |
| laps | 7.0 | 6.0 |
| min elevation, whole run | 11.1° | 11.4° |

**The two paths harvest the same power: +0.1 %.** Not "about the same" — 9002
against 8993 W, and 646.4 against 646.0 kJ, which is well inside run-to-run
variation. The externally optimized path adds nothing measurable here.

**It costs the whole steering margin to fly.** The command sits ON `max_steering`
for 5 % of the settled window and the tape is rate-limited 7 % of the time,
against 0 % and 0 % for the lemniscate. Same power, no headroom left for a gust.

**Why the tie, and it is not a coincidence.** Reel-out power on this plant is
`F * v_ro` with `v_ro = kv*sqrt(F)`, so it is set almost entirely by the mean
tether force — and both paths pull 3673 N to within 2 N. The force comes from the
wind, the depower and the tether length, none of which the path shape moves much.
The shape sweep of 2026-08-15 already said this from the other direction: the
whole f8_a 19..30 x f8_b 8..12 grid spanned +4.8 % of mean power, and width was
the only axis with any effect. A path optimizer working on shape alone is
optimizing a weak lever on this system.

That is a statement about THIS configuration, not about the optimizer: fixed
depower, one winch law, one wind. The levers it holds that the lemniscate does not
— the depower profile it optimizes per node, and the radial schedule this repo
currently discards — are untested. Stage 4, and applying `input_depower`, are
where a difference would have to come from.

## Optimized path from 180 m, stopping on length (2026-08-18)

`simple_opt_reelout.jl` on the new `system_reelout_180m.yaml`, with
`n_fig_eight: 0` so reel-out stops on length like the lemniscate baseline.
All 8 criteria passed. `output/reelout_180m_opt.yaml`.

| | optimized, 180 m | optimized, 150 m | lemniscate, 150 m |
|:--|--:|--:|--:|
| mean reel-out power | 9002 W | 8742 W | 9058 W |
| energy | 646.4 kJ | 594.4 kJ | 759.0 kJ |
| reeled | 180 -> 350 m | 150 -> 308.9 m | 150 -> 350 m |
| stop criterion | length | laps | length |
| predicted / ratio | 6538 W / 1.38 | 6080 W / 1.44 | — |
| RMS d | 1.31° | 1.22° | 1.24° |
| peak steering (time at peak) | 0.320 (5 %) | 0.320 (2 %) | 0.291 (0 %) |
| tape rate-limited | 7 % | 5 % | 1 % |
| curvature margin at the start | 0.96 | 0.94 | — |
| flown clearance | 42.0 m | 43.0 m | — |

**The optimized path still does not beat the lemniscate.** 9002 W against 9058 W
is a tie, and the tie is on the optimized path's own terms: starting at 180 m
skips the lowest, least productive 30 m of tether that the lemniscate run had to
fly through, so if anything the comparison now flatters it. Three runs in, the
optimizer's path has not produced more power than the hand-tuned eight in this
simulation.

**It costs steering margin to fly.** The command sits ON `max_steering` for 5 % of
the settled window and the KCU tape is rate-limited 7 % of the time, against 0 %
and 1 % for the lemniscate — worse than the 150 m run's 2 %/5 %, even though the
curvature margin improved from 0.94 to 0.96. Tracking still holds (RMS d 1.31°),
but there is no authority left for a gust, which is exactly the side condition
`optimize_fig8.jl` rejects a shape for.

**The clearance choice is visible in the log.** Flown minimum 42.0 m, against a
path minimum of 42.6 m at the anchor (tracking error takes the kite slightly below
its own reference) and AWETrim's own 50 m floor. The kite really does fly ~8 m
below the floor the path was designed under; that is the known cost of installing
the angles and discarding the radial profile, and `min_height: 40.0` is the
deliberate acknowledgement of it, not an oversight.

**What is still missing for a fair comparison**: a LEMNISCATE run from 180 m.
Every optimized-vs-lemniscate number above spans different tether ranges. One run
of `simple_reelout.jl` on `system_reelout_180m.yaml` closes that gap.

## Height floor: no starting length clears AWETrim's 50 m (2026-08-18)

The optimizer constrains HEIGHT, `DEFAULT_LIMITS["height"] = (50, 400)` m applied
to `pattern.z(distance_radial, s)`; its elevation bound is `(0, 160°)`, i.e. none.
The bound is active — the returned paths sit ON it to the centimetre. But it is
reached at the END of a lap's reel-out, and this repo installs the reply as a
curve in (azimuth, elevation) with the radial profile discarded, so the clearance
we actually get at the anchor radius is `50 * r0/r_low`:

| anchor r0 | r over one lap | lowest elevation | z at r_low | z at the anchor |
|--:|--:|--:|--:|--:|
| 150 m | 150.0 -> 184.3 m | 16.11° | 50.00 m | 41.6 m |
| 180 m | 180.0 -> 215.1 m | 13.69° | 50.00 m | 42.6 m |

A lap reels out ~35 m whatever the anchor is, so `r0/r_low` barely moves and the
anchor-radius clearance is ~42 m at any starting length. **Moving the start length
cannot fix this** — 180 m was tried for exactly that reason and did not.

What the run actually does is climb through 50 m within the first lap, because it
is reeling out: the flown minimum of the 150 m run was 43.0 m, at 152.9 m of
tether, not the 29.4 m of the constant-length run. So `min_height` in
`data/traj_opt.yaml` is 40.0, describing the anchor radius, and every summary
carries `traj_opt.clearance.flown_min_m` as the number that matters. The real fix
is to stop discarding `distance_radial`/`speed_radial` (stage 4).

Two other things the 180 m anchor moved, worth having on record: the curvature
margin improves to 0.96 from 0.94 (still below 1), and the predicted power rises
to 6538 W from 6080 W.

## Externally optimized path — first REEL-OUT (2026-08-18)

First run of `examples/simple_opt_reelout.jl`, `system_reelout_150m.yaml`, against
the lemniscate run of the same project from 05:12. Both reeled out; all 8 criteria
passed in both. Summaries: `output/reelout_150m_opt.yaml` and
`output/reelout_150m.yaml`.

| | optimized | lemniscate |
|:--|--:|--:|
| mean reel-out power | 8742 W | 9058 W |
| energy over the window | 594.4 kJ | 759.0 kJ |
| reeling window | 68.0 s | 83.8 s |
| tether at stop | 308.9 m | 350.0 m |
| stop criterion | laps | length |
| RMS d | 1.22° | 1.24° |
| mean force (reeling) | 3624 N | 3692 N |
| peak steering | **0.320 (clamped, 2 %)** | 0.291 (0 %) |
| tape rate-limited | 5 % | 1 % |
| min elevation | 13.7° | 10.5° |

**The optimizer's 6080 W and the run's 8742 W are not the same quantity**, and
`GET /trajectory` says by exactly how much. Its 6080 W is one cycle of 16.08 s
over `distance_radial` 150.0 -> 184.3 m: the optimizer reels out WITHIN the
pattern and prices that one lap. The run reeled 150 -> 308.9 m over 68 s through
an EXPLOG profile that strengthens with height, so its mean is taken over a window
that spends most of its time on a much longer tether in much more wind. The 1.44x
is the difference between those two windows, not a model error. Nothing here
validates or refutes the optimizer; the comparison that would is stage 4, where
the path is re-optimized for the length being flown.

**The optimizer constrains HEIGHT, not elevation, and the constraint is active.**
`DEFAULT_LIMITS["height"] = (50, 400)` m in AWETrim's `utils/defaults.py`, applied
to `pattern.z(distance_radial, s)` in `timeseries/phase_parametrized.py`. Its
elevation bound is `(0, 160°)`, i.e. no floor at all. The returned path's minimum
height is 50.00 m to the centimetre — it sits ON the bound. The 16.11° minimum
elevation that comes with it is a CONSEQUENCE, reached at r = 180.3 m rather than
at the 150 m anchor, so it cannot be read as an elevation the optimizer chose.
Two things follow. This repo's `min_elevation = 10°` criterion and the
optimizer's floor are different quantities that scale opposite ways: at 150 m the
50 m floor forces elevation up, while at 350 m it permits 8.2°, below our own
criterion.

**The two steering limits are not comparable as written.** AWETrim's
`input_steering = ±0.35` is a steering-line differential in METRES — it enters
the model as `roll = input_steering * k_steering` with `k_steering` fitted from
the CS polynomial, and `steering_delta_limit` bounds the same quantity as "the
maximum |delta| [m] of the rigid-segment bridle-steering triangle". This repo's
`max_steering = 0.32` is the dimensionless `u_s` the KCU takes. Under V3Kite's
calibration (`L_left = 1.6 - 1.4*u_s`, i.e. 1.4 m of tape per unit `u_s` per
side) ±0.35 m is roughly `u_s` 0.25, which would make the optimizer's bound
TIGHTER than the controller's rather than looser — but AWETrim's bridle-triangle
geometry is not V3Kite's tape calibration, so treat that as an estimate until the
conversion is confirmed. What remains solid about the 0.94 curvature margin is
the other half: the optimizer uses its own steering-to-roll model, not the V3's
identified turn-rate law `1/(L*c1*u_s)`, so nothing in the solve knows the radius
this kite can actually fly.

**Against the lemniscate, the optimized path did not win** — 8742 W against
9058 W, and 594 kJ against 759 kJ. That comparison is confounded too, and in a way
worth fixing before reading anything into it: the two runs stopped for DIFFERENT
reasons. `n_fig_eight = 4` fires on lap count, and the optimized pattern flies
faster laps, so it stopped 41 m short of `reelout_l_max` while the lemniscate ran
to the full 350 m. The late, long-tether part of the window is the high-power
part, so the lemniscate's mean is flattered by the extra 15 s and its energy by
the extra 41 m. Set `n_fig_eight: 0` on both runs to compare them on the same
distance.

**The early laps are at the curvature limit, and now it shows in the actuator.**
`margin_start` is 0.94 at 150 m against 2.18 at 350 m. The steering command
touches `max_steering` exactly (0.320, 2 % of the settled window within 2 % of
peak) and the KCU tape is rate-limited 5 % of the time, against 0 % and 1 % for
the lemniscate. Tracking is nevertheless as good as the lemniscate's — RMS d 1.22°
against 1.24° — because the margin improves as the tether grows, unlike the
constant-length run of the entry above, where the same path never left margin 0.93
and RMS d was 2.55°. Same path, same controller: the difference is the tether.

## Externally optimized path — first flight (2026-08-18)

First run of `examples/simple_opt_fig8.jl`: the AWETrim optimizer's power-optimal
reel-out path, flown at CONSTANT 150 m under `system_reelout_150m.yaml` (6 m/s at
6 m, EXPLOG, `kv = 0.0408`, `f_high = 7600`). It flies. Against the lemniscate
baseline of the same project (`output/reelout_150m.yaml`, 05:12), which is a
REEL-OUT run 150 -> 350 m and therefore not a like-for-like comparison:

| | optimized, L = 150 m fixed | lemniscate, 150 -> 350 m |
|:--|--:|--:|
| RMS d | 2.55° | 1.24° |
| max d | 5.02° | 4.97° |
| laps | 13 in 119.6 s (9.2 s/lap) | 6.5 |
| mean tether force | 9008 N | 3735 N |
| mean `v_app` (phase 4) | 33.8 m/s | 22.0 m/s |
| min elevation | 11.3° | 10.5° |
| peak steering | 0.263 (1 % near peak) | 0.291 (0 %) |

Three things to take from it.

**Tracking is curvature-limited, not steering-limited or tuning-limited.** The
steering command peaks at 0.263 of the `max_steering` 0.32 and sits within 2 % of
its own peak only 1 % of the time, so there is authority left; RMS d is
nevertheless twice the lemniscate's. The optimized path comes back at
`check_pattern_feasible` **margin 0.93** at 150 m (tightest path radius 3.9°
against the kite's 4.2°) — it is slightly tighter than the V3 can turn, and the
tracking error is the geometry saying so. The same path measures 2.18 at 350 m,
because the minimum ANGULAR turn radius is `1/(L*c1*u_s)` and a longer tether
turns tighter on the sphere. `data/traj_opt.yaml` therefore ships
`min_feasibility_margin: 0.9`; raise it to 1.0 once the optimizer can be given a
curvature constraint.

**The 9 kN is the test rig, not the path.** `max_force` is 8000 N nominal and
`f_high` 7600 N, so a mean of 9008 N would pin the UpperForceController for the
whole window — the very side condition `optimize_fig8.jl` rejects a shape for.
But the optimizer sized this path for a winch reeling out at `kv*sqrt(F)`, i.e.
~3.9 m/s at that force, and constant length removes exactly that relief. The
number is expected here and is the argument for stage 3, not evidence against the
path.

**The pattern is low, fast and asymmetric.** Centre elevation 22.4°, flown extent
azimuth -23.9°…+24.1° and elevation 11.3°…28.7°, laps every 9.2 s against the
lemniscate's ~18 s. The returned curve is asymmetric (-19.5°…+21.6° as
optimized), which is what makes the azimuth-handedness question in
`PlanOptFig8.md` both testable and worth settling before stage 3.

The optimizer predicted 6080 W of mean reel-out power for this path. That number
has not been measured yet — it cannot be, at constant length; the lemniscate
baseline harvested 9058 W while reeling 150 -> 350 m, which is a different
question. Stage 3 is what compares them.

## ENTRY_CHI_MAX (entry descent limiter)

Why the limiter exists (2026-07-26, first run): without it the kite dove
straight from the 73° park to the pattern, converting 40° of potential energy
into a 3.3x overspeed — `v_app` 15.7 -> 51 m/s in 7 s, AoA driven negative as
the wing unloaded, and the solver aborted at `t = 7.35 s`. The guidance was
working (cross-track error 35° -> 1.7°); the descent was simply flown far too
steeply.

`105 -> 95` (2026-07-26): at 105° the descent from the 73° park still reached
45.6 m/s by elevation 40° (AoA -21.9°). 95° is only 5° below the local
horizontal, so the kite spirals down slowly enough for drag to bleed the energy
it gains.

An earlier version of the limiter commanded the path tangent instead of the
clamped homing course — pure feed-forward with no reference to the path's
POSITION — and the kite flew off to azimuth -65° and sat in a limit cycle for
180 s with the cross-track error frozen at 24°. Latching the sign arbitrarily
and then reversing mid-descent broke two further runs.

## ENTRY_D_BLEND — the limiter handover was a step, not a switch (2026-08-02)

The limiter's `d`-gate was a hard `if`, so releasing it stepped the commanded
course by the whole clamp violation in ONE timestep. Measured at `t = 33.30 s`
of the 2026-08-02 run (`fig8_run`):

| t [s]   | d [deg] | chi_set  | chi_cmd  | err     | set_steering |
|--------:|--------:|---------:|---------:|--------:|-------------:|
| 33.2833 |  12.06  | +124.42° |  +95.00° | +65.9°  | -0.3200      |
| 33.3083 |  11.995 | +124.46° | +124.46° | +35.8°  | +0.2100      |

`d` crossed `ENTRY_D_GATE = 12.0`, `chi_cmd` jumped +95 -> +124.5°, and the
regulated error stepped -29.7° (-0.517 rad). The PID's discrete derivative gain
is `K*N*Td/(Td + N*dt) = 0.78*2*0.15/0.16667 = 1.40` there, so that step alone
contributed **+0.73** against a P term of -0.49: the command left the -0.32
clamp and REVERSED sign to +0.28 for ~0.075 s (the `N/Td` filter time constant)
before returning to the clamp.

Nothing in the plant moved — `d`, course, heading and `v_app` are all smooth
through 33.3 s. It was a discontinuity in the REFERENCE, entirely self-inflicted
by the gate.

Fix: blend the limited and raw courses linearly over `ENTRY_D_BLEND = 4°` above
the gate, on the WRAPPED difference (`chi_set + w*wrap_to_pi(chi_lim -
chi_set)`, the same form as the heading/course feedback blend) so it stays
continuous across the ±180° cut the limiter exists to tame. Replayed over the
logged `chi_set`/`d` of that run, the largest phase-3 command step falls
**29.46° -> 0.68°** (43x), and the 0.68° remainder is at `t = 36.28 s` where the
limiter is already off (`w = 0`) — i.e. it is jitter in the raw guidance course
itself, pre-existing and untouched by this change.

Sizing: `d` closes at ~3.4 deg/s here, so 4° is ~1.2 s of traversal — 16x slower
than the derivative filter, so the ramp is tracked rather than differentiated
(D contribution ~0.05 instead of +0.73). It also makes gate CHATTER harmless:
`d` is not monotonic, and a hard switch re-fires the full step on every
recrossing. `ENTRY_D_BLEND = 0` restores the old switch.

### Still open: the hold -> fig8 handover reverses the command by 175°

Found while measuring the above, NOT fixed. At `t = 30.55 s` phase 2 -> 3 hands
over from the open-loop `CHI_HOLD = -90°` to the limiter's `+95°`: a 175°
reversal of the commanded course in one timestep, stepping `err` from -11.2° to
+163.8°. No spike is visible because the tape is already saturated at -0.32 on
both sides, so the loop just stays pinned at the clamp — but the loop is being
handed a reference it cannot act on, and the entry is turn-rate limited anyway.
This is the entry state machine, a different mechanism from the `d`-gate, and
worth its own look: the reference controller hands over at the pattern's
rightmost point already moving into the first turn, which should not require a
course reversal at all.

## ELEVATION — the settling-elevation cache bug

`ELEVATION` currently has NO EFFECT on where the run starts (2026-07-26). It was
added to settle the kite on the pattern instead of at the 73° park, but the
settled-geometry cache key encodes depower, steering, tip/TE, wind, tether
length, gravity, system and body damping — NOT the settling elevation.
`settle_wing` therefore reuses the existing 73° geometry and the run still
starts there (verified: logged elevation at `t = 0` is 73.0°, not 33.0°).
Forcing `remake = true` would overwrite a cache file shared with
`simple_sinus.jl` / `simple_parking.jl` with geometry they do not expect. Fixing
it properly means adding the elevation to the cache key in `stabilization.jl` —
see `PlanFig8.md`, Findings 4.

## Sign and frame conventions — the measurements behind the code

Moved out of `src/figure_eight_controller.jl` and `examples/simple_fig8.jl` on
2026-08-03, when the comment rules were applied. These are the measurements the
sign choices in those files rest on; originally established in V3Kite.jl's
`PlanFig8.md` STEP 0 against `data/tmp_steering.arrow` and `data/tmp_sinus.arrow`
(`system_reelout.yaml`, v_wind 9.51 m/s).

**`rel_steering` is fed unnegated.** Positive `rel_steering` produces a positive
heading rate: correlation **+0.998** between tape position `sl.steering` and the
frame-transport-corrected turn rate. This matches V3Kite's
`examples/simple_auto_parking.jl`. The branch for the other kite negates; that
does not transfer.

**The guidance course matches `SysState.heading`** — same zero, same sign,
circular-mean offset **+13.3°** with a 7.6° spread over the samples where the
kite actually flies (>2°/s). No `neg_azimuth` and no π correction is needed
anywhere in `figure_eight_controller.jl`. The +13° is the kite's real
course-minus-heading drift angle: the guidance commands a *course* while the
inner loop regulates *heading*, so a small steady-state cross-track bias is
expected, and the heading PID's integral term absorbs it.

**`SysState.course` DOES need +π**, unlike the heading. It is
SymbolicAWEModels' raw tangent-frame course, whose zero points away from zenith,
while `SysState.heading` and the guidance both use 0 = towards zenith. The flip
is not symmetric between the two fields — the V3's body x-axis is reversed
w.r.t. the sensor convention, which cancels the frame flip for the heading but
cannot for a velocity direction. Measured over the flying samples (>2°/s): with
the correction, `course - heading` is the +13..15° drift angle above; without
it, **-165°**. Feeding the raw field back is positive feedback and diverged the
run at t = 19.7 s.

## `fig8_metrics` — why the lap counter and the extent checks look like this

Moved out of `src/fig8_metrics.jl` on 2026-08-03. Each guard below exists
because a degenerate run scored well without it.

**Laps need an excursion band, not bare sign changes.** A lemniscate crosses its
centre azimuth twice per lap, but a kite stuck in a limit cycle jitters across
the centre and reports dozens of phantom laps — a run that never reached the
pattern at all scored **42.5**. The azimuth must reach `lap_frac` of the
pattern's half-width on alternating sides before a crossing counts.

**The band needs an absolute floor as well as a relative one.** Scaling it by
the kite's own azimuth range normalises away exactly the thing being tested: a
±1.4° limit cycle still crossed a purely relative band and scored 42.5 laps.
Hence `min_excursion`. When the COMMANDED half-width is known it is used instead
of the flown range, so the band is a property of the pattern asked for and a
kite flying an eight far smaller than commanded fails to cross it.

**Crossings are counted against the PATTERN's centre azimuth**, not the flown
mean. Using the mean lets a kite orbiting a circle off to one side score laps it
never flew: a run that circled at azimuth 29-45°, never crossing the pattern
centre at 0°, reported **14 laps**.

**Reach is recorded per excursion.** `maximum(az)` alone is satisfied by ONE
good lobe at the start of an otherwise degenerate run. Only completed
excursions are kept — the last one is cut off by the end of the window and would
drag the mean down for no flight reason.

**Both azimuth sides are checked separately**, because a single "span" check
passes on a kite that reaches +2A on one lobe and never crosses to the other.

**`tape_rate_frac` is the honest companion to `steering_sat_frac`.** The command
and what the KCU tape reached are nothing like each other on the V3. Measured on
the 2026-08-02 run: the command sat on its ±0.300 clamp **88 %** of the time
while the tape reached it in **0 %** of samples — it was slewing at its rate
limit for **67 %** of them instead. Reading `sat_frac` alone says "out of
steering authority" when the truth is "out of steering RATE", and the two call
for opposite fixes: more amplitude makes a rate-limited actuator worse, because
a larger command means longer slewing and more phase lag (measured 62.7° at the
loop's own 0.181 Hz, against 24.7° for the identified small-signal dead time).

## Settings whose rationale lived only in `FC_Settings`

Moved out of `src/fc_settings.jl` on 2026-08-03 when the comment rules were
applied. Everything here was previously a field docstring; the sections above
already cover `sim_time`, `dt`, `depower_setpoint`, `tether_length`,
`warmup_time`, the entry state machine, the pattern size, `el_center`,
`attractor_dist`, `up_loops`, `heading_p`, `heading_d_n`, the `v_kite_*` band,
`max_steering`, `entry_chi_max`, `entry_d_blend` and `elevation`.

### COMPLIANCE and the force-mode winch

In force mode the drum holds a low-passed reference force, and a load above that
reference stretches the length trim like a spring: the steady-state yield is
`dF / winch_len_kp`, so the compliance IS `1/winch_len_kp` [m/N]. `compliance`
divides both `winch_len_kp` and `winch_damp` by itself, so the yield scales
linearly while their ratio — the length loop's own time constant — is left
alone. The result is a softer or stiffer winch of the SAME character, not a
different one. `winch_force_tau` is untouched: it sets WHICH frequencies the drum
yields to, not by how much, and should sit above the lap rate so the kite can
trade line length for speed within a lap.

**At exactly 0 the controller changes**, because an infinitely stiff spring is
not representable: the run switches to POSITION mode (`set_length = l0`) and
hands the winch to V3Kite's `winch_position_torque!`, whose force feed-forward
cancels the measured load exactly — the drum sees nothing to accelerate it and
travels 0.009 m over a 30 s run. The limit is continuous in BEHAVIOUR
(`compliance` 0.01 already means `winch_len_kp` = 10 kN/m) but not in code path,
so a sweep should not expect the two sides to agree to the millimetre.

**`winch_damp` is not optional.** Without it force mode has no velocity feedback
at all — the drum is a free mass on the `winch_len_kp` spring and its speed runs
away: measured 3.5 m/s of reel-out, `v_app` 49.6 m/s, run lost at t = 19.8 s. It
is what makes the winch compliant rather than free-wheeling. That runaway also
bounds the useful range of `compliance` above 1.0 (2.0 = twice as soft).

### PARK_TIME

Hold zero steering so the transients left by init/settling decay before the
controller starts demanding maneuvers. Without it the guidance engaged at t = 0
and drove the steering straight to its clamp while the model was still relaxing.
The guidance still runs during the park — its course estimate is low-passed and
needs warming up — but its output is not applied.

### ENTRY_GAIN and ENTRY_DEPOWER

The entry is turn-rate limited: the steering command sits on its clamp for the
whole dive. Detuning the loop there therefore costs nothing in tracking and takes
the command off the clamp, which is the only way to shape the descent from the
loop side. `entry_depower` is the second lever: a higher depower than the
pattern's lowers `c1` (less turn authority, which a clamp-limited entry does not
need) and unloads the wing, bleeding some of the energy the dive from the 73°
park converts out of height. The park is excluded on purpose — `init` settles at
`depower_setpoint`, and changing the tape during the park would inject exactly
the transient the park exists to let decay. Both transitions are rate limited by
the KCU tape speed inside `step!`, so each change is a ramp, not a step.

### FIG8_PURE_COURSE — schedule on kite speed, or gate on phase

The feedback blend is scheduled on `|vel_kite|`, NOT on `v_app`. A parked V3
already sees `v_app` ≈ the ambient wind, so apparent wind speed cannot tell
"flying" from "hanging still":

| signal | park | flying (t >= 15 s) |
|:---|:---|:---|
| `v_app` | 9.1 m/s | 21.1 m/s, never below 10 |
| `\|vel_kite\|` | 4.2 m/s | 15.5 m/s (8.3 .. 22.3) |

A 10 m/s threshold on `v_app` puts the PARK at blend weight 0.82 — nearly full
course feedback on a kite that is barely moving, the one case the rule exists to
prevent. On `|vel_kite|` the park is unambiguously heading, and the weight
modulates in flight when the kite slows through a turn, which is when the course
estimate is worst.

`fig8_pure_course` then bypasses that schedule in phase 3 (SmallPlan.md's "gate
on phase instead of speed"). Path following is a course problem, and on the
pattern the kite is fast enough that the schedule asks for course anyway — it
only dips into the band during the slow part of a turn, swapping the feedback
signal mid-manoeuvre for no benefit (`|v_kite|` crossed the 10 m/s edge twice in
the 15-27 s window of the 2026-08-02 run, 9.8 % of it inside the band). The
entry phases keep the schedule: during park and dive the kite really can be too
slow for a meaningful course.

### V_APP_REF — an anchor, not a gain

Only the product `heading_p * v_app_ref` is physical. `v_app_ref` serves two
roles, and they agree only because it is the speed really flown: it anchors the
1/v_app gain schedule (so `heading_p` reads as the gain the kite actually flies
at) and it sets the kinematics for the attractor lead. The 30 -> 27 correction
was paired with the inverse scaling of `heading_p`: same flight, the anchor is
now the measured phase-3 average. Before that it was 13.1, a parking speed
carried over from `simple_auto_parking.jl` that the kite never flies here; that
anchor also overstated the attractor lead.

### V_APP_ABORT

Stop the run above this apparent wind speed. The first run's failure showed up as
an opaque solver `dt_epsilon` abort; catching the overspeed that causes it
reports the actual problem instead.

### MIN_SPAN_FRAC — the pattern-SIZE criterion

Every other criterion is measured against the CLOSEST POINT of the path, so all
of them pass on a kite flying a small eight, or one lobe in half the wind
window — it is on the path, it just is not going anywhere on it. Sized against
the flown spans on record: the reference run holds azimuth -43.5..+42.2° against
A = 40 (fill 1.06/1.09) and an elevation span of 19.9° against B = 15 (1.33), so
0.7 has real margin against a good run while a degenerate one lands far below it.

### BODY_DAMPING

A FLIGHT parameter, not just a solver setting: it sets `c1` and hence the
achievable turn radius. `init`'s default `[0, 0, 40]` is the most agile and the
only one that flies this pattern inside the identified steering range. Raising it
costs turn authority; it buys a smaller parked AoA ripple and ~3.4x fewer solver
steps.

### VSM_INTERVAL

Steps between VSM aero updates; the load is held frozen inside the DAE in
between. `1` is the TIGHTEST coupling available, so this can only be raised —
trading aero lag for wall time. Exposed to sweep the coupling mode described
under DT, not as a lever that can stabilize it.

## Turn-rate table — per-row identification notes

Moved out of `data/turn_rate_coeffs.yaml` on 2026-08-03 when the comment rules
were applied; each row there now carries a one-line pointer here. All three are
2026-07-27 re-runs at 200 m tether. The grid's usual elevation floor is 50°.

**`[0,0,40]` / depower 0.55.** Re-run at 200 m with `MIN_ELEVATION` relaxed to
40° for THIS run only (same fix as `[10,10,40]`/0.55 — the previous 200 m
attempt dove at `u_s_max` = 0.05, the first amplitude step, so a lower
`max_steering_cap` could not have helped). PASS on identification, completion
(`sweep_done`, reached the full 0.175 cap) and elevation (min 46.83° >= 40°,
though below the usual 50° floor). `c1` = 0.1073 lands almost exactly on the
legacy 150 m value (0.1071) and `delay` matches it exactly at 0.55 — a clean,
low tether-length-sensitivity result, unlike depower 0.25's delay discrepancy.

**`[10,10,40]` / depower 0.25.** Re-run at `max_steering` 0.175, matching the
legacy cap; the earlier low-elevation attempt had climbed to `u_s_max` = 0.325
before hitting the floor. PASS on identification (G scatter 7.96 %), elevation
and completion. Same cap-matching fix that resolved `[0,0,40]`/0.25's
divergence.

**`[10,10,40]` / depower 0.55.** Re-run with `MIN_ELEVATION` relaxed to 40°: the
settled trim at this damping/depower sits close enough to 50° that the sweep
tripped the floor at its very first amplitude step (`u_s_max` was 0.05 — not a
"climbed too far" case, so lowering `max_steering_cap` could not have helped,
since the dive happens before the cap is ever reached). At 40° the full sweep
completes, min elevation 44.01° — above 40 but below the grid's normal floor, so
this row's `outcome: time_limit` is not on quite the same footing as the rest of
the grid. `c1` and G-scatter are otherwise solid (0.10 %, 15.4 %).

**`[0,0,40]` / depower 0.23 — 2026-08-10.** Identified because `fc_settings.yaml`
moved to depower 0.23, below the grid's 0.25 floor, and the table refuses to
extrapolate. The FIRST row identified with `min_damping` passed explicitly:
`steering_test_v3.jl` now sets it to `[0,0,40]`, equal to `body_damping`, so the
sweep flew that damping throughout instead of whatever the settling decay left
in place. The rows above were identified before the floor was introduced
(2026-08-08). Run at 150 m / dt 0.05, which the table no longer penalises —
neither enters `c1`, whose fit is normalised by apparent wind speed.

PASS on all three criteria: `sweep_done` at the full 0.175 cap, min elevation
73.05°, G scatter 14.49 %. `c1` = 0.3225 ± 0.21 %, and it sits just above the
0.25 row's 0.3104 — monotone in depower, as it must be, and close enough that
the damping-regime change did not move `c1` much.

`c2` is the caveat: -1.1124 ± 12.5 %, against +0.0758 at depower 0.25 and a
smooth +0.0758 -> +0.5925 trend across 0.25..0.55. `c2` is the gravity term and
the one that moves most with damping (it swings from -0.38 to -2.08 across the
damping levels at fixed depower), so the sign flip is plausibly the damping
regime rather than depower — but it means the 0.23 and 0.25 rows should not be
read as one continuous `c2` curve. Interpolating `c2` between them is
meaningless until 0.25 is re-identified the same way. `c1` and `delay`
(0.25 s vs 0.2667 s) are consistent across the pair.

## REEL_OUT winch (`examples/simple_reelout.jl`)

### Lagging the square-root law is CLOSED — it costs the force regulation (2026-08-14)

`v_set = kv*sqrt(F)` on the INSTANTANEOUS measured force looks like a loop worth
damping and is not. At 6 m/s wind, 150 m, the engagement of reel-out rings: `v_ro`
overshoots to 4.6 m/s against a 3.0 m/s steady setpoint and oscillates at 2.04 s
for about ten seconds, decaying (zeta = 0.12). The chain is visible in the log —
force peaks, `v_set` (`var_11`) peaks 0.24 s later, `v_reelout` 0.39 s after that
— and the loop gain `kv/(2*sqrt(F)) * dF/dv_ro` measured 0.77 (`dF/dv_ro` = 1400
N per m/s from the oscillation amplitudes). No saturation involved: peak torque
159 N·m against the 500 N·m correction limit, WinchController in speed mode
(`var_12` = 1) throughout.

A first-order low-pass with tau = 3 s on the speed setpoint (state 1 only, so the
force limiters kept the raw signal) killed the ring and the run with it:

| | raw law | tau = 3 s |
|:--|--:|--:|
| mean power over the reel-out | 7.2 kW | **5.0 kW** |
| tether force | ~2500 N, near constant | mean 3330 N, sigma 2337, 411 .. 8137 N |
| corr(F, v_ro) | — | **-0.773** |
| `mean(F)*mean(v)` vs `mean(F*v)` | — | 8.7 kW vs 5.0 kW, i.e. -3.65 kW of covariance |

The fast force feedback IS the force regulation. Reeling out harder exactly when
the kite pulls harder is what holds the force flat; with a 3 s lag the speed
arrives a quarter period late, so the drum reels SLOWLY into the force peaks
(8137 N) and fast into the troughs (411 N). That anti-correlation is the whole
2.2 kW: `mean(F*v)` loses 3.65 kW of covariance while the means barely move.

So the ring is a startup transient of a loop that earns its keep in steady state.
Shape the ENGAGEMENT instead — `reelout_t_startup` (2.0 s here, against
WinchControllers' 0.25 s default). Do not slow the law itself — and note that the
other candidate, the `acceleration_limit` `simple_reelout.jl` hands `step!`
(`rcs.max_acc` = 8 m/s²), was tried at the plant's own 4 m/s² and made the ring
WORSE, not better; see Plan.md for the measurement.

### `depower_final` sweep — force jumps once the length freezes (2026-08-16)

Phase 5 ("final", `sys_state == 5`) begins the moment `l_set` reaches
`reelout_l_max` and keeps flying the pattern at `depower_final` instead of
`depower_setpoint`. The premise — that a frozen length needs MORE depower than
reel-out did to hold the same mean force — is confirmed, and by a much larger
margin than expected: at 150 -> 350 m, 6 m/s wind, `system_reelout_150m.yaml`,
`sim_time` extended to 190 s so phase 5 gets a ~74 s window (t = 115.7..190 s):

| `depower_final` | phase-5 mean force [N] | std [N] |
|--:|--:|--:|
| 0.30 | 5149 | 285 |
| 0.328 | **3456** | 190 |
| 0.333 | 3232 | 185 |
| 0.35 | 2578 | 201 |

Target was 3458 N, `reelout_power`'s mean force over the reel-out window at
`depower_setpoint` = 0.27 (same run, unaffected by `depower_final`). The
relationship is clearly nonlinear over this range (a quadratic fit through the
0.30/0.333/0.35 points, solved for 3458 N, landed almost exactly on the 0.328
that was then confirmed directly) — do not linearly interpolate between two
points far apart; fit at least three. The frozen-length force at
`depower_final = depower_setpoint` (0.27, untested directly) would be well
above the reel-out mean, consistent with the POSITION loop needing more torque
against a static `l_set` than a growing one — see `depower_final`'s docstring
in `src/fc_settings.jl` for the mechanism. This value is specific to 150 -> 350 m
at 6 m/s; a different `reelout_l_max`, wind speed or `depower_setpoint` changes
the target force and needs its own sweep.

### `reelout_softstop` — a hard reel-out stop leaves a reel-in power dip (2026-08-16)

A hard cut of `v_set` to 0 the instant `l_set` clamps to `reelout_l_max` (the
mirror image of the engagement transient `reelout_softstart` fixes) leaves the
drum with the old command's momentum: it overshoots `reelout_l_max` by ~0.36 m
before the POSITION loop reels it back in, a transient dip to about -3 kW at
the reel-out/final boundary (150 -> 350 m, 6 m/s).

**Loosening `step!`'s `acceleration_limit` for a few seconds after entering
phase 5 (final), instead of shaping `v_set`, is CLOSED — it makes the dip
WORSE, not better.** Tried at `reelout_stop_acc = 1.0` m/s² for
`reelout_stop_time = 5.0` s (against the usual `rcs.max_acc` = 8 m/s²): the
drum coasts much further past `reelout_l_max` before turning around —
overshoot grew from 0.36 m to **3.5 m** — and the correction the position loop
then applies scales with that error, so the dip grew from -3 kW to **-6.7 kW**.
A looser rate limiter cannot introduce a discontinuity of its own, but it also
does nothing to stop the SETPOINT itself from freezing abruptly, so all it
does here is let more excess distance accumulate before the mismatch is
corrected.

The fix that worked: decelerate `v_set` itself, but CONTINUOUSLY — matching
whatever speed reel-out is actually flying at the moment braking starts, not 0.
A first attempt using a rest-to-rest smoothstep (`3f^2-2f^3`, `f = (t-t_0)/T`)
got this wrong at the OTHER end: its velocity is 0 at `f=0` by construction, so
latching it mid-flight (at ~2.4 m/s) still stepped the command to 0 for one
sample — small (a single-sample gap, `diff(l_set) <= 0` once), but visible in
the log as a rougher run overall (whole-run force CV jumped to 41 % against the
usual 5-8 %), plausibly a chaotic-ish knock-on since the pattern is a repeating
closed loop.

`reelout_softstop`'s actual mechanism: latch once the remaining reel-out would
finish within `reelout_softstop` seconds at the CURRENT rate, record the
CURRENT `v_set` as `v_entry`, then decelerate LINEARLY from `v_entry` to 0.
A linear ramp's area is `v_entry*T/2`, so `T = 2*remaining/v_entry` is solved
exactly (not guessed) to land precisely on `reelout_l_max` — `T` comes out to
roughly 2x `reelout_softstop`, since braking from `v_entry` covers only half
the distance a constant `v_entry` would over the same time. Measured at
`reelout_softstop = 6.0` s (150 -> 350 m, 6 m/s): overshoot **0.016 m** (from
0.36 m), dip **-0.49 kW** (from -3 kW), whole-run force CV back to a normal
17 % (blended phase-4/phase-5 window — plain phase-5 CV alone is single
digits, per the `depower_final` sweep above), all 7 success criteria still
pass. `v_reelout` itself dips to only -0.09 m/s in the process — the residual
dip is the plant's own response lag, not the setpoint shape.
