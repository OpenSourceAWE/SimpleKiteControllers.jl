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
the power zone at 3494 N — which is exactly what a paying-out winch relieves.
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
away: measured 3.5 m/s of payout, `v_app` 49.6 m/s, run lost at t = 19.8 s. It
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
| mean power over the pay-out | 7.2 kW | **5.0 kW** |
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
WinchControllers' 0.25 s default), and if more is needed, the
`acceleration_limit` `simple_reelout.jl` hands `step!`, currently `rcs.max_acc` =
8 m/s² where the command only ever needs ~3. Do not slow the law itself.
