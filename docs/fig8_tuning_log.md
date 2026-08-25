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

## A 0.002 change in one constant cost 255 W, via a basin flip the power floor could not see (2026-08-22)

`output/scenarios/v04_3` measured 1581 W against `v04_2`'s 1836 W and `v04`'s
1885 W, same 4 m/s, same 150 m. Every archived settings file of the two later
runs is byte-identical and both ran clean on `ufryzen`; the only difference is
commit `c29158b`, `AWETRIM_V3KITE_DEPOWER_OFFSET` 0.109 -> 0.107. That constant
is in the flight loop, not just the report: `fly_opt_depower: true` feeds
`awetrim_depower_to_v3kite(l_dp)` straight into `rel_depower` for all of phase 4,
so v04_3 flew 0.002 less depower from the first phase-4 step.

The two runs diverge from the first re-optimization (t = 64.3 s vs 64.4 s) and
track each other for four installs. Then the fifth solve is asked a different
question, because by then the drift had stretched one lap by 3.4 s:

```
install #5   v04_2:  t=180.6 s  L=308 m  l_dp 1.468  ->  2217 W predicted
             v04_3:  t=182.4 s  L=312 m  l_dp 1.606  ->  1065 W predicted
```

Four metres further out, the warm-started solver converged into a different,
much worse local optimum — more depowered, higher and taller (`commanded_el_height`
15.3° vs 10.4°, `el_min_final` 16.6° vs 9.3°, clearance 103.9 m vs 82.5 m) — and
every later solve warm-started from there (943 W, 1022 W). That path held **27 %
of the reeling window**; measured/predicted is 1.02 in both runs, so the plant
flew exactly what it was handed. The arithmetic closes: put 2200 W back into that
27 % and the weighted prediction goes 1550 -> 1856 W, i.e. v04_2's 1836 W.

**Not a verdict on 0.107.** 0.12 -> 1885 W, 0.109 -> 1836 W, 0.107 -> 1581 W is
not a trend; it is a discrete basin flip in a chaotic run, and the same constant
would very likely give ~1850 W again at a slightly different re-opt phase.

**What should have caught it.** `min_power_frac` (0.3) is measured against the
STARTUP prediction, 1475 W here, so the floor was 442 W and 1065 W sailed
through. Because predicted power RISES with tether length, the startup value is
the run's minimum and that gate is effectively disabled by the time it matters.
Added `min_power_frac_prev` (default 0.7), the same rejection and cold retry
against the PREVIOUS INSTALL's prediction. It needs no anti-ratchet of its own —
a rejected reply is never installed, so the reference cannot move while a retry
chain runs.

It is skipped until one re-optimization has installed a path. The startup path is
solved at the starting length from the parametric guess, and the first
re-anchoring may legitimately predict far less: `v03` goes 200 -> 116 W (0.58)
and passes all nine criteria. That first step keeps `min_power_frac` as its only
power gate.

Replayed against every archived run in `output/scenarios/`, 0.7 rejects exactly
two candidates: this one (0.507 of its predecessor) and the collapsed pattern of
2026-08-20 (which `min_power_frac` already caught). The worst drop in any good
run is 0.946 (`v04_2`'s last install), and the startup step aside, everything
else measured sits above 0.98.

## The clearance gate was rejecting three replies in a row, and nothing in the request knew about it (2026-08-22)

`output/scenarios/v06_4`, 6 m/s, 150 m: four of seven re-optimizations were gated
out, three of them on clearance and by less than 1.5 m.

```
t_044.7_s: rejected — clearance 38.6 m   # at L = 187 m
t_061.1_s: rejected — clearance 40.0 m   # at L = 226 m
t_079.9_s: rejected — clearance 39.5 m   # at L = 270 m
t_101.5_s: rejected — descends to 7.4°, below min_elevation + margin = 9.5°
```

The cost is the whole point of re-optimizing: the startup path flew **99 % of the
reeling window at 7451 W predicted**, against the 9273 W of the first reply that
got through, at L = 378 m. Four `max_reopt` slots and ~32 s of frozen wall time
bought nothing. The kite's own `flown_min_m` was 47.0 m — nothing unsafe was being
prevented, the run was reeling out and climbing through the floor within the lap.

**The ratio is bigger than "Height floor: no starting length clears AWETrim's
50 m" (2026-08-18) assumed.** That entry computed the anchor clearance as
`50 * r0/r_low` ≈ 42 m from ~35 m of reel-out per lap. Back it out of these
rejections instead: `50/38.6 = 1.30` at 187 m and `50/39.5 = 1.27` at 270 m, i.e.
55-70 m per lap at this wind. What the optimizer delivers at the anchor is 38-40 m
and the floor is 40, so every reply was a coin flip on the third digit.

**The gate was a post-hoc one.** `min_height` is checked after the 8 s solve is
paid for, and nothing in the request mentioned it: `pattern_elevation_min` was
`0.0`, so `pattern_limits_from` returned `nothing` and no elevation constraint went
out at all. The same fix that closed this for curvature on 2026-08-19 applies —
send what the gate will demand, and stop discovering it afterwards.

**`elevation_min_from_gates` (new, default `true`).** Each request now carries
`max(asind(min_height/L), min_elevation + candidate_elevation_margin)`, computed at
the length being asked for (`elevation_min_request` in `awetrim_client.jl`). It
inverts `path_min_height`, which is the gate itself, so the request and the gate
are the same question. What it would have asked for on this run:

| L | ask | which floor binds | anchor clearance at the ask |
|--:|--:|:--|--:|
| 150 m | 15.47° | clearance | 40.0 m |
| 187 m | 12.35° | clearance | 40.0 m |
| 226 m | 10.19° | clearance | 40.0 m |
| 270 m | 9.50° | elevation | 44.6 m |
| 320 m | 9.50° | elevation | 52.8 m |
| 380 m | 9.50° | elevation | 62.7 m |

All four rejections are inside that table — the three clearance ones below 270 m,
and the 7.4° one at 320 m where the elevation floor takes over. The floor FALLS as
the tether grows, which is why it travels with every request, warm `/step`
included: a box built once at 150 m would ask for 15.5° at 320 m, where 9.5° is
enough, and buy a needlessly high and less powerful pattern. Same argument as
`min_turn_radius`, opposite direction.

**`elevation_min_retry_margin` (new, 0.5°) — and why there was no retry.** The
curvature/clearance/elevation gates gave up on the spot, and correctly so under
the old code: a cold re-ask with identical inputs solves to the identical reply
(measured 2026-08-20 on the fold retries, three attempts with nothing varied).
`reopt_retry_el_offset` covers a solve that FAILED, never a reply that was
rejected. With a floor in the request the second question is a different one, so a
clearance/elevation rejection now re-asks at a floor raised by the measured
shortfall plus this margin, spending the `blend_max_retries` budget. The guess
centre moves too, but only upward for these — the alternating +/- is aimed at
folds. Curvature still does not retry: nothing about that question would change.

The shortfall is carried across cycles rather than reset per request
(`el_min_extra`, in the summary as `traj_opt.clearance.elevation_min_extra_deg`).
It is structural — an angle curve installed at the anchor, where the optimizer
earns part of its height by reeling out within the lap — so the next length has it
too, and paying one retry for it per run is enough.

Two things to know before the next run. The failure cache keys on the request as
the server sees it, and that now includes the pattern box, so entries recorded
before this change will not match and their requests will be sent again — which is
the correct behaviour, since they were a different question. And the price every
sent constraint carries applies here as it did to `min_turn_radius`: a constrained
cold solve lands on the best branch less often than a free one, so a 422 where
there used to be a rejected reply is the expected new failure mode.

Not fixed, still the real fix: the reply's radial profile is discarded. Stage 4.

## The depower SEED has to follow the wind, and a 422 at 8 m/s is what says so (2026-08-20)

`input_depower` is only a seed — the server optimizes it — so it had never looked
like a number that has to move with the conditions. At 8 m/s it is, and the
symptom is a solve that fails rather than a path that flies badly.

The solve is against an `angle_of_attack <= 14°` cap that is already BINDING at
6 m/s, and stage 1 of `/step` — which carries NO turn-radius constraint — is where
a seed on the wrong side of that cap shows up. Measured at 150 m, both from
`input_depower = 1.6`:

| | stage 1 | AoA | tension | outcome |
| --- | --- | --- | --- | --- |
| 6.0 m/s | 0.85 s, 21 iterations | 8.3-14.0° | 2753-3950 N | solved |
| 8.0 m/s | ran to the iteration cap | -25…88° | — | trim, winch-law and radial-continuity residuals all INFEASIBLE |

Stage 2 then converged to a point of local infeasibility, i.e. a 422, cached, with
the SAME 12.20 m turn-radius ask that had solved eighteen minutes earlier at
6 m/s. **So it was never the turn-radius knobs**, which is where the search
started.

**The fix is a one-sided ramp on the seed**: the request sends `input_depower +
input_depower_per_wind * (v - input_depower_wind_ref)`, clamped to the server's
1.1-2.3 m (`depower_seed`, `examples/awetrim_client.jl`), and adds nothing below
the reference. One-sided on purpose — less wind wants less tape, but all six runs
of the 5.0-7.0 m/s scan in `docs/wind_scan_results.yaml` converged from 1.6 m, and
lowering their seed would move answers that are already measured.

It is provisional, and it rests on two anchors: 1.6 m is the largest seed known to
converge, at 7.0 m/s, and 1.85 m is what 8.0 m/s was first solved with. That is
`input_depower_wind_ref: 7.0` and `input_depower_per_wind: 0.25` in
`data/traj_opt.yaml`. **A third measured wind should REPLACE this slope, not
extend it.** At `0.0` the seed is `input_depower` whatever the wind, i.e. the
behaviour before today.

## `min_feasibility_margin` 0.75 -> 0.82: what was measured at 6 m/s does not carry to 8 (2026-08-20)

0.75 was chosen the same morning as the split between turn authority and delivered
shape (the entry below), and everything behind that choice was measured at 6 m/s,
where it bought ~0.95 of flown margin. The 8 m/s run of 08:45 got 1.00, 1.00,
0.76, 0.84, 0.77 out of its five installs — three of them below the 1.00 at which
a corner stops being flyable at all.

**The run failed on the third of them** (t = 60.4 s, margin 0.76, L = 259 m):

- 18.8 % of 60-70 s in the steering clamp, against 0 % in every window from 30 to
  60 s;
- cross-track error growing 0.2 -> 6.4° while it was;
- at t = 64.31 s the closest point re-acquired GLOBALLY (`reacquire_margin`,
  `figure_eight_controller.jl`) and stepped the attractor 6°, the commanded course
  ~55° and the steering by 0.45 in one sample;
- peak d 10.15°, which is the `max d < 8.0°` criterion, on that path and no other.

So 0.82, one 10 % step up, both to reject a reply like that at the gate and to
widen what is asked for — the number is sent as well as checked. It is the same
story as the depower seed above: a value identified at 6 m/s that has to move with
the wind. When a second wind is measured, this one probably wants a ramp too
rather than a bigger constant.

## `turn_radius_lap_reelout_m`: one number works because the lap's reel-out barely moves with length (2026-08-20)

The turn-radius request is scaled by `1 + dL/L`, and every re-optimization
measures that ratio off the previous reply (`reelout_anchor_ratio`). The startup
`/init` is the one request with no reply to measure — and it is where the ratio
matters most, since the same `dL` is a bigger fraction of a short tether: 35 m is
1.23 at 150 m against 1.09 at 380 m.

One assumed number covers it because the lap's reel-out is nearly
length-independent: measured 33.2 m at 150 m and 35.0 m at 380 m.
`data/traj_opt.yaml` ships 30.0. It IS an assumption though — it is `v_reelout`
times a lap, so it moves with the wind and the winch.

Left at `0.0` the startup asks for `turn_radius_headroom` alone, which at 150 m
bought a path flown at margin 0.90 against the 0.74 demanded at the time:
accepted, but on the steering clamp in its tightest corner until the first
re-optimization replaced it.

## The whole-run minimum is the LAST in-air shift missing the gate by 0.04 (2026-08-20)

The run's minimum elevation fell 9.66 -> 8.70° between archives `_070905` and
`_071750`. Nothing in `data/` changed between them — the six settings files are
byte-identical — and the only code that moved was the Q rate limit
(`q_rate_gain = 2.0`, the entry below). It is not a Q effect.

**Where the minimum is.** t = 129.1 s, azimuth -7.9°, phase 5, tether 379.9 m,
`v_reelout` 0.08 m/s — 6.0 s after reel-out stopped at t = 123.1 s, in the FIRST
phase-5 lobe. It is a one-off: the lobe minima either side of it read 10.36
(t = 122.4, phase 4) and 10.46, 9.64, 10.45 (phases 5). Cross-track error at the
dip is 3.62°, the worst of the settled window against a phase-4 typical 2.4-2.8°.

**Why that lobe.** `depower_final` (0.274 -> 0.328, delivered over ~1.5 s by the
KCU's rate limit) costs 23 % of `c1`, so every phase-5 lobe sags ~1° deeper than a
phase-4 one — that part is by design and shows in all four. The extra 1.7° is that
the reference the kite entered phase 5 on had received NONE of its phase-5 lift:

| | shift owed | at | margin | outcome | floor |
|---|---|---|---|---|---|
| `_070905` | +1.60° | t = 116.9 | 0.75 | held back | |
| | +1.60° | t = 117.8 | 0.75 | **blended in** | **9.66°** |
| `_071750` | +1.58° | t = 117.1 | **0.71** | held back | |
| | +1.58° | t = 127.6 | 0.86 | carried by an install | **8.70°** |

Same settings, same gate (`min_feasibility_margin` 0.75). One run cleared it on the
nose and flew the last 5 s of reel-out and all of phase 5 with the lift on the path;
the other missed by 0.04, so `el_offset_final` plus the converged bias reached the
kite only at the 380 m install — 4.5 s after the winch had stopped, blending until
t = 131.6 s, i.e. straight past the minimum at 129.1 s.

**The in-air route was all-or-nothing where the install route rations.** Fixed in
`examples/simple_opt_reelout.jl` by giving it the same ladder: the shift's MEAN goes
in whole and its SPREAD is scaled back until the curvature gate passes, then the
mean itself down to a quarter, the remainder staying in `el_target - el_applied` to
be retried next lap exactly as before. The split is the one the install ladder was
measured on (2026-08-20, 380 m: a uniform lift of the mean 0.95 -> 0.97, the spread
0.95 -> 0.41), so the mean — which is most of the 1.6° and all of the height — is
no longer thrown away with the spread. A rung that moves no band by more than 0.01°
falls through to the hold-back rather than spending a 4 s blend.

**FLOWN, and the mean-only rung is the whole result** (archive `_080541`, against
`_074433`; nothing else moved, 7 of 7 solves either way). The t = 117.1 s shift that
was refused at 0.71 goes in as `100 % of the mean, 0 % of the spread` at **0.76**,
4 s before the last of the reel-out, and 50 % of the spread follows at t = 121.1 s
(0.75). A second mean-only rung lands in phase 5 at t = 139.2 s (+1.09°, margin
0.90), which the all-or-nothing route had never delivered either.

| | `_074433` | `_080541` |
|---|---|---|
| min elevation, whole run | 8.70° | **9.65°** |
| first phase-5 lobe (t ~ 129) | **8.70°** | **10.19°** |
| where the run's minimum is | the 4->5 transient | an ordinary lobe, t = 140.6 s |
| `delivered_deg` | 2.33° | **3.42°** |
| power | 8644 W | 8637 W |
| RMS d | 1.53° | 1.56° |
| laps / criteria | 9.0 / 8 of 8 | 9.0 / 8 of 8 |

The transient is gone as a transient: the first phase-5 lobe reads 10.19° against
the 10.23° of the last phase-4 one, so the 4->5 boundary no longer sets the run's
floor at all. What is left is the ordinary phase-5 sag, and its worst lobe is the
number to beat next. Cost: 7 W (0.08 %) and 0.03° of RMS d.

What it does NOT fix is the SHAPE. `delivered_profile_deg` is still flat
(`[3.42 ×5]`) — the t = 128.0 s install rations the spread to 0 % as ever, and the
t = 125.1 and 132.0 retries read 0.57 and 0.38. The mean is delivered, the profile
is not; that is the same 380 m limitation as the entry below, unchanged.

**`reelout_softstop` is NOT the follow-up, and the reasoning that said it was is
worth recording so it is not proposed again.** The stop is hard (`reelout_softstop`
= 0.0): `v_reelout` overshoots to -0.8 m/s, the drum reels 1.5 m back in, tether
force spikes 3500 -> 4470 N and `v_app` 21.6 -> 28.7 m/s over t = 123.3-127.0,
immediately before the dip — so a soft stop looks like the other half of the same
transient. It is not, for two reasons that compound:

- **The stop ramp is open-loop and discards `v_cmd` for its whole duration.** Once
  `stop_start` latches, `v_set = stop_v_entry * (1 - (t-t0)/stop_T)` and nothing
  else — neither the `force_release` bypass nor the `UpperForceController`'s output
  reaches the drum for ~12 s. That is the same shape as the `reelout_softstart` bug
  fixed on 2026-08-19 (an open-loop timer overriding closed-loop tether
  protection), except the softstart at least has a force release and this has none.
- **The ramp is flown at `depower_setpoint`, not `depower_final`.** The depower step
  fires at phase 5, i.e. at the END of the ramp, so the entire deceleration happens
  with the kite fully powered while its reel-out relief decays to zero. The
  `depower_final` sweep below gives the destination: frozen length at 0.30 is
  **5149 N** mean and 0.274 is on the steep side of it (0.30 -> 0.328 alone is
  5149 -> 3456 N). The 4470 N being removed is a ~2 s transient; what replaces it is
  a monotone climb toward several times that, with the winch unable to pay out.

The 2026-08-16 measurement below is real (overshoot 0.36 -> 0.016 m, power dip
-3 -> -0.49 kW at `reelout_softstop = 6.0`) but it scored the power undershoot, not
the peak force during the ramp. Anything here needs the ramp to take the force law
back — `max(ramp, force_release * v_cmd)`, which gives up the exact landing on
`reelout_l_max` — or the depower step moved to the START of the ramp, or both. Until
one of those exists, the hard stop is the cheaper of the two problems.

## The learnt SPREAD reaches the kite everywhere but phase 5, and the gate is not the lever (2026-08-20)

The elevation learner converges on a profile and the kite flies its MEAN. Baseline
(`1ba6dab`, archive `2026-08-20_013624`, re-run of `_012624` and bit-identical to it
in every physical number): learnt `[+1.59 +1.81 +2.45 +2.77 +1.79]°`, the phase-5
install rationing 100 % of the spread away, the `+1.57°` shift behind it refused
three times in the air (margins 0.70, 0.67, 0.34 against 0.78) and
`delivered_profile_deg` flat at `[2.33 ×5]`.

Two levers, one run each, on the same seed and wind:

1. **Ration order reversed** — the lobe lift given up before the droop spread
   (`examples/simple_opt_reelout.jl`). Archive `_014616`, 8 of 8, 8641 W. The
   358 m install went from "75 % of the spread" to the spread WHOLE (lobe lift at
   75 % instead), and the in-air route was unaffected. The 380 m install still took
   0 %: dropping the lift frees ~0.14 of margin where the spread costs ~0.54, so it
   buys one install, not the decisive one.
2. **`min_feasibility_margin` 0.78 -> 0.72** — archive `_015130`, 8 of 8, 8640 W.
   It bought one more in-air blend (`t_103.3`, 0.74 vs 0.72) and the 358 m install
   now carries spread AND lobe lift whole. Phase 5 unchanged: 0 % of the spread,
   `delivered_profile_deg` still `[2.33 ×5]`, the shift refused at 0.62 and 0.34.

**Why the gate is not the lever.** The same number is SENT to the optimizer
(`min_turn_radius = margin/(c1*max_steering)`, scaled), so a lower gate also asks
for a tighter reply, and the reply lands ~1.15x above whatever was demanded. The
installed margins moved with it — 0.79/0.81/0.82/0.87 at 0.78 against
0.83/0.73/0.76/0.80 at 0.72 — leaving the SLACK the lift has to spend unchanged to
±0.01 (0.09 -> 0.08 at the last install), and `turn_radius_request_m` 14.19 -> 13.08
with the predicted power flat (9249 -> 9266 W). What it did cost is turn authority
the run keeps: `margin_final_flown` 0.87 -> 0.80 and phase-5 minimum elevation
9.51 -> 9.38°.

So `turn_radius_headroom` (1.10) is the knob that MAKES slack — it raises the
request without moving the gate — and it is the one to try next for the phase-5
spread.

**Tested, and it is the one: `turn_radius_headroom` 1.10 -> 1.20** (third run,
archive `_020401`, gate left at 0.72 so only one thing moved). The phase-5 install
took **25 % of the spread** where every run before it took 0, and
`delivered_profile_deg` is a shape at last — `[2.09 2.18 2.41 2.56 2.41]°` against
the flat `[2.33 ×5]` of all three earlier runs. It cost nothing measurable: 8641 W
against 8640, 8 of 8 criteria, phase-5 minimum elevation 9.55° (the best of the
three), `turn_radius_request_m` 13.08 -> 14.27. The in-air route improved too —
the `t_118` shift was refused at 0.70 against 0.72, i.e. it now misses by 0.02.

The ladder still stops one step in: the 380 m install lands at margin 0.73 with 25 %
of the spread, one rung above the gate. Two more runs settled what is on either side
of that.

**1.30 reaches the next rung and is not worth it** (archive `_021401`, gate 0.72).
The 380 m install carried the spread WHOLE, but phase-5 minimum elevation fell
9.55 -> 7.95°, power 8641 -> 8600 W and RMS d 1.52 -> 1.56° (8 of 8 and 8.5 laps
either way). Part of the extra delivery is not delivery: the wider pattern tracks
better, so there is less to correct — the converged profile fell from
`[1.64 1.90 2.60 2.90 1.79]` to `[0.48 0.65 0.68 0.21 -0.14]°`. Headroom went back
to 1.20.

**The gate is a real trade at 1.20, not a free revert** (archive `_054646`, headroom
1.20, gate back to 0.78). The delivery is lost again — phase-5 install at 0 % of the
spread, `delivered_profile_deg` flat at `[2.33 ×5]` — and what comes back is turn
authority and tracking: `margin_final_flown` 0.80 -> 0.94, laps 8.5 -> 9.0, RMS d
1.52 -> 1.49°, power 8641 -> 8642 W, energy 836.9 -> 837.1 kJ. The price is the
thing the correction exists for: phase-5 minimum elevation 9.55 -> 8.65°.

So at headroom 1.20 the choice is 0.72 for +0.9° of phase-5 floor and a delivered
SHAPE, against 0.78 for 0.14 of phase-5 turn authority, half a lap and 0.03° of RMS.
Power and energy do not separate them (within 2 W and 0.2 kJ over four runs).

**0.75, the split** (archive `_060552`, 7 requests, 7 installed, on the fixed
client — the two runs before it lost 3 and 4 requests to the broken pipe below and
are not comparable). It is the best of the three on the number the correction exists
for, phase-5 minimum elevation **9.66°** (9.55 at 0.72, 8.65 at 0.78), while keeping
`margin_final_flown` 0.88, 9.0 laps and 8636 W — i.e. most of what 0.78 buys, at
0.72's elevation. RMS d 1.54° is the worst of the three by 0.05.

What it does NOT do is leave a shape on the path. The in-air route delivered the
whole +1.60° per-band shift at t = 117.8 s (margin 0.75 against 0.75 required), and
then the 380 m install at t = 128.5 s rationed the spread to 0 % and REPLACED what
was flying with the flat mean — `delivered_profile_deg` `[2.35 ×5]`.

**That take-back is geometric, not bookkeeping.** It looked like the ration ladder
scoring from `el_target` while ignoring what the path in the air already carried, so
the ladder was given a floor at the applied spread (`bias_floor`, the applied
deviation projected onto the new target's shape; rungs at or above it first, the
floor itself last among them, and a `@warn` when the install still ends below it).
The run at 1.20 / 0.75 with the floor in is **byte-identical** to the one without it
(`_061442` vs `_060552`): the ladder had already tried 100 % of the spread first and
the 380 m reply cannot carry it — with the spread on, that curve reads ~0.34 against
the 0.88 it reads flat. The floor only bites in the intermediate case (a floor of
0.6 now stops there instead of falling through to 0.5), and the warning stops the
downgrade being silent. Keeping the shape at 380 m needs a different reply, not a
different ladder.

## The steering steps mid-strand are Q re-selection, and `branch_tol` had no hysteresis (2026-08-20)

Steering steps of up to 0.38 between t = 20 and 121 s, some of them into the
±0.32 clamp. They are not the plant, the winch or the KCU: every one coincides with
a step in the raw guidance course (`var_05`) and in the cross-track error
(`var_01`), which is Q's OWN distance — at t = 120.5 s it flips 2.54 <-> 3.50° in
consecutive samples. 24 events in the window, every one at |azimuth| **8.5-10.2°**,
on both lobes, nowhere near the crossing.

The cause is in `calc_attractor`. Among local minima within `dmin + branch_tol`
(3.0°) it takes the smallest steering effort, and that comparison was rebuilt from
scratch every step with the incumbent being whichever point is nearest THIS step —
not the Q chosen last step. Mid-strand the kite runs nearly parallel to the path
~2° off it, so the distance profile is flat, several minima sit inside 3°, and a
wiggle in the course estimate changes the winner: Q moves ~5 points, the attractor
~1°, the commanded course 6-17°, and the P path answers with a step.

Fixed with hysteresis (`branch_hysteresis`, 10.0° default, beside `branch_tol`): a
challenger must be better aligned by that much to take Q on merit, and inside the
band the tie goes to the candidate nearest the PREVIOUS Q. No lag results — the
candidate set is rebuilt each step, and on a synthetic walk the mean Q lag went
0.66 -> 0.77 index.

**It is a partial fix**, measured on the 150 -> 380 m run (archive `_064132`,
7 of 7 solves): flips 24 -> 19, `hf_std_steering` 0.0157 -> **0.0109** (-31 %),
`hf_std_turnrate` 0.64 -> 0.60°/s, RMS d 1.54 -> 1.51°, power 8636 -> 8641 W, 9.0
laps, 8 of 8. The largest single-sample step is still 0.39 and the survivors are at
the same |azimuth| 8.4-10.2°, i.e. the competing candidates there differ in
alignment by MORE than 10° and are switched on merit, not as ties — the same thing
a synthetic rippled path showed (84 -> 82 hops at hysteresis 10°).

**`branch_tol` 3.0 -> 1.0 is a NULL result** (archive `_064919`): the run is
bit-identical to `_064132` in every logged sample — same 19 flips at the same
azimuths, same 0.0109, same 8641 W. So the survivors do not come from the
disambiguation band at all: every candidate it was ranking was already within 1°.
`branch_tol` went back to 3.0, since 1.0 bought nothing and 3.0 is the value the
crossing guard was tuned with.

**What the instrumented run says (archives `_070342` and the A/B after it).** The
log has no free channel — `var_14`/`var_15`/`var_16` are V3Kite's `rel_depower`,
`L/D_wing` and `L/D_eff`, written by `log_state!` AFTER the example's own
assignments, so a first instrumented run came back with a depower signal in
`var_14` and was wasted. The diagnostics now go to `output/q_diag.csv` (`t`,
`q_idx`, `q_near`, `q_align_deg`, `u_s`), `q_near` being the plain nearest point
with alignment and window ignored.

The cause is upstream of every guard: **the nearest point itself teleports**, 15-16
times between t = 20 and 121 s, by 3 to 20 indices on a 360-point path (the count is
identical with the hysteresis on and off — it is raw geometry, not ranking). Q
follows it, and the attractor 8° of arc beyond Q moves ~1°, which is the steering
step. `q_align` at those moments is 18-28°, far from the `aligned` gate's 90°, so
that gate is not involved either.

The hysteresis limits the DAMAGE rather than the cause, which the A/B confirms
(same run, `branch_hysteresis` 10 vs 0): big Q steps 29 vs 38, backward moves 6 vs
10, worst single-sample steering step 0.39 vs 0.467, mean Q lag 1.57 vs 1.23 points.
So it is kept at 10.0 — it costs a third of a path point of lag and buys a quarter
of the big steps. An earlier reading of these same rows as "the hysteresis causes a
lag-then-snap" was wrong: the snap is there without it, and worse.

**The RATE LIMIT is the fix** (`q_rate_gain = 2.0`, archive `_071750`): Q may move
at most that many times the arc the KITE flew this step, floor of one point, either
direction — suspended when the search goes global, since re-acquisition is the one
case where Q must jump. A swapped argmin is then walked to over a few steps.

Measured against the run before it: Q steps larger than 2 indices **29 -> 1** (the
survivor is `set_path!` re-indexing at an install), steering steps above 0.05 in the
window **20 -> 1**, and the one survivor is at t = 22.27 s, which is not a Q event at
all — it is the entry handover, where the regulated error wraps -32° -> +143° with
the kite 44.5° off path and the command goes to the clamp for the dive. **After
t = 30 s the largest steering step in the whole run is 0.04.** `hf_std_steering`
0.0109 -> **0.0085** (0.0157 before any of this, so -46 % overall). Everything else
is unchanged or better: RMS d 1.51 -> 1.53°, power 8641 -> 8644 W, 9.0 laps, 8 of 8,
7 of 7 solves, mean Q lag 1.3 points, and 0.1 ms per step of cost.

The instrumentation was removed once it had answered. To bring it back: collect
`t`, `fec.last_idx`, `argmin` of `SimpleKiteControllers._dist` over the path with
alignment IGNORED, and `rad2deg(abs(wrap2pi(fec.tangent[fec.last_idx] -
fec.course)))` into vectors in the loop and dump them next to the log — NOT into
`var_14`/`var_15`/`var_16`, which are V3Kite's `rel_depower`, `L/D_wing` and
`L/D_eff` and are overwritten by `log_state!` after the example writes them.

That leaves the plain nearest point. `imin` is the minimum over the window, and
the tie-break only ever reconsiders STRICT local minima (`d <= dists[i±1]`); a
competing point on a plateau, or one that flips in and out of the `aligned` set as
the course estimate wiggles, is never a candidate and moves Q without either guard
seeing it. Before another knob is turned here, log the Q INDEX per step (a free
`var_14`) and read what it does at a flip — two runs have now been spent on guards
that rank candidates, and the evidence says the jump is upstream of the ranking.

## Two more unguarded branch-teleport paths at the crossing: `reacquire_margin` and `set_path!` (2026-08-20)

Found from a plotted run (archive `2026-08-20_100839`, the 150 -> 380 m reel-out),
not from a dedicated diagnostic pass: unwrapped ψ swings between -50° and -313°
for 3 lobes, then between +135° and -320° for 2 lobes, then back. The kite's
actual azimuth/elevation trace is the SAME size in both windows (checked
directly against the log, not inferred) — the extra ~180-360° is heading/course
alone, both moving together within a few degrees the whole way through, so it is
a real extra turn, not a plotting artifact. It starts at t = 62.75-62.9 s, where
`var_02` (attractor azimuth) jumps ~7.4° in one 0.011 s step and `var_06`
(regulated error) spikes -3.9° -> +44.3°, with the kite's own measured angular
speed a steady ~5.8°/s through the same window (rules out a `min_speed` dropout)
and cross-track error (`var_01`) never above ~6° (rules out `reacquire_dist`
genuine off-path re-acquisition). A jump that size in one step, at that speed,
is more than `q_rate_gain = 2.0` should ever allow — the guard from
"The steering steps mid-strand…" above should have capped it to about a point
of arc.

Two gaps in the guards explain it, both in `src/figure_eight_controller.jl`, and
both fixed here. **Neither fix was verified by rerunning the sim** — the finding
and the fix are from reading the code and the log, not a measured before/after
on the reel-out; that is still open.

**1. `reacquire_margin`'s global search shares `aligned`'s 90° gate**, and near
the crossing the far branch can pass it too. The RATE LIMIT block exempted any
`went_global` jump on the theory that re-acquisition is the one case Q must be
allowed to jump — true for a kite genuinely beyond `reacquire_dist` (`use_window
== false` from the start, never touches this flag), but that is not the only
way `went_global` becomes true: an active window whose best ALIGNED candidate is
merely `reacquire_margin` (3°) worse than the global ALIGNED best also sets it,
and that comparison can go to the wrong branch without the kite being off-path
at all. Fixed by rate-limiting a `reacquire_margin` jump exactly like any other
candidate swap; genuine off-path re-acquisition is untouched, since it was
already exempt via `use_window` alone.

Verified synthetically, not on the real run: `_make_test_controller` with a
small `search_window` (1-2°) and `calc_attractor` called in bigger index strides
(to force the window to fall behind) reliably triggers `went_global`. Comparing
the index jump against the bound the guard itself computes
(`q_rate_gain * speed * dt`), the exempted code let it through at up to **2.33x**
that bound; with the fix it never exceeds 1.0x across a sweep of window/stride/
offset combinations. Regression test: `reacquire_respects_rate_limit`.

**2. `set_path!` had no branch guard at all.** `examples/simple_opt_reelout.jl`
calls it every simulation step for the whole `path_blend_time` (4.0 s in this
run's `traj_opt.yaml`) of a gradual path swap (`blend_paths` then `set_path!` in
the main loop), and it re-indexed Q to the plain nearest point on the new path —
no `aligned`, no `branch_hysteresis`, no `q_rate_gain`, since it writes
`fec.last_idx` directly and those guards only ever see it downstream, inside
`calc_attractor`. As the blended curve reshapes step by step near the
self-intersection, the nearer of the two branches can swap, same failure as an
unranked `argmin` elsewhere in this file. The timing fits: this run's reopt
install nearest the anomaly is logged at t = 60.8 s, and 60.8 + 4.0 s = 64.8 s
brackets the observed t = 62.75-62.9 s.

Fixed by restricting the remap to points whose tangent is within 90° of the OLD
`last_idx`'s tangent, falling back to the plain nearest point if none qualify —
the same idea as `aligned`, applied with the only reference direction
`set_path!` has (it gets no live position or course estimate). Gated on
`fec.has_prev`: a fresh, never-flown controller's `last_idx` is the constructor
default (`1`), whose tangent means nothing, and applying the filter there broke
the existing `set_path` test (`dmin` at index 40 came back 2.51° instead of
~0 — the guard picked the tangent-nearest point on the new path over the true
nearest one, exactly as designed, but the "old branch" it was preserving was
never real).

## The extra loop survived both guards above — the fold is IN `blend_paths`, not in Q (2026-08-20)

Reran the 150 -> 380 m reel-out after the two fixes above (archive
`2026-08-20_104750`): the discontinuity in `var_02`/`var_06`/`set_steering` at
t = 62.85 s is gone — smooth signals, no jump — but the unwrapped-ψ "extra loop"
itself was still there, same timing, same shape, now perfectly smooth. That
rules out a Q-selection bug: nothing upstream of `calc_attractor` was jumping,
so the guidance had to be tracking something that genuinely asks for the extra
turn.

Instrumented `blend_paths`'s own output (`path_min_radius` of the blended curve,
logged at every step of every blend — removed again once it had answered) and
reran once more (archive `2026-08-20_110305`). Confirmed directly: three blend
windows collapse to `path_min_radius` ~0° at some point strictly BETWEEN the
endpoints, though both endpoints are individually fine —

| blend window | radius at w=0 | min DURING the blend | radius at w=1 |
|---|---|---|---|
| t=60.8-64.8 s | 1.33° | **0.000°** | 0.86° |
| t=81.5-85.5 s | 0.91° | **0.003°** | 0.65° |
| t=102.4-106.4 s | 0.91° | **0.000°** | 0.59° |

— and those three windows are exactly the three extra-loop episodes. Earlier
blends (t=39-58 s), where the old and new pattern are still similar in size,
show no collapse at all (min stays within ~0.1° of both endpoints). `blend_paths`
interpolates two closed curves point BY INDEX; as reel-out narrows the pattern
more each install, the two endpoint curves diverge more, and a plain linear
blend between two differently-shaped closed curves can produce a cusp in the
MIDDLE that neither endpoint has. The guidance then tracks that cusp correctly
— no bug in it — which from the ground reads as a smooth extra loop with zero
cross-track error, exactly what was measured. Nothing existing catches it: the
curvature gate only ever checks the CANDIDATE endpoint, never the path in
between.

**Fixed by rate-limiting `w` itself**, the same shape of fix as the two above:
`examples/simple_opt_reelout.jl`'s blend loop tracks `blend_w_eff` (the weight
actually flown) separately from the clock-driven `w`, and only advances it to a
candidate whose `check_pattern_feasible` margin clears `min_feasibility_margin`.
A folded patch holds Q on the last safe shape instead of flying through the
cusp. `blend_w_eff`/`blend_w_warned` reset at both places a blend starts (the
reopt install and the el_bias shift-blend).

First cut (archive `2026-08-20_111030`) checked the candidate against the live
`l_now` and looked clean — unwrapped ψ in -333.2..-49.6°, every lobe the same
~260° swing, all 8 criteria — but the check was ONLY against ψ, and the very
next run (`_111412`) came back with `reopt.requests: 1` instead of 6 and an
EMPTY `el_bias.shift_delivery`: the first blend of the run held and never let
go, and both later mechanisms are gated on `isnothing(blend_to)`. Two bugs, not
one:

1. **The live `l_now` was the wrong length to check against.** Margin generally
   worsens with `l_tether`, and reel-out keeps advancing while a hold is in
   effect, so a hold under the live length turns into a gap that widens every
   step it stays held. Fixed with `blend_l0`, the length captured ONCE at
   `blend_t0`. Alone, this did not fix the stuck run (`_111947`, still
   `requests: 1`).

2. **The candidate margin was checked at the wrong resolution.** The accept-time
   gate checks `chk_az`/`chk_el` at the server's native ~99 points, not
   `cand_az`/`cand_el` at the run's flying `n_path` (360) — an upsampled check
   overstates an otherwise-smooth curve's tightness, so the two resolutions
   exist on purpose. Checking the blend against `n_path` directly meant `w = 1`
   (`blend_to` itself) could fail a gate it had already passed at accept time,
   holding forever. The tempting fix — downsample the candidate to
   `chk_points` before checking, matching the accept-time gate exactly — is
   WRONG: measured on archive `_112525`, it restored `reopt.requests: 5` but
   brought the +133°/-108° extra-loop episodes straight back, because the fold
   this guard exists to catch only shows up once the curve is resampled UP to
   flying resolution — the coarser check cannot see it. So the candidate margin
   stays checked at flying resolution (`cand_az`/`cand_el` as `blend_paths`
   returns them), which means `blend_to` is genuinely NOT guaranteed to pass
   here even though it already cleared the coarser accept-time gate.

Given that, a hold can legitimately never clear — the accepted candidate itself
folds once flown at `n_path`, a resolution the accept-time gate never checked.
So a `w = 1` failure no longer holds indefinitely: it ABANDONS the install,
reverting `fec` to `blend_from` and clearing `blend_from`/`blend_to`, freeing
the next reopt cycle to request a fresh reply rather than starving for the rest
of the run.

Measured (archive `2026-08-20_113021`): `reopt.requests: 5`, `installed: 5`,
all 8 criteria pass, and unwrapped ψ stays in -333.2..-2.7° for the whole run —
every lobe the same swing, no extra-loop episodes. Power 19434 W vs 20560-20582
W before (a ~5.5 % dip — abandoning a folded candidate costs more than holding
briefly did, since the run flies the OLDER path for longer while it waits on
the next reopt cycle instead of the folded one at all).

**Waiting out the ordinary schedule was still a bad trade** — the kite flies a
path already known stale for up to `reopt_every_n_laps` laps for no reason, on
top of the wall time already spent on the reply that got thrown away. Fixed by
rewinding `reopt_lap` on abandonment (`fig8_idx_progress / n_path -
reopt_every_n_laps`), so the very next loop pass is eligible to request a
replacement rather than waiting out the normal cadence. Costs one extra solve
from `max_reopt`'s budget per abandonment; worth it; a stale path was the whole
problem this fix exists to avoid.

Measured (archive `2026-08-20_114258`): `reopt.requests: 7`, `installed: 7` —
MORE than the 5-6 of a run with no abandonments, since immediate retries add to
the ordinary schedule rather than replacing a slot in it. All 8 criteria pass,
unwrapped ψ again -333.2..-2.7° with no extra-loop episodes, power 19632 W
(between the hold-only 20012 W and the wait-out-the-schedule 19434 W — flying
the stale path for less time costs less than the full ordinary wait, but more
than never abandoning at all).

**The `check_pattern_feasible`/`min_feasibility_margin` check above was itself
wrong — do not bring it back.** The very next run came back with `reopt.
requests: 1` and an EMPTY `shift_delivery`, and this time it was not a fluke:
the accept-time gate checks candidates at the server's native ~99 points
(`chk_az`/`chk_el`), but `blend_paths`/`blend_to` fly at `n_path` (360,
upsampled), and **upsampling alone roughly HALVES `path_min_radius`** —
measured directly, an ordinary already-accepted candidate with no fold in it at
all read 3.39° at 99 points and 1.54° at 360, same curve. Checking the upsampled
version against a threshold calibrated for the native one failed nearly every
candidate, including `blend_to` itself at `w = 1`: the whole run's worth of
reopt replies were silently abandoned one after another and the kite flew the
STARTUP path for all 112 s — invisible to `criteria`, which scores whatever
`fec` was actually given, not the discarded replies. A plotted comparison
against the optimizer's own (uncorrected) path made it obvious: the flown
pattern was the wide, short-tether STARTUP shape throughout, nothing like the
narrow one the optimizer kept sending.

**Fixed by comparing like resolution to like, not an absolute threshold from a
different one.** `blend_r0`, the smaller of the two blend ENDPOINTS' own
`path_min_radius` — BOTH measured at `n_path`, captured once at blend start —
replaces `check_pattern_feasible`/`min_feasibility_margin`/`blend_l0` entirely;
an intermediate `w` must clear `blend_fold_margin * blend_r0` (default 0.5).
Since `blend_r0` IS the smaller endpoint, both `w = 0` and `w = 1` trivially
clear it for any margin `<= 1` — no absolute-threshold mismatch, and no "even
the endpoint folds" case to abandon out of. A genuine fold is nowhere near the
bar regardless: measured, a folded blend's minimum was ~0.000° against
endpoints of 1.33°/0.86°.

**That reintroduced a milder version of the same problem `blend_l0` was for.**
A fold zone in `w` is not always a brief dip — measured, one spanned `w` =
0.2-0.78, over half the blend, `path_min_radius` under 10 % of `blend_r0`
throughout. There is no smooth crossing: every point strictly between fails the
same gate the endpoints on either side pass. Resuming with a single jump (fine
for a NARROW fold) meant a 0.58 jump in blend weight once the clock-driven `w`
cleared the far side — most of the pattern's shape changing in one step:
`criteria` failed `max d < 8.0°` (10.36° measured) and the extra-loop episodes
were back, just later in the run. `blend_max_jump` (default 0.3) bounds it: a
recovery bigger than that ABANDONS the install — reverts to `blend_from`,
rewinds `reopt_lap` for an immediate replacement — instead of crossing a fold
that wide.

Measured (archive `2026-08-20_120810`): `reopt.requests: 7`, `installed: 7`,
all 8 criteria pass (max d 5.99° vs the 10.36° that failed), power 20445 W, and
unwrapped ψ stays in -333.2..-2.7° for the whole run with every lobe the same
~260-280° swing — no extra-loop episodes anywhere. **This is NOT the version in
the script** — see the next entry for why, and what replaced it.

## Why retry, not react — checking the whole blend BEFORE flying any of it (2026-08-20)

`blend_max_jump`'s own abandon path still visibly discontinuous. Plotted a run
(`d`, elevation, ψ/χ/χ_set, `Δχ`, `u_s`, tether force) and found `u_s` stepping
and `d` spiking to ~6° at t = 70-73 s. Traced it to the abandon branch itself:
it called `set_path!(fec, blend_from...)`, snapping `fec` back to `w = 0` —
but `fec` was ALREADY at `blend_w_eff`'s shape (already verified safe, already
what was installed by the `set_path!` at the bottom of the loop every step it
held), so the revert was a second, gratuitous jump on the far side of the one
this whole mechanism exists to avoid.

Removing the revert (stay at `blend_w_eff`, just clear `blend_from`/`blend_to`)
fixed the jump but broke something worse: five reopt installs fired in 10 s
(each abandon rewinding `reopt_lap` for an immediate retry), each one's
`blend_from` inheriting the PREVIOUS attempt's barely-blended, increasingly
lopsided shape, burning through `max_reopt` while the kite ended up frozen at
`blend_w_eff = 0.094` — 9 % blended — for the last 40+ s of a 112 s run.
Unwrapped ψ reached +1886°: not oscillating, continuously spinning on the one
lobe the degenerate shape happened to cover. `criteria` still passed (scored
against whatever `fec` was actually given, over the WHOLE run, so 70 good
seconds hid 40 broken ones) — this was WORSE than the jump it replaced, just
less visible in one plot.

**The user's question was the right one: why should a bad reply ever get
`blend_from`/`blend_to` set for it at all?** Every fix so far reacted to a fold
AFTER a blend had already started. Checking the WHOLE prospective blend BEFORE
either global is ever assigned removes the entire class: if `blend_to` is
guaranteed fold-free across every `w` in `[0, 1]`, nothing downstream needs a
guard, a jump cap, or an abandon path — the blend loop goes back to a plain
linear ramp, no state but `blend_from`/`blend_to`/`blend_t0`.

`blend_folds(az0, el0, az1, el1)` (`examples/simple_opt_reelout.jl`) samples
`path_min_radius` at `blend_probe_points` values of `w` and compares each
against `blend_fold_margin * min(radius(az0,el0), radius(az1,el1))` — the same
relative test as `blend_r0`, just run ONCE, upfront, over the whole range
instead of reactively at the clock-driven `w`. In the reopt accept gate it
becomes a fourth condition alongside curvature/clearance/elevation, wrapped in
`for blend_attempt in 0:blend_max_retries`: a fold requests a fresh WARM
`/step` and retries (holding the simulation — the wall time already shows up in
`blocked_s`), and only gives up — same as any other rejection, falling back to
the ordinary `reopt_every_n_laps` schedule — once retries are exhausted. The
el_bias shift's own rung ladder gets the same check for free: a rung that folds
is treated exactly like a rung short of margin, tried and passed over for the
next, smaller one.

**Two bugs on the way to a working version, both self-inflicted:**

1. First cut checked `blend_folds(fec.az_path, fec.el_path, cand_az, cand_el)`
   directly — but `cand_az`/`cand_el` come out of `prepare_path` (resampled,
   rotated to the azimuth extreme), and raw `fec.az_path` is neither. Blending
   two curves whose point correspondence doesn't match TELLS `blend_paths`
   completely different things are the same index, which folds catastrophically
   at every `w`, not just where the two curves actually differ. Measured:
   `installed: 0` for the WHOLE run — even the very first reopt reply, at
   L = 189 m, previously margin 1.02 and clean at every resolution checked so
   far, was rejected every time. Fixed by canonicalizing `fec.az_path`/
   `el_path` through the SAME `prepare_path(...; resample = n_path, up_loops =
   fcs.up_loops)` used to build `cand_az`/`cand_el`, hoisted once per attempt as
   `cand_from` and reused for both the check and the eventual `blend_from`.
2. (Caught by the same fix, not separately measured): the retry loop's `for
   blend_attempt in 0:blend_max_retries` needed an extra `end` when it went in
   around the existing `if margin < ... elseif ... else ... end` chain — a
   plain missing-`end` `ParseError`, not a logic bug, but worth naming since
   Julia's error pointed at a line number far from the actual mismatch.

Measured (archive `2026-08-20_125009`, after both fixes): `reopt.requests: 5`,
`installed: 5` — every reply installed FIRST TRY, no retries spent. All 8
criteria pass, max d 4.95° (below the 5.99° a runtime hold still left), power
19438 W, unwrapped ψ -333.2..-2.7° for the whole run, and the largest
`set_steering` single-step change anywhere in the settled window (t >= 28.5 s)
is 0.031 — no discontinuity, however small, anywhere. **Every number here is
real except the power** — see the next entry: `blend_to` was never actually
set for a reopt install this whole time, so none of these five runs ever flew
a re-optimized path at all. `blend_fold_margin`, `blend_probe_points` and
`blend_max_retries` are `TrajOptSettings` fields (`src/traj_opt_settings.jl`);
`blend_l0` and `blend_max_jump` no longer exist.

## `blend_to` was never assigned — every "installed" run this session flew the STARTUP path (2026-08-20)

Every verification above was read off `reopt_events`/`criteria`, which score
whatever `fec` was actually given — and `fec` was never given anything but the
STARTUP path, for five straight runs, despite every one logging `installed: 5`
with a distinct margin and predicted power per install. Caught only because
the user asked for a pattern plot and read it correctly: "flown" was the wide,
150 m-anchor STARTUP shape for the WHOLE run, "optimizer, uncorrected" showed
five genuinely narrower replies, and they did not match. Power alone should
have been the tell — 19438 W repeated bit-for-bit across three different code
versions (the buggy fold-check giving `installed: 0`, and both "fixed"
versions after it) is not three re-optimized runs converging on the same
number, it is the SAME unre-optimized run three times.

The cause: hoisting `cand_from` (`blend_folds` canonicalization fix, above)
replaced

```julia
global blend_from = prepare_path(fec.az_path, fec.el_path;
    resample = n_path, up_loops = fcs.up_loops)
global blend_to = (cand_az, cand_el)
```

with `global blend_from = cand_from` — and dropped the `blend_to` line
entirely. `blend_to` stayed `nothing` (cleared at the end of the PREVIOUS
blend, and nothing after ever set it again), so `if !isnothing(blend_to)`
in the blend loop was false every step: `set_path!` was never called with the
new candidate, `fec` never moved. Every OTHER piece of accept-branch
bookkeeping — `opt_paths_raw`, `el_applied`, `fig8_idx_progress` rescaling,
the "installed" event with its real margin and predicted power — ran exactly
as if the install had happened, because none of it actually checks that
`blend_to` is what the guidance is using.

Restored the missing line. Verified directly, not just from the log this
time: archive `2026-08-20_130953`, `fec.az_path` at the end of the run spans
-7.90..7.91°, matching `opt_paths_raw[end]` (the LAST, narrowest reply) to four
significant figures — not `opt_paths_raw[1]` (the startup reply, -18.5..19.4°)
the way it silently had every time before. Flown azimuth extent narrows
progressively through the run (±17° at t=45-50 s, ±14° at t=70-75 s, ±7° at
t=105-110 s), matching the five installs at L = 189/229/274/324/382 m. Power
20515 W — finally a DIFFERENT number from the 19438 W that recurred across
three runs that never re-optimized at all. Unwrapped ψ still -333.2..-2.7° for
the whole run. This is the version in the script.

**Lesson for next time a check here reads clean:** `criteria` and the reopt
event log both score/report what a run BELIEVES it did, not what `fec` was
actually given. When a fix touches `blend_from`/`blend_to` again, confirm
`fec.az_path`'s extent against `opt_paths_raw[end]` directly before trusting
either.

## `heading_range` — a ninth success criterion, for the whole class of bug above (2026-08-20)

Every guard added today (`calc_attractor`'s rate limit, `set_path!`'s branch
guard, `blend_folds`) targets ONE mechanism that produces an extra loop. None
of them, and none of the EXISTING criteria, would have caught it as a
regression: `rms_d`/`max_d` stay small because the guidance tracks whatever
reference it is given correctly (the reference is what goes wrong), and the
extent/lap checks are blind to how many times the heading wound around to fly
a lobe. Today's bugs were all caught by eye, off a plotted `ψ`/`χ` panel or a
pattern-vs-optimizer comparison — worth a check that runs without a human
looking at a plot.

`fig8_metrics` now returns `heading_range`: `max - min` of the heading
UNWRAPPED over the settled window (`wrap2pi`'d steps, cumulative sum — same
technique used throughout today's diagnosis). An oscillating figure-eight
stays bounded here regardless of lap count, since each lobe swings back into
the same band — every clean run measured today reads ~330°, independent of
whether it flew 8 or 9 laps. `print_fig8_metrics` gates it at
`max_heading_range = 400.0`° (a new ninth criterion, keyword-configurable):
70° of margin above the measured baseline, comfortably under even the
mildest extra-loop episode measured today (+150-450° over baseline, i.e.
480-780° total — several of which passed EVERY existing criterion on their
own run). Regression tests in `test/test_fig8_controller.jl`
(`metrics_heading_range`): a synthetic oscillating heading stays under the
gate, a synthetic steady spin fails it by a wide margin.

Existing `print_fig8_metrics` regression tests that count `criteria` exactly
(`test_fig8_controller.jl`, `metrics_pattern_extent`) went `7 -> 8` and
`4 -> 5` for the new check.

## The blend-fold retry was asking the same question three times, and once fixed, converged on a collapsed pattern (2026-08-20)

The user asked directly: between the three retries logged as "blend folds
after 3 retries" (`t_060.8_s`, `t_072.5_s`, `t_085.3_s` on one run), did
anything actually change between attempts? No — the retry called `opt_step`
(a WARM request) with an unchanged `length`/`winch_params`/`min_turn_radius`
every time. A warm step re-anchors the server's OWN previous optimum, so an
unchanged query from an unchanged warm-start state solves to the same local
optimum every time: the three retries almost certainly re-asked the identical
question and got the identical folding answer back, at the cost of ~3x the
wall time for nothing. `reopt_retry_el_offset`/`el_seeds` exist for exactly
this reason on a solver FAILURE (docstring: "a retry from a different seed
usually lands elsewhere") — reused here.

**First cut scaled the guess offset UP by `blend_attempt`**
(`guess_el_center + blend_attempt * reopt_retry_el_offset`, so retry 3 asked
from 3x the one offset that magnitude is actually validated at). Measured:
`criteria` FAILED `RMS d`, `max d` AND the new `heading range` — the first
regression the new check caught for real. One install (`t_087.7_s`) predicted
2094 W against ~22000 W everywhere else in the SAME run, margin 0.87,
clearance 153.6 m — a pattern collapsed to near-zero amplitude, which
trivially clears curvature margin, clearance and `blend_folds` (there is
almost no pattern left to be tight, low, or foldable), got installed, and
unwrapped ψ reached -693° by the end.

Two fixes, both needed:

1. **Bounded the retry seed.** Alternating `guess_el_center +-
   reopt_retry_el_offset` (never scaled past the one validated offset)
   instead of walking further away each attempt.
2. **A power floor**, checked alongside `blend_folds` and retried the same
   way: `new_pred >= min_power_frac * opt_power_pred` (default 0.3), against
   the STARTUP prediction specifically — not the previous install's, so a
   chain of retries cannot ratchet the floor down alongside itself. This is
   the one that actually would have caught the collapsed pattern; the seed
   bound only reduces how often a bad basin is reached in the first place.

Measured (archive `2026-08-20_134122`): all 9 criteria pass, 7 installs, every
predicted power back in the ~21000-23200 W band (the `t_087.7_s`-equivalent
install this time predicted 23244 W), power 20519 W, unwrapped ψ
-333.2..-2.7°. `min_power_frac` is a `TrajOptSettings` field
(`src/traj_opt_settings.jl`).

**No way to say how many retries a run actually spent, short of grepping
`@info` text out of stdout** — asked directly, and the honest answer was an
inference from wall time (28.3 s/install measured vs a clean, no-retry
baseline of 15.1 s/install two runs earlier), not a number. Added
`blend_retries_total`, incremented once per cold-restart attempt, reported as
`reopt.blend_retries` in the run summary (`examples/reelout_results.jl`)
alongside `requests`/`installed`/`blocked_s`. Measured directly on the next
run (archive `2026-08-20_135218`): `blend_retries: 3` across the 7 installs,
criteria and power unchanged (all 9 pass, 20519 W) — confirms retries are
firing in normal operation, not just in the pathological runs measured above.

## `Broken pipe` on POST: HTTP.jl only auto-retries a dead pooled socket for GET (2026-08-20)

`docs/reelout_next_steps.md` had this as a client-side stale socket that
`retry_non_idempotent = true` does not cover, and worth fixing "before the next
campaign". It bit hard here: two consecutive runs lost 3 and 4 of their 7 solves
(`SystemError: write: Broken pipe`, one `unexpected EOF while reading HTTP/1 data`),
installed 4 and 1 paths against the usual 7, and produced numbers that looked like a
tuning WIN — every in-air shift blended in, RMS d 1.17°, phase-5 floor 11.17° —
because the run was flying an old, wide 295 m path all the way to 380 m. A server
restart did not help; the second run was the worse of the two.

The cause is in HTTP.jl v2.6.5, not the server: it does retry a request whose reused
pooled connection died, but `_retryable_method` (`http_transport.jl:1476`) admits
only `GET/HEAD/OPTIONS/TRACE/QUERY`, so no POST in `examples/awetrim_client.jl` was
ever covered. `retry_non_idempotent` feeds the status/policy controller, which is a
different layer. Re-optimizations are ~13 s apart, well past the server's keep-alive,
so the pooled socket is usually dead when it is picked up again.

Fixed in `post`: the pool is emptied before each request
(`HTTP.close_idle_connections!`) and up to 3 attempts are made, retrying only on
`stale_conn_error` — `SystemError`, `EOFError`, `HTTP.ParseError`, unwrapped through
nested `.error`/`.ex` payloads with a message match as backstop. Safe for this API:
re-sending `/init` or `/step` recomputes and overwrites what the server holds under
the optimization's name. The next run was clean, 7 of 7.

**Read `traj_opt.reopt.requests` against `installed` before quoting any run.** A run
that lost requests is not a run at different settings, and it flatters every metric
the correction work is measured by.

## `guess_el_center` 22 -> 26: retuning the winch moved the optimizer's request (2026-08-19)

Raising `wc_settings.yaml`'s `f_high` 7600 -> 8000 N is a controller change, but
`winch_from_wc` maps `f_max = f_high`, so it also changes the request the solve is
made under. At 150.0 m and 9 m/s the 22 seed that had converged at `f_max = 7600`
throws IPOPT's iteration limit at 8000.

**A LOOSER force bound, so the feasible set grew** — this is the multi-modality of
"IPOPT 422 at 180 m" below, not an infeasible problem. 26 is the only other seed
with recorded convergence at 150 m, so it is the one to try, and it solves. Note
what that record was taken at, though: 6 m/s and `f_max = 7600`, not these
conditions.

**The failure-pocket warning below still stands.** 26 is the NARROWER basin in
tether length — it is what threw 422 at 180.00027 and 181.0 m on 2026-08-18 — and
a reel-out run sweeps length continuously, so re-optimizations are where this will
show up first (`reopt_retry_el_offset` gives one retry at +2°). If the pockets
bite, the alternatives are 24 (untested) or decoupling the optimizer's `f_max`
from `f_high`, so that retuning the winch controller stops moving what is asked of
the optimizer.

## The runs DO repeat, and the wind is what the settings were never tested against (2026-08-19)

`docs/reelout_next_steps.md` asked for independent repeats: every run of the
2026-08-19 session was bit-identical, so nothing had been tested against the
optimizer answering differently. Four runs, all on `8231f95` with a clean tree.

**A repeat is exact, even across a server restart.** R1 (2026-08-19_094153)
re-flew the flown configuration after `bin/run_server restart` AND with
`output/opt_failure_cache.yaml` DELETED, so the five requests the baseline had
skipped were re-sent to a fresh server process. Every one of them failed again,
including the retry from guess el 24 deg, and the run reproduced
2026-08-19_092659 in every number: 8 laps, RMS d 1.28 deg, min elevation
10.4 deg, 8631 W, the learnt profile `[+0.51 +0.81 +1.68 +2.21 +1.20]` deg, the
same 4 installs, the same rejections at 0.68 and 0.73, the same shift held back at
0.48. So the "**the runs do not repeat**" trap recorded on 2026-08-18 does NOT
hold for the current code — the whole pipeline, server included, is deterministic
for a repeated request sequence, and a single run's numbers can be quoted. It also
confirms the failure cache's contents are honest and costs ~130 s of wall time
(300 s against 175 s) to verify.

**The wind is therefore the only lever that gets a different answer out of the
optimizer**, and it is what the settings had never been flown against. Five
speeds, everything else the flown configuration; the full table with force, power
and speed crest factors is `docs/wind_scan_results.yaml`:

    v_wind   laps   RMS d   max d   min el   power    ratio  cf_F  span/lap  % of B  sat   verdict
    5.0      6.0    2.05    11.21   11.1     4774 W   1.08   1.21  9.1 deg   90 %    10 %  FAILED max d, phase 5
    5.5      7.5    1.27     4.99    9.8     6568 W   1.03   1.12  7.5 deg   87 %     8 %  all 8 passed
    6.0      8.0    1.28     4.94   10.4     8631 W   1.00   1.07  7.4 deg   86 %     7 %  all 8 passed
    6.5     10.5    1.06     4.92   10.9    11041 W   0.99   1.26  6.6 deg   65 %     0 %  FAILED span
    7.0      9.5    1.26     4.98   10.5    13701 W   0.97   1.43  8.1 deg   94 %     0 %  all 8 passed

**The flown window is 5.5 to 7.0 m/s, and both ends fail for different reasons.**
At 5.0 the run never finishes reeling out (375.7 m of 380 in 150 s, stop reason
"none", so phase 5 is never entered) and tracking degrades badly — RMS d 2.05 deg
and a max of 11.21 against the 8.0 deg criterion, with steering saturated 10 % of
the time. That is the kite running out of turn authority in weak wind, and the
phase-5 half of the verdict is partly an artefact of `sim_time = 150 s`: a fair
5 m/s test needs a longer run.

**6.5 m/s fails the elevation-span criterion**, the one the size budget had just
named as binding: 6.92 deg flown against 7.46 deg required, `lift_budget`
reporting **-0.55 deg (-7 %)** where 6.0 m/s has +1.52 (+23 %). Everything else
improves — 27 % more power at 0.99 x predicted, the best RMS d of the scan, the
highest minimum elevation, and NO steering saturation at all. The failure is
purely the pattern's height: the optimizer commands a TALLER pattern at 6.5
(10.66 deg against 9.34) while the kite fills only 65 % of it.

**It is not monotone in wind, and that is the point.** 7.0 m/s passes with the
span at +2.21 deg (+34 %), because there the optimizer commands 9.17 deg and the
kite fills 94 % of it. The criterion is relative — 0.70 x the pattern COMMANDED —
so a run fails when the optimizer happens to hand it a tall pattern the kite will
not fill, and which pattern comes back is not smooth in the conditions. Judge the
span criterion across conditions, never off one wind speed.

**Crest factors are the number the scan adds for sizing.** Over the reeling
window, force goes 1.07 (6.0 m/s) -> 1.26 (6.5) -> **1.43** (7.0) and power 1.10
-> 1.26 -> 1.31, while at 5.0 it is the low mean rather than the peak that lifts
them (1.21 / 1.34). Reel-out speed stays tame throughout (1.12 … 1.22). So the
ground station sees its worst peak-to-mean force at the TOP of the flown window,
where every flight criterion is comfortable.

**It is the lobe lift that spends the span.** R4 (2026-08-19_095327), 6.5 m/s with
`el_offset_wing = 0` and nothing else changed:

    el_offset_wing   span/lap   % of B   budget         min el   RMS d   c2l    power
    1.5 deg          6.6 deg    65 %     -0.55  (-7 %)  10.9     1.06    0.00   11041 W
    0                8.2 deg    90 %     +1.92 (+28 %)   9.5     1.20    1.22   11004 W

so the 1.5 deg lobe lift costs ~1.6 deg of flown pattern height, which is the
whole failure, and buys 1.4 deg of minimum elevation, 0.14 deg of RMS d and the
droop (`centre_to_lobe_deg` 1.22 -> 0.00) for 0.3 % of the power. Both are
defensible at 6.5 m/s; the criterion is what picks. `el_offset_wing` is NOT a
setting that transfers across wind speed, and the answer asked for under "the
elevation SPAN is the tight criterion now" is now measured: raising the pattern
does narrow it, roughly degree for degree at the lobes.

**The shoulder band peaks at four of the five wind speeds**, so the open "bound
the shape" item is not an artefact of one condition: the converged profiles are
`[-0.30 -0.20 +0.42 +1.18 +0.85]` at 5.0, `[-0.02 +0.29 +1.15 +1.75 +1.15]` at
5.5, `[+0.51 +0.81 +1.68 +2.21 +1.20]` at 6.0 and `[+0.84 +1.09 +1.75 +1.87
+0.87]` at 7.0 — band 4 highest every time, and its error still open at the end
(-0.31 deg at 7.0, -0.91 at 6.0). The exception is 6.5, where every band closes to
+-0.06 deg by lap 9 and the profile ends nearly flat
(`[+0.60 +0.69 +0.88 +0.78 +0.33]`). That is NOT simply "no saturation": 7.0 also
never saturates and still lags. What 6.5 has alone is the collapsed pattern — the
kite flying 65 % of a commanded height it therefore has room to track. So the
runaway follows how hard the pattern is to fly, not the wind, and it cannot be
tuned at 6.5 m/s because the failure mode does not appear there.

**A transport failure silently changes the seed.** `SystemError: write: Broken
pipe` on `POST /step` appeared in every run of the scan except the 6.0 m/s repeat
(0, then 3, 4, 3, 2, 4 events), and
`output/awetrim_server.log` records none of them — the server never saw the
request, so it is a stale pooled socket client-side and
`retry_non_idempotent = true` in `examples/awetrim_client.jl` did not cover it.
The run treats it like a solver failure, so `reopt_retry_el_offset` retries it
from guess el 24 deg and usually installs THAT path (the `_2` events at the same
time and length); one at L = 212 m was lost until the next lap instead. So a run
can fly a path from a seed it never asked for, and the events log is the only
place it shows.

## Scored against the pattern it was COMMANDED, the size budget changes sign (2026-08-19)

The entry below measured the size budget against `az_amplitude`/`el_height` of the
STARTUP path, which is what `fig8_metrics` had always been given. That is the
widest and tallest pattern a reel-out ever flies — every re-optimization returns a
smaller one, a third narrower and half as tall by 380 m — so the late laps were
scored against a reach nothing ever asked them to fly. Fixed: `az_center`,
`az_amplitude` and `el_height` now each take either one number or **one per log
sample**, and `examples/simple_opt_reelout.jl` records the geometry it installs and
passes it through. The same flight, rescored (2026-08-19_092659 against _084439,
identical in every flight number — RMS d 1.28 deg, 8 laps, 8631 W):

    criterion          pinned to the startup path      against what was commanded
    azimuth reach +    14.14 vs 13.77   +0.36  (+3 %)  15.16 vs 11.13   +4.03  (+36 %)
    azimuth reach -    15.81 vs 13.77   +2.04  (+15 %) 15.52 vs 11.13   +4.39  (+39 %)
    elevation span     18.53 vs  8.79   +9.75  (+111 %) 8.02 vs  6.51   +1.52  (+23 %)

**Both halves of the old reading were wrong.** The azimuth reach was not nearly
exhausted — the kite flies 95-98 % of the width it is given, where the pinned
score said 71-80 % and failed the run of 2026-08-19_072132 outright. And the
elevation span was not comfortable at +111 %: that number was the pattern's own
DESCENT over the run, since a span taken over the whole settled window has its top
from the first lap and its bottom from the last. Per lap the kite flies 8.0 deg of
a 9.3 deg pattern, and the elevation span is the criterion with the least room.
`traj_opt.el_bias` and `lift_budget` were both flattered by this; what a lift
actually has to spend is 1.5 deg of elevation span, not 0.36 deg of azimuth reach.

**`el_fill` is now measured per lobe-to-lobe excursion, always**, scalar geometry
included, and `el_span_lap` is reported next to `el_span`: the two together say
whether the pattern moved. On a fixed pattern flown perfectly they agree exactly.

**The lap COUNT was affected too, and that is the part to watch.** The excursion
band a crossing must clear is `lap_frac` of the commanded amplitude, so a band
pinned to the startup pattern eventually exceeds the pattern being flown and stops
seeing crossings altogether: on a synthetic run shrinking 1.0 -> 0.3 it counts 4.0
laps where 5.5 were flown. The real reel-out does not shrink far enough to lose
one (8.0 laps either way), but a longer or windier run would, and a lost lap is a
lost re-optimization and a lost learning update.

## The size budget is ONE criterion, and it has 0.3 deg left (2026-08-19)

**Superseded the same day** — every number in this entry is scored against the
STARTUP pattern, and the entry above shows what changes when they are scored
against the pattern actually commanded. The conclusion it draws (azimuth reach is
the binding criterion) is wrong; the block it describes is still what reports the
trade.

`min_span_frac` interacts with every lift, so the run now reports both halves of
that trade: `traj_opt.lift_budget` carries the correction the path in the air
actually carries and the elevation it bought, against the room each of the three
size criteria has left, with `tightest` naming the one the next lift has to spend.
The same numbers print as a line at the end of the run.

On the flown configuration (`el_bias_bins: 5`, `el_offset_lead: 8`, 2026-08-19_084439):

    criterion            flown     required   margin
    azimuth reach +      14.14     13.77      +0.36 deg   (+3 %)
    azimuth reach -      15.81     13.77      +2.04 deg   (+15 %)
    elevation span       18.53      8.79      +9.75 deg   (+111 %)

**The budget is not three numbers, it is one.** The elevation span has more than
twice what it needs and the negative reach has 15 %; everything any lift has to
spend is the 0.36 deg on the POSITIVE azimuth reach. The pattern is flown
asymmetrically — 14.1 deg one side against 15.8 deg the other — and it is the weak
side that gates.

Read across today's runs, as margin against each criterion:

    bins  lead   reach+   reach-   span     verdict
    1     0      -0.09    +1.41    +9.98    FAILED azimuth reach +13.8
    5     0      +0.31    +2.01    +9.78    all 8 passed
    5     8      +0.31    +2.01    +9.68    all 8 passed

(from the archived summaries, so rounded to their 0.1 deg — the in-run block reads
+0.36 where this table reads +0.31.) Two things fall out. The per-band elevation
learner BOUGHT ~0.4 deg of positive reach rather than costing any, which is why
that criterion flipped from FAILED to passed and is a stronger reason to keep it
than anything in its own entry. And the 8 s lead costs 0.1 deg of elevation span
out of 9.7 — nothing, on the axis with room.

**The threshold does not move — the pattern does.** `az_amplitude` is captured from
the STARTUP optimizer path and never updated, while every re-optimization returns a
pattern narrower in azimuth: a third narrower by 380 m. The requirement therefore
stays pinned to the WIDEST pattern the run ever flies, and the late laps are scored
against a reach they were never commanded to fly; the flown number they are
compared with is a mean over the whole settled window, early wide laps included. A
run that fails this criterion by hundredths is therefore not necessarily
under-flying its reference — it may be flying, accurately, a reference the
optimizer made smaller. That is the next thing to fix if 0.36 deg turns out to gate
a lift worth having.

## The 8 s `el_offset_lead` does not circle — it is what DELIVERS the phase-5 lift (2026-08-19)

The `+1.39 turns` that closed `el_offset_lead` are withdrawn. They were measured
under the broken in-air shift gate, where the lift could only reach the kite
through a mid-reel-out install, and they do not reproduce. Re-measured on today's
code (`el_bias_bins: 5`, everything else as flown), two runs each:

    el_offset_lead   phase-5 turns   mean u_s   phase-5 sat   phase-5 el_min   el_mean
    0                -0.642          -2.1 %     0 %           10.31 deg        13.68
    8                -0.638          -2.1 %     0 %           11.75 deg        15.09

Net heading rotation over phase 5 is the SAME to within 0.005 turns, there is no
one-sided steering bias (the old measurement's tell was +6.1 %), and phase 5 never
touches the steering clamp in either. Runs 2026-08-19_083042 and _083528 (lead 8,
bit-identical to each other) against _080350 and _080918 (lead 0).

**What the lead actually buys is the lift itself.** With lead 0 the 1.5 deg
`el_offset_final` is attempted at t = 123.6 s, scored at margin 0.67 against the
0.74 required, and REFUSED — and every retry over the remaining 26 s is refused
too, so phase 5 flies without it. With lead 8 the same 1.5 deg latches at
t = 115.6 s with 19.7 m of reel-out left and blends in at t = 117.1 s at margin
0.83. The difference is `c1_at(phase)`: from phase 5 on the in-air gate scores
with `depower_final`'s `c1`, which is 0.775x the pattern's, and 0.85 x 0.775 = 0.66
is the 0.67 that was refused. The lead's real function is to get the shift in
while the kite still HAS the turn authority the gate demands — not to move the
lift a few seconds earlier for its own sake.

The price is nothing much: power 8642 -> 8631 W (-0.13 %), RMS d 1.25 -> 1.28 deg,
all 8 criteria in both, `margin_final_flown` 0.87 either way. Whole-run minimum
elevation only moves 10.3 -> 10.4 deg because it is set in phase 4; what the lift
raises is phase 5, by 1.44 deg at its lowest point.

`el_offset_lead: 8.0` is the flown value again. Note this is NOT the 2.5 s lead of
the earlier session, which was measured (post-fix) to buy nothing and to fail the
pattern-size criterion: 2.5 s is under one lap and lands the shift inside phase 5
anyway, where the gate has already tightened.

## The elevation bias LEARNS a profile now, and smoothing it costs shape (2026-08-19)

`el_bias` learnt one number per lap from a whole-lap mean, which is the mean of a
profile: the sag measured at 380 m is ~2.3 deg at the lobes against ~0.4 deg at the
crossing, so the rigid shift lifts the crossing by a degree it never needed. The
generalisation asked for in `docs/reelout_next_steps.md` — the same update per
azimuth band — is built: `fcs.el_bias_bins` (`1` = the old scalar learner
exactly), `fcs.el_bias_smooth`, and
`bias_lift`/`smooth_bins`/`azimuth_bin` in `src/figure_eight_controller.jl`.

**FLOWN, twice** (2026-08-19_080350 and _080918, 150 -> 380 m, 6 m/s, no
turbulence, `el_bias_bins: 5`, `el_bias_smooth: 0.25`), against the rigid-shift
baseline of 2026-08-19_072132:

    bins   laps  RMS d   min el   power   sat   centre_to_lobe   margin_final_flown
    1      8.5   1.43    10.1     8625    7 %   0.70             0.83   FAILED azimuth reach +13.8
    5      8.0   1.25    10.3     8642    6 %   0.22             0.87   all 8 passed

The two profile runs are IDENTICAL in every flight number — same optimizer
session, same request sequence, so the server answered the same, and the only
difference is the failure cache saving 75 s of wall time in the second. That is
weaker evidence than two independent runs, not stronger: it says the pipeline is
deterministic here, not that the result is robust to the paths the optimizer
happens to return.

**What it learnt is not the sag.** The converged profile is
`[0.51 0.80 1.64 2.16 1.16]` deg, crossing first: it peaks at the SHOULDER (60-80 %
of the amplitude), not at the lobe, and the lobe band asks for barely more than the
crossing. That is `el_offset_wing` doing its job — the lobes are already lifted
1.5 deg — and the learner filling the gap the fixed profile's ramp leaves between
the crossing and the lobe. The flown droop follows: `centre_to_lobe_deg` 0.70 ->
0.22.

**The shoulder band does not converge, and it is the one to watch.** Bands 1, 2
and 5 close to +-0.15 deg by lap 6; band 4 sits at -0.74 … -1.29 deg for the whole
run while its correction climbs 0.58 -> 2.16 deg. It is not a sag the reference can
fix — it is the shoulder, where the kite is bounded by TURN AUTHORITY and not by
where the reference is, which is exactly the reason the scalar learner averaged a
whole lap in the first place. The per-band learner has no such averaging, so it
integrates against a wall there. Two consequences, both visible in this run:
`reopt` installed 4 paths against the baseline's 6 (the two extra replies were
refused at margin 0.68 and 0.73, against 0.74 required), and the phase-5
`el_offset_final` shift was held back at 0.67. A bump in the reference costs
curvature margin where the azimuth-keyed `el_offset_wing`, which TRANSLATES the
lobe, costs none. Band 4 also sees the fewest samples (106-189 a lap against
300-370 at the lobe), so it is the noisiest band as well as the one that runs away.

The run is still better on every flown metric, so the profile stays on
(`el_bias_bins: 5`). What is open is bounding the shape: a cap on how far a band
may depart from the profile's mean, a lower gain for the bands than for the mean,
or `el_bias_smooth` back up to 0.5 — the offline table below says 0.5 costs 0.11
deg of shape, and this run says an unbounded band costs two installs.

The rest of this entry is the update rule exercised offline against a synthetic
sag, which is what the defaults were chosen from.

**The bands are learnt, not applied.** What goes onto a path is the bands
interpolated with a smoothstep between their CENTRES, held flat outside the outer
two. A staircase over the bands would put a corner in the reference at every band
edge, and a corner is exactly what `el_offset_wing_blend` measured the cost of: a
4 deg ramp moved the tightest turn of the whole pattern off the lobe and onto the
ramp (radius 4.1 -> 1.86 deg, margin 0.94 -> 0.43). The interpolation leaves every
band centre with zero slope, so the profile cannot grow one however the bands move.

**The key is normalized azimuth**, `|az - centre|` over the pattern's own
amplitude, the same key the `droop_profile` diagnostic already pools on. The
pattern shrinks by a third in azimuth over a 150 -> 380 m run, so bands fixed in
degrees would walk out of it — the failure mode `el_offset_wing_mode:
"azimuth_frac"` exists for.

**Smoothing is shape-only, and it costs shape.** `smooth_bins` moves each band a
fraction `lambda` towards the mean of its neighbours and then restores the
profile's mean, so the rigid part of the correction — everything the scalar
learner ever found — is untouched by construction. It still flattens what the
learner settles on, because it damps the shape every lap while the gain only
closes a fraction of it. Iterated to steady state against a noiseless quadratic
sag (0.4 deg at the crossing, 2.3 deg at the lobe, `el_bias_gain` 0.5), as the RMS
the converged profile still leaves:

    bands    lambda 0    0.25    0.5    0.75
    1                            0.67 (one number, any lambda)
    3              0.18    0.37   0.53
    5              0.10    0.21   0.32    0.40
    7              0.07    0.15   0.22

Every entry beats the rigid shift, so the smoothing is affordable at any setting
tried; `0.25` at 5 bands is the default because the table is noiseless and the
flown measurement is not — a band sees a fifth of the samples the scalar learner
had, and a band that jumps on noise is a step the curvature gate refuses, which
delivers nothing at all. Tune it DOWN if the flown per-band profile
(`traj_opt.el_bias.lap_profiles`) is steady lap over lap.

**What the fixed point is.** Unchanged in kind: each band closes on the error
against the OPTIMIZER's curve, i.e. the sag plus the correction the flown path
carries AT THAT AZIMUTH — the profile's interpolated value, not the band's own
number, which is what makes the two agree inside a band the profile ramps across.
A band with no samples in a lap keeps its value and is carried by its neighbours
through the smoothing. `el_offset_wing` and `el_offset_final` still compose with
it: they are in the flown path and out of the error, so the learner converges on
the optimizer's curve plus both.

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

**The margin the fix needs is 3°, measured twice.** The gate reads the REFERENCE
path's lowest point while the criterion scores the FLOWN one, and the kite flies
below its reference near the lobe tips, where the steering is already on the
clamp. Both stage-4 runs of the day put a number on that gap: reference 10.0° ->
flown 7.0° here, and reference 10.0° -> flown 7.3° on the second.
`candidate_elevation_margin` in `data/traj_opt.yaml` is 3.0 to cover both, i.e.
a candidate must clear `fcs.min_elevation + 3.0` before it is installed.

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

### The soft law at 9 vs 10 m/s: it cuts the DUTY CYCLE, not the PEAK (2026-08-26)

The shipped configuration measured at both ends of the soft band, against the
archived hard-limiter scenarios. Like-for-like on each side: same `k_v` from
`winch_kv_table`, `optimize_k_v` off, `first_lap_force_frac` 1.0, `f_max` 8000 N
sent in every request, `force_limit_tau` 2.48 s, 150 -> 380 m. Both runs passed
all 9 criteria.

| reel-out window | 9 m/s soft | 9 m/s hard | 10 m/s soft | 10 m/s hard |
|:--|--:|--:|--:|--:|
| mean force [N] | 7229 | 7811 | 7405 | 7804 |
| 99th pct force [N] | 8731 | 8951 | 11804 | 11791 |
| **peak force [N]** | **9001** | 9418 | **12365** | 12366 |
| % of time over 8000 N | 26.9 | 70.0 | 18.9 | 56.2 |
| **% over the 8400 N rating** | **3.9** | 49.1 | **8.4** | 19.7 |
| measured power [W] | 29899 | 30260 | 37865 | 38942 |
| power cost | **-1.2 %** | — | **-2.8 %** | — |
| RMS d [deg] | 1.66 | 1.61 | 1.88 | 2.08 |
| force crest factor | 1.16 | 1.12 | 1.61 | 1.50 |

**At 9 m/s it is a clear win**: half the reel-out window sat over the winch's
rating before, 3.9 % of it does now, and the peak comes down 417 N, for 1.2 % of
the power. No sign of the limit cycle the first soft measurement found — that is
`force_limit_tau` 2.48 doing its job. Tracking is unchanged.

**At 10 m/s it is half a win.** The duty cycle improves the same way (19.7 % ->
8.4 % over the rating) but **the peak does not move at all: 12365 N against
12366 N**, identical to the newton, and the 99th percentile with it. The power
cost more than doubles. Tracking improves, oddly, but that is not what the law
is for.

**Why the peak survives is `force_limit_tau`, and NOTHING else was binding.**
Both obvious suspects were checked and cleared:

| at the 10 m/s peak | value | limit |
|:--|--:|--:|
| drum acceleration, whole window | 5.06 m/s^2 | 8.0 (`max_acc`) |
| samples at >= 95 % of the accel cap | 0.00 % | |
| commanded `v_set`, whole window | 7.67 m/s | 8.0 (`v_sat`) |
| **commanded `v_set` AT the 12365 N peak** | **3.37 m/s** | |

The law is not clipped, not rate-limited, and not straining — at the peak it asks
for 3.37 m/s, less than half what it commands elsewhere in the same run. It is
BLIND, because `calc_vro_soft` is fed the FILTERED force and the filter deletes
the excursion:

| t = 26.0 s, the peak | |
|:--|--:|
| raw force | 12365 N |
| **filtered force** | **6109 N** |
| attenuation | 6256 N, **51 %** |

At 6109 N a ~3.4 m/s command is the correct answer; the law never sees 12 kN.
Over the whole window the filtered force maxes at 8309 N against the raw 12365,
and crosses `f_high` only 5.8 % of the time against the raw 18.9 %.

The excursion is a SINGLE 1.52 s spike over 10 kN — 1.52 s against a 2.48 s time
constant, so a first-order lag halves it. The hard baseline has the same one
(1.53 s, 2.2 % of the window either way), so the spike belongs to the path, not
to the limiter.

**So this is a filter trade, not a physical limit.** The winch has the authority
to answer the peak; `force_limit_tau` is what stops it being asked to:

- `tau` 1.0 — force controlled, +-1.1 m/s limit cycle at 2.63 s
- `tau` 0.2 — CLOSED, amplified the ring to +-2.12 m/s
- `tau` 2.48 — shipped, no oscillation, and blind to a 1.5 s spike

One first-order lag cannot do both jobs at 2.48 s against a 1.52 s spike. What is
worth trying is a shape that is not a single lag — an ASYMMETRIC filter, fast on
rising force and slow on falling, would see the spike without handing the loop
the phase that made it ring. Untested.

The `f_max`-in-the-request lever (`PlanWinchSpeed.md` stage 2b) still applies to
the path that MAKES the spike, but it is no longer the only option, and the
cheaper experiment is the filter.

### `force_limit` is WIND-DEPENDENT — the soft law is undefined below ~5 m/s (2026-08-25)

The soft limiter cannot be the law at every wind speed, and no amount of tuning
changes that. `force_limit` therefore moved into `data/winch_kv_table.yaml` beside
`kv` and `f_low`, read by `winch_force_limit(v_wind)`: `"hard"` at 3 and 4 m/s,
`"soft"` from 6 up. `data/wc_settings.yaml` ships `"hard"` so a script that never
consults the table gets the safe law rather than one that may stall the winch.

**Why it cannot be flat.** The soft law inverts the optimizer's tension curve,
whose effective floor is `sp(beta*f_low)/beta` — NOT `f_low`:

| `f_low` [N] | 0 | 100 | 200 | 350 | 700 |
|:--|--:|--:|--:|--:|--:|
| floor at `beta` 1e-3 [N] | **693** | 744 | 798 | 883 | 1103 |

Since `sp(0) = ln 2`, the floor is **>= ln(2)/beta = 693 N for ANY `f_low`**. The
whole 3 m/s force range is 587 +- 165 N, i.e. below it, where `calc_vro_soft`
returns 0 and the winch reels IN instead of out.

**Raising `softminus_beta` to move the floor is CLOSED — it breaks the solver.**
The obvious fix (5e-3 puts the floor at 382 N for `f_low` 350) was implemented,
sent, and measured. A/B on one 3 m/s startup request:

| `softminus_beta` | 3 m/s `/step` |
|--:|:--|
| 5e-3 | **422, Max_Iterations_Exceeded at node 0** |
| 2e-3 | **422** |
| 1e-3 | converged |

The softplus smoothing is part of what CONDITIONS the NLP, and 3 m/s is the
marginal case, so sharpening the corner tips it over. The 884 N floor is an
artifact that helps the optimizer converge. Reverted to 1e-3.

**Beta IS now carried on the wire** (plan stage 3), which is what made that
experiment cheap: `WinchParams.softplus_beta`/`softminus_beta` in
`examples/awetrim_client.jl`, sent by `winch_from_wc` from `WCSettings`, and
honoured server-side in `_apply_winch_params` before its two `setdefault(1e-3)`
fallbacks — AWETrim branch `add_winch_beta`, commit `254dd9e`, both fields optional
so omitting them reproduces the old behaviour. The two sides previously agreed only
because both hard-coded 1e-3. **The runtime server must be on that branch**, since
the Julia client now sends the fields unconditionally and a `develop` server
rejects them with a 422.

**3 m/s confirmed on the wind-dependent selection** (`_235108`), back to baseline:

| | this run (`"hard"` from the table) | v03 baseline |
|:--|--:|--:|
| mean force | 583 +- 165 N | 587 +- 165 N |
| power | 553 W | 559 W |
| power ratio | 1.48 | 1.44 |
| `LowerForceController` share | 22.4 % | 20.6 % |
| verdict | all 9 passed, 380 m on `length` | all 9 passed |

Two things this does NOT fix. **5 m/s is untested** and takes the 4 m/s row's
`"hard"` by the step rule — at 4 m/s the mean force (1350 N) sits just above the
1104 N floor where the inverse is steep, and the loop-gain estimate there is 1.16,
the worst of any wind speed, so `"hard"` is the conservative choice rather than a
measured one. And the **1.48 power ratio at 3 m/s is unchanged**: the optimizer
still plans against an 884 N floor the kite never reaches, which is the likeliest
explanation for its bad low-wind prediction and is now known to be unfixable by
beta.

### `force_limit_tau` 1.0 -> 2.48 — the ring is gone, and it was free (2026-08-25)

**This is the entry that settles the soft limiter.** Sweeping the filter UPWARD in
+20 % steps, the direction the failed 0.2 s experiment below pointed at, kills the
limit cycle outright and IMPROVES every other axis at the same time. There was no
force-vs-oscillation trade to pay after all — the oscillation WAS the cost.

All at 9 m/s, no turbulence, `k_v` 0.0408 fixed, `optimize_k_v` off,
`force_limit: "soft"`, 150 -> 380 m, every run reaching 380 m on `length` and
passing all 9 criteria. Mid-run window, 31-34 s of steady reel-out clear of the
entry ring and the soft-stop:

| `tau` [s] | `v_ro` std | mean force | peak (mid) | % over 8400 N | power | archive |
|--:|--:|--:|--:|--:|--:|:--|
| 0.2 | 2.12 | 6327 +- 2075 N | 9900 N | 24.5 % | 27671 W | `_223245` |
| 1.0 | 1.10 | 7459 +- 1076 N | 9024 N | 13.0 % | 29279 W | `_221525` |
| 1.2 | 0.77 | 7679 +- 729 N | 8950 N | 9.5 % | 29381 W | `_223918` |
| 1.44 | 0.65 | 7737 +- 645 N | 8842 N | 6.8 % | 29567 W | `_224133` |
| 1.73 | 0.68 | 7725 +- 658 N | 8692 N | 4.0 % | 29643 W | `_224349` |
| **2.07** | 0.28 | 7803 +- 322 N | **8377 N** | **0.0 %** | 29702 W | `_224650` |
| **2.48** | 0.19 | 7803 +- 313 N | 8381 N | **0.0 %** | 29737 W | `_224946` |
| 2.98 | 0.15 | 7818 +- 309 N | 8348 N | **0.0 %** | 29908 W | `_225219` |
| v09, `"hard"` | 0.05 | 8521 +- 188 N | 8771 N | 73.7 % | 30260 W | `scenarios/v09` |

`2.07` is where the steady window stops touching 8400 N at all. Past it the curve
is a PLATEAU, not continued improvement: 2.07 -> 2.48 moves mean force by 0 N and
power by 35 W. **`force_limit_tau: 2.48` is shipped** — middle of the measured good
region rather than its edge, and ~31 % of the ~8 s lap period at 9 m/s. 2.98 is
marginally better on paper (+171 W) and was NOT taken: the plateau's far end is
unmapped, and a filter approaching the lap period stops resolving within-lap force,
at which point the limiter goes inert and the failure is SILENT — it shows only as
mean force creeping back up.

Two things worth keeping:

- **Force std FELL as the filter grew** (1076 -> 313 N), the opposite of the
  prediction that heavy filtering would let force swing free. The limit cycle was
  itself the main source of force variation, so killing the ring removed the
  excursions it was generating. Both sides improved together from 1.44 on.
- **Whole-run peak is pinned at 9001 N and ~4.8 % over 8400 for 2.07, 2.48 AND
  2.98** — identical to the digit. That residual is the ENTRY transient, where the
  limiter is still charging its filter, and it is completely independent of `tau`.
  It is now the binding constraint on peak load, and no value of this parameter
  touches it.

**Confirmed at 6 m/s** (`_225748`, `tau` 2.48), where the soft law is a NO-OP as
predicted: mean force 3451 +- 436 N against v06's 3423 +- 414, power 8672 vs
8647 W, ratio 1.00 both, 100 % speed control, `zeta` 0.113 (no ring), 380 m on
`length`. At 3451 N the operating point is far from both limits (floor 1104 N,
`f_high` 8000 N) so the inverse is essentially the nominal law — computed 2.367 vs
2.387 m/s beforehand, and the run agreed.

**NOT confirmed at 3 m/s, and expected to FAIL there** — see the `f_low` entry
below. The effective floor is 884 N (`f_low` 350 at beta 1e-3) while v03's whole
force range is 587 +- 165 N, so `calc_vro_soft` returns 0 at the mean and the winch
would reel in rather than out. That is a defect of the soft law, not of `tau`, and
no filter value rescues it; it needs `softminus_beta` raised, which needs stage 3
to carry beta on the wire (`WinchParams` is `extra="forbid"` and the server's
`_apply_winch_params` never reads it from the request).

### `force_limit: "soft"` — buys the force, pays a limit cycle (2026-08-25)

**Superseded by the `tau` sweep above** — the limit cycle this entry measures is
gone at `tau` 2.48. Kept because it is the first measurement of the soft law and
because its gain analysis is what explains the sweep.

**Measured at 9 m/s. It works on force and it OSCILLATES.** Sustained overload
collapses; the dead-steady reel-out speed does not survive. Shipped in
`data/wc_settings.yaml` and left ON, because the force result is the larger of the
two effects and the oscillation has an untested cheap fix — but read the numbers
below before comparing any run from this date on against an earlier archived
scenario.

Run `output/archives/2026-08-25_221525`, against `output/scenarios/v09`. Clean
like-for-like: 9 m/s, no turbulence on either side, same `k_v` 0.0408,
`optimize_k_v` off in both, same 150 -> 380 m. The ONLY difference is the limiter.

Mid-run window, 31 s of steady reel-out clear of the entry ring and the
soft-stop — this is where the two laws actually differ:

| | `"soft"` | v09 baseline (`"hard"`, UFC off) |
|:--|--:|--:|
| `v_ro` | 4.21 +- **1.10** m/s (2.05 .. 7.09) | 3.77 +- **0.05** m/s (3.62 .. 3.83) |
| mean force | **7459** +- 1076 N | 8521 +- 188 N |
| % of window over the plant's 8400 N | **13.0 %** | 73.7 % |
| `v_ro` swing period | **2.63 s** | none |

Whole run:

| | `"soft"` | v09 |
|:--|--:|--:|
| peak force | 9166 N | 9418 N |
| % of reeling window over 8400 N | **13.3 %** | 58.1 % |
| measured power | 29279 W | 30260 W |
| power ratio | 0.94 | 0.97 |
| ringing `zeta` | **0.005** | 0.072 |
| criteria | all 9 passed | all 9 passed |

What it buys: the mean force falls 1062 N and time above the winch's RATING drops
from three quarters of the window to an eighth. That is the thing the stage
existed for and it is not a small effect.

What it costs: the baseline held 3.77 +- 0.05 m/s — dead steady — and the soft law
replaces that with a +-1.1 m/s limit cycle at a fixed 2.63 s period running the
WHOLE mid-window. `zeta` 0.005 is undamped, not slowly-settling. Peak force barely
moves (-2.7 %) because the oscillation manufactures its own peaks, and power is
down 3.2 %. Every criterion still passes, because none of them scores this.

Why, from the law's own local gain at the flown operating point:

| force [N] | 6000 | 7000 | **7459** | 7900 | 7990 |
|:--|--:|--:|--:|--:|--:|
| `dv/dF` [(m/s)/kN] | 0.30 | 0.37 | **0.53** | 2.10 | 17.21 |
| nominal `kv*sqrt(F)` | 0.26 | 0.24 | 0.24 | 0.23 | 0.23 |

At 7459 +- 1076 N every lap sweeps from the flat region into the near-vertical one
and back. `PlanWinchSpeed.md`'s own loop-gain table predicted exactly this ("soft
law ... railed" at 9-10 m/s) before the code existed.

Two levers checked and CLOSED for this:

- **`softplus_beta` is not the fix.** A softer corner makes it WORSE at this
  operating point, not better: `dv/dF` at 7459 N goes 0.53 -> 0.85 (5e-4) ->
  1.29 (2.5e-4), because a wider corner starts bending earlier. Sharper keeps the
  curve flat longer but goes vertical closer to `f_high`. Neither helps here, and
  neither can move at all without stage 3.
- **Raising `f_high`** would only move the knee, not remove it, and the plant's
  rating is 8400 N.

`optimize_k_v` stays OFF: this run does not clear it. A limiter that rings at
+-1.1 m/s is not the stable force cap that gate was waiting for.

### `force_limit_tau` 1.0 -> 0.2 is CLOSED — it AMPLIFIES the ring (2026-08-25)

Hypothesis, from the entry above: the 1 s force filter is the phase that closes
the loop, since a first-order 1 s lag is 67 deg at the measured 2.63 s period.
**Wrong, and wrong in a way worth keeping.** Run
`output/archives/2026-08-25_223245`, same 9 m/s / no turbulence / `k_v` 0.0408 /
`optimize_k_v` off as the other two. Mid-run window:

| | `tau` 0.2 | `tau` 1.0 | v09 baseline (`"hard"`) |
|:--|--:|--:|--:|
| `v_ro` | 4.76 +- **2.12** m/s | 4.21 +- 1.10 m/s | 3.77 +- **0.05** m/s |
| mean force | 6327 +- **2075** N | 7459 +- 1076 N | 8521 +- **188** N |
| peak force in window | **9900** N | 9024 N | 8771 N |
| % over the plant's 8400 N | 24.5 % | **13.0 %** | 73.7 % |
| ring period | **3.55 s** | 2.63 s | none |

Whole run: peak force **9900** N (vs 9166 at `tau` 1.0 and 9418 for the baseline —
so this is the worst of the three), `v_ro` peak **8.09 m/s** i.e. hard against
`v_sat`, power **27671 W** (vs 29279 and 30260), ratio 0.90 (vs 0.94, 0.97),
`zeta` **0.001** (vs 0.005, 0.072). All 9 criteria still passed, which by now says
more about the criteria than about the run.

Worse on every axis, and the DIAGNOSTIC is the period: it went UP, 2.63 -> 3.55 s.
If the filter's phase lag were setting the loop, taking lag out would raise the
ring frequency. It fell. So the filter is not the destabilising phase — it is
attenuating loop GAIN at the ring frequency, and removing it handed the loop that
gain back. The 67 deg argument treated the low-pass as pure phase in an otherwise
fixed loop and ignored that it also cuts magnitude, which was the dominant effect.

Reverted to 1.0. Note the mean force falls monotonically as the filter shortens
(8521 -> 7459 -> 6327 N) — the law really is shedding mean force, but it buys that
with a limit cycle whose PEAKS are worse than the steady overload it replaced.
`force_limit_tau` is therefore a real knob trading mean force against oscillation,
just not in the direction the hypothesis assumed.

What that leaves, in order:

1. **`force_limit_tau` LONGER** (2-3 s), the direction the data actually points.
   **DONE — see the sweep entry above. It worked, and `tau` 2.48 is shipped.**
2. **Lower `f_high`** — `PlanWinchSpeed.md` stage 2b item 3, feeding both the
   request and the runtime law through `winch_from_wc` so the two move together.
   NOT self-evidently a fix: the knee sits AT `f_high`, so lowering it moves the
   knee down along with the operating point. It only helps if the optimizer
   re-solves onto a path that pulls enough less that the mean lands below the new
   `f_high` with margin. That is stage 2's premise and it is still untested. NO
   LONGER NEEDED for the steady window, which `tau` 2.48 already holds at 0 % over
   8400 N; it would be aimed at the ENTRY transient (9001 N) instead, which is a
   different problem and probably wants a different lever.

**How it works.** `WCSettings.force_limit` (WinchControllers, new; `"hard"` is the struct default
and reproduces the old behaviour bit for bit) selects HOW the limits are
enforced, orthogonally to `mode`, which selects the law's shape. Under `"soft"`
the reel-out speed comes from `calc_vro_soft`, the exact inverse of the
soft-saturating tension curve the trajectory optimizer already plans against, so
the runtime winch and the optimizer's winch model are one law instead of two.
The `UpperForceController` is held in reset for the whole run — it *is* the
thing this replaces, and it is what `DISABLE_UFC` had been switching off by hand
since it oscillated. `DISABLE_UFC` is gone.

Three consequences worth knowing before reading a summary from this setting:

- `winch_state.upper_force_pct` is **0 by construction**. State 2 cannot occur.
  The `optimize_fig8.jl` side condition built on it is vacuous here.
- The effective force FLOOR is not `f_low`. `1/softminus_beta` = 1000 N is larger
  than `f_low` itself, so the smoothing dominates the limit it smooths: 700 N
  acts as an 1103 N floor, and below that the law commands a standstill and hands
  over to the `LowerForceController` (which stays — the soft law never reels in).
- Above `f_high` the law commands `v_sat` flat out. Measured at 9 m/s the law does
  pull the mean down to 7459 N, BELOW `f_high` — but with a 1076 N std, so it is
  in and out of the rail every lap rather than pinned to it. That is the
  operating-point problem of `PlanWinchSpeed.md` stage 2, which this does not
  solve and was never going to.

The force is low-passed (`force_limit_tau`, 1.0 s) before it is inverted, because
`dv/dF` rises by roughly 50x over the last kN below `f_high` against a per-lap
force std of order 1 kN. That filter is itself the prime suspect for the 2.63 s
limit cycle above — the reason it exists and the reason it may have to shrink are
the same number.

Both `beta`s must equal the server's, which applies `setdefault(1e-3)` and does
not read them off the wire — carrying them is stage 3 and is not done. Both sides
say 1e-3 today; that agreement is a coincidence, not a contract.

### `optimize_k_v` disarms the soft limiter — CLOSED for a sharper reason (2026-08-25)

Re-measured against `force_limit: "soft"` at `tau` 2.48, the limiter this lever was
originally said to be waiting for (`_231835`, 9 m/s, no turbulence, otherwise
identical to the fixed-gain `_224946`). **Still wrong, and the mechanism is worse
than "there is no cap".**

The steady window is indistinguishable from fixed gain — 7841 +- 278 N against
7803 +- 313, peak 8343 vs 8381 N, both 0.0 % over 8400. But the WHOLE-run peak is
**11242 N**, 34 % over the plant's rating, against 9001 N fixed. Power 28999 W
against 29737.

The gain history says exactly where it came from:

| t [s] | L [m] | `k_v` | vs seed |
|--:|--:|--:|--:|
| **0.0** | 150 | **0.02237** | **-45.2 %** |
| 36.8 | 192 | 0.04079 | -0.0 % |
| 47.5 / 57.7 | 239 / 280 | ~0.0408 | -0.0 % |
| 67.9 | 322 | 0.02383 | -41.6 % (REJECTED, still applied) |
| 67.9 | 322 | 0.04077 | -0.1 % (retry) |
| 80.0 | 371 | 0.04079 | -0.0 % |

Peak force lands at **t = 36.0 s with `k_v` = 0.02237 in effect**, reeling at
2.01 m/s; ALL over-limit samples sit in t = 25.2 .. 41.6 s (601 of 740 before
t = 40), i.e. exactly the window flown at the startup gain. Once the gain returns
to the seed the force is controlled for the rest of the run.

**The soft law sheds force by reeling out FASTER, and its entire speed scale is
`k_v*sqrt(T)`.** Halving `k_v` halves the speed the law can command at any force.
So `optimize_k_v` does not merely bypass the cap — it turns down the gain of the
loop that enforces it, precisely when the solver has decided to sit on the cap.
That is a stronger objection than the original one below, and it does not go away
by making the limiter better.

Reopening this needs BOTH: the read-backs moved behind the gates (stage 2b item 2
— the t = 67.9 s row above is a rejected candidate that still moved the gain, and
`input_depower` has the same bug), and a bracket that cannot halve the limiter's
authority.

Confirmed working on that run, for what it is worth: the read-back plumbing and
the `k_v` power-plot panel ran against real replies for the first time, and
`k_v_flown` matched the last history entry.

#### The original closure (no limiter at all)

Letting AWETrim solve for the winch gain (`TrajOptSettings.optimize_k_v`, sent as
`WinchParams.optimize_k_v`, the reply's value assigned to the run's `WCSettings`)
is implemented, tested, and switched **off**. At 9 m/s it drove the force 51 %
over the winch's rating while COSTING power:

| | `optimize_k_v` | v09, fixed `k_v` |
|:--|--:|--:|
| mean force, settled | 9116 +- 1522 N | 8070 +- 866 N |
| mean force, t = 30 .. 43 s | **11680 N** | 8068 N |
| **peak force** | **12684 N** | 9418 N |
| mean `v_ro`, t = 30 .. 43 s | 2.30 m/s | 3.66 m/s |
| measured power | 29581 W | 30260 W |
| verdict | all 9 passed | all 9 passed |

The optimizer wanted `k_v` **halved**, not raised — 0.02265 at the startup solve
and 0.0205 by 35 s, against a 0.0408 seed and a factor-2 bracket floor of 0.0204
it sat on. Its model is `T = (v_r/k_v)^2` soft-capped at `f_max`, so a low gain
looks free: the softplus clamps the nominal force to ~8000 N and it reads a winch
holding 8000 N at low speed. The plant caps nothing (`DISABLE_UFC`), so the force
simply built to 12.7 kN. **Reeling out slower does not shed force, it builds it.**

Every success criterion passed both runs — they score tracking and pattern
geometry, and nothing scores force against the winch's rating. A run can be a
pass and still be one that would break the drum.

Two process notes worth more than the result:

- **The first attempt measured a no-op and looked like a clean negative.** The
  gain was written to `wc.kv`, but `DISABLE_UFC` builds the controller from
  `deepcopy(rcs)` and `calc_vro` reads the copy, so nothing reached the winch.
  Power and speed came back unchanged, which read as "the optimizer agrees with
  the seed" and was nearly recorded as such. `rc.wcs === wc` was false. When a
  change produces no effect, check that it was applied before concluding it does
  not matter.
- **Reading only the final logged value inverted the conclusion.** `k_v_flown`
  ended at 0.0408, which said "the optimizer left the gain alone"; the per-reply
  `k_v_optimized` trajectory said it had spent half the run at the bracket floor.
  That is why the summary now lists every reply, and why the power figure plots
  `k_v` step-held against its seed.

Re-enable only after stage 4's `force_limit` gives the runtime a real upper
limiter, or paired with an `f_max` low enough that an unlimited plant still stays
under 8400 N. See `PlanWinchSpeed.md`.

### `f_low` 350 -> 700 N — it was sized for a 4000 N winch (2026-08-25)

`f_low = 350` predates this winch. WinchControllers' own module docstring still
describes the plant it was tuned against — "20 kW winch, 4000 N max force" — and
350 N is 8.75 % of that. Against this winch's `f_high` = 8000 N (plant max 8400 N)
the same fraction is 700 N.

The number does not stay local: `winch_from_wc` sends `f_min = wc.f_low` to
AWETrim, which saturates its tension curve with a *softminus* of width `1/beta`
(`softminus_beta` = 1e-3, i.e. 1000 N). Because `1/beta` is LARGER than `f_min`
itself, the smoothing dominates the limit it smooths — the effective floor the
optimizer plans against is `sp(beta*f_min)/beta`, not `f_min`:

| `f_min` | beta=1e-3 | beta=2e-3 | beta=5e-3 |
|--:|--:|--:|--:|
| 350 | 883 (+152 %) | 552 (+58 %) | 382 (+9 %) |
| 700 | 1103 (+58 %) | 810 (+16 %) | **706 (+1 %)** |
| 2000 | 2127 (+6 %) | 2009 (+0 %) | 2000 (+0 %) |

The 3 m/s run is the evidence that this bites: its entire force range
(249 .. 1000 N, mean 585) lies below the 883 N floor, and it is the only wind
speed whose `power_ratio` is badly wrong — 1.44, against 0.97 .. 1.02 for
4 .. 10 m/s. The optimizer's model cannot represent the operating point, so it
does not predict it.

Raising `f_low` alone does NOT close this: at beta = 1e-3, `f_min` = 700 still
gives an 1103 N floor. Sending `softminus_beta` = 5e-3 as well (706 N) needs a
beta field on `WinchParams`, which the contract does not carry — AWETrim applies
`setdefault(1e-3)` to both betas, so the two sides agree today only by
coincidence of defaults. See `PlanWinchSpeed.md` stage 3.

What moves at runtime, all in `examples/simple_opt_reelout.jl`: the
`LowerForceController` setpoint, the `guard_lfc` force floor for phases 0-2, and
the `force_release` blend that bypasses `reelout_softstart` (0 at `f_low`, 1 at
`f_high`). `f_reelin` (700 N, unused here) now equals `f_low`; re-derive it if a
reel-in phase is ever flown.

**Raising `f_low` alone CRASHED 4 m/s — `f_low` was two limits sharing a name.**
The first run after the change diverged at t = 41.5 s (`next_step!` failed). The
cause was not the reel-out limiter but `guard_lfc`, the standalone entry force
floor for phases 0-2, which took its setpoint from `rcs.f_low` and, when active,
drives the drum directly (`v_set = v_guard`, reel-in only). During the depowered
dive the wing pulls a few hundred N and **cannot reach 700 N**, so the guard's
PID never released, wound up, and ran the reel-in away. Same window, v04 archive
(`f_low` = 350, completed) against the failing run:

| t [s] | phase | old F / v_ro | new F / v_ro |
|--:|--:|--:|--:|
| 18.0 | 1 | 319 N / **-1.87** | 605 N / **-4.82** |
| 20.2 | 2 | 322 N / -2.05 | 617 N / **-5.42** |
| 22 .. 25 | 3 | 119 .. 250 N / +0.7 | **4 .. 10 N** / +1.6 |
| 34.4 | 3 | 590 N / +1.05 | 1500 N / **-6.31** |
| 35.3 | 3 | — | **7912 N** |

The old run's force sits at ~320 N, just under its 350 N setpoint — the guard
idling against a floor it can reach. The new one sits at ~605 N under a 700 N
setpoint it cannot, reels in to 6.3 m/s, slackens the tether to 4 N, and the
kite falls until the tether snaps taut at 7912 N. Reeling in *raises* force at
first (605 > 319), which is why the runaway is stable right up to the point the
kite can no longer follow.

So `f_low` conflated two floors, and only one of them scales with the winch:
the **reel-out** limiter's floor does, the **entry slack-tether guard** is
bounded by what a DEPOWERED wing can pull and does not. Split on 2026-08-25:
`f_low` = 700 keeps the reel-out job, and the guard takes the new
`FC_Settings.entry_f_min` (350 N, `data/fc_settings_reelout.yaml`) in both
`simple_reelout.jl` and `simple_opt_reelout.jl`. The two objects were already
deliberately separate — only the setpoint was shared.

**Measured after the split, and `f_low` is now WIND-DEPENDENT.** 4 m/s passed
and 3 m/s did not, which settled the question of whether one flat floor can
serve the range:

| | 3 m/s, 350 (v03) | 3 m/s, 700 | 4 m/s, 700 | 4 m/s, 350 (v04) |
|:--|--:|--:|--:|--:|
| measured power [W] | 559 | **675** | **2043** | 1926 |
| mean force [N] | 587 +- 165 | 725 +- 129 | 1312 +- 196 | 1296 +- 188 |
| `lower_force_pct` | 20.6 | **63.2** | **1.1** | 3.5 |
| RMS d / max d [deg] | 1.95 / 5.54 | 1.73 / **8.02** | — | — |
| verdict | all 9 passed | **FAILED max d** | all 9 passed | passed |

At 4 m/s the higher floor is plainly right — the limiter share FELL, 3.5 -> 1.1 %,
because the settled force (1312 N) is far above either value and the 3.5 % was
mostly transient. At 3 m/s it is plainly wrong: mean force sits at 725 N against
a 700 N floor, so the limiter, not `kv*sqrt(F)`, is setting the operating point
for 63 % of the run. Tracking failed `max d` by 0.02° while RMS and mean d both
IMPROVED — a single transient excursion, consistent with the reel-out/reel-in
transitions a limiter running two thirds of the time forces on the pattern.

Power at 3 m/s rose 21 % (559 -> 675 W), so the higher floor is not simply worse.
But it buys that in a mode the tuning was never designed around, and the entry
in this log exists to say so rather than to bank the number.

The two things `f_low` sits between scale differently — the winch's rating does
not move with the wind, the kite's force goes with its square — so it moved into
`data/winch_kv_table.yaml` beside `kv` (350 N at 3 m/s, 700 N from 4 m/s up,
interpolated, clamped outside), read by `winch_f_low(v_wind)` and assigned to
`WCSettings.f_low` in `simple_opt_reelout.jl` exactly as `kv` already was. The
`kv` column is unchanged in value at every wind speed. `simple_reelout.jl` still
uses the flat file values for both, as it already did for `kv`.

Confirmed at 3 m/s after the table change: `all 9 passed`, and v03 reproduced
almost exactly — `max d` 5.53° against 5.54°, mean force 582 N against 587,
`lower_force_pct` 22.1 % against 20.6, power 553 W against 559. The archived
`winch_kv_table.yaml` of that run carries `f_low: 350.0` on its 3 m/s row, so
the lookup is doing the work and not a stale flat value.

So the range is now covered at both ends by measurement: 3 m/s passes on 350 N,
4 m/s passes on 700 N, and 5 .. 10 m/s keep the 700 N they were already flying
(`lower_force_pct` 0.0 everywhere above 4, so the floor is not binding there and
the change is inert for them). The one number given up is the 675 W the 3 m/s
run made at 700 N, 21 % above the 553 W it makes at 350 — available only in a
mode where the limiter flies the winch 63 % of the time and `max d` fails. If
that power is ever wanted, the lever is the pattern or `kv`, not the floor.

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

### `EFFECTIVE_SIM_TIME`'s linear wind scaling undershoots below `default_v_wind` (2026-08-20)

`simple_opt_reelout.jl`'s `EFFECTIVE_SIM_TIME` scaled the project's `sim_time`
by `default_v_wind / WIND_SPEED`, assuming reel-out speed scales linearly with
wind. At a 150 -> 380 m, 4 m/s override (225 s from that ratio against a
150 s / 6 m/s baseline) the run reached only 378.2 m, missing `reelout_l_max`
by 1.8 m and failing the "phase 5 (final) reached" criterion outright — the
`el_offset_final` lift latched at `t = 217.9` s with only 7.1 s of the 225 s
budget left. An earlier run at the same override (commit `3ea53e1`, before
`guess_el_center` moved from 26° to 30° in `ad08158`) had reached 380 m, but
with only 12.8 m of margin left when the lift latched — already a near-miss,
not a comfortable pass.

The measured reeling-window duration confirms the linear ratio is the wrong
model, not just unlucky timing: 176.8 s at 4 m/s against the 150 s the ratio
predicts at 6 m/s is an 18% miss
(`ln(176.8/150)/ln(6/4) = 1.35` vs. the ratio's implicit exponent of 1).
Apparent wind during phase 4 was 47.7% below the 27 m/s reference at 4 m/s,
worse than the 20.3% deviation at 6 m/s — it falls off faster than the ambient
wind ratio because apparent wind is dominated by the kite's own cross-wind
flight speed, not ambient wind directly.

At 3 m/s the effective problem is worse than a bad exponent: the winch spends
38.8% of the reeling window in the `LowerForceController` (reeling IN, state 0)
rather than reeling out — tether force averages 486 N against `f_low = 350 N`,
close enough to the floor that force dips trigger reel-in cycles that undo
progress. A 300 s budget (already 2x the plain-ratio baseline) only reached
264.6 m of 380 m, achieving 114.6 m of reel-out over that window — an average
rate of 0.45 m/s including the stalls, against `steady_v_reelout_m_s` of
0.81 m/s when not reeling in. That is a force-floor/winch-tuning problem — see
`winch_kv_table.yaml`, itself tuned only down to 9 m/s, and `f_low` in
`data/wc_settings.yaml` — not a timing one, and fixing it properly needs its
own investigation, deliberately left alone here (2026-08-20: don't touch
`winch_kv_table.yaml`).

Fix, scoped to `EFFECTIVE_SIM_TIME` only: `simple_opt_reelout.jl` raises the
ratio to `BELOW_DEFAULT_EXPONENT = 2.0` when `WIND_SPEED < default_v_wind`.
1.5 (a small margin over the measured 1.35 exponent from the 4 m/s data) was
tried first but is not enough at 3 m/s: extrapolating the 0.45 m/s average
rate over the full 230 m reel-out needs roughly 510 s of reeling alone, well
past the 424 s that exponent 1.5 gives (`150 s * 2^1.5`). Exponent 2.0 gives
600 s at 3 m/s (`150 s * 2^2`) and 337.5 s at 4 m/s (`150 s * 1.5^2`) — both
comfortably above what was measured or extrapolated, at the cost of being
generous rather than tight. At/above `default_v_wind` the plain ratio is
unchanged since it already carries 9-30 s of margin there (measured at
6-10 m/s, all passing). The extrapolation is untested at the full budget —
the underlying LowerForceController stalling means more sim_time should let
3 m/s finish, but does not make the run efficient, and a longer stall episode
than the one measured could still leave it short.
