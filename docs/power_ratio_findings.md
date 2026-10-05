# Power ratio: depower re-identification and the model mismatch behind it

Findings of 2026-10-03, from re-identifying the depower conversion
(`examples/identify_depower_conversion.jl`) with Maasvlakte 3.5 and 4 m/s added,
and the follow-up on why those two runs do not fit.

## Summary

- The quadratic depower conversion (`data/depower_conversion.yaml`) is confirmed:
  refitted on the 10 runs it was identified on, slope 0.108 and curvature 0.214
  against 0.1147 and 0.1905 in force, weighted RMS residual 0.0037.
- A **tether-length term** would pay off: `-0.00005 rel_depower/m` × (L − 220 m)
  lowers the RMS residual from 0.0038 to 0.0024. Every run shows the trend,
  0.002–0.004 less depower per path as the tether grows.
- **Maasvlakte 3.5 and 4 m/s do not belong in the depower fit.** The plant flies
  only 77–90 % of the predicted tension there. The cause is the winch model, not
  the depower, see (1) below. Fitted anyway, they wreck the curve (RMS 0.0172,
  negative slope).
- **The power ratio above 1 at low wind is not a controller or solver effect.**
  The plant beats even the planner's free-speed optimum, by 6–8 % at 3.5 m/s
  and 2–3 % at 4 m/s, with cold and warm solves agreeing. The planner's
  quasi-steady model and the dynamic plant **disagree about the aerodynamics at
  every wind speed**: on the same path the simulated kite has a 5–14 % higher glide
  ratio and 11–17 % less force per unit of dynamic pressure, see (3) below.
- Whether that shows up as more or less power depends on which limits are
  active. From about 6 m/s up the reel-out speed is at its 3.5 m/s limit and the
  force near 7 kN, so the better glide ratio cannot be harvested, and the lower
  force coefficient costs power: ratio 0.94–0.97. At low wind no limit is
  active, and the plant harvests more than the planner thinks possible: ratio
  above 1.
- **Adding wing drag to the plant does not close the gap** (2026-10-05, see (4)).
  `wing_drag_coeff = 0.07` on the wing nodes lowers the apparent wind by 14–15 %,
  but it also pitches the wing up by 3.5° of angle of attack, so the force per
  dynamic pressure rises by 40 % and the tension and power do not fall. The power
  ratio at low wind rises, to 1.30 at Maasvlakte 3.5 m/s and 1.11 at Cabauw 4 m/s.
  The drag is kept in the plant; the identified values are being re-identified
  on it.

## (1) Low-wind tension: the planner's soft winch floor

At low force the planner predicts a much slower reel-out than the plant's winch
law gives (path means over each run, laptop base runs):

| run | F predicted → measured [N] | v_r predicted → measured [m/s] | v_r/√F planner / plant |
|---|---|---|---|
| Maasvlakte 3.5 | 1143 → 885 (0.77) | 0.69 → 1.12 (1.63) | 0.020 / 0.038 |
| Maasvlakte 4 | 1433 → 1267 (0.88) | 1.09 → 1.40 (1.28) | 0.029 / 0.039 |
| Cabauw 4 | 3136 → 3059 (0.98) | 2.25 → 2.20 (0.98) | 0.040 / 0.040 |
| Cabauw 6 | 6836 → 6637 (0.97) | 3.46 → 3.36 (0.97) | 0.042 / 0.041 |
| Maasvlakte 10 | 7061 → 6657 (0.94) | 3.50 → 3.35 (0.96) | 0.042 / 0.041 |

The planner's tension curve is `T = softminus_floor(softplus_cap(v²/k_v²))`
(`AWETrim/src/awetrim/system/winch.py`), with the floor adding
`(1/β)·ln(1 + exp(β(f_min − T)))`. The β sent is pinned to 1e-3 per N
(`AWETRIM_SOFTMINUS_BETA`, `src/opt_conditions.jl`), because sharper values make
the low-wind solves fail. A lower `f_min` fails as well. At Maasvlakte 3.5 m/s
(`f_min` = 525 N) and 0.686 m/s the plain law gives 283 N, the floor
283 + 1000·ln(1 + e^0.242) = 1104 N; the planner predicted 1143 N. The plant
clamps hard at `f_low` (`soft_lfc = true`) and follows `v = k_v·√F` above it, so
it reels out 30–60 % faster than planned. The extra force of the floor is about
`1000·e^{0.001(f_min − F)}` N: 600–800 N at 1.1–1.4 kN, about 80 N (3 %) at
3 kN, negligible above 6 kN.

The depower identification compares measured with predicted tension on the
paths as installed, i.e. against this winch model. Below about 1.5 kN that
target is therefore wrong for a winch reason, and those runs have to stay out of
the fit.

## (2) The free-speed optimum is exceeded

`create_overview.jl` reports `power_ratio_free_speed` when a run has it: the
measured power over the planner's optimum with **no winch law** (`winch_mode =
"free_speed"`, the reel-out speed a direct control, the tension hard-bounded to
`[f_min, f_max]`). The 3.5 m/s row of the paper's table (1.10) is that ratio; the
4 m/s row (1.10) is against the winch law, because its ratio was just below the
1.1 trigger.

Free-speed solves per flown path, at the path's own length, pattern box and
minimum turn radius, cold from the startup guess and warm from the flown curve
and its depower:

| run | path | L [m] | winch-law prediction [W] | free-speed cold / warm [W] | measured [W] | measured / free-speed |
|---|---|---|---|---|---|---|
| Maasvlakte 3.5 | 2 | 212 | 778 | 921 / 921 | 980 | 1.064 |
| | 3 | 261 | 847 | 947 / 945 | 1026 | 1.084 |
| | 4 | 319 | 863 | 929 / 944 | 1005 | 1.065 |
| Maasvlakte 4 | 2 | 216 | 1561 | 1731 / 1731 | 1787 | 1.033 |
| | 3 | 264 | 1632 | failed / 1765 | 1820 | 1.031 |
| | 4 | 316 | 1647 | failed / 1706 | 1746 | 1.024 |

Measured is the mean winch power while the path was flown (phase 4, from 3 s
after its install to the next install). Cold and warm agree within 15 W where
both converge, so the reference is not a local optimum. At 4 m/s most cold
solves fail, so the reference a run computes for itself (cold) is unreliable
there.

Ruled out:

- **Wind:** the planner's EXPLOG profile (z0 = 0.0002 m, α = 0.08163) matches the
  plant's logged wind at kite height within 0.05 m/s from 30 to 110 m.
- **Flying lower than planned:** the flown phase-4 elevation (mean 16.0°) follows
  the paths (20.4° falling to 13.6°); the lowest flown point, 10.8°, is just
  below the lowest path point, 11.1°.
- **The reel-out window:** the reel-out starts at 158 m, not 195 m, because the
  low-force controller reels in during the dive; the free-speed probes span the
  right window.

## (3) The aerodynamic mismatch

The planner's trajectory table has no angle of attack or force coefficients,
only kinematics and ground tension. Both sides are therefore compared on the
same kinematic basis: the kite velocity from L, β, φ, their rates and v_r; the
wind at kite height from the plant's log; then

- apparent wind `v_a = |v_w − v_k|` (for the plant this reproduces its logged
  `v_app` to 0.01 m/s),
- effective glide ratio `G = v_t / (v_w cos β cos φ − v_r)`, tether drag included,
- force coefficient `C = F / v_a²` [N s²/m²], ground tension per dynamic pressure.

Path 3 of each run, planner prediction (winch law, as installed) → plant:

| run | G | C | v_a [m/s] | v_r [m/s] | P [W] |
|---|---|---|---|---|---|
| Maasvlakte 3.5 | 3.14 → 3.34 (+6 %) | 8.43 → 7.32 (−13 %) | 11.6 → 10.9 | 0.70 → 1.14 | 847 → 1026 |
| Maasvlakte 4 | 3.26 → 3.63 (+11 %) | 8.79 → 7.48 (−15 %) | 12.7 → 13.0 | 1.10 → 1.42 | 1632 → 1820 |
| Cabauw 4 | 3.55 → 4.04 (+14 %) | 9.14 → 7.61 (−17 %) | 18.0 → 20.0 | 2.19 → 2.26 | 6544 → 6929 |
| Cabauw 6 | 3.72 → 4.01 (+8 %) | 8.83 → 7.41 (−16 %) | 28.0 → 30.1 | 3.48 → 3.45 | 24161 → 23198 |
| Maasvlakte 10 | 3.59 → 3.76 (+5 %) | 7.52 → 6.68 (−11 %) | 30.7 → 31.8 | 3.50 → 3.46 | 24832 → 23397 |

On the same path the plant flies faster, with relatively less drag and less
force per dynamic pressure, at every wind speed. This agrees with the
model-validation result in the paper: the dynamic model underpredicts drag at
higher angles of attack, so the simulated kite flies faster.

Angle of attack: the planner's constraint report gives 5.7–14° at Maasvlakte
3.5 m/s, with the **14° upper bound binding** in both the winch-law and the
free-speed solve, and 8.3–11.7° at Cabauw 6 m/s (not binding). The plant logs
0.6–5.2° and 1.9–2.8°. The 6–9° gap at Cabauw 6 m/s, where the power ratio is
about 1, shows that the two `AoA` values use different references, so they
cannot be compared directly. What does depend on the wind speed: at low wind
both models swing the angle of attack much more over a lap, and only there the
planner runs into its 14° bound. The polars reach maximum lift at 12°.

Why the ratio flips sign: when neither the force limit nor `v_sat` is active,
the power grows roughly with C·G², so the better glide ratio of the simulated kite wins. Above
about 6 m/s both limits are active (v_r at 3.5 m/s, F near 7 kN); the glide
ratio cannot be harvested and the lower force coefficient costs power. The
depower conversion hides part of this: it is calibrated to match the tension, so
it compensates for the lower C with less depower, but it cannot also match G.
The free-speed ratio is therefore probably above 1 at all low and medium wind
speeds, not only at 3.5 m/s. At Cabauw 4 m/s the plant beats its path
prediction by 6 %, but no free-speed reference was computed there because the
run's ratio stayed below the trigger.

## (4) Wing drag in the plant: `wing_drag_coeff = 0.07`

Since 2026-10-05 the plant carries parasitic drag on the wing. `wing_drag_coeff`
in `data/kite_settings_psm_kernel.yaml` and `data/kite_settings_psm.yaml` is 0.07,
the value of V3Kite's flight replays (`kite_settings_psm_replay.yaml`). Before, the
key was 0.0 and inert in this package: V3Kite's `init` reads it but does not apply
it. `apply_wing_drag!` (`examples/model_setup.jl`) now applies it with V3Kite's
`distribute_wing_drag!` after settling, in `init_model` and in the relay flights
of `build_turn_rate_table.jl`: the projected wing area (17.5 m²) is split equally
over the 20 wing nodes, 0.88 m² each, each with a drag coefficient of 0.07. The
kernel backend syncs the point drag and area every step; switching the drag off
at a logged state changes the wing nodes' accelerations by up to 5.3 m/s².

Two runs with the drag, flown with the turn-rate law and depower conversion
identified WITHOUT it, against the base runs of 2026-10-03 (summaries) and the
paper's scenario logs (AoA, pitch; phase 4, elevation below 40°):

| | Maasvlakte 3.5: base → drag | Cabauw 4: base → drag |
|---|---|---|
| v_a [m/s] | 10.77 → 9.14 (−15 %) | 19.99 → 17.19 (−14 %) |
| F [N] | 885 → 891 | 3059 → 3233 (+6 %) |
| C = F/v_a² [N s²/m²] | 7.6 → 10.7 (+40 %) | 7.7 → 10.9 (+43 %) |
| AoA [°] | 3.7 → 7.3 | 3.0 → 6.3 |
| pitch [°] | 8.0 → 12.9 | 6.2 → 10.0 |
| v_r [m/s] | 1.12 → 1.12 | 2.20 → 2.27 |
| P [W] | 1012 → 1021 | 6742 → 7331 (+9 %) |
| power ratio | 1.22 → 1.30 | 1.01 → 1.11 |
| free-speed ratio | 1.10 → 1.11 | – |
| cross-track RMS [°] | 1.01 → 1.37 | 0.88 → 1.29 |

Both runs passed all 10 success criteria. The drag does slow the kite, but the
wing nodes lie behind the pivot of the bridle, so their drag also pitches the wing
up: about 3.5° more angle of attack, and with it a higher lift coefficient. The
kite pulls the same or more tension at a lower apparent wind. The logged
lift/drag ratio falls from about 7.7 to about 5 (the `CL2`/`CD2` columns of the
logs are zero, so the coefficients themselves could not be read).

Against (3): the glide ratio of the simulated kite, 5–14 % above the planner's before, falls by
about the drop in apparent wind and now roughly matches it, while its force
coefficient flips from 11–17 % below the planner's to roughly 20–25 % above. The
low-wind power ratio therefore rises instead of falling. Point drag on the wing
nodes is not a pure drag correction; a correction that lowers only the glide ratio
would have to act without the pitching moment.

Against (1): the tension does not fall, so the soft-floor error stays where it
was (Maasvlakte 3.5 m/s at about 890 N), and Cabauw 4 m/s moves slightly away
from it (3.2 kN).

The cross-track error grows by about 40 %, because the turn-rate law was
identified without the drag. Its re-identification with the drag
(`build_turn_rate_table.jl`, 2026-10-05) is the first step; the depower
conversion, the entry tuning and the scenarios follow.

## Consequences

- **Paper (main.tex:602):** the r_P spread of 0.94–1.10 is mainly this
  aerodynamic mismatch, showing up differently below and above the speed and
  force limits, not controller loss. At the lowest wind speeds the plant
  exceeds even the planner's free-speed optimum. The 3.5 and 4 m/s rows of the
  Maasvlakte table use different references (free speed vs winch law).
- **Depower identification:** Maasvlakte 3.5 and 4 m/s are left out of
  `SCENARIOS`; their runs stay in the record. The tether-length term is not
  implemented yet.
- **The fix is in the models, not in the controller.** Either the planner gets
  the dynamic model's lift and drag (an effective polar fitted from plant logs),
  or the dynamic model's drag is corrected towards the measurements. With
  matching aerodynamics, the free-speed ratio should drop below 1 and the
  depower conversion should become simpler. The first attempt at the second
  route, point drag of 0.07 on the wing nodes, shifts the trim and swaps the
  mismatch in G for one in C, see (4).
- **With the wing drag kept,** every identified value has to be re-identified on
  the new plant (turn-rate law first, then the depower conversion), and the
  results of this document re-checked: they were all obtained without it.
- **The winch floor** remains a separate low-wind error in the winch-law
  predictions, until the planner can solve with a sharp floor or the plant flies
  the planner's soft one (`use_awe_trim = 1`, which first needs the run's
  `f_low` and `f_high` instead of the stale constants `AWE_TRIM_F_MIN = 350` and
  `AWE_TRIM_F_MAX` in `WinchControllers/src/wc_components.jl`).

## Data and reproduction

- **Wing-drag runs (4):** `output/archives/2026-10-05_140859` (Maasvlakte 3.5 m/s)
  and `output/archives/2026-10-05_141248` (Cabauw 4 m/s), on this machine,
  `simple_opt_reelout.jl` without plots at the projects' own settings.

- **Identification record:** `SimulationResults/depower_conversion/2026-10-03/runs/`,
  the 12 base runs (Cabauw 4–10, Maasvlakte 3.5, 4, 8, 10, 11 m/s), each with
  `_opt_paths.yaml`, `_opt_entries.json` and the run summary, 2.6 MB in total. They
  replay without the solution cache.
- **Base runs** were flown on the laptop with the code of 2026-10-03 (commit
  79c74e3) and passed all 10 success criteria. They are not the paper's
  scenarios: runs on different machines agree only for the first 2–4 paths and
  then drift apart by 0.02–2.7°. The paper's 3.5 m/s scenario and the laptop run
  give the same picture (992 W against a free-speed reference of 902 and 906 W).
- **Replay logs** of the identification (36 runs, about 70 MB each) are only in
  `output/depower_conversion/` on the laptop. `fly = false` refits from them.
- **Replaying across machines:** since this change every run writes
  `<log>_opt_entries.json` next to `_opt_paths.yaml`, and `replay_entries` reads it
  before falling back to `output/opt_chain_cache`. The cache is local to each
  machine, so the scenario folders made before 2026-10-03 replay only on the
  machine that flew them.
- The free-speed and kinematic comparisons were ad-hoc scripts on the base runs
  and the AWETrim server (`free_speed` solves through `InitParams`/`chain_step`,
  constraint reports from `output/awetrim_server.log`). `/trajectory?resimulate=true`
  failed for the 3.5 m/s free-speed solve (quasi-steady solve at node 0 did not
  converge) and returned no `angle_of_attack` column for Cabauw 6 m/s.
