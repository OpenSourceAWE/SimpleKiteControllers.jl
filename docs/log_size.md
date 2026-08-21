# Why the `.arrow` flight logs are so large

Written 2026-08-21, after `git push` of `SimulationResults` was rejected:

```
remote: error: File scenarios/v03/reelout_150m_opt.arrow is 144.84 MB;
this exceeds GitHub's file size limit of 100.00 MB
```

Reference run: `scenarios/v03/reelout_150m_opt.arrow`, 151,878,498 B (= 144.84 MiB),
produced by `examples/simple_opt_reelout.jl`, summarised in the sibling
`reelout_150m_opt.yaml`: 474.0 s of sim time, `dt = 1/90 s`, **42,662 steps**.

## Where the bytes go

Almost everything is the `X`/`Y`/`Z` particle-position arrays of `KiteUtils.SysState`.
Their length is fixed by `SymbolicAWEModels.position_slots` (`src/symbolic_awe_model.jl`),
which the `Logger(sam, steps)` constructor calls:

| slots | contents | bytes/sample (Float32) | share |
|------:|----------|-----------------------:|------:|
| 44 | structural points — 39 from `V3Kite/data/struc_geometry.yaml` + 5 tether interior nodes (`n_segments: 6`) | 528 | 11.8 % |
| **288** | **VSM panel corners — 4 per panel × `n_panels: 72` (`V3Kite/data/vsm_settings.yaml`)** | **3456** | **77.5 %** |
| 2 | wing origin + the co-located rigid body's origin (one slot and one `orient` frame each) | 24 | 0.5 % |
| — | all remaining `SysState` fields (quaternions, turn rates, wind vectors, forces, moments, `twist_angles`, `flap_angle`, winch set values, `var_01..16`, …) | 454 | 10.2 % |
| | **total** | **4462** | |

4462 B × 42,663 samples ≈ **190 MB raw**. `save_log` defaults to `compress = true`
(LZ4), which recovers only ~20 % on Float32 position data — hence 152 MB on disk.

So: **~90 % of the file is `X`/`Y`/`Z`, and ~78 % of the whole file is VSM panel
corners.**

## What is actually redundant

1. **Panel corners — 78 % of the file.** Written by `write_aero_log_points!`
   (`SymbolicAWEModels/src/aero_modes/common.jl`) purely for visualisation. They are
   fully derived data: recomputable from the structural points, the wing pose, the
   logged `twist_angles`/`flap_angle`, and the static `aero_geometry.yaml`.
   On top of that they are *internally* duplicated — 72 spanwise panels share edges,
   so there are only 73 × 2 = 146 distinct corner positions, stored as 288. ≈2× within
   that block alone.
2. **Unused winch slots — ~1.6 %.** `l_tether`, `v_reelout`, `winch_force`,
   `set_torque`, `set_speed`, `set_force` are each `MVector{4, Float32}`. The V3 has one
   tether, so 18 of those 24 floats are always zero.
3. **`var_01..var_16` — ~1.4 %.** The unassigned generic variables are constant zero.

Items 2 and 3 are noise. The corners are the whole story.

## Options

| change | file size | notes |
|---|---:|---|
| as-is, 90 Hz | 152 MB | |
| resample to 20 Hz | **~34 MB** | 4.5× — the one-line fix, plenty for KiteViewers replay |
| don't log panel corners (46 slots instead of 334), keep 90 Hz | ~33 MB | no signal loss, but needs a flag upstream: `write_aero_log_points!` indexes `X`/`Y`/`Z` unconditionally, so a shorter `Logger` would go out of bounds |
| both | ~8 MB | |

## Doing it after the fact: `examples/compress.jl`

The third row needs no upstream flag as long as the trimming happens *after* the
run: `compress_log(file)` rewrites a saved log without the corner block, in place
or to a new file, with `every = n` to decimate on top. Nothing downstream has to
change — `load_log` takes the position count from the file itself
(`P = length(table.Z[1])`), and `KiteViewers.update_segments!` only reads slots
`1:n_points` — so a trimmed log replays in `simple_reelout_play.jl` unchanged.
The `created_at` stamp and the `var_01..16` column names survive the rewrite.

Measured on the reference run: **152 MB -> 35 MB** (4.3x) in a few seconds, and 11.9 MB
(12.8x) with `every = 3`. The corner block is slots 45..332; the two origin slots
move down to 45..46, which only code recomputing `position_slots` from a live
model would notice.

**Caveat on resampling:** decimating aliases the fast signals — `winch_force` peaks,
`t_sim`, steering. The metrics already recorded in the run's `.yaml` (peak 951 N,
std 195 N, `n_samples: 33211`) were computed at 90 Hz and stay valid, but recomputing
them from a 20 Hz log gives different numbers. Treat a resampled log as a viewing
artefact, not as the scoring input.

## Fixing the rejected push

Shrinking the file on disk does not unblock the push on its own — GitHub rejects the
blob inside the commits being sent, not the working tree. In `SimulationResults` the
offending blob sits in `55f7d4e` ("New results"), which was the only unpushed commit,
so replacing the file and `git commit --amend` is enough. The earlier version of that
path (`2cb4cd6`, 96.8 MB) is already on origin and needs no rewriting.

Also at the edge: `scenarios/v04/reelout_150m_opt.arrow` is 93.9 MB in the same commit.
It passes, but only just.
