# Changelog

## Unreleased

### Changed

- `examples/simple_opt_reelout.jl`: the tether/bridle structural damping
  `damping_per_stiffness` (the keyword default of `init_model` in
  `examples/opt_reelout_lib.jl`) is now 0.002 s instead of 0.001 s. The flight
  barely changes: at Maasvlakte 8.25 and 3.5 m/s all 10 success criteria pass,
  cross-track RMS and mean reel-out power change by less than 2 %; the
  standard deviation of the tether force at 3.5 m/s rises by about 20 %. The
  simulation is faster: 2.77 → 2.19 ms per step at 8.25 m/s (1.26 x) and
  2.87 → 1.75 ms per step at 3.5 m/s (1.64 x), from 3 interleaved replays of
  each setting, spread within 3 %.
