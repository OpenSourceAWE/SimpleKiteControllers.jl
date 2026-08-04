# Where generated `.bin` files are written

V3Kite generates binary artifacts while settling the wing and compiling the
model. Where they land depends on **how V3Kite was obtained**: a development
checkout caches in place, a Pkg-installed copy caches in a depot scratch
directory.

## The rule

`default_cache_path(data_path)` in `src/stabilization.jl` decides:

- `abspath(data_path)` lies under any `DEPOT_PATH/packages/` →
  `DEPOT_PATH[1]/scratchspaces/4caac9c8-c726-438f-ab10-3553e918eab1/v3kite_cache`
  (the UUID is V3Kite's, see `Project.toml`)
- otherwise → `cache_path = data_path`, i.e. caching happens in place

The prefix test joins a trailing separator onto `DEPOT_PATH/packages` so a bare
prefix does not also match a sibling directory such as `packages_old`.

## Why the split

A package directory under `DEPOT_PATH/packages` is not ours to write to:

- it is usually read-only, and
- `Pkg.gc` deletes the whole version tree once no environment references that
  version — taking any cache stored inside it along.

A `scratchspaces` directory keyed by the package UUID survives reinstalling.
A development checkout keeps caching in place, so `] dev` behaves as before and
existing `data/settled_*.bin` files stay in use.

## What is written there

All three generated artifacts go under `cache_path`:

| Artifact | Written by |
| --- | --- |
| `settled_<suffix>.bin` — settled-geometry cache | `settle_wing` |
| settling log | `run_power_zone_settling!` (`log_path = cache_path`) |
| `model_*.bin` — serialized compiled model | `SymbolicAWEModels.init!` |

The model binary only lands in the cache because of `with_model_cache`:
`init!` serializes under `KiteUtils.get_data_path()` and takes no path
argument, so the global data path is temporarily redirected to `cache_path` for
exactly that one write and restored afterwards. The redirect is narrow on
purpose — `init!` reads nothing else through the data path.

## `data_path` vs `cache_path`

- `data_path` — the directory the geometry/settings YAMLs are **read** from
  (default `v3_data_path()`).
- `cache_path` — where everything generated is **written**
  (default `default_cache_path(data_path)`).

Both `init` and `settle_wing` accept `cache_path` explicitly, which overrides
the default entirely. A per-project cache directory keeps one project's
re-settles from invalidating another's.

Redirecting `data_path` as well means that directory must also hold the source
geometry (`struc_geometry.yaml`, `aero_geometry.yaml`, `vsm_settings.yaml`, the
system YAML and the settings file it names), since the settling stage reads all
of them from it.

## When nothing is written

The settling run is skipped — and no file is written — when the destination
`settled_*.bin` already exists and `remake=false` (the default). Pass
`remake=true` to force re-settling and overwrite the cache entry.

Note that the cache file name encodes the settling inputs (depower/steering
tape lengths, tip and trailing-edge reductions, wind speed, tether length,
elevation, gravity, system YAML, KCU mass, and the damping settings), so a
changed input produces a *different* file rather than silently reusing the old
one.

## Unrelated: test temp dirs

`mktempdir()` in `test/test_ripple_metrics.jl` creates an ordinary test scratch
directory and has nothing to do with the caching path described here.
