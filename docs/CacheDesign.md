# Optimizer cache design

`examples/simple_opt_reelout.jl` asks the AWETrim server for optimized flight paths: once
at startup, and repeatedly during the reel-out as the tether grows. A converged solve
takes seconds, and a failed one runs to IPOPT's iteration cap (about 81 s measured). In
`reopt_blocking` mode the simulation is frozen for all of it. A rerun of the same
scenario sends the same requests and gets the same answers, so two caches keep it from
paying for them again:

| Cache | Stores | Keyed by | Where | Setting |
|---|---|---|---|---|
| Failure cache | cold requests that failed | the request (`InitParams`) | `output/opt_failure_cache.yaml` | `opt_failure_cache` |
| Solution cache | applied results, and failed **warm** steps | the request **chain** | `output/opt_chain_cache/*.json` | `opt_success_cache` (failed warm steps: `opt_failure_cache`) |

Both settings are in `data/traj_opt.yaml` and default to `true`. The code is in
[`examples/awetrim_client.jl`](../examples/awetrim_client.jl): the failure cache near
[`OPT_FAILURE_CACHE`](../examples/awetrim_client.jl#L382), the solution cache from
[`OPT_CHAIN_CACHE`](../examples/awetrim_client.jl#L807) on.

## The failure cache (unchanged)

A failed cold request (`/init` followed by `/step`) is recorded under
[`opt_request_key`](../examples/awetrim_client.jl#L409): a hash of every `InitParams`
field except `name`. The script checks it before sending a cold request and skips the
request if it is listed. The tether length is keyed exactly, not bucketed, because
failures are isolated pockets in length (180.0 m converges where 180.00027 m fails).

This works because a cold request depends only on itself. A warm step does not, which
is why the solution cache needs a different key.

## Why a result needs a chain key

The server holds one session. `/init` builds it; every `/step` then solves **from the
optimum the session currently holds** (IPOPT's warm start), after applying the step's
own length, winch and limits to the session. Two things follow from reading
`AWETrim/src/awetrim/server/session.py`:

1. The result of a warm step depends on every request since the last `/init`, not on
   the step alone. A step with `min_turn_radius = nothing` means "keep what the session
   has", so even its constraints come from earlier requests.
2. A step changes the session config **even when its solve fails**, and the solver's
   last iterate is not something the client can see or restore.

AWETrim has no endpoint to load a state. So a result served from the cache leaves the
server behind: it does not hold the optimum the client now believes in, and the next
warm step, sent as-is, would be solved from the wrong state.

The design therefore:

- keys every step by its **whole lineage**, so equal keys mean equal history;
- tracks what the server **really** holds separately from where the chain is;
- **rebuilds** the server's state before sending a request when the two differ.

## Keys

- A chain starts at `/init`. Its key is `opt_request_key(p)`, shared with the failure
  cache.
- Every step's key is
  [`chain_key`](../examples/awetrim_client.jl#L847)`(parent_key, step_fields)`, a hash
  of the parent's key and everything of the step the server sees: the flattened
  `StepParams`, the `inflow_conditions` override and `max_iter`. The `wait` flag is
  transport only and not part of the key.
- A rebuilt state has the key `chain_key(parent_key, "rebuilt")`, or
  `chain_key(parent_key, "rebuild failed")` when the rebuild did not succeed.

Structs are flattened to tuples of field values by `_key_fields` before hashing. The
default `hash` of a struct that holds a `Vector` goes by identity, so two equal requests
would otherwise never share a key. Keys are prefixed with a version (`v3-` for requests,
`c1-` for chain steps), so a change to what goes into a key retires the old entries
instead of misreading them.

`hash` is not stable across Julia versions. An upgrade therefore turns every entry into
a miss, one wasted solve each, and never into a false hit.

## The client object

[`OptChain`](../examples/awetrim_client.jl#L824) is held by the caller (one per run,
`opt_chain` in the script). Its state:

| Field | Meaning |
|---|---|
| `state` | key of the state the next warm step starts from |
| `server` | key of the state the server really holds; `""` = unknown |
| `config` | the session config at `state`: the `/init` params with every step's changes applied |
| `table` | the `/trajectory` payload at `state`, as the server would serve it |
| `current` | entry of the latest step, sent or served |
| `served` | whether `current` came from the cache |
| `pending` | steps of this lineage that were sent but are not stored yet |
| `hits`, `misses`, `rebuilds` | counters, reported in the run summary |

Four functions replace the endpoint calls one for one, and a fifth marks a result as
applied:

| Chain call | Replaces | Behaviour |
|---|---|---|
| [`chain_init`](../examples/awetrim_client.jl#L919) | `opt_init` | starts a new chain; `/init` **always** goes to the server, because it only fits the starting path and the following step needs that fitted path |
| [`chain_step`](../examples/awetrim_client.jl#L943) | `opt_step` | hit: served without the server; miss: rebuild if needed, then send |
| [`chain_status`](../examples/awetrim_client.jl#L1020) | `opt_status` | the known outcome for a served or finished step; otherwise asks the server and records the outcome when the solve finishes |
| [`chain_trajectory`](../examples/awetrim_client.jl#L1042) | `opt_trajectory` | a copy of `table`, so callers that edit it cannot change what gets stored |
| [`record_opt_success!`](../examples/awetrim_client.jl#L1054) | — | the latest result was applied: store it and its lineage |

## Step lifecycle

```
chain_step(sp)
 ├─ look up chain_key(state, sp)
 ├─ not usable, and server ≠ state?
 │     look up chain_key(chain_key(state, "rebuilt"), sp)   ← what an earlier run solved after a rebuild
 ├─ HIT:  state ← key, config updated, nothing sent
 │        converged → return the stored reply (blocking) / 0 (async)
 │        failed    → throw the same HTTP 422 the server would (blocking) / status "failed" (async)
 └─ MISS: server ≠ state → rebuild_session!, recompute the key under the rebuilt lineage
          send the step; state ← server ← key; push the entry onto `pending`
          converged → fetch /trajectory into the entry at once
          422       → mark failed; store it now if `failures`
          other error → server state unknown (server ← ""), entry dropped
```

A stored entry is **usable** if it converged and `successes` is on (a blocking call also
needs the stored blocking reply), or if it failed and `failures` is on.

For a converged step the trajectory is fetched **as soon as the solve finishes**, not
when the script asks for it. A later step overwrites the server's result, and a stored
ancestor without its table could not be replayed.

A cached failure is thrown as a real `HTTP.StatusError` with status 422 and a JSON body
carrying the recorded reason. The script's existing
`exc isa HTTP.StatusError && exc.status == 422` handling, and its
`String(copy(exc.response.body))` diagnostics, therefore work unchanged.

## Rebuilding the server's state

[`rebuild_session!`](../examples/awetrim_client.jl#L1079) runs before a miss whenever
`server ≠ state`:

1. `/init` with the chain's current `config`. When the chain has a converged optimum,
   the init uses that optimum's trajectory (converted from the table's radians to
   degrees) as its `trajectory` and its optimized `input_depower` as the seed.
2. One blocking `/step` at the config's length and winch, which starts from that
   optimum and converges quickly.
3. `state ← server ← chain_key(state, "rebuilt")`.

The rebuilt state is close to the lost one but **not bit-identical**: IPOPT's warm start
(iterate and multipliers) is gone. That is why it gets its own key. Whatever is solved
from it is stored under the rebuilt lineage and never passes for a result of the
original one. On the next identical rerun, `chain_step` finds it under that key (the
second lookup above) and serves it without rebuilding.

A rebuild that fails with an HTTP error is not fatal. The request is then solved from
whatever the server holds, under a `"rebuild failed"` key.

## What is stored, and when

The user-facing rule is that **only applied results count as successes**. A converged
reply that a gate rejects is not stored as a result. The script calls
`record_opt_success!` at the three places a path is applied:

| Place in `simple_opt_reelout.jl` | Applied result |
|---|---|
| [after the startup install](../examples/simple_opt_reelout.jl#L713) | the startup solve |
| [a startup retry that takes over](../examples/simple_opt_reelout.jl#L1026) | the corrected startup path |
| [the re-optimization accept gate](../examples/simple_opt_reelout.jl#L1797) | an installed re-optimization |

`record_opt_success!` writes every converged entry in `pending`, not only the applied
one. Those are the replies the applied result was warm-started from, and a replay
cannot reach the result without them. For example, a startup retry that is rejected
and then warm-starts the retry that takes over is stored together with it. `pending`
is cleared at every `chain_init` and after every record, so a lineage that never leads
to an applied result is not stored.

Failed warm steps are the exception: they are stored **immediately** (under
`opt_failure_cache`). In blocking mode a failed warm step is followed by a cold
fallback, which starts a new chain and would otherwise drop the failure from `pending`.
The rerun would then pay the full failed solve again. Cold failures stay in the YAML
failure cache as before.

## Entry format

One JSON file per key, `output/opt_chain_cache/<key>.json`:

| Field | Content |
|---|---|
| `key`, `parent` | the step's key and its parent's |
| `status` | `"converged"` or `"failed"` |
| `request` | the `StepParams` as sent, for humans |
| `length_m`, `when` | tether length and time of the solve, for humans |
| `table` | the `/trajectory` payload (converged only) |
| `reply` | the raw blocking `/step` reply (blocking converged steps only) |
| `reason` | the server's 422 detail (failed only) |
| `applied` | `true` on the result that was applied |

An entry is written to a temporary file and moved into place, so a parallel sweep
worker never reads half an entry. An unreadable entry is a miss, with a warning.

## Integration in the script

- [`opt_chain`](../examples/simple_opt_reelout.jl#L414) is created right after
  `ensure_server`, from `tos.opt_success_cache` and `tos.opt_failure_cache`.
- Every `opt_init`, `opt_step`, `opt_status` and `opt_trajectory` of the startup solve,
  the startup retries, the re-optimizations and the blend retries goes through the
  chain. The free-speed reference solves in `reelout_results.jl` are unaffected: they
  run after the flight and are not applied.
- The existing cold failure-cache checks before `chain_init` are unchanged.
- The run summary (`reelout_results.jl`) prints and saves `cache_hits`,
  `cache_misses` and `cache_rebuilds`.

## Invalidation

The key covers everything the client sends, but not the server itself: kite, tether,
solver defaults, AWETrim's code. **After any AWETrim change**, clear the cache:

```julia
clear_opt_chain_cache()   # the solution cache; or delete output/opt_chain_cache/
clear_opt_failures()      # the failure cache;  or delete output/opt_failure_cache.yaml
```

Alternatively, set `opt_success_cache: false` to bypass the solution cache without
deleting it. With both settings off, `OptChain` passes every request straight through.

## Costs and limits

- `/init` still reaches the server on every cold request (model build and path fit, no
  IPOPT solve).
- A rebuild costs one short blocking solve, and it holds the simulation even when
  `reopt_blocking` is off.
- A run that deviates from a cached one hits up to the deviation, rebuilds once, and
  solves from there. The first rerun of that new path then hits everything.
- A converged reply that was rejected and is not an ancestor of an applied result is
  solved again on every rerun, by design.
- Only `simple_opt_reelout.jl` uses the chain. The other examples call the endpoints
  directly.

## Verification

The chain was checked against a mock server that derives each optimum from the
session's history and counts solves. The scenario was: startup, a warm async step, a
warm failure, a cold fallback, then a blocking step that fails.

| Run | Solves | Notes |
|---|---|---|
| first run | 5 | everything sent |
| identical rerun | 0 | same optima, cached 422 thrown |
| rerun that deviates at the second step | 3 | one rebuild |
| identical rerun of the deviating run | 0 | found under the rebuilt lineage |
| `successes = false` | 3 | only the failures served |

It has not yet been exercised against the real server in a full run.
