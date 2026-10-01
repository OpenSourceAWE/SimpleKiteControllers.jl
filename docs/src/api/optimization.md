```@meta
CurrentModule = SimpleKiteControllers
```

# Shape optimization

The parallel sweep of the figure-of-eight shape driven by `examples/optimize_fig8.jl`.
Worker processes share one results file and claim grid points through a file lock.

## Grid

```@docs
OptSettings
opt_grid
task_key
pattern_margin
filter_grid
```

## Shared results file and claims

```@docs
with_file_lock
init_results_file
record_result!
load_results
claim_task!
release_claims!
reset_claims!
n_unclaimed
```

## Scoring and ranking

```@docs
run_metrics
side_conditions
rank_results
unique_results
format_results_table
```
