```@meta
CurrentModule = SimpleKiteControllers
```

# Reel-out runs

The building blocks of `examples/simple_opt_reelout.jl`, in the order a run uses them.

## Running an example script

```@docs
run_example
script_inputs
run_input_defaults
muted
latest_global
first_error_line
```

## Simulated-time budget

```@docs
reelout_budget
sim_budget
BUDGET_HEIGHT
```

## Feasibility gates

```@docs
ReeloutFeasibility
Phase5MarginState
c1_at
phase5_margin
check_reelout_feasibility
check_startup_path
```

## Run state and loop

```@docs
RunSetup
RunState
step_commands!
record_step!
check_overspeed
apply_optimized_kv!
```

## Run log and startup retries

```@docs
with_run_log
startup_ladder_report
ladder_line
startup_log_lines
```
