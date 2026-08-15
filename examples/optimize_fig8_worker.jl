# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
One worker of the figure-of-eight shape sweep. NOT run by hand — it is launched,
`max_processes` at a time, by `examples/optimize_fig8.jl`, which is where the
sweep is documented.

    julia --project=examples examples/optimize_fig8_worker.jl <worker_id>

The worker pulls one (`f8_a`, `f8_b`) grid point at a time from the shared claim
file, flies it by `include`ing `simple_reelout.jl` with `FCS_OVERRIDES` set, and
appends the run's metrics to the results table. Both the claim and the append go
through the file lock in `SimpleKiteControllers.with_file_lock`, so any number of
workers can share one table.

Why a long-lived process pulling tasks, rather than one process per grid point:
loading the packages and building the model costs about a minute, the simulation
itself only a few, so a fresh process per run would spend a third of the sweep on
startup. `max_runs_per_process` in `data/optimization.yaml` bounds how long a
worker lives anyway; the driver restarts it.

Each worker writes its log, run summary and any archive into its OWN
`output/optimization/w<id>/` directory (via `OUTPUT_PATH`), because
`simple_reelout.jl`'s output file names come from the project and would otherwise
be the same file for all of them.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
using Printf
import Dates

const WORKER_ID = isempty(ARGS) ? 1 : parse(Int, ARGS[1])

os = OptSettings("optimization.yaml")
grid = opt_grid(os)

const OUTPUT_ROOT = normpath(joinpath(@__DIR__, "..", "output"))
const RESULTS_PATH = joinpath(OUTPUT_ROOT, os.results_file)
# Private output directory: same file names as an interactive run, own folder.
const WORK_PATH = joinpath(OUTPUT_ROOT, "optimization", "w$(WORKER_ID)")
mkpath(WORK_PATH)

log_stamp() = Dates.format(Dates.now(), "HH:MM:SS")
# Flushed: stdout is a FILE here, which Julia block-buffers, and a progress line
# that only appears when the process exits is not progress. The simulation's own
# chatter in between can stay buffered — it is read after the fact, not watched.
say(msg) = (println("[w$(WORKER_ID) $(log_stamp())] $msg"); flush(stdout))

# The summary of the run just flown, which `run_metrics` reads back. Deleted
# BEFORE each run: if the include throws early, a summary left over from this
# worker's previous grid point would otherwise be recorded as this one's result.
summary_path() = joinpath(WORK_PATH, "reelout_summary.yaml")

say("Worker started, $(length(grid)) grid points to share, results -> $(RESULTS_PATH).")

runs = 0
consecutive_failures = 0

while true
    global runs, consecutive_failures
    if os.max_runs_per_process > 0 && runs >= os.max_runs_per_process
        say("Reached max_runs_per_process = $(os.max_runs_per_process); exiting so the \
             driver can restart me.")
        break
    end
    # Three in a row means the failure is the worker or the config, not the shape:
    # keep claiming and every remaining grid point is burned for nothing.
    if consecutive_failures >= 3
        say("Three consecutive failures — exiting rather than burning the rest of the grid.")
        break
    end

    # The worker id goes into the claim: only the driver, and only for a worker it
    # has seen exit, may release it again (see `release_claims!`).
    task = claim_task!(grid, RESULTS_PATH; worker = WORKER_ID)
    if isnothing(task)
        say("No grid points left. Done after $runs run(s).")
        break
    end
    say(@sprintf("Run %d: f8_a = %.1f deg, f8_b = %.1f deg.", runs + 1, task.f8_a, task.f8_b))

    # Stale outputs of the PREVIOUS run in this worker, cleared so that they can
    # not be mistaken for this run's (see summary_path above).
    for f in readdir(WORK_PATH; join = true)
        endswith(f, ".yaml") && rm(f; force = true)
    end

    t_run = time()
    status = "ok"
    try
        # simple_reelout.jl reads and clears all four, so they are set per run.
        global FCS_OVERRIDES = Dict{Symbol, Any}(:f8_a => task.f8_a, :f8_b => task.f8_b)
        global OUTPUT_PATH = WORK_PATH
        global SHOW_PLOTS = false
        global RUN_ARCHIVE = false
        include(joinpath(@__DIR__, "simple_reelout.jl"))
        # `simple_reelout.jl` catches a diverging solve itself, logs what it has
        # and returns normally, so "no exception" is not "flew the whole run".
        flown = Float64(s.sys_state.time)
        planned = s.steps * s.dt
        if flown < 0.99 * planned
            status = @sprintf("incomplete: stopped at %.1f s of %.1f s", flown, planned)
        end
    catch exc
        exc isa InterruptException && rethrow()
        status = "failed: " * first(sprint(showerror, exc), 200)
        @error "Run failed" f8_a=task.f8_a f8_b=task.f8_b exception=(exc, catch_backtrace())
    end

    # The name simple_reelout.jl actually wrote: <project log_file>.yaml. Found
    # rather than assumed, so a project rename cannot silently blank the results.
    # NOT named `summary`: the included script leaves a global of that name behind,
    # and assigning it here would be an ambiguous soft-scope assignment.
    summaries = filter(f -> endswith(f, ".yaml"), readdir(WORK_PATH; join = true))
    summary_file = isempty(summaries) ? summary_path() : first(summaries)
    entry = run_metrics(summary_file, task.f8_a, task.f8_b; status, worker = WORKER_ID)
    push!(entry, "wall_time_run_s" => round(time() - t_run; digits = 1))
    push!(entry, "finished" => Dates.format(Dates.now(), "yyyy-mm-dd HH:MM:SS"))
    record_result!(RESULTS_PATH, entry)

    power = entry[findfirst(p -> first(p) == "mean_power_W", entry)][2]
    say(@sprintf("Recorded: status = %s, mean power = %s W, %.0f s wall.",
                 status, isnothing(power) ? "-" : string(power), time() - t_run))

    runs += 1
    consecutive_failures = status == "ok" ? 0 : consecutive_failures + 1
    # The previous model is unreachable now; hand its memory back before the next
    # one is built, so six workers do not each keep two models alive.
    GC.gc()
end
