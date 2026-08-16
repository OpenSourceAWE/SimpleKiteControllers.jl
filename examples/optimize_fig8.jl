# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Parallel sweep of the figure-of-eight SHAPE: fly `simple_reelout.jl` once per
(`f8_a`, `f8_b`) grid point, several simulations at a time in separate Julia
processes, and rank the shapes by the mean reel-out power they harvest.

    include("examples/optimize_fig8.jl")

# What is swept, and what is not

`data/optimization.yaml` defines the grid — `min_f8_a` … `max_f8_a` and
`min_f8_b` … `max_f8_b` in `step_size` degree increments — and nothing else about
a run. The plant, the tether length, the wind, the turbulence and the whole
controller tuning come from the selected system project exactly as they would for
an interactive run, so the sweep compares SHAPES against one fixed set of
conditions. Change the conditions and the ranking is about a different question;
that is what the `project`, `sim_time` and `turbulence` lines of the banner below
are for.

# Objective and side conditions

Ranked by `mean_power_W`, the mean mechanical power over the reel-out window. Two side conditions come first, and a run that breaks either is reported
but never ranked (`data/optimization.yaml` holds both limits):

  - **The upper force controller must never engage.** Its share of the reel-out
    window is `reelout.winch_state.upper_force_pct` in the run summary. A shape
    that pulls harder than `f_high` (`data/wc_settings.yaml`) has its power set by the force cap,
    so what would be ranked is the limiter, not the shape.
  - **`pct_time_within_2pct_of_peak` ≤ 5 %.** The share of the settled window the
    steering command spends within 2 % of its own peak, i.e. clamped against
    `max_steering`. A shape only flyable at full steering has no margin left for
    a gust.

A run that never reeled out, or that stopped early, is likewise recorded and not
ranked — see `status` in the results table.

# How the parallelism works

`max_processes` workers (`examples/optimize_fig8_worker.jl`) each run one
simulation at a time in their own OS process, pulling the next grid point from a
shared claim file and appending their metrics to
`output/optimization_results.yaml`. Both operations happen under one lock
(`SimpleKiteControllers.with_file_lock`, an atomic `mkdir`), so no combination is
flown twice and no two entries interleave. Each worker keeps its logs in its own
`output/optimization/w<id>/`, since the output file names come from the project
and would otherwise collide.

The FIRST worker runs alone until it has finished one grid point. That is not
politeness: on a cold checkout the settled-geometry and model caches do not exist
yet, and six processes building them into the same files at once would corrupt
them. Once one run has completed, the caches are there and read-only for the
rest.

The results table is also the resume state. Interrupt the sweep and re-run this
script and it picks up the combinations that have no entry yet; delete
`output/optimization_results.yaml` to start over.

# Cost, and a smoke test first

One run is a full `simple_reelout.jl` — a couple of minutes of wall time — and
the grid is `((max_f8_a - min_f8_a)/step + 1) * ((max_f8_b - min_f8_b)/step + 1)`
of them, 55 at the shipped settings. Each worker holds its own kite model, so
`max_processes` should be no larger than what the machine has RAM and cores for;
the banner prints both so an over-subscribed sweep is visible before it starts.

Before committing hours, fly two points: set `max_f8_a: 21.0` and
`max_f8_b: 8.0` in `data/optimization.yaml` (a 2 x 1 grid) with
`max_processes: 2`, run this script, and check that
`output/optimization_results.yaml` has two entries whose `mean_power_W` and
`pct_time_upper_force` are filled in. Then restore the bounds and delete the
results file.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using SimpleKiteControllers
# `Settings` for the plant's starting tether length, which the pre-flight
# curvature check needs. Named explicitly rather than pulled in wholesale: this
# script deliberately does not load the kite model, only what reads its config.
using KiteUtils: Settings
using Printf
import Dates

# Also creates data/gui.yaml from its .default if missing — done HERE, once,
# rather than by six workers racing to copy the same file.
include(joinpath(@__DIR__, "gui_state.jl"))

const OUTPUT_ROOT = normpath(joinpath(@__DIR__, "..", "output"))
const WORKER_SCRIPT = joinpath(@__DIR__, "optimize_fig8_worker.jl")

os = OptSettings("optimization.yaml")
full_grid = opt_grid(os)
results_path = init_results_file(joinpath(OUTPUT_ROOT, os.results_file))
work_root = joinpath(OUTPUT_ROOT, "optimization")
mkpath(work_root)

# ==================== PRE-FLIGHT ==================== #

# The shapes are filtered on curvature BEFORE anything is flown: a pattern
# tighter than the kite's minimum turn radius does not measure the shape, it
# measures the steering clamp, and takes a full simulation to say so. The
# geometry says it in microseconds. Same settings the workers will fly, read the
# same way, and at the STARTING tether length, which is the worst case.
sweep_project = project_file(selected_reelout_project())
sweep_fcs = FC_Settings(fc_settings(sweep_project))
l_tether_start = Settings(sweep_project).l_tether
c1_sweep = try
    turn_rate_coeffs(sweep_fcs.body_damping, sweep_fcs.depower_setpoint).c1
catch err
    err isa ArgumentError || rethrow()
    @warn "No turn-rate coefficients for this damping/depower — filtering the grid \
           with the default c1 = $(V3_TURN_RATE_C1) instead.\n$(err.msg)"
    V3_TURN_RATE_C1
end
grid, too_tight = filter_grid(full_grid, sweep_fcs, l_tether_start;
                              c1 = c1_sweep, min_margin = os.min_feasibility_margin)

# Recorded, not merely skipped: the table then covers every combination that was
# asked for, and says why the missing ones were not flown.
if !isempty(too_tight)
    already = Set(task_key(r["f8_a"], r["f8_b"]) for r in load_results(results_path))
    for (task, margin) in too_tight
        task_key(task.f8_a, task.f8_b) in already && continue
        entry = run_metrics("", task.f8_a, task.f8_b;
                            status = @sprintf("skipped: pattern too tight (curvature \
                                               margin %.2f < %.2f at %.0f m)",
                                              margin, os.min_feasibility_margin,
                                              l_tether_start))
        push!(entry, "feasibility_margin" => round(margin; digits = 3))
        record_result!(results_path, entry)
    end
    @info "Skipped $(length(too_tight)) of $(length(full_grid)) shapes as tighter than \
           the kite can turn (margin < $(os.min_feasibility_margin)): " *
          join((@sprintf("%.0f/%.0f (%.2f)", t.f8_a, t.f8_b, m) for (t, m) in too_tight), ", ")
end

# ==================== PLAN ==================== #

# Claims of a previous, interrupted sweep belong to processes that are gone; kept,
# they would make their grid points invisible to this one.
orphaned = reset_claims!(grid, results_path)
# Counted over the FLYABLE grid only: the skipped shapes have results too, and
# including them would have the sweep report itself finished before it started.
flyable = Set(task_key(t.f8_a, t.f8_b) for t in grid)
n_done() = count(r -> task_key(r["f8_a"], r["f8_b"]) in flyable,
                 load_results(results_path))
done_at_start = n_done()
gb(bytes) = bytes / 2^30

@info """
Figure-eight shape sweep
  grid          : f8_a $(os.min_f8_a):$(os.step_size):$(os.max_f8_a) deg x \
f8_b $(os.min_f8_b):$(os.step_size):$(os.max_f8_b) deg = $(length(full_grid)) shapes, \
$(length(grid)) to fly$(isempty(too_tight) ? "" : ", $(length(too_tight)) too tight")
  already done  : $done_at_start (resumed from $(basename(results_path))\
$(orphaned > 0 ? ", $orphaned stale claim(s) dropped" : ""))
  processes     : $(os.max_processes) on $(Sys.CPU_THREADS) CPU threads, \
$(round(gb(Sys.free_memory()); digits = 1)) GB of $(round(gb(Sys.total_memory()); digits = 1)) GB free
  project       : $(selected_reelout_project())
  sim_time      : $(something(selected_sim_time(), "project default"))
  turbulence    : $(selected_turbulence())
  objective     : max mean reel-out power
  pre-flight    : curvature margin >= $(os.min_feasibility_margin) at \
$(round(Int, l_tether_start)) m (c1 = $(round(c1_sweep; digits = 4)))
  side cond.    : upper force controller <= $(os.max_pct_time_upper_force) % of the \
reel-out window, steering saturation <= $(os.max_pct_time_within_2pct_of_peak) %
  results       : $results_path
  worker logs   : $(joinpath(work_root, "w<id>.log"))"""

if done_at_start >= length(grid)
    @info "Every grid point already has a result — nothing to fly. Delete \
           $(basename(results_path)) to sweep again."
else

# ==================== WORKERS ==================== #

"""
    launch(id) -> Base.Process

Start worker `id` detached, with its console output going to its own log file.

Single-threaded on purpose, and that takes FOUR settings, not one: the
parallelism here is `max_processes` whole simulations, and a worker that also
spreads itself over the cores only fights the other five for them. `--threads=1`
bounds Julia's own tasks, but the numerical libraries have their own pools:
`OPENBLAS_NUM_THREADS` for OpenBLAS and `MKL_NUM_THREADS`/`OMP_NUM_THREADS` for
MKL, which this stack also loads (`libmkl_rt` + `libiomp5`) and which OpenBLAS's
variable does not reach.

Measured, because the default is a trap. Unpinned, MKL takes the physical core
count — 8 here — so each worker ran 10 OS threads at ~250 % CPU while the model
is far too small for threaded BLAS to pay: one run alone takes 102 s of
simulation whether MKL has 8 threads or 1. Six such workers saturated all 16
hardware threads and had not finished a single 128 s run in eight minutes, i.e.
six processes delivering well under twice the throughput of one. Intel's OpenMP
spin-waits by default, so the cost is real even when the threads have nothing to
do.
"""
function launch(id)
    logfile = joinpath(work_root, "w$(id).log")
    args = ["--project=$(@__DIR__)", "--startup-file=no", "--threads=1",
            WORKER_SCRIPT, string(id)]
    cmd = addenv(`$(Base.julia_cmd()) $args`,
                 "OPENBLAS_NUM_THREADS" => "1",
                 "MKL_NUM_THREADS" => "1",
                 "OMP_NUM_THREADS" => "1")
    return run(pipeline(cmd; stdout = logfile, stderr = logfile, append = true);
               wait = false)
end

alive(procs) = count(Base.process_running, procs)
stop_all(procs) = foreach(p -> Base.process_running(p) && kill(p), procs)

workers = Base.Process[]
t_start = time()
interrupted = false
aborted = false

try
    # ---- Warm-up: one worker alone until the caches exist (see the docstring).
    @info "Starting worker 1 alone to build the model and settling caches …"
    push!(workers, launch(1))
    while n_done() == done_at_start && Base.process_running(workers[1])
        sleep(5)
    end
    first_result = n_done() > done_at_start ? last(load_results(results_path)) : nothing
    log1 = joinpath(work_root, "w1.log")
    if isnothing(first_result)
        global aborted = true
        @error "Worker 1 exited without recording a result. See $log1."
    elseif first_result["status"] != "ok"
        # Every worker would now fail the same way; stop while it costs one run.
        global aborted = true
        @error "The first run did not complete, so the sweep stops here rather than \
                repeating the failure $(length(grid)) times. Status: \
                $(first_result["status"]). See $log1."
    end
    if aborted
        stop_all(workers)
    else
        @info @sprintf("First run done in %.0f s; starting the remaining %d worker(s).",
                       time() - t_start, os.max_processes - 1)
        for id in 2:os.max_processes
            push!(workers, launch(id))
            # A second or two apart: they all read the same caches on startup.
            sleep(2)
        end

        # ---- Monitor: report progress, restart what dies while work is left.
        t_pool = time()
        done_at_pool = n_done()
        last_report = time()
        while alive(workers) > 0
            sleep(5)
            done = n_done()
            if time() - last_report >= 30
                last_report = time()
                rate = (done - done_at_pool) / max(time() - t_pool, 1)
                eta = rate > 0 ? (length(grid) - done) / rate : NaN
                @info @sprintf("%d/%d runs done, %d worker(s) alive, %.0f min elapsed%s",
                               done, length(grid), alive(workers), (time() - t_start) / 60,
                               isnan(eta) ? "" : @sprintf(", ~%.0f min left", eta / 60))
                # Same reason as the worker's `say`: this script is also run with
                # its output redirected to a file, where Julia buffers by block.
                flush(stderr)
            end
            # A worker that hit max_runs_per_process, crashed or was killed: release
            # ITS unfinished claim, so that grid point is flown by somebody, and
            # restart it only if a point is actually waiting. A worker also exits
            # when the last points are already being flown by its colleagues, and
            # restarting it then would just have it exit again — while releasing
            # every unfinished claim would hand a running colleague's point out a
            # second time, which is how this loop once produced a duplicate run.
            for (i, p) in enumerate(workers)
                Base.process_running(p) && continue
                released = release_claims!(results_path, i)
                if n_unclaimed(grid, results_path) > 0
                    @info "Worker $i is gone ($released claim(s) released); restarting it."
                    workers[i] = launch(i)
                end
            end
        end
    end
catch exc
    exc isa InterruptException || rethrow()
    global interrupted = true
    @warn "Interrupted — stopping the workers. The results recorded so far are kept; \
           re-run this script to continue where it stopped."
    stop_all(workers)
end

# ==================== RESULTS ==================== #

outcome = interrupted ? "interrupted" : aborted ? "aborted" : "finished"
results = load_results(results_path)
report = format_results_table(results, os)
println("\n", report)

report_path = joinpath(OUTPUT_ROOT, "optimization_report.txt")
open(report_path, "w") do io
    println(io, "Figure-eight shape sweep, ",
            Dates.format(Dates.now(), "yyyy-mm-dd HH:MM:SS"),
            outcome == "finished" ? "" : " ($(uppercase(outcome)))")
    println(io, "Project $(selected_reelout_project()), turbulence \
                 $(selected_turbulence()), $(n_done()) of $(length(grid)) shapes flown \
                 ($(length(too_tight)) skipped as too tight).\n")
    println(io, report)
end
@info @sprintf("Sweep %s after %.0f min: %d of %d shapes flown. Report: %s",
               outcome, (time() - t_start) / 60,
               n_done(), length(grid), report_path)

accepted, _ = rank_results(results, os)
if !isempty(accepted)
    best = first(accepted)
    @info @sprintf("Best shape so far: f8_a = %.1f deg, f8_b = %.1f deg at %d W \
                    mean reel-out power.", best["f8_a"], best["f8_b"],
                   best["mean_power_W"])
end

end # if done_at_start >= length(grid)
