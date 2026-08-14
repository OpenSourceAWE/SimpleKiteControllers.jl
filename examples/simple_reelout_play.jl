# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Replay of a saved `simple_reelout.jl` log in the KiteViewers 3D window. No
simulation runs here — the analogue of KiteViewers' own `park_v3.jl`, but for
the V3 model's topology and this package's project/log conventions instead of
a `v3_segments.csv`.

# Where the pieces come from

The project is resolved exactly like `simple_reelout.jl`: `selected_project()`
(`data/gui.yaml`, set via `select_project()`) names a `system*.yaml`, whose
`sim_settings` gives `log_file` — the arrow log this script loads from
`output/`. Run `simple_reelout.jl` first so that log exists.

The point/segment topology comes from `examples/v3_segments.jl`, exactly as in
`simple_fig8_live.jl`, but built the cheap way `examples/export_v3_segments.jl`
does: `load_sys_struct_from_yaml` on V3Kite's `struc_geometry.yaml` is the YAML
half of `init` with no model build, a second instead of minutes. Only the
topology (which points a segment connects) has to match the log; point
POSITIONS come from the log itself on every drawn frame, so the untrimmed YAML
geometry this loads is enough.

# Controls

RUN plays the log back from the start at `REPLAY_TIME_LAPSE`x (1 = realtime);
PAUSE (the same button, relabelled) or STOP ends it. The window is left open
afterwards; clicking RUN again replays from the start. Runs once automatically
when included.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using SimpleKiteControllers
# Selectively: KiteViewers exports `init` in some versions too, which would clash with V3Kite's.
import KiteViewers
using KiteViewers: Viewer3D, update_segments!, update_status_text!, clear_viewer, stop,
                   bring_viewer_to_front, on

@info "simple_reelout_play.jl: replaying a saved reel-out log in the KiteViewers 3D window."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
include(joinpath(@__DIR__, "v3_segments.jl"))
VIEWER_INTERVAL = 3     # draw every n-th logged row
REPLAY_TIME_LAPSE = 2.0 # 1 = realtime, N = N times faster
VIEWER_SCALE = 0.06     # world -> scene units, as in KiteViewers' park_v3.jl
VIEWER_KITE_SCALE = 3.0 # bridle and wing only: a 5 m wing on a 150 m tether is a dot at 1
VIEWER_PX_PER_UNIT = 2.0 # GLMakie screen supersampling; smooths thin tether/segment cylinders
TEXT_UPDATE_HZ = 15      # cap the on-screen status text to this many refreshes per second

PROJECT = selected_project() # e.g. system_reelout_150m.yaml, set via select_project()
project = project_file(PROJECT)
project_set = Settings(project)
log_name = basename(project_set.log_file)
output_path = normpath(joinpath(@__DIR__, "..", "output"))
@info "simple_reelout_play.jl: project = $PROJECT, log = $log_name."

syslog = load_log(log_name; path = output_path)
sl = syslog.syslog
log_dt = length(sl.time) > 1 ? Float64(sl.time[2] - sl.time[1]) : 1 / project_set.sample_freq

# ======================== TOPOLOGY ========================= #

# Cheap-built structure, not a full `init`: only the topology is needed, and it is the
# same whether the geometry is trimmed/settled or not (see the docstring above).
sys_struct = load_sys_struct_from_yaml(joinpath(v3_data_path(), "struc_geometry.yaml");
    set = project_set, aero_mode = SymbolicAWEModels.AeroNone())

# ==================== VIEWER SETUP ========================= #

viewer = Viewer3D(project_set, false; px_per_unit = VIEWER_PX_PER_UNIT)
KiteViewers.init_segments(viewer, segment_matrix(sys_struct,
    Dict("tether" => Int(KiteViewers.TETHER), "bridle" => Int(KiteViewers.BRIDLE),
         "wing" => Int(KiteViewers.WING))))
clear_viewer(viewer; stop_ = false) # stop_ = false: a stopped viewer breaks the loop at once

# Guards against a second replay starting while one is already in flight. `viewer.stop`
# cannot serve as that guard: the viewer's own RUN/PAUSE handler flips it on EVERY click,
# and being wired up in the constructor it always runs before the handler below.
replaying = Ref(false)

"""
    replay()

Play `sl` back in the 3D viewer, from the first row to the last, drawing every
`VIEWER_INTERVAL`-th one and holding each frame for as long as it covers in logged
time divided by `REPLAY_TIME_LAPSE` (1 = realtime). Returns when the log is
exhausted or `viewer.stop` is set by PAUSE/STOP.
"""
function replay()
    replaying[] = true
    frame_ns = VIEWER_INTERVAL * log_dt / REPLAY_TIME_LAPSE * 1e9
    text_update_ns = round(UInt64, 1e9 / TEXT_UPDATE_HZ)
    last_text_update = zero(UInt64)
    try
        viewer.stop = false
        clear_viewer(viewer; stop_ = false)
        bring_viewer_to_front()
        deadline = time_ns() + frame_ns
        for (i, state) in enumerate(sl)
            viewer.stop && break
            if i % VIEWER_INTERVAL == 0 || i == length(sl)
                update_segments!(viewer, state; scale = VIEWER_SCALE,
                                 kite_scale = VIEWER_KITE_SCALE)
                now = time_ns()
                if now - last_text_update >= text_update_ns
                    update_status_text!(viewer, state; height = state.Z[1])
                    last_text_update = now
                    yield()
                end
                wait_until(deadline; always_sleep = true)
                deadline = time_ns() + frame_ns
            end
        end
    catch exc
        # An @async task that dies silently would just leave the button dead.
        @error "Replay stopped early" exception=(exc, catch_backtrace())
    finally
        replaying[] = false
        stop(viewer)
    end
end

on(viewer.btn_PLAY.clicks) do _
    # The built-in handler has already toggled `viewer.stop`: clicking RUN during a
    # replay therefore reads as the PAUSE it is labelled, breaking the loop above
    # instead of starting a second playback on top of it.
    replaying[] || @async replay()
end
@info "Press RUN in the viewer to replay at \
       $(REPLAY_TIME_LAPSE == 1 ? "realtime" : "$(REPLAY_TIME_LAPSE)x") speed."

replay()
nothing
