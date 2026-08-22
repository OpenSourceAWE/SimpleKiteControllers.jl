# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Replay of a saved `simple_reelout.jl` log in the KiteViewers 3D window. No
simulation runs here — the analogue of KiteViewers' own `park_v3.jl`, but for
the V3 model's topology and this package's project/log conventions instead of
a `v3_segments.csv`.

# Where the pieces come from

The project is resolved exactly like `simple_reelout.jl`:
`selected_reelout_project()` (`data/gui.yaml`, set via `select_project()`, and
falling back to `system_reelout_150m.yaml` whenever the selection is a fig8 one)
names a `system_reelout_*.yaml`, whose `sim_settings` gives `log_file` — the
arrow log this script loads from `output/`. Run `simple_reelout.jl` first so that
log exists.

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
when included, or, with `RECORD_VIDEO = true`, that automatic run is written to
`output/<log_file>.mp4` instead of paced onscreen — `record_video()` can also
be called manually afterwards, any number of times, since it just replays `sl`
again. Above `VIDEO_MAX_FPS` the frames are thinned by an integer factor, which
keeps the encoded speed a true `REPLAY_TIME_LAPSE` rather than encoding a huge
file. `VIDEO_SIZE` fixes the recording's resolution independently of how the
window has been dragged; `nothing` records it as it is.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers; tic()
using V3Kite
using SimpleKiteControllers
using SimpleKiteControllers: project_file   # V3Kite exports a project_file(project, entry) of its own
# Selectively: KiteViewers exports `init` in some versions too, which would clash with V3Kite's.
import KiteViewers
using KiteViewers: Viewer3D, update_segments!, update_status_text!, clear_viewer, stop,
                   set_status, bring_viewer_to_front, on
using Printf
# `GLMakie.save` stays qualified: `save` is a name several packages here export.
import GLMakie
using GLMakie: VideoStream, recordframe!

@info "simple_reelout_play.jl: replaying a saved reel-out log in the KiteViewers 3D window."
toc("Loaded packages in: ")

# ==================== USER PARAMETERS ==================== #

# This package's data/ is the default for config file lookups; the model's is asked for by name.
set_data_path(normpath(joinpath(@__DIR__, "..", "data")))
include(joinpath(@__DIR__, "gui_state.jl"))
include(joinpath(@__DIR__, "v3_segments.jl"))
VIEWER_INTERVAL = 3     # draw every n-th logged row
REPLAY_TIME_LAPSE = 3.0 # 1 = realtime, N = N times faster
RECORD_VIDEO = false     # true: write output/<log_file>.mp4 instead of the automatic replay
VIDEO_MAX_FPS = 60       # frames above this are dropped, at the same playback speed
VIDEO_SIZE = (1820, 1024) # window size to record at, restored after; `nothing` = as it is
VIEWER_SCALE = 0.08     # world -> scene units, as in KiteViewers' park_v3.jl
VIEWER_KITE_SCALE = 3.0 # bridle and wing only: a 5 m wing on a 150 m tether is a dot at 1
VIEWER_PX_PER_UNIT = 2.0 # GLMakie screen supersampling; smooths thin tether/segment cylinders
TEXT_UPDATE_HZ = 15      # cap the on-screen status text to this many refreshes per second

PROJECT = selected_reelout_project() # system_reelout_*.yaml; a fig8 selection falls back to the default
project = project_file(PROJECT)
project_set = Settings(project)
log_name = basename(project_set.log_file)
output_path = normpath(joinpath(@__DIR__, "..", "output"))
@info "simple_reelout_play.jl: project = $PROJECT, log = $log_name."

syslog = load_log(log_name; path = output_path)
sl = syslog.syslog
log_dt = length(sl.time) > 1 ? Float64(sl.time[2] - sl.time[1]) : 1 / project_set.sample_freq

# The status text reads `e_mech` straight off the row, so a log recorded before
# simple_reelout.jl started filling it replays at a flat 0.00 Wh. Rebuild it from
# the same p_mech the viewer shows per frame; an all-zero column is the only case
# where there is nothing to overwrite.
if all(iszero, sl.e_mech)
    p_mech = Float64.(getindex.(sl.winch_force, 1)) .*
             Float64.(getindex.(sl.v_reelout, 1))
    try
        copyto!(sl.e_mech, cumsum(p_mech) .* (log_dt / 3600))
        @info @sprintf("Log carries no e_mech; rebuilt it for the replay (%.1f Wh total).",
                       sl.e_mech[end])
    catch exc
        # A memory-mapped Arrow column is read-only; the replay is still fine.
        @warn "Log carries no e_mech and the column is not writable — the status \
               text will show 0.00 Wh. Re-run simple_reelout.jl to log it." exception=exc
    end
end

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
    replay(; video = nothing)

Play `sl` back in the 3D viewer, from the first row to the last, drawing every
`VIEWER_INTERVAL`-th one and holding each frame for as long as it covers in logged
time divided by `REPLAY_TIME_LAPSE` (1 = realtime). Returns when the log is
exhausted or `viewer.stop` is set by PAUSE/STOP.

`video` is a path to write the same frames to as an MP4 instead of pacing them:
the encoded frame rate carries the playback speed, so the file plays at
`REPLAY_TIME_LAPSE` while the recording itself runs as fast as the window draws.
Use [`record_video`](@ref) rather than this keyword.
"""
function replay(; video = nothing)
    replaying[] = true
    frame_ns = VIEWER_INTERVAL * log_dt / REPLAY_TIME_LAPSE * 1e9
    text_update_ns = round(UInt64, 1e9 / TEXT_UPDATE_HZ)
    last_text_update = zero(UInt64)
    frame = 0
    try
        viewer.stop = false
        clear_viewer(viewer; stop_ = false)
        set_status(viewer, isnothing(video) ? "Replay" : "Recording")
        bring_viewer_to_front()
        # Drawn frames can outrun any sane frame rate, so the video takes every
        # stride-th one — an INTEGER factor, which keeps the encoded rate a true
        # REPLAY_TIME_LAPSE.
        stride = max(1, ceil(Int, 1e9 / frame_ns / VIDEO_MAX_FPS))
        # visible = true: the config is applied to the window that is ALREADY open, and
        # VideoStream's own default (false) would hide it for the rest of the session.
        stream = isnothing(video) ? nothing :
                 VideoStream(viewer.fig; visible = true,
                             framerate = max(1, round(Int, 1e9 / frame_ns / stride)))
        # GC off for the paced playback only: a collection landing inside a frame
        # shows up as a visible stutter. Recording keeps GC on — it is unpaced anyway.
        # Re-enabled in the finally below, so an abort (STOP, a dead viewer, the
        # @error path) cannot leave it off in the REPL.
        isnothing(stream) && GC.enable(false)
        deadline = time_ns() + frame_ns
        for (i, state) in enumerate(sl)
            viewer.stop && break
            if i % VIEWER_INTERVAL == 0 || i == length(sl)
                update_segments!(viewer, state; scale = VIEWER_SCALE,
                                 kite_scale = VIEWER_KITE_SCALE)
                frame += 1
                if isnothing(stream)
                    now = time_ns()
                    if now - last_text_update >= text_update_ns
                        update_status_text!(viewer, state; height = state.Z[1])
                        last_text_update = now
                        yield()
                    end
                    wait_until(deadline; always_sleep = true)
                    deadline = time_ns() + frame_ns
                elseif frame % stride == 0
                    # Recorded video: keep text fresh on every captured frame, not
                    # throttled by TEXT_UPDATE_HZ — the encoded rate is already capped.
                    update_status_text!(viewer, state; height = state.Z[1])
                    recordframe!(stream)
                    yield()
                end
            end
        end
        isnothing(stream) || GLMakie.save(video, stream)
    catch exc
        # An @async task that dies silently would just leave the button dead.
        @error "Replay stopped early" exception=(exc, catch_backtrace())
    finally
        isnothing(video) && GC.enable(true)
        replaying[] = false
        stop(viewer)
    end
end

"""
    record_video(file = joinpath(output_path, log_name * ".mp4"); window_size = VIDEO_SIZE)

Record the replay of `sl` to `file` and return its path. Runs the same loop as
[`replay`](@ref) but unpaced, so it finishes well inside the flown time; the MP4
still plays at `REPLAY_TIME_LAPSE`, because that is what sets the encoded frame
rate. Needs the viewer window open — it captures what is on it, buttons and all —
and refuses while a replay is in flight.

The video is as many pixels as the window's framebuffer, so `window_size` resizes
the window for the recording and puts it back afterwards; `nothing` records it as
it currently is. Sizes are rounded UP to even numbers, which h264 requires. A
display that scales (HiDPI) gives a file that many times larger, since the
framebuffer is what is read.

`RECORD_VIDEO = true` in the USER PARAMETERS section above does this
automatically instead of the plain replay. The first call in a session also
pays for compiling Makie's video path, which is tens of seconds and is not
repeated.
"""
function record_video(file = joinpath(output_path, log_name * ".mp4");
                      window_size = VIDEO_SIZE)
    replaying[] && error("A replay is running — press PAUSE before recording.")
    mkpath(dirname(abspath(file)))
    old_size = size(viewer.fig.scene)
    rec_size = isnothing(window_size) ? old_size : Tuple(2 .* cld.(window_size, 2))
    if rec_size != old_size
        resize!(viewer.fig, rec_size...)
        sleep(0.3) # GLFW resizes on its next poll, and VideoStream fixes the frame size at once
    end
    t_rec = @elapsed try
        replay(; video = file)
    finally
        rec_size == old_size || resize!(viewer.fig, old_size...)
    end
    @info @sprintf("Video: %s at %d x %d (%.1f s of flight recorded in %.1f s).",
                   file, rec_size[1], rec_size[2], sl.time[end] - sl.time[1], t_rec)
    file
end

on(viewer.btn_PLAY.clicks) do _
    # The built-in handler has already toggled `viewer.stop`: clicking RUN during a
    # replay therefore reads as the PAUSE it is labelled, breaking the loop above
    # instead of starting a second playback on top of it.
    replaying[] || @async replay()
end
@info "Press RUN in the viewer to replay at \
       $(REPLAY_TIME_LAPSE == 1 ? "realtime" : "$(REPLAY_TIME_LAPSE)x") speed, \
       or call record_video() to write it to output/$(log_name).mp4."

RECORD_VIDEO ? record_video() : replay()
nothing
