# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Builds a PackageCompiler system image with MakieControlPlots and most of
V3Kite's own dependency chain precompiled for the example scripts.

Measured 2026-08-05 against a `using V3Kite, MakieControlPlots` baseline of
15.2 s (no image) / 1.8 s (V3Kite's own 2.19 GB image): baking in the
solver-stack packages alone (StaticArrays, Rotations, NonlinearSolve, the
OrdinaryDiffEq* packages, SteadyStateDiffEq, SymbolicIndexingInterface) only
reached 12.3 s at 1.57 GB — `--trace-compile` showed almost nothing left to
JIT, so the remaining cost is *loading* `ModelingToolkit` (~5.8 s alone) and
`SymbolicAWEModels` (~5.1 s more on top), neither a dependency of the solver
packages and so not pulled in by `include_transitive_dependencies`. Those two
are also most of what makes V3Kite's own image large, so there is no package
set that is both small and fast. On Windows the full image only fits when it is
compressed (`compress_sysimage`, Julia 1.13+); `SMALL_IMAGE` below drops
SymbolicAWEModels on older Windows Julia versions, trading speed for size.

Neither V3Kite nor RobustKiteControllers goes into the image, so Revise keeps
applying to both packages' sources (`docs/sysimage.md`, "What goes into the
image"). Instead, `RKC_DEPENDENCIES` adds RobustKiteControllers' own
dependencies that the solver stack does not pull in. Packages that load in under
0.2 s without the image (table in `docs/sysimage.md`) are not listed; they still go
in when a listed package depends on them. `INVALIDATING` is the exception: packages
that load fast but, loaded on top of the image, invalidate the GLMakie code its
workload compiled, so that the first plots compile it all again. V3Kite is left to Pkg's ordinary
precompilation, where its `@compile_workload` (see its `src/precompile.jl`) covers
the model.

Run via `bin/create_sys_image`, which stacks a throwaway environment holding
PackageCompiler onto `examples/`'s (PackageCompiler is not a dependency of either)
and moves the resulting image into `bin/`.

The workload traced into the image (via `precompile_execution_file`, in an
isolated process) still runs `init`/`step!` on the V3 model, plus a `save_log`/
`load_log` round trip — `simple_fig8.jl`'s ~10 s Arrow write/read of a
`Logger` — so `KiteUtils` is baked in too and its call path gets exercised the
same way. It then draws what `simple_reelout_plots.jl` draws from that log, in
invisible GLMakie windows: `log_created_at`'s `Arrow.Table` read, a `plotxy`, a
twin-axis `plotx` with LaTeX labels and `build_path3d_figure`'s `Axis3`. Building
the image therefore needs OpenGL, as precompiling GLMakie does anyway.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(@__DIR__)
end

using PackageCompiler
using Base64

# Windows needs more RAM + pagefile than Linux: with two image threads the build reached a
# commit charge of 37.3 GB (AWEMeta RKC1, 2026-10-23)
const EXTRA_RAM_WINDOWS = 5
const MIN_TOTAL_GB = 30 + (Sys.iswindows() ? EXTRA_RAM_WINDOWS : 0)
const MIN_TOTAL_TWO_THREADS_GB = 34 + (Sys.iswindows() ? EXTRA_RAM_WINDOWS : 0)

"""
PowerShell script printing RAM + the size in MB the pagefile can grow to. A pagefile
without a maximum ("System managed", or `MaximumSize` 0) grows up to max(3 × RAM, 4 GB),
but at most 1/8 of its volume.
"""
const PAGEFILE_MAX_PS = raw"""
$cs = Get-CimInstance Win32_ComputerSystem
$ram = [math]::Floor($cs.TotalPhysicalMemory / 1MB)
function AutoMax($drive) {
    $vol = Get-CimInstance Win32_LogicalDisk -Filter "DeviceID='$drive'"
    [math]::Min([math]::Max(3 * $ram, 4096), [math]::Floor($vol.Size / 1MB / 8))
}
$max = 0
if ($cs.AutomaticManagedPagefile) {
    $max = AutoMax $env:SystemDrive
} else {
    foreach ($s in @(Get-CimInstance Win32_PageFileSetting)) {
        if ($s.MaximumSize -gt 0) { $max += $s.MaximumSize } else { $max += AutoMax $s.Name.Substring(0, 2) }
    }
}
$ram + $max
"""

"""
    total_ram_swap_gb()

RAM + swap in GB (1024³ bytes), with swap as defined in `docs/sysimage.md`: `SwapTotal` on
Linux, the size the pagefile can grow to on Windows (its current size if that can't be
determined), and twice the free space of the swap volume minus 5 GB on macOS, which writes
compressed memory to its swap files.
"""
function total_ram_swap_gb()
    if Sys.iswindows()
        # MEMORYSTATUSEX layout (x64): dwLength(u32), dwMemoryLoad(u32), then 7 x UInt64
        buf = zeros(UInt8, 64)
        unsafe_store!(Ptr{UInt32}(pointer(buf)), UInt32(64))
        ok = ccall((:GlobalMemoryStatusEx, "kernel32"), stdcall, Cint, (Ptr{UInt8},), buf)
        ok == 0 && error("GlobalMemoryStatusEx failed")
        current = unsafe_load(Ptr{UInt64}(pointer(buf) + 24)) / 1_073_741_824
        max_gb = try
            # -EncodedCommand (Base64 of UTF-16LE) avoids quoting problems on the command line
            script = Base64.base64encode(reinterpret(UInt8, transcode(UInt16, PAGEFILE_MAX_PS)))
            parse(Float64, strip(read(`powershell.exe -NoProfile -EncodedCommand $script`, String))) / 1024
        catch e
            @warn "Could not determine the maximum pagefile size, using its current size" exception=e
            0.0
        end
        max(current, max_gb)
    elseif Sys.isapple()
        vm = isdir("/System/Volumes/VM") ? "/System/Volumes/VM" : "/"
        swap = 2 * max(0, diskstat(vm).available - 5 * 1_073_741_824)
        (Sys.total_memory() + swap) / 1_073_741_824
    else
        info = read("/proc/meminfo", String)
        memtotal = parse(Int, match(r"MemTotal:\s+(\d+)", info).captures[1])
        swaptotal = parse(Int, match(r"SwapTotal:\s+(\d+)", info).captures[1])
        (memtotal + swaptotal) / 1_048_576
    end
end

"""
    choose_image_threads!()

Applies the Julia-script rules of `docs/sysimage.md`: error below 30 GB of RAM + swap, one
image thread below 34 GB (both `EXTRA_RAM_WINDOWS` higher on Windows), otherwise keep a
`JULIA_IMAGE_THREADS` set by the caller or by `bin/create_sys_image` (which marks its own
choice with `SYSIMAGE_THREADS_AUTO`), else
`clamp(Sys.CPU_THREADS ÷ 2, 1, 16)`. A caller's value is kept in every case.
"""
function choose_image_threads!()
    total = total_ram_swap_gb()
    @info "Total RAM + swap: $(round(total; digits=1)) GB"
    if total < MIN_TOTAL_GB
        error("At least $MIN_TOTAL_GB GB of RAM + swap is needed to create a system image even with " *
              "one thread, but only $(round(total; digits=1)) GB is available. " *
              (Sys.iswindows() ? "Increase the maximum size of the pagefile and retry." :
               Sys.isapple()   ? "Free some disk space and retry." :
                                 "Increase your swap file and retry."))
    end

    auto = get(ENV, "SYSIMAGE_THREADS_AUTO", "false") == "true"
    if haskey(ENV, "JULIA_IMAGE_THREADS") && !auto
        @info "JULIA_IMAGE_THREADS already set to $(ENV["JULIA_IMAGE_THREADS"]), leaving as is"
    elseif total < MIN_TOTAL_TWO_THREADS_GB
        ENV["JULIA_IMAGE_THREADS"] = "1"
        @info "Setting JULIA_IMAGE_THREADS=1 based on $(round(total; digits=1)) GB total memory"
    elseif haskey(ENV, "JULIA_IMAGE_THREADS")
        @info "Using JULIA_IMAGE_THREADS=$(ENV["JULIA_IMAGE_THREADS"]) as chosen by the shell script"
    else
        threads = clamp(Sys.CPU_THREADS ÷ 2, 1, 16)
        ENV["JULIA_IMAGE_THREADS"] = string(threads)
        @info "Setting JULIA_IMAGE_THREADS=$threads based on $(round(total; digits=1)) GB total memory"
    end
end

choose_image_threads!()

workload = tempname() * ".jl"
write(workload, """
    using V3Kite
    using MakieControlPlots
    using KiteUtils

    set_data_path(v3_data_path())
    s = init(9.51, 150.0; system_yaml = "system_reelout.yaml")
    for _ in 1:5
        step!(s; rel_depower = 0.25)
    end

    log_path = mktempdir()
    save_log(s.logger, "precompile_workload"; path = log_path, colmeta = timestamp_colmeta())
    sl = load_log("precompile_workload"; path = log_path).syslog
    log_created_at("precompile_workload"; path = log_path)

    using GLMakie, LaTeXStrings
    GLMakie.activate!(visible = false)
    include($(repr(joinpath(@__DIR__, "plot_pattern_utils.jl"))))
    sl = plain_log(sl)
    rng = 2:length(sl.time)
    t = Float64.(sl.time[rng])
    y = Float64.(sl.var_01[rng])
    plotxy([y, y], [y, y]; xlabel = "azimuth [°]", ylabel = "elevation [°]",
           legend = ["flown", "attractor"], fig = "workload pattern", disp = true)
    display(plotx(t, y, [y, y], (y, y), [y, y];
        xlabel = L"\\mathrm{time}~[\\mathrm{s}]", ysize = 18, legendsize = 16,
        ylabels = [L"d~[°]", L"\\psi~[°]", L"u_{\\mathrm{s}}~[\\%]",
                   [L"l_{\\mathrm{tether}}~[\\mathrm{m}]", L"\\mathrm{cycle}~[-]"]],
        labels = [nothing, [L"\\psi", L"\\chi"], [L"u_{\\mathrm{s}}", L"u_{\\mathrm{s,set}}"],
                  [L"l", L"\\mathrm{cycle}"]],
        fig = "workload time series"))
    fig3, _ = build_path3d_figure(sl, rng)
    screen3 = GLMakie.Screen(title = "workload 3D path")
    display(screen3, fig3)
    GLMakie.colorbuffer(screen3)
    sleep(2)
    GLMakie.closeall()
    """)

const SOLVER_PACKAGES = [:MakieControlPlots, :KiteUtils, :NonlinearSolve, :OrdinaryDiffEqBDF,
    :OrdinaryDiffEqCore, :OrdinaryDiffEqNonlinearSolve, :SteadyStateDiffEq]
const RKC_DEPENDENCIES = [:AtmosphericModels, :ControlSystemsBase, :WinchControllers]
const INVALIDATING = [:HTTP, :DataFrames]
const COMPRESS_IMAGE = Sys.iswindows() && VERSION >= v"1.13"
const SMALL_IMAGE = Sys.iswindows() && !COMPRESS_IMAGE
packages = SMALL_IMAGE ? [SOLVER_PACKAGES; RKC_DEPENDENCIES; INVALIDATING; :ModelingToolkit] :
                         [SOLVER_PACKAGES; RKC_DEPENDENCIES; INVALIDATING; :ModelingToolkit; :SymbolicAWEModels]

@info "Creating sysimage..." packages compress = COMPRESS_IMAGE
try
    PackageCompiler.create_sysimage(
        packages;
        sysimage_path = "kps-image_tmp.so",
        include_transitive_dependencies = true,
        precompile_execution_file = workload,
        # Julia 1.13 otherwise builds for several CPU targets, needing over twice the memory (JuliaLang/julia#63141).
        cpu_target = "native",
        (COMPRESS_IMAGE ? (; compress_sysimage = true) : (;))...,
    )
finally
    rm(workload)
end
