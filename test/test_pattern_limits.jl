# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the pattern box sent to the optimizer (`src/pattern_limits.jl`): the box and
floors of a request, and the edits the startup retries and re-optimizations make to it. Pure
numbers, no optimizer.
"""

using Test
using SimpleKiteControllers
import SimpleKiteControllers: PatternLimits, pattern_limits_from, elevation_min_request,
    elevation_amplitude_max_at, with_elevation_max, with_azimuth_amplitude_min, with_size_box,
    elevation_amplitude

@testset verbose = true "pattern_limits" begin
    # Every side off; each test switches on what it needs.
    off = (; pattern_azimuth_max = 0.0, pattern_elevation_min = 0.0, pattern_elevation_max = 0.0,
           pattern_azimuth_amplitude_min = 0.0, pattern_elevation_amplitude_max = 0.0,
           pattern_elevation_amplitude_max_high = 0.0,
           pattern_elevation_amplitude_max_wind_ref = 10.0, pattern_symmetric = false,
           elevation_min_from_gates = false, candidate_elevation_margin = 2.0, min_height = 0.0)
    sides(b) = Tuple(getfield(b, f) for f in fieldnames(PatternLimits))

    @testset "pattern_limits_from" begin
        # All off leaves the optimizer's own defaults alone.
        @test isnothing(pattern_limits_from(off))
        b = pattern_limits_from(merge(off, (; pattern_azimuth_max = 40, pattern_symmetric = true)))
        @test b.azimuth_max === 40.0
        @test b.symmetric === true
        @test isnothing(b.elevation_min) && isnothing(b.elevation_max)
        # The per-request floor overrides the file's.
        b = pattern_limits_from(merge(off, (; pattern_elevation_min = 10.0)); elevation_min = 25.0)
        @test b.elevation_min == 25.0
    end

    @testset "elevation_amplitude_max_at" begin
        tos = merge(off, (; pattern_elevation_amplitude_max = 8.0,
                          pattern_elevation_amplitude_max_high = 6.0))
        @test elevation_amplitude_max_at(tos, 9.9) == 8.0
        @test elevation_amplitude_max_at(tos, 10.0) == 6.0
        # An unknown wind, or the step disabled, keeps the base cap.
        @test elevation_amplitude_max_at(tos, nothing) == 8.0
        @test elevation_amplitude_max_at(merge(tos, (; pattern_elevation_amplitude_max_high = 0.0)),
                                         20.0) == 8.0
        @test pattern_limits_from(tos; wind_speed = 12.0).elevation_amplitude_max == 6.0
    end

    @testset "elevation_min_request" begin
        fcs = (; min_elevation = 20.0)
        @test isnothing(elevation_min_request(fcs, off, 150.0))
        @test elevation_min_request(fcs, merge(off, (; pattern_elevation_min = 12.0)), 150.0) == 12.0
        gates = merge(off, (; elevation_min_from_gates = true, min_height = 100.0))
        # The clearance floor asind(100/150) = 41.8° beats the elevation gate's 22°...
        @test elevation_min_request(fcs, gates, 150.0) ≈ asind(100 / 150)
        # ...and falls as the tether grows, until the elevation gate's is the higher one.
        @test elevation_min_request(fcs, gates, 400.0) ≈ 22.0
        @test elevation_min_request(fcs, gates, 400.0; extra = 1.5) ≈ 23.5
    end

    @testset "with_one_side" begin
        box = PatternLimits(; azimuth_max = 40.0, elevation_min = 20.0, elevation_max = 50.0,
                            azimuth_amplitude_min = 10.0, elevation_amplitude_max = 8.0,
                            symmetric = true)
        b = with_elevation_max(box, 45.0)
        @test b.elevation_max == 45.0
        @test sides(b) == (40.0, 20.0, 45.0, 10.0, 8.0, true)
        b = with_azimuth_amplitude_min(box, 15.0)
        @test sides(b) == (40.0, 20.0, 50.0, 15.0, 8.0, true)
        # From no box at all, only the one side is set.
        @test sides(with_elevation_max(nothing, 45.0)) == (nothing, nothing, 45.0, nothing, nothing, nothing)
        @test sides(with_azimuth_amplitude_min(nothing, 15.0)) ==
              (nothing, nothing, nothing, 15.0, nothing, nothing)
    end

    @testset "with_size_box" begin
        az, el = figure_eight_path(20.0, 8.0, 0.0, 30.0, 0.0, 100)
        el_lo, el_hi = extrema(el)
        slack = 0.5 * 0.2 * (el_hi - el_lo)
        # Growth off returns the box unchanged, nothing included.
        @test isnothing(with_size_box(nothing, az, el, 0.0))
        # From no box: every size side from the path, grown by 20 %.
        b = with_size_box(nothing, az, el, 1.2)
        @test b.azimuth_max ≈ 1.2 * maximum(abs, az)
        @test b.elevation_amplitude_max ≈ 1.2 * elevation_amplitude(el)
        @test b.elevation_min ≈ el_lo - slack
        @test b.elevation_max ≈ el_hi + slack
        @test isnothing(b.symmetric)
        # A box that is already tighter keeps its sides; a looser one is tightened.
        tight = PatternLimits(; azimuth_max = 5.0, elevation_min = 29.0, elevation_max = 31.0,
                              elevation_amplitude_max = 1.0, azimuth_amplitude_min = 3.0,
                              symmetric = true)
        @test sides(with_size_box(tight, az, el, 1.2)) == sides(tight)
        loose = PatternLimits(; azimuth_max = 80.0, elevation_min = 0.0, elevation_max = 80.0,
                              elevation_amplitude_max = 30.0)
        b = with_size_box(loose, az, el, 1.2)
        @test b.azimuth_max ≈ 1.2 * maximum(abs, az)
        @test b.elevation_min ≈ el_lo - slack
        @test b.elevation_max ≈ el_hi + slack
        @test b.elevation_amplitude_max ≈ 1.2 * elevation_amplitude(el)
    end
end
