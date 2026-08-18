# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Unit tests for the guidance, the metrics and the turn-rate table. Pure geometry:
no simulation and no kite model, so the whole file runs in well under a second.
"""

using Test
using SimpleKiteControllers
import KiteUtils: wrap2pi
# Via the package: environments this runs from, like examples/, lack a direct YAML dep.
import SimpleKiteControllers: YAML
# The guidance's own metric, for checking a remapped Q landed near the old one.
import SimpleKiteControllers: _dist as _skc_dist

_make_test_controller(; kwargs...) =
    FigureEightController(FigureEightSettings(; dt = 0.02, A = 10.0, B = 5.0,
                                              C = 0.0, D = -1.0,
                                              el_center = 45.0,
                                              attractor_distance = 10.0,
                                              up_loops = false, kwargs...))

"""
Walk a kite twice around `fec`'s own path, `offset` degrees off it along the local
normal, and return the largest cyclic index jump Q made between two calls.
"""
function _max_q_jump(fec; offset = 0.0)
    n = length(fec.az_path)
    max_jump = 0
    prev = nothing
    for i in 1:2:(2n)
        k = mod1(i, n)
        kn = mod1(k + 1, n)
        dx = fec.az_path[kn] - fec.az_path[k]
        dy = fec.el_path[kn] - fec.el_path[k]
        len = hypot(dx, dy)
        calc_attractor(fec, fec.az_path[k] - offset * dy / len,
                       fec.el_path[k] + offset * dx / len)
        if prev !== nothing
            jump = abs(fec.last_idx - prev)
            max_jump = max(max_jump, min(jump, n - jump))   # cyclic distance
        end
        prev = fec.last_idx
    end
    max_jump
end

@testset verbose = true "fig8_controller" begin

    @testset "path_direction" begin
        # up_loops decides whether the right lobe's azimuth extreme is passed going up.
        for up in (false, true)
            fec = _make_test_controller(up_loops = up)
            n = length(fec.az_path)
            imax = argmax(fec.az_path)
            going_up = fec.el_path[mod1(imax + 1, n)] - fec.el_path[imax] > 0
            @test going_up == up
        end
    end

    @testset "attractor" begin
        fec = _make_test_controller()
        i0 = 30
        az0, el0 = fec.az_path[i0], fec.el_path[i0]
        az_attr, el_attr, dmin = calc_attractor(fec, az0, el0)
        # sitting exactly on the path -> zero cross-track error
        @test dmin < 1e-9
        @test fec.last_idx == i0
        # the attractor is attractor_distance° of arc ahead of Q
        n = length(fec.az_path)
        cum = 0.0
        k = i0
        while cum < fec.fes.attractor_distance
            cum += fec.seg_len[k]
            k = mod1(k + 1, n)
        end
        @test az_attr == fec.az_path[k]
        @test el_attr == fec.el_path[k]
    end

    @testset "branch_disambiguation" begin
        # The guard only ranks candidates the search offers, so this exercises it
        # with the window off: within a window the far branch is out of reach.
        fec = _make_test_controller(search_window = 0.0)
        # The self-intersection: both branches pass through it (D = -1 puts it low).
        n = length(fec.az_path)
        crossings = [i for i in 1:n if abs(fec.az_path[i] - fec.fes.az_center) < 0.2]
        @test length(crossings) >= 2
        az0 = fec.fes.az_center
        el0 = sum(fec.el_path[i] for i in crossings) / length(crossings)
        # One candidate per branch: the indices cluster at the seam and at t ≈ π.
        i_b = crossings[argmin(abs.(crossings .- n ÷ 2))]
        picked = Int[]
        for i in (crossings[1], i_b)
            # Pretend we fly along branch i, one small step behind along its tangent.
            f = _make_test_controller(search_window = 0.0)
            chi = f.tangent[i]
            step = 0.05  # [deg]
            f.has_prev = true
            f.prev_el = el0 - step * cos(chi)
            f.prev_az = az0 - step * sin(chi) / cosd(el0)
            f.fx = cos(chi)
            f.fy = sin(chi)
            f.course = chi
            calc_attractor(f, az0, el0)
            push!(picked, f.last_idx)
        end
        # a different course picks a different branch
        @test picked[1] != picked[2]
        @test abs(wrap2pi(fec.tangent[picked[1]] - fec.tangent[picked[2]])) > deg2rad(30)
    end

    @testset "navigate_fig8" begin
        fec = _make_test_controller()
        # kite far below the path center: chi_set points upwards (|chi| small)
        chi, az_attr, el_attr, dmin = navigate_fig8(fec, 0.0, deg2rad(20.0))
        @test abs(chi) < deg2rad(60)
        @test dmin > 10.0
        # kite far on the left, at path elevation: chi_set points to the right
        fec2 = _make_test_controller()
        chi2, _, _, _ = navigate_fig8(fec2, deg2rad(-60.0), deg2rad(45.0))
        @test chi2 > deg2rad(30)
    end

    @testset "set_path_center" begin
        fec = _make_test_controller(el_center = 70.0)
        @test fec.fes.el_center == 70.0
        i0 = 30
        el0 = fec.el_path[i0]
        set_path_center!(fec, 5.0, 50.0)
        @test fec.fes.az_center == 5.0
        @test fec.fes.el_center == 50.0
        # the path actually moved
        @test !isapprox(fec.el_path[i0], el0; atol = 1e-6)
        # and the moved path is internally consistent (mirrors "attractor")
        _, _, dmin = calc_attractor(fec, fec.az_path[i0], fec.el_path[i0])
        @test dmin < 1e-9
    end

    @testset "set_path" begin
        # An externally supplied path replaces the lemniscate and stays flyable.
        src = _make_test_controller()
        fec = _make_test_controller(A = 30.0, B = 12.0, el_center = 25.0)
        set_path!(fec, src.az_path, src.el_path)
        @test fec.az_path == src.az_path
        @test fec.el_path == src.el_path
        @test fec.seg_len ≈ src.seg_len
        @test fec.tangent ≈ src.tangent
        _, _, dmin = calc_attractor(fec, fec.az_path[40], fec.el_path[40])
        @test dmin < 1e-9

        # A closed path (last point repeats the first) loses the duplicate.
        n0 = length(src.az_path)
        fec2 = _make_test_controller()
        set_path!(fec2, [src.az_path; src.az_path[1]], [src.el_path; src.el_path[1]])
        @test length(fec2.az_path) == n0

        # The traversal direction is forced to match up_loops, whatever came in.
        for up in (false, true)
            fec3 = _make_test_controller()
            set_path!(fec3, reverse(src.az_path), reverse(src.el_path); up_loops = up)
            n = length(fec3.az_path)
            imax = argmax(fec3.az_path)
            going_up = fec3.el_path[mod1(imax + 1, n)] - fec3.el_path[imax] > 0
            @test going_up == up
            @test fec3.fes.up_loops == up
        end

        # Q is remapped onto the new path, not left pointing at a stale index.
        fec4 = _make_test_controller()
        calc_attractor(fec4, fec4.az_path[50], fec4.el_path[50])
        q_az, q_el = fec4.az_path[fec4.last_idx], fec4.el_path[fec4.last_idx]
        set_path!(fec4, src.az_path, src.el_path; resample = 120)
        @test length(fec4.az_path) == 120
        @test _skc_dist(q_az, q_el, fec4.az_path[fec4.last_idx],
                        fec4.el_path[fec4.last_idx]) < 1.0

        # A repeated point has no direction; it is rejected rather than flown.
        bad_az = [src.az_path[1:10]; src.az_path[10]; src.az_path[11:end]]
        bad_el = [src.el_path[1:10]; src.el_path[10]; src.el_path[11:end]]
        @test_throws ArgumentError set_path!(_make_test_controller(), bad_az, bad_el)
        @test_throws ArgumentError set_path!(_make_test_controller(),
                                             src.az_path, src.el_path[1:end-1])
    end

    @testset "resample_path" begin
        src = _make_test_controller()
        # Equidistant in the guidance's own metric, and still the same curve.
        az, el = resample_path(src.az_path, src.el_path, 180)
        @test length(az) == length(el) == 180
        fec = _make_test_controller()
        set_path!(fec, az, el)
        seg = fec.seg_len
        @test maximum(seg) / minimum(seg) < 1.02
        @test isapprox(sum(seg), sum(src.seg_len); rtol = 1e-3)
        # The tightest curvature is a property of the curve, not of its sampling.
        @test isapprox(path_min_radius(fec), path_min_radius(src); rtol = 0.05)
        # Upsampling works too, and a non-uniformly sampled input is the point:
        # take every other point of one lobe and check the result is even again.
        idx = [1:2:200; 201:length(src.az_path)]
        az2, el2 = resample_path(src.az_path[idx], src.el_path[idx], 300)
        fec2 = _make_test_controller()
        set_path!(fec2, az2, el2)
        @test maximum(fec2.seg_len) / minimum(fec2.seg_len) < 1.05
        @test_throws ArgumentError resample_path(src.az_path, src.el_path, 3)
    end

    @testset "search_window_continuity" begin
        # Q must advance along the path, never jump to the far branch.
        fec = _make_test_controller()
        n = length(fec.az_path)
        # Slack allowed, but nothing like the n/2 jump a branch switch produces.
        @test _max_q_jump(fec) < n ÷ 8

        # A path smaller than twice `search_window` must still keep a window:
        # slightly off the path, an uncapped window lets Q hop the crossing.
        fec_s = _make_test_controller()
        @test sum(fec_s.seg_len) < 2 * fec_s.fes.search_window
        @test _max_q_jump(fec_s; offset = 0.3) < n ÷ 8

        # With the window disabled the search is global again.
        fec2 = _make_test_controller(search_window = 0.0)
        @test fec2.fes.search_window == 0.0
        _, _, d2 = calc_attractor(fec2, fec2.az_path[30], fec2.el_path[30])
        @test d2 < 1e-9

        # A kite far off the path re-acquires globally, not trapped in a stale window.
        fec3 = _make_test_controller()
        calc_attractor(fec3, fec3.az_path[10], fec3.el_path[10])
        stale = fec3.last_idx
        far_az = fec3.fes.az_center + 80.0     # way outside reacquire_dist
        calc_attractor(fec3, far_az, fec3.fes.el_center)
        # then bring it back onto the path on the OTHER side of the eight
        j = mod1(stale + n ÷ 2, n)
        _, _, d3 = calc_attractor(fec3, fec3.az_path[j], fec3.el_path[j])
        @test d3 < 1e-6      # found it despite the window
    end

    @testset "metrics_lap_counting" begin
        # Regression tests for three lap-counting bugs; see docs/fig8_tuning_log.md.
        dt = 0.05
        tt = collect(0.0:dt:60.0)
        n = length(tt)
        # Both tape (`steering`) and command (`set_steering`) are read by the metrics.
        mk(azf, elf = t -> deg2rad(45.0)) =
                  (; time = tt,
                   azimuth = Float32.(azf.(tt)),
                   elevation = Float32.(elf.(tt)),
                   heading = zeros(Float32, n),
                   var_01 = zeros(Float32, n),
                   set_steering = zeros(Float32, n),
                   steering = zeros(Float32, n),
                   winch_force = [Float32[100, 0, 0, 0] for _ in 1:n])

        # a circle off to one side: oscillates about -40°, never crosses 0
        off = mk(t -> deg2rad(-40.0 + 8.0 * sin(2pi * t / 10)))
        @test fig8_metrics(off; settle_time = 0.0, az_center = 0.0).laps == 0.0
        # ... and the printer must reach the same conclusion (bug 3)
        m_print = print_fig8_metrics(off; settle_time = 0.0, az_center = 0.0)
        @test m_print.laps == 0.0

        # 6 centre crossings in 60 s at a 10 s period -> 6 laps by the /2 convention.
        real8 = mk(t -> deg2rad(40.0 * sin(2pi * t / 10)))
        laps = fig8_metrics(real8; settle_time = 0.0, az_center = 0.0).laps
        @test laps >= 5.0
        @test print_fig8_metrics(real8; settle_time = 0.0, az_center = 0.0).laps == laps
    end

    @testset "metrics_pattern_extent" begin
        # Perfect tracking, wrong flight: these must fail on extent alone.
        A, B, el_c, T = 40.0, 15.0, 26.0, 10.0
        dt = 0.05
        tt = collect(0.0:dt:60.0)
        n = length(tt)
        mk(azf, elf) = (; time = tt,
                        azimuth = Float32.(azf.(tt)),
                        elevation = Float32.(elf.(tt)),
                        heading = zeros(Float32, n),
                        var_01 = zeros(Float32, n),
                        set_steering = zeros(Float32, n),
                        steering = zeros(Float32, n),
                        winch_force = [Float32[100, 0, 0, 0] for _ in 1:n])
        # elevation of the lemniscate: sin(2wt), peak to peak B, scaled by `f`
        el8(f) = t -> deg2rad(el_c + f * (B / 2) * sin(2 * 2pi * t / T))
        score(sl) = print_fig8_metrics(sl; settle_time = 0.0, az_center = 0.0,
                                       az_amplitude = A, el_height = B,
                                       min_span_frac = 0.7)

        # the pattern as commanded: full width both sides, full height
        full = score(mk(t -> deg2rad(A * sin(2pi * t / T)), el8(1.0)))
        @test isempty(full.criteria_failed)
        @test full.criteria == 7          # 4 tracking + 2 azimuth reach + 1 span
        @test full.az_reach_pos ≈ A rtol=0.01
        @test full.az_reach_neg ≈ A rtol=0.01
        @test full.el_fill ≈ 1.0 rtol=0.01

        # a small eight about the same centre: tracked perfectly, 15% of the size
        small = score(mk(t -> deg2rad(0.15A * sin(2pi * t / T)), el8(0.15)))
        @test small.rms_d == 0.0          # nothing in the tracking numbers says so
        @test count(f -> occursin("azimuth reach", f), small.criteria_failed) == 2
        @test any(f -> occursin("elevation span", f), small.criteria_failed)

        # one half of the wind window: azimuth 0..+A, never crossing the centre
        half = score(mk(t -> deg2rad(A / 2 * (1 + sin(2pi * t / T))), el8(1.0)))
        @test half.rms_d == 0.0
        @test half.az_reach_pos >= 0.7A   # the flown lobe is full size ...
        @test half.az_reach_neg < 1.0     # ... and there is no other lobe
        @test count(f -> occursin("azimuth reach", f), half.criteria_failed) == 1
        # and the failure names the side, so a log line is enough to diagnose it
        @test any(f -> occursin("azimuth reach -", f), half.criteria_failed)

        # Without the geometry the extent is reported but not scored.
        bare = print_fig8_metrics(mk(t -> deg2rad(0.15A * sin(2pi * t / T)), el8(0.15));
                                  settle_time = 0.0, az_center = 0.0)
        @test bare.criteria == 4
        @test isnan(bare.az_fill_pos)
    end

    @testset "turn_rate_coeffs" begin
        # Body damping changes the steering response by 5.6x: keyed on it, never guessed.
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1 ≈ 0.31038589289512725
        # Confounded by amplitude range (u_s_max 0.175 -> 0.40), not re-run.
        @test turn_rate_coeffs([10.0, 10.0, 40.0], 0.25).c1 ≈ 0.12028768896596123
        @test turn_rate_coeffs([20.0, 20.0, 40.0], 0.25).c1 ≈ 0.0567
        # more damping -> less agile
        @test turn_rate_coeffs([20.0, 20.0, 40.0], 0.25).c1 <
              turn_rate_coeffs([10.0, 10.0, 40.0], 0.25).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        # 0.23, the first row identified with an explicit damping floor (flown [0,0,40]).
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.23).c1 ≈ 0.3225
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.23).c1 >
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        # DEPOWER costs authority too: 0.25 -> 0.55 is ~2.95x of c1 and 16x the dead time.
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).c1 ≈ 0.1073
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).delay >
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).delay
        # Integer input is converted; unidentified combinations throw.
        @test turn_rate_coeffs([0, 0, 40], 0.25).c1 ≈ V3_TURN_RATE_C1
        @test_throws ArgumentError turn_rate_coeffs([5.0, 5.0, 40.0], 0.25)
        @test_throws ArgumentError turn_rate_coeffs([0.0, 0.0, 40.0], 0.70)
        # depower 0.40 sits between the 0.25 and 0.55 rows, as monotonicity requires.
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.40).c1 ≈ 0.1513
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.40).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        # the exported defaults track init's default damping at depower 0.25
        @test V3_TURN_RATE_C1 == turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        @test V3_TURN_RATE_C2 == turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c2
        # Exact grid hits are never interpolated.
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.40).interpolated == false
        @test turn_rate_coeffs([20.0, 20.0, 40.0], 0.25).interpolated == false
        # A row carrying its own l_tether/dt is data like any other: no warning.
        @test_logs turn_rate_coeffs([20.0, 20.0, 40.0], 0.25)
    end

    @testset "turn_rate_coeffs interpolation (conditions block)" begin
        # Synthetic table, so this does not depend on how many real rows exist yet.
        bd = [0.0, 0.0, 40.0]
        entries = [
            (body_damping = bd, depower = 0.30, c1 = 0.20, c2 = -0.10, delay = 0.05,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done),
            (body_damping = bd, depower = 0.50, c1 = 0.05, c2 = 0.20, delay = 0.50,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done),
            # A failed sweep: kept in the table, but never used, not even on an exact hit.
            (body_damping = bd, depower = 0.60, c1 = 999.0, c2 = 0.0, delay = 0.0,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :low_elevation),
        ]
        synthetic = SimpleKiteControllers.TurnRateTable(
            Dict{Symbol, Any}(:system => "test.yaml", :v_wind => 9.51,
                               :l_tether => 200.0, :dt => 0.05 / 3), entries)
        old = SimpleKiteControllers._TURN_RATE_TABLE[]
        SimpleKiteControllers._TURN_RATE_TABLE[] = synthetic
        try
            # exact grid points reproduce the input exactly, not interpolated
            r30 = turn_rate_coeffs(bd, 0.30)
            @test r30.c1 == 0.20 && r30.c2 == -0.10 && r30.delay == 0.05
            @test r30.interpolated == false
            r50 = turn_rate_coeffs(bd, 0.50)
            @test r50.c1 == 0.05 && r50.interpolated == false

            # c1 is log-linear, c2/delay linear, delay rounded UP to a dt multiple.
            r = turn_rate_coeffs(bd, 0.40)
            @test r.interpolated == true
            @test r.c1 ≈ sqrt(0.20 * 0.05)
            @test r.c2 ≈ (-0.10 + 0.20) / 2
            raw_delay = (0.05 + 0.50) / 2
            dt = 0.05 / 3
            @test r.delay ≈ ceil(raw_delay / dt) * dt
            @test r.delay >= raw_delay

            # c1 decreases monotonically between the two grid points
            @test turn_rate_coeffs(bd, 0.35).c1 > turn_rate_coeffs(bd, 0.45).c1

            # No extrapolation, ever; a failed row does not extend the range.
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.20)
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.55)
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.58)
            # An exact hit on the failed row throws rather than returning c1 = 999.0.
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.60)

            # interpolate=false refuses to interpolate even inside the range
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.40; interpolate = false)

            # unknown body_damping still throws
            @test_throws ArgumentError turn_rate_coeffs([1.0, 1.0, 40.0], 0.40)
        finally
            SimpleKiteControllers._TURN_RATE_TABLE[] = old
        end
    end

    @testset "turn_rate_coeffs own-conditions rows are ordinary data" begin
        # A row carrying its own l_tether/dt (identified at a different tether
        # length or timestep than the conditions block) is used unchanged and
        # silently: neither quantity enters c1/c2, which are normalised by v_app.
        bd = [0.0, 0.0, 40.0]
        entries = [
            (body_damping = bd, depower = 0.20, c1 = 0.5, c2 = 0.0, delay = 0.03,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done),
            (body_damping = bd, depower = 0.60, c1 = 0.05, c2 = 0.0, delay = 0.6,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done),
        ]
        synthetic = SimpleKiteControllers.TurnRateTable(
            Dict{Symbol, Any}(:system => "test.yaml", :v_wind => 9.51,
                               :l_tether => 200.0, :dt => 0.05 / 3), entries)
        old = SimpleKiteControllers._TURN_RATE_TABLE[]
        SimpleKiteControllers._TURN_RATE_TABLE[] = synthetic
        try
            @test_logs turn_rate_coeffs(bd, 0.20)           # exact hit, no warning
            @test turn_rate_coeffs(bd, 0.20).c1 == 0.5
            # and they are interpolation neighbours like any other row
            r = turn_rate_coeffs(bd, 0.40)
            @test r.interpolated == true
            @test r.c1 ≈ sqrt(0.5 * 0.05)
        finally
            SimpleKiteControllers._TURN_RATE_TABLE[] = old
        end
    end

    @testset "turn_radius_feasibility" begin
        # rho = 1/(L*c1*u_s), independent of apparent wind speed
        @test min_turn_radius(150.0, 0.175) ≈
              rad2deg(1 / (150.0 * V3_TURN_RATE_C1 * 0.175))
        # a more damped kite cannot turn as tightly
        @test min_turn_radius(150.0, 0.175; c1 = turn_rate_coeffs([10.0,10.0,40.0], 0.25).c1) >
              min_turn_radius(150.0, 0.175; c1 = turn_rate_coeffs([0.0,0.0,40.0], 0.25).c1)
        # a longer tether allows a TIGHTER angular turn (rho = 1/(L*c1*u_s))
        @test min_turn_radius(300.0, 0.30) < min_turn_radius(150.0, 0.30)
        # more authority or a longer tether -> tighter achievable turn
        @test min_turn_radius(150.0, 0.35) < min_turn_radius(150.0, 0.175)
        @test min_turn_radius(300.0, 0.175) < min_turn_radius(150.0, 0.175)

        # A = B still gives a figure-eight, so test the monotonicity instead.
        small = FigureEightController(FigureEightSettings(; dt = 0.02, A = 10.0,
                                                          B = 5.0, el_center = 45.0))
        big = FigureEightController(FigureEightSettings(; dt = 0.02, A = 40.0,
                                                        B = 20.0, el_center = 45.0))
        # a larger pattern is less tightly curved
        @test path_min_radius(big) > path_min_radius(small)
        @test path_min_radius(small) > 0
        @test path_min_radius(big) == minimum(path_radius_profile(big))

        # Raising the pattern makes it TIGHTER: cos(elevation) compresses azimuth.
        low  = FigureEightController(FigureEightSettings(; dt = 0.02, A = 45.0,
                                                         B = 20.0, el_center = 35.0))
        high = FigureEightController(FigureEightSettings(; dt = 0.02, A = 45.0,
                                                         B = 20.0, el_center = 60.0))
        @test path_min_radius(high) < path_min_radius(low)

        # feasibility margin is the ratio of the two radii
        f = check_pattern_feasible(big, 150.0, 0.175; prn = false)
        @test f.margin ≈ f.path_radius / f.kite_radius
        @test f.feasible == (f.margin >= 1.0)
    end

    @testset "winch_force_gains" begin
        fcs = FC_Settings()
        fcs.compliance = 1.0
        g = winch_force_gains(fcs)
        # Keys must match the winch controller's field names; the caller splats them.
        @test keys(g) == (:force_tau, :len_kp, :damp, :force_min)
        @test g.len_kp == fcs.winch_len_kp
        @test g.damp == fcs.winch_damp

        # Both gains scale together, so the length loop's time constant does not move.
        fcs.compliance = 0.5
        h = winch_force_gains(fcs)
        @test h.len_kp == fcs.winch_len_kp / 0.5
        @test h.damp == fcs.winch_damp / 0.5
        @test h.damp / h.len_kp ≈ g.damp / g.len_kp
        # force_tau sets WHICH frequencies the drum yields to, not by how much.
        @test h.force_tau == fcs.winch_force_tau
        @test h.force_min == fcs.winch_force_min

        # compliance 0 is position mode and must not silently divide by zero.
        fcs.compliance = 0.0
        @test_throws ErrorException winch_force_gains(fcs)
    end

    @testset "pattern_height" begin
        fec = _make_test_controller(el_center = 20.0)
        el_min = minimum(fec.el_path)
        # Straight-tether relation, and the lowest point is the lowest elevation.
        @test path_min_height(fec, 200.0) ≈ 200.0 * sind(el_min)
        # A path specified in (azimuth, elevation) rises as the tether grows, so the
        # SHORTEST tether is the worst case.
        @test path_min_height(fec, 150.0) < path_min_height(fec, 350.0)
        @test path_min_height(fec, 2 * 150.0) ≈ 2 * path_min_height(fec, 150.0)

        floor_m = path_min_height(fec, 200.0)
        @test check_pattern_height(fec, 200.0, floor_m - 1; prn = false).ok
        @test !check_pattern_height(fec, 200.0, floor_m + 1; prn = false).ok
        res = check_pattern_height(fec, 200.0, 50.0; prn = false)
        @test res.elevation ≈ el_min
        @test res.height ≈ path_min_height(fec, 200.0)
        @test res.min_height == 50.0
    end

    @testset "prepare_path_and_blend" begin
        src = _make_test_controller()
        n = 180
        az, el = prepare_path(src.az_path, src.el_path; resample = n,
                              up_loops = src.fes.up_loops)
        @test length(az) == n
        # Canonical form: index 1 is the azimuth extreme, whatever the input's
        # starting point, and the traversal direction is the requested one.
        @test argmax(az) == 1
        # Rotating the input gives the same curve back, but only up to the SAMPLING
        # GRID: `resample_path` measures arc length from the first input point, and
        # the alignment then picks the nearest sample to the azimuth extreme, so the
        # two runs can sit up to half a sample apart in phase. That is the accuracy
        # a blend inherits, and half a spacing here is ~0.3°.
        rot = 57
        az2, el2 = prepare_path(circshift(src.az_path, rot), circshift(src.el_path, rot);
                                resample = n, up_loops = src.fes.up_loops)
        @test maximum(abs.(az2 .- az)) < 0.5
        @test maximum(abs.(el2 .- el)) < 0.5
        # Same curve reversed comes back identical once oriented and aligned.
        az3, el3 = prepare_path(reverse(src.az_path), reverse(src.el_path);
                                resample = n, up_loops = src.fes.up_loops)
        @test maximum(abs.(az3 .- az)) < 0.5
        for up in (false, true)
            a, e = prepare_path(src.az_path, src.el_path; resample = n, up_loops = up)
            m = length(a)
            @test (e[mod1(argmax(a) + 1, m)] - e[argmax(a)] > 0) == up
        end

        # Blending: the endpoints are the inputs, the middle is halfway.
        bz, be = prepare_path(src.az_path, 5.0 .+ src.el_path; resample = n,
                              up_loops = src.fes.up_loops)
        @test blend_paths(az, el, bz, be, 0.0)[1] ≈ az
        @test blend_paths(az, el, bz, be, 1.0)[2] ≈ be
        @test blend_paths(az, el, bz, be, 0.5)[2] ≈ 0.5 .* (el .+ be)
        # w is clamped, so a blend cannot overshoot past the target.
        @test blend_paths(az, el, bz, be, 2.0)[2] ≈ be
        @test_throws ArgumentError blend_paths(az, el, bz[1:end-1], be[1:end-1], 0.5)
    end

    @testset "path_queries_on_bare_vectors" begin
        # The checks are about a PATH, not about a controller: stage 4 validates a
        # candidate before there is a controller holding it.
        fec = _make_test_controller()
        az, el = fec.az_path, fec.el_path
        @test path_min_radius(az, el) ≈ path_min_radius(fec)
        @test path_radius_profile(az, el) ≈ path_radius_profile(fec)
        @test path_min_height(az, el, 200.0) ≈ path_min_height(fec, 200.0)
        @test check_pattern_feasible(az, el, 200.0, 0.3; prn = false).margin ≈
              check_pattern_feasible(fec, 200.0, 0.3; prn = false).margin
        @test check_pattern_height(az, el, 200.0, 40.0; prn = false).height ≈
              check_pattern_height(fec, 200.0, 40.0; prn = false).height
    end

    @testset "traj_opt_settings" begin
        tos = TrajOptSettings("traj_opt.yaml")
        # The shipped file loads, and its guess is NOT the reel-out pattern: those
        # 20°/11°-at-18° values do not converge at 150 m and 6 m/s (2026-08-18).
        @test tos.base_url == "http://127.0.0.1:8000"
        @test tos.guess_a == 30.0
        @test tos.guess_b == 12.0
        # 22, not the 26 that was measured at 150 m: 26 converges at 150.0, 179.0,
        # 180.0 and 185.0 m but throws IPOPT's iteration limit at 180.00027 and
        # 181.0, while 22 converges at all six and reaches the same optimum wherever
        # 26 also worked (2026-08-18). See the YAML header.
        @test tos.guess_el_center == 22.0
        @test tos.guess_points == 361
        @test tos.resample_points == 361
        # Below the struct's 1.0: the optimizer returns paths right at the V3's
        # curvature limit at EVERY length, not just the shortest (11.0-11.5 m of
        # physical turn radius against the kite's 11.35 m, measured 2026-08-18), so
        # at 1.0 almost nothing installs. A rejection-rate knob that moves with
        # every sweep (0.74 on 2026-08-18), so only the inequality is pinned.
        @test 0 < tos.min_feasibility_margin < TrajOptSettings().min_feasibility_margin
        # 40, not AWETrim's 50: an installed (azimuth, elevation) curve clears only
        # ~50*r0/r_low at its anchor radius, whatever the anchor. See the YAML.
        @test tos.min_height == 40.0
        # The DEFAULT, not the YAML: reopt_enabled is a per-run switch and the file
        # carries whatever the last experiment needed.
        @test !TrajOptSettings().reopt_enabled
        # 1, not the struct's 2: the shipped file re-anchors every lap, which is
        # what the reel-out window is long enough for.
        @test tos.reopt_every_n_laps == 1
        # At least the struct's bound: re-anchoring every lap needs one solve per
        # lap of the reel-out window, which is more than the default (7 on
        # 2026-08-18) — the exact count follows the window, so it is not pinned.
        @test tos.max_reopt >= TrajOptSettings().max_reopt
        @test tos.path_blend_time == 4.0
        # The gate reads the reference path, the criterion scores the flown one, and
        # ~3° of undershoot was measured twice on 2026-08-18.
        @test tos.candidate_elevation_margin == 3.0
        @test tos.detect_simple_bounds

        mktempdir() do dir
            good = joinpath(dir, "t.yaml")
            write(good, "traj_opt:\n    guess_a: 25.0\n")
            t2 = TrajOptSettings(good)
            @test t2.guess_a == 25.0          # given
            @test t2.guess_b == 12.0          # and the default for the rest
            bad = joinpath(dir, "b.yaml")
            write(bad, "traj_opt:\n    guess_with_a_typo: 1.0\n")
            @test_throws ErrorException TrajOptSettings(bad)
            tiny = joinpath(dir, "s.yaml")
            write(tiny, "traj_opt:\n    guess_points: 4\n")
            @test_throws ErrorException TrajOptSettings(tiny)
            neg = joinpath(dir, "n.yaml")
            write(neg, "traj_opt:\n    candidate_elevation_margin: -1.0\n")
            @test_throws ErrorException TrajOptSettings(neg)
        end
    end

    @testset "project_file" begin
        p = project_file()
        @test isabspath(p)
        @test dirname(p) == skc_data_path()
        @test isfile(p)
        # KiteUtils resolves the sim settings relative to the project, so they must sit here.
        system = YAML.load_file(p)["system"]
        settings = joinpath(skc_data_path(), system["sim_settings"])
        @test isfile(settings)
        # wc_settings is a bare name; the example points the data path here before init.
        @test isfile(joinpath(skc_data_path(), system["wc_settings"]))
        # The run length and timestep come from there, not from FC_Settings.
        settings_dict = YAML.load_file(settings)
        sys = settings_dict["system"]
        @test sys["sim_time"] > 0
        @test sys["sample_freq"] >= 60
        @test !hasfield(FC_Settings, :sim_time)
        @test !hasfield(FC_Settings, :dt)
        @test !hasfield(FC_Settings, :project)
        # The initial tether length is a plant condition (sim_settings' l_tethers), not FC_Settings.
        @test !hasfield(FC_Settings, :tether_length)
        @test haskey(settings_dict["initial"], "l_tethers")
        # So is the wind speed: it is read from `environment.v_wind` alone.
        @test !hasfield(FC_Settings, :v_wind)
        @test settings_dict["environment"]["v_wind"] > 0
        # A project this package does not carry stays a lookup under the active data path.
        @test project_file("no_such_system.yaml") == "no_such_system.yaml"
    end
end
