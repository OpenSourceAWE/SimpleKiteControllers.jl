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

_make_test_controller(; kwargs...) =
    FigureEightController(FigureEightSettings(; dt = 0.02, A = 10.0, B = 5.0,
                                              C = 0.0, D = -1.0,
                                              el_center = 45.0,
                                              attractor_distance = 10.0,
                                              up_loops = false, kwargs...))

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
        fec = _make_test_controller()
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
            f = _make_test_controller()
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

    @testset "search_window_continuity" begin
        # Q must advance along the path, never jump to the far branch.
        fec = _make_test_controller()
        n = length(fec.az_path)
        # Walk along the path and check Q never makes a large index jump.
        max_jump = 0
        prev = nothing
        for i in 1:2:(2n)
            k = mod1(i, n)
            calc_attractor(fec, fec.az_path[k], fec.el_path[k])
            if prev !== nothing
                jump = abs(fec.last_idx - prev)
                jump = min(jump, n - jump)          # cyclic distance
                max_jump = max(max_jump, jump)
            end
            prev = fec.last_idx
        end
        # Slack allowed, but nothing like the n/2 jump a branch switch produces.
        @test max_jump < n ÷ 8

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
