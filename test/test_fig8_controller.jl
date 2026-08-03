# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Unit tests for src/figure_eight_controller.jl. Ported from the in-module tests that
# shipped with the original guidance code, plus tests for the V3-specific
# turn-radius feasibility helpers. Everything here is pure geometry — no
# simulation, no model — so the whole file runs in well under a second.

using Test
using SimpleKiteControllers
import KiteUtils: wrap2pi

_make_test_controller(; kwargs...) =
    FigureEightController(FigureEightSettings(; dt = 0.02, A = 10.0, B = 5.0,
                                              C = 0.0, D = -1.0,
                                              el_center = 45.0,
                                              attractor_distance = 10.0,
                                              up_loops = false, kwargs...))

@testset verbose = true "fig8_controller" begin

    @testset "path_direction" begin
        # up_loops decides whether the kite passes the azimuth extreme of the
        # right lobe moving up or down
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
        # the self-intersection of the eight: azimuth = az_center, below the
        # center (D = -1); both branches pass through it
        n = length(fec.az_path)
        crossings = [i for i in 1:n if abs(fec.az_path[i] - fec.fes.az_center) < 0.2]
        @test length(crossings) >= 2
        az0 = fec.fes.az_center
        el0 = sum(fec.el_path[i] for i in crossings) / length(crossings)
        # one candidate per branch: the crossing indices cluster around the
        # cyclic seam (t ≈ 0/2π, same branch) and around t ≈ π (the other one)
        i_b = crossings[argmin(abs.(crossings .- n ÷ 2))]
        picked = Int[]
        for i in (crossings[1], i_b)
            # pretend we fly along branch i: fresh controller, course = path
            # tangent there, and a previous position one small step behind
            # along that course (so _update_course! keeps speed and course
            # consistent with the pretended motion)
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
        # Q must advance along the path, not jump to the far branch at the
        # self-intersection — that flip is what reversed the commanded course by
        # ~180° and broke a run at t=13.3 s.
        fec = _make_test_controller()
        n = length(fec.az_path)
        # walk the kite along the path for a few hundred steps and check that Q
        # never makes a large index jump
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
        # steps of 2 path points; allow slack, but nothing like the n/2 jump a
        # branch switch would produce
        @test max_jump < n ÷ 8

        # with the window disabled the search is global again (original
        # behaviour), so a point equidistant from both branches may pick either
        fec2 = _make_test_controller(search_window = 0.0)
        @test fec2.fes.search_window == 0.0
        _, _, d2 = calc_attractor(fec2, fec2.az_path[30], fec2.el_path[30])
        @test d2 < 1e-9

        # a kite far off the path re-acquires globally rather than being trapped
        # in a stale window
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
        # Regression tests for three successive lap-counting bugs, each of which
        # reported laps that were never flown:
        #   1. bare azimuth sign changes -> 42.5 laps for a kite in a limit cycle
        #   2. counting against the FLOWN MEAN -> 14 laps for a circle off to one
        #      side that never crossed the pattern centre
        #   3. print_fig8_metrics accepted az_center but did not FORWARD it, so
        #      passing it from the example silently did nothing -> 18 phantom laps
        dt = 0.05
        tt = collect(0.0:dt:60.0)
        n = length(tt)
        # `steering` is the TAPE position and `set_steering` the command; the
        # metrics read both (the tape rate limit is scored against `steering`),
        # so a fixture missing either does not exercise the function at all.
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

        # a genuine lemniscate sweeping +-40° about the centre: 6 crossings of
        # the centre in 60 s at a 10 s period -> 6 laps by the /2 convention
        real8 = mk(t -> deg2rad(40.0 * sin(2pi * t / 10)))
        laps = fig8_metrics(real8; settle_time = 0.0, az_center = 0.0).laps
        @test laps >= 5.0
        @test print_fig8_metrics(real8; settle_time = 0.0, az_center = 0.0).laps == laps
    end

    @testset "metrics_pattern_extent" begin
        # The tracking criteria are all measured to the CLOSEST POINT of the
        # path, so they say nothing about how much of the pattern was flown: a
        # kite tracing a small eight, or one lobe's worth of it in half the wind
        # window, sits on the path at every instant and scores RMS d = 0. These
        # fixtures are exactly that — perfect tracking, wrong flight — and must
        # fail on extent alone.
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

        # without the geometry the extent is reported but not scored — the old
        # four criteria, so an existing caller keeps its meaning
        bare = print_fig8_metrics(mk(t -> deg2rad(0.15A * sin(2pi * t / T)), el8(0.15));
                                  settle_time = 0.0, az_center = 0.0)
        @test bare.criteria == 4
        @test isnan(bare.az_fill_pos)
    end

    @testset "turn_rate_coeffs" begin
        # the in-plane body damping changes the steering response by 5.6x, so
        # the lookup must be keyed on it and must refuse to guess
        #
        # re-identified at 200 m capped at the SAME u_s_max = 0.175 as the
        # original 150 m sweep (2026-07-27), for a clean apples-to-apples
        # tether-length comparison: c1 0.3159 -> 0.3104, i.e. -1.7%, comfortably
        # under the <10% "Conditions" (PlanC1C2.md) assumed. That resolves the
        # +22.5% seen on [10,10,40]/0.25 below as a confound, not a real
        # tether-length effect: THAT re-run reached u_s_max = 0.40, well past
        # the 0.175 the legacy value was identified over, so it mixes a
        # tether-length change with an amplitude-range change. This clean
        # comparison is the one "Conditions" actually needs and it passes.
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1 ≈ 0.31038589289512725
        # confounded by amplitude range (u_s_max 0.175 -> 0.40), see above --
        # not re-run at a matched cap because [10,10,40]/0.25 passed on its
        # first (uncapped) attempt and there was no reason to redo it
        @test turn_rate_coeffs([10.0, 10.0, 40.0], 0.25).c1 ≈ 0.12028768896596123
        @test turn_rate_coeffs([20.0, 20.0, 40.0], 0.25).c1 ≈ 0.0567
        # more damping -> less agile
        @test turn_rate_coeffs([20.0, 20.0, 40.0], 0.25).c1 <
              turn_rate_coeffs([10.0, 10.0, 40.0], 0.25).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        # DEPOWER costs authority too: 0.25 -> 0.55 is a factor ~2.95 of c1,
        # and 16x the steering dead time. Getting this wrong is what made a run
        # fly a circle instead of the pattern (margin 1.41 assumed vs 0.48 real).
        #
        # re-identified at 200 m (2026-07-27, MIN_ELEVATION relaxed to 40 deg
        # for this run -- see PlanC1C2.md): c1 0.1071 -> 0.1073 (+0.2%) and
        # delay unchanged at 0.55 -- as clean a confirmation of low
        # tether-length sensitivity as depower 0.25's -1.7% was
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).c1 ≈ 0.1073
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).delay >
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).delay
        # integer input is accepted (converted); unidentified combinations throw
        # rather than silently returning a wrong c1
        @test turn_rate_coeffs([0, 0, 40], 0.25).c1 ≈ V3_TURN_RATE_C1
        @test_throws ArgumentError turn_rate_coeffs([5.0, 5.0, 40.0], 0.25)
        @test_throws ArgumentError turn_rate_coeffs([0.0, 0.0, 40.0], 0.70)
        # depower 0.40 (identified 2026-07-26 at 200 m tether) sits between the
        # 0.25 and 0.55 rows, as the monotonic trend requires
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.40).c1 ≈ 0.1513
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.55).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.40).c1 <
              turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        # the exported defaults track init's default damping at depower 0.25
        @test V3_TURN_RATE_C1 == turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c1
        @test V3_TURN_RATE_C2 == turn_rate_coeffs([0.0, 0.0, 40.0], 0.25).c2
        # exact grid hits are never interpolated -- neither a current-conditions
        # row nor a legacy one (e.g. [20,20,40]/0.25, still at 150 m)
        @test turn_rate_coeffs([0.0, 0.0, 40.0], 0.40).interpolated == false
        @test turn_rate_coeffs([20.0, 20.0, 40.0], 0.25).interpolated == false
    end

    @testset "turn_rate_coeffs interpolation (conditions block)" begin
        # Exercise the interpolation math directly against a synthetic table,
        # independent of how many real depower values examples/
        # build_turn_rate_table.jl has identified so far -- right after STEP 1
        # the real [0,0,40] group has only one non-legacy (200 m) row, too few
        # to interpolate at all.
        bd = [0.0, 0.0, 40.0]
        entries = [
            (body_damping = bd, depower = 0.30, c1 = 0.20, c2 = -0.10, delay = 0.05,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done,
             legacy = false, overrides = Dict{Symbol, Any}()),
            (body_damping = bd, depower = 0.50, c1 = 0.05, c2 = 0.20, delay = 0.50,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done,
             legacy = false, overrides = Dict{Symbol, Any}()),
            # a failed sweep at the same damping: kept, but never used, either
            # as a neighbour or (despite the exact depower hit) directly
            (body_damping = bd, depower = 0.60, c1 = 999.0, c2 = 0.0, delay = 0.0,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :low_elevation,
             legacy = false, overrides = Dict{Symbol, Any}()),
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

            # midpoint: c1 is log-linear (geometric mean at t=0.5), c2/delay
            # linear, delay rounded UP to a dt multiple (never optimistic)
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

            # no extrapolation, ever -- even up to the failed row's own depower,
            # which does not extend the interpolatable range
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.20)
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.55)
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.58)
            # an exact hit on the failed row itself throws rather than
            # returning a divergent-run c1 = 999.0
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.60)

            # interpolate=false refuses to interpolate even inside the range
            @test_throws ArgumentError turn_rate_coeffs(bd, 0.40; interpolate = false)

            # unknown body_damping still throws
            @test_throws ArgumentError turn_rate_coeffs([1.0, 1.0, 40.0], 0.40)
        finally
            SimpleKiteControllers._TURN_RATE_TABLE[] = old
        end
    end

    @testset "turn_rate_coeffs legacy-row warning (conditions block)" begin
        # [20,20,40]/0.25 is deliberately never re-identified at 200 m (see
        # PlanC1C2.md) -- the one legacy (150 m) row left in the real table,
        # now that [0,0,40] and [10,10,40] are fully promoted across all five
        # depowers (2026-07-27). Confirms the real table still warns on an
        # exact hit and excludes it from interpolation.
        empty!(SimpleKiteControllers._TURN_RATE_WARNED_LEGACY[])
        @test_logs (:warn,) turn_rate_coeffs([20.0, 20.0, 40.0], 0.25)
        @test_logs turn_rate_coeffs([20.0, 20.0, 40.0], 0.25)  # same row again: silent
        empty!(SimpleKiteControllers._TURN_RATE_WARNED_LEGACY[])

        # A distinct legacy row must not be silenced by an earlier one having
        # already warned (a plain `@warn ... maxlog=1` would dedup by call
        # site, not by which row, and wrongly suppress the second). Exercised
        # against a synthetic table rather than real data, which now has only
        # one legacy row to spare -- this property should hold regardless of
        # how many legacy rows the real table happens to have left.
        bd = [0.0, 0.0, 40.0]
        legacy_entries = [
            (body_damping = bd, depower = 0.20, c1 = 0.5, c2 = 0.0, delay = 0.03,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done,
             legacy = true, overrides = Dict{Symbol, Any}(:l_tether => 100.0)),
            (body_damping = bd, depower = 0.60, c1 = 0.05, c2 = 0.0, delay = 0.6,
             c1_rel_std = 0.001, g_rel_std = 0.05, outcome = :sweep_done,
             legacy = true, overrides = Dict{Symbol, Any}(:l_tether => 100.0)),
        ]
        synthetic = SimpleKiteControllers.TurnRateTable(
            Dict{Symbol, Any}(:system => "test.yaml", :v_wind => 9.51,
                               :l_tether => 200.0, :dt => 0.05 / 3), legacy_entries)
        old = SimpleKiteControllers._TURN_RATE_TABLE[]
        SimpleKiteControllers._TURN_RATE_TABLE[] = synthetic
        empty!(SimpleKiteControllers._TURN_RATE_WARNED_LEGACY[])
        try
            @test_logs (:warn,) turn_rate_coeffs(bd, 0.20)
            @test_logs turn_rate_coeffs(bd, 0.20)          # same row again: silent
            @test_logs (:warn,) turn_rate_coeffs(bd, 0.60) # different legacy row: warns
        finally
            SimpleKiteControllers._TURN_RATE_TABLE[] = old
            empty!(SimpleKiteControllers._TURN_RATE_WARNED_LEGACY[])
        end
    end

    @testset "turn_rate_coeffs conditions mismatch (conditions block)" begin
        old_ac = SimpleKiteControllers._ACTIVE_TURN_RATE_CONDITIONS[]
        try
            # nothing stashed (as in this bare test file, with no `init` call)
            # -> never warns
            SimpleKiteControllers._ACTIVE_TURN_RATE_CONDITIONS[] = nothing
            @test_logs turn_rate_coeffs([0.0, 0.0, 40.0], 0.40)

            # stashed conditions matching the table -> no warning
            table = SimpleKiteControllers._TURN_RATE_TABLE[]
            set_turn_rate_conditions!(v_wind = table.conditions[:v_wind],
                l_tether = table.conditions[:l_tether], system_yaml = table.conditions[:system])
            @test_logs turn_rate_coeffs([0.0, 0.0, 40.0], 0.40)

            # a mismatched wind speed warns once, but still returns the real
            # coefficients -- the warning is advisory, not a refusal
            set_turn_rate_conditions!(v_wind = 5.0,
                l_tether = table.conditions[:l_tether], system_yaml = table.conditions[:system])
            r = @test_logs (:warn,) turn_rate_coeffs([0.0, 0.0, 40.0], 0.40)
            @test r.c1 ≈ 0.1513
            # same stashed conditions again -> that field is already warned
            @test_logs turn_rate_coeffs([0.0, 0.0, 40.0], 0.40)
        finally
            SimpleKiteControllers._ACTIVE_TURN_RATE_CONDITIONS[] = old_ac
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

        # a circle of known angular radius is recovered by path_min_radius:
        # build one by using the lemniscate machinery with A = B (theta=0,
        # C=D=0 gives a figure-eight, so test the monotonicity instead)
        small = FigureEightController(FigureEightSettings(; dt = 0.02, A = 10.0,
                                                          B = 5.0, el_center = 45.0))
        big = FigureEightController(FigureEightSettings(; dt = 0.02, A = 40.0,
                                                        B = 20.0, el_center = 45.0))
        # a larger pattern is less tightly curved
        @test path_min_radius(big) > path_min_radius(small)
        @test path_min_radius(small) > 0
        @test path_min_radius(big) == minimum(path_radius_profile(big))

        # raising the pattern makes it TIGHTER (the azimuth axis is compressed
        # by cos(elevation)) — this is why a figure-eight near zenith is
        # geometrically impossible for this kite, see simple_fig8.jl
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
        # Keys must match the force-mode winch controller's field names, since
        # the caller splats this straight into it.
        @test keys(g) == (:force_tau, :len_kp, :damp, :force_min)
        @test g.len_kp == fcs.winch_len_kp
        @test g.damp == fcs.winch_damp

        # Halving the compliance stiffens both gains by the same factor, so the
        # length loop's own time constant (damp/len_kp) does not move.
        fcs.compliance = 0.5
        h = winch_force_gains(fcs)
        @test h.len_kp == fcs.winch_len_kp / 0.5
        @test h.damp == fcs.winch_damp / 0.5
        @test h.damp / h.len_kp ≈ g.damp / g.len_kp
        # force_tau sets WHICH frequencies the drum yields to, not by how much.
        @test h.force_tau == fcs.winch_force_tau
        @test h.force_min == fcs.winch_force_min

        # compliance 0 is position mode; an infinitely stiff spring is not
        # representable here and must not silently divide by zero.
        fcs.compliance = 0.0
        @test_throws ErrorException winch_force_gains(fcs)
    end
end
