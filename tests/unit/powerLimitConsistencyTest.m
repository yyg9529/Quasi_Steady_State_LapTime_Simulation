classdef powerLimitConsistencyTest < matlab.unittest.TestCase
    properties
        ProjectRoot
        SyntheticResult
        PowerTolerance_W
        ForceTolerance_N
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            testCase.ProjectRoot = fileparts(fileparts( ...
                fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "src"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "data"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "preprocessing"), ...
                IncludingSubfolders=true));

            track = powerLimitConsistencyTest.makeStraightTrack();
            [vehicle, models, options] = ...
                powerLimitConsistencyTest.makeConceptConfiguration();
            testCase.SyntheticResult = run_qss_lap( ...
                track, vehicle, models, options);
            testCase.PowerTolerance_W = max(1.0, 1e-5 * ...
                max(testCase.SyntheticResult.powertrain.tsac_power_cap_W));
            testCase.ForceTolerance_N = max(1e-3, 1e-5 * ...
                max(testCase.SyntheticResult.powertrain.wheel_force_available_N));
        end
    end

    methods (Test)
        function testNodePowerUsageDoesNotExceedCap(testCase)
            actual_W = testCase.SyntheticResult.powertrain.tsac_power_used_W;
            limit_W = testCase.SyntheticResult.powertrain.tsac_power_cap_W ...
                + testCase.PowerTolerance_W;

            testCase.verifyLessThanOrEqual(actual_W, limit_W);
        end

        function testNodeWheelForceDoesNotExceedAvailable(testCase)
            actual_N = testCase.SyntheticResult.powertrain.wheel_force_used_N;
            limit_N = ...
                testCase.SyntheticResult.powertrain.wheel_force_available_N ...
                + testCase.ForceTolerance_N;

            testCase.verifyLessThanOrEqual(actual_N, limit_N);
        end

        function testNodeCurrentUsageDoesNotExceedCap(testCase)
            powertrain = powertrain_emrax228_hvcc_demo();
            nodePowerCap_W = ...
                testCase.SyntheticResult.powertrain.tsac_power_cap_W;
            currentCap_A = nodePowerCap_W ...
                ./ testCase.SyntheticResult.powertrain.V_bus_V;

            testCase.verifyLessThanOrEqual( ...
                testCase.SyntheticResult.powertrain.tsac_dc_current_used_A, ...
                currentCap_A + testCase.PowerTolerance_W ...
                / powertrain.battery.V_bus_assumed_V);
        end

        function testSegmentPowerUsageDoesNotExceedCap(testCase)
            energy = testCase.SyntheticResult.energy;

            testCase.verifyEqual(energy.segment_tsac_power_margin_W, ...
                energy.segment_tsac_power_cap_W ...
                - energy.segment_tsac_power_W, AbsTol=1e-9);
            testCase.verifyGreaterThanOrEqual( ...
                energy.segment_tsac_power_margin_W, ...
                -testCase.PowerTolerance_W);
        end

        function testPowerLimitedSegmentReachability(testCase)
            options = default_qss_options();
            options.start_speed_mps = 10;
            track.s_m = [0; 20];
            track.ds_m = 20;
            track.kappa_1pm = zeros(2, 1);
            track.is_closed = false;
            ggv = powerLimitConsistencyTest.makePowerLimitedGgv( ...
                options.gravity_mps2);

            speed_mps = forward_pass(track, ggv, [20; 20], options);
            requiredAx_mps2 = (speed_mps(2)^2 - speed_mps(1)^2) ...
                / (2 * track.ds_m);
            representativeSpeed_mps = mean(speed_mps);
            representativeCap = interp_ggv(ggv, ...
                representativeSpeed_mps, 0, options);

            testCase.verifyLessThanOrEqual(requiredAx_mps2, ...
                representativeCap.ax_max_mps2 ...
                + options.accel_tolerance_mps2);
        end

        function testClosedTrackFinalSegmentReachability(testCase)
            options = default_qss_options();
            options.initial_profile_mps = [20; 20; 10];
            track.s_m = [0; 20; 40];
            track.ds_m = 20*ones(3, 1);
            track.kappa_1pm = zeros(3, 1);
            track.is_closed = true;
            ggv = powerLimitConsistencyTest.makePowerLimitedGgv( ...
                options.gravity_mps2);

            speed_mps = forward_pass(track, ggv, 20*ones(3, 1), options);
            requiredAx_mps2 = (speed_mps(1)^2 - speed_mps(3)^2) ...
                / (2 * track.ds_m(3));
            representativeSpeed_mps = mean(speed_mps([3, 1]));
            representativeCap = interp_ggv(ggv, ...
                representativeSpeed_mps, 0, options);

            testCase.verifyLessThanOrEqual(requiredAx_mps2, ...
                representativeCap.ax_max_mps2 ...
                + options.accel_tolerance_mps2);
        end

        function testRealTrackPowerCapLocal(testCase)
            trackFile = fullfile(testCase.ProjectRoot, "data", "track", ...
                "tianji_kart_QSS_track_closed.csv");
            testCase.assumeTrue(isfile(trackFile), ...
                "The local-only Tianji validation track is unavailable.");
            track = read_track_csv(trackFile);
            [vehicle, models, options] = ...
                powerLimitConsistencyTest.makeConceptConfiguration();

            result = run_qss_lap(track, vehicle, models, options);
            tolerance_W = max(1.0, 1e-5 * ...
                max(result.powertrain.tsac_power_cap_W));

            testCase.verifyLessThanOrEqual( ...
                result.powertrain.tsac_power_used_W, ...
                result.powertrain.tsac_power_cap_W + tolerance_W);
            testCase.verifyGreaterThanOrEqual( ...
                result.energy.segment_tsac_power_margin_W, -tolerance_W);
            testCase.verifyTrue(result.solver.converged);
            testCase.verifyEqual(track.length_m, ...
                857.461445744691, AbsTol=1e-9);
            testCase.verifyEqual(track.ds_m(end), ...
                1.461445744691, AbsTol=1e-9);
        end
    end

    methods (Static, Access=private)
        function track = makeStraightTrack()
            nPoint = 121;
            track.s_m = (0:5:5*(nPoint - 1)).';
            track.ds_m = 5*ones(nPoint - 1, 1);
            track.kappa_1pm = zeros(nPoint, 1);
            track.is_closed = false;
        end

        function [vehicle, models, options] = makeConceptConfiguration()
            vehicle = vehicle_baseline();
            models.tire = tire_load_sensitive_baseline();
            models.aero = aero_baseline();
            models.powertrain = powertrain_emrax228_hvcc_demo();
            models.brake = brake_baseline();
            models.endurance = struct("num_laps", 1, "safety_factor", 1);
            options = default_qss_options();
            options.start_speed_mps = 5;
            options.v_grid_mps = (0:1:45).';
            options.ay_grid_g = -4:0.1:4;
        end

        function ggv = makePowerLimitedGgv(gravity_mps2)
            ggv.v_mps = [0; 10; 20];
            ggv.ay_g = [-1, 0, 1];
            centerAx_g = [0.8; 0.8; 0.1];
            ggv.ax_max_g = [zeros(3, 1), centerAx_g, zeros(3, 1)];
            ggv.ax_min_g = repmat([0, -1, 0], 3, 1);
            ggv.feasible = true(3, 3);
            ggv.ay_limit_pos_g = ones(3, 1);
            ggv.ay_limit_neg_g = -ones(3, 1);
            ggv.accel_limiter = repmat("rule_power", 3, 3);
            ggv.brake_limiter = repmat("synthetic_brake", 3, 3);
            ggv.gravity_mps2 = gravity_mps2;
        end
    end
end
