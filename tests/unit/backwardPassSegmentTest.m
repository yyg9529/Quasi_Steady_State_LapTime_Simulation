classdef backwardPassSegmentTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testMidpointCapabilityLimitsMaximumEntrySpeed(testCase)
            options = default_qss_options();
            track.s_m = [0; 15];
            track.ds_m = 15;
            track.kappa_1pm = [0; 0];
            track.is_closed = false;
            input_mps = [20; 10];
            lateralLimit_mps = [30; 30];
            ggv = backwardPassSegmentTest.makeSpeedSensitiveGgv( ...
                options.gravity_mps2);

            [profile_mps, ~, info] = backward_pass(track, ggv, ...
                input_mps, lateralLimit_mps, options);
            midpoint_mps = mean(profile_mps);
            entryCap = interp_ggv(ggv, profile_mps(1), 0, options);
            midpointCap = interp_ggv(ggv, midpoint_mps, 0, options);
            limitingAxMin_mps2 = max( ...
                entryCap.ax_min_mps2, midpointCap.ax_min_mps2);
            requiredAx_mps2 = (profile_mps(2)^2 - profile_mps(1)^2) ...
                / (2 * track.ds_m);
            higherEntry_mps = profile_mps(1) + 1e-4;
            higherMidpoint_mps = 0.5 * (higherEntry_mps + profile_mps(2));
            higherEntryCap = interp_ggv(ggv, higherEntry_mps, 0, options);
            higherMidpointCap = interp_ggv( ...
                ggv, higherMidpoint_mps, 0, options);
            higherLimitingAxMin_mps2 = max( ...
                higherEntryCap.ax_min_mps2, ...
                higherMidpointCap.ax_min_mps2);
            higherRequiredAx_mps2 = (profile_mps(2)^2 ...
                - higherEntry_mps^2) / (2 * track.ds_m);

            testCase.verifyLessThan(profile_mps(1), input_mps(1));
            testCase.verifyGreaterThan(profile_mps(1), profile_mps(2));
            testCase.verifyGreaterThanOrEqual(requiredAx_mps2, ...
                limitingAxMin_mps2 - 1e-10);
            testCase.verifyLessThan(higherRequiredAx_mps2, ...
                higherLimitingAxMin_mps2);
            testCase.verifyFalse(info.segment_capability_violation(1));
            testCase.verifyEqual(info.violation_count, 0);
        end

        function testNonmonotonicCapabilityReturnsHighestReachableEntry( ...
                testCase)
            options = default_qss_options();
            options.finish_speed_mps = 10;
            track.s_m = [0; 15];
            track.ds_m = 15;
            track.kappa_1pm = [0; 0];
            track.is_closed = false;
            input_mps = [25; 10];
            lateralLimit_mps = [30; 30];
            ggv = backwardPassSegmentTest.makeNonmonotonicBrakingGgv( ...
                options.gravity_mps2);
            expectedEntry_mps = (-780 + sqrt(681040)) / 2;

            [profile_mps, ~, info] = backward_pass(track, ggv, ...
                input_mps, lateralLimit_mps, options);

            testCase.verifyEqual(profile_mps(1), expectedEntry_mps, ...
                AbsTol=2 * options.solver_tolerance_mps);
            testCase.verifyGreaterThan(profile_mps(1), 20);
            testCase.verifyFalse(info.segment_capability_violation(1));
            testCase.verifyEqual(info.violation_count, 0);
        end

        function testAyGridIslandReturnsHighestReachableEntry(testCase)
            options = default_qss_options();
            options.finish_speed_mps = 10;
            track.s_m = [0; 15];
            track.ds_m = 15;
            track.kappa_1pm = [0.00980665; 0];
            track.is_closed = false;
            input_mps = [30; 10];
            lateralLimit_mps = [30; 30];
            ggv = backwardPassSegmentTest.makeAyGridIslandGgv( ...
                options.gravity_mps2);
            expectedEntry_mps = sqrt(114300 / 217);

            [profile_mps, ~, info] = backward_pass(track, ggv, ...
                input_mps, lateralLimit_mps, options);

            testCase.verifyEqual(profile_mps(1), expectedEntry_mps, ...
                AbsTol=2 * options.solver_tolerance_mps);
            testCase.verifyGreaterThan(profile_mps(1), 22.5);
            testCase.verifyFalse(info.segment_capability_violation(1));
            testCase.verifyEqual(info.violation_count, 0);
        end
    end

    methods (Static, Access=private)
        function ggv = makeSpeedSensitiveGgv(gravity_mps2)
            ggv.v_mps = [0; 10; 15; 20; 30];
            ggv.ay_g = [-1, 0, 1];
            ggv.ax_max_g = ones(5, 3);
            axMin_mps2 = [-4; -4; -22/3; -10; -10];
            ggv.ax_min_g = repmat(axMin_mps2 / gravity_mps2, 1, 3);
            ggv.feasible = true(5, 3);
            ggv.ay_limit_pos_g = ones(5, 1);
            ggv.ay_limit_neg_g = -ones(5, 1);
            ggv.accel_limiter = repmat("synthetic_drive", 5, 3);
            ggv.brake_limiter = repmat("synthetic_brake", 5, 3);
            ggv.gravity_mps2 = gravity_mps2;
        end

        function ggv = makeNonmonotonicBrakingGgv(gravity_mps2)
            ggv.v_mps = [0; 10; 13; 14; 22; 23; 25; 30];
            ggv.ay_g = [-1, 0, 1];
            ggv.ax_max_g = ones(8, 3);
            axMin_mps2 = [-4; -4; -4; -30; -30; -4; -4; -4];
            ggv.ax_min_g = repmat(axMin_mps2 / gravity_mps2, 1, 3);
            ggv.feasible = true(8, 3);
            ggv.ay_limit_pos_g = ones(8, 1);
            ggv.ay_limit_neg_g = -ones(8, 1);
            ggv.accel_limiter = repmat("synthetic_drive", 8, 3);
            ggv.brake_limiter = repmat("synthetic_brake", 8, 3);
            ggv.gravity_mps2 = gravity_mps2;
        end

        function ggv = makeAyGridIslandGgv(gravity_mps2)
            ggv.v_mps = [0; 30];
            ggv.ay_g = [-1, 0, 0.24, 0.2640625, 0.29, ...
                0.47, 0.50625, 0.54, 1];
            ggv.ax_max_g = ones(2, numel(ggv.ay_g));
            axMin_mps2 = -4 * ones(1, numel(ggv.ay_g));
            axMin_mps2([4, 7]) = -30;
            ggv.ax_min_g = repmat( ...
                axMin_mps2 / gravity_mps2, 2, 1);
            ggv.feasible = true(2, numel(ggv.ay_g));
            ggv.ay_limit_pos_g = ones(2, 1);
            ggv.ay_limit_neg_g = -ones(2, 1);
            ggv.accel_limiter = repmat( ...
                "synthetic_drive", size(ggv.ax_min_g));
            ggv.brake_limiter = repmat( ...
                "synthetic_brake", size(ggv.ax_min_g));
            ggv.gravity_mps2 = gravity_mps2;
        end
    end
end
