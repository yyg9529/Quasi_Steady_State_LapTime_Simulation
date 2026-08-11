classdef actualAyPowertrainIntegrationTest < matlab.unittest.TestCase
    properties
        Ggv
        Options
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.Options = default_qss_options();
            testCase.Options.start_speed_mps = 10;
            testCase.Options.finish_speed_mps = 10;
            testCase.Ggv = actualAyPowertrainIntegrationTest.makeGgv( ...
                testCase.Options.gravity_mps2);
        end
    end

    methods (Test)
        function testForwardPassQueriesActualAy(testCase)
            track.s_m = [0; 10];
            track.ds_m = 10;
            track.kappa_1pm = [0.5 * testCase.Options.gravity_mps2 / 100; 0];
            track.is_closed = false;
            vLat_mps = [20; 20];

            vFwd_mps = forward_pass( ...
                track, testCase.Ggv, vLat_mps, testCase.Options);
            representativeSpeed_mps = mean(vFwd_mps);
            ay_g = representativeSpeed_mps^2 * track.kappa_1pm(1) ...
                / testCase.Options.gravity_mps2;
            cap = interp_ggv(testCase.Ggv, ...
                representativeSpeed_mps, ay_g, testCase.Options);
            requiredAx_mps2 = (vFwd_mps(2)^2 - vFwd_mps(1)^2) ...
                / (2 * track.ds_m);

            testCase.verifyLessThanOrEqual(requiredAx_mps2, ...
                cap.ax_max_mps2 + testCase.Options.accel_tolerance_mps2);
            testCase.verifyGreaterThan(vFwd_mps(2), vFwd_mps(1));
        end

        function testBackwardPassQueriesActualAy(testCase)
            track.s_m = [0; 2];
            track.ds_m = 2;
            track.kappa_1pm = [0.5 * testCase.Options.gravity_mps2 / 100; 0];
            track.is_closed = false;
            vInput_mps = [10; 5];
            vLat_mps = [20; 20];

            [vFinal_mps, ~, info] = backward_pass(track, testCase.Ggv, ...
                vInput_mps, vLat_mps, testCase.Options);
            midpoint_mps = mean(vFinal_mps);
            entryAy_g = vFinal_mps(1)^2 * track.kappa_1pm(1) ...
                / testCase.Options.gravity_mps2;
            midpointAy_g = midpoint_mps^2 * track.kappa_1pm(1) ...
                / testCase.Options.gravity_mps2;
            entryCap = interp_ggv(testCase.Ggv, ...
                vFinal_mps(1), entryAy_g, testCase.Options);
            midpointCap = interp_ggv(testCase.Ggv, ...
                midpoint_mps, midpointAy_g, testCase.Options);
            limitingAxMin_mps2 = max( ...
                entryCap.ax_min_mps2, midpointCap.ax_min_mps2);
            requiredAx_mps2 = (vFinal_mps(2)^2 - vFinal_mps(1)^2) ...
                / (2 * track.ds_m);

            testCase.verifyGreaterThan(vFinal_mps(1), vFinal_mps(2));
            testCase.verifyGreaterThanOrEqual(requiredAx_mps2, ...
                limitingAxMin_mps2 - 1e-10);
            testCase.verifyFalse(info.segment_capability_violation(1));
        end
    end

    methods (Static, Access=private)
        function ggv = makeGgv(gravity_mps2)
            ggv.v_mps = [0; 20];
            ggv.ay_g = [-1, 0, 1];
            ggv.ax_max_g = repmat([0, 1, 0], 2, 1);
            ggv.ax_min_g = repmat([0, -1, 0], 2, 1);
            ggv.feasible = true(2, 3);
            ggv.ay_limit_pos_g = ones(2, 1);
            ggv.ay_limit_neg_g = -ones(2, 1);
            ggv.accel_limiter = repmat("synthetic_drive", 2, 3);
            ggv.brake_limiter = repmat("synthetic_brake", 2, 3);
            ggv.gravity_mps2 = gravity_mps2;
        end
    end
end
