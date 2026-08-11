classdef runQssValidityContractTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        Ggv
        Options
        Vehicle
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));

            testCase.Vehicle = vehicle_baseline();
            testCase.Options = default_qss_options();
            testCase.Options.v_max_mps = 35;
            testCase.Options.v_grid_mps = (0:0.5:35).';
            testCase.Options.ay_grid_g = -2:0.05:2;
            testCase.Ggv = generate_model_ggv(testCase.Vehicle, ...
                tire_simple_baseline(), struct("enabled", false), ...
                struct("enabled", false), brake_baseline(), ...
                testCase.Options);
        end
    end

    methods (Test)
        function testLapAndAssessmentUseSamePrimaryFailure(testCase)
            unhealthy = testCase.Ggv;
            feasibleIndex = find(unhealthy.feasible, 1);
            unhealthy.solve_converged_accel(feasibleIndex) = false;
            models.ggv = unhealthy;
            options = testCase.Options;
            options.solver_max_iterations = 1;
            options.solver_tolerance_mps = 0;

            track.s_m = [0; 8; 19; 26; 39; 48];
            track.ds_m = [8; 11; 7; 13; 9; 10];
            track.kappa_1pm = [0; 0.09; -0.04; 0.15; 0; -0.08];
            track.is_closed = true;

            result = run_qss_lap( ...
                track, testCase.Vehicle, models, options);
            assessment = assess_analysis_result(result);

            testCase.verifyFalse(result.propagation_converged);
            testCase.verifyFalse(result.ggv_healthy);
            testCase.verifyEqual(result.status, assessment.status);
            testCase.verifyEqual( ...
                result.reject_reason, assessment.reject_reason);
        end

        function testFrozenProjectionReportIsCanonicalResultMetadata( ...
                testCase)
            speeds = testCase.Ggv.v_mps;
            scaleTable = table(speeds, 1.2 * ones(size(speeds)), ...
                1.2 * ones(size(speeds)), 1.2 * ones(size(speeds)), ...
                VariableNames=["v_mps", "scale_lat", ...
                "scale_acc", "scale_brake"]);
            [projected, projectionReport] = ...
                apply_ggv_calibration_scales(testCase.Ggv, scaleTable);
            models.ggv = projected;

            radius_m = 8;
            pointCount = 16;
            segmentLength_m = 2 * pi * radius_m / pointCount;
            track.s_m = (0:pointCount-1).' * segmentLength_m;
            track.ds_m = segmentLength_m * ones(pointCount, 1);
            track.kappa_1pm = ones(pointCount, 1) / radius_m;
            track.is_closed = true;

            result = run_qss_lap(track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyTrue(isfield( ...
                result.calibration_report, "hard_limit_projection"));
            testCase.verifyEqual( ...
                result.calibration_report.hard_limit_projection, ...
                projectionReport);
        end

        function testNoFeasibleDomainFailsAtHealthGate(testCase)
            unusable = testCase.Ggv;
            unusable.feasible(:) = false;
            models.ggv = unusable;
            track = runQssValidityContractTest.makeCircleTrack();

            action = @() run_qss_lap(track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, "QSSLTS:GGVUnusable");
        end

        function testGravityMismatchFailsAtHealthGate(testCase)
            unusable = testCase.Ggv;
            unusable.gravity_mps2 = unusable.gravity_mps2 + 0.1;
            models.ggv = unusable;
            track = runQssValidityContractTest.makeCircleTrack();

            action = @() run_qss_lap(track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, "QSSLTS:GGVUnusable");
        end
    end

    methods (Static, Access = private)
        function track = makeCircleTrack()
            radius_m = 8;
            pointCount = 16;
            segmentLength_m = 2 * pi * radius_m / pointCount;
            track.s_m = (0:pointCount-1).' * segmentLength_m;
            track.ds_m = segmentLength_m * ones(pointCount, 1);
            track.kappa_1pm = ones(pointCount, 1) / radius_m;
            track.is_closed = true;
        end
    end
end
