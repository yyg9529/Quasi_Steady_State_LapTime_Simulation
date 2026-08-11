classdef runDoeValidityTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        BaseConfig
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            track.s_m = [0; 10; 20; 30];
            track.ds_m = 10 * ones(4, 1);
            track.kappa_1pm = [0; 0.02; -0.02; 0];
            track.is_closed = true;
            testCase.BaseConfig.track = track;
            testCase.BaseConfig.vehicle = vehicle_baseline();
            testCase.BaseConfig.models.tire = tire_simple_baseline();
            testCase.BaseConfig.models.aero = struct("enabled", false);
            testCase.BaseConfig.models.powertrain = struct("enabled", false);
            testCase.BaseConfig.models.brake = brake_baseline();
            options = default_qss_options();
            options.v_max_mps = 20;
            options.v_grid_mps = (0:2:20).';
            options.ay_grid_g = -2:0.1:2;
            testCase.BaseConfig.options = options;
        end
    end

    methods (Test)
        function testExpectedInvalidCaseIsIsolatedFromRanking(testCase)
            cases = table([300; -1; 320], VariableNames="mass_kg");

            result = run_doe(testCase.BaseConfig, cases);

            testCase.verifyEqual(result.status, "partial");
            testCase.verifyEqual(result.valid_case_count, 2);
            testCase.verifyEqual(result.rejected_case_count, 1);
            testCase.verifyEqual(result.all_cases.valid, ...
                [true; false; true]);
            testCase.verifyEqual(result.rejected_cases.case_id, 2);
            testCase.verifyFalse(any(result.ranking.case_id == 2));
            testCase.verifyNotEqual(result.fastest_case_id, 2);
            testCase.verifyNotEqual(result.slowest_case_id, 2);
            testCase.verifyEqual(result.rejected_cases.reject_reason, ...
                "vehicle_parameters");
            testCase.verifyEmpty(fieldnames(result.case_results{2}));
        end

        function testNoValidCasesReturnEmptyDecisionProducts(testCase)
            cases = table([-1; -2], VariableNames="mass_kg");

            result = run_doe(testCase.BaseConfig, cases);

            testCase.verifyEqual(result.status, "no_valid_cases");
            testCase.verifyEqual(result.valid_case_count, 0);
            testCase.verifyEqual(result.rejected_case_count, 2);
            testCase.verifyEmpty(result.ranking);
            testCase.verifyTrue(isnan(result.fastest_case_id));
            testCase.verifyTrue(isnan(result.slowest_case_id));
            testCase.verifyEmpty(fieldnames(result.fastest_summary));
            testCase.verifyEmpty(result.extreme_limiter_comparison);
        end

        function testInvalidBaselineIsExplicitlyRejected(testCase)
            config = testCase.BaseConfig;
            config.vehicle.mass.total_kg = -1;
            cases = table(300, VariableNames="mass_kg");

            action = @() run_doe(config, cases);

            testCase.verifyError(action, "QSSLTS:DOEInvalidBaseline");
        end

        function testNumericallyInvalidBaselineIsExplicitlyRejected( ...
                testCase)
            config = testCase.BaseConfig;
            config.track.s_m = [0; 8; 19; 26; 39; 48];
            config.track.ds_m = [8; 11; 7; 13; 9; 10];
            config.track.kappa_1pm = ...
                [0; 0.09; -0.04; 0.15; 0; -0.08];
            config.options.solver_max_iterations = 1;
            config.options.solver_tolerance_mps = 0;
            cases = table(300, VariableNames="mass_kg");

            action = @() run_doe(config, cases);

            testCase.verifyError(action, "QSSLTS:DOEInvalidBaseline");
        end

        function testUnknownBaselineContractDefectIsRethrown(testCase)
            config = testCase.BaseConfig;
            config.track = rmfield(config.track, "ds_m");
            cases = table(300, VariableNames="mass_kg");

            action = @() run_doe(config, cases);

            testCase.verifyError(action, "QSSLTS:TrackFields");
        end

        function testUnknownCaseContractDefectIsRethrown(testCase)
            cases = table(1, VariableNames="track.ds_m");

            action = @() run_doe(testCase.BaseConfig, cases);

            testCase.verifyError(action, "QSSLTS:TrackSegments");
        end
    end
end
