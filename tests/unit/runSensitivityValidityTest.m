classdef runSensitivityValidityTest < matlab.unittest.TestCase
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
        function testInvalidDesignIsExcludedFromSensitivityCurve(testCase)
            definition.parameter = "mass_kg";
            definition.values = [300; -1; 320];
            definition.make_plot = false;

            result = run_sensitivity_sweep( ...
                testCase.BaseConfig, definition);

            testCase.verifyEqual(result.status, "partial");
            testCase.verifyEqual(result.valid_case_count, 2);
            testCase.verifyEqual(result.rejected_case_count, 1);
            testCase.verifyEqual(result.table.parameter_value, [300; 320]);
            testCase.verifyEqual(result.rejected_cases.case_id, 2);
            testCase.verifyFalse(any(result.table.parameter_value == -1));
        end

        function testSensitivityPreservesRequestedValueOrder(testCase)
            definition.parameter = "tire_mu_scale";
            definition.values = [0.5; 1.0];
            definition.make_plot = false;

            result = run_sensitivity_sweep( ...
                testCase.BaseConfig, definition);

            testCase.verifyEqual(result.table.parameter_value, [0.5; 1.0], ...
                AbsTol=0);
        end
    end
end
