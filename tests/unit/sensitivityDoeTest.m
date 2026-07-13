classdef sensitivityDoeTest < matlab.unittest.TestCase
    properties
        BaseConfig
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            track.s_m = [0; 20; 40; 60; 80; 100];
            track.ds_m = 20 * ones(6, 1);
            track.kappa_1pm = [0; 0.08; 0.08; 0; -0.10; -0.04];
            track.is_closed = true;
            testCase.BaseConfig.track = track;
            testCase.BaseConfig.vehicle = vehicle_baseline();
            testCase.BaseConfig.models.tire = tire_load_sensitive_baseline();
            testCase.BaseConfig.models.aero = struct("enabled", false);
            testCase.BaseConfig.models.powertrain = powertrain_baseline();
            testCase.BaseConfig.models.brake = brake_baseline();
            options = default_qss_options();
            options.v_grid_mps = [0; 10; 20; 30; 40; 45];
            options.ay_grid_g = -2:0.2:2;
            options.calibration_smoothing_window = 1;
            testCase.BaseConfig.options = options;
        end
    end

    methods (Test)
        function testMassSweepReturnsPhysicalOrdering(testCase)
            sweep.parameter = "vehicle.mass.total_kg";
            sweep.values = [280; 300; 320];

            result = run_sensitivity_sweep(testCase.BaseConfig, sweep);

            testCase.verifyEqual(height(result.table), 3);
            testCase.verifyEqual(result.calibration_mode, "theory_only");
            testCase.verifyGreaterThan(result.table.lap_time_s(3), ...
                result.table.lap_time_s(1));
        end

        function testDoeRunsTwentyCasesAndSorts(testCase)
            mass_kg = repelem([280; 300; 320; 340], 5);
            max_power_W = repmat((60000:10000:100000).', 4, 1);
            cases = table(mass_kg, max_power_W);

            result = run_doe(testCase.BaseConfig, cases);

            testCase.verifyEqual(height(result.ranking), 20);
            testCase.verifyTrue(all(isfinite(result.ranking.lap_time_s)));
            testCase.verifyGreaterThanOrEqual(diff(result.ranking.lap_time_s), ...
                -1e-12 * ones(19, 1));
            testCase.verifyGreaterThanOrEqual(result.fastest_case_id, 1);
            testCase.verifyLessThanOrEqual(result.slowest_case_id, 20);
        end

        function testRealCalibrationIsFrozenAtBaseline(testCase)
            baseConfig = testCase.BaseConfig;
            baselineGgv = generate_model_ggv(baseConfig.vehicle, ...
                baseConfig.models.tire, baseConfig.models.aero, ...
                baseConfig.models.powertrain, baseConfig.models.brake, ...
                baseConfig.options);
            baselineGgv.source = "identity_real_fixture";
            baseConfig.models.ggv_real = baselineGgv;
            sweep.parameter = "vehicle.mass.total_kg";
            sweep.values = [280; 320];

            result = run_sensitivity_sweep(baseConfig, sweep);

            testCase.verifyEqual(result.calibration_mode, "frozen_baseline");
            testCase.verifyNotEqual(result.table.lap_time_s(1), ...
                result.table.lap_time_s(2));
        end
    end
end
