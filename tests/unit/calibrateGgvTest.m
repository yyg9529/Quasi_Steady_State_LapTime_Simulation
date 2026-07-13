classdef calibrateGgvTest < matlab.unittest.TestCase
    properties
        Model
        Options
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.Options = default_qss_options();
            testCase.Options.v_grid_mps = [0; 10; 20];
            testCase.Options.ay_grid_g = -2:0.25:2;
            testCase.Options.calibration_smoothing_window = 1;
            testCase.Model = generate_model_ggv(vehicle_baseline(), ...
                tire_simple_baseline(), struct("enabled", false), ...
                struct("enabled", false), brake_baseline(), testCase.Options);
        end
    end

    methods (Test)
        function testIdentityCalibrationPreservesMap(testCase)
            real = testCase.Model;
            real.source = "identity_fixture";

            [calibrated, report] = calibrate_ggv( ...
                testCase.Model, real, testCase.Options);

            testCase.verifyEqual(report.scale_table.scale_lat, ...
                ones(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(report.scale_table.scale_acc, ...
                ones(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(report.scale_table.scale_brake, ...
                ones(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(calibrated.ax_max_g, ...
                testCase.Model.ax_max_g, AbsTol=1e-12);
            testCase.verifyEqual(calibrated.ax_min_g, ...
                testCase.Model.ax_min_g, AbsTol=1e-12);
        end

        function testScaleFactorsAreBounded(testCase)
            real = testCase.Model;
            real.ay_limit_pos_g = 2 * real.ay_limit_pos_g;
            real.ay_limit_neg_g = 2 * real.ay_limit_neg_g;
            real.ax_max_g = 2 * real.ax_max_g;
            real.ax_min_g = 2 * real.ax_min_g;
            real.source = "overscale_fixture";

            [~, report] = calibrate_ggv(testCase.Model, real, testCase.Options);

            testCase.verifyLessThanOrEqual(report.scale_table.scale_lat, ...
                testCase.Options.scale_max * ones(3, 1));
            testCase.verifyLessThanOrEqual(report.scale_table.scale_acc, ...
                testCase.Options.scale_max * ones(3, 1));
            testCase.verifyLessThanOrEqual(report.scale_table.scale_brake, ...
                testCase.Options.scale_max * ones(3, 1));
        end

        function testReportCanBeSavedExplicitly(testCase)
            temporaryFolder = string(tempname);
            mkdir(temporaryFolder);
            testCase.addTeardown(@() rmdir(temporaryFolder, "s"));
            options = testCase.Options;
            options.calibration_report_file = fullfile( ...
                temporaryFolder, "report.csv");
            real = testCase.Model;
            real.source = "save_fixture";

            [~, report] = calibrate_ggv(testCase.Model, real, options);

            testCase.verifyTrue(isfile(report.file));
            saved = readtable(report.file);
            testCase.verifyEqual(height(saved), numel(testCase.Model.v_mps));
        end
    end
end
