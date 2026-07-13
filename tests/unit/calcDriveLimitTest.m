classdef calcDriveLimitTest < matlab.unittest.TestCase
    properties
        Tire
        Powertrain
        Options
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.Tire = tire_simple_baseline();
            testCase.Powertrain = powertrain_baseline();
            testCase.Options = default_qss_options();
        end
    end

    methods (Test)
        function testLowSpeedTorqueLimit(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = testCase.Powertrain;
            powertrain.layout = "AWD";

            drive = calc_drive_limit(1, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            expected_N = powertrain.max_total_wheel_torque_Nm ...
                / testCase.Tire.rolling_radius_m;
            testCase.verifyEqual(drive.Fx_drive_max_N, expected_N, ...
                AbsTol=1e-10);
            testCase.verifyEqual(drive.limiter, "torque");
        end

        function testHighSpeedPowerLimit(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = testCase.Powertrain;
            powertrain.layout = "AWD";

            drive = calc_drive_limit(30, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            expected_N = powertrain.max_power_W ...
                * powertrain.drive_efficiency / 30;
            testCase.verifyEqual(drive.Fx_drive_max_N, expected_N, ...
                AbsTol=1e-10);
            testCase.verifyEqual(drive.limiter, "power");
        end

        function testRwdUsesRearTireCapacity(testCase)
            loads.Fx_available_N = [1000; 1000; 100; 100];

            drive = calc_drive_limit(1, loads, testCase.Tire, ...
                testCase.Powertrain, testCase.Options);

            testCase.verifyEqual(drive.Fx_drive_max_N, 200, AbsTol=1e-12);
            testCase.verifyEqual(drive.limiter, "traction");
        end

        function testZeroSpeedIsFinite(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = testCase.Powertrain;
            powertrain.layout = "AWD";

            drive = calc_drive_limit(0, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyTrue(isfinite(drive.Fx_drive_max_N));
        end
    end
end
