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
            testCase.Powertrain = powertrain_emrax228_hvcc_demo();
            testCase.Options = default_qss_options();
        end
    end

    methods (Test)
        function testLowSpeedCompositeLimit(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = testCase.Powertrain;
            capability = evaluate_powertrain_constraints(1, ...
                testCase.Tire, powertrain, ...
                powertrain.battery.V_bus_assumed_V);

            drive = calc_drive_limit(1, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyEqual(drive.Fx_drive_max_N, ...
                capability.available_wheel_force_N, ...
                AbsTol=1e-10);
            testCase.verifyEqual(drive.limiter, capability.limiter);
        end

        function testHighSpeedCompositeLimit(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = testCase.Powertrain;
            capability = evaluate_powertrain_constraints(30, ...
                testCase.Tire, powertrain, ...
                powertrain.battery.V_bus_assumed_V);

            drive = calc_drive_limit(30, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyEqual(drive.Fx_drive_max_N, ...
                capability.available_wheel_force_N, ...
                AbsTol=1e-10);
            testCase.verifyEqual(drive.limiter, capability.limiter);
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

            drive = calc_drive_limit(0, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyTrue(isfinite(drive.Fx_drive_max_N));
        end

        function testCompositeMotorSpeedCutoffReturnsCapability(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = powertrain_emrax228_hvcc_demo();

            drive = calc_drive_limit(120 / 3.6, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyEqual(drive.Fx_drive_max_N, 0, AbsTol=1e-9);
            testCase.verifyEqual(drive.powertrain.motor_speed_rpm, 6084, ...
                AbsTol=1e-10);
            testCase.verifyEqual(drive.powertrain_force_limit_N, 0, ...
                AbsTol=1e-9);
            testCase.verifyEqual(drive.limiter, "motor_speed");
        end

        function testCompositeVoltageLimiterAt4500Rpm(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = powertrain_emrax228_hvcc_demo();
            speed_mps = 4500 * 2 * pi / 60 ...
                * testCase.Tire.rolling_radius_m / powertrain.gear_ratio;

            drive = calc_drive_limit(speed_mps, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyEqual(drive.limiter, "motor_voltage");
            testCase.verifyEqual(drive.Fx_drive_max_N, ...
                drive.powertrain.available_wheel_force_N, AbsTol=1e-9);
        end

        function testCompositeTieKeepsInternalLimiter(testCase)
            powertrain = powertrain_emrax228_hvcc_demo();
            capability = evaluate_powertrain_constraints(1, ...
                testCase.Tire, powertrain, ...
                powertrain.battery.V_bus_assumed_V);
            loads.Fx_available_N = [1e5; 1e5; ...
                capability.available_wheel_force_N / 2; ...
                capability.available_wheel_force_N / 2];

            drive = calc_drive_limit(1, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyEqual(drive.traction_limit_N, ...
                capability.available_wheel_force_N, AbsTol=1e-9);
            testCase.verifyEqual(drive.limiter, capability.limiter);
        end

        function testDisabledCompositeKeepsIdealTireLimit(testCase)
            loads.Fx_available_N = [100; 200; 300; 400];
            powertrain = powertrain_emrax228_hvcc_demo();
            powertrain.enabled = false;

            drive = calc_drive_limit(10, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyEqual(drive.Fx_drive_max_N, 1000, AbsTol=0);
            testCase.verifyEqual(drive.limiter, "tire");
        end

        function testPartialCompositeConfigErrors(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            powertrain = rmfield( ...
                powertrain_emrax228_hvcc_demo(), "battery");

            action = @() calc_drive_limit(10, loads, testCase.Tire, ...
                powertrain, testCase.Options);

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end
    end
end
