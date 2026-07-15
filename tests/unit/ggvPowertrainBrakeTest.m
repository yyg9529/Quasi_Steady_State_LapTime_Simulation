classdef ggvPowertrainBrakeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testTorqueAndPowerLimitersAppear(testCase)
            vehicle = vehicle_baseline();
            vehicle.drivetrain.layout = "AWD";
            tire = tire_simple_baseline();
            tire.mu_x = 5;
            powertrain = powertrain_baseline();
            powertrain.layout = "AWD";
            options = default_qss_options();
            options.v_grid_mps = [1; 30];
            options.ay_grid_g = [-0.1, 0, 0.1];

            ggv = generate_model_ggv(vehicle, tire, struct("enabled", false), ...
                powertrain, brake_baseline(), options);

            testCase.verifyEqual(ggv.accel_limiter(1, 2), "torque");
            testCase.verifyEqual(ggv.accel_limiter(2, 2), "power");
        end

        function testAwdHasMoreLowSpeedTractionThanRwd(testCase)
            rwdVehicle = vehicle_baseline();
            awdVehicle = rwdVehicle;
            awdVehicle.drivetrain.layout = "AWD";
            rwdPowertrain = powertrain_baseline();
            awdPowertrain = rwdPowertrain;
            awdPowertrain.layout = "AWD";
            rwdPowertrain.max_total_wheel_torque_Nm = inf;
            awdPowertrain.max_total_wheel_torque_Nm = inf;
            rwdPowertrain.max_power_W = inf;
            awdPowertrain.max_power_W = inf;
            options = default_qss_options();
            options.v_grid_mps = [0; 1];
            options.ay_grid_g = [-0.1, 0, 0.1];

            rwd = generate_model_ggv(rwdVehicle, tire_simple_baseline(), ...
                struct("enabled", false), rwdPowertrain, ...
                brake_baseline(), options);
            awd = generate_model_ggv(awdVehicle, tire_simple_baseline(), ...
                struct("enabled", false), awdPowertrain, ...
                brake_baseline(), options);

            testCase.verifyGreaterThan(awd.ax_max_g(1, 2), rwd.ax_max_g(1, 2));
        end

        function testCompositeLimitersAndCleanProvenance(testCase)
            vehicle = vehicle_baseline();
            tire = tire_simple_baseline();
            tire.mu_x = 100;
            powertrain = powertrain_emrax228_hvcc_demo();
            speed4500_mps = 4500 * 2 * pi / 60 ...
                * tire.rolling_radius_m / powertrain.gear_ratio;
            options = default_qss_options();
            options.v_grid_mps = [speed4500_mps; 120 / 3.6];
            options.ay_grid_g = [-0.1, 0, 0.1];

            ggv = generate_model_ggv(vehicle, tire, ...
                struct("enabled", false), powertrain, ...
                brake_baseline(), options);

            testCase.verifyEqual(ggv.accel_limiter(:, 2), ...
                ["motor_voltage"; "motor_speed"]);
            testCase.verifyEqual(sort(string(fieldnames( ...
                ggv.provenance.powertrain))), ...
                sort(string(fieldnames(powertrain))));
            testCase.verifyFalse(isfield(ggv.provenance.powertrain, ...
                "max_power_W"));
            testCase.verifyFalse(isfield(ggv.provenance.powertrain, ...
                "max_speed_mps"));
            testCase.verifyFalse(isfield(ggv.provenance.powertrain, ...
                "max_total_wheel_torque_Nm"));
            testCase.verifyFalse(isfield(ggv.provenance.powertrain, ...
                "drive_efficiency"));
        end

        function testActualAyTractionExceedsLateralBoundary(testCase)
            vehicle = vehicle_baseline();
            tire = tire_simple_baseline();
            powertrain = powertrain_emrax228_hvcc_demo();
            options = default_qss_options();
            options.v_grid_mps = [5; 6];
            options.ay_grid_g = -2:0.25:2;

            ggv = generate_model_ggv(vehicle, tire, ...
                struct("enabled", false), powertrain, ...
                brake_baseline(), options);
            zeroAy = interp_ggv(ggv, 5, 0, options);
            actualAy = interp_ggv(ggv, 5, 0.5, options);
            boundary = interp_ggv(ggv, 5, ...
                ggv.ay_limit_pos_g(1), options);

            testCase.verifyGreaterThan(actualAy.ax_max_mps2, ...
                boundary.ax_max_mps2);
            testCase.verifyGreaterThan(zeroAy.ax_max_mps2, ...
                actualAy.ax_max_mps2);
        end

        function testPartialCompositeConfigErrors(testCase)
            powertrain = rmfield( ...
                powertrain_emrax228_hvcc_demo(), "battery");

            action = @() generate_model_ggv(vehicle_baseline(), ...
                tire_simple_baseline(), struct("enabled", false), ...
                powertrain, brake_baseline(), default_qss_options());

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end
    end
end
