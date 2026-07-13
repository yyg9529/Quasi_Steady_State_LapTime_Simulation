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
    end
end
