classdef calcBrakeLimitTest < matlab.unittest.TestCase
    properties
        Tire
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
            testCase.Options = default_qss_options();
        end
    end

    methods (Test)
        function testBiasRespectsWeakFrontAxle(testCase)
            loads.Fx_available_N = [100; 100; 1000; 1000];
            loads.vehicle_mass_kg = 300;
            brake = struct("enabled", true, "front_bias", 0.6, ...
                "max_decel_g_mechanical", inf, ...
                "max_total_brake_force_N", inf);

            result = calc_brake_limit(20, loads, testCase.Tire, ...
                brake, testCase.Options);

            testCase.verifyEqual(result.Fx_brake_min_N, -200 / 0.6, ...
                AbsTol=1e-10);
            testCase.verifyLessThan(result.ax_min_mps2, 0);
            testCase.verifyEqual(result.limiter, "brake_traction_bias");
        end

        function testMechanicalDecelLimit(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            loads.vehicle_mass_kg = 300;
            brake = struct("enabled", true, "front_bias", 0.5, ...
                "max_decel_g_mechanical", 1.0, ...
                "max_total_brake_force_N", inf);

            result = calc_brake_limit(20, loads, testCase.Tire, ...
                brake, testCase.Options);

            testCase.verifyEqual(result.ax_min_mps2, ...
                -testCase.Options.gravity_mps2, AbsTol=1e-12);
            testCase.verifyEqual(result.limiter, "brake_mechanical");
        end

        function testInfiniteTorqueLimitDoesNotRequireTireRadius(testCase)
            loads.Fx_available_N = 1000 * ones(4, 1);
            loads.vehicle_mass_kg = 300;
            brake = struct("enabled", true, "front_bias", 0.5, ...
                "max_decel_g_mechanical", 1.0, ...
                "max_total_brake_force_N", inf);

            result = calc_brake_limit(20, loads, struct(), ...
                brake, testCase.Options);

            testCase.verifyEqual(result.ax_min_mps2, ...
                -testCase.Options.gravity_mps2, AbsTol=1e-12);
        end

        function testFiniteWheelTorqueLimitUsesRollingRadius(testCase)
            loads.Fx_available_N = 1e5 * ones(4, 1);
            loads.vehicle_mass_kg = 300;
            tire.rolling_radius_m = 0.25;
            brake = struct("enabled", true, "front_bias", 0.5, ...
                "max_decel_g_mechanical", inf, ...
                "max_total_brake_force_N", inf, ...
                "max_total_brake_torque_Nm", 1000);

            result = calc_brake_limit(20, loads, tire, ...
                brake, testCase.Options);

            testCase.verifyEqual(result.Fx_brake_min_N, -4000, ...
                AbsTol=1e-12);
            testCase.verifyEqual(result.limiter, "brake_mechanical");
        end
    end
end
