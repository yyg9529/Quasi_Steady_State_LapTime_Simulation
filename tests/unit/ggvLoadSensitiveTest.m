classdef ggvLoadSensitiveTest < matlab.unittest.TestCase
    properties
        Options
        NoAero
        NoPower
        Brake
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.Options = default_qss_options();
            testCase.Options.v_grid_mps = [0; 15];
            testCase.Options.ay_grid_g = -2:0.1:2;
            testCase.NoAero = struct("enabled", false);
            testCase.NoPower = struct("enabled", false);
            testCase.Brake = brake_baseline();
        end
    end

    methods (Test)
        function testHigherCGLowersLoadSensitiveLateralLimit(testCase)
            lowVehicle = vehicle_baseline();
            lowVehicle.mass.cg_height_m = 0.05;
            highVehicle = lowVehicle;
            highVehicle.mass.cg_height_m = 0.30;
            tire = tire_load_sensitive_baseline();

            low = generate_model_ggv(lowVehicle, tire, testCase.NoAero, ...
                testCase.NoPower, testCase.Brake, testCase.Options);
            high = generate_model_ggv(highVehicle, tire, testCase.NoAero, ...
                testCase.NoPower, testCase.Brake, testCase.Options);

            testCase.verifyLessThan(high.ay_limit_pos_g(1), ...
                low.ay_limit_pos_g(1));
        end

        function testConstantMuIsInvariantToCGLateralTransfer(testCase)
            lowVehicle = vehicle_baseline();
            lowVehicle.mass.cg_height_m = 0.05;
            highVehicle = lowVehicle;
            highVehicle.mass.cg_height_m = 0.30;
            tire = tire_simple_baseline();

            low = generate_model_ggv(lowVehicle, tire, testCase.NoAero, ...
                testCase.NoPower, testCase.Brake, testCase.Options);
            high = generate_model_ggv(highVehicle, tire, testCase.NoAero, ...
                testCase.NoPower, testCase.Brake, testCase.Options);

            testCase.verifyEqual(high.ay_limit_pos_g, low.ay_limit_pos_g, ...
                AbsTol=1e-10);
        end

        function testLongitudinalFixedPointsConverge(testCase)
            ggv = generate_model_ggv(vehicle_baseline(), ...
                tire_load_sensitive_baseline(), testCase.NoAero, ...
                testCase.NoPower, testCase.Brake, testCase.Options);

            testCase.verifyTrue(all(ggv.solve_converged_accel(ggv.feasible)));
            testCase.verifyTrue(all(ggv.solve_converged_brake(ggv.feasible)));
            testCase.verifyLessThanOrEqual(max(abs( ...
                ggv.solve_residual_accel_mps2(ggv.feasible))), ...
                testCase.Options.ggv_tolerance_mps2);
            testCase.verifyLessThanOrEqual(max(abs( ...
                ggv.solve_residual_brake_mps2(ggv.feasible))), ...
                testCase.Options.ggv_tolerance_mps2);
        end
    end
end
