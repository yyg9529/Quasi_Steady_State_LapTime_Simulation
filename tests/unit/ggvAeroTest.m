classdef ggvAeroTest < matlab.unittest.TestCase
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
        function testDownforceRaisesHighSpeedLateralCapability(testCase)
            options = default_qss_options();
            options.v_grid_mps = [0; 30];
            options.ay_grid_g = -3:0.05:3;
            vehicle = vehicle_baseline();
            tire = tire_simple_baseline();
            noAero = aero_baseline();
            noAero.enabled = false;

            baseline = generate_model_ggv(vehicle, tire, noAero, ...
                struct("enabled", false), brake_baseline(), options);
            withAero = generate_model_ggv(vehicle, tire, aero_baseline(), ...
                struct("enabled", false), brake_baseline(), options);

            testCase.verifyEqual(withAero.ay_limit_pos_g(1), ...
                baseline.ay_limit_pos_g(1), AbsTol=1e-12);
            testCase.verifyGreaterThan(withAero.ay_limit_pos_g(2), ...
                baseline.ay_limit_pos_g(2));
        end
    end
end
