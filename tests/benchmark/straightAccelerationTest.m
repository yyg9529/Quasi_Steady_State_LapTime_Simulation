classdef straightAccelerationTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test, TestTags={'Benchmark'})
        function testOpenStraightAccelerationProfile(testCase)
            track.s_m = (0:10:2000).';
            track.ds_m = 10 * ones(200, 1);
            track.kappa_1pm = zeros(201, 1);
            track.is_closed = false;
            models = struct();
            models.tire = tire_simple_baseline();
            models.aero = struct("enabled", false);
            models.powertrain = powertrain_baseline();
            models.powertrain.max_power_W = 20000;
            models.powertrain.max_total_wheel_torque_Nm = 400;
            models.brake = brake_baseline();
            options = default_qss_options();
            options.v_max_mps = 50;
            options.v_grid_mps = (0:1:50).';
            options.ay_grid_g = -2:0.1:2;
            options.start_speed_mps = 0;

            result = run_qss_lap(track, vehicle_baseline(), models, options);

            testCase.verifyEqual(result.v_mps(1), 0, AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(diff(result.v_mps), ...
                -1e-9 * ones(200, 1));
            testCase.verifyEqual(max(result.v_mps), ...
                models.powertrain.max_speed_mps, AbsTol=1e-5);
            testCase.verifyTrue(any(result.limiter == "torque"));
            testCase.verifyTrue(any(result.limiter == "power"));
            testCase.verifyTrue(any(result.limiter == "top_speed"));
        end
    end
end
