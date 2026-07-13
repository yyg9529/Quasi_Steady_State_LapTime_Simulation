classdef straightBrakingTest < matlab.unittest.TestCase
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
        function testOpenStraightBrakingProfile(testCase)
            track.s_m = (0:5:100).';
            track.ds_m = 5 * ones(20, 1);
            track.kappa_1pm = zeros(21, 1);
            track.is_closed = false;
            options = default_qss_options();
            options.ay_grid_g = -2:0.1:2;
            options.finish_speed_mps = 0;
            options.v_grid_mps = (0:1:50).';
            vehicle = vehicle_baseline();
            vehicle.mass.cg_height_m = 0;
            brake = brake_baseline();
            brake.max_decel_g_mechanical = 1;
            brake.max_total_brake_force_N = inf;
            brake.max_total_brake_torque_Nm = inf;
            brake.front_bias = 0.5;
            ggv = generate_model_ggv(vehicle, ...
                tire_simple_baseline(), struct("enabled", false), ...
                struct("enabled", false), brake, options);
            inputSpeed_mps = 50 * ones(21, 1);
            lateralLimit_mps = 50 * ones(21, 1);

            [brakingSpeed_mps, ~, info] = backward_pass(track, ggv, ...
                inputSpeed_mps, lateralLimit_mps, options);
            expectedSpeed_mps = sqrt(2 * options.gravity_mps2 ...
                * (track.s_m(end) - track.s_m));

            testCase.verifyEqual(brakingSpeed_mps(end), 0, AbsTol=1e-12);
            testCase.verifyEqual(brakingSpeed_mps, expectedSpeed_mps, ...
                AbsTol=1e-8);
            testCase.verifyTrue(info.converged);
        end
    end
end
