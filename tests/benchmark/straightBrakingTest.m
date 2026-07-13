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
            options.v_grid_mps = (0:1:40).';
            options.ay_grid_g = -2:0.1:2;
            options.finish_speed_mps = 0;
            ggv = generate_model_ggv(vehicle_baseline(), ...
                tire_simple_baseline(), struct("enabled", false), ...
                struct("enabled", false), brake_baseline(), options);
            inputSpeed_mps = 30 * ones(21, 1);
            lateralLimit_mps = 40 * ones(21, 1);

            brakingSpeed_mps = backward_pass(track, ggv, ...
                inputSpeed_mps, lateralLimit_mps, options);

            testCase.verifyEqual(brakingSpeed_mps(end), 0, AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(diff(brakingSpeed_mps), ...
                1e-9 * ones(20, 1));
            testCase.verifyGreaterThanOrEqual(brakingSpeed_mps, zeros(21, 1));
            testCase.verifyGreaterThan(brakingSpeed_mps(1), 0);
        end
    end
end
