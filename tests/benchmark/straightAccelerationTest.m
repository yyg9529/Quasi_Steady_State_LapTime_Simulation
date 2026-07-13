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
            track.s_m = (0:5:150).';
            track.ds_m = 5 * ones(30, 1);
            track.kappa_1pm = zeros(31, 1);
            track.is_closed = false;
            models.tire = tire_simple_baseline();
            models.aero = struct("enabled", false);
            models.powertrain = powertrain_baseline();
            models.brake = brake_baseline();
            options = default_qss_options();
            options.v_grid_mps = (0:1:45).';
            options.ay_grid_g = -2:0.1:2;
            options.start_speed_mps = 0;

            result = run_qss_lap(track, vehicle_baseline(), models, options);

            testCase.verifyEqual(result.v_mps(1), 0, AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(diff(result.v_mps), ...
                -1e-9 * ones(30, 1));
            testCase.verifyLessThanOrEqual(max(result.v_mps), ...
                options.v_max_mps + 1e-9);
            testCase.verifyTrue(any(result.limiter == "power"));
        end
    end
end
