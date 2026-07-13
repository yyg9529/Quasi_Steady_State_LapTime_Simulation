classdef constantRadiusSkidpadTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
        end
    end

    methods (Test, TestTags={'Benchmark'})
        function testAnalyticConstantMuCircle(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            track = read_track_csv(fullfile(projectRoot, "data", "track", ...
                "skidpad_constant_radius.csv"));
            vehicle = vehicle_baseline();
            models.tire = tire_simple_baseline();
            models.powertrain = struct("enabled", false);
            models.brake = brake_baseline();
            options = default_qss_options();
            options.v_max_mps = 30;
            options.v_grid_mps = (0:0.25:30).';

            result = run_qss_lap(track, vehicle, models, options);

            radius_m = 1 / track.kappa_1pm(1);
            expectedSpeed_mps = sqrt(models.tire.mu_y ...
                * options.gravity_mps2 * radius_m);
            relativeError = abs(mean(result.v_mps) - expectedSpeed_mps) ...
                / expectedSpeed_mps;
            testCase.verifyLessThan(relativeError, 5e-3);
        end
    end
end
