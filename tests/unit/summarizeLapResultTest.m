classdef summarizeLapResultTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testLimiterPercentagesAreDistanceWeighted(testCase)
            result.options.gravity_mps2 = 10;
            result.track.ds_m = [1; 9];
            result.limiter = ["lateral"; "power"];
            result.lap_time_s = 5;
            result.v_mps = [10; 20];
            result.ax_mps2 = [5; -10];
            result.ay_mps2 = [0; 20];

            summary = summarize_lap_result(result);

            testCase.verifyEqual(summary.percent_lateral_limited, 10, ...
                AbsTol=1e-12);
            testCase.verifyEqual(summary.percent_power_limited, 90, ...
                AbsTol=1e-12);
            testCase.verifyEqual(summary.max_brake_g, 1, AbsTol=1e-12);
            testCase.verifyEqual(summary.max_ay_g, 2, AbsTol=1e-12);
        end
    end
end
