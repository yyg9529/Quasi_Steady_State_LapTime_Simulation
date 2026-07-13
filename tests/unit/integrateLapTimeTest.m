classdef integrateLapTimeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testClosedConstantSpeed(testCase)
            track.s_m = [0; 10; 20; 30];
            track.ds_m = 10 * ones(4, 1);
            track.is_closed = true;
            speed_mps = 8 * ones(4, 1);

            [lapTime_s, segmentTime_s] = integrate_lap_time(track, speed_mps);

            testCase.verifyEqual(lapTime_s, 5, AbsTol=1e-12);
            testCase.verifyEqual(segmentTime_s, 1.25 * ones(4, 1), ...
                AbsTol=1e-12);
        end

        function testOpenConstantAccelerationFromRest(testCase)
            track.s_m = [0; 10];
            track.ds_m = 10;
            track.is_closed = false;
            speed_mps = [0; sqrt(20)];

            lapTime_s = integrate_lap_time(track, speed_mps);

            testCase.verifyEqual(lapTime_s, sqrt(20), AbsTol=1e-12);
        end

        function testZeroSpeedAtBothEndsErrors(testCase)
            track.s_m = [0; 1];
            track.ds_m = 1;
            track.is_closed = false;

            action = @() integrate_lap_time(track, [0; 0]);

            testCase.verifyError(action, "QSSLTS:ZeroSegmentSpeed");
        end
    end
end
