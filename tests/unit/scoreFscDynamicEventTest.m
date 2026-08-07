classdef scoreFscDynamicEventTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testAccelerationUsesOpenCourseTime(testCase)
            result.lap_time_s = 4.5;
            result.track.s_m = [0; 0.3; 75.3];
            result.cumulative_time_s = [0; 0.4; 4.5];
            result.v_mps = [0; 2; 30];
            event.type = "fsc_acceleration";
            event.rollout_distance_m = 0.30;
            event.timed_distance_m = 75;

            score = score_fsc_dynamic_event(result, event);

            testCase.verifyEqual(score.scored_time_s, 4.1, ...
                AbsTol=1e-12);
            testCase.verifyEqual(score.path_time_s, 4.5, AbsTol=1e-12);
            testCase.verifyEqual(score.start_gate_speed_mps, 2, ...
                AbsTol=1e-12);
            testCase.verifyEqual(score.timed_distance_m, 75);
        end

        function testSkidpadUsesSecondRightAndSecondLeftLaps(testCase)
            circleLength_m = 2 * pi * 9.125;
            result.track.s_m = 10 + (0:4).' * circleLength_m;
            result.cumulative_time_s = [0; 5; 11; 18; 26];
            event.type = "fsc_skidpad";
            event.circle_length_m = circleLength_m;
            event.scoring_diameter_m = 17.10;
            event.timed_laps = [2; 4];

            score = score_fsc_dynamic_event(result, event);

            testCase.verifyEqual(score.right_timed_lap_s, 6, ...
                AbsTol=1e-12);
            testCase.verifyEqual(score.left_timed_lap_s, 8, ...
                AbsTol=1e-12);
            testCase.verifyEqual(score.scored_time_s, 7, ...
                AbsTol=1e-12);
            testCase.verifyEqual(score.right_lateral_accel_g, ...
                2.012 * 17.10 / 6^2, AbsTol=1e-12);
            testCase.verifyEqual(score.left_lateral_accel_g, ...
                2.012 * 17.10 / 8^2, AbsTol=1e-12);
        end

        function testSkidpadRejectsWrongTimedLaps(testCase)
            circleLength_m = 2 * pi * 9.125;
            result.track.s_m = (0:4).' * circleLength_m;
            result.cumulative_time_s = (0:4).';
            event = struct(type="fsc_skidpad", ...
                circle_length_m=circleLength_m, ...
                scoring_diameter_m=17.10, timed_laps=[4; 2]);

            action = @() score_fsc_dynamic_event(result, event);

            testCase.verifyError(action, "QSSLTS:DynamicEventLaps");
        end
    end
end
