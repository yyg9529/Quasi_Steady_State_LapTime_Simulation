classdef fscDynamicEventIntegrationTest < matlab.unittest.TestCase
    properties
        ProjectRoot
        Models
        Options
        Vehicle
    end

    methods (TestClassSetup)
        function prepareCase(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "src"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "data"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
            testCase.Vehicle = vehicle_baseline();
            testCase.Models.tire = tire_simple_baseline();
            testCase.Models.powertrain = struct("enabled", false);
            testCase.Models.brake = brake_baseline();
            testCase.Options = default_qss_options(struct( ...
                v_max_mps=30, ...
                v_grid_mps=(0:1:30).', ...
                ay_grid_g=-2:0.2:2));
        end
    end

    methods (Test)
        function testAccelerationAssetStartsAtRestAndDoesNotBrake(testCase)
            track = testCase.readTrack("fsc_2025_acceleration_open.csv");

            result = run_qss_lap(track, testCase.Vehicle, ...
                testCase.Models, testCase.Options);
            event = struct(type="fsc_acceleration", ...
                rollout_distance_m=0.30, timed_distance_m=75);
            score = score_fsc_dynamic_event(result, event);

            testCase.verifyTrue(result.solver.converged);
            testCase.verifyEqual(result.v_mps(1), 0, AbsTol=1e-12);
            testCase.verifyGreaterThan(result.v_mps(end), 0);
            testCase.verifyGreaterThan(score.start_gate_speed_mps, 0);
            testCase.verifyLessThan(score.scored_time_s, ...
                score.path_time_s);
            testCase.verifyGreaterThanOrEqual(diff(result.v_mps), ...
                -1e-9 * ones(numel(result.v_mps) - 1, 1));
        end

        function testSkidpadAssetProducesFiniteTimedLaps(testCase)
            track = testCase.readTrack("fsc_2025_skidpad_event_open.csv");
            result = run_qss_lap(track, testCase.Vehicle, ...
                testCase.Models, testCase.Options);
            radius_m = 9.125;
            event = struct( ...
                type="fsc_skidpad", ...
                circle_length_m=2 * pi * radius_m, ...
                timed_laps=[2; 4], ...
                scoring_diameter_m=17.10);

            score = score_fsc_dynamic_event(result, event);

            testCase.verifyTrue(result.solver.converged);
            testCase.verifyGreaterThan(score.right_timed_lap_s, 0);
            testCase.verifyGreaterThan(score.left_timed_lap_s, 0);
            testCase.verifyEqual(score.scored_time_s, mean([ ...
                score.right_timed_lap_s, score.left_timed_lap_s]), ...
                AbsTol=1e-12);
            testCase.verifyTrue(any(result.ay_mps2 < 0));
            testCase.verifyTrue(any(result.ay_mps2 > 0));
        end
    end

    methods (Access = private)
        function track = readTrack(testCase, filename)
            track = read_track_csv(fullfile(testCase.ProjectRoot, ...
                "data", "track", filename));
        end
    end
end
