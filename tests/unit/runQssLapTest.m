classdef runQssLapTest < matlab.unittest.TestCase
    properties
        Vehicle
        Models
        Options
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.Vehicle = vehicle_baseline();
            testCase.Models.tire = tire_simple_baseline();
            testCase.Models.aero = struct("enabled", false);
            testCase.Models.powertrain = struct("enabled", false);
            testCase.Models.brake = brake_baseline();
            testCase.Options = default_qss_options();
            testCase.Options.v_max_mps = 35;
            testCase.Options.v_grid_mps = (0:0.5:35).';
            testCase.Options.ay_grid_g = -2:0.05:2;
        end
    end

    methods (Test)
        function testConstantRadiusMatchesAnalyticSpeedAndTime(testCase)
            radius_m = 8;
            nPoint = 64;
            ds_m = 2 * pi * radius_m / nPoint;
            track.s_m = (0:nPoint-1).' * ds_m;
            track.ds_m = ds_m * ones(nPoint, 1);
            track.kappa_1pm = ones(nPoint, 1) / radius_m;
            track.is_closed = true;

            result = run_qss_lap(track, testCase.Vehicle, ...
                testCase.Models, testCase.Options);

            expectedSpeed_mps = sqrt(testCase.Models.tire.mu_y ...
                * testCase.Options.gravity_mps2 * radius_m);
            expectedTime_s = 2 * pi * radius_m / expectedSpeed_mps;
            testCase.verifyTrue(result.solver.converged);
            testCase.verifyEqual(result.v_mps, ...
                expectedSpeed_mps * ones(nPoint, 1), AbsTol=1e-7);
            testCase.verifyEqual(result.lap_time_s, expectedTime_s, ...
                AbsTol=1e-7);
        end

        function testClosedLapIsInvariantToArrayStart(testCase)
            track.s_m = [0; 8; 19; 26; 39; 48];
            track.ds_m = [8; 11; 7; 13; 9; 10];
            track.kappa_1pm = [0.00; 0.09; -0.04; 0.15; 0.00; -0.08];
            track.is_closed = true;
            rotated.ds_m = circshift(track.ds_m, -2);
            rotated.kappa_1pm = circshift(track.kappa_1pm, -2);
            rotated.s_m = [0; cumsum(rotated.ds_m(1:end-1))];
            rotated.is_closed = true;

            original = run_qss_lap(track, testCase.Vehicle, ...
                testCase.Models, testCase.Options);
            shifted = run_qss_lap(rotated, testCase.Vehicle, ...
                testCase.Models, testCase.Options);

            testCase.verifyTrue(original.solver.converged);
            testCase.verifyTrue(shifted.solver.converged);
            testCase.verifyEqual(shifted.lap_time_s, original.lap_time_s, ...
                AbsTol=1e-8);
            testCase.verifyEqual(shifted.v_mps, circshift(original.v_mps, -2), ...
                AbsTol=1e-7);
        end
    end
end
