classdef readTrackCsvTest < matlab.unittest.TestCase
    properties
        ProjectRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts( ...
                fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testClosedTrackContract(testCase)
            filename = fullfile(testCase.ProjectRoot, "data", "track", ...
                "skidpad_constant_radius.csv");

            track = read_track_csv(filename);

            testCase.verifyTrue(track.is_closed);
            testCase.verifySize(track.s_m, [8, 1]);
            testCase.verifySize(track.ds_m, [8, 1]);
            testCase.verifyGreaterThan(track.ds_m, zeros(8, 1));
            testCase.verifyEqual(track.length_m, 16 * pi, AbsTol=1e-5);
            testCase.verifyEqual(track.kappa_1pm, 0.125 * ones(8, 1), ...
                AbsTol=1e-12);
            testCase.verifyTrue(track.closure_is_estimated);
        end

        function test_track_curvature_alias(testCase)
            temporaryFolder = string(tempname);
            mkdir(temporaryFolder);
            testCase.addTeardown(@() rmdir(temporaryFolder, "s"));
            filename = fullfile(temporaryFolder, "track.csv");
            writetable(table([0; 1; 3], [0; 1; 3], [0; 0; 0], ...
                [0.10; -0.20; 0.05], [6; 6; 6], false(3, 1), ...
                VariableNames=["s_m", "x_m", "y_m", ...
                "curvature_1_m", "track_width_m", "is_closed"]), ...
                filename);

            track = read_track_csv(filename);

            testCase.verifyEqual(track.kappa_1pm, [0.10; -0.20; 0.05], ...
                AbsTol=1e-12);
            testCase.verifyEqual(track.x_m, [0; 1; 3], AbsTol=1e-12);
            testCase.verifyEqual(track.y_m, zeros(3, 1), AbsTol=1e-12);
            testCase.verifyEqual(track.track_width_m, 6 * ones(3, 1), ...
                AbsTol=1e-12);
            testCase.verifyEqual(track.ds_m, [1; 2], AbsTol=1e-12);
            testCase.verifyEqual(track.length_m, 3, AbsTol=1e-12);
            testCase.verifyFalse(track.is_closed);
            testCase.verifyFalse(track.closure_is_estimated);
        end

        function test_closed_track_duplicate_endpoint(testCase)
            temporaryFolder = string(tempname);
            mkdir(temporaryFolder);
            testCase.addTeardown(@() rmdir(temporaryFolder, "s"));
            filename = fullfile(temporaryFolder, "closed_track.csv");
            writetable(table([0; 2; 5; 7], [0; 2; 0; 0], ...
                [0; 0; 3; 0], [0.10; 0.20; 0.30; 0.10], ...
                [5; 5.5; 6; 5], VariableNames=["s_m", "x_m", ...
                "y_m", "curvature_1_m", "track_width_m"]), filename);

            track = read_track_csv(filename);

            testCase.verifyTrue(track.is_closed);
            testCase.verifySize(track.s_m, [3, 1]);
            testCase.verifySize(track.ds_m, [3, 1]);
            testCase.verifyEqual(track.x_m, [0; 2; 0], AbsTol=1e-12);
            testCase.verifyEqual(track.y_m, [0; 0; 3], AbsTol=1e-12);
            testCase.verifyEqual(track.track_width_m, [5; 5.5; 6], ...
                AbsTol=1e-12);
            testCase.verifyEqual(track.length_m, 7, AbsTol=1e-12);
            testCase.verifyFalse(track.closure_is_estimated);
        end

        function test_exact_closing_segment(testCase)
            temporaryFolder = string(tempname);
            mkdir(temporaryFolder);
            testCase.addTeardown(@() rmdir(temporaryFolder, "s"));
            filename = fullfile(temporaryFolder, "exact_closure.csv");
            writetable(table([10; 12; 17; 26], [4; 6; 9; 4], ...
                [1; 2; 4; 1], [0.01; 0.02; 0.03; 0.01], ...
                8 * ones(4, 1), VariableNames=["s_m", "x_m", ...
                "y_m", "curvature_1_m", "track_width_m"]), filename);

            track = read_track_csv(filename);

            testCase.verifyEqual(track.ds_m, [2; 5; 9], AbsTol=1e-12);
            testCase.verifyEqual(track.length_m, 16, AbsTol=1e-12);
            testCase.verifyFalse(track.closure_is_estimated);
        end
    end
end
