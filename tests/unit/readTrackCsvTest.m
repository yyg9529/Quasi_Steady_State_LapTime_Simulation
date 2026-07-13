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
        end

        function testXYInputHasExplicitError(testCase)
            temporaryFolder = string(tempname);
            mkdir(temporaryFolder);
            testCase.addTeardown(@() rmdir(temporaryFolder, "s"));
            filename = fullfile(temporaryFolder, "xy.csv");
            writetable(table([0; 1], [0; 1], ...
                VariableNames=["x_m", "y_m"]), filename);

            action = @() read_track_csv(filename);

            testCase.verifyError(action, "QSSLTS:TrackXYNotImplemented");
        end
    end
end
