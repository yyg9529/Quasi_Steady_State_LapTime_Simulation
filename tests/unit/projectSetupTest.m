classdef projectSetupTest < matlab.unittest.TestCase
    methods (Test)
        function testNestedProductionFoldersAreVisible(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            originalPath = path;
            testCase.addTeardown(@() path(originalPath));

            addpath(projectRoot);
            returnedRoot = project_setup();

            testCase.verifyEqual(string(returnedRoot), string(projectRoot));
            testCase.verifyNotEmpty(which("run_qss_lap"));
            testCase.verifyNotEmpty(which("read_track_csv"));
        end
    end
end
