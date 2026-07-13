classdef readRealGgvTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testSyntheticFileCreatesExpectedGrid(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            filename = fullfile(projectRoot, "data", "ggv_real", ...
                "synthetic_real_ggv.csv");

            ggv = read_real_ggv(filename);

            testCase.verifyEqual(ggv.v_mps, [0; 15; 30; 45], AbsTol=0);
            testCase.verifyEqual(ggv.ay_g, [-1.6, 0, 1.6], AbsTol=0);
            testCase.verifySize(ggv.ax_max_g, [4, 3]);
            testCase.verifyTrue(all(ggv.feasible, "all"));
        end
    end
end
