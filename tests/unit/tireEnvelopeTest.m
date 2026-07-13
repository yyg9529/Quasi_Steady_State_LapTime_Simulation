classdef tireEnvelopeTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testConstantMuScalesWithLoad(testCase)
            tire = tire_simple_baseline();

            env = tire_envelope([0; 500; 1000], 0, tire);

            testCase.verifyEqual(env.Fx_max_N, [0; 750; 1500], ...
                AbsTol=1e-12);
            testCase.verifyEqual(env.Fy_max_N, [0; 750; 1500], ...
                AbsTol=1e-12);
        end

        function testNonpositiveLoadProducesZeroForce(testCase)
            tire = tire_simple_baseline();

            env = tire_envelope([-10; 0], 0, tire);

            testCase.verifyEqual(env.Fx_max_N, zeros(2, 1), AbsTol=0);
            testCase.verifyEqual(env.Fy_max_N, zeros(2, 1), AbsTol=0);
        end
    end
end
