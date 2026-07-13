classdef tireLoadSensitiveTest < matlab.unittest.TestCase
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
        function testMuFallsButForceRisesWithLoad(testCase)
            tire = tire_load_sensitive_baseline();

            env = tire_envelope([750; 1500], 0, tire);

            testCase.verifyGreaterThan(env.mu_y(1), env.mu_y(2));
            testCase.verifyGreaterThan(env.Fy_max_N(2), env.Fy_max_N(1));
        end

        function testUnevenLoadsReduceTotalCapacity(testCase)
            tire = tire_load_sensitive_baseline();

            uniform = tire_envelope(750 * ones(4, 1), 0, tire);
            uneven = tire_envelope([0; 1500; 0; 1500], 0, tire);

            testCase.verifyLessThan(sum(uneven.Fy_max_N), ...
                sum(uniform.Fy_max_N));
        end

        function testCombinedSlipEndpoints(testCase)
            longitudinalOnly_N = tire_combined_simple(1000, 800, 0, 2);
            lateralBoundary_N = tire_combined_simple(1000, 800, 800, 2);

            testCase.verifyEqual(longitudinalOnly_N, 1000, AbsTol=1e-12);
            testCase.verifyEqual(lateralBoundary_N, 0, AbsTol=1e-12);
        end
    end
end
