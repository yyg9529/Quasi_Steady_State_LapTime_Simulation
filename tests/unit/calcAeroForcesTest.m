classdef calcAeroForcesTest < matlab.unittest.TestCase
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
        function testZeroSpeedProducesZeroForces(testCase)
            force = calc_aero_forces(0, aero_baseline());

            testCase.verifyEqual(force.drag_N, 0, AbsTol=0);
            testCase.verifyEqual(force.downforce_total_N, 0, AbsTol=0);
        end

        function testForcesScaleWithSpeedSquared(testCase)
            aero = aero_baseline();

            low = calc_aero_forces(10, aero);
            high = calc_aero_forces(20, aero);

            testCase.verifyEqual(high.drag_N, 4 * low.drag_N, AbsTol=1e-10);
            testCase.verifyEqual(high.downforce_total_N, ...
                4 * low.downforce_total_N, AbsTol=1e-10);
            testCase.verifyEqual(high.downforce_front_N ...
                + high.downforce_rear_N, high.downforce_total_N, AbsTol=1e-10);
        end
    end
end
