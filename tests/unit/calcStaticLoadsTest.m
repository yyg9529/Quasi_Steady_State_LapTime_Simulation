classdef calcStaticLoadsTest < matlab.unittest.TestCase
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
        function testStaticLoadsConserveWeight(testCase)
            vehicle = vehicle_baseline();
            gravity_mps2 = 9.80665;

            loads = calc_static_loads(vehicle, gravity_mps2);

            wheelSum_N = loads.Fz_FL_N + loads.Fz_FR_N ...
                + loads.Fz_RL_N + loads.Fz_RR_N;
            testCase.verifyEqual(wheelSum_N, ...
                vehicle.mass.total_kg * gravity_mps2, AbsTol=1e-10);
            testCase.verifyEqual(loads.Fz_front_total_N / loads.Fz_total_N, ...
                vehicle.mass.front_static_frac, AbsTol=1e-12);
        end
    end
end
