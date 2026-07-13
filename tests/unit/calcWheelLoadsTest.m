classdef calcWheelLoadsTest < matlab.unittest.TestCase
    properties
        Vehicle
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.Vehicle = vehicle_baseline();
        end
    end

    methods (Test)
        function testStaticStateConservesWeight(testCase)
            state = make_vehicle_state(0, 0, 0);

            loads = calc_wheel_loads(state, testCase.Vehicle);

            testCase.verifyEqual(loads.Fz_total_N, ...
                testCase.Vehicle.mass.total_kg * 9.80665, AbsTol=1e-10);
            testCase.verifyFalse(loads.has_wheel_lift);
        end

        function testAccelerationTransfersLoadRearward(testCase)
            staticLoads = calc_wheel_loads( ...
                make_vehicle_state(0, 0, 0), testCase.Vehicle);
            accelLoads = calc_wheel_loads( ...
                make_vehicle_state(10, 3, 0), testCase.Vehicle);

            testCase.verifyLessThan(accelLoads.Fz_front_total_N, ...
                staticLoads.Fz_front_total_N);
            testCase.verifyGreaterThan(accelLoads.Fz_rear_total_N, ...
                staticLoads.Fz_rear_total_N);
        end

        function testLeftTurnLoadsRightWheels(testCase)
            loads = calc_wheel_loads( ...
                make_vehicle_state(10, 0, 8), testCase.Vehicle);

            testCase.verifyGreaterThan(loads.Fz_FR_N, loads.Fz_FL_N);
            testCase.verifyGreaterThan(loads.Fz_RR_N, loads.Fz_RL_N);
        end

        function testWheelLiftSaturationPreservesTotalLoad(testCase)
            state = make_vehicle_state(10, 0, 50);
            state.suppress_warnings = true;

            loads = calc_wheel_loads(state, testCase.Vehicle);

            testCase.verifyTrue(loads.has_wheel_lift);
            testCase.verifyGreaterThanOrEqual(loads.Fz_vector_N, zeros(4, 1));
            testCase.verifyEqual(loads.Fz_total_N, ...
                testCase.Vehicle.mass.total_kg * 9.80665, AbsTol=1e-10);
        end
    end
end
