classdef activeConstraintsTest < matlab.unittest.TestCase
    properties
        Options
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.Options = default_qss_options();
        end
    end

    methods (Test)
        function testExactFieldsAndLateralBrakePriority(testCase)
            ggv = activeConstraintsTest.makeGgv();
            powertrain = activeConstraintsTest.makePowertrainResult(3);
            powertrain.motor_torque_used_Nm(1) = 100;
            powertrain.candidate_torque_available_Nm(1, 11) = 100;
            speed_mps = [10; 10; 20];
            ax_mps2 = [testCase.Options.gravity_mps2; -1; ...
                -testCase.Options.gravity_mps2];
            ay_mps2 = zeros(3, 1);
            lateralSpeed_mps = [10; 20; 20];
            lateralLimiter = ["lateral"; "top_speed"; "top_speed"];

            [active, limiter] = classify_active_constraints( ...
                speed_mps, ax_mps2, ay_mps2, lateralSpeed_mps, ...
                lateralLimiter, ggv, powertrain, testCase.Options);

            expectedFields = sort(["lateral", "brake", "traction", ...
                "motor_torque", "motor_power", "motor_speed", ...
                "motor_voltage", "rule_power", "rule_current", ...
                "battery_power", "battery_current", "inverter_power", ...
                "inverter_current", "top_speed"].');
            testCase.verifyEqual(sort(string(fieldnames(active))), ...
                expectedFields);
            testCase.verifyTrue(all(structfun( ...
                @(value) islogical(value) && isequal(size(value), [3 1]), ...
                active)));
            testCase.verifyTrue(active.lateral(1));
            testCase.verifyTrue(active.motor_torque(1));
            testCase.verifyEqual(limiter(1), "lateral");
            testCase.verifyFalse(active.brake(2));
            testCase.verifyTrue(active.brake(3));
            testCase.verifyTrue(active.top_speed(3));
            testCase.verifyEqual(limiter(3), "brake");
        end

        function testPowertrainCandidateUtilizationAndPriority(testCase)
            ggv = activeConstraintsTest.makeGgv();
            powertrain = activeConstraintsTest.makePowertrainResult(1);
            powertrain.motor_speed_rpm = 1000;
            powertrain.motor_stop_speed_rpm = 1000;
            powertrain.motor_torque_used_Nm = 100;
            powertrain.candidate_torque_available_Nm = 100 * ones(1, 12);
            powertrain.wheel_force_used_N = 100;
            powertrain.traction_force_available_N = 100;

            [active, limiter] = classify_active_constraints(10, 0, 0, ...
                20, "top_speed", ggv, powertrain, testCase.Options);

            expectedPowertrainActive = [active.traction; ...
                active.motor_torque; active.motor_power; ...
                active.motor_speed; active.motor_voltage; ...
                active.rule_power; active.rule_current; ...
                active.battery_power; active.battery_current; ...
                active.inverter_power; active.inverter_current];
            testCase.verifyTrue(all(expectedPowertrainActive));
            testCase.verifyFalse(active.lateral);
            testCase.verifyFalse(active.brake);
            testCase.verifyFalse(active.top_speed);
            testCase.verifyEqual(limiter, "rule_power");
        end

        function testUtilizationToleranceInsideAndOutside(testCase)
            ggv = activeConstraintsTest.makeGgv();
            powertrain = activeConstraintsTest.makePowertrainResult(2);
            powertrain.motor_torque_used_Nm = [99.95; 99.8];
            powertrain.candidate_torque_available_Nm = 100 * ones(2, 12);

            [active, ~] = classify_active_constraints([10; 10], ...
                [0; 0], [0; 0], [20; 20], ...
                ["top_speed"; "top_speed"], ggv, powertrain, ...
                testCase.Options);

            testCase.verifyEqual(active.rule_power, [true; false]);
            testCase.verifyEqual(active.motor_torque, [true; false]);
        end

        function testLateralSpeedToleranceInsideAndOutside(testCase)
            ggv = activeConstraintsTest.makeGgv();
            powertrain = activeConstraintsTest.makePowertrainResult(2);
            speed_mps = [9.981; 9.979];

            [active, ~] = classify_active_constraints(speed_mps, ...
                [0; 0], [0; 0], [10; 10], ["lateral"; "lateral"], ...
                ggv, powertrain, testCase.Options);

            testCase.verifyEqual(active.lateral, [true; false]);
        end

        function testBrakeAccelerationToleranceInsideAndOutside(testCase)
            ggv = activeConstraintsTest.makeGgv();
            powertrain = activeConstraintsTest.makePowertrainResult(2);
            gravity_mps2 = testCase.Options.gravity_mps2;
            ax_mps2 = [-gravity_mps2 + 0.019; ...
                -gravity_mps2 + 0.021];

            [active, ~] = classify_active_constraints([10; 10], ...
                ax_mps2, [0; 0], [20; 20], ...
                ["top_speed"; "top_speed"], ggv, powertrain, ...
                testCase.Options);

            testCase.verifyEqual(active.brake, [true; false]);
        end

        function testTopSpeedToleranceInsideAndOutside(testCase)
            ggv = activeConstraintsTest.makeGgv();
            powertrain = activeConstraintsTest.makePowertrainResult(2);
            speed_mps = [19.971; 19.969];

            [active, ~] = classify_active_constraints(speed_mps, ...
                [0; 0], [0; 0], [25; 25], ...
                ["top_speed"; "top_speed"], ggv, powertrain, ...
                testCase.Options);

            testCase.verifyEqual(active.top_speed, [true; false]);
        end

        function testMotorSpeedToleranceInsideAndOutside(testCase)
            ggv = activeConstraintsTest.makeGgv();
            powertrain = activeConstraintsTest.makePowertrainResult(2);
            powertrain.motor_speed_rpm = [998.995; 998.985];

            [active, ~] = classify_active_constraints([10; 10], ...
                [0; 0], [0; 0], [20; 20], ...
                ["top_speed"; "top_speed"], ggv, powertrain, ...
                testCase.Options);

            testCase.verifyEqual(active.motor_speed, [true; false]);
        end
    end

    methods (Static, Access=private)
        function ggv = makeGgv()
            ggv.v_mps = [0; 20];
            ggv.ay_g = [-1, 0, 1];
            ggv.ax_max_g = ones(2, 3);
            ggv.ax_min_g = -ones(2, 3);
            ggv.feasible = true(2, 3);
            ggv.ay_limit_pos_g = ones(2, 1);
            ggv.ay_limit_neg_g = -ones(2, 1);
            ggv.accel_limiter = repmat("traction", 2, 3);
            ggv.brake_limiter = repmat("brake", 2, 3);
            ggv.gravity_mps2 = 9.80665;
        end

        function result = makePowertrainResult(nPoint)
            names = ["rule_power", "rule_current", "battery_power", ...
                "battery_current", "inverter_power", ...
                "inverter_dc_current", "inverter_phase_current", ...
                "motor_speed", "motor_voltage", "motor_power", ...
                "motor_peak_torque", "motor_phase_current"];
            limiters = ["rule_power", "rule_current", "battery_power", ...
                "battery_current", "inverter_power", ...
                "inverter_current", "inverter_current", "motor_speed", ...
                "motor_voltage", "motor_power", "motor_torque", ...
                "motor_torque"];
            result.candidate_names = names;
            result.candidate_limiters = limiters;
            result.candidate_torque_available_Nm = 1000 * ones(nPoint, 12);
            result.motor_speed_rpm = zeros(nPoint, 1);
            result.motor_stop_speed_rpm = 1000 * ones(nPoint, 1);
            result.motor_torque_used_Nm = zeros(nPoint, 1);
            result.motor_torque_available_Nm = 1000 * ones(nPoint, 1);
            result.wheel_force_used_N = zeros(nPoint, 1);
            result.wheel_force_available_N = 1000 * ones(nPoint, 1);
            result.traction_force_available_N = 1000 * ones(nPoint, 1);
            result.powertrain_force_available_N = 1000 * ones(nPoint, 1);
            result.tsac_power_cap_W = 1e5 * ones(nPoint, 1);
            result.tsac_power_used_W = zeros(nPoint, 1);
            result.tsac_dc_current_used_A = zeros(nPoint, 1);
            result.battery_dc_current_used_A = zeros(nPoint, 1);
            result.inverter_dc_current_used_A = zeros(nPoint, 1);
            result.motor_phase_current_used_Arms = zeros(nPoint, 1);
            result.inverter_phase_current_used_Arms = zeros(nPoint, 1);
        end
    end
end
