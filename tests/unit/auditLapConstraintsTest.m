classdef auditLapConstraintsTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        ManualResult
        ManualModels
        Vehicle
        Options
        PowertrainResult
        PowertrainModels
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.Vehicle = vehicle_baseline();
            testCase.Options = default_qss_options();
            testCase.Options.v_grid_mps = [0; 5; 10; 15];
            testCase.Options.ay_grid_g = [-2, -1, 0, 1, 2];

            ggv = auditLapConstraintsTest.makeExternalGgv( ...
                testCase.Options.gravity_mps2);
            manual.track.s_m = [0; 10];
            manual.track.ds_m = 10;
            manual.track.kappa_1pm = [0; 0];
            manual.track.is_closed = false;
            manual.v_mps = [10; 10];
            manual.ax_mps2 = [0; 0];
            manual.ay_mps2 = [0; 0];
            manual.ggv_used = ggv;
            testCase.ManualResult = manual;
            testCase.ManualModels.ggv = ggv;

            track.s_m = [0; 10; 20];
            track.ds_m = 10 * ones(3, 1);
            track.kappa_1pm = zeros(3, 1);
            track.is_closed = true;
            models.tire = tire_simple_baseline();
            models.aero = struct("enabled", false);
            models.powertrain = powertrain_emrax228_hvcc_demo();
            models.brake = brake_baseline();
            models.endurance = struct("num_laps", 26, ...
                "safety_factor", 1.10);
            models.ggv = generate_model_ggv(testCase.Vehicle, ...
                models.tire, models.aero, models.powertrain, ...
                models.brake, testCase.Options);
            testCase.PowertrainModels = models;
            testCase.PowertrainResult = run_qss_lap( ...
                track, testCase.Vehicle, models, testCase.Options);
        end
    end

    methods (Test)
        function testExternalGgvPassesAndComponentsAreNotApplicable( ...
                testCase)
            audit = audit_lap_constraints(testCase.ManualResult, ...
                testCase.Vehicle, testCase.ManualModels, testCase.Options);

            testCase.verifyTrue(audit.valid);
            testCase.verifyEqual(audit.status, "pass");
            testCase.verifyEqual(audit.checks.ggv_ax_max.status, "pass");
            testCase.verifyEqual(audit.checks.motor_torque.status, ...
                "not_applicable");
            testCase.verifyEqual(audit.checks.brake.status, ...
                "not_applicable");
        end

        function testGgvLongitudinalViolationIsRejected(testCase)
            violating = testCase.ManualResult;
            violating.ax_mps2(1) = 2 * testCase.Options.gravity_mps2;

            audit = audit_lap_constraints(violating, ...
                testCase.Vehicle, testCase.ManualModels, testCase.Options);

            testCase.verifyFalse(audit.valid);
            testCase.verifyEqual(audit.checks.ggv_ax_max.status, "fail");
            testCase.verifyTrue(ismember( ...
                "ggv_ax_max_violation", audit.reason_codes));
            testCase.verifyEqual( ...
                audit.checks.ggv_ax_max.violating_indices, 1);
        end

        function testRunQssIntegratesPassingPowertrainAudit(testCase)
            result = testCase.PowertrainResult;
            requiredChecks = ["wheel_force", "combined_slip", "brake", ...
                "motor_torque", "motor_speed", "motor_voltage", ...
                "tsac_voltage", "tsac_power", "tsac_current", ...
                "battery_power", "battery_current", ...
                "inverter_power", "inverter_current", "phase_current"];
            statuses = arrayfun(@(name) ...
                result.constraint_audit.checks.(name).status, ...
                requiredChecks);

            testCase.verifyTrue(result.constraint_audit.valid);
            testCase.verifyEqual(result.constraint_audit.status, "pass");
            testCase.verifyTrue(result.valid);
            testCase.verifyEqual(statuses, repmat("pass", ...
                size(requiredChecks)));
        end

        function testTsacPowerOveruseHasStableReason(testCase)
            violating = testCase.PowertrainResult;
            violating.powertrain.tsac_power_used_W(1) = ...
                violating.powertrain.tsac_power_cap_W(1) + 1;

            audit = audit_lap_constraints(violating, ...
                testCase.Vehicle, testCase.PowertrainModels, ...
                testCase.Options);

            testCase.verifyFalse(audit.valid);
            testCase.verifyEqual(audit.checks.tsac_power.status, "fail");
            testCase.verifyTrue(ismember( ...
                "tsac_power_violation", audit.reason_codes));
        end

        function testTsacRulePowerLimitIsAuditedDirectly(testCase)
            violating = testCase.PowertrainResult;
            ruleLimit_W = ...
                testCase.PowertrainModels.powertrain.rules.max_ts_power_W;
            violating.powertrain.tsac_power_cap_W(1) = ruleLimit_W + 1000;
            violating.powertrain.tsac_power_used_W(1) = ruleLimit_W + 1000;

            audit = audit_lap_constraints(violating, ...
                testCase.Vehicle, testCase.PowertrainModels, ...
                testCase.Options);

            testCase.verifyFalse(audit.valid);
            testCase.verifyEqual(audit.checks.tsac_power.status, "fail");
            testCase.verifyEqual( ...
                audit.checks.tsac_power.violating_indices, 1);
            testCase.verifyTrue(ismember( ...
                "tsac_power_violation", audit.reason_codes));
        end

        function testBatteryMinimumVoltageIsAudited(testCase)
            violating = testCase.PowertrainResult;
            minimum_V = ...
                testCase.PowertrainModels.powertrain.battery.V_min_V;
            violating.powertrain.V_bus_V(1) = minimum_V - 1;

            audit = audit_lap_constraints(violating, ...
                testCase.Vehicle, testCase.PowertrainModels, ...
                testCase.Options);

            testCase.verifyFalse(audit.valid);
            testCase.verifyEqual(audit.checks.tsac_voltage.status, "fail");
            testCase.verifyEqual( ...
                audit.checks.tsac_voltage.violating_indices, 1);
            testCase.verifyTrue(ismember( ...
                "tsac_voltage_violation", audit.reason_codes));
        end

        function testLargeWheelForceViolationExceedsTolerance(testCase)
            violating = testCase.PowertrainResult;
            violating.ax_mps2(1) = 2 * testCase.Options.gravity_mps2;

            audit = audit_lap_constraints(violating, ...
                testCase.Vehicle, testCase.PowertrainModels, ...
                testCase.Options);

            testCase.verifyEqual(audit.checks.combined_slip.status, "fail");
            testCase.verifyEqual(audit.checks.wheel_force.status, "fail");
            testCase.verifyTrue(ismember(1, ...
                audit.checks.combined_slip.violating_indices));
            testCase.verifyTrue(ismember(1, ...
                audit.checks.wheel_force.violating_indices));
        end

        function testEnergyFeasibilityDoesNotChangeConstraintAudit( ...
                testCase)
            result = testCase.PowertrainResult;
            result.energy.can_finish_endurance_estimated = false;

            audit = audit_lap_constraints(result, testCase.Vehicle, ...
                testCase.PowertrainModels, testCase.Options);

            testCase.verifyTrue(audit.valid);
            testCase.verifyEqual(audit.status, "pass");
        end
    end

    methods (Static, Access=private)
        function ggv = makeExternalGgv(gravity_mps2)
            ggv.v_mps = [0; 20];
            ggv.ay_g = [-1, 0, 1];
            ggv.ax_max_g = ones(2, 3);
            ggv.ax_min_g = -ones(2, 3);
            ggv.feasible = true(2, 3);
            ggv.ay_limit_pos_g = ones(2, 1);
            ggv.ay_limit_neg_g = -ones(2, 1);
            ggv.accel_limiter = repmat("measured", 2, 3);
            ggv.brake_limiter = repmat("measured", 2, 3);
            ggv.gravity_mps2 = gravity_mps2;
            ggv.source = "real_csv";
        end
    end
end
