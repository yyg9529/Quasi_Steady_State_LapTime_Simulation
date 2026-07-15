classdef powertrainConstraintKernelTest < matlab.unittest.TestCase
    properties (Constant)
        RollingRadius_m = 0.2286
        GearRatio = 4.369334602435052
    end

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
        function testDemoUsesFrozenCompositeInterface(testCase)
            powertrain = powertrain_emrax228_hvcc_demo();
            expectedFields = sort(["enabled"; "motor_count"; "layout"; ...
                "gear_ratio"; "drivetrain_efficiency"; "motor"; ...
                "battery"; "inverter"; "rules"]);

            testCase.verifyEqual(sort(string(fieldnames(powertrain))), ...
                expectedFields);
            testCase.verifyEqual(powertrain.motor_count, 1, AbsTol=0);
            testCase.verifyEqual(powertrain.layout, "RWD");
            testCase.verifyEqual(powertrain.gear_ratio, testCase.GearRatio, ...
                AbsTol=1e-15);
            testCase.verifyEqual(powertrain.drivetrain_efficiency, 0.90, ...
                AbsTol=1e-15);
            testCase.verifyFalse(isfield(powertrain, "rolling_radius_m"));
            testCase.verifyFalse(isfield(powertrain, "overall_gear_ratio"));
            testCase.verifyFalse(isfield(powertrain, "drive_efficiency"));
            expectedBatteryFields = sort(["V_max_V"; "V_nominal_V"; ...
                "V_min_V"; "V_bus_assumed_V"; "E_nominal_kWh"; ...
                "SOC_init"; "SOC_min"; "P_discharge_peak_W"; ...
                "I_discharge_peak_A"; "eta_discharge"; "P_ts_aux_W"]);
            expectedInverterFields = sort(["V_dc_max_V"; "P_dc_peak_W"; ...
                "I_dc_peak_A"; "I_phase_peak_Arms"; "eta_const"]);
            testCase.verifyEqual( ...
                sort(string(fieldnames(powertrain.battery))), ...
                expectedBatteryFields);
            testCase.verifyEqual( ...
                sort(string(fieldnames(powertrain.inverter))), ...
                expectedInverterFields);
            testCase.verifyEqual(powertrain.battery.E_nominal_kWh, 8, ...
                AbsTol=0);
            testCase.verifyFalse(isfield(powertrain.battery, "max_voltage_V"));
            testCase.verifyFalse(isfield(powertrain.inverter, ...
                "max_dc_voltage_V"));
        end

        function test600VEnvelopeAnchors(testCase)
            powertrain = testCase.makeConfig();

            envelope = calc_motor_voltage_envelope(0, 600, powertrain);

            testCase.verifyEqual(envelope.effective_voltage_V, 600, AbsTol=0);
            testCase.verifyEqual(envelope.peak_power_available_W, ...
                104e3 * 600 / 830, AbsTol=1e-9);
            testCase.verifyEqual(envelope.peak_load_speed_rpm, 3390, ...
                AbsTol=1e-12);
            testCase.verifyEqual(envelope.stop_speed_rpm, 6084, ...
                AbsTol=1e-12);
        end

        function test600VEnvelopeIsBoundedAndNonincreasing(testCase)
            powertrain = testCase.makeConfig();
            rpm = [0; 3390; 4500; 6000; 6084];

            envelope = calc_motor_voltage_envelope(rpm, 600, powertrain);

            testCase.verifyLessThanOrEqual(envelope.available_torque_Nm, ...
                220 * ones(size(rpm)));
            testCase.verifyLessThanOrEqual(envelope.available_power_W, ...
                (104e3 * 600 / 830) * ones(size(rpm)));
            testCase.verifyLessThanOrEqual(diff(envelope.available_torque_Nm), ...
                zeros(numel(rpm) - 1, 1));
            testCase.verifyEqual(envelope.available_torque_Nm(end), 0, ...
                AbsTol=1e-12);
        end

        function test600VCapabilityDoesNotExceed830V(testCase)
            powertrain600 = testCase.makeConfig();
            powertrain830 = testCase.make830VConfig();
            rpm = [0; 3390; 4500; 6000];

            envelope600 = calc_motor_voltage_envelope( ...
                rpm, 600, powertrain600);
            envelope830 = calc_motor_voltage_envelope( ...
                rpm, 830, powertrain830);

            testCase.verifyLessThanOrEqual(envelope600.available_torque_Nm, ...
                envelope830.available_torque_Nm);
            testCase.verifyLessThanOrEqual(envelope600.available_power_W, ...
                envelope830.available_power_W);
        end

        function testRulePowerAndAuxiliaryChain(testCase)
            powertrain = testCase.makeConfig();

            cap = calc_ts_power_cap(600, powertrain);

            expectedMechanical_W = (80e3 - 500) * 0.97 * 0.94;
            testCase.verifyEqual(cap.power_candidates_W.rule_power, 80e3, ...
                AbsTol=0);
            testCase.verifyEqual(cap.tsac_power_cap_W, 80e3, AbsTol=0);
            testCase.verifyEqual(cap.limiter, "rule_power");
            testCase.verifyEqual(cap.aux_power_W, 500, AbsTol=0);
            testCase.verifyEqual(cap.mechanical_power_cap_W, ...
                expectedMechanical_W, AbsTol=1e-9);
        end

        function testRuleCurrentLimit(testCase)
            powertrain = testCase.makeUnconstrainedConfig();
            powertrain.rules.max_ts_current_A = 100;

            cap = calc_ts_power_cap(600, powertrain);

            testCase.verifyEqual(cap.tsac_power_cap_W, 60e3, AbsTol=0);
            testCase.verifyEqual(cap.limiter, "rule_current");
        end

        function testBatteryPowerLimit(testCase)
            powertrain = testCase.makeUnconstrainedConfig();
            powertrain.battery.P_discharge_peak_W = 40e3;

            cap = calc_ts_power_cap(600, powertrain);

            testCase.verifyEqual(cap.tsac_power_cap_W, 40e3, AbsTol=0);
            testCase.verifyEqual(cap.limiter, "battery_power");
        end

        function testBatteryCurrentLimit(testCase)
            powertrain = testCase.makeUnconstrainedConfig();
            powertrain.battery.I_discharge_peak_A = 50;

            cap = calc_ts_power_cap(600, powertrain);

            testCase.verifyEqual(cap.tsac_power_cap_W, 30e3, AbsTol=0);
            testCase.verifyEqual(cap.limiter, "battery_current");
        end

        function testInverterPowerLimit(testCase)
            powertrain = testCase.makeUnconstrainedConfig();
            powertrain.inverter.P_dc_peak_W = 35e3;

            cap = calc_ts_power_cap(600, powertrain);

            testCase.verifyEqual(cap.tsac_power_cap_W, 35e3, AbsTol=0);
            testCase.verifyEqual(cap.limiter, "inverter_power");
        end

        function testInverterDcCurrentLimit(testCase)
            powertrain = testCase.makeUnconstrainedConfig();
            powertrain.inverter.I_dc_peak_A = 40;

            cap = calc_ts_power_cap(600, powertrain);

            testCase.verifyEqual(cap.tsac_power_cap_W, 24e3, AbsTol=0);
            testCase.verifyEqual(cap.limiter, "inverter_current");
            testCase.verifyEqual(cap.dc_current_limits_A.inverter, 40, ...
                AbsTol=0);
        end

        function testDcAndPhaseCurrentRemainSeparate(testCase)
            powertrain = testCase.makeConfig();
            tire.rolling_radius_m = testCase.RollingRadius_m;

            capability = evaluate_powertrain_constraints(1, tire, ...
                powertrain, 600);

            testCase.verifyEqual(capability.dc_current_limits_A.rule, 500, ...
                AbsTol=0);
            testCase.verifyEqual(capability.dc_current_limits_A.inverter, ...
                300, AbsTol=0);
            testCase.verifyEqual( ...
                capability.phase_current_limits_Arms.motor, 235, AbsTol=0);
            testCase.verifyEqual( ...
                capability.phase_current_limits_Arms.inverter, 250, AbsTol=0);
            testCase.verifyEqual( ...
                capability.candidates.inverter_phase_current_torque_Nm, ...
                250 * 0.94, AbsTol=1e-12);
        end

        function testInverterPhaseCurrentLimitsTorque(testCase)
            powertrain = testCase.makeUnconstrainedConfig();
            powertrain.inverter.I_phase_peak_Arms = 100;
            tire.rolling_radius_m = testCase.RollingRadius_m;

            capability = evaluate_powertrain_constraints(1, tire, ...
                powertrain, 600);

            expectedTorque_Nm = 100 * 0.94;
            expectedForce_N = expectedTorque_Nm * testCase.GearRatio ...
                * 0.90 / testCase.RollingRadius_m;
            testCase.verifyEqual(capability.available_motor_torque_Nm, ...
                expectedTorque_Nm, AbsTol=1e-10);
            testCase.verifyEqual(capability.available_wheel_force_N, ...
                expectedForce_N, AbsTol=1e-9);
            testCase.verifyEqual(capability.limiter, "inverter_current");
        end

        function testMotorPeakAndPhaseCurrentCandidates(testCase)
            powertrain = testCase.makeConfig();
            tire.rolling_radius_m = testCase.RollingRadius_m;

            capability = evaluate_powertrain_constraints(1, tire, ...
                powertrain, 600);

            testCase.verifyEqual( ...
                capability.candidates.motor_peak_torque_Nm, 220, AbsTol=0);
            testCase.verifyEqual( ...
                capability.candidates.motor_phase_current_torque_Nm, ...
                235 * 0.94, AbsTol=1e-12);
            testCase.verifyEqual(capability.available_motor_torque_Nm, ...
                220, AbsTol=1e-12);
            testCase.verifyEqual(capability.limiter, "motor_torque");
        end

        function testGearMappingAnd600VSpeedCutoff(testCase)
            powertrain = testCase.makeConfig();
            tire.rolling_radius_m = testCase.RollingRadius_m;

            capability = evaluate_powertrain_constraints(120 / 3.6, tire, ...
                powertrain, 600);

            testCase.verifyEqual(capability.motor_speed_rpm, 6084, ...
                AbsTol=1e-10);
            testCase.verifyEqual(capability.available_motor_torque_Nm, 0, ...
                AbsTol=1e-10);
            testCase.verifyEqual(capability.available_wheel_force_N, 0, ...
                AbsTol=1e-9);
            testCase.verifyEqual(capability.limiter, "motor_speed");
        end

        function testPhysicalMotorSpeedCutoffAt830V(testCase)
            powertrain = testCase.make830VConfig();
            tire.rolling_radius_m = testCase.RollingRadius_m;
            speed_mps = 6500 * 2 * pi / 60 * testCase.RollingRadius_m ...
                / testCase.GearRatio;

            capability = evaluate_powertrain_constraints( ...
                speed_mps, tire, powertrain, 830);

            testCase.verifyEqual(capability.available_motor_torque_Nm, 0, ...
                AbsTol=1e-10);
            testCase.verifyEqual(capability.limiter, "motor_speed");
        end

        function test600VVoltageDeratingAndVectorLimiterShape(testCase)
            powertrain = testCase.makeConfig();
            tire.rolling_radius_m = testCase.RollingRadius_m;
            motorSpeed_rpm = [4500; 5000];
            speed_mps = motorSpeed_rpm * 2 * pi / 60 ...
                * testCase.RollingRadius_m / testCase.GearRatio;

            envelope = calc_motor_voltage_envelope( ...
                motorSpeed_rpm, 600, powertrain);
            capability = evaluate_powertrain_constraints( ...
                speed_mps, tire, powertrain, 600);

            testCase.verifyEqual(capability.available_motor_torque_Nm(1), ...
                envelope.linear_torque_limit_Nm(1), AbsTol=1e-10);
            testCase.verifyEqual(capability.limiter(1), "motor_voltage");
            testCase.verifySize(capability.limiter, [2, 1]);
        end

        function testCandidateNamesAndLimitersAreReturned(testCase)
            powertrain = testCase.makeConfig();
            tire.rolling_radius_m = testCase.RollingRadius_m;

            capability = evaluate_powertrain_constraints(10, tire, ...
                powertrain, 600);

            testCase.verifySize(capability.candidate_torque_Nm, [1, 12]);
            testCase.verifySize(capability.candidate_names, [1, 12]);
            testCase.verifySize(capability.candidate_limiters, [1, 12]);
            testCase.verifyTrue(any(capability.candidate_names ...
                == "inverter_dc_current"));
            testCase.verifyTrue(any(capability.candidate_names ...
                == "inverter_phase_current"));
        end

        function testDiagnosticFlagsUseFrozenNames(testCase)
            powertrain = testCase.makeConfig();
            tire.rolling_radius_m = testCase.RollingRadius_m;

            capability = evaluate_powertrain_constraints(10, tire, ...
                powertrain, 600);

            testCase.verifyFalse( ...
                capability.thermal_feasibility_evaluated);
            testCase.verifyFalse(capability.regen_enabled);
            testCase.verifyFalse(isfield(capability, ...
                "thermal_model_enabled"));
            testCase.verifyFalse(isfield(capability, ...
                "regen_enabled_in_model"));
        end

        function testMotorCountOtherThanOneErrors(testCase)
            powertrain = testCase.makeConfig();
            powertrain.motor_count = 2;

            action = @() validate_powertrain_config(powertrain);

            testCase.verifyError(action, "QSSLTS:PowertrainMotorCount");
        end

        function testSocValuesMustBeRealFiniteScalars(testCase)
            nanInitial = testCase.makeConfig();
            nanInitial.battery.SOC_init = NaN;
            nanMinimum = testCase.makeConfig();
            nanMinimum.battery.SOC_min = NaN;

            initialAction = @() validate_powertrain_config(nanInitial);
            minimumAction = @() validate_powertrain_config(nanMinimum);

            testCase.verifyError(initialAction, ...
                "QSSLTS:PowertrainConfig");
            testCase.verifyError(minimumAction, ...
                "QSSLTS:PowertrainConfig");
        end

        function testRollingRadiusComesOnlyFromTire(testCase)
            powertrain = testCase.makeConfig();
            tire = struct();

            action = @() evaluate_powertrain_constraints( ...
                10, tire, powertrain, 600);

            testCase.verifyError(action, "QSSLTS:PowertrainTire");
        end
    end

    methods (Static, Access=private)
        function powertrain = makeConfig()
            powertrain.enabled = true;
            powertrain.motor_count = 1;
            powertrain.layout = "RWD";
            powertrain.gear_ratio = 4.369334602435052;
            powertrain.drivetrain_efficiency = 0.90;
            powertrain.motor.name = "EMRAX_228_HV_CC";
            powertrain.motor.mass_kg = 13.2;
            powertrain.motor.max_mechanical_speed_rpm = 6500;
            powertrain.motor.physical_peak_power_W = 104e3;
            powertrain.motor.physical_peak_power_rpm = 4500;
            powertrain.motor.physical_cont_power_W = 75e3;
            powertrain.motor.peak_torque_Nm = 220;
            powertrain.motor.cont_torque_Nm = 130;
            powertrain.motor.required_voltage_peak_power_V = 830;
            powertrain.motor.peak_phase_current_Arms = 235;
            powertrain.motor.cont_phase_current_Arms = 120;
            powertrain.motor.Kv_no_load_rpm_per_V = 10.14;
            powertrain.motor.Kv_nominal_load_rpm_per_V = 7.85;
            powertrain.motor.Kv_peak_load_rpm_per_V = 5.65;
            powertrain.motor.Kt_Nm_per_Arms = 0.94;
            powertrain.motor.eta_const = 0.94;
            powertrain.motor.thermal_model_enabled = false;
            powertrain.battery.V_max_V = 600;
            powertrain.battery.V_nominal_V = 540;
            powertrain.battery.V_min_V = 450;
            powertrain.battery.V_bus_assumed_V = 600;
            powertrain.battery.E_nominal_kWh = 8;
            powertrain.battery.SOC_init = 0.95;
            powertrain.battery.SOC_min = 0.10;
            powertrain.battery.P_discharge_peak_W = 100e3;
            powertrain.battery.I_discharge_peak_A = 300;
            powertrain.battery.eta_discharge = 0.98;
            powertrain.battery.P_ts_aux_W = 500;
            powertrain.inverter.V_dc_max_V = 600;
            powertrain.inverter.P_dc_peak_W = 100e3;
            powertrain.inverter.I_dc_peak_A = 300;
            powertrain.inverter.I_phase_peak_Arms = 250;
            powertrain.inverter.eta_const = 0.97;
            powertrain.rules.name = "Formula Student Rules";
            powertrain.rules.season = 2026;
            powertrain.rules.version = "1.1";
            powertrain.rules.max_ts_voltage_V = 600;
            powertrain.rules.max_ts_power_W = 80e3;
            powertrain.rules.max_ts_current_A = 500;
            powertrain.rules.regen_enabled_in_model = false;
        end

        function powertrain = makeUnconstrainedConfig()
            powertrain = powertrainConstraintKernelTest.makeConfig();
            powertrain.rules.max_ts_power_W = 1e9;
            powertrain.rules.max_ts_current_A = 1e9;
            powertrain.battery.P_discharge_peak_W = 1e9;
            powertrain.battery.I_discharge_peak_A = 1e9;
            powertrain.inverter.P_dc_peak_W = 1e9;
            powertrain.inverter.I_dc_peak_A = 1e9;
            powertrain.inverter.I_phase_peak_Arms = 1e9;
            powertrain.battery.P_ts_aux_W = 0;
        end

        function powertrain = make830VConfig()
            powertrain = powertrainConstraintKernelTest.makeConfig();
            powertrain.rules.max_ts_voltage_V = 830;
            powertrain.battery.V_max_V = 830;
            powertrain.battery.V_bus_assumed_V = 830;
            powertrain.inverter.V_dc_max_V = 830;
        end
    end
end
