classdef analysisPowertrainMigrationTest < matlab.unittest.TestCase
    properties
        Config
        Tire
        Options
    end

    properties (TestParameter)
        legacyDottedPath = struct( ...
            "maxPower", "models.powertrain.max_power_W", ...
            "wheelTorque", "models.powertrain.max_wheel_torque_Nm", ...
            "totalWheelTorque", ...
                "models.powertrain.max_total_wheel_torque_Nm", ...
            "maxSpeed", "models.powertrain.max_speed_mps", ...
            "overallRatio", "models.powertrain.overall_gear_ratio", ...
            "driveEfficiency", ...
                "models.powertrain.drive_efficiency")
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));

            testCase.Tire = tire_simple_baseline();
            testCase.Options = default_qss_options();
            testCase.Options.v_max_mps = 20;
            testCase.Options.v_grid_mps = [0; 10; 20];
            testCase.Options.ay_grid_g = [-2, -1, 0, 1, 2];

            track.s_m = [0; 20; 40; 60];
            track.ds_m = 20 * ones(4, 1);
            track.kappa_1pm = [0; 0.03; 0; -0.03];
            track.is_closed = true;
            testCase.Config.track = track;
            testCase.Config.vehicle = vehicle_baseline();
            testCase.Config.models.tire = testCase.Tire;
            testCase.Config.models.aero = aero_baseline();
            testCase.Config.models.powertrain = ...
                powertrain_emrax228_hvcc_demo();
            testCase.Config.models.brake = brake_baseline();
            testCase.Config.models.endurance = struct( ...
                "num_laps", 1, "safety_factor", 1.0);
            testCase.Config.options = testCase.Options;
        end
    end

    methods (Test)
        function testOptionalHandlingFailurePreservesLapResult(testCase)
            handlingConfig = struct( ...
                "enabled", true, ...
                "tir_file", string(tempname) + ".tir", ...
                "failure_policy", "report_unavailable");

            [result, handling] = run_analysis_case( ...
                testCase.Config, table(), handlingConfig);

            testCase.verifyTrue(result.solver.converged);
            testCase.verifyFalse(handling.available);
            testCase.verifyEqual(handling.error_id, ...
                "QSSLTS:HandlingTirMissing");
        end

        function testDottedRuleFieldIsImmutable(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "models.powertrain.rules.max_ts_power_W", 70000);

            testCase.verifyError(action, ...
                "QSSLTS:AnalysisRulesImmutable");
        end

        function testCaseInsensitiveRulePathIsImmutable(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "MODELS.POWERTRAIN.RULES.max_ts_current_A", 450);

            testCase.verifyError(action, ...
                "QSSLTS:AnalysisRulesImmutable");
        end

        function testWholeRuleStructIsImmutable(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "models.powertrain.rules", struct());

            testCase.verifyError(action, ...
                "QSSLTS:AnalysisRulesImmutable");
        end

        function testCompactRuleStructIsImmutable(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "rules", struct());

            testCase.verifyError(action, ...
                "QSSLTS:AnalysisRulesImmutable");
        end

        function testWholePowertrainCompositeReplacementIsImmutable(testCase)
            original = testCase.Config;
            replacement = original.models.powertrain;
            replacement.rules.max_ts_power_W = 70000;
            action = @() apply_analysis_parameter(original, ...
                "models.powertrain", replacement);

            testCase.verifyError(action, ...
                "QSSLTS:AnalysisRulesImmutable");
            testCase.verifyEqual(testCase.Config, original);
        end

        function testWholePowertrainFlatReplacementIsImmutable(testCase)
            original = testCase.Config;
            replacement = ...
                analysisPowertrainMigrationTest.makeFlatPowertrain();
            action = @() apply_analysis_parameter(original, ...
                "models.powertrain", replacement);

            testCase.verifyError(action, ...
                "QSSLTS:AnalysisRulesImmutable");
            testCase.verifyEqual(testCase.Config, original);
        end

        function testCompactInverterPowerMapsExactly(testCase)
            expected = testCase.Config.models.powertrain;
            expected.inverter.P_dc_peak_W = 70000;

            output = apply_analysis_parameter(testCase.Config, ...
                "inverter_power_W", 70000);

            testCase.verifyEqual(output.models.powertrain, expected);
        end

        function testCompactInverterPowerIsValidated(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "inverter_power_W", -1);

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end

        function testDottedCompositeUpdateIsValidated(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "models.powertrain.inverter.P_dc_peak_W", -1);

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end

        function testCompositeWithLegacyTopFieldIsRejected(testCase)
            powertrain = testCase.Config.models.powertrain;
            powertrain.max_power_W = 80000;
            action = @() validate_powertrain_config(powertrain);

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end

        function testGearRatioOnlyChangesCompositeRatio(testCase)
            expected = testCase.Config.models.powertrain;
            expected.gear_ratio = 5;

            output = apply_analysis_parameter(testCase.Config, ...
                "gear_ratio", 5);

            testCase.verifyEqual(output.models.powertrain, expected);
        end

        function testDottedGearRatioRejectsVector(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "models.powertrain.gear_ratio", [4, 5]);

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end

        function testLegacyMaxPowerAliasIsRejected(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "max_power_W", 70000);

            testCase.verifyError(action, "QSSLTS:AnalysisParameter");
        end

        function testLegacyPowerAliasIsRejected(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "power", 70000);

            testCase.verifyError(action, "QSSLTS:AnalysisParameter");
        end

        function testLegacyOverallGearRatioAliasIsRejected(testCase)
            action = @() apply_analysis_parameter(testCase.Config, ...
                "overall_gear_ratio", 5);

            testCase.verifyError(action, "QSSLTS:AnalysisParameter");
        end

        function testDottedLegacyFlatFieldIsRejected( ...
                testCase, legacyDottedPath)
            config = testCase.Config;
            config.models.powertrain = ...
                analysisPowertrainMigrationTest.makeFlatPowertrain();
            action = @() apply_analysis_parameter( ...
                config, legacyDottedPath, 1);

            testCase.verifyError(action, "QSSLTS:AnalysisParameter");
        end

        function testDottedLegacyFieldOnDisabledSentinelIsRejected(testCase)
            config = testCase.Config;
            config.models.powertrain = struct("enabled", false);
            action = @() apply_analysis_parameter(config, ...
                "models.powertrain.max_power_W", 70000);

            testCase.verifyError(action, "QSSLTS:AnalysisParameter");
        end

        function testTheoryOnlyCompositeAeroRunsOnline(testCase)
            result = run_analysis_case(testCase.Config, table());

            testCase.verifyTrue(result.solver.converged);
            testCase.verifyTrue(isfield(result, "powertrain"));
            testCase.verifyTrue(isfield(result, "energy"));
        end

        function testPrepareContextPreservesEndurance(testCase)
            context = prepare_analysis_context(testCase.Config);

            testCase.verifyEqual(context.base_config.models.endurance, ...
                testCase.Config.models.endurance);
        end

        function testCalcDriveRejectsFlatConfig(testCase)
            loads.Fx_available_N = 1000 * ones(4, 1);
            flat = analysisPowertrainMigrationTest.makeFlatPowertrain();
            action = @() calc_drive_limit(10, loads, testCase.Tire, ...
                flat, testCase.Options);

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end

        function testGenerateGgvRejectsFlatConfig(testCase)
            flat = analysisPowertrainMigrationTest.makeFlatPowertrain();
            action = @() generate_model_ggv(vehicle_baseline(), ...
                testCase.Tire, struct("enabled", false), flat, ...
                brake_baseline(), testCase.Options);

            testCase.verifyError(action, "QSSLTS:PowertrainConfig");
        end

        function testCalcDriveAcceptsNoPowertrainSentinels(testCase)
            loads.Fx_available_N = [100; 200; 300; 400];

            emptyDrive = calc_drive_limit(10, loads, testCase.Tire, ...
                struct(), testCase.Options);
            disabledDrive = calc_drive_limit(10, loads, testCase.Tire, ...
                struct("enabled", false), testCase.Options);

            testCase.verifyEqual(emptyDrive.Fx_drive_max_N, 1000, ...
                AbsTol=0);
            testCase.verifyEqual(disabledDrive.Fx_drive_max_N, 1000, ...
                AbsTol=0);
            testCase.verifyEqual(emptyDrive.limiter, "tire");
            testCase.verifyEqual(disabledDrive.limiter, "tire");
            testCase.verifyFalse(isfield(emptyDrive, "torque_limit_N"));
            testCase.verifyFalse(isfield(emptyDrive, "power_limit_N"));
            testCase.verifyFalse(isfield(disabledDrive, "torque_limit_N"));
            testCase.verifyFalse(isfield(disabledDrive, "power_limit_N"));
        end

        function testGenerateGgvCanonicalizesNoPowertrainSentinels(testCase)
            emptyGgv = generate_model_ggv(vehicle_baseline(), ...
                testCase.Tire, struct("enabled", false), struct(), ...
                brake_baseline(), testCase.Options);
            disabledGgv = generate_model_ggv(vehicle_baseline(), ...
                testCase.Tire, struct("enabled", false), ...
                struct("enabled", false), brake_baseline(), ...
                testCase.Options);

            testCase.verifyEqual(emptyGgv.provenance.powertrain, ...
                struct("enabled", false));
            testCase.verifyEqual(disabledGgv.provenance.powertrain, ...
                struct("enabled", false));
        end
    end

    methods (Static, Access=private)
        function powertrain = makeFlatPowertrain()
            powertrain.enabled = true;
            powertrain.layout = "RWD";
            powertrain.max_power_W = 80000;
            powertrain.max_total_wheel_torque_Nm = 1000;
            powertrain.max_speed_mps = 45;
            powertrain.drive_efficiency = 0.9;
        end
    end
end
