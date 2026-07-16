classdef runQssPowertrainEnergyTest < matlab.unittest.TestCase
    properties
        Track
        Vehicle
        Models
        Options
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));

            testCase.Track.s_m = [0; 10; 20];
            testCase.Track.ds_m = 10 * ones(3, 1);
            testCase.Track.kappa_1pm = zeros(3, 1);
            testCase.Track.is_closed = true;
            testCase.Vehicle = vehicle_baseline();
            testCase.Models.tire = tire_simple_baseline();
            testCase.Models.aero = struct("enabled", false);
            testCase.Models.powertrain = powertrain_emrax228_hvcc_demo();
            testCase.Models.brake = brake_baseline();
            testCase.Models.endurance = struct("num_laps", 26, ...
                "safety_factor", 1.10);
            testCase.Options = default_qss_options();
            testCase.Options.v_max_mps = 15;
            testCase.Options.v_grid_mps = [0; 5; 10; 15];
            testCase.Options.ay_grid_g = [-2, -1, 0, 1, 2];
            testCase.Models.ggv = generate_model_ggv( ...
                testCase.Vehicle, testCase.Models.tire, ...
                testCase.Models.aero, testCase.Models.powertrain, ...
                testCase.Models.brake, testCase.Options);
        end
    end

    methods (Test)
        function testEnabledCompositeReturnsFrozenContracts(testCase)
            result = run_qss_lap(testCase.Track, testCase.Vehicle, ...
                testCase.Models, testCase.Options);

            testCase.verifyTrue(isfield(result, "powertrain"));
            testCase.verifyTrue(isfield(result, "energy"));
            testCase.verifyTrue(isfield(result, "active_constraints"));
            testCase.verifySize(result.limiter, [3 1]);
            pointFields = ["motor_speed_rpm", ...
                "motor_torque_available_Nm", "motor_torque_used_Nm", ...
                "wheel_force_available_N", "wheel_force_used_N", ...
                "tsac_power_cap_W", "tsac_power_used_W", ...
                "tsac_dc_current_used_A", ...
                "battery_dc_current_used_A", ...
                "inverter_dc_current_used_A", ...
                "motor_phase_current_used_Arms", ...
                "inverter_phase_current_used_Arms", ...
                "V_bus_V", ...
                "effective_voltage_V", ...
                "traction_force_available_N", ...
                "powertrain_force_available_N"];
            pointValues = arrayfun(@(name) result.powertrain.(name), ...
                pointFields, UniformOutput=false);
            testCase.verifyTrue(all(cellfun( ...
                @(value) isequal(size(value), [3 1]), pointValues)));
            testCase.verifyFalse(result.powertrain.thermal_feasibility_evaluated);
            testCase.verifyFalse(result.powertrain.regen_enabled);
            testCase.verifyEqual( ...
                result.powertrain.endurance_energy_feasible, ...
                result.energy.can_finish_endurance_estimated);
            testCase.verifyTrue(isscalar(result.powertrain.voltage_scenario));
            runQssPowertrainEnergyTest.verifyEnergyContract(testCase, ...
                result.energy, 3);
            runQssPowertrainEnergyTest.verifyActiveContract(testCase, ...
                result.active_constraints, 3);
        end

        function testDisabledCompositeDoesNotAddContracts(testCase)
            models = rmfield(testCase.Models, "ggv");
            models.powertrain.enabled = false;

            result = run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyFalse(any(isfield(result, ...
                ["powertrain", "energy", "active_constraints"])));
        end

        function testPrebuiltGgvWithoutProvenanceErrors(testCase)
            models = testCase.Models;
            models.ggv = rmfield(models.ggv, "provenance");

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testPrebuiltGgvPowertrainMismatchErrors(testCase)
            models = testCase.Models;
            models.ggv.provenance.powertrain.gear_ratio = ...
                1.01 * models.powertrain.gear_ratio;

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testPrebuiltGgvTireMismatchErrors(testCase)
            models = testCase.Models;
            models.ggv.provenance.tire.mu_x = 1.01 * models.tire.mu_x;

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testPrebuiltGgvVehicleMismatchErrors(testCase)
            vehicle = testCase.Vehicle;
            vehicle.mass.total_kg = 1.01 * vehicle.mass.total_kg;

            action = @() run_qss_lap(testCase.Track, vehicle, ...
                testCase.Models, testCase.Options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testPrebuiltGgvBrakeMismatchErrors(testCase)
            models = testCase.Models;
            models.brake.front_bias = 0.95 * models.brake.front_bias;

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testDisabledCompositeRejectsStaleEnabledGgv(testCase)
            models = testCase.Models;
            models.powertrain.enabled = false;

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testPrebuiltGgvGenerationOptionMismatchErrors(testCase)
            options = testCase.Options;
            options.ggv_tolerance_mps2 = ...
                10 * options.ggv_tolerance_mps2;

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                testCase.Models, options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testPrebuiltAeroEnabledGgvErrors(testCase)
            models = testCase.Models;
            models.aero = aero_baseline();
            models.ggv.aero_enabled = true;

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, ...
                "QSSLTS:PowertrainGGVProvenance");
        end

        function testEnabledCompositeRequiresTireWithPrebuiltGgv(testCase)
            models = rmfield(testCase.Models, "tire");

            action = @() run_qss_lap(testCase.Track, testCase.Vehicle, ...
                models, testCase.Options);

            testCase.verifyError(action, "QSSLTS:MissingModel");
        end
    end

    methods (Static, Access=private)
        function verifyEnergyContract(testCase, energy, nSegment)
            required = ["segment_ax_mps2", "segment_speed_mean_mps", ...
                "segment_drive_force_N", "segment_wheel_power_W", ...
                "segment_tsac_power_W", ...
                "segment_energy_ts_kWh", "segment_energy_stored_kWh", ...
                "cumulative_energy_ts_kWh", ...
                "cumulative_energy_stored_kWh", "E_lap_ts_kWh", ...
                "E_lap_stored_kWh", "E_endurance_stored_kWh", ...
                "E_nominal_required_kWh", "SOC_end_estimated", ...
                "can_finish_endurance_estimated"].';
            actual = string(fieldnames(energy));
            testCase.verifyEqual(sort(actual), sort(required));
            testCase.verifySize(energy.segment_energy_ts_kWh, ...
                [nSegment 1]);
            testCase.verifySize(energy.cumulative_energy_ts_kWh, ...
                [nSegment + 1 1]);
            testCase.verifyEqual(energy.cumulative_energy_ts_kWh(1), ...
                0, AbsTol=0);
        end

        function verifyActiveContract(testCase, active, nPoint)
            expected = sort(["lateral", "brake", "traction", ...
                "motor_torque", "motor_power", "motor_speed", ...
                "motor_voltage", "rule_power", "rule_current", ...
                "battery_power", "battery_current", "inverter_power", ...
                "inverter_current", "top_speed"].');
            testCase.verifyEqual(sort(string(fieldnames(active))), expected);
            testCase.verifyTrue(all(structfun( ...
                @(value) islogical(value) ...
                && isequal(size(value), [nPoint 1]), active)));
        end
    end
end
