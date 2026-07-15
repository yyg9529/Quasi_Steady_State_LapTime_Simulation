classdef emraxPresetTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectDataPath(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testIdentityAndSource(testCase)
            expectedUrl = "https://emrax.com/wp-content/uploads/2025/03/EMRAX_228_datasheet_v1.6.pdf";

            motor = emrax228_hv_cc();

            testCase.verifyEqual(motor.name, "EMRAX_228_HV_CC");
            testCase.verifyEqual(motor.source, "EMRAX 228 datasheet v1.6");
            testCase.verifyEqual(motor.source_url, expectedUrl);
            testCase.verifyEqual(motor.voltage_variant, "HV");
            testCase.verifyEqual(motor.cooling, "CC");
        end

        function testPhysicalRatings(testCase)
            motor = emrax228_hv_cc();

            testCase.verifyEqual(motor.mass_kg, 13.2, AbsTol=1e-12);
            testCase.verifyEqual(motor.max_mechanical_speed_rpm, 6500, AbsTol=0);
            testCase.verifyEqual(motor.physical_peak_power_W, 104e3, AbsTol=0);
            testCase.verifyEqual(motor.physical_peak_power_rpm, 4500, AbsTol=0);
            testCase.verifyEqual(motor.physical_peak_power_duration_s, 120, AbsTol=0);
            testCase.verifyEqual(motor.physical_cont_power_W, 75e3, AbsTol=0);
            testCase.verifyEqual(motor.peak_torque_Nm, 220, AbsTol=0);
            testCase.verifyEqual(motor.cont_torque_Nm, 130, AbsTol=0);
            testCase.verifyEqual(motor.required_voltage_peak_power_V, 830, AbsTol=0);
        end

        function testCurrentAndMachineConstants(testCase)
            motor = emrax228_hv_cc();

            testCase.verifyEqual(motor.peak_phase_current_Arms, 235, AbsTol=0);
            testCase.verifyEqual(motor.cont_phase_current_Arms, 120, AbsTol=0);
            testCase.verifyEqual(motor.Kv_no_load_rpm_per_V, 10.14, AbsTol=1e-12);
            testCase.verifyEqual(motor.Kv_nominal_load_rpm_per_V, 7.85, AbsTol=1e-12);
            testCase.verifyEqual(motor.Kv_peak_load_rpm_per_V, 5.65, AbsTol=1e-12);
            testCase.verifyEqual(motor.Kt_Nm_per_Arms, 0.94, AbsTol=1e-12);
        end

        function testEfficiencyAssumptionIsExplicit(testCase)
            motor = emrax228_hv_cc();

            testCase.verifyEqual(motor.eta_const, 0.94, AbsTol=1e-12);
            testCase.verifyEqual(motor.eta_const_source, ...
                "conservative engineering assumption");
            testCase.verifyEqual(motor.official_peak_efficiency, 0.96, ...
                AbsTol=1e-12);
            testCase.verifyFalse(motor.thermal_model_enabled);
        end

        function testAlternativeInterfaceAliasesAreAbsent(testCase)
            motor = emrax228_hv_cc();

            testCase.verifyFalse(isfield(motor, "weight_kg"));
            testCase.verifyFalse(isfield(motor, "max_speed_rpm"));
            testCase.verifyFalse(isfield(motor, "peak_power_W"));
            testCase.verifyFalse(isfield(motor, "peak_power_speed_rpm"));
            testCase.verifyFalse(isfield(motor, "continuous_power_W"));
            testCase.verifyFalse(isfield(motor, "continuous_torque_Nm"));
            testCase.verifyFalse(isfield(motor, "kv_no_load_rpm_per_VDC"));
            testCase.verifyFalse(isfield(motor, "kt_Nm_per_Arms"));
            testCase.verifyFalse(isfield(motor, "ingress_protection"));
            testCase.verifyFalse(isfield(motor, "max_temperature_C"));
            testCase.verifyFalse(isfield(motor, ...
                "combined_cooling_requires_air_and_liquid"));
            testCase.verifyFalse(isfield(motor, "motor_connection"));
        end
    end
end
