function motor = emrax228_hv_cc()
%EMRAX228_HV_CC Return EMRAX 228 HV combined-cooling motor data.
%   Datasheet ratings are kept separate from the explicit V1 efficiency
%   assumption. The preset alone does not establish thermal capability.

motor.name = "EMRAX_228_HV_CC";
motor.source = "EMRAX 228 datasheet v1.6";
motor.source_url = "https://emrax.com/wp-content/uploads/2025/03/EMRAX_228_datasheet_v1.6.pdf";
motor.voltage_variant = "HV";
motor.cooling = "CC";

motor.mass_kg = 13.2;

motor.required_voltage_peak_power_V = 830;
motor.official_peak_efficiency = 0.96;

motor.physical_peak_power_W = 104e3;
motor.physical_peak_power_rpm = 4500;
motor.physical_peak_power_duration_s = 120;
motor.physical_peak_power_duty = "S2 2 min";
motor.physical_cont_power_W = 75e3;
motor.physical_cont_power_duty = "S1";
motor.peak_torque_Nm = 220;
motor.cont_torque_Nm = 130;
motor.max_mechanical_speed_rpm = 6500;

motor.Kv_no_load_rpm_per_V = 10.14;
motor.Kv_nominal_load_rpm_per_V = 7.85;
motor.Kv_peak_load_rpm_per_V = 5.65;
motor.Kt_Nm_per_Arms = 0.94;
motor.peak_phase_current_Arms = 235;
motor.cont_phase_current_Arms = 120;

motor.eta_const = 0.94;
motor.eta_const_source = "conservative engineering assumption";
motor.thermal_model_enabled = false;
end
