function powertrain = powertrain_emrax228_hvcc_demo()
%POWERTRAIN_EMRAX228_HVCC_DEMO Return the concept-level V1 demo setup.
%   This configuration is an engineering demo and does not represent the
%   actual car.

powertrain.enabled = true;
powertrain.motor_count = 1;
powertrain.layout = "RWD";
powertrain.gear_ratio = 4.369334602435052;
powertrain.drivetrain_efficiency = 0.90;
powertrain.motor = emrax228_hv_cc();

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

powertrain.rules = fsg_2026_ev_rules();
end
