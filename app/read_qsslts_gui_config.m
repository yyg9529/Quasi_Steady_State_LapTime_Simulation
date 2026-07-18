function state = read_qsslts_gui_config(filePath)
%READ_QSSLTS_GUI_CONFIG Read and validate versioned GUI parameter JSON.

arguments
    filePath {mustBeTextScalar}
end

filePath = string(filePath);
if ~isfile(filePath)
    error("QSSLTS:GuiConfig", ...
        "Configuration file does not exist: %s", filePath);
end

try
    document = jsondecode(fileread(filePath));
catch exception
    error("QSSLTS:GuiConfig", ...
        "Invalid JSON configuration: %s", exception.message);
end

requiredHeader = ["schema", "schema_version", "units"];
missing = requiredHeader(~isfield(document, requiredHeader));
if ~isempty(missing)
    error("QSSLTS:GuiConfig", ...
        "Missing configuration header: %s", strjoin(missing, ", "));
end
if string(document.schema) ~= "qsslts-gui-parameters" ...
        || string(document.units) ~= "SI"
    error("QSSLTS:GuiConfig", ...
        "Unsupported GUI configuration schema or units.");
end

version = document.schema_version;
if ~isnumeric(version) || ~isscalar(version) || ~ismember(version, [1, 2])
    error("QSSLTS:GuiConfig", ...
        "Unsupported GUI configuration version.");
end
requiredTop = ["track", "endurance", "vehicle", "tire", ...
    "aero", "brake", "powertrain", "options"];
missing = requiredTop(~isfield(document, requiredTop));
if ~isempty(missing)
    error("QSSLTS:GuiConfig", ...
        "Missing configuration group: %s", strjoin(missing, ", "));
end

try
    if version == 1
        state = readVersion1(document);
    else
        state = readVersion2(document);
    end
catch exception
    error("QSSLTS:GuiConfig", ...
        "Missing or invalid configuration field: %s", exception.message);
end

state = validate_qsslts_gui_state(state);
end

function state = readVersion1(document)
state = qsslts_gui_default_state();
state.track_preset = string(document.track.preset);
state.track_source_file = string(document.track.source_file);
state.endurance_num_laps = document.endurance.num_laps;
state.endurance_safety_factor = document.endurance.safety_factor;
state.vehicle_preset = string(document.vehicle.preset);
state.vehicle_mass_total_kg = document.vehicle.mass_total_kg;
state.vehicle_wheelbase_m = document.vehicle.wheelbase_m;
state.vehicle_cg_height_m = document.vehicle.cg_height_m;
state.vehicle_front_static_frac = document.vehicle.front_static_frac;
state.vehicle_drivetrain_layout = ...
    string(document.vehicle.drivetrain_layout);
state.tire_preset = string(document.tire.preset);
state.tire_mu_x_ref = document.tire.mu_x_ref;
state.tire_mu_y_ref = document.tire.mu_y_ref;
state.tire_rolling_radius_m = document.tire.rolling_radius_m;
state.aero_preset = string(document.aero.preset);
state.aero_CLA_m2 = document.aero.CLA_m2;
state.aero_CDA_m2 = document.aero.CDA_m2;
state.brake_preset = string(document.brake.preset);
state.powertrain_preset = string(document.powertrain.preset);
state.powertrain_gear_ratio = document.powertrain.gear_ratio;
state.options_v_max_mps = document.options.v_max_mps;
state.options_v_grid_step_mps = document.options.v_grid_step_mps;
state.options_solver_tolerance_mps = ...
    document.options.solver_tolerance_mps;
end

function state = readVersion2(document)
state = qsslts_gui_default_state();
state.track_preset = string(document.track.preset);
state.track_source_file = string(document.track.source_file);
state.endurance_num_laps = document.endurance.num_laps;
state.endurance_safety_factor = document.endurance.safety_factor;

state.vehicle_preset = string(document.vehicle.preset);
state.vehicle_mass_total_kg = document.vehicle.mass_total_kg;
state.vehicle_wheelbase_m = document.vehicle.wheelbase_m;
state.vehicle_track_front_m = document.vehicle.track_front_m;
state.vehicle_track_rear_m = document.vehicle.track_rear_m;
state.vehicle_cg_height_m = document.vehicle.cg_height_m;
state.vehicle_front_static_frac = document.vehicle.front_static_frac;
state.vehicle_inertia_Iz_kgm2 = document.vehicle.inertia_Iz_kgm2;
state.vehicle_front_lateral_load_transfer_frac = ...
    document.vehicle.front_lateral_load_transfer_frac;
state.vehicle_drivetrain_layout = ...
    string(document.vehicle.drivetrain_layout);

state.tire_preset = string(document.tire.preset);
state.tire_Fz_ref_N = document.tire.Fz_ref_N;
state.tire_mu_x_ref = document.tire.mu_x_ref;
state.tire_mu_y_ref = document.tire.mu_y_ref;
state.tire_load_sensitivity_x = document.tire.load_sensitivity_x;
state.tire_load_sensitivity_y = document.tire.load_sensitivity_y;
state.tire_combined_n = document.tire.combined_n;
state.tire_rolling_radius_m = document.tire.rolling_radius_m;

state.aero_preset = string(document.aero.preset);
state.aero_CLA_m2 = document.aero.CLA_m2;
state.aero_CDA_m2 = document.aero.CDA_m2;
state.aero_front_downforce_frac = document.aero.front_downforce_frac;

state.brake_preset = string(document.brake.preset);
state.brake_enabled = document.brake.enabled;
state.brake_force_limit_enabled = document.brake.force_limit_enabled;
state.brake_max_total_brake_force_N = ...
    document.brake.max_total_brake_force_N;
state.brake_max_decel_g_mechanical = ...
    document.brake.max_decel_g_mechanical;
state.brake_max_total_brake_torque_Nm = ...
    document.brake.max_total_brake_torque_Nm;
state.brake_front_bias = document.brake.front_bias;

state.powertrain_preset = string(document.powertrain.preset);
state.powertrain_gear_ratio = document.powertrain.gear_ratio;
state.powertrain_drivetrain_efficiency = ...
    document.powertrain.drivetrain_efficiency;
state = readMotor(state, document.powertrain.motor);
state = readBattery(state, document.powertrain.battery);
state = readInverter(state, document.powertrain.inverter);

state.options_v_max_mps = document.options.v_max_mps;
state.options_v_grid_step_mps = document.options.v_grid_step_mps;
state.options_solver_tolerance_mps = ...
    document.options.solver_tolerance_mps;
end

function state = readMotor(state, motor)
state.motor_max_mechanical_speed_rpm = motor.max_mechanical_speed_rpm;
state.motor_physical_peak_power_W = motor.physical_peak_power_W;
state.motor_physical_peak_power_rpm = motor.physical_peak_power_rpm;
state.motor_physical_cont_power_W = motor.physical_cont_power_W;
state.motor_peak_torque_Nm = motor.peak_torque_Nm;
state.motor_cont_torque_Nm = motor.cont_torque_Nm;
state.motor_required_voltage_peak_power_V = ...
    motor.required_voltage_peak_power_V;
state.motor_peak_phase_current_Arms = motor.peak_phase_current_Arms;
state.motor_cont_phase_current_Arms = motor.cont_phase_current_Arms;
state.motor_Kv_no_load_rpm_per_V = motor.Kv_no_load_rpm_per_V;
state.motor_Kv_nominal_load_rpm_per_V = ...
    motor.Kv_nominal_load_rpm_per_V;
state.motor_Kv_peak_load_rpm_per_V = motor.Kv_peak_load_rpm_per_V;
state.motor_Kt_Nm_per_Arms = motor.Kt_Nm_per_Arms;
state.motor_eta_const = motor.eta_const;
end

function state = readBattery(state, battery)
state.battery_V_max_V = battery.V_max_V;
state.battery_V_nominal_V = battery.V_nominal_V;
state.battery_V_min_V = battery.V_min_V;
state.battery_V_bus_assumed_V = battery.V_bus_assumed_V;
state.battery_E_nominal_kWh = battery.E_nominal_kWh;
state.battery_SOC_init = battery.SOC_init;
state.battery_SOC_min = battery.SOC_min;
state.battery_P_discharge_peak_W = battery.P_discharge_peak_W;
state.battery_I_discharge_peak_A = battery.I_discharge_peak_A;
state.battery_eta_discharge = battery.eta_discharge;
state.battery_P_ts_aux_W = battery.P_ts_aux_W;
end

function state = readInverter(state, inverter)
state.inverter_V_dc_max_V = inverter.V_dc_max_V;
state.inverter_P_dc_peak_W = inverter.P_dc_peak_W;
state.inverter_I_dc_peak_A = inverter.I_dc_peak_A;
state.inverter_I_phase_peak_Arms = inverter.I_phase_peak_Arms;
state.inverter_eta_const = inverter.eta_const;
end
