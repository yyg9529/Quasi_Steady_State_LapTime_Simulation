function config = qsslts_gui_state_to_runtime_config(state, projectRoot)
%QSSLTS_GUI_STATE_TO_RUNTIME_CONFIG Build solver inputs from GUI state.

arguments
    state (1,1) struct
    projectRoot {mustBeTextScalar}
end

state = validate_qsslts_gui_state(state);
projectRoot = string(projectRoot);
if ~isfolder(projectRoot)
    fail("Project root does not exist: " + projectRoot);
end

requirePreset(state.track_preset, ...
    "tianji_kart_QSS_track_closed", "track");
requirePreset(state.vehicle_preset, "vehicle_baseline", "vehicle");
requirePreset(state.tire_preset, ...
    "tire_load_sensitive_baseline", "tire");
requirePreset(state.aero_preset, "aero_baseline", "aero");
requirePreset(state.brake_preset, "brake_baseline", "brake");
requirePreset(state.powertrain_preset, ...
    "powertrain_emrax228_hvcc_demo", "powertrain");

trackFile = resolveProjectFile(projectRoot, state.track_source_file);
track = read_track_csv(trackFile);

vehicle = vehicle_baseline();
vehicle.mass.total_kg = state.vehicle_mass_total_kg;
vehicle.mass.front_static_frac = state.vehicle_front_static_frac;
vehicle.mass.cg_height_m = state.vehicle_cg_height_m;
vehicle.geometry.wheelbase_m = state.vehicle_wheelbase_m;
vehicle.geometry.track_front_m = state.vehicle_track_front_m;
vehicle.geometry.track_rear_m = state.vehicle_track_rear_m;
vehicle.inertia.Iz_kgm2 = state.vehicle_inertia_Iz_kgm2;
vehicle.load_transfer.front_lateral_distribution = ...
    state.vehicle_front_lateral_load_transfer_frac;
vehicle.drivetrain.layout = state.vehicle_drivetrain_layout;

tire = tire_load_sensitive_baseline();
tire.Fz_ref_N = state.tire_Fz_ref_N;
tire.mu_x_ref = state.tire_mu_x_ref;
tire.mu_y_ref = state.tire_mu_y_ref;
tire.load_sensitivity_x = state.tire_load_sensitivity_x;
tire.load_sensitivity_y = state.tire_load_sensitivity_y;
tire.combined_n = state.tire_combined_n;
tire.rolling_radius_m = state.tire_rolling_radius_m;

aero = aero_baseline();
aero.CLA_m2 = state.aero_CLA_m2;
aero.CDA_m2 = state.aero_CDA_m2;
aero.front_downforce_frac = state.aero_front_downforce_frac;

brake = brake_baseline();
brake.enabled = state.brake_enabled;
if state.brake_force_limit_enabled
    brake.max_total_brake_force_N = ...
        state.brake_max_total_brake_force_N;
else
    brake.max_total_brake_force_N = Inf;
end
brake.max_decel_g_mechanical = state.brake_max_decel_g_mechanical;
brake.max_total_brake_torque_Nm = ...
    state.brake_max_total_brake_torque_Nm;
brake.front_bias = state.brake_front_bias;

powertrain = powertrain_emrax228_hvcc_demo();
powertrain.layout = state.vehicle_drivetrain_layout;
powertrain.gear_ratio = state.powertrain_gear_ratio;
powertrain.drivetrain_efficiency = ...
    state.powertrain_drivetrain_efficiency;
powertrain.motor.max_mechanical_speed_rpm = ...
    state.motor_max_mechanical_speed_rpm;
powertrain.motor.physical_peak_power_W = ...
    state.motor_physical_peak_power_W;
powertrain.motor.physical_peak_power_rpm = ...
    state.motor_physical_peak_power_rpm;
powertrain.motor.physical_cont_power_W = ...
    state.motor_physical_cont_power_W;
powertrain.motor.peak_torque_Nm = state.motor_peak_torque_Nm;
powertrain.motor.cont_torque_Nm = state.motor_cont_torque_Nm;
powertrain.motor.required_voltage_peak_power_V = ...
    state.motor_required_voltage_peak_power_V;
powertrain.motor.peak_phase_current_Arms = ...
    state.motor_peak_phase_current_Arms;
powertrain.motor.cont_phase_current_Arms = ...
    state.motor_cont_phase_current_Arms;
powertrain.motor.Kv_no_load_rpm_per_V = ...
    state.motor_Kv_no_load_rpm_per_V;
powertrain.motor.Kv_nominal_load_rpm_per_V = ...
    state.motor_Kv_nominal_load_rpm_per_V;
powertrain.motor.Kv_peak_load_rpm_per_V = ...
    state.motor_Kv_peak_load_rpm_per_V;
powertrain.motor.Kt_Nm_per_Arms = state.motor_Kt_Nm_per_Arms;
powertrain.motor.eta_const = state.motor_eta_const;
powertrain.battery.V_max_V = state.battery_V_max_V;
powertrain.battery.V_nominal_V = state.battery_V_nominal_V;
powertrain.battery.V_min_V = state.battery_V_min_V;
powertrain.battery.V_bus_assumed_V = state.battery_V_bus_assumed_V;
powertrain.battery.E_nominal_kWh = state.battery_E_nominal_kWh;
powertrain.battery.SOC_init = state.battery_SOC_init;
powertrain.battery.SOC_min = state.battery_SOC_min;
powertrain.battery.P_discharge_peak_W = ...
    state.battery_P_discharge_peak_W;
powertrain.battery.I_discharge_peak_A = ...
    state.battery_I_discharge_peak_A;
powertrain.battery.eta_discharge = state.battery_eta_discharge;
powertrain.battery.P_ts_aux_W = state.battery_P_ts_aux_W;
powertrain.inverter.V_dc_max_V = state.inverter_V_dc_max_V;
powertrain.inverter.P_dc_peak_W = state.inverter_P_dc_peak_W;
powertrain.inverter.I_dc_peak_A = state.inverter_I_dc_peak_A;
powertrain.inverter.I_phase_peak_Arms = ...
    state.inverter_I_phase_peak_Arms;
powertrain.inverter.eta_const = state.inverter_eta_const;
powertrain = validate_powertrain_config(powertrain);

endurance = struct( ...
    num_laps=state.endurance_num_laps, ...
    safety_factor=state.endurance_safety_factor);
models = struct( ...
    tire=tire, ...
    aero=aero, ...
    brake=brake, ...
    powertrain=powertrain, ...
    endurance=endurance);

options = default_qss_options();
options.v_max_mps = state.options_v_max_mps;
options.v_grid_mps = makeSpeedGrid( ...
    state.options_v_max_mps, state.options_v_grid_step_mps);
options.solver_tolerance_mps = state.options_solver_tolerance_mps;

config = struct( ...
    track=track, ...
    vehicle=vehicle, ...
    models=models, ...
    options=options);
end

function filePath = resolveProjectFile(projectRoot, relativePath)
relativePath = replace(string(relativePath), "\", "/");
pathParts = split(relativePath, "/");
isDrivePath = ~isempty(regexp(char(relativePath), ...
    '^[A-Za-z]:', 'once'));
if startsWith(relativePath, "/") || isDrivePath ...
        || any(pathParts == "..")
    fail("track_source_file must remain inside the project root.");
end

filePath = fullfile(projectRoot, pathParts{:});
if ~isfile(filePath)
    fail("Track source file does not exist: " + relativePath);
end
end

function grid = makeSpeedGrid(vMax_mps, step_mps)
grid = (0:step_mps:vMax_mps).';
tolerance = max(8 * eps(vMax_mps), 8 * eps(step_mps));
if vMax_mps - grid(end) <= tolerance
    grid(end) = vMax_mps;
else
    grid(end + 1, 1) = vMax_mps;
end
end

function requirePreset(actual, expected, groupName)
if actual ~= expected
    fail("Unsupported " + groupName + " preset: " + actual);
end
end

function fail(message)
error("QSSLTS:GuiConfig", "%s", message);
end
