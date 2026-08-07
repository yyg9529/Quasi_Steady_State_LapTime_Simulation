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
    ["tianji_kart_QSS_track_closed", ...
    "fsec_hefei_2025_high_speed_avoidance_closed", ...
    "fsc_2025_acceleration_open", ...
    "fsc_2025_skidpad_event_open"], "track");
expectedTrackSource = trackPresetSourceFile(state.track_preset);
actualTrackSource = replace(string(state.track_source_file), "\", "/");
if actualTrackSource ~= expectedTrackSource
    fail("Track preset/source mismatch: " + state.track_preset + ...
        " requires " + expectedTrackSource);
end
requirePreset(state.vehicle_preset, "vehicle_baseline", "vehicle");
requirePreset(state.tire_preset, ...
    ["tire_load_sensitive_baseline", "tire_mfeval_qss_local"], ...
    "tire");
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

if state.tire_preset == "tire_mfeval_qss_local"
    envelopeFile = fullfile(projectRoot, "data", "tire", ...
        "Hoosier_16x75_10_R20.qss-envelope.mat");
    tire = load_qss_tire_envelope(envelopeFile);
    exported = tire;
else
    tire = tire_load_sensitive_baseline();
    exported = struct();
end
tire.Fz_ref_N = state.tire_Fz_ref_N;
tire.mu_x_ref = state.tire_mu_x_ref;
tire.mu_y_ref = state.tire_mu_y_ref;
tire.load_sensitivity_x = state.tire_load_sensitivity_x;
tire.load_sensitivity_y = state.tire_load_sensitivity_y;
tire.combined_n = state.tire_combined_n;
tire.rolling_radius_m = state.tire_rolling_radius_m;
if state.tire_preset == "tire_mfeval_qss_local"
    tire.provenance.gui_override_applied = any(abs([ ...
        tire.Fz_ref_N - exported.Fz_ref_N, ...
        tire.mu_x_ref - exported.mu_x_ref, ...
        tire.mu_y_ref - exported.mu_y_ref, ...
        tire.load_sensitivity_x - exported.load_sensitivity_x, ...
        tire.load_sensitivity_y - exported.load_sensitivity_y, ...
        tire.combined_n - exported.combined_n]) > 1e-12);
    tire.provenance.runtime_parameter_source = ...
        "offline QSS envelope initialized into user-editable GUI fields";
end

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
event = trackPresetEvent(state.track_preset);
if ismember(string(state.track_preset), ...
        ["fsc_2025_acceleration_open", ...
        "fsc_2025_skidpad_event_open"])
    options.start_speed_mps = 0;
    options.finish_speed_mps = NaN;
end

config = struct( ...
    track=track, ...
    vehicle=vehicle, ...
    models=models, ...
    options=options);
if ~isempty(fieldnames(event))
    config.event = event;
end
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
if ~ismember(actual, expected)
    fail("Unsupported " + groupName + " preset: " + actual);
end
end

function sourceFile = trackPresetSourceFile(preset)
switch string(preset)
    case "tianji_kart_QSS_track_closed"
        sourceFile = "data/track/tianji_kart_QSS_track_closed.csv";
    case "fsec_hefei_2025_high_speed_avoidance_closed"
        sourceFile = ...
            "data/track/fsec_hefei_2025_high_speed_avoidance_closed.csv";
    case "fsc_2025_acceleration_open"
        sourceFile = "data/track/fsc_2025_acceleration_open.csv";
    case "fsc_2025_skidpad_event_open"
        sourceFile = "data/track/fsc_2025_skidpad_event_open.csv";
    otherwise
        fail("Unsupported track preset: " + preset);
end
end

function event = trackPresetEvent(preset)
event = struct();
switch string(preset)
    case "fsc_2025_acceleration_open"
        event = struct( ...
            type="fsc_acceleration", ...
            rollout_distance_m=0.30, ...
            timed_distance_m=75, ...
            minimum_width_m=4.9);
    case "fsc_2025_skidpad_event_open"
        radius_m = 9.125;
        event = struct( ...
            type="fsc_skidpad", ...
            centerline_radius_m=radius_m, ...
            circle_length_m=2 * pi * radius_m, ...
            timed_laps=[2; 4], ...
            scoring_diameter_m=17.10);
end
end

function fail(message)
error("QSSLTS:GuiConfig", "%s", message);
end
