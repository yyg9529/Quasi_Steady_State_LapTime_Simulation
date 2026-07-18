function state = validate_qsslts_gui_state(state)
%VALIDATE_QSSLTS_GUI_STATE Validate and normalize GUI parameter state.

arguments
    state (1,1) struct
end

defaults = qsslts_gui_default_state();
required = string(fieldnames(defaults));
missing = required(~isfield(state, required));
if ~isempty(missing)
    fail("Missing GUI parameter: " + strjoin(missing, ", "));
end

textFields = ["track_preset", "track_source_file", ...
    "vehicle_preset", "vehicle_drivetrain_layout", ...
    "tire_preset", "aero_preset", "brake_preset", ...
    "powertrain_preset"];
for name = textFields
    state.(name) = requiredText(state.(name), name);
end
state.track_source_file = replace(state.track_source_file, "\", "/");
state.vehicle_drivetrain_layout = ...
    upper(state.vehicle_drivetrain_layout);
if ~ismember(state.vehicle_drivetrain_layout, ["RWD", "FWD", "AWD"])
    fail("vehicle_drivetrain_layout must be RWD, FWD, or AWD.");
end

state.endurance_num_laps = positiveScalar( ...
    state.endurance_num_laps, "endurance_num_laps");
if state.endurance_num_laps ~= fix(state.endurance_num_laps)
    fail("endurance_num_laps must be a positive integer.");
end
state.endurance_safety_factor = boundedScalar( ...
    state.endurance_safety_factor, "endurance_safety_factor", 1, Inf);

positiveFields = [ ...
    "vehicle_mass_total_kg", "vehicle_wheelbase_m", ...
    "vehicle_track_front_m", "vehicle_track_rear_m", ...
    "vehicle_inertia_Iz_kgm2", "tire_Fz_ref_N", ...
    "tire_mu_x_ref", "tire_mu_y_ref", "tire_combined_n", ...
    "tire_rolling_radius_m", "brake_max_total_brake_force_N", ...
    "powertrain_gear_ratio", "motor_max_mechanical_speed_rpm", ...
    "motor_physical_peak_power_W", ...
    "motor_physical_peak_power_rpm", ...
    "motor_physical_cont_power_W", "motor_peak_torque_Nm", ...
    "motor_cont_torque_Nm", ...
    "motor_required_voltage_peak_power_V", ...
    "motor_peak_phase_current_Arms", ...
    "motor_cont_phase_current_Arms", ...
    "motor_Kv_no_load_rpm_per_V", ...
    "motor_Kv_nominal_load_rpm_per_V", ...
    "motor_Kv_peak_load_rpm_per_V", "motor_Kt_Nm_per_Arms", ...
    "battery_V_max_V", "battery_V_nominal_V", "battery_V_min_V", ...
    "battery_V_bus_assumed_V", "battery_E_nominal_kWh", ...
    "battery_P_discharge_peak_W", "battery_I_discharge_peak_A", ...
    "inverter_V_dc_max_V", "inverter_P_dc_peak_W", ...
    "inverter_I_dc_peak_A", "inverter_I_phase_peak_Arms", ...
    "options_v_max_mps", "options_v_grid_step_mps", ...
    "options_solver_tolerance_mps"];
for name = positiveFields
    state.(name) = positiveScalar(state.(name), name);
end

nonnegativeFields = ["vehicle_cg_height_m", "aero_CLA_m2", ...
    "aero_CDA_m2", "brake_max_decel_g_mechanical", ...
    "brake_max_total_brake_torque_Nm", "battery_P_ts_aux_W"];
for name = nonnegativeFields
    state.(name) = boundedScalar(state.(name), name, 0, Inf);
end

fractionFields = ["vehicle_front_static_frac", ...
    "vehicle_front_lateral_load_transfer_frac", ...
    "aero_front_downforce_frac", "brake_front_bias", ...
    "battery_SOC_init", "battery_SOC_min"];
for name = fractionFields
    state.(name) = boundedScalar(state.(name), name, 0, 1);
end

efficiencyFields = ["powertrain_drivetrain_efficiency", ...
    "motor_eta_const", "battery_eta_discharge", ...
    "inverter_eta_const"];
for name = efficiencyFields
    state.(name) = boundedScalar(state.(name), name, 0, 1);
    if state.(name) == 0
        fail(name + " must be greater than zero.");
    end
end

finiteFields = ["tire_load_sensitivity_x", ...
    "tire_load_sensitivity_y"];
for name = finiteFields
    state.(name) = finiteScalar(state.(name), name);
end

booleanFields = ["brake_enabled", "brake_force_limit_enabled"];
for name = booleanFields
    state.(name) = booleanScalar(state.(name), name);
end

if state.motor_physical_cont_power_W ...
        > state.motor_physical_peak_power_W ...
        || state.motor_cont_torque_Nm > state.motor_peak_torque_Nm ...
        || state.motor_cont_phase_current_Arms ...
        > state.motor_peak_phase_current_Arms
    fail("Motor continuous ratings must not exceed peak ratings.");
end
if state.battery_V_min_V > state.battery_V_nominal_V ...
        || state.battery_V_nominal_V > state.battery_V_max_V ...
        || state.battery_V_bus_assumed_V > state.battery_V_max_V
    fail("Battery voltage values are inconsistent.");
end
if state.battery_SOC_min >= state.battery_SOC_init
    fail("battery_SOC_min must be less than battery_SOC_init.");
end

normalized = defaults;
for name = required.'
    normalized.(name) = state.(name);
end
state = normalized;
end

function value = requiredText(value, name)
if ~(ischar(value) || (isstring(value) && isscalar(value)))
    fail(name + " must be a text scalar.");
end
value = strtrim(string(value));
if ismissing(value) || strlength(value) == 0
    fail(name + " must not be empty.");
end
end

function value = positiveScalar(value, name)
value = boundedScalar(value, name, 0, Inf);
if value == 0
    fail(name + " must be greater than zero.");
end
end

function value = finiteScalar(value, name)
if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) ...
        || ~isfinite(value)
    fail(name + " must be a finite real scalar.");
end
value = double(value);
end

function value = boundedScalar(value, name, lower, upper)
value = finiteScalar(value, name);
if value < lower || value > upper
    fail(name + " must be in [" + string(lower) ...
        + ", " + string(upper) + "].");
end
end

function value = booleanScalar(value, name)
if ~(islogical(value) || isnumeric(value)) || ~isscalar(value) ...
        || ~ismember(double(value), [0, 1])
    fail(name + " must be a scalar logical value.");
end
value = logical(value);
end

function fail(message)
error("QSSLTS:GuiConfig", "%s", message);
end
