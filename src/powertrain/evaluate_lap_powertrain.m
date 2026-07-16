function result = evaluate_lap_powertrain(track, v_mps, ax_mps2, ...
        ay_mps2, vehicle, tire, aero, powertrain, options)
%EVALUATE_LAP_POWERTRAIN Reconstruct node-level drive usage and capability.
%   Capability uses actual node v/ax/ay for aero, wheel loads, combined
%   slip, traction, and composite powertrain constraints.

arguments
    track (1,1) struct
    v_mps double {mustBeFinite, mustBeNonnegative}
    ax_mps2 double {mustBeFinite}
    ay_mps2 double {mustBeFinite}
    vehicle (1,1) struct
    tire (1,1) struct
    aero (1,1) struct
    powertrain (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
powertrain = validate_powertrain_config(powertrain);
if ~logical(powertrain.enabled)
    error("QSSLTS:PowertrainDisabled", ...
        "Node powertrain evaluation requires powertrain.enabled=true.");
end

v_mps = v_mps(:);
ax_mps2 = ax_mps2(:);
ay_mps2 = ay_mps2(:);
validateInputs(track, v_mps, ax_mps2, ay_mps2, vehicle);
nPoint = numel(v_mps);

motorTorqueAvailable_Nm = zeros(nPoint, 1);
wheelForceAvailable_N = zeros(nPoint, 1);
tractionForceAvailable_N = zeros(nPoint, 1);
powertrainForceAvailable_N = zeros(nPoint, 1);
tsacPowerCap_W = zeros(nPoint, 1);
candidateTorque_Nm = zeros(nPoint, 12);
candidateNames = strings(1, 12);
candidateLimiters = strings(1, 12);
aeroForce = calc_aero_forces(v_mps, aero);

for iPoint = 1:nPoint
    pointAero.downforce_total_N = aeroForce.downforce_total_N(iPoint);
    pointAero.downforce_front_N = aeroForce.downforce_front_N(iPoint);
    pointAero.downforce_rear_N = aeroForce.downforce_rear_N(iPoint);
    state = make_vehicle_state(v_mps(iPoint), ax_mps2(iPoint), ...
        ay_mps2(iPoint));
    state.gravity_mps2 = options.gravity_mps2;
    state.suppress_warnings = true;
    loads = calc_wheel_loads(state, vehicle, pointAero);
    envelope = tire_envelope(loads.Fz_vector_N, zeros(4, 1), tire);

    totalLateralCapacity_N = sum(envelope.Fy_max_N);
    totalLateralDemand_N = vehicle.mass.total_kg * ay_mps2(iPoint);
    if totalLateralCapacity_N > 0
        lateralDemand_N = totalLateralDemand_N ...
            * envelope.Fy_max_N / totalLateralCapacity_N;
    else
        lateralDemand_N = zeros(4, 1);
    end
    loads.Fx_available_N = tire_combined_simple( ...
        envelope.Fx_max_N, envelope.Fy_max_N, lateralDemand_N, ...
        tire.combined_n);
    loads.vehicle_mass_kg = vehicle.mass.total_kg;
    drive = calc_drive_limit(v_mps(iPoint), loads, tire, ...
        powertrain, options);
    capability = drive.powertrain;

    motorTorqueAvailable_Nm(iPoint) = ...
        capability.available_motor_torque_Nm;
    wheelForceAvailable_N(iPoint) = drive.Fx_drive_max_N;
    tractionForceAvailable_N(iPoint) = drive.traction_limit_N;
    powertrainForceAvailable_N(iPoint) = ...
        drive.powertrain_force_limit_N;
    tsacPowerCap_W(iPoint) = capability.tsac_power_cap_W;
    candidateTorque_Nm(iPoint, :) = capability.candidate_torque_Nm;
    candidateNames = capability.candidate_names;
    candidateLimiters = capability.candidate_limiters;
end

wheelForceUsed_N = max(vehicle.mass.total_kg * ax_mps2 ...
    + aeroForce.drag_N, 0);
usageMask = true(nPoint, 1);
if ~logical(track.is_closed)
    usageMask(end) = false;
end
wheelForceUsed_N(~usageMask) = 0;

motorTorqueUsed_Nm = wheelForceUsed_N * tire.rolling_radius_m ...
    / (powertrain.gear_ratio * powertrain.drivetrain_efficiency);
tsacPowerUsed_W = zeros(nPoint, 1);
efficiency = powertrain.drivetrain_efficiency ...
    * powertrain.inverter.eta_const * powertrain.motor.eta_const;
tsacPowerUsed_W(usageMask) = wheelForceUsed_N(usageMask) ...
    .* v_mps(usageMask) / efficiency + powertrain.battery.P_ts_aux_W;
busVoltage_V = powertrain.battery.V_bus_assumed_V;
tsacCurrentUsed_A = tsacPowerUsed_W / busVoltage_V;
batteryCurrentUsed_A = tsacCurrentUsed_A;
inverterCurrentUsed_A = max(tsacPowerUsed_W ...
    - powertrain.battery.P_ts_aux_W .* usageMask, 0) / busVoltage_V;
motorPhaseCurrentUsed_Arms = motorTorqueUsed_Nm ...
    / powertrain.motor.Kt_Nm_per_Arms;
inverterPhaseCurrentUsed_Arms = motorPhaseCurrentUsed_Arms;

wheelSpeed_radps = v_mps / tire.rolling_radius_m;
motorSpeed_rpm = wheelSpeed_radps * powertrain.gear_ratio * 60 / (2 * pi);
voltageEnvelope = calc_motor_voltage_envelope(motorSpeed_rpm, ...
    busVoltage_V, powertrain);

result.motor_speed_rpm = motorSpeed_rpm;
result.motor_stop_speed_rpm = voltageEnvelope.stop_speed_rpm;
result.motor_torque_available_Nm = motorTorqueAvailable_Nm;
result.motor_torque_used_Nm = motorTorqueUsed_Nm;
result.wheel_force_available_N = wheelForceAvailable_N;
result.wheel_force_used_N = wheelForceUsed_N;
result.tsac_power_cap_W = tsacPowerCap_W;
result.tsac_power_used_W = tsacPowerUsed_W;
result.tsac_dc_current_used_A = tsacCurrentUsed_A;
result.battery_dc_current_used_A = batteryCurrentUsed_A;
result.inverter_dc_current_used_A = inverterCurrentUsed_A;
result.motor_phase_current_used_Arms = motorPhaseCurrentUsed_Arms;
result.inverter_phase_current_used_Arms = inverterPhaseCurrentUsed_Arms;
result.V_bus_V = repmat(busVoltage_V, nPoint, 1);
result.effective_voltage_V = voltageEnvelope.effective_voltage_V;
result.traction_force_available_N = tractionForceAvailable_N;
result.powertrain_force_available_N = powertrainForceAvailable_N;
result.candidate_names = candidateNames;
result.candidate_limiters = candidateLimiters;
result.candidate_torque_available_Nm = candidateTorque_Nm;
result.thermal_feasibility_evaluated = false;
result.regen_enabled = false;
result.voltage_scenario = voltageEnvelope.voltage_scenario(1);
end

function validateInputs(track, v_mps, ax_mps2, ay_mps2, vehicle)
if ~isfield(track, "s_m") || ~isfield(track, "is_closed") ...
        || ~isscalar(track.is_closed) ...
        || ~ismember(double(track.is_closed), [0, 1])
    error("QSSLTS:PowertrainTrack", "Track fields are invalid.");
end
nPoint = numel(track.s_m);
if nPoint < 2 || numel(v_mps) ~= nPoint ...
        || numel(ax_mps2) ~= nPoint || numel(ay_mps2) ~= nPoint
    error("QSSLTS:PowertrainTrack", ...
        "Track and node-state dimensions are inconsistent.");
end
if ~isfield(vehicle, "mass") ...
        || ~isfield(vehicle.mass, "total_kg") ...
        || ~isscalar(vehicle.mass.total_kg) ...
        || ~isfinite(vehicle.mass.total_kg) ...
        || vehicle.mass.total_kg <= 0
    error("QSSLTS:VehicleParameters", ...
        "A positive finite vehicle mass is required.");
end
end
