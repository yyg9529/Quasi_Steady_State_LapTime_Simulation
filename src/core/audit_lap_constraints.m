function audit = audit_lap_constraints(result, vehicle, models, options)
%AUDIT_LAP_CONSTRAINTS Audit applicable hard constraints at every lap node.
%   The audit is independent from active-constraint classification: a limit
%   can be inactive and pass, active and pass, or exceeded and fail.

arguments
    result (1,1) struct
    vehicle (1,1) struct
    models (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
validateResultContract(result);
nPoint = numel(result.v_mps);
checks = auditGgv(result, options);

hasTire = isfield(models, "tire") && ~isempty(models.tire);
if hasTire
    componentChecks = auditVehicleComponents( ...
        result, vehicle, models, options);
    checks.wheel_lift = componentChecks.wheel_lift;
    checks.combined_slip = componentChecks.combined_slip;
    checks.wheel_force = componentChecks.wheel_force;
    checks.brake = componentChecks.brake;
else
    checks.wheel_lift = notApplicable();
    checks.combined_slip = notApplicable();
    checks.wheel_force = notApplicable();
    checks.brake = notApplicable();
end

hasPowertrain = enabledPowertrain(models);
if hasPowertrain
    if ~isfield(result, "powertrain") || isempty(result.powertrain)
        error("QSSLTS:ConstraintAuditContract", ...
            "Enabled powertrain requires result.powertrain for auditing.");
    end
    powertrainChecks = auditPowertrain( ...
        result.powertrain, models.powertrain, nPoint);
    checks.wheel_force = combineChecks( ...
        checks.wheel_force, powertrainChecks.wheel_force);
    checks.motor_torque = powertrainChecks.motor_torque;
    checks.motor_speed = powertrainChecks.motor_speed;
    checks.motor_voltage = powertrainChecks.motor_voltage;
    checks.tsac_voltage = powertrainChecks.tsac_voltage;
    checks.tsac_power = powertrainChecks.tsac_power;
    checks.tsac_current = powertrainChecks.tsac_current;
    checks.battery_power = powertrainChecks.battery_power;
    checks.battery_current = powertrainChecks.battery_current;
    checks.inverter_power = powertrainChecks.inverter_power;
    checks.inverter_current = powertrainChecks.inverter_current;
    checks.phase_current = powertrainChecks.phase_current;
else
    checks.motor_torque = notApplicable();
    checks.motor_speed = notApplicable();
    checks.motor_voltage = notApplicable();
    checks.tsac_voltage = notApplicable();
    checks.tsac_power = notApplicable();
    checks.tsac_current = notApplicable();
    checks.battery_power = notApplicable();
    checks.battery_current = notApplicable();
    checks.inverter_power = notApplicable();
    checks.inverter_current = notApplicable();
    checks.phase_current = notApplicable();
end

audit.checks = checks;
audit.reason_codes = failedReasonCodes(checks);
audit.valid = isempty(audit.reason_codes);
if audit.valid
    audit.status = "pass";
else
    audit.status = "fail";
end
statuses = structfun(@(check) string(check.status), checks);
audit.applicable_check_count = nnz(statuses ~= "not_applicable");
audit.failed_check_count = nnz(statuses == "fail");
end

function checks = auditGgv(result, options)
nPoint = numel(result.v_mps);
ay_g = result.ay_mps2(:) / options.gravity_mps2;
axMaxMargin_mps2 = nan(nPoint, 1);
axMinMargin_mps2 = nan(nPoint, 1);
ayMargin_g = nan(nPoint, 1);
for iPoint = 1:nPoint
    cap = interp_ggv(result.ggv_used, result.v_mps(iPoint), ...
        ay_g(iPoint), options);
    axMaxMargin_mps2(iPoint) = ...
        cap.ax_max_mps2 - result.ax_mps2(iPoint);
    axMinMargin_mps2(iPoint) = ...
        result.ax_mps2(iPoint) - cap.ax_min_mps2;
    if ay_g(iPoint) >= 0
        ayMargin_g(iPoint) = cap.lateral_limit_g - ay_g(iPoint);
    else
        ayMargin_g(iPoint) = ay_g(iPoint) - cap.lateral_limit_g;
    end
end
checks.ggv_ay = makeCheck(ayMargin_g, ...
    options.accel_tolerance_mps2 / options.gravity_mps2);
checks.ggv_ax_max = makeCheck( ...
    axMaxMargin_mps2, options.accel_tolerance_mps2);
checks.ggv_ax_min = makeCheck( ...
    axMinMargin_mps2, options.accel_tolerance_mps2);
end

function checks = auditVehicleComponents(result, vehicle, models, options)
nPoint = numel(result.v_mps);
componentRelativeTolerance = 2e-4;
tire = models.tire;
aero = optionalModel(models, "aero");
hasBrake = isfield(models, "brake") && ~isempty(models.brake);
aeroForce = calc_aero_forces(result.v_mps, aero);
wheelLiftMargin = ones(nPoint, 1);
combinedSlipMargin_N = nan(nPoint, 1);
wheelForceMargin_N = nan(nPoint, 1);
brakeMargin_N = inf(nPoint, 1);
combinedSlipReference_N = nan(nPoint, 1);
wheelForceReference_N = nan(nPoint, 1);
brakeReference_N = nan(nPoint, 1);

for iPoint = 1:nPoint
    pointAero.downforce_total_N = aeroForce.downforce_total_N(iPoint);
    pointAero.downforce_front_N = aeroForce.downforce_front_N(iPoint);
    pointAero.downforce_rear_N = aeroForce.downforce_rear_N(iPoint);
    state = make_vehicle_state(result.v_mps(iPoint), ...
        result.ax_mps2(iPoint), result.ay_mps2(iPoint));
    state.gravity_mps2 = options.gravity_mps2;
    state.suppress_warnings = true;
    loads = calc_wheel_loads(state, vehicle, pointAero);
    envelope = tire_envelope(loads.Fz_vector_N, zeros(4, 1), tire);
    lateralDemand_N = distributeLateralDemand( ...
        vehicle.mass.total_kg * result.ay_mps2(iPoint), ...
        envelope.Fy_max_N);
    combinedAvailable_N = tire_combined_simple( ...
        envelope.Fx_max_N, envelope.Fy_max_N, lateralDemand_N, ...
        tire.combined_n);
    requiredWheelForce_N = vehicle.mass.total_kg ...
        * result.ax_mps2(iPoint) + aeroForce.drag_N(iPoint);
    wheelLiftMargin(iPoint) = 1 - 2 * double(loads.has_wheel_lift);
    wheelForceMargin_N(iPoint) = sum(envelope.Fx_max_N) ...
        - abs(requiredWheelForce_N);
    combinedSlipMargin_N(iPoint) = sum(combinedAvailable_N) ...
        - abs(requiredWheelForce_N);
    wheelForceReference_N(iPoint) = max( ...
        sum(envelope.Fx_max_N), abs(requiredWheelForce_N));
    combinedSlipReference_N(iPoint) = max( ...
        sum(combinedAvailable_N), abs(requiredWheelForce_N));
    if hasBrake && requiredWheelForce_N < 0
        loads.Fx_available_N = combinedAvailable_N;
        loads.vehicle_mass_kg = vehicle.mass.total_kg;
        brakeResult = calc_brake_limit(result.v_mps(iPoint), ...
            loads, tire, models.brake, options);
        brakeMargin_N(iPoint) = requiredWheelForce_N ...
            - brakeResult.Fx_brake_min_N;
        brakeReference_N(iPoint) = max( ...
            abs(brakeResult.Fx_brake_min_N), abs(requiredWheelForce_N));
    end
end

checks.wheel_lift = makeCheck(wheelLiftMargin, 0);
checks.combined_slip = makeCheck( ...
    combinedSlipMargin_N, relativeTolerance( ...
    combinedSlipReference_N, componentRelativeTolerance));
checks.wheel_force = makeCheck( ...
    wheelForceMargin_N, relativeTolerance( ...
    wheelForceReference_N, componentRelativeTolerance));
if hasBrake
    checks.brake = makeCheck( ...
        brakeMargin_N, relativeTolerance( ...
        brakeReference_N, componentRelativeTolerance));
else
    checks.brake = notApplicable();
end
end

function checks = auditPowertrain(point, powertrain, nPoint)
powertrain = validate_powertrain_config(powertrain);
required = ["motor_speed_rpm", "motor_stop_speed_rpm", ...
    "motor_torque_available_Nm", "motor_torque_used_Nm", ...
    "wheel_force_available_N", "wheel_force_used_N", ...
    "tsac_power_cap_W", "tsac_power_used_W", ...
    "tsac_dc_current_used_A", "battery_dc_current_used_A", ...
    "inverter_dc_current_used_A", "motor_phase_current_used_Arms", ...
    "inverter_phase_current_used_Arms", "V_bus_V"];
if ~all(isfield(point, cellstr(required))) ...
        || any(arrayfun(@(name) numel(point.(name)) ~= nPoint, required))
    error("QSSLTS:ConstraintAuditContract", ...
        "Powertrain node-result fields are incomplete or mis-sized.");
end

voltageEnvelope = calc_motor_voltage_envelope( ...
    point.motor_speed_rpm, point.V_bus_V, powertrain);
checks.wheel_force = makeCheck( ...
    point.wheel_force_available_N - point.wheel_force_used_N, ...
    relativeTolerance(point.wheel_force_available_N));
checks.motor_torque = makeCheck( ...
    point.motor_torque_available_Nm - point.motor_torque_used_Nm, ...
    relativeTolerance(point.motor_torque_available_Nm));
checks.motor_speed = makeCheck( ...
    min(point.motor_stop_speed_rpm, ...
    powertrain.motor.max_mechanical_speed_rpm) - point.motor_speed_rpm, ...
    relativeTolerance(point.motor_stop_speed_rpm));
checks.motor_voltage = makeCheck( ...
    voltageEnvelope.available_torque_Nm - point.motor_torque_used_Nm, ...
    relativeTolerance(voltageEnvelope.available_torque_Nm));

voltageLimit_V = min([powertrain.rules.max_ts_voltage_V, ...
    powertrain.battery.V_max_V, powertrain.inverter.V_dc_max_V]);
voltageMargin_V = min( ...
    voltageLimit_V - point.V_bus_V, ...
    point.V_bus_V - powertrain.battery.V_min_V);
checks.tsac_voltage = makeCheck( ...
    voltageMargin_V, relativeTolerance([ ...
    powertrain.battery.V_min_V, voltageLimit_V]));
hardTsacPowerCap_W = min( ...
    point.tsac_power_cap_W, powertrain.rules.max_ts_power_W);
checks.tsac_power = makeCheck( ...
    hardTsacPowerCap_W - point.tsac_power_used_W, ...
    relativeTolerance(hardTsacPowerCap_W));
checks.tsac_current = makeCheck( ...
    powertrain.rules.max_ts_current_A ...
    - point.tsac_dc_current_used_A, ...
    relativeTolerance(powertrain.rules.max_ts_current_A));
checks.battery_power = makeCheck( ...
    powertrain.battery.P_discharge_peak_W ...
    - point.tsac_power_used_W, ...
    relativeTolerance(powertrain.battery.P_discharge_peak_W));
checks.battery_current = makeCheck( ...
    powertrain.battery.I_discharge_peak_A ...
    - point.battery_dc_current_used_A, ...
    relativeTolerance(powertrain.battery.I_discharge_peak_A));
inverterPowerUsed_W = max(point.tsac_power_used_W ...
    - powertrain.battery.P_ts_aux_W, 0);
checks.inverter_power = makeCheck( ...
    powertrain.inverter.P_dc_peak_W - inverterPowerUsed_W, ...
    relativeTolerance(powertrain.inverter.P_dc_peak_W));
checks.inverter_current = makeCheck( ...
    powertrain.inverter.I_dc_peak_A - point.inverter_dc_current_used_A, ...
    relativeTolerance(powertrain.inverter.I_dc_peak_A));
phaseMargin_Arms = min( ...
    powertrain.motor.peak_phase_current_Arms ...
        - point.motor_phase_current_used_Arms, ...
    powertrain.inverter.I_phase_peak_Arms ...
        - point.inverter_phase_current_used_Arms);
checks.phase_current = makeCheck( ...
    phaseMargin_Arms, relativeTolerance([ ...
    powertrain.motor.peak_phase_current_Arms, ...
    powertrain.inverter.I_phase_peak_Arms]));
end

function check = combineChecks(left, right)
if left.status == "not_applicable"
    check = right;
    return
end
check = makeCheck(min(left.margin(:), right.margin(:)), ...
    max(left.tolerance, right.tolerance));
end

function demand_N = distributeLateralDemand(totalDemand_N, capacity_N)
totalCapacity_N = sum(capacity_N);
if totalCapacity_N > 0
    demand_N = totalDemand_N * capacity_N / totalCapacity_N;
else
    demand_N = zeros(size(capacity_N));
end
end

function check = makeCheck(margin, tolerance)
margin = margin(:);
violation = isnan(margin) | margin == -inf | margin < -tolerance;
check.margin = margin;
check.tolerance = tolerance;
check.violating_indices = find(violation);
if any(violation)
    check.status = "fail";
else
    check.status = "pass";
end
end

function check = notApplicable()
check.status = "not_applicable";
check.margin = zeros(0, 1);
check.tolerance = 0;
check.violating_indices = zeros(0, 1);
end

function reasonCodes = failedReasonCodes(checks)
names = ["ggv_ay", "ggv_ax_max", "ggv_ax_min", "wheel_lift", ...
    "combined_slip", "wheel_force", "brake", "motor_torque", ...
    "motor_speed", "motor_voltage", "tsac_voltage", "tsac_power", ...
    "tsac_current", "battery_power", "battery_current", ...
    "inverter_power", "inverter_current", "phase_current"];
codes = names + "_violation";
reasonCodes = strings(0, 1);
for iCheck = 1:numel(names)
    if checks.(names(iCheck)).status == "fail"
        reasonCodes(end + 1, 1) = codes(iCheck);
    end
end
end

function tolerance = relativeTolerance(reference, relativeScale)
if nargin < 2
    relativeScale = 1e-9;
end
finiteReference = abs(reference(isfinite(reference)));
if isempty(finiteReference)
    scale = 1;
else
    scale = max(1, max(finiteReference, [], "all"));
end
tolerance = relativeScale * scale;
end

function enabled = enabledPowertrain(models)
enabled = isfield(models, "powertrain") ...
    && isstruct(models.powertrain) && isscalar(models.powertrain) ...
    && isfield(models.powertrain, "enabled") ...
    && isscalar(models.powertrain.enabled) ...
    && logical(models.powertrain.enabled);
end

function model = optionalModel(models, name)
if isfield(models, name)
    model = models.(name);
else
    model = struct();
end
end

function validateResultContract(result)
required = ["v_mps", "ax_mps2", "ay_mps2", "ggv_used", "track"];
if ~all(isfield(result, cellstr(required))) ...
        || numel(result.v_mps) ~= numel(result.ax_mps2) ...
        || numel(result.v_mps) ~= numel(result.ay_mps2) ...
        || any(~isfinite(result.v_mps), "all") ...
        || any(~isfinite(result.ax_mps2), "all") ...
        || any(~isfinite(result.ay_mps2), "all")
    error("QSSLTS:ConstraintAuditContract", ...
        "Lap result is incomplete, mis-sized, or nonfinite.");
end
end
