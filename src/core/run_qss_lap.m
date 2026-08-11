function result = run_qss_lap(track, vehicle, models, options)
%RUN_QSS_LAP Run the fixed-raceline quasi-steady-state lap simulation.
%   Track distances use m, speed m/s, acceleration m/s^2, and time s.

arguments
    track (1,1) struct
    vehicle (1,1) struct
    models (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
validateTrack(track);

[hasCompositePowertrain, hasEnabledCompositePowertrain, ...
    compositePowertrain] = compositePowertrainState(models);
hasPrebuiltGgv = isfield(models, "ggv") && ~isempty(models.ggv);
if hasCompositePowertrain && hasPrebuiltGgv
    validatePrebuiltPowertrainGgv(models.ggv, vehicle, ...
        requireModel(models, "tire"), optionalModel(models, "aero"), ...
        compositePowertrain, optionalModel(models, "brake"), options);
end

if hasPrebuiltGgv
    ggv = models.ggv;
else
    ggv = generate_model_ggv(vehicle, requireModel(models, "tire"), ...
        optionalModel(models, "aero"), optionalModel(models, "powertrain"), ...
        optionalModel(models, "brake"), options);
end
calibrationReport = prebuiltCalibrationReport(ggv);
if isfield(models, "ggv_real") && ~isempty(models.ggv_real)
    [ggv, calibrationReport] = calibrate_ggv( ...
        ggv, models.ggv_real, options);
end
ggvHealth = assess_ggv_health(ggv, options);
if ggvHealth.structure.status == "fail" ...
        || ggvHealth.finite_data.status == "fail" ...
        || ggvHealth.feasible_domain.status == "fail" ...
        || ggvHealth.gravity_consistency.status == "fail"
    error("QSSLTS:GGVUnusable", ...
        "GGV structural/data health failed: %s", ...
        strjoin(ggvHealth.reason_codes, ", "));
end

[vLat_mps, lateralLimiter] = calc_lateral_speed_limit(track, ggv, options);
profile_mps = vLat_mps;
maxChange_mps = inf;

for iIteration = 1:options.solver_max_iterations
    previous_mps = profile_mps;
    passOptions = options;
    passOptions.initial_profile_mps = profile_mps;
    if hasEnabledCompositePowertrain
        passOptions.segment_power_constraint = ...
            makeSegmentPowerConstraint(vehicle, ...
            optionalModel(models, "aero"), compositePowertrain);
    end
    [vForward_mps, ~, forwardInfo] = forward_pass( ...
        track, ggv, vLat_mps, passOptions);
    [profile_mps, ~, backwardInfo] = backward_pass( ...
        track, ggv, vForward_mps, vLat_mps, passOptions);
    maxChange_mps = max(abs(profile_mps - previous_mps));
    if maxChange_mps <= options.solver_tolerance_mps
        break
    end
end

converged = maxChange_mps <= options.solver_tolerance_mps ...
    && forwardInfo.converged && backwardInfo.converged;
if ~converged
    warning("QSSLTS:SolverNotConverged", ...
        "QSS speed propagation did not converge in %d iterations.", ...
        options.solver_max_iterations);
end

[ax_mps2, ay_mps2] = reconstructAcceleration(track, profile_mps);
limiter = classifyLimiters(profile_mps, vLat_mps, ...
    ax_mps2, ay_mps2, ggv, lateralLimiter, options);
[lapTime_s, segmentTime_s, cumulativeTime_s] = ...
    integrate_lap_time(track, profile_mps);

powertrainResult = struct();
energy = struct();
activeConstraints = struct();
if hasEnabledCompositePowertrain
    tire = requireModel(models, "tire");
    aero = optionalModel(models, "aero");
    endurance = requireModel(models, "endurance");
    powertrainResult = evaluate_lap_powertrain(track, profile_mps, ...
        ax_mps2, ay_mps2, vehicle, tire, aero, ...
        compositePowertrain, options);
    energy = calc_lap_energy(track, profile_mps, vehicle, aero, ...
        compositePowertrain, endurance);
    [activeConstraints, limiter] = classify_active_constraints( ...
        profile_mps, ax_mps2, ay_mps2, vLat_mps, lateralLimiter, ...
        ggv, powertrainResult, options);
    powertrainResult.endurance_energy_feasible = ...
        energy.can_finish_endurance_estimated;
end

result.lap_time_s = lapTime_s;
result.s_m = track.s_m(:);
result.v_mps = profile_mps;
result.ax_mps2 = ax_mps2;
result.ay_mps2 = ay_mps2;
result.limiter = limiter;
result.ggv_used = ggv;
result.options = options;
result.v_lateral_limit_mps = vLat_mps;
result.segment_time_s = segmentTime_s;
result.cumulative_time_s = cumulativeTime_s;
result.track = track;
result.calibration_report = calibrationReport;
result.solver.converged = converged;
result.solver.propagation_converged = converged;
result.solver.iterations = iIteration;
result.solver.max_change_mps = maxChange_mps;
result.propagation_converged = converged;
result.ggv_health = ggvHealth;
result.ggv_healthy = ggvHealth.valid;
if hasEnabledCompositePowertrain
    result.powertrain = powertrainResult;
    result.energy = energy;
    result.active_constraints = activeConstraints;
end
result.constraint_audit = audit_lap_constraints( ...
    result, vehicle, models, options);
result.valid = converged && ggvHealth.valid ...
    && result.constraint_audit.valid;
assessment = assess_analysis_result(result);
result.valid = assessment.valid;
result.status = assessment.status;
result.reject_reason = assessment.reject_reason;
end

function report = prebuiltCalibrationReport(ggv)
report = struct();
if isfield(ggv, "calibration_report") ...
        && isstruct(ggv.calibration_report)
    report = ggv.calibration_report;
    return
end
if isfield(ggv, "calibration_scale_table")
    report.scale_table = ggv.calibration_scale_table;
end
if isfield(ggv, "hard_limit_projection_report")
    report.hard_limit_projection = ...
        ggv.hard_limit_projection_report;
end
end

function constraint = makeSegmentPowerConstraint(vehicle, aero, powertrain)
constraint.vehicle_mass_kg = vehicle.mass.total_kg;
constraint.aero = aero;
constraint.powertrain = powertrain;
end

function [ax_mps2, ay_mps2] = reconstructAcceleration(track, v_mps)
isClosed = isfield(track, "is_closed") && track.is_closed;
if isClosed
    nextSpeed_mps = circshift(v_mps, -1);
    ax_mps2 = (nextSpeed_mps.^2 - v_mps.^2) ./ (2 * track.ds_m(:));
else
    segmentAx_mps2 = (v_mps(2:end).^2 - v_mps(1:end-1).^2) ...
        ./ (2 * track.ds_m(:));
    ax_mps2 = [segmentAx_mps2; segmentAx_mps2(end)];
end
ay_mps2 = v_mps.^2 .* track.kappa_1pm(:);
end

function limiter = classifyLimiters(v_mps, vLat_mps, ...
        ax_mps2, ay_mps2, ggv, lateralLimiter, options)
nPoint = numel(v_mps);
limiter = strings(nPoint, 1);
speedCap_mps = min(options.v_max_mps, ggv.v_mps(end));
for iPoint = 1:nPoint
    isAtLateralLimit = abs(v_mps(iPoint) - vLat_mps(iPoint)) ...
        <= options.limiter_tolerance_mps ...
        && lateralLimiter(iPoint) == "lateral";
    if isAtLateralLimit
        limiter(iPoint) = "lateral";
        continue
    end

    ay_g = ay_mps2(iPoint) / options.gravity_mps2;
    cap = interp_ggv(ggv, v_mps(iPoint), ay_g, options);
    isAtAccelerationLimit = cap.is_feasible ...
        && abs(ax_mps2(iPoint) - cap.ax_max_mps2) ...
        <= options.accel_tolerance_mps2;
    isAtBrakingLimit = cap.is_feasible ...
        && abs(ax_mps2(iPoint) - cap.ax_min_mps2) ...
        <= options.accel_tolerance_mps2;
    if isAtAccelerationLimit
        limiter(iPoint) = cap.accel_limiter;
    elseif isAtBrakingLimit
        limiter(iPoint) = cap.brake_limiter;
    elseif v_mps(iPoint) >= speedCap_mps ...
            - options.limiter_tolerance_mps
        limiter(iPoint) = "top_speed";
    elseif abs(ax_mps2(iPoint)) <= options.accel_tolerance_mps2
        limiter(iPoint) = "coasting";
    else
        limiter(iPoint) = "unconstrained";
    end
end
end

function model = requireModel(models, name)
if ~isfield(models, name) || isempty(models.(name))
    error("QSSLTS:MissingModel", "models.%s is required.", name);
end
model = models.(name);
end

function model = optionalModel(models, name)
if isfield(models, name)
    model = models.(name);
else
    model = struct();
end
end

function [present, enabled, powertrain] = compositePowertrainState(models)
present = false;
enabled = false;
powertrain = struct();
if ~isfield(models, "powertrain") || isempty(models.powertrain) ...
        || isNoPowertrainSentinel(models.powertrain)
    return
end
present = true;
powertrain = validate_powertrain_config(models.powertrain);
enabled = logical(powertrain.enabled);
end

function result = isNoPowertrainSentinel(powertrain)
names = string(fieldnames(powertrain));
isEmpty = isempty(names);
isMinimalDisabled = isequal(names, "enabled") ...
    && (islogical(powertrain.enabled) || isnumeric(powertrain.enabled)) ...
    && isscalar(powertrain.enabled) ...
    && ismember(double(powertrain.enabled), 0);
result = isEmpty || isMinimalDisabled;
end

function validatePrebuiltPowertrainGgv( ...
        ggv, vehicle, tire, aero, powertrain, brake, options)
try
    validProvenance = isfield(ggv, "provenance") ...
        && isstruct(ggv.provenance) ...
        && isscalar(ggv.provenance) ...
        && isfield(ggv.provenance, "vehicle") ...
        && isfield(ggv.provenance, "powertrain") ...
        && isfield(ggv.provenance, "tire") ...
        && isfield(ggv.provenance, "brake") ...
        && isfield(ggv, "options") ...
        && sameStruct(vehicleSignature(ggv.provenance.vehicle), ...
            vehicleSignature(vehicle)) ...
        && sameStruct(powertrainSignature(ggv.provenance.powertrain), ...
            powertrainSignature(powertrain)) ...
        && sameStruct(ggv.provenance.tire, ...
            tire_envelope_provenance(tire)) ...
        && sameStruct(normalizeBrake(ggv.provenance.brake), ...
            normalizeBrake(brake)) ...
        && sameStruct(generationOptionsSignature(ggv.options), ...
            generationOptionsSignature(options));
catch
    error("QSSLTS:PowertrainGGVProvenance", ...
        "Composite powertrain prebuilt GGV provenance is invalid.");
end
isAeroDisabled = isfield(ggv, "aero_enabled") ...
    && isscalar(ggv.aero_enabled) && ~logical(ggv.aero_enabled) ...
    && (~isfield(aero, "enabled") || ~logical(aero.enabled));
if ~validProvenance || ~isAeroDisabled
    error("QSSLTS:PowertrainGGVProvenance", ...
        "Composite powertrain prebuilt GGV provenance is invalid.");
end
end

function signature = vehicleSignature(vehicle)
signature.mass = selectFields(vehicle.mass, [ ...
    "total_kg", "front_static_frac", "cg_height_m"]);
signature.geometry = selectFields(vehicle.geometry, [ ...
    "wheelbase_m", "track_front_m", "track_rear_m"]);
signature.load_transfer = selectFields(vehicle.load_transfer, ...
    "front_lateral_distribution");
signature.drivetrain = selectFields(vehicle.drivetrain, "layout");
signature.drivetrain.layout = ...
    upper(string(signature.drivetrain.layout));
end

function signature = powertrainSignature(powertrain)
try
    powertrain = validate_powertrain_config(powertrain);
catch
    error("QSSLTS:PowertrainGGVProvenance", ...
        "Prebuilt GGV powertrain provenance is incomplete or invalid.");
end
signature.enabled = logical(powertrain.enabled);
signature.motor_count = powertrain.motor_count;
signature.layout = upper(string(powertrain.layout));
signature.gear_ratio = powertrain.gear_ratio;
signature.drivetrain_efficiency = powertrain.drivetrain_efficiency;
signature.motor = selectFields(powertrain.motor, [ ...
    "max_mechanical_speed_rpm", "physical_peak_power_W", ...
    "physical_peak_power_rpm", "physical_cont_power_W", ...
    "peak_torque_Nm", "cont_torque_Nm", ...
    "required_voltage_peak_power_V", "peak_phase_current_Arms", ...
    "cont_phase_current_Arms", "Kv_no_load_rpm_per_V", ...
    "Kv_nominal_load_rpm_per_V", "Kv_peak_load_rpm_per_V", ...
    "Kt_Nm_per_Arms", "eta_const", "thermal_model_enabled"]);
signature.battery = selectFields(powertrain.battery, [ ...
    "V_max_V", "V_nominal_V", "V_min_V", "V_bus_assumed_V", ...
    "E_nominal_kWh", "SOC_init", "SOC_min", ...
    "P_discharge_peak_W", "I_discharge_peak_A", ...
    "eta_discharge", "P_ts_aux_W"]);
signature.inverter = selectFields(powertrain.inverter, [ ...
    "V_dc_max_V", "P_dc_peak_W", "I_dc_peak_A", ...
    "I_phase_peak_Arms", "eta_const"]);
signature.rules = selectFields(powertrain.rules, [ ...
    "max_ts_voltage_V", "max_ts_power_W", "max_ts_current_A", ...
    "regen_enabled_in_model"]);
end

function brake = normalizeBrake(brake)
if ~isfield(brake, "enabled"), brake.enabled = true; end
if ~isfield(brake, "front_bias"), brake.front_bias = 0.5; end
if ~isfield(brake, "max_decel_g_mechanical")
    brake.max_decel_g_mechanical = inf;
end
if ~isfield(brake, "max_total_brake_force_N")
    brake.max_total_brake_force_N = inf;
end
if ~isfield(brake, "max_total_brake_torque_Nm")
    brake.max_total_brake_torque_Nm = inf;
end
end

function signature = generationOptionsSignature(options)
signature = selectFields(options, ["gravity_mps2", "v_grid_mps", ...
    "ay_grid_g", "ggv_max_iterations", "ggv_tolerance_mps2", ...
    "ggv_relaxation"]);
signature.v_grid_mps = signature.v_grid_mps(:);
signature.ay_grid_g = signature.ay_grid_g(:).';
end

function selected = selectFields(value, names)
selected = struct();
for index = 1:numel(names)
    name = names(index);
    selected.(name) = value.(name);
end
end

function result = sameStruct(left, right)
if ~isstruct(left) || ~isscalar(left) || ~isstruct(right) ...
        || ~isscalar(right)
    result = false;
    return
end
result = isequaln(orderfields(left), orderfields(right));
end

function validateTrack(track)
required = ["s_m", "ds_m", "kappa_1pm", "is_closed"];
if ~all(isfield(track, cellstr(required)))
    error("QSSLTS:TrackFields", "Track is missing required fields.");
end
if numel(track.s_m) ~= numel(track.kappa_1pm) ...
        || any(diff(track.s_m) <= 0) || any(track.ds_m <= 0)
    error("QSSLTS:TrackShape", "Track node and segment data are invalid.");
end
expectedSegments = numel(track.s_m) - 1 + double(track.is_closed);
if numel(track.ds_m) ~= expectedSegments
    error("QSSLTS:TrackSegments", ...
        "track.ds_m does not match the open/closed track contract.");
end
end
