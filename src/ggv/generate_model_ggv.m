function ggv = generate_model_ggv(vehicle, tire, aero, powertrain, brake, options)
%GENERATE_MODEL_GGV Generate a speed-dependent whole-vehicle capability map.
%   Speed uses m/s. GGV acceleration fields use g. Each feasible point
%   allocates lateral demand by tire lateral capacity, applies a per-wheel
%   p-norm combined-slip envelope, and solves accel/brake load transfer as
%   separate fixed points. GGV acceleration is net CG acceleration, so drag
%   remains present at the lateral tire-force boundary.

arguments
    vehicle (1,1) struct
    tire (1,1) struct
    aero (1,1) struct = struct()
    powertrain (1,1) struct = struct()
    brake (1,1) struct = struct()
    options (1,1) struct = struct()
end

options = default_qss_options(options);
validateInputs(vehicle, tire, options);
powertrain = fillPowertrainDefaults(powertrain, options);
brake = fillBrakeDefaults(brake);
aero = fillAeroDefaults(aero);
validateDrivetrainConsistency(vehicle, powertrain);

vGrid_mps = options.v_grid_mps(:);
ayGrid_g = options.ay_grid_g(:).';
nSpeed = numel(vGrid_mps);
nLateral = numel(ayGrid_g);

axMax_g = zeros(nSpeed, nLateral);
axMin_g = zeros(nSpeed, nLateral);
feasible = false(nSpeed, nLateral);
accelLimiter = strings(nSpeed, nLateral);
brakeLimiter = strings(nSpeed, nLateral);
accelConverged = false(nSpeed, nLateral);
brakeConverged = false(nSpeed, nLateral);
accelResidual_mps2 = nan(nSpeed, nLateral);
brakeResidual_mps2 = nan(nSpeed, nLateral);
wheelLift = false(nSpeed, nLateral);
ayLimitPos_g = zeros(nSpeed, 1);
ayLimitNeg_g = zeros(nSpeed, 1);
lateralLimitTruncated = false(nSpeed, 2);
axLateralBoundary_g = zeros(nSpeed, 1);

for iSpeed = 1:nSpeed
    speed_mps = vGrid_mps(iSpeed);
    aeroForce = calc_aero_forces(speed_mps, aero);
    axLateralBoundary_g(iSpeed) = -aeroForce.drag_N ...
        / vehicle.mass.total_kg / options.gravity_mps2;
    [ayLimitPos_g(iSpeed), lateralLimitTruncated(iSpeed, 1)] = ...
        solveLateralLimit(speed_mps, 1, vehicle, tire, aeroForce, ...
        ayGrid_g, options);
    [negativeLimitMagnitude_g, lateralLimitTruncated(iSpeed, 2)] = ...
        solveLateralLimit(speed_mps, -1, vehicle, tire, aeroForce, ...
        ayGrid_g, options);
    ayLimitNeg_g(iSpeed) = -negativeLimitMagnitude_g;

    for iLateral = 1:nLateral
        ay_g = ayGrid_g(iLateral);
        isLateralFeasible = ay_g <= ayLimitPos_g(iSpeed) + 1e-12 ...
            && ay_g >= ayLimitNeg_g(iSpeed) - 1e-12;
        if ~isLateralFeasible
            accelLimiter(iSpeed, iLateral) = "lateral_infeasible";
            brakeLimiter(iSpeed, iLateral) = "lateral_infeasible";
            continue
        end

        feasible(iSpeed, iLateral) = true;
        [axMax_mps2, accelLimiter(iSpeed, iLateral), ...
            accelConverged(iSpeed, iLateral), ...
            accelResidual_mps2(iSpeed, iLateral), accelLift] = ...
            solveLongitudinalBranch("accel", speed_mps, ay_g, vehicle, ...
            tire, aeroForce, powertrain, brake, options);
        [axMin_mps2, brakeLimiter(iSpeed, iLateral), ...
            brakeConverged(iSpeed, iLateral), ...
            brakeResidual_mps2(iSpeed, iLateral), brakeLift] = ...
            solveLongitudinalBranch("brake", speed_mps, ay_g, vehicle, ...
            tire, aeroForce, powertrain, brake, options);

        axMax_g(iSpeed, iLateral) = axMax_mps2 / options.gravity_mps2;
        axMin_g(iSpeed, iLateral) = axMin_mps2 / options.gravity_mps2;
        wheelLift(iSpeed, iLateral) = accelLift || brakeLift;
    end
end

ggv.v_mps = vGrid_mps;
ggv.ay_g = ayGrid_g;
ggv.ax_max_g = axMax_g;
ggv.ax_min_g = axMin_g;
ggv.feasible = feasible;
ggv.ay_limit_pos_g = ayLimitPos_g;
ggv.ay_limit_neg_g = ayLimitNeg_g;
ggv.accel_limiter = accelLimiter;
ggv.brake_limiter = brakeLimiter;
ggv.solve_converged_accel = accelConverged;
ggv.solve_converged_brake = brakeConverged;
ggv.solve_residual_accel_mps2 = accelResidual_mps2;
ggv.solve_residual_brake_mps2 = brakeResidual_mps2;
ggv.wheel_lift = wheelLift;
ggv.lateral_limit_truncated = lateralLimitTruncated;
ggv.ax_max_lateral_boundary_g = axLateralBoundary_g;
ggv.ax_min_lateral_boundary_g = axLateralBoundary_g;
ggv.source = "model_" + string(tire.model_type) + "_v0.4";
ggv.notes = "Four-wheel loads, capacity-weighted Fy, p-norm combined slip";
ggv.gravity_mps2 = options.gravity_mps2;
ggv.aero_enabled = logical(aero.enabled);
ggv.provenance.vehicle = vehicle;
ggv.provenance.tire = tire;
ggv.provenance.powertrain = powertrain;
ggv.provenance.brake = brake;
ggv.options = options;
end

function [limit_g, truncated] = solveLateralLimit( ...
        speed_mps, turnSign, vehicle, tire, aeroForce, ayGrid_g, options)
searchUpper_g = max(abs(ayGrid_g));
upperMargin_N = lateralMargin(searchUpper_g * turnSign, speed_mps, ...
    vehicle, tire, aeroForce, options);
if upperMargin_N >= 0
    limit_g = searchUpper_g;
    truncated = true;
    return
end

lower_g = 0;
upper_g = searchUpper_g;
for iIteration = 1:60
    midpoint_g = 0.5 * (lower_g + upper_g);
    margin_N = lateralMargin(midpoint_g * turnSign, speed_mps, ...
        vehicle, tire, aeroForce, options);
    if margin_N >= 0
        lower_g = midpoint_g;
    else
        upper_g = midpoint_g;
    end
end
limit_g = lower_g;
truncated = false;
end

function margin_N = lateralMargin( ...
        ay_g, speed_mps, vehicle, tire, aeroForce, options)
state = make_vehicle_state(speed_mps, 0, ay_g * options.gravity_mps2);
state.gravity_mps2 = options.gravity_mps2;
state.suppress_warnings = true;
loads = calc_wheel_loads(state, vehicle, aeroForce);
env = tire_envelope(loads.Fz_vector_N, zeros(4, 1), tire);
requiredLateralForce_N = vehicle.mass.total_kg ...
    * abs(ay_g) * options.gravity_mps2;
margin_N = sum(env.Fy_max_N) - requiredLateralForce_N;
end

function [ax_mps2, limiter, converged, residual_mps2, hasWheelLift] = ...
        solveLongitudinalBranch(branch, speed_mps, ay_g, vehicle, tire, ...
        aeroForce, powertrain, brake, options)
hasWheelLift = false;
converged = false;
[minimumPitchAx_mps2, maximumPitchAx_mps2] = ...
    pitchAccelerationBounds(vehicle, aeroForce, options.gravity_mps2);
lowerBound_mps2 = max(minimumPitchAx_mps2, -5 * options.gravity_mps2);
upperBound_mps2 = min(maximumPitchAx_mps2, 5 * options.gravity_mps2);

previousAx_mps2 = clamp(0, lowerBound_mps2, upperBound_mps2);
[previousCandidate_mps2, ~, previousLift] = branchAcceleration( ...
    branch, previousAx_mps2, speed_mps, ay_g, vehicle, tire, ...
    aeroForce, powertrain, brake, options);
hasWheelLift = hasWheelLift || previousLift;
previousResidual_mps2 = previousCandidate_mps2 - previousAx_mps2;
axEstimate_mps2 = clamp(previousCandidate_mps2, ...
    lowerBound_mps2, upperBound_mps2);

for iIteration = 1:options.ggv_max_iterations
    [candidate_mps2, ~, thisWheelLift] = branchAcceleration( ...
        branch, axEstimate_mps2, speed_mps, ay_g, vehicle, tire, ...
        aeroForce, powertrain, brake, options);
    hasWheelLift = hasWheelLift || thisWheelLift;
    residual_mps2 = candidate_mps2 - axEstimate_mps2;
    if abs(residual_mps2) <= options.ggv_tolerance_mps2
        converged = true;
        break
    end

    residualDelta = residual_mps2 - previousResidual_mps2;
    if abs(residualDelta) > 1e-12
        nextAx_mps2 = axEstimate_mps2 - residual_mps2 ...
            * (axEstimate_mps2 - previousAx_mps2) / residualDelta;
    else
        nextAx_mps2 = axEstimate_mps2 ...
            + options.ggv_relaxation * residual_mps2;
    end
    previousAx_mps2 = axEstimate_mps2;
    previousResidual_mps2 = residual_mps2;
    axEstimate_mps2 = clamp(nextAx_mps2, ...
        lowerBound_mps2, upperBound_mps2);
end

[candidate_mps2, limiter, thisWheelLift] = branchAcceleration( ...
    branch, axEstimate_mps2, speed_mps, ay_g, vehicle, tire, ...
    aeroForce, powertrain, brake, options);
hasWheelLift = hasWheelLift || thisWheelLift;
residual_mps2 = candidate_mps2 - axEstimate_mps2;
ax_mps2 = candidate_mps2;
converged = converged || abs(residual_mps2) <= options.ggv_tolerance_mps2;
end

function [candidate_mps2, limiter, hasWheelLift] = branchAcceleration( ...
        branch, axEstimate_mps2, speed_mps, ay_g, vehicle, tire, ...
        aeroForce, powertrain, brake, options)
[minimumPitchAx_mps2, maximumPitchAx_mps2] = ...
    pitchAccelerationBounds(vehicle, aeroForce, options.gravity_mps2);
axForLoads_mps2 = clamp(axEstimate_mps2, ...
    minimumPitchAx_mps2, maximumPitchAx_mps2);
state = make_vehicle_state(speed_mps, axForLoads_mps2, ...
    ay_g * options.gravity_mps2);
state.gravity_mps2 = options.gravity_mps2;
state.suppress_warnings = true;
loads = calc_wheel_loads(state, vehicle, aeroForce);
env = tire_envelope(loads.Fz_vector_N, zeros(4, 1), tire);

totalFyCapacity_N = sum(env.Fy_max_N);
totalFyDemand_N = vehicle.mass.total_kg * ay_g * options.gravity_mps2;
if totalFyCapacity_N <= 0
    FyDemand_N = zeros(4, 1);
else
    FyDemand_N = totalFyDemand_N * env.Fy_max_N / totalFyCapacity_N;
end
availableFx_N = tire_combined_simple(env.Fx_max_N, env.Fy_max_N, ...
    FyDemand_N, tire.combined_n);
loads.Fx_available_N = availableFx_N;
loads.vehicle_mass_kg = vehicle.mass.total_kg;

switch branch
    case "accel"
        drive = calc_drive_limit( ...
            speed_mps, loads, tire, powertrain, options);
        limiter = drive.limiter;
        candidate_mps2 = (drive.Fx_drive_max_N - aeroForce.drag_N) ...
            / vehicle.mass.total_kg;
        if candidate_mps2 > maximumPitchAx_mps2
            candidate_mps2 = maximumPitchAx_mps2;
            limiter = "front_axle_lift";
        end
    case "brake"
        brakeResult = calc_brake_limit( ...
            speed_mps, loads, tire, brake, options);
        limiter = brakeResult.limiter;
        candidate_mps2 = brakeResult.ax_min_mps2 ...
            - aeroForce.drag_N / vehicle.mass.total_kg;
        if candidate_mps2 < minimumPitchAx_mps2
            candidate_mps2 = minimumPitchAx_mps2;
            limiter = "rear_axle_lift";
        end
    otherwise
        error("QSSLTS:GGVBranch", "Unknown GGV branch: %s", branch);
end
hasWheelLift = loads.has_wheel_lift;
end

function [minimumAx_mps2, maximumAx_mps2] = ...
        pitchAccelerationBounds(vehicle, aeroForce, gravity_mps2)
height_m = vehicle.mass.cg_height_m;
if height_m <= 0
    minimumAx_mps2 = -inf;
    maximumAx_mps2 = inf;
    return
end

staticLoads = calc_static_loads(vehicle, gravity_mps2);
frontNormal_N = staticLoads.Fz_front_total_N + aeroForce.downforce_front_N;
rearNormal_N = staticLoads.Fz_rear_total_N + aeroForce.downforce_rear_N;
scale = vehicle.geometry.wheelbase_m ...
    / (vehicle.mass.total_kg * height_m);
maximumAx_mps2 = frontNormal_N * scale;
minimumAx_mps2 = -rearNormal_N * scale;
end

function validateInputs(vehicle, tire, options)
requiredVehicleMass = ["total_kg", "front_static_frac", "cg_height_m"];
if ~isfield(vehicle, "mass") || ...
        ~all(isfield(vehicle.mass, cellstr(requiredVehicleMass))) ...
        || vehicle.mass.total_kg <= 0
    error("QSSLTS:VehicleParameters", ...
        "Vehicle mass parameters are invalid or incomplete.");
end
if ~isfield(tire, "model_type") || ...
        ~ismember(string(tire.model_type), ["constant_mu", "load_sensitive"])
    error("QSSLTS:TireModel", "Unsupported tire model for GGV generation.");
end
if any(diff(options.v_grid_mps(:)) <= 0) || options.v_grid_mps(1) < 0
    error("QSSLTS:GGVSpeedGrid", ...
        "options.v_grid_mps must be nonnegative and strictly increasing.");
end
if any(diff(options.ay_grid_g(:)) <= 0) || ...
        options.ay_grid_g(1) >= 0 || options.ay_grid_g(end) <= 0
    error("QSSLTS:GGVLateralGrid", ...
        "options.ay_grid_g must be increasing and span zero.");
end
if options.ggv_relaxation <= 0 || options.ggv_relaxation > 1
    error("QSSLTS:GGVRelaxation", "ggv_relaxation must be in (0,1].");
end
end

function model = fillPowertrainDefaults(model, options)
if ~isfield(model, "enabled"), model.enabled = false; end
if ~isfield(model, "max_power_W"), model.max_power_W = inf; end
if ~isfield(model, "max_total_wheel_torque_Nm")
    if isfield(model, "max_wheel_torque_Nm")
        model.max_total_wheel_torque_Nm = model.max_wheel_torque_Nm;
    else
        model.max_total_wheel_torque_Nm = inf;
    end
end
if ~isfield(model, "max_speed_mps"), model.max_speed_mps = options.v_max_mps; end
if ~isfield(model, "drive_efficiency"), model.drive_efficiency = 1; end
if ~isfield(model, "layout"), model.layout = "AWD"; end
end

function model = fillBrakeDefaults(model)
if ~isfield(model, "enabled"), model.enabled = true; end
if ~isfield(model, "max_decel_g_mechanical")
    model.max_decel_g_mechanical = inf;
end
end

function model = fillAeroDefaults(model)
if ~isfield(model, "enabled"), model.enabled = false; end
if ~isfield(model, "rho_kgpm3"), model.rho_kgpm3 = 1.225; end
if ~isfield(model, "CDA_m2"), model.CDA_m2 = 0; end
if ~isfield(model, "CLA_m2"), model.CLA_m2 = 0; end
if ~isfield(model, "front_downforce_frac")
    model.front_downforce_frac = 0.5;
end
end

function validateDrivetrainConsistency(vehicle, powertrain)
if powertrain.enabled && isfield(vehicle, "drivetrain") ...
        && isfield(vehicle.drivetrain, "layout") ...
        && upper(string(vehicle.drivetrain.layout)) ...
        ~= upper(string(powertrain.layout))
    error("QSSLTS:DrivetrainMismatch", ...
        "vehicle.drivetrain.layout and powertrain.layout must agree.");
end
end
