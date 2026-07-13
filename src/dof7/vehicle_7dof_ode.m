function [dx, diagnostics] = vehicle_7dof_ode( ...
        ~, x, control, vehicle, tire, powertrain, brake)
%VEHICLE_7DOF_ODE Planar body 3DOF plus four wheel rotational DOF.
%   State order is [vx; vy; yaw_rate; omega_FL; omega_FR; omega_RL;
%   omega_RR]. Body axes are X forward, Y left, Z up; yaw is positive for
%   a left turn. This event-validation model is not used by the QSS core.

    arguments
        ~
        x (7, 1) double {mustBeFinite, mustBeReal}
        control (1, 1) struct
        vehicle (1, 1) struct
        tire (1, 1) struct
        powertrain (1, 1) struct
        brake (1, 1) struct
    end

    validateParameters(vehicle, tire);
    validateActuatorModels(vehicle, powertrain, brake);
    [steer_rad, throttle, brakeCommand] = validateControl(control);
    mass_kg = vehicle.mass.total_kg;
    yawInertia_kgm2 = vehicle.inertia.Iz_kgm2;
    wheelRadius_m = tire.rolling_radius_m;
    wheelInertia_kgm2 = tire.wheel_inertia_kgm2;

    vx_mps = x(1);
    vy_mps = x(2);
    yawRate_radps = x(3);
    wheelSpeed_radps = x(4:7);
    [wheelX_m, wheelY_m, wheelSteer_rad] = wheelGeometry( ...
        vehicle, steer_rad);
    [wheelVx_mps, wheelVy_mps] = wheelVelocities( ...
        vx_mps, vy_mps, yawRate_radps, wheelX_m, wheelY_m, ...
        wheelSteer_rad);

    regularization_mps = tire.slip_force.velocity_regularization_mps;
    referenceSpeed_mps = hypot(wheelVx_mps, regularization_mps);
    slipRatio = (wheelRadius_m .* wheelSpeed_radps - wheelVx_mps) ...
        ./ referenceSpeed_mps;
    slipAngle_rad = atan2(wheelVy_mps, referenceSpeed_mps);

    [loads, tireForce, FxBody_N, FyBody_N, loadSolution] = ...
        solveWheelForces( ...
        vx_mps, yawRate_radps, slipRatio, slipAngle_rad, ...
        wheelSteer_rad, vehicle, tire);
    totalFx_N = sum(FxBody_N);
    totalFy_N = sum(FyBody_N);
    yawMoment_Nm = sum(wheelX_m .* FyBody_N - wheelY_m .* FxBody_N);

    driveTorque_Nm = driveTorques( ...
        vx_mps, wheelSpeed_radps, throttle, powertrain, tire);
    brakeTorque_Nm = brakeTorques( ...
        wheelSpeed_radps, brakeCommand, brake, vehicle, tire);
    wheelAcceleration_radps2 = (driveTorque_Nm - brakeTorque_Nm ...
        - tireForce.Fx_N .* wheelRadius_m) ./ wheelInertia_kgm2;

    vxDerivative_mps2 = totalFx_N / mass_kg + yawRate_radps * vy_mps;
    vyDerivative_mps2 = totalFy_N / mass_kg - yawRate_radps * vx_mps;
    yawAcceleration_radps2 = yawMoment_Nm / yawInertia_kgm2;
    dx = [vxDerivative_mps2; vyDerivative_mps2; ...
        yawAcceleration_radps2; wheelAcceleration_radps2];

    diagnostics.wheel_order = ["FL"; "FR"; "RL"; "RR"];
    diagnostics.slip_ratio = slipRatio;
    diagnostics.slip_angle_rad = slipAngle_rad;
    diagnostics.wheel_vx_mps = wheelVx_mps;
    diagnostics.wheel_vy_mps = wheelVy_mps;
    diagnostics.Fz_N = loads.Fz_vector_N;
    diagnostics.Fx_wheel_N = tireForce.Fx_N;
    diagnostics.Fy_wheel_N = tireForce.Fy_N;
    diagnostics.Fx_body_N = FxBody_N;
    diagnostics.Fy_body_N = FyBody_N;
    diagnostics.drive_torque_Nm = driveTorque_Nm;
    diagnostics.brake_torque_Nm = brakeTorque_Nm;
    diagnostics.ax_body_mps2 = totalFx_N / mass_kg;
    diagnostics.ay_body_mps2 = totalFy_N / mass_kg;
    diagnostics.yaw_accel_radps2 = yawAcceleration_radps2;
    diagnostics.load_solution_converged = loadSolution.converged;
    diagnostics.load_solution_residual_mps2 = loadSolution.residual_mps2;
    diagnostics.load_solution_iterations = loadSolution.iterations;
end

function validateParameters(vehicle, tire)
    vehicleFields = ["mass", "geometry", "inertia", "load_transfer"];
    nestedFieldsPresent = all(isfield(vehicle, cellstr(vehicleFields))) ...
        && all(isfield(vehicle.mass, ...
        ["total_kg", "front_static_frac", "cg_height_m"])) ...
        && all(isfield(vehicle.geometry, ...
        ["wheelbase_m", "track_front_m", "track_rear_m"])) ...
        && isfield(vehicle.inertia, "Iz_kgm2") ...
        && isfield(vehicle.load_transfer, "front_lateral_distribution");
    if ~nestedFieldsPresent
        error("QSSLTS:DOF7Vehicle", ...
            "Vehicle mass, geometry, inertia, or load-transfer fields are missing.");
    end
    vehicleValues = [vehicle.mass.total_kg, ...
        vehicle.mass.front_static_frac, vehicle.mass.cg_height_m, ...
        vehicle.geometry.wheelbase_m, vehicle.geometry.track_front_m, ...
        vehicle.geometry.track_rear_m, vehicle.inertia.Iz_kgm2, ...
        vehicle.load_transfer.front_lateral_distribution];
    if ~all(isfinite(vehicleValues)) || ~all(isreal(vehicleValues)) ...
            || vehicle.mass.total_kg <= 0 ...
            || vehicle.mass.front_static_frac <= 0 ...
            || vehicle.mass.front_static_frac >= 1 ...
            || vehicle.mass.cg_height_m < 0 ...
            || vehicle.geometry.wheelbase_m <= 0 ...
            || vehicle.geometry.track_front_m <= 0 ...
            || vehicle.geometry.track_rear_m <= 0 ...
            || vehicle.inertia.Iz_kgm2 <= 0 ...
            || vehicle.load_transfer.front_lateral_distribution < 0 ...
            || vehicle.load_transfer.front_lateral_distribution > 1
        error("QSSLTS:DOF7Vehicle", ...
            "Vehicle parameters must be finite and inside physical ranges.");
    end
    tireFieldsMissing = ~isfield(tire, "rolling_radius_m") ...
            || ~isfield(tire, "wheel_inertia_kgm2") ...
            || ~isfield(tire, "slip_force") ...
            || ~isfield(tire.slip_force, "velocity_regularization_mps");
    if tireFieldsMissing
        error("QSSLTS:DOF7Tire", ...
            "7DOF tire radius, inertia, or regularization field is missing.");
    end
    tireValues = [tire.rolling_radius_m, tire.wheel_inertia_kgm2, ...
        tire.slip_force.velocity_regularization_mps];
    if ~all(isfinite(tireValues)) || ~all(isreal(tireValues)) ...
            || any(tireValues <= 0)
        error("QSSLTS:DOF7Tire", ...
            "7DOF tire radius, inertia, and regularization must be positive.");
    end
end

function validateActuatorModels(vehicle, powertrain, brake)
    powerFields = ["enabled", "layout", "max_power_W", ...
        "max_total_wheel_torque_Nm", "max_speed_mps", ...
        "drive_efficiency"];
    if ~all(isfield(powertrain, cellstr(powerFields)))
        error("QSSLTS:DOF7Powertrain", ...
            "7DOF powertrain parameters are incomplete.");
    end
    limits = [powertrain.max_power_W, ...
        powertrain.max_total_wheel_torque_Nm, powertrain.max_speed_mps];
    validLimits = all(arrayfun(@isNonnegativeLimit, limits));
    validEfficiency = isnumeric(powertrain.drive_efficiency) ...
        && isscalar(powertrain.drive_efficiency) ...
        && isreal(powertrain.drive_efficiency) ...
        && isfinite(powertrain.drive_efficiency) ...
        && powertrain.drive_efficiency >= 0 ...
        && powertrain.drive_efficiency <= 1;
    validEnabled = isLogicalScalar(powertrain.enabled);
    layout = upper(string(powertrain.layout));
    validLayout = isscalar(layout) && ismember(layout, ["FWD", "RWD", "AWD"]);
    finiteActuation = isfinite(powertrain.max_power_W) ...
        || isfinite(powertrain.max_total_wheel_torque_Nm);
    if ~validLimits || ~validEfficiency || ~validEnabled ...
            || ~validLayout || ~finiteActuation
        error("QSSLTS:DOF7Powertrain", ...
            "7DOF powertrain parameters are outside valid ranges.");
    end
    if isfield(vehicle, "drivetrain") ...
            && isfield(vehicle.drivetrain, "layout") ...
            && upper(string(vehicle.drivetrain.layout)) ~= layout
        error("QSSLTS:DOF7DrivetrainMismatch", ...
            "Vehicle and powertrain drivetrain layouts must match.");
    end

    if ~isfield(brake, "enabled") || ~isfield(brake, "front_bias") ...
            || ~isLogicalScalar(brake.enabled) ...
            || ~isnumeric(brake.front_bias) || ~isscalar(brake.front_bias) ...
            || ~isreal(brake.front_bias) || ~isfinite(brake.front_bias) ...
            || brake.front_bias < 0 || brake.front_bias > 1
        error("QSSLTS:DOF7Brake", ...
            "7DOF brake enabled/front_bias parameters are invalid.");
    end
    brakeLimits = [optionalField(brake, "max_total_brake_torque_Nm", inf), ...
        optionalField(brake, "max_total_brake_force_N", inf), ...
        optionalField(brake, "max_decel_g_mechanical", inf)];
    if ~all(arrayfun(@isNonnegativeLimit, brakeLimits)) ...
            || (brake.enabled && ~any(isfinite(brakeLimits)))
        error("QSSLTS:DOF7Brake", ...
            "7DOF brake limits must be nonnegative with one finite limit.");
    end
end

function valid = isNonnegativeLimit(value)
    valid = isnumeric(value) && isscalar(value) && isreal(value) ...
        && ~isnan(value) && value >= 0;
end

function valid = isLogicalScalar(value)
    valid = (islogical(value) || isnumeric(value)) && isscalar(value) ...
        && isreal(value) && isfinite(value) && (value == 0 || value == 1);
end

function [steer_rad, throttle, brakeCommand] = validateControl(control)
    required = ["steer_rad", "throttle", "brake"];
    if ~all(isfield(control, cellstr(required)))
        error("QSSLTS:DOF7Control", ...
            "Control must define steer_rad, throttle, and brake.");
    end
    steer_rad = control.steer_rad;
    throttle = control.throttle;
    brakeCommand = control.brake;
    numericScalars = isnumeric(steer_rad) && isnumeric(throttle) ...
        && isnumeric(brakeCommand) && isscalar(steer_rad) ...
        && isscalar(throttle) && isscalar(brakeCommand);
    if ~numericScalars || ~all(isfinite([steer_rad, throttle, brakeCommand])) ...
            || ~all(isreal([steer_rad, throttle, brakeCommand])) ...
            || ~isscalar(brakeCommand) || throttle < 0 || throttle > 1 ...
            || brakeCommand < 0 || brakeCommand > 1
        error("QSSLTS:DOF7Control", ...
            "Steer must be finite; throttle and brake must be in [0, 1].");
    end
end

function [wheelX_m, wheelY_m, wheelSteer_rad] = wheelGeometry( ...
        vehicle, steer_rad)
    wheelbase_m = vehicle.geometry.wheelbase_m;
    frontFraction = vehicle.mass.front_static_frac;
    cgToFront_m = (1 - frontFraction) * wheelbase_m;
    cgToRear_m = frontFraction * wheelbase_m;
    wheelX_m = [cgToFront_m; cgToFront_m; -cgToRear_m; -cgToRear_m];
    wheelY_m = [0.5 * vehicle.geometry.track_front_m; ...
        -0.5 * vehicle.geometry.track_front_m; ...
        0.5 * vehicle.geometry.track_rear_m; ...
        -0.5 * vehicle.geometry.track_rear_m];
    wheelSteer_rad = [steer_rad; steer_rad; 0; 0];
end

function [wheelVx_mps, wheelVy_mps] = wheelVelocities( ...
        vx_mps, vy_mps, yawRate_radps, wheelX_m, wheelY_m, steer_rad)
    contactVx_mps = vx_mps - yawRate_radps .* wheelY_m;
    contactVy_mps = vy_mps + yawRate_radps .* wheelX_m;
    cosine = cos(steer_rad);
    sine = sin(steer_rad);
    wheelVx_mps = cosine .* contactVx_mps + sine .* contactVy_mps;
    wheelVy_mps = -sine .* contactVx_mps + cosine .* contactVy_mps;
end

function [loads, force, FxBody_N, FyBody_N, solution] = solveWheelForces( ...
        vx_mps, yawRate_radps, slipRatio, slipAngle_rad, steer_rad, ...
        vehicle, tire)
    axGuess_mps2 = 0;
    ayGuess_mps2 = vx_mps * yawRate_radps;
    tolerance_mps2 = 1e-5;
    maximumIterations = 12;
    converged = false;
    residual_mps2 = inf;
    for iteration = 1:maximumIterations
        state.v_mps = max(vx_mps, 0);
        state.ax_mps2 = axGuess_mps2;
        state.ay_mps2 = ayGuess_mps2;
        state.suppress_warnings = true;
        loads = calc_wheel_loads(state, vehicle);
        force = tire_force_from_slip(slipRatio, slipAngle_rad, ...
            loads.Fz_vector_N, zeros(4, 1), tire);
        cosine = cos(steer_rad);
        sine = sin(steer_rad);
        FxBody_N = cosine .* force.Fx_N - sine .* force.Fy_N;
        FyBody_N = sine .* force.Fx_N + cosine .* force.Fy_N;
        nextAx_mps2 = sum(FxBody_N) / vehicle.mass.total_kg;
        nextAy_mps2 = sum(FyBody_N) / vehicle.mass.total_kg;
        residual_mps2 = max(abs([nextAx_mps2 - axGuess_mps2, ...
            nextAy_mps2 - ayGuess_mps2]));
        if residual_mps2 <= tolerance_mps2
            converged = true;
            break
        end
        axGuess_mps2 = nextAx_mps2;
        ayGuess_mps2 = nextAy_mps2;
    end
    solution.converged = converged;
    solution.residual_mps2 = residual_mps2;
    solution.iterations = iteration;
    if ~converged
        error("QSSLTS:DOF7LoadConvergence", ...
            "7DOF load solve failed after %d iterations (residual %.6g m/s^2).", ...
            iteration, residual_mps2);
    end
end

function torque_Nm = driveTorques( ...
        vx_mps, wheelSpeed_radps, throttle, powertrain, tire)
    torque_Nm = zeros(4, 1);
    if throttle == 0
        return
    end
    if ~isfield(powertrain, "enabled") || ~powertrain.enabled
        error("QSSLTS:DOF7Powertrain", ...
            "Positive throttle requires enabled powertrain; QSS disabled semantics differ.");
    end
    driven = drivenIndices(powertrain.layout);
    minimumWheelSpeed_radps = tire.slip_force.velocity_regularization_mps ...
        / tire.rolling_radius_m;
    meanWheelSpeed_radps = max( ...
        mean(abs(wheelSpeed_radps(driven))), minimumWheelSpeed_radps);
    powerTorque_Nm = powertrain.max_power_W ...
        * powertrain.drive_efficiency / meanWheelSpeed_radps;
    totalTorque_Nm = throttle * min( ...
        powertrain.max_total_wheel_torque_Nm, powerTorque_Nm);
    if vx_mps >= powertrain.max_speed_mps
        totalTorque_Nm = 0;
    end
    torque_Nm(driven) = totalTorque_Nm / numel(driven);
end

function torque_Nm = brakeTorques( ...
        wheelSpeed_radps, brakeCommand, brake, vehicle, tire)
    torque_Nm = zeros(4, 1);
    if ~isfield(brake, "enabled") || ~brake.enabled || brakeCommand == 0
        return
    end
    if isfield(brake, "regen_enabled") && brake.regen_enabled
        error("QSSLTS:DOF7Regen", ...
            "Regenerative braking is not implemented in the V0.8 7DOF model.");
    end
    maxTorque_Nm = optionalField( ...
        brake, "max_total_brake_torque_Nm", inf);
    maxForce_N = optionalField(brake, "max_total_brake_force_N", inf);
    maxDecel_g = optionalField(brake, "max_decel_g_mechanical", inf);
    if maxTorque_Nm < 0 || maxForce_N < 0 || maxDecel_g < 0
        error("QSSLTS:DOF7Brake", ...
            "7DOF brake torque, force, and deceleration limits must be nonnegative.");
    end
    gravity_mps2 = 9.80665;
    decelForceLimit_N = maxDecel_g * vehicle.mass.total_kg * gravity_mps2;
    forceBasedTorque_Nm = min(maxForce_N, decelForceLimit_N) ...
        * tire.rolling_radius_m;
    totalTorque_Nm = brakeCommand * min(maxTorque_Nm, forceBasedTorque_Nm);
    magnitudes_Nm = totalTorque_Nm * [brake.front_bias / 2; ...
        brake.front_bias / 2; (1 - brake.front_bias) / 2; ...
        (1 - brake.front_bias) / 2];
    signRegularization_radps = tire.slip_force.velocity_regularization_mps ...
        / tire.rolling_radius_m / 20;
    torque_Nm = magnitudes_Nm .* tanh( ...
        wheelSpeed_radps / signRegularization_radps);
end

function value = optionalField(input, fieldName, defaultValue)
    if isfield(input, fieldName)
        value = input.(fieldName);
    else
        value = defaultValue;
    end
end

function indices = drivenIndices(layout)
    switch upper(string(layout))
        case "FWD"
            indices = [1, 2];
        case "RWD"
            indices = [3, 4];
        case "AWD"
            indices = 1:4;
        otherwise
            error("QSSLTS:DrivetrainLayout", ...
                "Unsupported drivetrain layout: %s", string(layout));
    end
end
