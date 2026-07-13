function trace = evaluate_7dof_trajectory( ...
        t_s, x, controlSource, vehicle, tire, powertrain, brake)
%EVALUATE_7DOF_TRAJECTORY Re-evaluate forces and accelerations on a solution.

    arguments
        t_s (:, 1) double {mustBeFinite, mustBeNonnegative}
        x (:, 7) double {mustBeFinite, mustBeReal}
        controlSource
        vehicle (1, 1) struct
        tire (1, 1) struct
        powertrain (1, 1) struct
        brake (1, 1) struct
    end

    if size(x, 1) ~= numel(t_s)
        error("QSSLTS:DOF7Trajectory", ...
            "Time and state histories must contain the same number of rows.");
    end

    count = numel(t_s);
    derivative = zeros(count, 7);
    axBody_mps2 = zeros(count, 1);
    ayBody_mps2 = zeros(count, 1);
    yawAcceleration_radps2 = zeros(count, 1);
    maxCombinedUsage = zeros(count, 1);
    loadSolutionConverged = false(count, 1);
    loadSolutionResidual_mps2 = zeros(count, 1);
    throttle = zeros(count, 1);
    brakeCommand = zeros(count, 1);
    steer_rad = zeros(count, 1);
    for index = 1:count
        control = resolveControl(controlSource, t_s(index), x(index, :).');
        [stateDerivative, diagnostics] = vehicle_7dof_ode( ...
            t_s(index), x(index, :).', control, ...
            vehicle, tire, powertrain, brake);
        derivative(index, :) = stateDerivative.';
        axBody_mps2(index) = diagnostics.ax_body_mps2;
        ayBody_mps2(index) = diagnostics.ay_body_mps2;
        yawAcceleration_radps2(index) = diagnostics.yaw_accel_radps2;
        usage = normalizedForceUsage(diagnostics, tire);
        maxCombinedUsage(index) = max(usage);
        loadSolutionConverged(index) = diagnostics.load_solution_converged;
        loadSolutionResidual_mps2(index) = ...
            diagnostics.load_solution_residual_mps2;
        throttle(index) = control.throttle;
        brakeCommand(index) = control.brake;
        steer_rad(index) = control.steer_rad;
    end

    trace.derivative = derivative;
    trace.ax_body_mps2 = axBody_mps2;
    trace.ay_body_mps2 = ayBody_mps2;
    trace.yaw_accel_radps2 = yawAcceleration_radps2;
    trace.max_combined_usage = maxCombinedUsage;
    trace.load_solution_converged = loadSolutionConverged;
    trace.load_solution_residual_mps2 = loadSolutionResidual_mps2;
    trace.throttle = throttle;
    trace.brake = brakeCommand;
    trace.steer_rad = steer_rad;
end

function control = resolveControl(source, time_s, state)
    if isa(source, "function_handle")
        control = source(time_s, state);
    else
        control = source;
    end
end

function usage = normalizedForceUsage(diagnostics, tire)
    env = tire_envelope(diagnostics.Fz_N, zeros(4, 1), tire);
    xUsage = zeros(4, 1);
    yUsage = zeros(4, 1);
    loadedX = env.Fx_max_N > 0;
    loadedY = env.Fy_max_N > 0;
    xUsage(loadedX) = abs(diagnostics.Fx_wheel_N(loadedX)) ...
        ./ env.Fx_max_N(loadedX);
    yUsage(loadedY) = abs(diagnostics.Fy_wheel_N(loadedY)) ...
        ./ env.Fy_max_N(loadedY);
    usage = xUsage .^ tire.combined_n + yUsage .^ tire.combined_n;
end
