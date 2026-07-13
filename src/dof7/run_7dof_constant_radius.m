function result = run_7dof_constant_radius( ...
        radius_m, vehicle, tire, powertrain, brake, options)
%RUN_7DOF_CONSTANT_RADIUS Scan speed with a simple speed-hold controller.
%   This is a controlled event sanity check, not an independent tire-data
%   validation. A case is stable only if speed, curvature, and dynamics
%   remain inside explicit tolerances over the final analysis window.

    arguments
        radius_m (1, 1) double {mustBeFinite, mustBeNonzero}
        vehicle (1, 1) struct
        tire (1, 1) struct
        powertrain (1, 1) struct
        brake (1, 1) struct
        options (1, 1) struct = struct()
    end

    options = radiusDefaults(options);
    initialSpeedGrid_mps = options.speed_grid_mps(:);
    if isempty(initialSpeedGrid_mps) || any(~isfinite(initialSpeedGrid_mps)) ...
            || any(initialSpeedGrid_mps <= 0) ...
            || any(diff(initialSpeedGrid_mps) <= 0)
        error("QSSLTS:DOF7RadiusGrid", ...
            "Constant-radius speed_grid_mps must be positive and increasing.");
    end
    initialCount = numel(initialSpeedGrid_mps);
    capacity = initialCount + options.bracket_max_iterations;
    speedGrid_mps = nan(capacity, 1);
    caseSolutions = cell(capacity, 1);
    for index = 1:initialCount
        speedGrid_mps(index) = initialSpeedGrid_mps(index);
        caseSolutions{index} = runRadiusCase( ...
            radius_m, speedGrid_mps(index), ...
            vehicle, tire, powertrain, brake, options);
    end
    initialStable = cellfun(@(solution) solution.is_stable, ...
        caseSolutions(1:initialCount));
    firstInitialUnstable = find(~initialStable, 1, "first");
    if ~isempty(firstInitialUnstable) ...
            && any(initialStable(firstInitialUnstable + 1:end))
        error("QSSLTS:DOF7RadiusNonMonotonic", ...
            "Constant-radius stable/unstable classification is nonmonotonic.");
    end

    caseCount = initialCount;
    if ~isempty(firstInitialUnstable) && firstInitialUnstable > 1
        lowerSpeed_mps = initialSpeedGrid_mps(firstInitialUnstable - 1);
        upperSpeed_mps = initialSpeedGrid_mps(firstInitialUnstable);
        iteration = 0;
        while upperSpeed_mps - lowerSpeed_mps ...
                > options.bracket_speed_tolerance_mps ...
                && iteration < options.bracket_max_iterations
            iteration = iteration + 1;
            midpoint_mps = 0.5 * (lowerSpeed_mps + upperSpeed_mps);
            caseCount = caseCount + 1;
            speedGrid_mps(caseCount) = midpoint_mps;
            caseSolutions{caseCount} = runRadiusCase( ...
                radius_m, midpoint_mps, vehicle, tire, ...
                powertrain, brake, options);
            if caseSolutions{caseCount}.is_stable
                lowerSpeed_mps = midpoint_mps;
            else
                upperSpeed_mps = midpoint_mps;
            end
        end
    end

    speedGrid_mps = speedGrid_mps(1:caseCount);
    caseSolutions = caseSolutions(1:caseCount);
    [speedGrid_mps, order] = sort(speedGrid_mps);
    caseSolutions = caseSolutions(order);
    count = caseCount;
    achievedSpeed_mps = zeros(count, 1);
    achievedCurvature_1pm = zeros(count, 1);
    achievedAy_mps2 = zeros(count, 1);
    speedError_mps = zeros(count, 1);
    curvatureErrorFraction = zeros(count, 1);
    meanYawAcceleration_radps2 = zeros(count, 1);
    meanVyDerivative_mps2 = zeros(count, 1);
    peakSpeedError_mps = zeros(count, 1);
    peakCurvatureErrorFraction = zeros(count, 1);
    rmsYawAcceleration_radps2 = zeros(count, 1);
    rmsVyDerivative_mps2 = zeros(count, 1);
    isStable = false(count, 1);
    for index = 1:count
        solution = caseSolutions{index};
        achievedSpeed_mps(index) = solution.achieved_speed_mps;
        achievedCurvature_1pm(index) = solution.achieved_curvature_1pm;
        achievedAy_mps2(index) = solution.achieved_ay_mps2;
        speedError_mps(index) = solution.speed_error_mps;
        curvatureErrorFraction(index) = solution.curvature_error_frac;
        meanYawAcceleration_radps2(index) = ...
            solution.mean_yaw_accel_radps2;
        meanVyDerivative_mps2(index) = solution.mean_vy_derivative_mps2;
        peakSpeedError_mps(index) = solution.peak_speed_error_mps;
        peakCurvatureErrorFraction(index) = ...
            solution.peak_curvature_error_frac;
        rmsYawAcceleration_radps2(index) = ...
            solution.rms_yaw_accel_radps2;
        rmsVyDerivative_mps2(index) = solution.rms_vy_derivative_mps2;
        isStable(index) = solution.is_stable;
    end

    caseTable = table(speedGrid_mps, achievedSpeed_mps, ...
        achievedCurvature_1pm, achievedAy_mps2, speedError_mps, ...
        curvatureErrorFraction, meanYawAcceleration_radps2, ...
        meanVyDerivative_mps2, peakSpeedError_mps, ...
        peakCurvatureErrorFraction, rmsYawAcceleration_radps2, ...
        rmsVyDerivative_mps2, isStable, ...
        VariableNames=["target_speed_mps", "achieved_speed_mps", ...
        "achieved_curvature_1pm", "achieved_ay_mps2", ...
        "speed_error_mps", "curvature_error_frac", ...
        "mean_yaw_accel_radps2", "mean_vy_derivative_mps2", ...
        "peak_speed_error_mps", "peak_curvature_error_frac", ...
        "rms_yaw_accel_radps2", "rms_vy_derivative_mps2", ...
        "is_stable"]);
    stableAy_mps2 = abs(achievedAy_mps2(isStable));
    stableSpeeds_mps = achievedSpeed_mps(isStable);
    if isempty(stableAy_mps2)
        maxStableAy_mps2 = NaN;
        maxStableSpeed_mps = NaN;
    else
        [maxStableAy_mps2, maximumIndex] = max(stableAy_mps2);
        maxStableSpeed_mps = stableSpeeds_mps(maximumIndex);
    end
    firstUnstable = find(~isStable, 1, "first");
    if ~isempty(firstUnstable) && any(isStable(firstUnstable + 1:end))
        error("QSSLTS:DOF7RadiusNonMonotonic", ...
            "Constant-radius stable/unstable classification is nonmonotonic.");
    end
    limitBracketed = ~isempty(firstUnstable) && firstUnstable > 1;
    if limitBracketed
        lowerBracketSpeed_mps = speedGrid_mps(firstUnstable - 1);
        upperBracketSpeed_mps = speedGrid_mps(firstUnstable);
        bracketWidth_mps = upperBracketSpeed_mps - lowerBracketSpeed_mps;
        estimatedLimitSpeed_mps = 0.5 * ( ...
            lowerBracketSpeed_mps + upperBracketSpeed_mps);
        estimatedLimitAy_mps2 = estimatedLimitSpeed_mps ^ 2 / abs(radius_m);
    else
        lowerBracketSpeed_mps = NaN;
        upperBracketSpeed_mps = NaN;
        bracketWidth_mps = NaN;
        estimatedLimitSpeed_mps = NaN;
        estimatedLimitAy_mps2 = NaN;
    end
    limitRefined = limitBracketed ...
        && bracketWidth_mps <= options.bracket_speed_tolerance_mps;

    result.radius_m = radius_m;
    result.case_table = caseTable;
    result.case_solutions = caseSolutions;
    result.max_stable_ay_mps2 = maxStableAy_mps2;
    result.max_stable_speed_mps = maxStableSpeed_mps;
    result.limit_bracketed = limitBracketed;
    result.limit_refined = limitRefined;
    result.max_stable_is_lower_bound = isfinite(maxStableAy_mps2);
    result.lower_bracket_speed_mps = lowerBracketSpeed_mps;
    result.upper_bracket_speed_mps = upperBracketSpeed_mps;
    result.bracket_width_mps = bracketWidth_mps;
    result.estimated_limit_speed_mps = estimatedLimitSpeed_mps;
    result.estimated_limit_ay_mps2 = estimatedLimitAy_mps2;
    result.comparison = compareLateral(options, ...
        estimatedLimitSpeed_mps, estimatedLimitAy_mps2, sign(radius_m), ...
        isfinite(maxStableAy_mps2), limitBracketed, limitRefined, ...
        vehicle, tire, powertrain, brake);
end

function solution = runRadiusCase(radius_m, targetSpeed_mps, ...
        vehicle, tire, powertrain, brake, options)
    steer_rad = sign(radius_m) ...
        * atan(vehicle.geometry.wheelbase_m / abs(radius_m));
    controlSource = @(time_s, state) radiusControl( ...
        time_s, state, targetSpeed_mps, radius_m, steer_rad, options);
    x0 = make_7dof_initial_state(targetSpeed_mps, tire);
    x0(3) = targetSpeed_mps / radius_m;
    odeFunction = @(time_s, state) vehicle_7dof_ode( ...
        time_s, state, controlSource(time_s, state), ...
        vehicle, tire, powertrain, brake);
    odeOptions = odeset("RelTol", options.relative_tolerance, ...
        "AbsTol", options.absolute_tolerance, ...
        "MaxStep", options.max_step_s);
    [t_s, x] = ode15s(odeFunction, [0, options.settle_time_s], x0, odeOptions);
    trace = evaluate_7dof_trajectory(t_s, x, controlSource, ...
        vehicle, tire, powertrain, brake);
    window = t_s >= t_s(end) - options.analysis_window_s;
    meanSpeed_mps = mean(x(window, 1));
    meanYawRate_radps = mean(x(window, 3));
    achievedCurvature_1pm = meanYawRate_radps / meanSpeed_mps;
    targetCurvature_1pm = 1 / radius_m;
    curvatureErrorFraction = abs( ...
        (achievedCurvature_1pm - targetCurvature_1pm) ...
        / targetCurvature_1pm);
    speedError_mps = meanSpeed_mps - targetSpeed_mps;
    meanYawAcceleration_radps2 = mean(trace.yaw_accel_radps2(window));
    meanVyDerivative_mps2 = mean(trace.derivative(window, 2));
    speedHistory_mps = x(window, 1);
    curvatureHistory_1pm = x(window, 3) ./ speedHistory_mps;
    peakSpeedError_mps = max(abs(speedHistory_mps - targetSpeed_mps));
    peakCurvatureErrorFraction = max(abs( ...
        (curvatureHistory_1pm - targetCurvature_1pm) ...
        / targetCurvature_1pm));
    rmsYawAcceleration_radps2 = sqrt(mean( ...
        trace.yaw_accel_radps2(window) .^ 2));
    rmsVyDerivative_mps2 = sqrt(mean(trace.derivative(window, 2) .^ 2));
    stable = peakSpeedError_mps <= options.speed_tolerance_mps ...
        && peakCurvatureErrorFraction <= options.curvature_tolerance_frac ...
        && rmsYawAcceleration_radps2 ...
        <= options.yaw_accel_tolerance_radps2 ...
        && rmsVyDerivative_mps2 <= options.vy_dot_tolerance_mps2;

    solution.t_s = t_s;
    solution.x = x;
    solution.trace = trace;
    solution.achieved_speed_mps = meanSpeed_mps;
    solution.achieved_curvature_1pm = achievedCurvature_1pm;
    solution.achieved_ay_mps2 = mean(trace.ay_body_mps2(window));
    solution.speed_error_mps = speedError_mps;
    solution.curvature_error_frac = curvatureErrorFraction;
    solution.mean_yaw_accel_radps2 = meanYawAcceleration_radps2;
    solution.mean_vy_derivative_mps2 = meanVyDerivative_mps2;
    solution.peak_speed_error_mps = peakSpeedError_mps;
    solution.peak_curvature_error_frac = peakCurvatureErrorFraction;
    solution.rms_yaw_accel_radps2 = rmsYawAcceleration_radps2;
    solution.rms_vy_derivative_mps2 = rmsVyDerivative_mps2;
    solution.is_stable = stable;
end

function control = radiusControl( ...
        ~, state, targetSpeed_mps, radius_m, baseSteer_rad, options)
    effort = options.speed_controller_gain_per_mps ...
        * (targetSpeed_mps - state(1));
    targetYawRate_radps = state(1) / radius_m;
    steerCorrection_rad = options.yaw_rate_controller_gain_s ...
        * (targetYawRate_radps - state(3));
    control.steer_rad = clamp(baseSteer_rad + steerCorrection_rad, ...
        -options.max_steer_rad, options.max_steer_rad);
    control.throttle = clamp(effort, 0, 1);
    control.brake = clamp(-effort, 0, options.max_speed_hold_brake);
end

function options = radiusDefaults(options)
    options = setDefault(options, "speed_grid_mps", (6:2:22).');
    options = setDefault(options, "settle_time_s", 4);
    options = setDefault(options, "analysis_window_s", 0.5);
    options = setDefault(options, "max_step_s", 0.02);
    options = setDefault(options, "relative_tolerance", 1e-6);
    options = setDefault(options, "absolute_tolerance", 1e-8);
    options = setDefault(options, "speed_controller_gain_per_mps", 0.3);
    options = setDefault(options, "yaw_rate_controller_gain_s", 0.5);
    options = setDefault(options, "max_steer_rad", 0.35);
    options = setDefault(options, "max_speed_hold_brake", 0.3);
    options = setDefault(options, "speed_tolerance_mps", 0.5);
    options = setDefault(options, "curvature_tolerance_frac", 0.15);
    options = setDefault(options, "yaw_accel_tolerance_radps2", 0.05);
    options = setDefault(options, "vy_dot_tolerance_mps2", 0.15);
    options = setDefault(options, "comparison_threshold_frac", 0.25);
    options = setDefault(options, "emit_warning", true);
    options = setDefault(options, "bracket_speed_tolerance_mps", 0.5);
    options = setDefault(options, "bracket_max_iterations", 8);
    numericValues = [options.settle_time_s, options.analysis_window_s, ...
        options.max_step_s, options.relative_tolerance, ...
        options.absolute_tolerance, options.speed_controller_gain_per_mps, ...
        options.yaw_rate_controller_gain_s, options.max_steer_rad, ...
        options.max_speed_hold_brake, options.speed_tolerance_mps, ...
        options.curvature_tolerance_frac, ...
        options.yaw_accel_tolerance_radps2, ...
        options.vy_dot_tolerance_mps2, options.comparison_threshold_frac, ...
        options.bracket_speed_tolerance_mps, options.bracket_max_iterations];
    invalid = ~all(isfinite(numericValues)) || ~all(isreal(numericValues)) ...
        || options.settle_time_s <= 0 || options.analysis_window_s <= 0 ...
        || options.analysis_window_s > options.settle_time_s ...
        || options.max_step_s <= 0 || options.relative_tolerance <= 0 ...
        || options.absolute_tolerance <= 0 ...
        || options.speed_controller_gain_per_mps < 0 ...
        || options.yaw_rate_controller_gain_s < 0 ...
        || options.max_steer_rad <= 0 ...
        || options.max_speed_hold_brake < 0 ...
        || options.max_speed_hold_brake > 1 ...
        || options.speed_tolerance_mps < 0 ...
        || options.curvature_tolerance_frac < 0 ...
        || options.yaw_accel_tolerance_radps2 < 0 ...
        || options.vy_dot_tolerance_mps2 < 0 ...
        || options.comparison_threshold_frac < 0 ...
        || options.bracket_speed_tolerance_mps <= 0 ...
        || options.bracket_max_iterations < 1 ...
        || fix(options.bracket_max_iterations) ~= options.bracket_max_iterations;
    if invalid
        error("QSSLTS:DOF7RadiusOptions", ...
            "Constant-radius solver options are outside valid ranges.");
    end
end

function comparison = compareLateral( ...
        options, speed_mps, measuredAy_mps2, turnSign, ...
        hasStableCase, limitBracketed, limitRefined, ...
        vehicle, tire, powertrain, brake)
    comparison.available = false;
    comparison.relative_difference = NaN;
    comparison.ggv_value_mps2 = NaN;
    if ~hasStableCase
        comparison.reason = "no_stable_case";
        return
    end
    if ~limitBracketed
        comparison.reason = "limit_not_bracketed";
        return
    end
    if ~limitRefined
        comparison.reason = "limit_not_refined";
        return
    end
    if ~isfinite(speed_mps)
        error("QSSLTS:DOF7RadiusBracket", ...
            "Refined radius bracket did not produce a finite estimate.");
    end
    if ~isfield(options, "ggv") || isempty(fieldnames(options.ggv))
        comparison.reason = "ggv_not_provided";
        return
    end
    validate_7dof_comparison_ggv( ...
        options.ggv, vehicle, tire, powertrain, brake);
    if speed_mps < options.ggv.v_mps(1) || speed_mps > options.ggv.v_mps(end)
        error("QSSLTS:DOF7GGVRange", ...
            "7DOF comparison speed is outside the GGV domain.");
    end
    if turnSign > 0
        limit_g = interp1(options.ggv.v_mps, ...
            options.ggv.ay_limit_pos_g, speed_mps, "linear");
    else
        limit_g = abs(interp1(options.ggv.v_mps, ...
            options.ggv.ay_limit_neg_g, speed_mps, "linear"));
    end
    reference_mps2 = limit_g * options.ggv.gravity_mps2;
    relativeDifference = abs(measuredAy_mps2 - reference_mps2) ...
        / max(abs(reference_mps2), eps);
    comparison.available = true;
    comparison.reason = "ok";
    comparison.relative_difference = relativeDifference;
    comparison.ggv_value_mps2 = reference_mps2;
    if options.emit_warning ...
            && relativeDifference > options.comparison_threshold_frac
        warning("QSSLTS:DOF7Mismatch", ...
            "7DOF constant-radius result differs from aero-off GGV by %.1f%%.", ...
            100 * relativeDifference);
    end
end

function output = setDefault(input, fieldName, value)
    output = input;
    if ~isfield(output, fieldName), output.(fieldName) = value; end
end
