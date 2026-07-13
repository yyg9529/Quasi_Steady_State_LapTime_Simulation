function result = run_7dof_accel_event( ...
        vehicle, tire, powertrain, brake, options)
%RUN_7DOF_ACCEL_EVENT Simulate full-throttle straight-line acceleration.

    arguments
        vehicle (1, 1) struct
        tire (1, 1) struct
        powertrain (1, 1) struct
        brake (1, 1) struct
        options (1, 1) struct = struct()
    end

    options = accelDefaults(options);
    if options.target_speed_mps <= options.initial_speed_mps
        error("QSSLTS:DOF7Event", ...
            "Acceleration target speed must exceed the initial speed.");
    end

    control = struct("steer_rad", 0, ...
        "throttle", options.throttle, "brake", 0);
    x0 = make_7dof_initial_state(options.initial_speed_mps, tire);
    odeFunction = @(time_s, state) vehicle_7dof_ode( ...
        time_s, state, control, vehicle, tire, powertrain, brake);
    eventFunction = @(time_s, state) targetSpeedEvent( ...
        time_s, state, options.target_speed_mps, 1);
    odeOptions = odeset("RelTol", options.relative_tolerance, ...
        "AbsTol", options.absolute_tolerance, ...
        "MaxStep", options.max_step_s, "Events", eventFunction);
    [t_s, x, eventTime_s, eventState, eventIndex] = ode15s( ...
        odeFunction, [0, options.max_time_s], x0, odeOptions);
    trace = evaluate_7dof_trajectory( ...
        t_s, x, control, vehicle, tire, powertrain, brake);

    targetReached = ~isempty(eventIndex);
    result.t_s = t_s;
    result.x = x;
    result.trace = trace;
    result.target_reached = targetReached;
    result.termination_reason = eventReason(targetReached, "target_speed");
    result.event_time_s = scalarOrNaN(eventTime_s);
    result.event_state = rowOrNaN(eventState);
    result.final_speed_mps = x(end, 1);
    result.peak_accel_mps2 = max(trace.ax_body_mps2);
    result.terminal_accel_mps2 = trace.ax_body_mps2(end);
    result.distance_m = trapz(t_s, max(x(:, 1), 0));
    result.comparison = compareLongitudinal( ...
        options, result.final_speed_mps, result.terminal_accel_mps2, ...
        "accel", targetReached, vehicle, tire, powertrain, brake);
end

function options = accelDefaults(options)
    options = setDefault(options, "initial_speed_mps", 0);
    options = setDefault(options, "target_speed_mps", 20);
    options = setDefault(options, "max_time_s", 10);
    options = setDefault(options, "max_step_s", 0.02);
    options = setDefault(options, "relative_tolerance", 1e-6);
    options = setDefault(options, "absolute_tolerance", 1e-8);
    options = setDefault(options, "throttle", 1);
    options = setDefault(options, "comparison_threshold_frac", 0.20);
    options = setDefault(options, "emit_warning", true);
end

function [value, isTerminal, direction] = targetSpeedEvent(~, state, target, sense)
    value = state(1) - target;
    isTerminal = 1;
    direction = sense;
end

function comparison = compareLongitudinal( ...
        options, speed_mps, measured, branch, eventReached, ...
        vehicle, tire, powertrain, brake)
    comparison.available = false;
    comparison.relative_difference = NaN;
    comparison.ggv_value_mps2 = NaN;
    if ~eventReached
        comparison.reason = "event_not_reached";
        return
    end
    if ~isfield(options, "ggv") || isempty(fieldnames(options.ggv))
        comparison.reason = "ggv_not_provided";
        return
    end
    validate_7dof_comparison_ggv( ...
        options.ggv, vehicle, tire, powertrain, brake);
    cap = interp_ggv(options.ggv, speed_mps, 0, ...
        struct("speed_query_policy", "error"));
    if ~cap.is_feasible
        error("QSSLTS:DOF7GGVRange", ...
            "7DOF comparison speed is outside the GGV domain.");
    end
    if branch == "accel"
        reference = cap.ax_max_mps2;
    else
        reference = cap.ax_min_mps2;
    end
    relativeDifference = abs(measured - reference) / max(abs(reference), eps);
    comparison.available = true;
    comparison.reason = "ok";
    comparison.relative_difference = relativeDifference;
    comparison.ggv_value_mps2 = reference;
    if options.emit_warning ...
            && relativeDifference > options.comparison_threshold_frac
        warning("QSSLTS:DOF7Mismatch", ...
            "7DOF %s differs from aero-off GGV by %.1f%%.", ...
            branch, 100 * relativeDifference);
    end
end

function value = scalarOrNaN(input)
    if isempty(input), value = NaN; else, value = input(end); end
end

function value = rowOrNaN(input)
    if isempty(input), value = nan(1, 7); else, value = input(end, :); end
end

function reason = eventReason(eventOccurred, label)
    if eventOccurred, reason = string(label); else, reason = "timeout"; end
end

function output = setDefault(input, fieldName, value)
    output = input;
    if ~isfield(output, fieldName), output.(fieldName) = value; end
end
