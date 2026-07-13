function result = run_7dof_brake_event( ...
        vehicle, tire, powertrain, brake, options)
%RUN_7DOF_BRAKE_EVENT Simulate straight-line braking to a positive speed.

    arguments
        vehicle (1, 1) struct
        tire (1, 1) struct
        powertrain (1, 1) struct
        brake (1, 1) struct
        options (1, 1) struct = struct()
    end

    options = brakeDefaults(options);
    if options.stop_speed_mps <= 0 ...
            || options.initial_speed_mps <= options.stop_speed_mps
        error("QSSLTS:DOF7Event", ...
            "Brake event requires initial_speed > stop_speed > 0.");
    end

    control = struct("steer_rad", 0, ...
        "throttle", 0, "brake", options.brake_command);
    x0 = make_7dof_initial_state(options.initial_speed_mps, tire);
    odeFunction = @(time_s, state) vehicle_7dof_ode( ...
        time_s, state, control, vehicle, tire, powertrain, brake);
    eventFunction = @(time_s, state) stopSpeedEvent( ...
        time_s, state, options.stop_speed_mps);
    odeOptions = odeset("RelTol", options.relative_tolerance, ...
        "AbsTol", options.absolute_tolerance, ...
        "MaxStep", options.max_step_s, "Events", eventFunction);
    [t_s, x, eventTime_s, eventState, eventIndex] = ode15s( ...
        odeFunction, [0, options.max_time_s], x0, odeOptions);
    trace = evaluate_7dof_trajectory( ...
        t_s, x, control, vehicle, tire, powertrain, brake);
    representativeDecel_mps2 = representativeDeceleration( ...
        x(:, 1), trace.ax_body_mps2, options);

    stopReached = ~isempty(eventIndex);
    result.t_s = t_s;
    result.x = x;
    result.trace = trace;
    result.stop_reached = stopReached;
    result.termination_reason = eventReason(stopReached, "stop_speed");
    result.event_time_s = scalarOrNaN(eventTime_s);
    result.event_state = rowOrNaN(eventState);
    result.final_speed_mps = x(end, 1);
    result.peak_decel_mps2 = max(-trace.ax_body_mps2);
    result.representative_decel_mps2 = representativeDecel_mps2;
    result.distance_m = trapz(t_s, max(x(:, 1), 0));
    comparisonOptions = options;
    comparisonSpeed_mps = 0.5 * ( ...
        options.initial_speed_mps + options.stop_speed_mps);
    result.comparison = compareBraking(comparisonOptions, ...
        comparisonSpeed_mps, -representativeDecel_mps2, stopReached, ...
        vehicle, tire, powertrain, brake);
end

function options = brakeDefaults(options)
    options = setDefault(options, "initial_speed_mps", 20);
    options = setDefault(options, "stop_speed_mps", 1);
    options = setDefault(options, "max_time_s", 10);
    options = setDefault(options, "max_step_s", 0.02);
    options = setDefault(options, "relative_tolerance", 1e-6);
    options = setDefault(options, "absolute_tolerance", 1e-8);
    options = setDefault(options, "brake_command", 1);
    options = setDefault(options, "comparison_threshold_frac", 0.20);
    options = setDefault(options, "emit_warning", true);
end

function [value, isTerminal, direction] = stopSpeedEvent(~, state, target)
    value = state(1) - target;
    isTerminal = 1;
    direction = -1;
end

function decel_mps2 = representativeDeceleration(speed_mps, ax_mps2, options)
    upper = options.stop_speed_mps + 0.8 * ...
        (options.initial_speed_mps - options.stop_speed_mps);
    lower = options.stop_speed_mps + 0.2 * ...
        (options.initial_speed_mps - options.stop_speed_mps);
    selected = speed_mps >= lower & speed_mps <= upper;
    decel_mps2 = median(-ax_mps2(selected));
end

function comparison = compareBraking( ...
        options, speed_mps, measuredAx_mps2, eventReached, ...
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
    reference = cap.ax_min_mps2;
    relativeDifference = abs(measuredAx_mps2 - reference) ...
        / max(abs(reference), eps);
    comparison.available = true;
    comparison.reason = "ok";
    comparison.relative_difference = relativeDifference;
    comparison.ggv_value_mps2 = reference;
    if options.emit_warning ...
            && relativeDifference > options.comparison_threshold_frac
        warning("QSSLTS:DOF7Mismatch", ...
            "7DOF braking differs from aero-off GGV by %.1f%%.", ...
            100 * relativeDifference);
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
