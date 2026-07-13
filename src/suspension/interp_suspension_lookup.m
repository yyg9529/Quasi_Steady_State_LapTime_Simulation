function state = interp_suspension_lookup(susp, jounce_mm, options)
%INTERP_SUSPENSION_LOOKUP Evaluate a lookup or constant suspension model.

    arguments
        susp (1, 1) struct
        jounce_mm (1, 1) double {mustBeFinite, mustBeReal}
        options.OutOfRange (1, 1) string ...
            {mustBeMember(options.OutOfRange, ["error", "clamp"])} = "error"
    end

    if ~isfield(susp, "model_type")
        error("QSSLTS:SuspensionModel", ...
            "Suspension model must define model_type.");
    end

    switch string(susp.model_type)
        case "constant"
            state = constantState(susp, jounce_mm);
        case "lookup"
            state = lookupState(susp, jounce_mm, options.OutOfRange);
        otherwise
            error("QSSLTS:SuspensionModel", ...
                "Unsupported suspension model_type: %s", susp.model_type);
    end
end

function state = lookupState(susp, requestedJounce_mm, outOfRange)
    lowerLimit = susp.jounce_mm(1);
    upperLimit = susp.jounce_mm(end);
    outside = requestedJounce_mm < lowerLimit || requestedJounce_mm > upperLimit;

    if outside && outOfRange == "error"
        error("QSSLTS:SuspensionRange", ...
            "Requested jounce %.6g mm is outside [%.6g, %.6g] mm.", ...
            requestedJounce_mm, lowerLimit, upperLimit);
    end

    queryJounce_mm = clamp(requestedJounce_mm, lowerLimit, upperLimit);
    state.requested_jounce_mm = requestedJounce_mm;
    state.jounce_mm = queryJounce_mm;
    state.was_clamped = outside;
    state.camber_rad = interpolate(susp, "camber_rad", queryJounce_mm);
    state.toe_rad = interpolate(susp, "toe_rad", queryJounce_mm);
    state.motion_ratio = interpolate(susp, "motion_ratio", queryJounce_mm);
    state.damper_stroke_mm = interpolate( ...
        susp, "damper_stroke_mm", queryJounce_mm);
    state.caster_rad = interpolate(susp, "caster_rad", queryJounce_mm);
    state.kpi_rad = interpolate(susp, "kpi_rad", queryJounce_mm);
    state.scrub_radius_mm = interpolate( ...
        susp, "scrub_radius_mm", queryJounce_mm);
    state.caster_trail_mm = interpolate( ...
        susp, "caster_trail_mm", queryJounce_mm);
    state.motion_ratio_available = isfinite(state.motion_ratio);
    state.wheel_rate_usage_confirmed = susp.wheel_rate_usage_confirmed;
end

function value = interpolate(susp, fieldName, queryJounce_mm)
    value = interp1(susp.jounce_mm, susp.(fieldName), ...
        queryJounce_mm, "linear");
end

function state = constantState(susp, requestedJounce_mm)
    state.requested_jounce_mm = requestedJounce_mm;
    state.jounce_mm = requestedJounce_mm;
    state.was_clamped = false;
    state.camber_rad = susp.camber_rad;
    state.toe_rad = susp.toe_rad;
    state.motion_ratio = NaN;
    state.damper_stroke_mm = 0;
    state.caster_rad = NaN;
    state.kpi_rad = NaN;
    state.scrub_radius_mm = NaN;
    state.caster_trail_mm = NaN;
    state.motion_ratio_available = false;
    state.wheel_rate_usage_confirmed = false;
end
