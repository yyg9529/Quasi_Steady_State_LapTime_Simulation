function susp = read_simscape_suspension_lookup(filename)
%READ_SIMSCAPE_SUSPENSION_LOOKUP Read an offline suspension-geometry export.
%   The input remains a preprocessing artifact. No Simscape model is loaded.

    arguments
        filename (1, 1) string {mustBeFile}
    end

    raw = readtable(filename, VariableNamingRule="preserve");
    required = ["jounce_mm", "damperStroke_mm", "motionRatio_local", ...
        "toe_deg", "camber_deg", "caster_deg", "kpi_deg", ...
        "scrubRadiusRaw_mm", "casterTrailRaw_mm", "statusFlag"];
    available = string(raw.Properties.VariableNames);
    missing = required(~ismember(required, available));
    if ~isempty(missing)
        error("QSSLTS:SuspensionSchema", ...
            "Suspension lookup is missing columns: %s", ...
            strjoin(missing, ", "));
    end

    valid = raw.statusFlag == 1;
    data = raw(valid, required);
    if height(data) < 2
        error("QSSLTS:SuspensionData", ...
            "Suspension lookup needs at least two rows with statusFlag == 1.");
    end

    numericData = table2array(data);
    if ~isnumeric(numericData) || ~all(isfinite(numericData), "all")
        error("QSSLTS:SuspensionData", ...
            "All values in valid suspension rows must be finite numeric data.");
    end

    data = sortrows(data, "jounce_mm");
    if any(diff(data.jounce_mm) <= 0)
        error("QSSLTS:SuspensionData", ...
            "Valid jounce_mm breakpoints must be unique and strictly increasing.");
    end

    susp.model_type = "lookup";
    susp.source_file = filename;
    susp.jounce_mm = data.jounce_mm;
    susp.damper_stroke_mm = data.damperStroke_mm;
    susp.motion_ratio = data.motionRatio_local;
    susp.toe_rad = deg2rad(data.toe_deg);
    susp.camber_rad = deg2rad(data.camber_deg);
    susp.caster_rad = deg2rad(data.caster_deg);
    susp.kpi_rad = deg2rad(data.kpi_deg);
    susp.scrub_radius_mm = data.scrubRadiusRaw_mm;
    susp.caster_trail_mm = data.casterTrailRaw_mm;
    susp.motion_ratio_definition = ...
        "d(damper_length)/d(wheel_jounce)";
    susp.motion_ratio_sign_convention = ...
        "positive when damper length increases with positive wheel jounce";
    susp.wheel_rate_usage_confirmed = false;
end
