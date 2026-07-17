function tire = tire_hoosier_16x75_10_r20_qss(rollingRadius)
%TIRE_HOOSIER_16X75_10_R20_QSS Return the offline-reduced QSS tire preset.

arguments
    rollingRadius (1, 1) struct
end

requiredFields = ["value_m", "load_N", "pressure_kPa", ...
    "speed_mps", "source"];
missingFields = requiredFields(~isfield(rollingRadius, requiredFields));
if ~isempty(missingFields)
    error("tire_hoosier_16x75_10_r20_qss:MissingRollingRadiusField", ...
        "rollingRadius.%s is required.", missingFields(1));
end

positiveFields = ["value_m", "load_N", "pressure_kPa"];
for fieldIndex = 1:numel(positiveFields)
    name = positiveFields(fieldIndex);
    value = rollingRadius.(name);
    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value <= 0
        error("tire_hoosier_16x75_10_r20_qss:InvalidRollingRadius", ...
            "rollingRadius.%s must be a positive finite numeric scalar.", ...
            name);
    end
end

speed = rollingRadius.speed_mps;
if ~isnumeric(speed) || ~isscalar(speed) || ~isfinite(speed) || speed < 0
    error("tire_hoosier_16x75_10_r20_qss:InvalidRollingRadius", ...
        "rollingRadius.speed_mps must be a nonnegative finite numeric scalar.");
end
if ~isTextScalar(rollingRadius.source) ...
        || strlength(strtrim(string(rollingRadius.source))) == 0
    error("tire_hoosier_16x75_10_r20_qss:InvalidRollingRadius", ...
        "rollingRadius.source must be nonempty scalar text.");
end
source = lower(strtrim(string(rollingRadius.source)));
allowedSources = ["loaded rolling-circumference measurement", ...
    "validated pac2002 effective rolling radius", ...
    "validated tire-test fit"];
if ~any(startsWith(source, allowedSources))
    error("tire_hoosier_16x75_10_r20_qss:UnvalidatedRollingRadius", ...
        "rollingRadius.source must identify a loaded measurement, " + ...
        "validated PAC2002 effective radius, or validated tire-test fit.");
end

tire.model_type = "load_sensitive";
tire.Fz_ref_N = 667;
tire.mu_x_ref = 1.334655;
tire.mu_y_ref = 1.804320;
tire.load_sensitivity_x = -0.0168396327140722;
tire.load_sensitivity_y = -0.0693256185155627;
tire.combined_n = 2;
tire.rolling_radius_m = rollingRadius.value_m;
tire.provenance.source_tir_sha256 = ...
    "6AB8AA1219B7910A1660966AAD1B6C8FC7EF2FDB46F40BF0FA9B6FC172E53A2A";
tire.provenance.reduction = "offline PAC2002 zero-camber pure-slip peak mapping";
tire.provenance.combined_n_basis = ...
    "QSS p-norm model-reduction assumption; " + ...
    "not directly extracted from TIR";
tire.provenance.rolling_radius = rollingRadius;
end

function tf = isTextScalar(value)
tf = (isstring(value) && isscalar(value) && ~ismissing(value)) || ...
    (ischar(value) && isrow(value));
end
