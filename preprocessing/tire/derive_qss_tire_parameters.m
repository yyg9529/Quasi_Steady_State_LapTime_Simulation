function tire = derive_qss_tire_parameters(parsed)
%DERIVE_QSS_TIRE_PARAMETERS Reduce PAC2002 scalars to the QSS envelope.

arguments
    parsed (1, 1) struct
end

if ~isfield(parsed, "sections") || ~isstruct(parsed.sections)
    error("derive_qss_tire_parameters:MissingSections", ...
        "Input must contain parsed.sections from read_pac2002_tir.");
end
if ~isfield(parsed, "source") || ~isstruct(parsed.source) ...
        || ~isfield(parsed.source, "sha256")
    error("derive_qss_tire_parameters:MissingSourceHash", ...
        "Input must contain parsed.source.sha256.");
end

FNOMIN = coefficient(parsed.sections, "VERTICAL", "FNOMIN");
LFZO = coefficient(parsed.sections, "SCALING_COEFFICIENTS", "LFZO");
LMUX = coefficient(parsed.sections, "SCALING_COEFFICIENTS", "LMUX");
LMUY = coefficient(parsed.sections, "SCALING_COEFFICIENTS", "LMUY");
PDX1 = coefficient(parsed.sections, "LONGITUDINAL_COEFFICIENTS", "PDX1");
PDX2 = coefficient(parsed.sections, "LONGITUDINAL_COEFFICIENTS", "PDX2");
PDY1 = coefficient(parsed.sections, "LATERAL_COEFFICIENTS", "PDY1");
PDY2 = coefficient(parsed.sections, "LATERAL_COEFFICIENTS", "PDY2");

if PDX1 == 0 || PDY1 == 0
    error("derive_qss_tire_parameters:ZeroPeakCoefficient", ...
        "PDX1 and PDY1 must be nonzero for load-sensitivity mapping.");
end

tire.model_type = "load_sensitive";
tire.Fz_ref_N = FNOMIN * LFZO;
tire.mu_x_ref = PDX1 * LMUX;
tire.mu_y_ref = PDY1 * LMUY;
tire.load_sensitivity_x = PDX2 / PDX1;
tire.load_sensitivity_y = PDY2 / PDY1;
tire.combined_n = 2;
tire.provenance.source_tir_sha256 = upper(string(parsed.source.sha256));
tire.provenance.reduction = "zero-camber pure-slip peak mapping";
tire.provenance.combined_n_basis = ...
    "QSS p-norm model-reduction assumption; " + ...
    "not directly extracted from TIR";
end

function value = coefficient(sections, sectionName, fieldName)
if ~isfield(sections, sectionName) ...
        || ~isfield(sections.(sectionName), fieldName)
    error("derive_qss_tire_parameters:MissingCoefficient", ...
        "Required coefficient %s.%s is missing.", ...
        sectionName, fieldName);
end
value = sections.(sectionName).(fieldName);
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("derive_qss_tire_parameters:InvalidCoefficient", ...
        "Coefficient %s.%s must be a finite numeric scalar.", ...
        sectionName, fieldName);
end
end
