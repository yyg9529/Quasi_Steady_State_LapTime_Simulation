function tire = load_qss_tire_envelope(filePath)
%LOAD_QSS_TIRE_ENVELOPE Load a numeric offline-reduced QSS tire artifact.

arguments
    filePath (1, 1) string
end

if ~isfile(filePath)
    error("QSSLTS:QssEnvelopeFileNotFound", ...
        "QSS tire envelope does not exist: %s", filePath);
end

saved = load(filePath, "qss_envelope");
if ~isfield(saved, "qss_envelope")
    invalid(filePath, "variable qss_envelope is missing");
end
artifact = saved.qss_envelope;
requiredArtifact = ["schema_version", "source", "tire", ...
    "pure_boundary", "combined_boundary", "fit", "extraction"];
if ~isstruct(artifact) || ...
        ~all(isfield(artifact, cellstr(requiredArtifact))) ...
        || ~isequal(artifact.schema_version, 1)
    invalid(filePath, "schema version 1 fields are incomplete");
end
if ~isfield(artifact.source, "sha256") || ...
        isempty(regexp(char(artifact.source.sha256), ...
        '^[0-9A-F]{64}$', 'once'))
    invalid(filePath, "source SHA-256 is missing or malformed");
end

tire = artifact.tire;
requiredTire = ["model_type", "Fz_ref_N", "mu_x_ref", ...
    "mu_y_ref", "load_sensitivity_x", "load_sensitivity_y", ...
    "combined_n", "provenance"];
if ~isstruct(tire) || ~all(isfield(tire, cellstr(requiredTire)))
    invalid(filePath, "reduced tire fields are incomplete");
end
if string(tire.model_type) ~= "load_sensitive"
    invalid(filePath, "only load_sensitive QSS envelopes are supported");
end
numericFields = requiredTire(2:7);
for fieldName = numericFields
    value = tire.(fieldName);
    if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
        invalid(filePath, "reduced tire numeric fields must be finite scalars");
    end
end
if tire.Fz_ref_N <= 0 || tire.mu_x_ref <= 0 || tire.mu_y_ref <= 0 ...
        || tire.combined_n < 1
    invalid(filePath, "reduced tire magnitudes are outside valid limits");
end
if ~isfield(tire.provenance, "source_tir_sha256") || ...
        upper(string(tire.provenance.source_tir_sha256)) ~= ...
        upper(string(artifact.source.sha256))
    invalid(filePath, "tire provenance does not match the artifact source");
end
if containsFunctionHandle(tire)
    invalid(filePath, "runtime QSS tire must not contain function handles");
end
end

function result = containsFunctionHandle(value)
if isa(value, "function_handle")
    result = true;
    return
end
if isstruct(value)
    names = fieldnames(value);
    result = any(cellfun(@(name) containsFunctionHandle(value.(name)), ...
        names));
    return
end
if iscell(value)
    result = any(cellfun(@containsFunctionHandle, value));
    return
end
result = false;
end

function invalid(filePath, reason)
error("QSSLTS:InvalidQssEnvelope", ...
    "Invalid QSS tire envelope %s: %s.", filePath, reason);
end
