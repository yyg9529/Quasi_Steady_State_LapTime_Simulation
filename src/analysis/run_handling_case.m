function handling = run_handling_case(config, handlingConfig)
%RUN_HANDLING_CASE Run PAC2002 four-wheel YMD and understeer studies.

arguments
    config (1,1) struct
    handlingConfig (1,1) struct
end

if isfield(handlingConfig, "enabled") ...
        && ~logical(handlingConfig.enabled)
    handling = struct( ...
        "available", false, ...
        "message", optionalText(handlingConfig, "message", ...
            "PAC2002 handling analysis is unavailable."));
    return
end
hasTireModel = isfield(handlingConfig, "tire_model") ...
    && isstruct(handlingConfig.tire_model) ...
    && isscalar(handlingConfig.tire_model);
if ~hasTireModel && (~isfield(handlingConfig, "tir_file") ...
        || ~isfile(handlingConfig.tir_file))
    error("QSSLTS:HandlingTirMissing", ...
        "PAC2002 TIR file is required for YMD and understeer: %s", ...
        optionalText(handlingConfig, "tir_file", "<not configured>"));
end
if ~isfield(config, "vehicle") || ~isfield(config, "models")
    error("QSSLTS:HandlingConfig", ...
        "Handling analysis requires config.vehicle and config.models.");
end

if hasTireModel
    tireModel = handlingConfig.tire_model;
else
    tireModel = load_pac2002_tire(handlingConfig.tir_file);
end
aero = optionalStruct(config.models, "aero", struct("enabled", false));
ymdStudy = optionalStruct(handlingConfig, "ymd_study", ...
    defaultYmdStudy());
understeerStudy = optionalStruct(handlingConfig, ...
    "understeer_study", defaultUndersteerStudy());

handling.available = true;
handling.ymd = generate_ymd( ...
    config.vehicle, tireModel, aero, ymdStudy);
handling.understeer = calc_understeer_gradient( ...
    config.vehicle, tireModel, aero, understeerStudy);
handling.provenance.tir_file = optionalText( ...
    handlingConfig, "tir_file", "<preloaded>");
handling.provenance.tir_sha256 = tireModel.source.sha256;
handling.provenance.tire_evaluator = tireModel.evaluator_name;
handling.provenance.mfeval_version = tireModel.mfeval.version;
handling.provenance.mfeval_parameter_file = ...
    tireModel.source.mfeval_parameter_file;
handling.provenance.vehicle = config.vehicle;
handling.provenance.ymd_study = ymdStudy;
handling.provenance.understeer_study = understeerStudy;
end

function study = defaultYmdStudy()
study.speed_mps = 12;
study.beta_rad = deg2rad((-3:1:3).');
study.steer_rad = deg2rad(-6:1.5:6);
end

function study = defaultUndersteerStudy()
study.radius_m = 30;
study.speed_mps = (5:2:15).';
study.linear_fit_range_g = [0 0.9];
end

function value = optionalStruct(source, name, defaultValue)
if isfield(source, name)
    value = source.(name);
else
    value = defaultValue;
end
end

function value = optionalText(source, name, defaultValue)
if isfield(source, name)
    value = string(source.(name));
else
    value = string(defaultValue);
end
end
