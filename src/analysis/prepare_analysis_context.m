function context = prepare_analysis_context(baseConfig)
%PREPARE_ANALYSIS_CONTEXT Freeze baseline calibration for sweep/DOE cases.

arguments
    baseConfig (1,1) struct
end

required = ["track", "vehicle", "models", "options"];
if ~all(isfield(baseConfig, cellstr(required)))
    error("QSSLTS:AnalysisConfig", ...
        "base_config requires track, vehicle, models, and options.");
end
if ~isfield(baseConfig.models, "tire")
    error("QSSLTS:AnalysisConfig", "base_config.models.tire is required.");
end

cleanConfig = baseConfig;
cleanConfig.models = removeField(cleanConfig.models, "ggv");
scaleTable = table();
calibrationMode = "theory_only";

if isfield(cleanConfig.models, "ggv_real") ...
        && ~isempty(cleanConfig.models.ggv_real)
    theoretical = generateConfigGgv(cleanConfig);
    calibrationOptions = default_qss_options(cleanConfig.options);
    calibrationOptions.calibration_report_file = "";
    [~, calibrationReport] = calibrate_ggv(theoretical, ...
        cleanConfig.models.ggv_real, calibrationOptions);
    scaleTable = calibrationReport.scale_table(:, ...
        ["v_mps", "scale_lat", "scale_acc", "scale_brake"]);
    calibrationMode = "frozen_baseline";
end
cleanConfig.models = removeField(cleanConfig.models, "ggv_real");

context.base_config = cleanConfig;
context.scale_table = scaleTable;
context.calibration_mode = calibrationMode;
end

function ggv = generateConfigGgv(config)
ggv = generate_model_ggv(config.vehicle, config.models.tire, ...
    optionalModel(config.models, "aero"), ...
    optionalModel(config.models, "powertrain"), ...
    optionalModel(config.models, "brake"), config.options);
end

function model = optionalModel(models, name)
if isfield(models, name)
    model = models.(name);
else
    model = struct();
end
end

function value = removeField(value, name)
if isfield(value, name)
    value = rmfield(value, name);
end
end
