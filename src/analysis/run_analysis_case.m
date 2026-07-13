function result = run_analysis_case(config, scaleTable)
%RUN_ANALYSIS_CASE Regenerate theoretical GGV and run one analysis case.

arguments
    config (1,1) struct
    scaleTable table = table()
end

models = config.models;
models = removeField(models, "ggv");
models = removeField(models, "ggv_real");
ggv = generate_model_ggv(config.vehicle, models.tire, ...
    optionalModel(models, "aero"), optionalModel(models, "powertrain"), ...
    optionalModel(models, "brake"), config.options);
if ~isempty(scaleTable)
    ggv = apply_ggv_calibration_scales(ggv, scaleTable);
end
models.ggv = ggv;
result = run_qss_lap(config.track, config.vehicle, models, config.options);
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
