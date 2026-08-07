function [result, handling] = run_analysis_case( ...
        config, scaleTable, handlingConfig)
%RUN_ANALYSIS_CASE Run one theoretical or frozen-calibration analysis case.

arguments
    config (1,1) struct
    scaleTable table = table()
    handlingConfig (1,1) struct = struct()
end

models = config.models;
models = removeField(models, "ggv");
models = removeField(models, "ggv_real");
if isempty(scaleTable)
    result = run_qss_lap( ...
        config.track, config.vehicle, models, config.options);
else
    ggv = generate_model_ggv(config.vehicle, models.tire, ...
        optionalModel(models, "aero"), optionalModel(models, "powertrain"), ...
        optionalModel(models, "brake"), config.options);
    ggv = apply_ggv_calibration_scales(ggv, scaleTable);
    models.ggv = ggv;
    result = run_qss_lap(config.track, config.vehicle, models, config.options);
end
if isfield(config, "event") && ~isempty(fieldnames(config.event))
    result.event = score_fsc_dynamic_event(result, config.event);
end
if isempty(fieldnames(handlingConfig))
    handling = struct();
else
    try
        handling = run_handling_case(config, handlingConfig);
    catch exception
        if handlingFailurePolicy(handlingConfig) ~= "report_unavailable"
            rethrow(exception)
        end
        handling = struct( ...
            "available", false, ...
            "message", "YMD/Understeer 未评估：" ...
                + string(exception.message), ...
            "error_id", string(exception.identifier), ...
            "error_message", string(exception.message));
    end
end
end

function policy = handlingFailurePolicy(handlingConfig)
if isfield(handlingConfig, "failure_policy")
    policy = string(handlingConfig.failure_policy);
else
    policy = "error";
end
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
