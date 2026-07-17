function config = apply_analysis_parameter(config, parameter, value)
%APPLY_ANALYSIS_PARAMETER Apply a supported DOE/sensitivity parameter.

arguments
    config (1,1) struct
    parameter (1,1) string
    value
end

parameterLower = lower(parameter);
if parameterLower == "rules" ...
        || parameterLower == "models.powertrain" ...
        || parameterLower == "models.powertrain.rules" ...
        || startsWith(parameterLower, "models.powertrain.rules.")
    error("QSSLTS:AnalysisRulesImmutable", ...
        "DOE and sensitivity studies cannot modify powertrain rules.");
end
if isLegacyPowertrainPath(parameterLower)
    error("QSSLTS:AnalysisParameter", ...
        "Legacy flat powertrain parameter is unsupported: %s", parameter);
end

if contains(parameter, ".")
    config = set_nested_field(config, parameter, value);
    config = validateCompositePowertrain(config);
    return
end

switch parameterLower
    case {"mass", "mass_kg"}
        config.vehicle.mass.total_kg = value;
    case {"cgh", "cg_height_m"}
        config.vehicle.mass.cg_height_m = value;
    case {"track_front", "track_front_m"}
        config.vehicle.geometry.track_front_m = value;
    case {"track_rear", "track_rear_m"}
        config.vehicle.geometry.track_rear_m = value;
    case {"cla", "cla_m2"}
        config.models.aero.CLA_m2 = value;
    case {"cda", "cda_m2"}
        config.models.aero.CDA_m2 = value;
    case "front_downforce_frac"
        config.models.aero.front_downforce_frac = value;
    case "inverter_power_w"
        config.models.powertrain.inverter.P_dc_peak_W = value;
    case "gear_ratio"
        config = applyGearRatio(config, value);
    case {"brake_bias", "front_bias"}
        config.models.brake.front_bias = value;
    case "tire_mu_scale"
        config.models.tire = scaleTireMu(config.models.tire, value);
    otherwise
        error("QSSLTS:AnalysisParameter", ...
            "Unsupported analysis parameter: %s", parameter);
end
config = validateCompositePowertrain(config);
end

function config = applyGearRatio(config, newRatio)
if ~isscalar(newRatio) || ~isfinite(newRatio) || newRatio <= 0
    error("QSSLTS:GearRatio", "Gear ratio must be positive and finite.");
end
config.models.powertrain.gear_ratio = newRatio;
end

function config = validateCompositePowertrain(config)
if ~isfield(config, "models") || ~isstruct(config.models) ...
        || ~isscalar(config.models) ...
        || ~isfield(config.models, "powertrain") ...
        || ~isstruct(config.models.powertrain) ...
        || ~isscalar(config.models.powertrain) ...
        || ~isCompositePowertrain(config.models.powertrain)
    return
end
config.models.powertrain = ...
    validate_powertrain_config(config.models.powertrain);
end

function result = isCompositePowertrain(powertrain)
markers = ["motor_count", "gear_ratio", "drivetrain_efficiency", ...
    "motor", "battery", "inverter", "rules"];
result = any(isfield(powertrain, cellstr(markers)));
end

function result = isLegacyPowertrainPath(parameter)
legacyFields = ["max_power_w", "max_wheel_torque_nm", ...
    "max_total_wheel_torque_nm", "max_speed_mps", ...
    "overall_gear_ratio", "drive_efficiency"];
legacyPaths = "models.powertrain." + legacyFields;
result = any(parameter == legacyPaths);
end

function tire = scaleTireMu(tire, scale)
if ~isscalar(scale) || ~isfinite(scale) || scale <= 0
    error("QSSLTS:TireMuScale", "Tire mu scale must be positive and finite.");
end
switch string(tire.model_type)
    case "constant_mu"
        tire.mu_x = tire.mu_x * scale;
        tire.mu_y = tire.mu_y * scale;
    case "load_sensitive"
        tire.mu_x_ref = tire.mu_x_ref * scale;
        tire.mu_y_ref = tire.mu_y_ref * scale;
    otherwise
        error("QSSLTS:TireMuScale", ...
            "Tire mu scaling is not defined for model %s.", tire.model_type);
end
end
