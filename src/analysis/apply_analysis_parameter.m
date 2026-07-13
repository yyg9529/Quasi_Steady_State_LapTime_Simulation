function config = apply_analysis_parameter(config, parameter, value)
%APPLY_ANALYSIS_PARAMETER Apply a supported DOE/sensitivity parameter.

arguments
    config (1,1) struct
    parameter (1,1) string
    value
end

if contains(parameter, ".")
    config = set_nested_field(config, parameter, value);
    return
end

switch lower(parameter)
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
    case {"power", "max_power_w"}
        config.models.powertrain.max_power_W = value;
    case {"gear_ratio", "overall_gear_ratio"}
        config = applyGearRatio(config, value);
    case {"brake_bias", "front_bias"}
        config.models.brake.front_bias = value;
    case "tire_mu_scale"
        config.models.tire = scaleTireMu(config.models.tire, value);
    otherwise
        error("QSSLTS:AnalysisParameter", ...
            "Unsupported analysis parameter: %s", parameter);
end
end

function config = applyGearRatio(config, newRatio)
if ~isscalar(newRatio) || ~isfinite(newRatio) || newRatio <= 0
    error("QSSLTS:GearRatio", "Gear ratio must be positive and finite.");
end
powertrain = config.models.powertrain;
if ~isfield(powertrain, "overall_gear_ratio") ...
        || powertrain.overall_gear_ratio <= 0
    error("QSSLTS:GearRatio", ...
        "Baseline powertrain.overall_gear_ratio is required.");
end
ratioScale = newRatio / powertrain.overall_gear_ratio;
powertrain.max_total_wheel_torque_Nm = ...
    powertrain.max_total_wheel_torque_Nm * ratioScale;
powertrain.max_speed_mps = powertrain.max_speed_mps / ratioScale;
powertrain.overall_gear_ratio = newRatio;
config.models.powertrain = powertrain;
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
