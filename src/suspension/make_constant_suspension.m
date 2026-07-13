function susp = make_constant_suspension(camber_rad, toe_rad)
%MAKE_CONSTANT_SUSPENSION Create a geometry-independent fallback model.

    arguments
        camber_rad (1, 1) double {mustBeFinite, mustBeReal} = 0
        toe_rad (1, 1) double {mustBeFinite, mustBeReal} = 0
    end

    susp.model_type = "constant";
    susp.camber_rad = camber_rad;
    susp.toe_rad = toe_rad;
    susp.motion_ratio_definition = "unavailable";
    susp.wheel_rate_usage_confirmed = false;
end
