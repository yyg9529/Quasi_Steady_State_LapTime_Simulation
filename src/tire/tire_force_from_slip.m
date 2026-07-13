function force = tire_force_from_slip( ...
        slipRatio, slipAngle_rad, Fz_N, camber_rad, tire, operatingPoint)
%TIRE_FORCE_FROM_SLIP Dispatch the transient tire-force constitutive model.

    arguments
        slipRatio double {mustBeFinite, mustBeReal}
        slipAngle_rad double {mustBeFinite, mustBeReal}
        Fz_N double {mustBeFinite, mustBeReal}
        camber_rad double {mustBeFinite, mustBeReal}
        tire (1, 1) struct
        operatingPoint (1, 1) struct = struct()
    end

    if ~isfield(tire, "slip_force") ...
            || ~isfield(tire.slip_force, "model_type")
        error("QSSLTS:TireForceModel", ...
            "tire.slip_force.model_type is required for 7DOF forces.");
    end

    switch string(tire.slip_force.model_type)
        case "simple_saturated"
            force = tire_slip_force_simple( ...
                slipRatio, slipAngle_rad, Fz_N, camber_rad, tire);
        case "external_adapter"
            force = tire_force_external_adapter( ...
                slipRatio, slipAngle_rad, Fz_N, camber_rad, tire, ...
                operatingPoint);
        otherwise
            error("QSSLTS:TireForceModel", ...
                "Unsupported slip-force model: %s", ...
                string(tire.slip_force.model_type));
    end
end
