function force = convert_tire_force_to_internal(nativeForce, convention)
%CONVERT_TIRE_FORCE_TO_INTERNAL Convert a native tire force to X-forward/Y-left.
%   Use this helper inside a concrete external evaluator. The generic
%   adapter accepts only canonical QSSLTS_TIRE_FORCE_V1 evaluator outputs.

    arguments
        nativeForce (1, 1) struct
        convention (1, 1) string
    end

    if ~all(isfield(nativeForce, ["Fx_N", "Fy_N"])) ...
            || ~areFiniteRealEqualSize(nativeForce.Fx_N, nativeForce.Fy_N)
        error("QSSLTS:TireAdapterOutput", ...
            "Native tire force must contain equal-size finite Fx_N/Fy_N.");
    end

    switch upper(convention)
        case "QSSLTS_X_FORWARD_Y_LEFT_Z_UP"
            lateralSign = 1;
        case "SAE_J670_X_FORWARD_Y_RIGHT_Z_DOWN"
            lateralSign = -1;
        otherwise
            error("QSSLTS:TireParameterSchema", ...
                "Unsupported tire-force coordinate convention: %s", ...
                convention);
    end

    force.Fx_N = nativeForce.Fx_N;
    force.Fy_N = lateralSign .* nativeForce.Fy_N;
end

function valid = areFiniteRealEqualSize(Fx_N, Fy_N)
    valid = isnumeric(Fx_N) && isnumeric(Fy_N) ...
        && isequal(size(Fx_N), size(Fy_N)) ...
        && all(isfinite(Fx_N), "all") && all(isfinite(Fy_N), "all") ...
        && isreal(Fx_N) && isreal(Fy_N);
end
