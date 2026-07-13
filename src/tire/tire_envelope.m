function env = tire_envelope(Fz_N, camber_rad, tire)
%TIRE_ENVELOPE Return per-tire longitudinal/lateral force limits.
%   Fz_N is the nonnegative normal-load magnitude [N]. camber_rad is
%   accepted for interface stability but is unused by the constant-mu model.

arguments
    Fz_N double
    camber_rad double
    tire (1,1) struct
end

if ~(isscalar(camber_rad) || isequal(size(camber_rad), size(Fz_N))) ...
        || any(~isfinite(camber_rad), "all")
    error("QSSLTS:TireCamber", ...
        "camber_rad must be finite and scalar or match Fz_N.");
end
if ~isfield(tire, "model_type") || ~isfield(tire, "combined_n") ...
        || tire.combined_n < 1
    error("QSSLTS:TireParameters", ...
        "Tire model_type/combined_n is missing or invalid.");
end

normalLoad_N = max(Fz_N, 0);
switch string(tire.model_type)
    case "constant_mu"
        if tire.mu_x < 0 || tire.mu_y < 0
            error("QSSLTS:TireParameters", ...
                "mu_x and mu_y must be nonnegative.");
        end
        [muX, muY] = tire_constant_mu(normalLoad_N, tire);
    case "load_sensitive"
        [muX, muY] = tire_load_sensitive_mu(normalLoad_N, tire);
    otherwise
        error("QSSLTS:TireModel", ...
            "Unsupported tire.model_type: %s", string(tire.model_type));
end

env.Fx_max_N = muX .* normalLoad_N;
env.Fy_max_N = muY .* normalLoad_N;
env.mu_x = muX;
env.mu_y = muY;
env.combined_n = tire.combined_n;
end
