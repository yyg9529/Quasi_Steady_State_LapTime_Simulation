function env = tire_envelope(Fz_N, camber_rad, tire)
%TIRE_ENVELOPE Return per-tire longitudinal/lateral force limits.
%   Fz_N is the nonnegative normal-load magnitude [N]. camber_rad is
%   accepted for interface stability but is unused by the constant-mu model.

arguments
    Fz_N double
    camber_rad double
    tire (1,1) struct
end

if ~isscalar(camber_rad) || ~isfinite(camber_rad)
    error("QSSLTS:TireCamber", "camber_rad must be a finite scalar.");
end
if ~isfield(tire, "model_type") || string(tire.model_type) ~= "constant_mu"
    error("QSSLTS:TireModel", ...
        "V0.1 supports only tire.model_type = constant_mu.");
end
if tire.mu_x < 0 || tire.mu_y < 0 || tire.combined_n < 1
    error("QSSLTS:TireParameters", ...
        "mu_x/mu_y must be nonnegative and combined_n must be >= 1.");
end

normalLoad_N = max(Fz_N, 0);
env.Fx_max_N = tire.mu_x .* normalLoad_N;
env.Fy_max_N = tire.mu_y .* normalLoad_N;
env.mu_x = tire.mu_x + zeros(size(normalLoad_N));
env.mu_y = tire.mu_y + zeros(size(normalLoad_N));
env.combined_n = tire.combined_n;
end
