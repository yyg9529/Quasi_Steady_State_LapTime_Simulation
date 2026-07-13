function [muX, muY] = tire_load_sensitive_mu(Fz_N, tire)
%TIRE_LOAD_SENSITIVE_MU Return friction coefficients versus normal load.
%   mu = mu_ref*(1+k*(Fz-Fz_ref)/Fz_ref). Fz uses N.

arguments
    Fz_N double
    tire (1,1) struct
end

required = ["Fz_ref_N", "mu_x_ref", "mu_y_ref", ...
    "load_sensitivity_x", "load_sensitivity_y"];
if ~all(isfield(tire, cellstr(required))) || tire.Fz_ref_N <= 0 ...
        || tire.mu_x_ref < 0 || tire.mu_y_ref < 0
    error("QSSLTS:TireLoadSensitivityParameters", ...
        "Load-sensitive tire parameters are invalid or incomplete.");
end

normalizedLoad = (max(Fz_N, 0) - tire.Fz_ref_N) / tire.Fz_ref_N;
muX = tire.mu_x_ref .* (1 + tire.load_sensitivity_x .* normalizedLoad);
muY = tire.mu_y_ref .* (1 + tire.load_sensitivity_y .* normalizedLoad);
muX = max(muX, 0);
muY = max(muY, 0);
end
