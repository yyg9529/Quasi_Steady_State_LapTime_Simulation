function force = tire_slip_force_simple( ...
        slipRatio, slipAngle_rad, Fz_N, camber_rad, tire)
%TIRE_SLIP_FORCE_SIMPLE Linear slip stiffness with p-norm saturation.
%   Positive slip ratio produces positive wheel-frame Fx. Positive slip
%   angle produces negative wheel-frame Fy, opposing lateral tire motion.

    parameters = tire.slip_force;
    required = ["Fz_ref_N", "longitudinal_stiffness_ref_N", ...
        "cornering_stiffness_ref_Nprad"];
    if ~all(isfield(parameters, cellstr(required)))
        error("QSSLTS:TireForceParameters", ...
            "Simple slip-force stiffness parameters are missing.");
    end
    stiffnessValues = [parameters.Fz_ref_N, ...
        parameters.longitudinal_stiffness_ref_N, ...
        parameters.cornering_stiffness_ref_Nprad];
    validCombinedExponent = isfield(tire, "combined_n") ...
        && isnumeric(tire.combined_n) && isscalar(tire.combined_n) ...
        && isreal(tire.combined_n) && isfinite(tire.combined_n) ...
        && tire.combined_n >= 1;
    if ~isnumeric(stiffnessValues) || numel(stiffnessValues) ~= 3 ...
            || ~all(isfinite(stiffnessValues)) ...
            || ~all(isreal(stiffnessValues)) ...
            || any(stiffnessValues <= 0) || ~validCombinedExponent
        error("QSSLTS:TireForceParameters", ...
            "Simple slip-force parameters must be finite positive scalars.");
    end
    if ~isequal(size(slipRatio), size(slipAngle_rad), size(Fz_N))
        error("QSSLTS:TireForceSize", ...
            "Slip ratio, slip angle, and Fz must have equal sizes.");
    end

    normalLoad_N = max(Fz_N, 0);
    loadScale = normalLoad_N ./ parameters.Fz_ref_N;
    FxLinear_N = parameters.longitudinal_stiffness_ref_N ...
        .* loadScale .* slipRatio;
    FyLinear_N = -parameters.cornering_stiffness_ref_Nprad ...
        .* loadScale .* slipAngle_rad;

    env = tire_envelope(normalLoad_N, camber_rad, tire);
    ux = normalizedDemand(FxLinear_N, env.Fx_max_N);
    uy = normalizedDemand(FyLinear_N, env.Fy_max_N);
    combinedDemand = ux .^ tire.combined_n + uy .^ tire.combined_n;
    scale = ones(size(combinedDemand));
    saturated = combinedDemand > 1;
    scale(saturated) = combinedDemand(saturated) ...
        .^ (-1 / tire.combined_n);

    force.Fx_N = FxLinear_N .* scale;
    force.Fy_N = FyLinear_N .* scale;
    force.Fx_linear_N = FxLinear_N;
    force.Fy_linear_N = FyLinear_N;
    force.combined_usage = normalizedDemand( ...
        force.Fx_N, env.Fx_max_N) .^ tire.combined_n ...
        + normalizedDemand(force.Fy_N, env.Fy_max_N) ...
        .^ tire.combined_n;
    force.is_saturated = saturated;
end

function demand = normalizedDemand(force_N, limit_N)
    demand = zeros(size(force_N));
    loaded = limit_N > 0;
    demand(loaded) = abs(force_N(loaded)) ./ limit_N(loaded);
    demand(~loaded & abs(force_N) > 0) = inf;
end
