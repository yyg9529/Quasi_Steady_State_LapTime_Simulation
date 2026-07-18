function result = calc_understeer_gradient( ...
        vehicle, tireModel, aero, study)
%CALC_UNDERSTEER_GRADIENT Evaluate fixed-radius road-wheel steer gradient.
%   linear_fit_gradient_deg_per_g is the classical zero-slip, small-angle
%   gradient from linearized axle cornering stiffness. The steady-curve and
%   local gradients retain the finite-radius four-wheel solution behavior.
%   All gradients use road-wheel steer in deg/g; positive means understeer.

arguments
    vehicle (1,1) struct
    tireModel (1,1) struct
    aero (1,1) struct
    study (1,1) struct
end

requiredStudy = ["radius_m", "speed_mps", "linear_fit_range_g"];
if ~all(isfield(study, cellstr(requiredStudy))) ...
        || study.radius_m == 0 || numel(study.speed_mps) < 2
    error("QSSLTS:UndersteerStudy", ...
        "Study requires nonzero radius, at least two speeds, and fit range.");
end

speed_mps = study.speed_mps(:);
nSpeed = numel(speed_mps);
beta_rad = nan(nSpeed, 1);
steer_rad = nan(nSpeed, 1);
yawRate_radps = nan(nSpeed, 1);
ay_g = nan(nSpeed, 1);
forceResidual_N = nan(nSpeed, 1);
momentResidual_Nm = nan(nSpeed, 1);
converged = false(nSpeed, 1);

for iSpeed = 1:nSpeed
    condition.speed_mps = speed_mps(iSpeed);
    condition.radius_m = study.radius_m;
    steady = solve_steady_state_cornering( ...
        vehicle, tireModel, aero, condition);
    beta_rad(iSpeed) = steady.beta_rad;
    steer_rad(iSpeed) = steady.roadwheel_steer_rad;
    yawRate_radps(iSpeed) = steady.yaw_rate_radps;
    ay_g(iSpeed) = steady.ay_g;
    forceResidual_N(iSpeed) = steady.force_residual_N;
    momentResidual_Nm(iSpeed) = steady.moment_residual_Nm;
    converged(iSpeed) = steady.converged;
end

steer_deg = rad2deg(steer_rad);
localGradient_deg_per_g = gradient(steer_deg, ay_g);
fitMask = ay_g >= min(study.linear_fit_range_g) ...
    & ay_g <= max(study.linear_fit_range_g) & converged;
if nnz(fitMask) < 2
    error("QSSLTS:UndersteerFitRange", ...
        "At least two converged cases must lie inside linear_fit_range_g.");
end

steadyFitCoefficients = polyfit(ay_g(fitMask), steer_deg(fitMask), 1);
fitPrediction_deg = polyval(steadyFitCoefficients, ay_g(fitMask));
fitResidual_deg = steer_deg(fitMask) - fitPrediction_deg;
centered_deg = steer_deg(fitMask) - mean(steer_deg(fitMask));
steadyFitR2 = 1 - sum(fitResidual_deg.^2) / sum(centered_deg.^2);
referenceSpeed_mps = mean(speed_mps(fitMask));
linearizedGradient_deg_per_g = linearizedUndersteerGradient( ...
    vehicle, tireModel, aero, referenceSpeed_mps);

result.radius_m = study.radius_m;
result.speed_mps = speed_mps;
result.ay_g = ay_g;
result.beta_rad = beta_rad;
result.roadwheel_steer_rad = steer_rad;
result.yaw_rate_radps = yawRate_radps;
result.local_gradient_deg_per_g = localGradient_deg_per_g;
result.linear_fit_gradient_deg_per_g = linearizedGradient_deg_per_g;
result.linear_fit_range_g = study.linear_fit_range_g;
result.linear_fit_R2 = 1;
result.steady_curve_gradient_deg_per_g = steadyFitCoefficients(1);
result.steady_curve_R2 = steadyFitR2;
result.force_residual_N = forceResidual_N;
result.moment_residual_Nm = momentResidual_Nm;
result.converged = converged;
result.provenance.linear_fit_definition = ...
    "zero-slip small-angle axle-stiffness linearization";
result.provenance.linear_fit_R2_definition = ...
    "unity by construction for the classical linearized relation";
result.provenance.steady_curve_definition = ...
    "least-squares fit of finite-radius four-wheel steady points";
end

function gradient_deg_per_g = linearizedUndersteerGradient( ...
        vehicle, tireModel, aero, referenceSpeed_mps)
slipStep_rad = 1e-6;
aeroForce = calc_aero_forces(referenceSpeed_mps, aero);
loadState = make_vehicle_state(referenceSpeed_mps, 0, 0);
loads = calc_wheel_loads(loadState, vehicle, aeroForce);

tireInput.Fz_N = loads.Fz_vector_N;
tireInput.kappa = zeros(4, 1);
tireInput.gamma_rad = zeros(4, 1);
tireInput.turn_slip_1pm = zeros(4, 1);
tireInput.Vx_mps = referenceSpeed_mps * ones(4, 1);
tireInput.mount_side = ["LEFT"; "RIGHT"; "LEFT"; "RIGHT"];
tireInput.alpha_rad = slipStep_rad * ones(4, 1);
plusOutput = tireModel.evaluate(tireInput);
tireInput.alpha_rad = -slipStep_rad * ones(4, 1);
minusOutput = tireModel.evaluate(tireInput);
wheelStiffness_Nprad = (plusOutput.Fy_N(:) - minusOutput.Fy_N(:)) ...
    / (2 * slipStep_rad);

frontStiffness_Nprad = sum(wheelStiffness_Nprad(1:2));
rearStiffness_Nprad = sum(wheelStiffness_Nprad(3:4));
if frontStiffness_Nprad <= 0 || rearStiffness_Nprad <= 0
    error("QSSLTS:UndersteerCorneringStiffness", ...
        "Linearized front and rear cornering stiffness must be positive.");
end

wheelbase_m = vehicle.geometry.wheelbase_m;
b_m = vehicle.mass.front_static_frac * wheelbase_m;
a_m = wheelbase_m - b_m;
gradient_rad_per_g = vehicle.mass.total_kg * 9.80665 / wheelbase_m ...
    * (b_m / frontStiffness_Nprad - a_m / rearStiffness_Nprad);
gradient_deg_per_g = rad2deg(gradient_rad_per_g);
end
