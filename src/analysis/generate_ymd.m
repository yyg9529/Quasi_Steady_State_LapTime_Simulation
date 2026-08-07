function ymd = generate_ymd(vehicle, tireModel, aero, study)
%GENERATE_YMD Generate a quasi-steady yaw-moment diagram over beta and steer.
%   Each grid point solves lateral force equilibrium for yaw rate while beta,
%   road-wheel steer, and speed remain fixed.

arguments
    vehicle (1,1) struct
    tireModel (1,1) struct
    aero (1,1) struct
    study (1,1) struct
end

requiredStudy = ["speed_mps", "beta_rad", "steer_rad"];
if ~all(isfield(study, cellstr(requiredStudy))) || study.speed_mps <= 0
    error("QSSLTS:YmdStudy", ...
        "YMD study requires positive speed and beta/steer grids.");
end

betaValues = study.beta_rad(:);
steerValues = study.steer_rad(:).';
nBeta = numel(betaValues);
nSteer = numel(steerValues);

ymd.speed_mps = study.speed_mps;
ymd.beta_rad = repmat(betaValues, 1, nSteer);
ymd.steer_rad = repmat(steerValues, nBeta, 1);
ymd.ay_mps2 = nan(nBeta, nSteer);
ymd.normal_acceleration_mps2 = nan(nBeta, nSteer);
ymd.ay_g = nan(nBeta, nSteer);
ymd.yaw_rate_radps = nan(nBeta, nSteer);
ymd.curvature_1pm = nan(nBeta, nSteer);
ymd.yaw_moment_cg_Nm = nan(nBeta, nSteer);
ymd.lateral_residual_N = nan(nBeta, nSteer);
ymd.converged = false(nBeta, nSteer);
ymd.within_tire_range = false(nBeta, nSteer);
ymd.iterations = zeros(nBeta, nSteer);
ymd.wheel.Fz_N = nan(nBeta, nSteer, 4);
ymd.wheel.Vx_tire_mps = nan(nBeta, nSteer, 4);
ymd.wheel.Vy_tire_mps = nan(nBeta, nSteer, 4);
ymd.wheel.alpha_rad = nan(nBeta, nSteer, 4);
ymd.wheel.kappa = nan(nBeta, nSteer, 4);
ymd.wheel.Fx_tire_N = nan(nBeta, nSteer, 4);
ymd.wheel.Fy_tire_N = nan(nBeta, nSteer, 4);
ymd.wheel.Mz_tire_Nm = nan(nBeta, nSteer, 4);
ymd.wheel.Fx_body_N = nan(nBeta, nSteer, 4);
ymd.wheel.Fy_body_N = nan(nBeta, nSteer, 4);

for iBeta = 1:nBeta
    for iSteer = 1:nSteer
        state.speed_mps = study.speed_mps;
        state.beta_rad = betaValues(iBeta);
        state.steer_rad = steerValues(iSteer);
        initialYawRate_radps = study.speed_mps * state.steer_rad ...
            / vehicle.geometry.wheelbase_m;
        [state.yaw_rate_radps, point, converged, iterations] = ...
            solveYawRate(vehicle, tireModel, aero, state, ...
            initialYawRate_radps);

        ymd.ay_mps2(iBeta, iSteer) = point.ay_mps2;
        ymd.normal_acceleration_mps2(iBeta, iSteer) = ...
            point.normal_acceleration_mps2;
        ymd.ay_g(iBeta, iSteer) = point.ay_mps2 / 9.80665;
        ymd.yaw_rate_radps(iBeta, iSteer) = state.yaw_rate_radps;
        ymd.curvature_1pm(iBeta, iSteer) = ...
            state.yaw_rate_radps / study.speed_mps;
        ymd.yaw_moment_cg_Nm(iBeta, iSteer) = ...
            point.yaw_moment_cg_Nm;
        ymd.lateral_residual_N(iBeta, iSteer) = ...
            point.lateral_residual_N;
        ymd.converged(iBeta, iSteer) = converged;
        ymd.within_tire_range(iBeta, iSteer) = ...
            all(point.within_tire_range);
        ymd.iterations(iBeta, iSteer) = iterations;
        ymd.wheel.Fz_N(iBeta, iSteer, :) = reshape( ...
            point.wheel.Fz_N, 1, 1, 4);
        ymd.wheel.alpha_rad(iBeta, iSteer, :) = reshape( ...
            point.wheel.alpha_rad, 1, 1, 4);
        ymd.wheel.Vx_tire_mps(iBeta, iSteer, :) = reshape( ...
            point.wheel.Vx_tire_mps, 1, 1, 4);
        ymd.wheel.Vy_tire_mps(iBeta, iSteer, :) = reshape( ...
            point.wheel.Vy_tire_mps, 1, 1, 4);
        ymd.wheel.kappa(iBeta, iSteer, :) = reshape( ...
            point.wheel.kappa, 1, 1, 4);
        ymd.wheel.Fx_tire_N(iBeta, iSteer, :) = reshape( ...
            point.wheel.Fx_tire_N, 1, 1, 4);
        ymd.wheel.Fy_tire_N(iBeta, iSteer, :) = reshape( ...
            point.wheel.Fy_tire_N, 1, 1, 4);
        ymd.wheel.Mz_tire_Nm(iBeta, iSteer, :) = reshape( ...
            point.wheel.Mz_tire_Nm, 1, 1, 4);
        ymd.wheel.Fx_body_N(iBeta, iSteer, :) = reshape( ...
            point.Fx_body_N, 1, 1, 4);
        ymd.wheel.Fy_body_N(iBeta, iSteer, :) = reshape( ...
            point.Fy_body_N, 1, 1, 4);
    end
end
yawMomentReference_Nm = vehicle.mass.total_kg * 9.80665 ...
    * vehicle.geometry.wheelbase_m;
ymd.yaw_moment_coefficient = ...
    ymd.yaw_moment_cg_Nm / yawMomentReference_Nm;
ymd.provenance.model = "four-wheel quasi-steady yaw moment diagram";
ymd.provenance.wheel_order = ["FL", "FR", "RL", "RR"];
ymd.provenance.yaw_moment_coefficient_reference_Nm = ...
    yawMomentReference_Nm;
if isfield(tireModel, "source")
    ymd.provenance.tire_source = tireModel.source;
end
end

function [yawRate_radps, point, converged, iterations] = solveYawRate( ...
        vehicle, tireModel, aero, state, initialYawRate_radps)
maxIterations = 30;
forceTolerance_N = 1e-8 * max(1, vehicle.mass.total_kg * 9.80665);
yawRate_radps = initialYawRate_radps;
iterations = 0;

for iIteration = 1:maxIterations
    iterations = iIteration;
    state.yaw_rate_radps = yawRate_radps;
    point = evaluate_four_wheel_state(vehicle, tireModel, aero, state);
    residual_N = point.lateral_residual_N;
    if abs(residual_N) <= forceTolerance_N
        converged = true;
        return
    end

    step_radps = max(1e-6, 1e-5 * max(1, abs(yawRate_radps)));
    plusState = state;
    minusState = state;
    plusState.yaw_rate_radps = yawRate_radps + step_radps;
    minusState.yaw_rate_radps = yawRate_radps - step_radps;
    plusPoint = evaluate_four_wheel_state( ...
        vehicle, tireModel, aero, plusState);
    minusPoint = evaluate_four_wheel_state( ...
        vehicle, tireModel, aero, minusState);
    derivative_Npradps = (plusPoint.lateral_residual_N ...
        - minusPoint.lateral_residual_N) / (2 * step_radps);
    if abs(derivative_Npradps) <= eps(max(1, abs(residual_N)))
        break
    end

    correction_radps = -residual_N / derivative_Npradps;
    correctionLimit_radps = max(1, ...
        2 * state.speed_mps / vehicle.geometry.wheelbase_m);
    correction_radps = max(-correctionLimit_radps, ...
        min(correctionLimit_radps, correction_radps));
    yawRate_radps = yawRate_radps + correction_radps;
end

state.yaw_rate_radps = yawRate_radps;
point = evaluate_four_wheel_state(vehicle, tireModel, aero, state);
converged = abs(point.lateral_residual_N) <= forceTolerance_N;
end
