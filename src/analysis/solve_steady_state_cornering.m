function steady = solve_steady_state_cornering( ...
        vehicle, tireModel, aero, condition)
%SOLVE_STEADY_STATE_CORNERING Solve beta and steer at fixed speed/radius.
%   The solution enforces both lateral-force and CG yaw-moment equilibrium.

arguments
    vehicle (1,1) struct
    tireModel (1,1) struct
    aero (1,1) struct
    condition (1,1) struct
end

requiredCondition = ["speed_mps", "radius_m"];
if ~all(isfield(condition, cellstr(requiredCondition))) ...
        || condition.speed_mps <= 0 || condition.radius_m == 0
    error("QSSLTS:SteadyCorneringCondition", ...
        "Condition requires positive speed and nonzero signed radius.");
end

yawRate_radps = condition.speed_mps / condition.radius_m;
unknown = [0; vehicle.geometry.wheelbase_m / condition.radius_m];
maxIterations = 40;
forceTolerance_N = 1e-6;
momentTolerance_Nm = 1e-6;
converged = false;
iterations = 0;

for iIteration = 1:maxIterations
    iterations = iIteration;
    [residual, ~] = evaluateResidual(unknown, yawRate_radps, ...
        condition.speed_mps, vehicle, tireModel, aero);
    if abs(residual(1)) <= forceTolerance_N ...
            && abs(residual(2)) <= momentTolerance_Nm
        converged = true;
        break
    end

    jacobian = finiteDifferenceJacobian(unknown, yawRate_radps, ...
        condition.speed_mps, vehicle, tireModel, aero);
    if rcond(jacobian) <= 1e-12
        break
    end
    correction = -jacobian \ residual;
    correctionNorm = norm(correction);
    if correctionNorm > 0.2
        correction = 0.2 * correction / correctionNorm;
    end
    unknown = acceptDampedStep(unknown, correction, residual, ...
        yawRate_radps, condition.speed_mps, vehicle, tireModel, aero);
end

[residual, point] = evaluateResidual(unknown, yawRate_radps, ...
    condition.speed_mps, vehicle, tireModel, aero);
converged = converged || (abs(residual(1)) <= forceTolerance_N ...
    && abs(residual(2)) <= momentTolerance_Nm);

steady.speed_mps = condition.speed_mps;
steady.radius_m = condition.radius_m;
steady.curvature_1pm = 1 / condition.radius_m;
steady.yaw_rate_radps = yawRate_radps;
steady.beta_rad = unknown(1);
steady.roadwheel_steer_rad = unknown(2);
steady.ay_mps2 = condition.speed_mps^2 / condition.radius_m;
steady.ay_g = steady.ay_mps2 / 9.80665;
steady.force_residual_N = residual(1);
steady.moment_residual_Nm = residual(2);
steady.converged = converged;
steady.iterations = iterations;
steady.wheel = point.wheel;
steady.within_tire_range = all(point.within_tire_range);
end

function [residual, point] = evaluateResidual(unknown, yawRate_radps, ...
        speed_mps, vehicle, tireModel, aero)
state.speed_mps = speed_mps;
state.beta_rad = unknown(1);
state.yaw_rate_radps = yawRate_radps;
state.steer_rad = unknown(2);
point = evaluate_four_wheel_state(vehicle, tireModel, aero, state);
residual = [point.lateral_residual_N; point.yaw_moment_cg_Nm];
end

function jacobian = finiteDifferenceJacobian(unknown, yawRate_radps, ...
        speed_mps, vehicle, tireModel, aero)
step_rad = 1e-6;
betaOffset = [step_rad; 0];
steerOffset = [0; step_rad];
betaPlus = evaluateResidual(unknown + betaOffset, yawRate_radps, ...
    speed_mps, vehicle, tireModel, aero);
betaMinus = evaluateResidual(unknown - betaOffset, yawRate_radps, ...
    speed_mps, vehicle, tireModel, aero);
steerPlus = evaluateResidual(unknown + steerOffset, yawRate_radps, ...
    speed_mps, vehicle, tireModel, aero);
steerMinus = evaluateResidual(unknown - steerOffset, yawRate_radps, ...
    speed_mps, vehicle, tireModel, aero);
jacobian = [(betaPlus - betaMinus) / (2 * step_rad), ...
    (steerPlus - steerMinus) / (2 * step_rad)];
end

function accepted = acceptDampedStep(unknown, correction, residual, ...
        yawRate_radps, speed_mps, vehicle, tireModel, aero)
forceScale_N = max(1, vehicle.mass.total_kg * 9.80665);
momentScale_Nm = forceScale_N * vehicle.geometry.wheelbase_m;
residualScale = [forceScale_N; momentScale_Nm];
currentNorm = norm(residual ./ residualScale);
accepted = unknown + correction;

for iDamping = 0:8
    candidate = unknown + (0.5^iDamping) * correction;
    candidateResidual = evaluateResidual(candidate, yawRate_radps, ...
        speed_mps, vehicle, tireModel, aero);
    if norm(candidateResidual ./ residualScale) < currentNorm
        accepted = candidate;
        return
    end
end
end
