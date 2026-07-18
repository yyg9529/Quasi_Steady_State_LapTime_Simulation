function point = evaluate_four_wheel_state(vehicle, tireModel, aero, state)
%EVALUATE_FOUR_WHEEL_STATE Evaluate tire forces and CG equilibrium residuals.
%   The injected tire evaluator receives four-wheel column vectors in the
%   fixed order [FL, FR, RL, RR]. Positive tire Fy corresponds to positive
%   internal slip angle and positive vehicle-body Y force.

arguments
    vehicle (1,1) struct
    tireModel (1,1) struct
    aero (1,1) struct
    state (1,1) struct
end

if ~isfield(tireModel, "evaluate") ...
        || ~isa(tireModel.evaluate, "function_handle")
    error("QSSLTS:HandlingTireEvaluator", ...
        "tireModel.evaluate must be a function handle.");
end

wheel = calc_wheel_kinematics(vehicle, state);
u_mps = state.speed_mps * cos(state.beta_rad);
v_mps = state.speed_mps * sin(state.beta_rad);
ax_mps2 = -state.yaw_rate_radps * v_mps;
ay_mps2 = state.yaw_rate_radps * u_mps;

aeroForce = calc_aero_forces(state.speed_mps, aero);
loadState = make_vehicle_state(state.speed_mps, ax_mps2, ay_mps2);
loadState.suppress_warnings = true;
loads = calc_wheel_loads(loadState, vehicle, aeroForce);

tireInput.Fz_N = loads.Fz_vector_N;
tireInput.kappa = zeros(4, 1);
tireInput.alpha_rad = wheel.alpha_rad;
tireInput.gamma_rad = zeros(4, 1);
tireInput.turn_slip_1pm = zeros(4, 1);
tireInput.Vx_mps = wheel.Vx_tire_mps;
tireInput.mount_side = ["LEFT"; "RIGHT"; "LEFT"; "RIGHT"];
tireOutput = tireModel.evaluate(tireInput);

requiredOutput = ["Fx_N", "Fy_N", "Mz_Nm"];
if ~all(isfield(tireOutput, cellstr(requiredOutput)))
    error("QSSLTS:HandlingTireOutput", ...
        "Tire evaluator must return Fx_N, Fy_N, and Mz_Nm.");
end
FxTire_N = tireOutput.Fx_N(:);
FyTire_N = tireOutput.Fy_N(:);
MzTire_Nm = tireOutput.Mz_Nm(:);
if any([numel(FxTire_N), numel(FyTire_N), numel(MzTire_Nm)] ~= 4)
    error("QSSLTS:HandlingTireOutput", ...
        "Tire evaluator outputs must each contain four wheel values.");
end

cosSteer = cos(wheel.steer_rad);
sinSteer = sin(wheel.steer_rad);
FxBody_N = cosSteer .* FxTire_N - sinSteer .* FyTire_N;
FyBody_N = sinSteer .* FxTire_N + cosSteer .* FyTire_N;
yawMoment_Nm = sum(wheel.x_m .* FyBody_N ...
    - wheel.y_m .* FxBody_N + MzTire_Nm);

wheel.Fz_N = loads.Fz_vector_N;
wheel.kappa = tireInput.kappa;
wheel.gamma_rad = tireInput.gamma_rad;
wheel.Fx_tire_N = FxTire_N;
wheel.Fy_tire_N = FyTire_N;
wheel.Mz_tire_Nm = MzTire_Nm;
wheel.Fx_body_N = FxBody_N;
wheel.Fy_body_N = FyBody_N;
if isfield(tireOutput, "within_range")
    wheel.within_tire_range = logical(tireOutput.within_range(:));
else
    wheel.within_tire_range = true(4, 1);
end

point.wheel = wheel;
point.aero_force = aeroForce;
point.Fx_body_N = FxBody_N;
point.Fy_body_N = FyBody_N;
point.Fx_total_N = sum(FxBody_N) - aeroForce.drag_N;
point.Fy_total_N = sum(FyBody_N);
point.yaw_moment_cg_Nm = yawMoment_Nm;
point.ax_mps2 = ax_mps2;
point.ay_mps2 = ay_mps2;
point.normal_acceleration_mps2 = ...
    state.yaw_rate_radps * state.speed_mps;
point.lateral_residual_N = point.Fy_total_N ...
    - vehicle.mass.total_kg * state.yaw_rate_radps * u_mps;
point.within_tire_range = wheel.within_tire_range;
end
