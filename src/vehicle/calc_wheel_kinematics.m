function wheel = calc_wheel_kinematics(vehicle, state)
%CALC_WHEEL_KINEMATICS Resolve four contact-patch velocities and slip angles.
%   Wheel order is [FL, FR, RL, RR]. Coordinates follow ISO vehicle axes:
%   +X forward, +Y left, +Z up. Positive steer and yaw are leftward.

arguments
    vehicle (1,1) struct
    state (1,1) struct
end

requiredState = ["speed_mps", "beta_rad", "yaw_rate_radps", "steer_rad"];
if ~all(isfield(state, cellstr(requiredState)))
    error("QSSLTS:HandlingState", ...
        "Handling state requires speed, beta, yaw rate, and steer fields.");
end
if state.speed_mps < 0 || ~isscalar(state.steer_rad)
    error("QSSLTS:HandlingState", ...
        "speed_mps must be nonnegative and steer_rad must be scalar.");
end

wheelbase_m = vehicle.geometry.wheelbase_m;
frontFraction = vehicle.mass.front_static_frac;
b_m = frontFraction * wheelbase_m;
a_m = wheelbase_m - b_m;

wheel.x_m = [a_m; a_m; -b_m; -b_m];
wheel.y_m = 0.5 * [ ...
    vehicle.geometry.track_front_m
    -vehicle.geometry.track_front_m
    vehicle.geometry.track_rear_m
    -vehicle.geometry.track_rear_m];
wheel.steer_rad = [state.steer_rad; state.steer_rad; 0; 0];

u_mps = state.speed_mps * cos(state.beta_rad);
v_mps = state.speed_mps * sin(state.beta_rad);
wheel.Vx_body_mps = u_mps - state.yaw_rate_radps .* wheel.y_m;
wheel.Vy_body_mps = v_mps + state.yaw_rate_radps .* wheel.x_m;

cosSteer = cos(wheel.steer_rad);
sinSteer = sin(wheel.steer_rad);
wheel.Vx_tire_mps = cosSteer .* wheel.Vx_body_mps ...
    + sinSteer .* wheel.Vy_body_mps;
wheel.Vy_tire_mps = -sinSteer .* wheel.Vx_body_mps ...
    + cosSteer .* wheel.Vy_body_mps;
wheel.alpha_rad = atan2(-wheel.Vy_tire_mps, wheel.Vx_tire_mps);
end
