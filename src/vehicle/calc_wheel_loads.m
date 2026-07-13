function loads = calc_wheel_loads(state, vehicle, aeroForce)
%CALC_WHEEL_LOADS Calculate four wheel normal-load magnitudes [N].
%   Wheel order is [FL, FR, RL, RR]. A saturated wheel-lift treatment keeps
%   each axle's total load while setting the lifted wheel to zero.

arguments
    state (1,1) struct
    vehicle (1,1) struct
    aeroForce (1,1) struct = struct()
end

state = fillStateDefaults(state);
aeroForce = fillAeroDefaults(aeroForce);
gravity_mps2 = 9.80665;
if isfield(state, "gravity_mps2")
    gravity_mps2 = state.gravity_mps2;
end

staticLoads = calc_static_loads(vehicle, gravity_mps2);
transfer = calc_load_transfer(state, vehicle);

frontTotal_N = staticLoads.Fz_front_total_N ...
    + aeroForce.downforce_front_N - transfer.longitudinal_N;
rearTotal_N = staticLoads.Fz_rear_total_N ...
    + aeroForce.downforce_rear_N + transfer.longitudinal_N;
if frontTotal_N < 0 || rearTotal_N < 0
    error("QSSLTS:AxleLiftModelInvalid", ...
        "Quasi-static pitch transfer produced a negative axle load.");
end

rawFront_N = [
    0.5 * frontTotal_N - transfer.front_lateral_per_side_N
    0.5 * frontTotal_N + transfer.front_lateral_per_side_N
];
rawRear_N = [
    0.5 * rearTotal_N - transfer.rear_lateral_per_side_N
    0.5 * rearTotal_N + transfer.rear_lateral_per_side_N
];

[front_N, frontLift] = saturateAxleLoads(rawFront_N, frontTotal_N);
[rear_N, rearLift] = saturateAxleLoads(rawRear_N, rearTotal_N);
wheelLoads_N = [front_N; rear_N];

loads.Fz_FL_N = wheelLoads_N(1);
loads.Fz_FR_N = wheelLoads_N(2);
loads.Fz_RL_N = wheelLoads_N(3);
loads.Fz_RR_N = wheelLoads_N(4);
loads.Fz_vector_N = wheelLoads_N;
loads.raw_Fz_vector_N = [rawFront_N; rawRear_N];
loads.Fz_front_total_N = sum(front_N);
loads.Fz_rear_total_N = sum(rear_N);
loads.Fz_total_N = sum(wheelLoads_N);
loads.has_wheel_lift = frontLift || rearLift;
loads.transfer = transfer;

expectedTotal_N = staticLoads.Fz_total_N + aeroForce.downforce_total_N;
if abs(loads.Fz_total_N - expectedTotal_N) ...
        > 1e-9 * max(1, expectedTotal_N)
    error("QSSLTS:VerticalLoadConservation", ...
        "Wheel loads do not conserve total vertical load.");
end
if loads.has_wheel_lift && ~state.suppress_warnings
    warning("QSSLTS:WheelLift", ...
        "Wheel lift detected; lateral load transfer was saturated by axle.");
end
end

function state = fillStateDefaults(state)
required = ["v_mps", "ax_mps2", "ay_mps2"];
if ~all(isfield(state, cellstr(required)))
    error("QSSLTS:VehicleState", "State is missing v/ax/ay fields.");
end
if ~isfield(state, "suppress_warnings")
    state.suppress_warnings = false;
end
end

function aero = fillAeroDefaults(aero)
if ~isfield(aero, "downforce_total_N"), aero.downforce_total_N = 0; end
if ~isfield(aero, "downforce_front_N"), aero.downforce_front_N = 0; end
if ~isfield(aero, "downforce_rear_N"), aero.downforce_rear_N = 0; end
if abs(aero.downforce_front_N + aero.downforce_rear_N ...
        - aero.downforce_total_N) > 1e-9 * max(1, aero.downforce_total_N)
    error("QSSLTS:AeroLoadBalance", ...
        "Front and rear downforce must sum to total downforce.");
end
end

function [loads_N, hasLift] = saturateAxleLoads(rawLoads_N, axleTotal_N)
hasLift = any(rawLoads_N < 0);
loads_N = rawLoads_N;
if rawLoads_N(1) < 0
    loads_N = [0; axleTotal_N];
elseif rawLoads_N(2) < 0
    loads_N = [axleTotal_N; 0];
end
end
