function brakeResult = calc_brake_limit(v_mps, loads, tire, brake, options)
%CALC_BRAKE_LIMIT Calculate maximum negative tire/brake force [N].
%   front_bias is the fraction of total braking demand assigned to the
%   front axle. loads.vehicle_mass_kg is required for acceleration output.

arguments
    v_mps (1,1) double {mustBeFinite, mustBeNonnegative}
    loads (1,1) struct
    tire (1,1) struct
    brake (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
if ~isfield(loads, "vehicle_mass_kg") || loads.vehicle_mass_kg <= 0
    error("QSSLTS:BrakeMass", ...
        "loads.vehicle_mass_kg must be a positive scalar.");
end
availableFx_N = getAvailableLongitudinalForce(loads, tire);
brake = fillDefaults(brake);

if ~brake.enabled
    forceMagnitude_N = 0;
    limiter = "brake_disabled";
else
    frontCapacity_N = sum(availableFx_N(1:2));
    rearCapacity_N = sum(availableFx_N(3:4));
    biasLimited_N = biasLimitedForce( ...
        frontCapacity_N, rearCapacity_N, brake.front_bias);
    decelForce_N = brake.max_decel_g_mechanical ...
        * loads.vehicle_mass_kg * options.gravity_mps2;
    torqueLimit_N = brakeTorqueForceLimit(brake, tire);
    mechanicalLimit_N = min([brake.max_total_brake_force_N, ...
        decelForce_N, torqueLimit_N]);
    [forceMagnitude_N, index] = min([biasLimited_N, mechanicalLimit_N]);
    labels = ["brake_traction_bias", "brake_mechanical"];
    limiter = labels(index);
end

brakeResult.Fx_brake_min_N = -forceMagnitude_N;
brakeResult.ax_min_mps2 = -forceMagnitude_N / loads.vehicle_mass_kg;
brakeResult.limiter = limiter;
brakeResult.speed_mps = v_mps;
brakeResult.front_capacity_N = sum(availableFx_N(1:2));
brakeResult.rear_capacity_N = sum(availableFx_N(3:4));
end

function torqueLimit_N = brakeTorqueForceLimit(brake, tire)
if isinf(brake.max_total_brake_torque_Nm)
    torqueLimit_N = inf;
    return
end
if ~isfield(tire, "rolling_radius_m") ...
        || ~isscalar(tire.rolling_radius_m) ...
        || ~isfinite(tire.rolling_radius_m) ...
        || tire.rolling_radius_m <= 0
    error("QSSLTS:BrakeParameters", ...
        "A positive finite tire.rolling_radius_m is required " ...
        + "for a finite brake torque limit.");
end
torqueLimit_N = brake.max_total_brake_torque_Nm / tire.rolling_radius_m;
end

function totalLimit_N = biasLimitedForce(front_N, rear_N, frontBias)
if frontBias <= 0
    frontBased_N = inf;
else
    frontBased_N = front_N / frontBias;
end
if frontBias >= 1
    rearBased_N = inf;
else
    rearBased_N = rear_N / (1 - frontBias);
end
totalLimit_N = min([frontBased_N, rearBased_N, front_N + rear_N]);
end

function availableFx_N = getAvailableLongitudinalForce(loads, tire)
if isfield(loads, "Fx_available_N")
    availableFx_N = loads.Fx_available_N(:);
elseif isfield(loads, "Fz_vector_N")
    env = tire_envelope(loads.Fz_vector_N, zeros(4, 1), tire);
    availableFx_N = env.Fx_max_N(:);
else
    error("QSSLTS:BrakeLoads", ...
        "loads must contain Fx_available_N or Fz_vector_N.");
end
if numel(availableFx_N) ~= 4 || any(availableFx_N < 0) ...
        || any(~isfinite(availableFx_N))
    error("QSSLTS:BrakeLoads", ...
        "Available longitudinal force must contain four finite values.");
end
end

function model = fillDefaults(model)
if ~isfield(model, "enabled"), model.enabled = true; end
if ~isfield(model, "front_bias"), model.front_bias = 0.5; end
if ~isfield(model, "max_decel_g_mechanical")
    model.max_decel_g_mechanical = inf;
end
if ~isfield(model, "max_total_brake_force_N")
    model.max_total_brake_force_N = inf;
end
if ~isfield(model, "max_total_brake_torque_Nm")
    model.max_total_brake_torque_Nm = inf;
end
if model.front_bias < 0 || model.front_bias > 1 ...
        || model.max_decel_g_mechanical < 0 ...
        || model.max_total_brake_force_N < 0 ...
        || model.max_total_brake_torque_Nm < 0
    error("QSSLTS:BrakeParameters", ...
        "Brake bias/limits are outside valid bounds.");
end
end
