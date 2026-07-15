function drive = calc_drive_limit(v_mps, loads, tire, powertrain, options)
%CALC_DRIVE_LIMIT Calculate maximum positive tractive force [N].
%   max_total_wheel_torque_Nm is the sum of wheel-side torque over all
%   driven wheels, after gearing. max_power_W is upstream power, so only
%   the power branch applies drive_efficiency. If powertrain.enabled=false,
%   the result is an ideal all-wheel tire limit for simple-model studies.

arguments
    v_mps (1,1) double {mustBeFinite, mustBeNonnegative}
    loads (1,1) struct
    tire (1,1) struct
    powertrain (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
availableFx_N = getAvailableLongitudinalForce(loads, tire);

if isCompositePowertrain(powertrain)
    powertrain = validate_powertrain_config(powertrain);
    if ~powertrain.enabled
        drive.Fx_drive_max_N = sum(availableFx_N);
        drive.limiter = "tire";
        drive.traction_limit_N = drive.Fx_drive_max_N;
        drive.torque_limit_N = inf;
        drive.power_limit_N = inf;
        return
    end

    drivenIndices = drivenWheelIndices(powertrain.layout);
    tractionLimit_N = sum(availableFx_N(drivenIndices));
    capability = evaluate_powertrain_constraints( ...
        v_mps, tire, powertrain, powertrain.battery.V_bus_assumed_V);
    powertrainForceLimit_N = capability.available_wheel_force_N;
    if powertrainForceLimit_N <= tractionLimit_N
        force_N = powertrainForceLimit_N;
        limiter = capability.limiter;
    else
        force_N = tractionLimit_N;
        limiter = "traction";
    end

    drive.Fx_drive_max_N = force_N;
    drive.limiter = limiter;
    drive.traction_limit_N = tractionLimit_N;
    drive.powertrain_force_limit_N = powertrainForceLimit_N;
    drive.driven_wheel_indices = drivenIndices;
    drive.powertrain = capability;
    return
end

powertrain = fillLegacyDefaults(powertrain, options);

if ~powertrain.enabled
    drive.Fx_drive_max_N = sum(availableFx_N);
    drive.limiter = "tire";
    drive.traction_limit_N = drive.Fx_drive_max_N;
    drive.torque_limit_N = inf;
    drive.power_limit_N = inf;
    return
end

drivenIndices = drivenWheelIndices(powertrain.layout);
tractionLimit_N = sum(availableFx_N(drivenIndices));
torqueLimit_N = powertrain.max_total_wheel_torque_Nm ...
    / tire.rolling_radius_m;
wheelPower_W = powertrain.max_power_W * powertrain.drive_efficiency;
if isinf(wheelPower_W)
    powerLimit_N = inf;
else
    powerLimit_N = calc_wheel_force_from_power( ...
        wheelPower_W, v_mps, options.min_query_speed_mps);
end

if v_mps >= powertrain.max_speed_mps
    force_N = 0;
    limiter = "top_speed";
else
    [force_N, index] = min([tractionLimit_N, torqueLimit_N, powerLimit_N]);
    labels = ["traction", "torque", "power"];
    limiter = labels(index);
end

drive.Fx_drive_max_N = force_N;
drive.limiter = limiter;
drive.traction_limit_N = tractionLimit_N;
drive.torque_limit_N = torqueLimit_N;
drive.power_limit_N = powerLimit_N;
drive.driven_wheel_indices = drivenIndices;
end

function availableFx_N = getAvailableLongitudinalForce(loads, tire)
if isfield(loads, "Fx_available_N")
    availableFx_N = loads.Fx_available_N(:);
elseif isfield(loads, "Fz_vector_N")
    env = tire_envelope(loads.Fz_vector_N, zeros(4, 1), tire);
    availableFx_N = env.Fx_max_N(:);
else
    error("QSSLTS:DriveLoads", ...
        "loads must contain Fx_available_N or Fz_vector_N.");
end
if numel(availableFx_N) ~= 4 || any(availableFx_N < 0) ...
        || any(~isfinite(availableFx_N))
    error("QSSLTS:DriveLoads", ...
        "Available longitudinal force must contain four finite values.");
end
end

function indices = drivenWheelIndices(layout)
switch upper(string(layout))
    case "FWD"
        indices = [1, 2];
    case "RWD"
        indices = [3, 4];
    case "AWD"
        indices = 1:4;
    otherwise
        error("QSSLTS:DrivetrainLayout", ...
            "Unsupported drivetrain layout: %s", string(layout));
end
end

function model = fillLegacyDefaults(model, options)
if ~isfield(model, "enabled"), model.enabled = false; end
if ~isfield(model, "layout"), model.layout = "AWD"; end
if ~isfield(model, "max_total_wheel_torque_Nm")
    if isfield(model, "max_wheel_torque_Nm")
        model.max_total_wheel_torque_Nm = model.max_wheel_torque_Nm;
    else
        model.max_total_wheel_torque_Nm = inf;
    end
end
if ~isfield(model, "max_power_W"), model.max_power_W = inf; end
if ~isfield(model, "max_speed_mps")
    model.max_speed_mps = options.v_max_mps;
end
if ~isfield(model, "drive_efficiency"), model.drive_efficiency = 1; end
if model.max_total_wheel_torque_Nm < 0 || model.max_power_W < 0 ...
        || model.max_speed_mps < 0 || model.drive_efficiency < 0 ...
        || model.drive_efficiency > 1
    error("QSSLTS:PowertrainParameters", ...
        "Powertrain limits/efficiency are outside valid bounds.");
end
end

function result = isCompositePowertrain(model)
markers = ["motor_count", "gear_ratio", "drivetrain_efficiency", ...
    "motor", "battery", "inverter", "rules"];
result = any(isfield(model, cellstr(markers)));
end
