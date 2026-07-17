function drive = calc_drive_limit(v_mps, loads, tire, powertrain, options)
%CALC_DRIVE_LIMIT Calculate maximum positive tractive force [N].
%   An empty struct or exact enabled=false sentinel selects the ideal
%   all-wheel tire limit used by no-powertrain studies.

arguments
    v_mps (1,1) double {mustBeFinite, mustBeNonnegative}
    loads (1,1) struct
    tire (1,1) struct
    powertrain (1,1) struct
    options (1,1) struct = struct() %#ok<INUSA>
end

availableFx_N = getAvailableLongitudinalForce(loads, tire);

if isNoPowertrainSentinel(powertrain)
    drive = idealTireDrive(availableFx_N);
    return
end

if isCompositePowertrain(powertrain)
    powertrain = validate_powertrain_config(powertrain);
    if ~powertrain.enabled
        drive = idealTireDrive(availableFx_N);
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

error("QSSLTS:PowertrainConfig", ...
    "Powertrain must be a complete composite config or disabled sentinel.");
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

function drive = idealTireDrive(availableFx_N)
drive.Fx_drive_max_N = sum(availableFx_N);
drive.limiter = "tire";
drive.traction_limit_N = drive.Fx_drive_max_N;
end

function result = isNoPowertrainSentinel(model)
names = string(fieldnames(model));
isEmpty = isempty(names);
isMinimalDisabled = isequal(names, "enabled") ...
    && (islogical(model.enabled) || isnumeric(model.enabled)) ...
    && isscalar(model.enabled) && ismember(double(model.enabled), 0);
result = isEmpty || isMinimalDisabled;
end

function result = isCompositePowertrain(model)
markers = ["motor_count", "gear_ratio", "drivetrain_efficiency", ...
    "motor", "battery", "inverter", "rules"];
result = any(isfield(model, cellstr(markers)));
end
