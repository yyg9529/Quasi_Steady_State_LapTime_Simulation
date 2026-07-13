function ggv = generate_model_ggv(vehicle, tire, aero, powertrain, brake, options)
%GENERATE_MODEL_GGV Generate a V0.1 point-mass constant-mu GGV map.
%   Acceleration fields are stored in g; speed uses m/s. Aerodynamic forces
%   and axle load transfer are intentionally excluded from V0.1.

arguments
    vehicle (1,1) struct
    tire (1,1) struct
    aero (1,1) struct = struct()
    powertrain (1,1) struct = struct()
    brake (1,1) struct = struct()
    options (1,1) struct = struct()
end

options = default_qss_options(options);
validateBaselineInputs(vehicle, tire);

vGrid_mps = options.v_grid_mps(:);
ayGrid_g = options.ay_grid_g(:).';
if any(diff(vGrid_mps) <= 0) || vGrid_mps(1) < 0
    error("QSSLTS:GGVSpeedGrid", ...
        "options.v_grid_mps must be nonnegative and strictly increasing.");
end
if any(diff(ayGrid_g) <= 0)
    error("QSSLTS:GGVLateralGrid", ...
        "options.ay_grid_g must be strictly increasing.");
end

mass_kg = vehicle.mass.total_kg;
g_mps2 = options.gravity_mps2;
nSpeed = numel(vGrid_mps);
nLateral = numel(ayGrid_g);

axMax_g = zeros(nSpeed, nLateral);
axMin_g = zeros(nSpeed, nLateral);
feasible = false(nSpeed, nLateral);
accelLimiter = strings(nSpeed, nLateral);
brakeLimiter = strings(nSpeed, nLateral);

powertrain = fillPowertrainDefaults(powertrain, options);
brake = fillBrakeDefaults(brake);

for iSpeed = 1:nSpeed
    speed_mps = vGrid_mps(iSpeed);
    for iLateral = 1:nLateral
        ay_g = ayGrid_g(iLateral);
        lateralRatio = abs(ay_g) / tire.mu_y;
        if lateralRatio > 1 + 10 * eps
            accelLimiter(iSpeed, iLateral) = "lateral_infeasible";
            brakeLimiter(iSpeed, iLateral) = "lateral_infeasible";
            continue
        end

        feasible(iSpeed, iLateral) = true;
        longitudinalScale = max(0, 1 - lateralRatio^tire.combined_n) ...
            ^ (1 / tire.combined_n);
        tireForce_N = tire.mu_x * mass_kg * g_mps2 * longitudinalScale;

        [driveForce_N, driveLimiter] = pointMassDriveForce( ...
            speed_mps, tireForce_N, tire, powertrain, options);
        [brakeForce_N, thisBrakeLimiter] = pointMassBrakeForce( ...
            tireForce_N, mass_kg, brake, g_mps2);

        axMax_g(iSpeed, iLateral) = driveForce_N / mass_kg / g_mps2;
        axMin_g(iSpeed, iLateral) = -brakeForce_N / mass_kg / g_mps2;
        accelLimiter(iSpeed, iLateral) = driveLimiter;
        brakeLimiter(iSpeed, iLateral) = thisBrakeLimiter;
    end
end

ggv.v_mps = vGrid_mps;
ggv.ay_g = ayGrid_g;
ggv.ax_max_g = axMax_g;
ggv.ax_min_g = axMin_g;
ggv.feasible = feasible;
ggv.ay_limit_pos_g = tire.mu_y * ones(nSpeed, 1);
ggv.ay_limit_neg_g = -tire.mu_y * ones(nSpeed, 1);
ggv.accel_limiter = accelLimiter;
ggv.brake_limiter = brakeLimiter;
ggv.source = "model_constant_mu_v0.1";
ggv.notes = "Point-mass superellipse; no aero or load transfer";
ggv.gravity_mps2 = g_mps2;
ggv.options = options;
ggv.aero_input_ignored = ~isempty(fieldnames(aero));
end

function validateBaselineInputs(vehicle, tire)
requiredVehicle = ["total_kg", "front_static_frac", "cg_height_m"];
if ~isfield(vehicle, "mass") || ...
        ~all(isfield(vehicle.mass, cellstr(requiredVehicle)))
    error("QSSLTS:VehicleParameters", ...
        "vehicle.mass is missing required baseline fields.");
end
if vehicle.mass.total_kg <= 0
    error("QSSLTS:VehicleMass", "vehicle mass must be positive.");
end
if string(tire.model_type) ~= "constant_mu" || ...
        tire.mu_x < 0 || tire.mu_y <= 0 || tire.combined_n < 1
    error("QSSLTS:TireParameters", ...
        "A valid constant-mu tire model is required for V0.1.");
end
end

function model = fillPowertrainDefaults(model, options)
if ~isfield(model, "enabled"), model.enabled = false; end
if ~isfield(model, "max_power_W"), model.max_power_W = inf; end
if ~isfield(model, "max_wheel_torque_Nm"), model.max_wheel_torque_Nm = inf; end
if ~isfield(model, "max_speed_mps"), model.max_speed_mps = options.v_max_mps; end
if ~isfield(model, "drive_efficiency"), model.drive_efficiency = 1; end
end

function model = fillBrakeDefaults(model)
if ~isfield(model, "enabled"), model.enabled = true; end
if ~isfield(model, "max_decel_g_mechanical")
    model.max_decel_g_mechanical = inf;
end
end

function [force_N, limiter] = pointMassDriveForce( ...
        speed_mps, tireForce_N, tire, powertrain, options)
if ~powertrain.enabled
    force_N = tireForce_N;
    limiter = "tire";
    return
end

if speed_mps >= powertrain.max_speed_mps
    force_N = 0;
    limiter = "top_speed";
    return
end

torqueForce_N = powertrain.max_wheel_torque_Nm ...
    / tire.rolling_radius_m;
powerForce_N = powertrain.max_power_W * powertrain.drive_efficiency ...
    / max(speed_mps, options.min_query_speed_mps);
[force_N, index] = min([tireForce_N, torqueForce_N, powerForce_N]);
labels = ["traction", "torque", "power"];
limiter = labels(index);
end

function [force_N, limiter] = pointMassBrakeForce( ...
        tireForce_N, mass_kg, brake, g_mps2)
if ~brake.enabled
    force_N = 0;
    limiter = "brake_disabled";
    return
end

mechanicalForce_N = brake.max_decel_g_mechanical * mass_kg * g_mps2;
[force_N, index] = min([tireForce_N, mechanicalForce_N]);
labels = ["brake_traction", "brake_mechanical"];
limiter = labels(index);
end
