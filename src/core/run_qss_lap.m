function result = run_qss_lap(track, vehicle, models, options)
%RUN_QSS_LAP Run the fixed-raceline quasi-steady-state lap simulation.
%   Track distances use m, speed m/s, acceleration m/s^2, and time s.

arguments
    track (1,1) struct
    vehicle (1,1) struct
    models (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
validateTrack(track);

if isfield(models, "ggv") && ~isempty(models.ggv)
    ggv = models.ggv;
else
    ggv = generate_model_ggv(vehicle, requireModel(models, "tire"), ...
        optionalModel(models, "aero"), optionalModel(models, "powertrain"), ...
        optionalModel(models, "brake"), options);
end
calibrationReport = struct();
if isfield(models, "ggv_real") && ~isempty(models.ggv_real)
    [ggv, calibrationReport] = calibrate_ggv( ...
        ggv, models.ggv_real, options);
end

[vLat_mps, lateralLimiter] = calc_lateral_speed_limit(track, ggv, options);
profile_mps = vLat_mps;
maxChange_mps = inf;

for iIteration = 1:options.solver_max_iterations
    previous_mps = profile_mps;
    passOptions = options;
    passOptions.initial_profile_mps = profile_mps;
    [vForward_mps, ~, forwardInfo] = forward_pass( ...
        track, ggv, vLat_mps, passOptions);
    [profile_mps, ~, backwardInfo] = backward_pass( ...
        track, ggv, vForward_mps, vLat_mps, passOptions);
    maxChange_mps = max(abs(profile_mps - previous_mps));
    if maxChange_mps <= options.solver_tolerance_mps
        break
    end
end

converged = maxChange_mps <= options.solver_tolerance_mps ...
    && forwardInfo.converged && backwardInfo.converged;
if ~converged
    warning("QSSLTS:SolverNotConverged", ...
        "QSS speed propagation did not converge in %d iterations.", ...
        options.solver_max_iterations);
end

[ax_mps2, ay_mps2] = reconstructAcceleration(track, profile_mps);
limiter = classifyLimiters(profile_mps, vLat_mps, ...
    ax_mps2, ay_mps2, ggv, lateralLimiter, options);
[lapTime_s, segmentTime_s, cumulativeTime_s] = ...
    integrate_lap_time(track, profile_mps);

result.lap_time_s = lapTime_s;
result.s_m = track.s_m(:);
result.v_mps = profile_mps;
result.ax_mps2 = ax_mps2;
result.ay_mps2 = ay_mps2;
result.limiter = limiter;
result.ggv_used = ggv;
result.options = options;
result.v_lateral_limit_mps = vLat_mps;
result.segment_time_s = segmentTime_s;
result.cumulative_time_s = cumulativeTime_s;
result.track = track;
result.calibration_report = calibrationReport;
result.solver.converged = converged;
result.solver.iterations = iIteration;
result.solver.max_change_mps = maxChange_mps;
end

function [ax_mps2, ay_mps2] = reconstructAcceleration(track, v_mps)
isClosed = isfield(track, "is_closed") && track.is_closed;
if isClosed
    nextSpeed_mps = circshift(v_mps, -1);
    ax_mps2 = (nextSpeed_mps.^2 - v_mps.^2) ./ (2 * track.ds_m(:));
else
    segmentAx_mps2 = (v_mps(2:end).^2 - v_mps(1:end-1).^2) ...
        ./ (2 * track.ds_m(:));
    ax_mps2 = [segmentAx_mps2; segmentAx_mps2(end)];
end
ay_mps2 = v_mps.^2 .* track.kappa_1pm(:);
end

function limiter = classifyLimiters(v_mps, vLat_mps, ...
        ax_mps2, ay_mps2, ggv, lateralLimiter, options)
nPoint = numel(v_mps);
limiter = strings(nPoint, 1);
speedCap_mps = min(options.v_max_mps, ggv.v_mps(end));
for iPoint = 1:nPoint
    isAtLateralLimit = abs(v_mps(iPoint) - vLat_mps(iPoint)) ...
        <= options.limiter_tolerance_mps ...
        && lateralLimiter(iPoint) == "lateral";
    if isAtLateralLimit
        limiter(iPoint) = "lateral";
        continue
    end

    ay_g = ay_mps2(iPoint) / options.gravity_mps2;
    cap = interp_ggv(ggv, v_mps(iPoint), ay_g, options);
    isAtAccelerationLimit = cap.is_feasible ...
        && abs(ax_mps2(iPoint) - cap.ax_max_mps2) ...
        <= options.accel_tolerance_mps2;
    isAtBrakingLimit = cap.is_feasible ...
        && abs(ax_mps2(iPoint) - cap.ax_min_mps2) ...
        <= options.accel_tolerance_mps2;
    if isAtAccelerationLimit
        limiter(iPoint) = cap.accel_limiter;
    elseif isAtBrakingLimit
        limiter(iPoint) = cap.brake_limiter;
    elseif v_mps(iPoint) >= speedCap_mps ...
            - options.limiter_tolerance_mps
        limiter(iPoint) = "top_speed";
    elseif abs(ax_mps2(iPoint)) <= options.accel_tolerance_mps2
        limiter(iPoint) = "coasting";
    else
        limiter(iPoint) = "unconstrained";
    end
end
end

function model = requireModel(models, name)
if ~isfield(models, name) || isempty(models.(name))
    error("QSSLTS:MissingModel", "models.%s is required.", name);
end
model = models.(name);
end

function model = optionalModel(models, name)
if isfield(models, name)
    model = models.(name);
else
    model = struct();
end
end

function validateTrack(track)
required = ["s_m", "ds_m", "kappa_1pm", "is_closed"];
if ~all(isfield(track, cellstr(required)))
    error("QSSLTS:TrackFields", "Track is missing required fields.");
end
if numel(track.s_m) ~= numel(track.kappa_1pm) ...
        || any(diff(track.s_m) <= 0) || any(track.ds_m <= 0)
    error("QSSLTS:TrackShape", "Track node and segment data are invalid.");
end
expectedSegments = numel(track.s_m) - 1 + double(track.is_closed);
if numel(track.ds_m) ~= expectedSegments
    error("QSSLTS:TrackSegments", ...
        "track.ds_m does not match the open/closed track contract.");
end
end
