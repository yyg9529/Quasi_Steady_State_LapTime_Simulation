function [vFwd_mps, limiter, info] = forward_pass(track, ggv, vLat_mps, options)
%FORWARD_PASS Enforce longitudinal acceleration reachability in space.
%   Closed tracks include the N-to-1 segment and are iterated to a periodic
%   fixed point. Speed uses m/s, distance m, and acceleration m/s^2.

arguments
    track (1,1) struct
    ggv (1,1) struct
    vLat_mps (:,1) double
    options (1,1) struct = struct()
end

options = default_qss_options(options);
nPoint = numel(track.s_m);
validateTrackProfile(track, vLat_mps, nPoint);

vFwd_mps = initialProfile(vLat_mps, options);
limiter = repmat("lateral_or_speed", nPoint, 1);
isClosed = isfield(track, "is_closed") && track.is_closed;

if ~isClosed
    vFwd_mps(1) = min(vFwd_mps(1), options.start_speed_mps);
    for iPoint = 1:nPoint-1
        [vFwd_mps, limiter] = enforceSegment( ...
            iPoint, iPoint + 1, vFwd_mps, limiter, track, ggv, options);
    end
    info.converged = true;
    info.iterations = 1;
    info.max_change_mps = 0;
    return
end

maxChange_mps = inf;
for iIteration = 1:options.solver_max_iterations
    before_mps = vFwd_mps;
    for iPoint = 1:nPoint
        nextPoint = mod(iPoint, nPoint) + 1;
        [vFwd_mps, limiter] = enforceSegment( ...
            iPoint, nextPoint, vFwd_mps, limiter, track, ggv, options);
    end
    maxChange_mps = max(abs(vFwd_mps - before_mps));
    if maxChange_mps <= options.solver_tolerance_mps
        break
    end
end
info.converged = maxChange_mps <= options.solver_tolerance_mps;
info.iterations = iIteration;
info.max_change_mps = maxChange_mps;
end

function [profile_mps, limiter] = enforceSegment( ...
        iPoint, nextPoint, profile_mps, limiter, track, ggv, options)
speed_mps = profile_mps(iPoint);
ay_g = speed_mps^2 * track.kappa_1pm(iPoint) / options.gravity_mps2;
cap = interp_ggv(ggv, speed_mps, ay_g, options);
if cap.is_feasible
    reachableSquared = speed_mps^2 + ...
        2 * cap.ax_max_mps2 * track.ds_m(iPoint);
    explicitReachable_mps = sqrt(max(0, reachableSquared));
    candidate_mps = min(profile_mps(nextPoint), explicitReachable_mps);
    [isReachable, segmentLimiter] = segmentIsReachable( ...
        speed_mps, candidate_mps, track.ds_m(iPoint), ...
        track.kappa_1pm(iPoint), ggv, options);
    if isReachable
        reachable_mps = candidate_mps;
    else
        [reachable_mps, segmentLimiter] = solveReachableSpeed( ...
            speed_mps, candidate_mps, track.ds_m(iPoint), ...
            track.kappa_1pm(iPoint), ggv, options);
    end
else
    reachable_mps = 0;
    segmentLimiter = cap.accel_limiter;
end

if reachable_mps < profile_mps(nextPoint)
    profile_mps(nextPoint) = reachable_mps;
    limiter(nextPoint) = segmentLimiter;
end
end

function [reachable_mps, limiter] = solveReachableSpeed( ...
        startSpeed_mps, upperSpeed_mps, ds_m, kappa_1pm, ggv, options)
candidate_mps = upperSpeed_mps;
lowerSpeed_mps = 0;
upperInfeasible_mps = upperSpeed_mps;
for iIteration = 1:min(options.solver_max_iterations, 20)
    [isReachable, limiter, limitingAx_mps2] = segmentIsReachable( ...
        startSpeed_mps, candidate_mps, ds_m, kappa_1pm, ggv, options);
    if isReachable
        lowerSpeed_mps = candidate_mps;
        break
    end
    upperInfeasible_mps = candidate_mps;
    if ~isfinite(limitingAx_mps2)
        break
    end
    revisedSquared = startSpeed_mps^2 ...
        + 2 * limitingAx_mps2 * ds_m;
    revised_mps = min(candidate_mps, sqrt(max(0, revisedSquared)));
    if candidate_mps - revised_mps <= options.solver_tolerance_mps
        break
    end
    candidate_mps = revised_mps;
end

for iIteration = 1:options.solver_max_iterations
    candidate_mps = 0.5 * (lowerSpeed_mps + upperInfeasible_mps);
    [isReachable, candidateLimiter] = segmentIsReachable( ...
        startSpeed_mps, candidate_mps, ds_m, kappa_1pm, ggv, options);
    if isReachable
        lowerSpeed_mps = candidate_mps;
    else
        upperInfeasible_mps = candidate_mps;
        limiter = candidateLimiter;
    end
    if upperInfeasible_mps - lowerSpeed_mps ...
            <= options.solver_tolerance_mps
        break
    end
end
reachable_mps = lowerSpeed_mps;
end

function [isReachable, limiter, limitingAx_mps2] = segmentIsReachable( ...
        startSpeed_mps, endSpeed_mps, ds_m, kappa_1pm, ggv, options)
representativeSpeeds_mps = [startSpeed_mps; ...
    0.5 * (startSpeed_mps + endSpeed_mps)];
nState = numel(representativeSpeeds_mps);
availableAx_mps2 = nan(nState, 1);
limiters = strings(nState, 1);
isFeasible = false(nState, 1);
for iState = 1:nState
    speed_mps = representativeSpeeds_mps(iState);
    ay_g = speed_mps^2 * kappa_1pm / options.gravity_mps2;
    cap = interp_ggv(ggv, speed_mps, ay_g, options);
    availableAx_mps2(iState) = cap.ax_max_mps2;
    limiters(iState) = cap.accel_limiter;
    isFeasible(iState) = cap.is_feasible;
end
[limitingAx_mps2, limitingIndex] = min(availableAx_mps2);
if isfield(options, "segment_power_constraint")
    [powerAxLimit_mps2, powerLimiter] = segmentPowerAxLimit( ...
        representativeSpeeds_mps(end), options.segment_power_constraint);
    if powerAxLimit_mps2 < limitingAx_mps2
        limitingAx_mps2 = powerAxLimit_mps2;
        limiter = powerLimiter;
    else
        limiter = limiters(limitingIndex);
    end
else
    limiter = limiters(limitingIndex);
end
requiredAx_mps2 = (endSpeed_mps^2 - startSpeed_mps^2) / (2 * ds_m);
isReachable = all(isFeasible) ...
    && requiredAx_mps2 <= limitingAx_mps2 ...
    + options.accel_tolerance_mps2;
end

function [axLimit_mps2, limiter] = segmentPowerAxLimit( ...
        speed_mps, constraint)
powertrain = constraint.powertrain;
powerCap = calc_ts_power_cap( ...
    powertrain.battery.V_bus_assumed_V, powertrain);
efficiency = powertrain.drivetrain_efficiency ...
    * powertrain.inverter.eta_const * powertrain.motor.eta_const;
mechanicalPowerCap_W = max(powerCap.tsac_power_cap_W ...
    - powertrain.battery.P_ts_aux_W, 0) * efficiency;
if speed_mps <= 0
    axLimit_mps2 = inf;
else
    aeroForce = calc_aero_forces(speed_mps, constraint.aero);
    wheelForceCap_N = mechanicalPowerCap_W / speed_mps;
    axLimit_mps2 = (wheelForceCap_N - aeroForce.drag_N) ...
        / constraint.vehicle_mass_kg;
end
limiter = powerCap.limiter;
end

function profile_mps = initialProfile(vLat_mps, options)
profile_mps = vLat_mps;
if isfield(options, "initial_profile_mps") && ...
        ~isempty(options.initial_profile_mps)
    candidate_mps = options.initial_profile_mps(:);
    if numel(candidate_mps) ~= numel(vLat_mps)
        error("QSSLTS:InitialProfileSize", ...
            "initial_profile_mps must match the track node count.");
    end
    profile_mps = min(vLat_mps, candidate_mps);
end
if any(~isfinite(profile_mps)) || any(profile_mps < 0)
    error("QSSLTS:InitialProfile", ...
        "Initial speed profile must be finite and nonnegative.");
end
end

function validateTrackProfile(track, profile, nPoint)
if numel(track.kappa_1pm) ~= nPoint || numel(profile) ~= nPoint
    error("QSSLTS:TrackShape", ...
        "Track node fields and speed profile must have equal lengths.");
end
isClosed = isfield(track, "is_closed") && track.is_closed;
expectedSegments = nPoint - 1 + double(isClosed);
if numel(track.ds_m) ~= expectedSegments || any(track.ds_m <= 0)
    error("QSSLTS:TrackSegments", ...
        "track.ds_m has the wrong length or contains nonpositive values.");
end
end
