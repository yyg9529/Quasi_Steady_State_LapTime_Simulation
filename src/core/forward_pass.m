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
    reachable_mps = sqrt(max(0, reachableSquared));
else
    reachable_mps = 0;
end

if reachable_mps < profile_mps(nextPoint)
    profile_mps(nextPoint) = reachable_mps;
    limiter(nextPoint) = cap.accel_limiter;
end
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
