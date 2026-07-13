function [vFinal_mps, limiter, info] = backward_pass( ...
        track, ggv, vInput_mps, vLat_mps, options)
%BACKWARD_PASS Enforce braking reachability in reverse spatial order.
%   Closed tracks include the N-to-1 segment and are iterated to a periodic
%   fixed point. ax_min queried from GGV is negative [m/s^2].

arguments
    track (1,1) struct
    ggv (1,1) struct
    vInput_mps (:,1) double
    vLat_mps (:,1) double
    options (1,1) struct = struct()
end

options = default_qss_options(options);
nPoint = numel(track.s_m);
if numel(vInput_mps) ~= nPoint || numel(vLat_mps) ~= nPoint
    error("QSSLTS:SpeedProfileSize", ...
        "Speed profiles must match the track node count.");
end

vFinal_mps = min(vInput_mps, vLat_mps);
limiter = repmat("input_profile", nPoint, 1);
isClosed = isfield(track, "is_closed") && track.is_closed;

if ~isClosed
    if isfinite(options.finish_speed_mps)
        vFinal_mps(end) = min(vFinal_mps(end), options.finish_speed_mps);
    end
    for iPoint = nPoint-1:-1:1
        [vFinal_mps, limiter] = enforceBrakingSegment( ...
            iPoint, iPoint + 1, vFinal_mps, limiter, track, ggv, options);
    end
    info.converged = true;
    info.iterations = 1;
    info.max_change_mps = 0;
    return
end

maxChange_mps = inf;
for iIteration = 1:options.solver_max_iterations
    before_mps = vFinal_mps;
    for iPoint = nPoint:-1:1
        nextPoint = mod(iPoint, nPoint) + 1;
        [vFinal_mps, limiter] = enforceBrakingSegment( ...
            iPoint, nextPoint, vFinal_mps, limiter, track, ggv, options);
    end
    maxChange_mps = max(abs(vFinal_mps - before_mps));
    if maxChange_mps <= options.solver_tolerance_mps
        break
    end
end
info.converged = maxChange_mps <= options.solver_tolerance_mps;
info.iterations = iIteration;
info.max_change_mps = maxChange_mps;
end

function [profile_mps, limiter] = enforceBrakingSegment( ...
        iPoint, nextPoint, profile_mps, limiter, track, ggv, options)
speed_mps = profile_mps(iPoint);
ay_g = speed_mps^2 * track.kappa_1pm(iPoint) / options.gravity_mps2;
cap = interp_ggv(ggv, speed_mps, ay_g, options);
if cap.is_feasible
    allowableSquared = profile_mps(nextPoint)^2 ...
        - 2 * cap.ax_min_mps2 * track.ds_m(iPoint);
    allowable_mps = sqrt(max(0, allowableSquared));
else
    allowable_mps = 0;
end

if allowable_mps < profile_mps(iPoint)
    profile_mps(iPoint) = allowable_mps;
    limiter(iPoint) = cap.brake_limiter;
end
end
