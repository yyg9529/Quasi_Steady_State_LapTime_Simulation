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
options.backward_brake_island_flags = ...
    assessBrakeCapabilityIslands(ggv);
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
    info = addSegmentDiagnostics( ...
        info, vFinal_mps, track, ggv, options);
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
info = addSegmentDiagnostics(info, vFinal_mps, track, ggv, options);
end

function [profile_mps, limiter] = enforceBrakingSegment( ...
        iPoint, nextPoint, profile_mps, limiter, track, ggv, options)
entrySpeed_mps = profile_mps(iPoint);
exitSpeed_mps = profile_mps(nextPoint);
[isReachable, segmentLimiter] = segmentIsReachable( ...
    entrySpeed_mps, exitSpeed_mps, track.ds_m(iPoint), ...
    track.kappa_1pm(iPoint), ggv, options);
if isReachable
    allowable_mps = entrySpeed_mps;
else
    [allowable_mps, segmentLimiter] = solveReachableEntrySpeed( ...
        exitSpeed_mps, entrySpeed_mps, track.ds_m(iPoint), ...
        track.kappa_1pm(iPoint), ggv, options);
end

if allowable_mps < profile_mps(iPoint)
    profile_mps(iPoint) = allowable_mps;
    limiter(iPoint) = segmentLimiter;
end
end

function [entrySpeed_mps, limiter] = solveReachableEntrySpeed( ...
        exitSpeed_mps, upperSpeed_mps, ds_m, kappa_1pm, ggv, options)
[lowerSpeed_mps, upperInfeasible_mps, limiter] = ...
    findHighestReachableBracket(exitSpeed_mps, upperSpeed_mps, ...
    ds_m, kappa_1pm, ggv, options);

for iIteration = 1:options.solver_max_iterations
    candidate_mps = 0.5 * (lowerSpeed_mps + upperInfeasible_mps);
    [isReachable, candidateLimiter] = segmentIsReachable( ...
        candidate_mps, exitSpeed_mps, ds_m, kappa_1pm, ggv, options);
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
[entrySpeed_mps, limiter] = refineReachabilityBoundary( ...
    lowerSpeed_mps, upperInfeasible_mps, exitSpeed_mps, ds_m, ...
    kappa_1pm, ggv, options, limiter);
end

function [entrySpeed_mps, limiter] = refineReachabilityBoundary( ...
        lowerReachable_mps, upperInfeasible_mps, exitSpeed_mps, ...
        ds_m, kappa_1pm, ggv, options, limiter)
[~, ~, ~, lowerMargin_mps2] = segmentIsReachable( ...
    lowerReachable_mps, exitSpeed_mps, ds_m, kappa_1pm, ggv, options);
[~, ~, ~, upperMargin_mps2] = segmentIsReachable( ...
    upperInfeasible_mps, exitSpeed_mps, ds_m, kappa_1pm, ggv, options);

for iIteration = 1:4
    if ~isfinite(lowerMargin_mps2) || ~isfinite(upperMargin_mps2) ...
            || lowerMargin_mps2 < 0 || upperMargin_mps2 >= 0
        break
    end
    candidate_mps = (lowerReachable_mps * -upperMargin_mps2 ...
        + upperInfeasible_mps * lowerMargin_mps2) ...
        / (lowerMargin_mps2 - upperMargin_mps2);
    if candidate_mps <= lowerReachable_mps ...
            || candidate_mps >= upperInfeasible_mps
        break
    end
    [isReachable, candidateLimiter, limitingAxMin_mps2, ...
        candidateMargin_mps2] = segmentIsReachable( ...
        candidate_mps, exitSpeed_mps, ds_m, kappa_1pm, ggv, options);
    numericalMargin_mps2 = 16 * eps(max(1, ...
        abs(limitingAxMin_mps2)));
    if isReachable ...
            && abs(candidateMargin_mps2) <= numericalMargin_mps2
        entrySpeed_mps = candidate_mps;
        return
    elseif isReachable
        lowerReachable_mps = candidate_mps;
        lowerMargin_mps2 = candidateMargin_mps2;
    else
        upperInfeasible_mps = candidate_mps;
        upperMargin_mps2 = candidateMargin_mps2;
        limiter = candidateLimiter;
    end
end
entrySpeed_mps = lowerReachable_mps;
end

function [lowerReachable_mps, upperInfeasible_mps, limiter] = ...
        findHighestReachableBracket(exitSpeed_mps, upperSpeed_mps, ...
        ds_m, kappa_1pm, ggv, options)
entryKnots_mps = makeEntrySpeedKnots( ...
    exitSpeed_mps, upperSpeed_mps, kappa_1pm, ggv, options);
requiresCellOptimization = selectBrakeIslandFlag( ...
    options.backward_brake_island_flags, kappa_1pm);
upperInfeasible_mps = upperSpeed_mps;
[~, limiter] = segmentIsReachable(upperInfeasible_mps, ...
    exitSpeed_mps, ds_m, kappa_1pm, ggv, options);

subdivisionCount = 4;
for iInterval = numel(entryKnots_mps)-1:-1:1
    intervalEdges_mps = linspace(entryKnots_mps(iInterval), ...
        entryKnots_mps(iInterval + 1), subdivisionCount + 1);
    for iSubdivision = subdivisionCount:-1:1
        lowerEdge_mps = intervalEdges_mps(iSubdivision);
        upperEdge_mps = intervalEdges_mps(iSubdivision + 1);
        candidates_mps = [upperEdge_mps; lowerEdge_mps];
        if requiresCellOptimization
            optimum_mps = maximizeReachabilityMargin( ...
                lowerEdge_mps, upperEdge_mps, exitSpeed_mps, ds_m, ...
                kappa_1pm, ggv, options);
            midpoint_mps = 0.5 * (lowerEdge_mps + upperEdge_mps);
            candidates_mps = [candidates_mps; ...
                optimum_mps; midpoint_mps];
        end
        candidates_mps = sort(unique(candidates_mps), "descend");
        for iCandidate = 1:numel(candidates_mps)
            candidate_mps = candidates_mps(iCandidate);
            [isReachable, candidateLimiter] = segmentIsReachable( ...
                candidate_mps, exitSpeed_mps, ds_m, kappa_1pm, ...
                ggv, options);
            if isReachable
                lowerReachable_mps = candidate_mps;
                return
            end
            upperInfeasible_mps = candidate_mps;
            limiter = candidateLimiter;
        end
    end
end

lowerReachable_mps = min(exitSpeed_mps, upperSpeed_mps);
end

function entryKnots_mps = makeEntrySpeedKnots( ...
        exitSpeed_mps, upperSpeed_mps, kappa_1pm, ggv, options)
lowerSpeed_mps = min(exitSpeed_mps, upperSpeed_mps);
speedKnots_mps = ggv.v_mps(:);
midpointEntryKnots_mps = 2 * speedKnots_mps - exitSpeed_mps;
entryAyKnots_mps = zeros(0, 1);
midpointAyKnots_mps = zeros(0, 1);
if kappa_1pm ~= 0
    speedSquared_mps2 = ggv.ay_g(:) ...
        * options.gravity_mps2 / kappa_1pm;
    stateSpeedKnots_mps = sqrt(speedSquared_mps2( ...
        isfinite(speedSquared_mps2) & speedSquared_mps2 >= 0));
    entryAyKnots_mps = stateSpeedKnots_mps;
    midpointAyKnots_mps = 2 * stateSpeedKnots_mps - exitSpeed_mps;
end
entryKnots_mps = [lowerSpeed_mps; upperSpeed_mps; ...
    speedKnots_mps; midpointEntryKnots_mps; ...
    entryAyKnots_mps; midpointAyKnots_mps];
entryKnots_mps = entryKnots_mps(isfinite(entryKnots_mps) ...
    & entryKnots_mps >= lowerSpeed_mps ...
    & entryKnots_mps <= upperSpeed_mps);
entryKnots_mps = unique(entryKnots_mps);
end

function required = hasBrakeCapabilityIsland(ggv, kappa_1pm)
ayTolerance_g = 1e-12;
if kappa_1pm > 0
    relevantAy = ggv.ay_g(:).' >= -ayTolerance_g;
elseif kappa_1pm < 0
    relevantAy = ggv.ay_g(:).' <= ayTolerance_g;
else
    relevantAy = abs(ggv.ay_g(:).') <= ayTolerance_g;
end

required = false;
for iSpeed = 1:numel(ggv.v_mps)
    valid = relevantAy & logical(ggv.feasible(iSpeed, :));
    if containsInteriorValley(ggv.ax_min_g(iSpeed, valid))
        required = true;
        return
    end
end
for iAy = find(relevantAy)
    valid = logical(ggv.feasible(:, iAy));
    if containsInteriorValley(ggv.ax_min_g(valid, iAy))
        required = true;
        return
    end
end
end

function flags = assessBrakeCapabilityIslands(ggv)
flags.positive = hasBrakeCapabilityIsland(ggv, 1);
flags.negative = hasBrakeCapabilityIsland(ggv, -1);
flags.zero = hasBrakeCapabilityIsland(ggv, 0);
end

function required = selectBrakeIslandFlag(flags, kappa_1pm)
if kappa_1pm > 0
    required = flags.positive;
elseif kappa_1pm < 0
    required = flags.negative;
else
    required = flags.zero;
end
end

function result = containsInteriorValley(values)
values = values(:);
if numel(values) < 3 || any(~isfinite(values))
    result = false;
    return
end
tolerance = 1e-10 * max(1, max(abs(values)));
change = diff(values);
decreaseIndex = find(change < -tolerance);
increaseIndex = find(change > tolerance);
result = ~isempty(decreaseIndex) && ~isempty(increaseIndex) ...
    && any(decreaseIndex(:) < increaseIndex(:).', "all");
end

function optimum_mps = maximizeReachabilityMargin( ...
        lowerSpeed_mps, upperSpeed_mps, exitSpeed_mps, ds_m, ...
        kappa_1pm, ggv, options)
if upperSpeed_mps - lowerSpeed_mps <= options.solver_tolerance_mps
    optimum_mps = 0.5 * (lowerSpeed_mps + upperSpeed_mps);
    return
end
searchOptions = optimset("Display", "off", ...
    "TolX", max([options.solver_tolerance_mps, 1e-10, ...
    1e-6 * (upperSpeed_mps - lowerSpeed_mps)]));
optimum_mps = fminbnd(@(entrySpeed_mps) ...
    negativeReachabilityMargin(entrySpeed_mps, exitSpeed_mps, ...
    ds_m, kappa_1pm, ggv, options), ...
    lowerSpeed_mps, upperSpeed_mps, searchOptions);
end

function objective = negativeReachabilityMargin( ...
        entrySpeed_mps, exitSpeed_mps, ds_m, kappa_1pm, ggv, options)
[~, ~, ~, margin_mps2] = segmentIsReachable( ...
    entrySpeed_mps, exitSpeed_mps, ds_m, kappa_1pm, ggv, options);
if margin_mps2 == inf
    objective = -realmax;
elseif ~isfinite(margin_mps2)
    objective = realmax;
else
    objective = -margin_mps2;
end
end

function [isReachable, limiter, limitingAxMin_mps2, ...
        margin_mps2, samplesFeasible] = segmentIsReachable( ...
        entrySpeed_mps, exitSpeed_mps, ds_m, kappa_1pm, ggv, options)
requiredAx_mps2 = (exitSpeed_mps^2 - entrySpeed_mps^2) / (2 * ds_m);
if requiredAx_mps2 >= 0
    isReachable = true;
    limiter = "no_braking_required";
    limitingAxMin_mps2 = -inf;
    margin_mps2 = inf;
    samplesFeasible = true;
    return
end

representativeSpeeds_mps = [entrySpeed_mps; ...
    0.5 * (entrySpeed_mps + exitSpeed_mps)];
availableAxMin_mps2 = nan(2, 1);
limiters = strings(2, 1);
sampleFeasible = false(2, 1);
for iState = 1:2
    speed_mps = representativeSpeeds_mps(iState);
    ay_g = speed_mps^2 * kappa_1pm / options.gravity_mps2;
    cap = interp_ggv(ggv, speed_mps, ay_g, options);
    availableAxMin_mps2(iState) = cap.ax_min_mps2;
    limiters(iState) = cap.brake_limiter;
    sampleFeasible(iState) = cap.is_feasible;
end
samplesFeasible = all(sampleFeasible) ...
    && all(isfinite(availableAxMin_mps2));
if samplesFeasible
    [limitingAxMin_mps2, limitingIndex] = ...
        max(availableAxMin_mps2);
    limiter = limiters(limitingIndex);
    margin_mps2 = requiredAx_mps2 - limitingAxMin_mps2;
    numericalMargin_mps2 = 16 * eps(max(1, ...
        abs(limitingAxMin_mps2)));
    isReachable = margin_mps2 >= -numericalMargin_mps2;
else
    infeasibleIndex = find(~sampleFeasible ...
        | ~isfinite(availableAxMin_mps2), 1);
    limiter = limiters(infeasibleIndex);
    limitingAxMin_mps2 = NaN;
    margin_mps2 = -inf;
    isReachable = false;
end
end

function info = addSegmentDiagnostics(info, profile_mps, track, ggv, options)
isClosed = isfield(track, "is_closed") && track.is_closed;
segmentCount = numel(track.ds_m);
requiredAx_mps2 = zeros(segmentCount, 1);
limitingAxMin_mps2 = -inf(segmentCount, 1);
margin_mps2 = inf(segmentCount, 1);
isFeasible = true(segmentCount, 1);
violation = false(segmentCount, 1);
for iSegment = 1:segmentCount
    if isClosed
        nextPoint = mod(iSegment, numel(profile_mps)) + 1;
    else
        nextPoint = iSegment + 1;
    end
    entrySpeed_mps = profile_mps(iSegment);
    exitSpeed_mps = profile_mps(nextPoint);
    requiredAx_mps2(iSegment) = (exitSpeed_mps^2 - entrySpeed_mps^2) ...
        / (2 * track.ds_m(iSegment));
    [~, ~, limitingAxMin_mps2(iSegment), margin_mps2(iSegment), ...
        isFeasible(iSegment)] = segmentIsReachable( ...
        entrySpeed_mps, exitSpeed_mps, track.ds_m(iSegment), ...
        track.kappa_1pm(iSegment), ggv, options);
    brakingRequired = requiredAx_mps2(iSegment) < 0;
    violation(iSegment) = brakingRequired ...
        && (~isFeasible(iSegment) ...
        || margin_mps2(iSegment) < -options.accel_tolerance_mps2);
end

finiteExcess_mps2 = max(0, -margin_mps2(violation ...
    & isfinite(margin_mps2)));
if any(violation & ~isfinite(margin_mps2))
    maximumViolation_mps2 = inf;
elseif isempty(finiteExcess_mps2)
    maximumViolation_mps2 = 0;
else
    maximumViolation_mps2 = max(finiteExcess_mps2);
end
info.segment_required_ax_mps2 = requiredAx_mps2;
info.segment_limiting_ax_min_mps2 = limitingAxMin_mps2;
info.segment_brake_margin_mps2 = margin_mps2;
info.segment_is_feasible = isFeasible;
info.segment_capability_violation = violation;
info.violation_count = nnz(violation);
info.max_violation_mps2 = maximumViolation_mps2;
end
