function [projected, report] = project_ggv_to_hard_limits( ...
        candidate, hardReference)
%PROJECT_GGV_TO_HARD_LIMITS Clip an empirical GGV to theoretical limits.
%   Both maps must use the same speed/lateral grid. No implicit resampling is
%   performed because that would obscure which hard constraint was applied.

arguments
    candidate (1,1) struct
    hardReference (1,1) struct
end

validateProjectionInputs(candidate, hardReference);
tolerance_g = 1e-12;
baseFeasible = logical(candidate.feasible) ...
    & logical(hardReference.feasible);
feasibleDomainClip = logical(candidate.feasible) ...
    & ~logical(hardReference.feasible);

positiveLateralClip = candidate.ay_limit_pos_g ...
    > hardReference.ay_limit_pos_g + tolerance_g;
negativeLateralClip = candidate.ay_limit_neg_g ...
    < hardReference.ay_limit_neg_g - tolerance_g;
projectedPositiveLimit_g = min( ...
    candidate.ay_limit_pos_g, hardReference.ay_limit_pos_g);
projectedNegativeLimit_g = max( ...
    candidate.ay_limit_neg_g, hardReference.ay_limit_neg_g);
lateralFeasible = candidate.ay_g <= projectedPositiveLimit_g + tolerance_g ...
    & candidate.ay_g >= projectedNegativeLimit_g - tolerance_g;
projectedFeasible = baseFeasible & lateralFeasible;

accelClip = projectedFeasible ...
    & candidate.ax_max_g > hardReference.ax_max_g + tolerance_g;
brakeClip = projectedFeasible ...
    & candidate.ax_min_g < hardReference.ax_min_g - tolerance_g;

projected = candidate;
projected.ax_max_g = min(candidate.ax_max_g, hardReference.ax_max_g);
projected.ax_min_g = max(candidate.ax_min_g, hardReference.ax_min_g);
projected.ay_limit_pos_g = projectedPositiveLimit_g;
projected.ay_limit_neg_g = projectedNegativeLimit_g;
projected.feasible = projectedFeasible;

projected.accel_limiter(accelClip) = ...
    hardReference.accel_limiter(accelClip);
projected.brake_limiter(brakeClip) = ...
    hardReference.brake_limiter(brakeClip);
projected.accel_limiter(~projectedFeasible) = "lateral_infeasible";
projected.brake_limiter(~projectedFeasible) = "lateral_infeasible";
accelBoundaryClip = false(numel(candidate.v_mps), 1);
brakeBoundaryClip = false(numel(candidate.v_mps), 1);
accelBoundaryExcess_g = zeros(0, 1);
brakeBoundaryExcess_g = zeros(0, 1);
if isfield(candidate, "ax_max_lateral_boundary_g") ...
        && isfield(hardReference, "ax_max_lateral_boundary_g")
    accelBoundaryClip = candidate.ax_max_lateral_boundary_g(:) ...
        > hardReference.ax_max_lateral_boundary_g(:) + tolerance_g;
    accelBoundaryExcess_g = ...
        candidate.ax_max_lateral_boundary_g(accelBoundaryClip) ...
        - hardReference.ax_max_lateral_boundary_g(accelBoundaryClip);
    projected.ax_max_lateral_boundary_g = min( ...
        candidate.ax_max_lateral_boundary_g, ...
        hardReference.ax_max_lateral_boundary_g);
end
if isfield(candidate, "ax_min_lateral_boundary_g") ...
        && isfield(hardReference, "ax_min_lateral_boundary_g")
    brakeBoundaryClip = candidate.ax_min_lateral_boundary_g(:) ...
        < hardReference.ax_min_lateral_boundary_g(:) - tolerance_g;
    brakeBoundaryExcess_g = ...
        hardReference.ax_min_lateral_boundary_g(brakeBoundaryClip) ...
        - candidate.ax_min_lateral_boundary_g(brakeBoundaryClip);
    projected.ax_min_lateral_boundary_g = max( ...
        candidate.ax_min_lateral_boundary_g, ...
        hardReference.ax_min_lateral_boundary_g);
end

report.accel_clip_mask = accelClip;
report.brake_clip_mask = brakeClip;
report.lateral_positive_clip_mask = positiveLateralClip;
report.lateral_negative_clip_mask = negativeLateralClip;
report.feasible_domain_clip_mask = feasibleDomainClip;
report.accel_boundary_clip_mask = accelBoundaryClip;
report.brake_boundary_clip_mask = brakeBoundaryClip;
report.accel_clip_count = nnz(accelClip);
report.brake_clip_count = nnz(brakeClip);
report.lateral_clip_count = nnz(positiveLateralClip) ...
    + nnz(negativeLateralClip);
report.feasible_domain_clip_count = nnz(feasibleDomainClip);
report.accel_boundary_clip_count = nnz(accelBoundaryClip);
report.brake_boundary_clip_count = nnz(brakeBoundaryClip);
report.max_accel_excess_g = maximumOrZero( ...
    candidate.ax_max_g(accelClip) - hardReference.ax_max_g(accelClip));
report.max_brake_excess_g = maximumOrZero( ...
    hardReference.ax_min_g(brakeClip) - candidate.ax_min_g(brakeClip));
report.max_lateral_excess_g = maximumOrZero([ ...
    candidate.ay_limit_pos_g(positiveLateralClip) ...
        - hardReference.ay_limit_pos_g(positiveLateralClip); ...
    hardReference.ay_limit_neg_g(negativeLateralClip) ...
        - candidate.ay_limit_neg_g(negativeLateralClip)]);
report.max_accel_boundary_excess_g = maximumOrZero( ...
    accelBoundaryExcess_g);
report.max_brake_boundary_excess_g = maximumOrZero( ...
    brakeBoundaryExcess_g);
report.candidate_source = sourceLabel(candidate);
report.hard_reference_source = sourceLabel(hardReference);
report.requires_component_recalibration = ...
    report.accel_clip_count > 0 || report.brake_clip_count > 0 ...
    || report.lateral_clip_count > 0 ...
    || report.feasible_domain_clip_count > 0 ...
    || report.accel_boundary_clip_count > 0 ...
    || report.brake_boundary_clip_count > 0;
report.reason_codes = projectionReasonCodes(report);

identity = sameCapability(candidate, hardReference);
if report.requires_component_recalibration
    report.status = "clipped";
elseif identity
    report.status = "identity";
else
    report.status = "conservative";
end
projected.hard_limit_projection_report = report;
end

function validateProjectionInputs(candidate, hardReference)
required = ["v_mps", "ay_g", "ax_max_g", "ax_min_g", "feasible", ...
    "ay_limit_pos_g", "ay_limit_neg_g", "accel_limiter", ...
    "brake_limiter", "gravity_mps2"];
if ~all(isfield(candidate, cellstr(required))) ...
        || ~all(isfield(hardReference, cellstr(required)))
    error("QSSLTS:GGVProjectionStructure", ...
        "Candidate and hard-reference GGV fields are incomplete.");
end
expectedSize = [numel(hardReference.v_mps), numel(hardReference.ay_g)];
validShape = isequal(size(candidate.ax_max_g), expectedSize) ...
    && isequal(size(candidate.ax_min_g), expectedSize) ...
    && isequal(size(candidate.feasible), expectedSize) ...
    && isequal(size(hardReference.ax_max_g), expectedSize) ...
    && isequal(size(hardReference.ax_min_g), expectedSize) ...
    && isequal(size(hardReference.feasible), expectedSize) ...
    && isequal(size(candidate.accel_limiter), expectedSize) ...
    && isequal(size(candidate.brake_limiter), expectedSize) ...
    && numel(candidate.ay_limit_pos_g) == expectedSize(1) ...
    && numel(candidate.ay_limit_neg_g) == expectedSize(1);
if ~validShape
    error("QSSLTS:GGVProjectionStructure", ...
        "Candidate and hard-reference GGV shapes are incompatible.");
end
sameGrid = isequal(candidate.v_mps(:), hardReference.v_mps(:)) ...
    && isequal(candidate.ay_g(:), hardReference.ay_g(:)) ...
    && isequal(candidate.gravity_mps2, hardReference.gravity_mps2);
if ~sameGrid
    error("QSSLTS:GGVProjectionGrid", ...
        "Candidate and hard-reference GGV grids/gravity must match exactly.");
end
end

function reasonCodes = projectionReasonCodes(report)
reasonCodes = strings(0, 1);
if report.accel_clip_count > 0
    reasonCodes(end + 1, 1) = "ggv_accel_hard_limit_clipped";
end
if report.brake_clip_count > 0
    reasonCodes(end + 1, 1) = "ggv_brake_hard_limit_clipped";
end
if report.lateral_clip_count > 0
    reasonCodes(end + 1, 1) = "ggv_lateral_hard_limit_clipped";
end
if report.feasible_domain_clip_count > 0
    reasonCodes(end + 1, 1) = ...
        "ggv_feasible_domain_hard_limit_clipped";
end
if report.accel_boundary_clip_count > 0
    reasonCodes(end + 1, 1) = ...
        "ggv_accel_boundary_hard_limit_clipped";
end
if report.brake_boundary_clip_count > 0
    reasonCodes(end + 1, 1) = ...
        "ggv_brake_boundary_hard_limit_clipped";
end
if ~isempty(reasonCodes)
    reasonCodes(end + 1, 1) = "component_recalibration_required";
end
end

function result = sameCapability(left, right)
result = isequaln(left.ax_max_g, right.ax_max_g) ...
    && isequaln(left.ax_min_g, right.ax_min_g) ...
    && isequaln(left.feasible, right.feasible) ...
    && isequaln(left.ay_limit_pos_g, right.ay_limit_pos_g) ...
    && isequaln(left.ay_limit_neg_g, right.ay_limit_neg_g) ...
    && optionalFieldEqual(left, right, "ax_max_lateral_boundary_g") ...
    && optionalFieldEqual(left, right, "ax_min_lateral_boundary_g");
end

function result = optionalFieldEqual(left, right, name)
leftHasField = isfield(left, name);
rightHasField = isfield(right, name);
if leftHasField && rightHasField
    result = isequaln(left.(name), right.(name));
else
    result = leftHasField == rightHasField;
end
end

function value = maximumOrZero(values)
if isempty(values)
    value = 0;
else
    value = max(values, [], "all");
end
end

function label = sourceLabel(ggv)
if isfield(ggv, "source") && isscalar(string(ggv.source))
    label = string(ggv.source);
else
    label = "unknown";
end
end
