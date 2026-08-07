function [axBoundary_g, ayBoundary_g, speedBoundary_mps] = ...
        prepare_ggv_boundary_surface(ggv, boundaryPointCount)
%PREPARE_GGV_BOUNDARY_SURFACE Build closed Ax-Ay contours over speed.

arguments
    ggv (1,1) struct
    boundaryPointCount (1,1) double ...
        {mustBeInteger, mustBeGreaterThanOrEqual(boundaryPointCount, 16)} = 161
end

required = ["v_mps", "ay_g", "ax_max_g", "ax_min_g", "feasible"];
if ~all(isfield(ggv, cellstr(required)))
    error("QSSLTS:GGVBoundaryShape", ...
        "GGV speed, lateral grid, branches, and feasibility are required.");
end
vGrid_mps = ggv.v_mps(:);
ayGrid_g = ggv.ay_g(:).';
nSpeed = numel(vGrid_mps);
nLateral = numel(ayGrid_g);
expectedSize = [nSpeed nLateral];
if nSpeed < 2 || nLateral < 2 ...
        || ~isequal(size(ggv.ax_max_g), expectedSize) ...
        || ~isequal(size(ggv.ax_min_g), expectedSize) ...
        || ~isequal(size(ggv.feasible), expectedSize)
    error("QSSLTS:GGVBoundaryShape", ...
        "GGV capability arrays must be Nv-by-Nay.");
end

axBoundary_g = nan(nSpeed, boundaryPointCount);
ayBoundary_g = nan(nSpeed, boundaryPointCount);
normalizedPerimeter = linspace(0, 1, boundaryPointCount);
for iSpeed = 1:nSpeed
    valid = ggv.feasible(iSpeed, :) ...
        & isfinite(ggv.ax_max_g(iSpeed, :)) ...
        & isfinite(ggv.ax_min_g(iSpeed, :));
    validIndex = find(valid);
    if numel(validIndex) < 2
        error("QSSLTS:GGVBoundaryCoverage", ...
            "Each GGV speed requires at least two feasible lateral points.");
    end
    if any(diff(validIndex) > 1)
        error("QSSLTS:GGVBoundaryCoverage", ...
            "GGV feasible lateral coverage must be contiguous at each speed.");
    end

    [ayAcceleration_g, axAcceleration_g, ...
        ayBraking_g, axBraking_g] = branchesAtSpeed( ...
        ggv, iSpeed, ayGrid_g, validIndex);
    contourAy_g = [ayAcceleration_g, fliplr(ayBraking_g), ...
        ayAcceleration_g(1)];
    contourAx_g = [axAcceleration_g, fliplr(axBraking_g), ...
        axAcceleration_g(1)];
    segmentLength = hypot(diff(contourAx_g), diff(contourAy_g));
    perimeter = [0 cumsum(segmentLength)];
    keep = [true diff(perimeter) > eps(max(1, perimeter(end)))];
    perimeter = perimeter(keep);
    contourAx_g = contourAx_g(keep);
    contourAy_g = contourAy_g(keep);
    if perimeter(end) <= 0
        error("QSSLTS:GGVBoundaryCoverage", ...
            "GGV boundary perimeter must be positive.");
    end
    perimeter = perimeter / perimeter(end);
    axBoundary_g(iSpeed, :) = interp1( ...
        perimeter, contourAx_g, normalizedPerimeter, "linear");
    ayBoundary_g(iSpeed, :) = interp1( ...
        perimeter, contourAy_g, normalizedPerimeter, "linear");
end
axBoundary_g(:, end) = axBoundary_g(:, 1);
ayBoundary_g(:, end) = ayBoundary_g(:, 1);
speedBoundary_mps = repmat(vGrid_mps, 1, boundaryPointCount);
end

function [ayAcceleration_g, axAcceleration_g, ...
        ayBraking_g, axBraking_g] = branchesAtSpeed( ...
        ggv, iSpeed, ayGrid_g, validIndex)
ayAcceleration_g = ayGrid_g(validIndex);
axAcceleration_g = ggv.ax_max_g(iSpeed, validIndex);
ayBraking_g = ayAcceleration_g;
axBraking_g = ggv.ax_min_g(iSpeed, validIndex);

boundaryFields = ["ay_limit_neg_g", "ay_limit_pos_g", ...
    "ax_max_lateral_boundary_g", "ax_min_lateral_boundary_g"];
if ~all(isfield(ggv, cellstr(boundaryFields)))
    return
end
negativeLimit_g = ggv.ay_limit_neg_g(iSpeed);
positiveLimit_g = ggv.ay_limit_pos_g(iSpeed);
axMaxNegative_g = endpointValue(ayAcceleration_g, ...
    axAcceleration_g, negativeLimit_g, ggv, ...
    "ax_max_lateral_boundary_g", iSpeed);
axMaxPositive_g = endpointValue(ayAcceleration_g, ...
    axAcceleration_g, positiveLimit_g, ggv, ...
    "ax_max_lateral_boundary_g", iSpeed);
axMinNegative_g = endpointValue(ayBraking_g, ...
    axBraking_g, negativeLimit_g, ggv, ...
    "ax_min_lateral_boundary_g", iSpeed);
axMinPositive_g = endpointValue(ayBraking_g, ...
    axBraking_g, positiveLimit_g, ggv, ...
    "ax_min_lateral_boundary_g", iSpeed);
interior = ayAcceleration_g > negativeLimit_g ...
    & ayAcceleration_g < positiveLimit_g;
ayAcceleration_g = [negativeLimit_g, ...
    ayAcceleration_g(interior), positiveLimit_g];
axAcceleration_g = [axMaxNegative_g, ...
    axAcceleration_g(interior), axMaxPositive_g];
ayBraking_g = ayAcceleration_g;
axBraking_g = [axMinNegative_g, ...
    axBraking_g(interior), axMinPositive_g];
end

function value = endpointValue(ayBranch_g, axBranch_g, targetAy_g, ...
        ggv, boundaryField, iSpeed)
[distance_g, index] = min(abs(ayBranch_g - targetAy_g));
tolerance_g = max(1e-12, 16 * eps(max(1, abs(targetAy_g))));
if distance_g <= tolerance_g
    value = axBranch_g(index);
elseif isfield(ggv, boundaryField)
    value = ggv.(boundaryField)(iSpeed);
else
    value = axBranch_g(index);
end
end
