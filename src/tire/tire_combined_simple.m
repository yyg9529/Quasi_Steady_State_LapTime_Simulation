function FxAvailable_N = tire_combined_simple( ...
        FxMax_N, FyMax_N, FyDemand_N, combined_n)
%TIRE_COMBINED_SIMPLE Return remaining longitudinal force from a p-norm.
%   (|Fx|/FxMax)^n + (|Fy|/FyMax)^n <= 1. Forces use N.

arguments
    FxMax_N double
    FyMax_N double
    FyDemand_N double
    combined_n (1,1) double {mustBeGreaterThanOrEqual(combined_n, 1)}
end

if ~isequal(size(FxMax_N), size(FyMax_N), size(FyDemand_N))
    error("QSSLTS:CombinedSlipSize", ...
        "FxMax, FyMax, and FyDemand must have equal sizes.");
end

lateralRatio = zeros(size(FyMax_N));
loaded = FyMax_N > 0;
lateralRatio(loaded) = abs(FyDemand_N(loaded)) ./ FyMax_N(loaded);
lateralRatio(~loaded & abs(FyDemand_N) > 0) = inf;
remainingFraction = max(0, 1 - lateralRatio.^combined_n) ...
    .^ (1 / combined_n);
FxAvailable_N = max(FxMax_N, 0) .* remainingFraction;
end
