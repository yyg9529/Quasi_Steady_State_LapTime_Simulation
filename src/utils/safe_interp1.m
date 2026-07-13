function yq = safe_interp1(x, y, xq, policy)
%SAFE_INTERP1 Interpolate finite tabulated values with explicit bounds policy.
%   x and xq use the same units. policy is "clamp" or "error".

arguments
    x (:,1) double
    y double
    xq double
    policy (1,1) string = "clamp"
end

if numel(x) ~= size(y, 1)
    error("QSSLTS:InterpolationSize", ...
        "The first dimension of y must match numel(x).");
end
if any(diff(x) <= 0) || any(~isfinite(x), "all")
    error("QSSLTS:InterpolationGrid", ...
        "Interpolation breakpoints must be finite and strictly increasing.");
end

switch policy
    case "clamp"
        xQuery = clamp(xq, x(1), x(end));
    case "error"
        if any(xq < x(1) | xq > x(end), "all")
            error("QSSLTS:InterpolationOutOfRange", ...
                "Query is outside the interpolation range.");
        end
        xQuery = xq;
    otherwise
        error("QSSLTS:InterpolationPolicy", ...
            "Unknown interpolation policy: %s", policy);
end

yq = interp1(x, y, xQuery, "linear");
end
