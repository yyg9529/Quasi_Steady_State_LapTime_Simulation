function cap = interp_ggv(ggv, v_mps, ay_g, options)
%INTERP_GGV Query longitudinal capability at speed [m/s] and lateral g.
%   Values outside the lateral feasible range return NaN and
%   is_feasible=false. Speed bounds follow options.speed_query_policy.

arguments
    ggv (1,1) struct
    v_mps (1,1) double {mustBeFinite, mustBeNonnegative}
    ay_g (1,1) double {mustBeFinite}
    options (1,1) struct = struct()
end

options = default_qss_options(options);
validateGgvShape(ggv);

vLower = ggv.v_mps(1);
vUpper = ggv.v_mps(end);
wasClamped = v_mps < vLower || v_mps > vUpper;
if wasClamped && options.speed_query_policy == "error"
    cap = infeasibleCapability("ggv_speed_range");
    cap.was_speed_clamped = false;
    return
end
vQuery_mps = clamp(v_mps, vLower, vUpper);

if ay_g >= 0
    lateralLimit_g = interp1(ggv.v_mps, ggv.ay_limit_pos_g, ...
        vQuery_mps, "linear");
    lateralFeasible = ay_g <= lateralLimit_g + 1e-12;
else
    lateralLimit_g = interp1(ggv.v_mps, ggv.ay_limit_neg_g, ...
        vQuery_mps, "linear");
    lateralFeasible = ay_g >= lateralLimit_g - 1e-12;
end

if ~lateralFeasible || ay_g < ggv.ay_g(1) || ay_g > ggv.ay_g(end)
    cap = infeasibleCapability("lateral_infeasible");
    cap.lateral_limit_g = lateralLimit_g;
    cap.was_speed_clamped = wasClamped;
    return
end

axMax_g = interp2(ggv.ay_g, ggv.v_mps, ggv.ax_max_g, ...
    ay_g, vQuery_mps, "linear");
axMin_g = interp2(ggv.ay_g, ggv.v_mps, ggv.ax_min_g, ...
    ay_g, vQuery_mps, "linear");

[~, iSpeed] = min(abs(ggv.v_mps - vQuery_mps));
[~, iLateral] = min(abs(ggv.ay_g - ay_g));
cap.ax_max_mps2 = axMax_g * ggv.gravity_mps2;
cap.ax_min_mps2 = axMin_g * ggv.gravity_mps2;
cap.is_feasible = isfinite(axMax_g) && isfinite(axMin_g);
cap.accel_limiter = ggv.accel_limiter(iSpeed, iLateral);
cap.brake_limiter = ggv.brake_limiter(iSpeed, iLateral);
cap.limiter = cap.accel_limiter;
cap.lateral_limit_g = lateralLimit_g;
cap.was_speed_clamped = wasClamped;
end

function cap = infeasibleCapability(limiter)
cap.ax_max_mps2 = NaN;
cap.ax_min_mps2 = NaN;
cap.is_feasible = false;
cap.accel_limiter = string(limiter);
cap.brake_limiter = string(limiter);
cap.limiter = string(limiter);
cap.lateral_limit_g = NaN;
end

function validateGgvShape(ggv)
required = ["v_mps", "ay_g", "ax_max_g", "ax_min_g", "feasible", ...
    "ay_limit_pos_g", "ay_limit_neg_g", "accel_limiter", ...
    "brake_limiter", "gravity_mps2"];
if ~all(isfield(ggv, cellstr(required)))
    error("QSSLTS:GGVFields", "GGV is missing required fields.");
end
expectedSize = [numel(ggv.v_mps), numel(ggv.ay_g)];
if ~isequal(size(ggv.ax_max_g), expectedSize) || ...
        ~isequal(size(ggv.ax_min_g), expectedSize) || ...
        ~isequal(size(ggv.feasible), expectedSize)
    error("QSSLTS:GGVShape", ...
        "GGV capability matrices must have size Nv-by-Nay.");
end
if any(diff(ggv.v_mps) <= 0) || any(diff(ggv.ay_g) <= 0)
    error("QSSLTS:GGVGrid", "GGV grids must be strictly increasing.");
end
end
