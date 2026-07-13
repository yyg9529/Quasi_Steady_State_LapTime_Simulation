function [vLat_mps, limiter] = calc_lateral_speed_limit(track, ggv, options)
%CALC_LATERAL_SPEED_LIMIT Solve v^2*kappa against the speed-dependent GGV.
%   Track curvature uses 1/m and the returned speed uses m/s.

arguments
    track (1,1) struct
    ggv (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
nPoint = numel(track.kappa_1pm);
vLat_mps = zeros(nPoint, 1);
limiter = repmat("lateral", nPoint, 1);
vUpper_mps = min(options.v_max_mps, ggv.v_mps(end));

for iPoint = 1:nPoint
    curvature_1pm = track.kappa_1pm(iPoint);
    if abs(curvature_1pm) <= 1e-12
        vLat_mps(iPoint) = vUpper_mps;
        limiter(iPoint) = "not_lateral_limited";
        continue
    end

    turnSign = sign(curvature_1pm);
    if lateralResidual(vUpper_mps, abs(curvature_1pm), ...
            turnSign, ggv, options.gravity_mps2) >= 0
        vLat_mps(iPoint) = vUpper_mps;
        continue
    end

    lower_mps = 0;
    upper_mps = vUpper_mps;
    for iIteration = 1:80
        midpoint_mps = 0.5 * (lower_mps + upper_mps);
        if lateralResidual(midpoint_mps, abs(curvature_1pm), ...
                turnSign, ggv, options.gravity_mps2) >= 0
            lower_mps = midpoint_mps;
        else
            upper_mps = midpoint_mps;
        end
    end
    vLat_mps(iPoint) = lower_mps;
end
end

function residual_g = lateralResidual( ...
        speed_mps, curvatureMagnitude_1pm, turnSign, ggv, gravity_mps2)
if turnSign > 0
    capacity_g = interp1(ggv.v_mps, ggv.ay_limit_pos_g, ...
        clamp(speed_mps, ggv.v_mps(1), ggv.v_mps(end)), "linear");
else
    capacity_g = -interp1(ggv.v_mps, ggv.ay_limit_neg_g, ...
        clamp(speed_mps, ggv.v_mps(1), ggv.v_mps(end)), "linear");
end
demand_g = speed_mps^2 * curvatureMagnitude_1pm / gravity_mps2;
residual_g = capacity_g - demand_g;
end
