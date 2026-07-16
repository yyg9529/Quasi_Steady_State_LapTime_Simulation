function summary = summarize_lap_result(result)
%SUMMARIZE_LAP_RESULT Return scalar lap metrics and distance-weighted limits.

arguments
    result (1,1) struct
end

gravity_mps2 = result.options.gravity_mps2;
weights_m = result.track.ds_m(:);
if numel(weights_m) ~= numel(result.limiter)
    weights_m = [weights_m; 0];
end
totalDistance_m = sum(weights_m);
limiter = string(result.limiter(:));

summary.lap_time_s = result.lap_time_s;
summary.max_speed_mps = max(result.v_mps);
summary.max_ax_g = max(result.ax_mps2) / gravity_mps2;
summary.max_brake_g = max(0, -min(result.ax_mps2) / gravity_mps2);
summary.max_ay_g = max(abs(result.ay_mps2)) / gravity_mps2;
summary.percent_lateral_limited = weightedPercent( ...
    limiter == "lateral", weights_m, totalDistance_m);
powertrainLimiters = ["torque", "power", "motor_torque", ...
    "motor_power", "motor_speed", "motor_voltage", "rule_power", ...
    "rule_current", "battery_power", "battery_current", ...
    "inverter_power", "inverter_current"];
summary.percent_power_limited = weightedPercent( ...
    ismember(limiter, powertrainLimiters), weights_m, totalDistance_m);
summary.percent_brake_limited = weightedPercent( ...
    startsWith(limiter, "brake") | limiter == "rear_axle_lift", ...
    weights_m, totalDistance_m);
summary.percent_top_speed_limited = weightedPercent( ...
    limiter == "top_speed", weights_m, totalDistance_m);
summary.limiter_table = summarize_limiter_usage(result);
end

function percentage = weightedPercent(mask, weights, total)
percentage = 100 * sum(weights(mask)) / total;
end
