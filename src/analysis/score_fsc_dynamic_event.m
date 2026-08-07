function score = score_fsc_dynamic_event(result, event)
%SCORE_FSC_DYNAMIC_EVENT Apply FSC acceleration or skidpad timing gates.

arguments
    result (1,1) struct
    event (1,1) struct
end

eventType = string(event.type);
switch eventType
    case "fsc_acceleration"
        score = scoreAcceleration(result, event);
    case "fsc_skidpad"
        score = scoreSkidpad(result, event);
    otherwise
        error("QSSLTS:DynamicEventType", ...
            "Unsupported FSC dynamic event type: %s", eventType);
end
end

function score = scoreAcceleration(result, event)
requiredEvent = ["rollout_distance_m", "timed_distance_m"];
if ~isfield(result, "track") || ~isfield(result.track, "s_m") ...
        || ~isfield(result, "cumulative_time_s") ...
        || ~all(isfield(event, cellstr(requiredEvent)))
    error("QSSLTS:DynamicEventFields", ...
        "Acceleration scoring inputs are incomplete.");
end
s_m = result.track.s_m(:);
time_s = result.cumulative_time_s(:);
startGate_m = s_m(1) + event.rollout_distance_m;
finishGate_m = startGate_m + event.timed_distance_m;
if numel(s_m) ~= numel(time_s) || finishGate_m > s_m(end) + 1e-9
    error("QSSLTS:DynamicEventRange", ...
        "Acceleration path does not cover both timing gates.");
end
gateTime_s = interp1(s_m, time_s, ...
    [startGate_m; finishGate_m], "linear");
score = event;
score.scored_time_s = diff(gateTime_s);
score.path_time_s = time_s(end);
score.start_gate_speed_mps = interp1(s_m, result.v_mps(:), ...
    startGate_m, "linear");
end

function score = scoreSkidpad(result, event)
requiredResult = ["track", "cumulative_time_s"];
requiredEvent = ["circle_length_m", "scoring_diameter_m", "timed_laps"];
if ~all(isfield(result, cellstr(requiredResult))) ...
        || ~isfield(result.track, "s_m") ...
        || ~all(isfield(event, cellstr(requiredEvent)))
    error("QSSLTS:DynamicEventFields", ...
        "Skidpad scoring inputs are incomplete.");
end

s_m = result.track.s_m(:);
time_s = result.cumulative_time_s(:);
if numel(s_m) ~= numel(time_s) || any(diff(s_m) <= 0) ...
        || any(diff(time_s) < 0)
    error("QSSLTS:DynamicEventShape", ...
        "Skidpad distance and cumulative time must be aligned and monotonic.");
end

circleLength_m = event.circle_length_m;
timedLaps = event.timed_laps(:);
if ~isequal(timedLaps, [2; 4])
    error("QSSLTS:DynamicEventLaps", ...
        "FSC skidpad timed_laps must be [2; 4].");
end
gates_m = s_m(1) + (0:4).' * circleLength_m;
if gates_m(end) > s_m(end) + 1e-9
    error("QSSLTS:DynamicEventRange", ...
        "Skidpad path does not cover four complete circles.");
end
gateTime_s = interp1(s_m, time_s, gates_m, "linear");
lapTime_s = diff(gateTime_s);
rightTime_s = lapTime_s(timedLaps(1));
leftTime_s = lapTime_s(timedLaps(2));

score = event;
score.right_timed_lap_s = rightTime_s;
score.left_timed_lap_s = leftTime_s;
score.scored_time_s = mean([rightTime_s, leftTime_s]);
score.path_time_s = time_s(end);
score.right_lateral_accel_g = ...
    2.012 * event.scoring_diameter_m / rightTime_s^2;
score.left_lateral_accel_g = ...
    2.012 * event.scoring_diameter_m / leftTime_s^2;
end
