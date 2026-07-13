function [lapTime_s, segmentTime_s, cumulativeTime_s] = ...
        integrate_lap_time(track, v_mps)
%INTEGRATE_LAP_TIME Integrate spatial segments using endpoint mean speed.
%   dt = 2*ds/(v_i+v_{i+1}) is exact for constant acceleration per segment.

arguments
    track (1,1) struct
    v_mps (:,1) double
end

nPoint = numel(track.s_m);
if numel(v_mps) ~= nPoint || any(~isfinite(v_mps)) || any(v_mps < 0)
    error("QSSLTS:SpeedProfile", ...
        "v_mps must be a finite, nonnegative value at every track node.");
end

isClosed = isfield(track, "is_closed") && track.is_closed;
if isClosed
    if numel(track.ds_m) ~= nPoint
        error("QSSLTS:TrackSegments", ...
            "A closed track requires one segment length per node.");
    end
    nextSpeed_mps = circshift(v_mps, -1);
    segmentStart_mps = v_mps;
else
    if numel(track.ds_m) ~= nPoint - 1
        error("QSSLTS:TrackSegments", ...
            "An open track requires N-1 segment lengths.");
    end
    segmentStart_mps = v_mps(1:end-1);
    nextSpeed_mps = v_mps(2:end);
end

speedSum_mps = segmentStart_mps + nextSpeed_mps;
if any(speedSum_mps <= 0)
    error("QSSLTS:ZeroSegmentSpeed", ...
        "A segment with zero speed at both endpoints has infinite time.");
end

segmentTime_s = 2 * track.ds_m(:) ./ speedSum_mps;
lapTime_s = sum(segmentTime_s);
cumulativeTime_s = [0; cumsum(segmentTime_s)];
end
