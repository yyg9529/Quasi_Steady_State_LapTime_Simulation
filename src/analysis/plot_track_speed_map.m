function fig = plot_track_speed_map(result)
%PLOT_TRACK_SPEED_MAP Plot track coordinates colored by node speed.

arguments
    result (1,1) struct
end

validateResult(result);
x_m = result.track.x_m(:);
y_m = result.track.y_m(:);
speed_mps = result.v_mps(:);
isClosed = isfield(result.track, "is_closed") ...
    && isscalar(result.track.is_closed) ...
    && logical(result.track.is_closed);
if isClosed
    x_m(end + 1, 1) = x_m(1);
    y_m(end + 1, 1) = y_m(1);
    speed_mps(end + 1, 1) = speed_mps(1);
end

fig = figure(Name="Track Speed Map");
ax = axes(fig);
surface(ax, [x_m x_m], [y_m y_m], zeros(numel(x_m), 2), ...
    [speed_mps speed_mps], FaceColor="none", EdgeColor="interp", ...
    LineWidth=2, Tag="track-speed-line", ...
    DisplayName="Vehicle speed");
axis(ax, "equal");
grid(ax, "on");
xlabel(ax, "x (m)");
ylabel(ax, "y (m)");
title(ax, "Track Speed Map");
colorScale = colorbar(ax);
colorScale.Label.String = "Speed (m/s)";
end

function validateResult(result)
if ~isfield(result, "track") || ~isstruct(result.track) ...
        || ~isfield(result.track, "x_m") ...
        || ~isfield(result.track, "y_m")
    error("QSSLTS:PlotTrackCoordinates", ...
        "result.track.x_m and result.track.y_m are required.");
end
x_m = result.track.x_m;
y_m = result.track.y_m;
if ~isnumeric(x_m) || ~isnumeric(y_m) || ~isreal(x_m) ...
        || ~isreal(y_m) || ~isvector(x_m) || ~isvector(y_m) ...
        || isempty(x_m) || any(~isfinite(x_m(:))) ...
        || any(~isfinite(y_m(:)))
    error("QSSLTS:PlotTrackCoordinates", ...
        "Track coordinates must be nonempty finite real vectors.");
end
if ~isfield(result, "v_mps") || ~isnumeric(result.v_mps) ...
        || ~isreal(result.v_mps) || ~isvector(result.v_mps) ...
        || any(~isfinite(result.v_mps(:))) ...
        || numel(x_m) ~= numel(y_m) ...
        || numel(x_m) ~= numel(result.v_mps)
    error("QSSLTS:PlotResultShape", ...
        "Track coordinates and result.v_mps must have equal node counts.");
end
end
