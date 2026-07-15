function track = read_track_csv(filename)
%READ_TRACK_CSV Read a discretized track from CSV.
%   track = READ_TRACK_CSV(filename) accepts either s_m,kappa_1pm or
%   s_m,x_m,y_m,curvature_1_m,track_width_m columns. Distances use m and
%   curvature uses 1/m.

arguments
    filename (1,1) string
end

if ~isfile(filename)
    error("QSSLTS:TrackFileNotFound", "Track file not found: %s", filename);
end

inputTable = readtable(filename, TextType="string", ...
    VariableNamingRule="preserve");
names = string(inputTable.Properties.VariableNames);

standardColumns = ["s_m", "kappa_1pm"];
geometryColumns = ["s_m", "x_m", "y_m", ...
    "curvature_1_m", "track_width_m"];
hasStandardSchema = all(ismember(standardColumns, names));
hasGeometrySchema = all(ismember(geometryColumns, names));
if hasGeometrySchema
    requiredColumns = geometryColumns;
elseif hasStandardSchema
    requiredColumns = standardColumns;
else
    error("QSSLTS:TrackColumns", ...
        ["Track CSV must contain s_m,kappa_1pm or ", ...
        "s_m,x_m,y_m,curvature_1_m,track_width_m columns."]);
end

nodeCount = height(inputTable);
for columnName = requiredColumns
    values = inputTable.(columnName);
    if ~isnumeric(values) || ~isreal(values) || size(values, 2) ~= 1
        error("QSSLTS:TrackColumnType", ...
            "Track column %s must be a real numeric column.", columnName);
    end
    if numel(values) ~= nodeCount
        error("QSSLTS:TrackColumnLength", ...
            "Track columns must have equal lengths.");
    end
    if any(~isfinite(values))
        error("QSSLTS:TrackColumnFinite", ...
            "Track column %s must be finite.", columnName);
    end
end

s_m = inputTable.s_m(:);
if numel(s_m) < 2 || any(diff(s_m) <= 0)
    error("QSSLTS:TrackArcLength", ...
        "s_m must contain at least two finite, strictly increasing nodes.");
end

if hasGeometrySchema
    kappa_1pm = inputTable.curvature_1_m(:);
    x_m = inputTable.x_m(:);
    y_m = inputTable.y_m(:);
    trackWidth_m = inputTable.track_width_m(:);
else
    kappa_1pm = inputTable.kappa_1pm(:);
    x_m = nan(size(s_m));
    y_m = nan(size(s_m));
    trackWidth_m = nan(size(s_m));
end

hasClosedFlag = ismember("is_closed", names);
if hasClosedFlag
    closedValues = inputTable.is_closed;
    if ~(isnumeric(closedValues) || islogical(closedValues)) || ...
            ~isreal(closedValues) || size(closedValues, 2) ~= 1 || ...
            numel(closedValues) ~= nodeCount
        error("QSSLTS:TrackClosedFlag", ...
            "is_closed must be a numeric or logical column matching the track length.");
    end
    closedValues = double(closedValues(:));
    if any(~isfinite(closedValues)) || ...
            any(closedValues ~= 0 & closedValues ~= 1) || ...
            any(closedValues ~= closedValues(1))
        error("QSSLTS:TrackClosedFlag", ...
            "is_closed must contain one consistent finite 0 or 1 value.");
    end
    isClosed = logical(closedValues(1));
elseif hasGeometrySchema
    coordinateScale = max(1, max(abs([x_m; y_m])));
    closureTolerance = max(1e-9, 64 * eps(coordinateScale));
    isClosed = hypot(x_m(end) - x_m(1), y_m(end) - y_m(1)) <= ...
        closureTolerance;
else
    isClosed = true;
end

closureIsEstimated = false;
if isClosed && hasGeometrySchema
    coordinateScale = max(1, max(abs([x_m; y_m])));
    closureTolerance = max(1e-9, 64 * eps(coordinateScale));
    hasDuplicateEndpoint = hypot(x_m(end) - x_m(1), ...
        y_m(end) - y_m(1)) <= closureTolerance;
    if ~hasDuplicateEndpoint
        error("QSSLTS:TrackClosure", ...
            "Closed geometry input must repeat the first XY point at the final row.");
    end

    sStart_m = s_m(1);
    sEnd_m = s_m(end);
    s_m(end) = [];
    kappa_1pm(end) = [];
    x_m(end) = [];
    y_m(end) = [];
    trackWidth_m(end) = [];
    if numel(s_m) < 2
        error("QSSLTS:TrackClosure", ...
            "Closed geometry input must contain at least two unique nodes.");
    end
    ds_m = [diff(s_m); sEnd_m - s_m(end)];
    length_m = sEnd_m - sStart_m;
elseif isClosed
    baseSpacing = diff(s_m);
    ds_m = [baseSpacing; median(baseSpacing)];
    length_m = sum(ds_m);
    closureIsEstimated = true;
else
    ds_m = diff(s_m);
    length_m = s_m(end) - s_m(1);
end

track.s_m = s_m;
track.ds_m = ds_m;
track.kappa_1pm = kappa_1pm;
track.x_m = x_m;
track.y_m = y_m;
track.track_width_m = trackWidth_m;
track.is_closed = isClosed;
track.closure_is_estimated = closureIsEstimated;
track.length_m = length_m;
track.source_file = filename;
end
