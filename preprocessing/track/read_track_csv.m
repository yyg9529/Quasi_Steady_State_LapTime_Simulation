function track = read_track_csv(filename)
%READ_TRACK_CSV Read a discretized track from CSV.
%   track = READ_TRACK_CSV(filename) accepts s_m,kappa_1pm columns. Each
%   row is a unique node; for a closed track the final row must not repeat
%   the first node. Distances use m and curvature uses 1/m.

arguments
    filename (1,1) string
end

if ~isfile(filename)
    error("QSSLTS:TrackFileNotFound", "Track file not found: %s", filename);
end

inputTable = readtable(filename, TextType="string", ...
    VariableNamingRule="preserve");
names = string(inputTable.Properties.VariableNames);

hasSpatialColumns = all(ismember(["s_m", "kappa_1pm"], names));
hasXYColumns = all(ismember(["x_m", "y_m"], names));
if ~hasSpatialColumns
    if hasXYColumns
        error("QSSLTS:TrackXYNotImplemented", ...
            "XY track input is not implemented yet.");
    end
    error("QSSLTS:TrackColumns", ...
        "Track CSV must contain s_m,kappa_1pm columns.");
end

s_m = inputTable.s_m(:);
kappa_1pm = inputTable.kappa_1pm(:);
if numel(s_m) < 2 || any(~isfinite(s_m)) || any(diff(s_m) <= 0)
    error("QSSLTS:TrackArcLength", ...
        "s_m must contain at least two finite, strictly increasing nodes.");
end
if any(~isfinite(kappa_1pm))
    error("QSSLTS:TrackCurvature", "kappa_1pm must be finite.");
end

isClosed = true;
if ismember("is_closed", names)
    isClosed = logical(inputTable.is_closed(1));
end

baseSpacing = diff(s_m);
if isClosed
    closingSpacing = median(baseSpacing);
    ds_m = [baseSpacing; closingSpacing];
else
    ds_m = baseSpacing;
end

track.s_m = s_m;
track.ds_m = ds_m;
track.kappa_1pm = kappa_1pm;
track.x_m = nan(size(s_m));
track.y_m = nan(size(s_m));
track.is_closed = isClosed;
track.length_m = sum(ds_m);
track.source_file = filename;
end
