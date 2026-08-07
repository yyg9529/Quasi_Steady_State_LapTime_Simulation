function generate_fsc_2025_dynamic_tracks(outputFolder)
%GENERATE_FSC_2025_DYNAMIC_TRACKS Write rule-dimensioned event centerlines.
%   Geometry follows the 2025 Formula Student China rules. Distances are m.

arguments
    outputFolder (1,1) string = fullfile(fileparts(fileparts( ...
        fileparts(mfilename("fullpath")))), "data", "track")
end

if ~isfolder(outputFolder)
    mkdir(outputFolder);
end

writeAcceleration(outputFolder);
writeSkidpad(outputFolder);
end

function writeAcceleration(outputFolder)
rollout_m = 0.30;
s_m = [0; rollout_m + (0:75).'];
x_m = s_m - rollout_m;
y_m = zeros(size(s_m));
curvature_1_m = zeros(size(s_m));
track_width_m = 4.9 * ones(size(s_m));
is_closed = false(size(s_m));
data = table(s_m, x_m, y_m, curvature_1_m, ...
    track_width_m, is_closed);
writetable(data, fullfile(outputFolder, ...
    "fsc_2025_acceleration_open.csv"));
end

function writeSkidpad(outputFolder)
radius_m = 9.125;
segmentsPerCircle = 360;
dPhi_rad = 2 * pi / segmentsPerCircle;

rightPhi_rad = (0:2 * segmentsPerCircle).' * dPhi_rad;
rightX_m = radius_m - radius_m * cos(rightPhi_rad);
rightY_m = radius_m * sin(rightPhi_rad);

leftPhi_rad = (1:2 * segmentsPerCircle).' * dPhi_rad;
leftX_m = -radius_m + radius_m * cos(leftPhi_rad);
leftY_m = radius_m * sin(leftPhi_rad);

x_m = [rightX_m; leftX_m];
y_m = [rightY_m; leftY_m];
s_m = (0:4 * segmentsPerCircle).' * radius_m * dPhi_rad;
curvature_1_m = [
    -ones(2 * segmentsPerCircle, 1) / radius_m
    ones(2 * segmentsPerCircle + 1, 1) / radius_m
    ];
track_width_m = 3.0 * ones(size(s_m));
is_closed = false(size(s_m));
data = table(s_m, x_m, y_m, curvature_1_m, ...
    track_width_m, is_closed);
writetable(data, fullfile(outputFolder, ...
    "fsc_2025_skidpad_event_open.csv"));
end
