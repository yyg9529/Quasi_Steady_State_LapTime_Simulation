function ggvCalibrated = apply_ggv_calibration_scales(ggv, scaleTable)
%APPLY_GGV_CALIBRATION_SCALES Apply an already identified scale table.
%   This function is used by DOE so every design case receives the frozen
%   baseline residual correction instead of being re-fitted to the same car.

arguments
    ggv (1,1) struct
    scaleTable table
end

required = ["v_mps", "scale_lat", "scale_acc", "scale_brake"];
if ~all(ismember(required, string(scaleTable.Properties.VariableNames))) ...
        || height(scaleTable) < 2
    error("QSSLTS:CalibrationScaleTable", ...
        "Scale table is missing fields or speed coverage.");
end
if any(diff(scaleTable.v_mps) <= 0) ...
        || any(scaleTable{:, 2:4} <= 0, "all") ...
        || any(~isfinite(scaleTable{:, 1:4}), "all")
    error("QSSLTS:CalibrationScaleTable", ...
        "Scale-table speeds/scales must be finite, positive, and ordered.");
end

scaleLat = interp1(scaleTable.v_mps, scaleTable.scale_lat, ...
    ggv.v_mps, "linear", "extrap");
scaleAccel = interp1(scaleTable.v_mps, scaleTable.scale_acc, ...
    ggv.v_mps, "linear", "extrap");
scaleBrake = interp1(scaleTable.v_mps, scaleTable.scale_brake, ...
    ggv.v_mps, "linear", "extrap");

ggvCalibrated = ggv;
ggvCalibrated.ax_max_g = zeros(size(ggv.ax_max_g));
ggvCalibrated.ax_min_g = zeros(size(ggv.ax_min_g));
ggvCalibrated.feasible = false(size(ggv.feasible));
ggvCalibrated.accel_limiter = repmat( ...
    "lateral_infeasible", size(ggv.accel_limiter));
ggvCalibrated.brake_limiter = repmat( ...
    "lateral_infeasible", size(ggv.brake_limiter));

for iSpeed = 1:numel(ggv.v_mps)
    mappedAy_g = ggv.ay_g / scaleLat(iSpeed);
    rowFeasible = mappedAy_g <= ggv.ay_limit_pos_g(iSpeed) + 1e-12 ...
        & mappedAy_g >= ggv.ay_limit_neg_g(iSpeed) - 1e-12;
    ggvCalibrated.ax_max_g(iSpeed, :) = interp1( ...
        ggv.ay_g, ggv.ax_max_g(iSpeed, :), mappedAy_g, "linear", 0) ...
        * scaleAccel(iSpeed);
    ggvCalibrated.ax_min_g(iSpeed, :) = interp1( ...
        ggv.ay_g, ggv.ax_min_g(iSpeed, :), mappedAy_g, "linear", 0) ...
        * scaleBrake(iSpeed);
    ggvCalibrated.feasible(iSpeed, :) = rowFeasible;

    nearestIndex = arrayfun(@(value) nearestAyIndex(ggv.ay_g, value), ...
        mappedAy_g);
    accelRow = ggv.accel_limiter(iSpeed, nearestIndex);
    brakeRow = ggv.brake_limiter(iSpeed, nearestIndex);
    accelRow(~rowFeasible) = "lateral_infeasible";
    brakeRow(~rowFeasible) = "lateral_infeasible";
    ggvCalibrated.accel_limiter(iSpeed, :) = accelRow;
    ggvCalibrated.brake_limiter(iSpeed, :) = brakeRow;
end

ggvCalibrated.ay_limit_pos_g = ggv.ay_limit_pos_g .* scaleLat;
ggvCalibrated.ay_limit_neg_g = ggv.ay_limit_neg_g .* scaleLat;
if isfield(ggv, "ax_max_lateral_boundary_g")
    ggvCalibrated.ax_max_lateral_boundary_g = ...
        ggv.ax_max_lateral_boundary_g .* scaleAccel;
end
if isfield(ggv, "ax_min_lateral_boundary_g")
    ggvCalibrated.ax_min_lateral_boundary_g = ...
        ggv.ax_min_lateral_boundary_g .* scaleBrake;
end
ggvCalibrated.source = "frozen_calibration(" + string(ggv.source) + ")";
ggvCalibrated.calibration_scale_table = scaleTable;
end

function index = nearestAyIndex(ayGrid_g, value_g)
[~, index] = min(abs(ayGrid_g - value_g));
end
