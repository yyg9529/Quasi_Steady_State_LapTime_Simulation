function [ggvCalibrated, report] = calibrate_ggv( ...
        ggvModel, ggvReal, options)
%CALIBRATE_GGV Apply bounded speed-dependent real/model residual scales.
%   Lateral scale deforms the ay axis; acceleration and brake scales deform
%   the corresponding longitudinal surfaces. Scale factors outside real-data
%   speed coverage remain 1. This is empirical calibration, not tire fitting.

arguments
    ggvModel (1,1) struct
    ggvReal (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
validateOptions(options);
modelSpeed_mps = ggvModel.v_mps(:);

modelLat_g = symmetricLateralLimit(ggvModel);
realLatAtModel_g = interpolateCovered(ggvReal.v_mps, ...
    symmetricLateralLimit(ggvReal), modelSpeed_mps);
rawLatScale = realLatAtModel_g ./ modelLat_g;

modelAccel0_g = zeroLateralCapability(ggvModel, "accel");
realAccel0_g = zeroLateralCapability(ggvReal, "accel");
realAccelAtModel_g = interpolateCovered( ...
    ggvReal.v_mps, realAccel0_g, modelSpeed_mps);
rawAccelScale = realAccelAtModel_g ./ modelAccel0_g;

modelBrake0_g = abs(zeroLateralCapability(ggvModel, "brake"));
realBrake0_g = abs(zeroLateralCapability(ggvReal, "brake"));
realBrakeAtModel_g = interpolateCovered( ...
    ggvReal.v_mps, realBrake0_g, modelSpeed_mps);
rawBrakeScale = realBrakeAtModel_g ./ modelBrake0_g;

[scaleLat, coverageLat] = processScale(rawLatScale, options);
[scaleAccel, coverageAccel] = processScale(rawAccelScale, options);
[scaleBrake, coverageBrake] = processScale(rawBrakeScale, options);

scaleTable = table(modelSpeed_mps, scaleLat, scaleAccel, scaleBrake, ...
    VariableNames=["v_mps", "scale_lat", "scale_acc", "scale_brake"]);
ggvCalibrated = apply_ggv_calibration_scales(ggvModel, scaleTable);
ggvCalibrated.source = "calibrated(" + string(ggvModel.source) + ")";
ggvCalibrated.notes = "Bounded residual calibration from " ...
    + string(ggvReal.source);

report.scale_table = scaleTable;
report.scale_table.coverage_lat = coverageLat;
report.scale_table.coverage_acc = coverageAccel;
report.scale_table.coverage_brake = coverageBrake;
report.raw_scale_lat = rawLatScale;
report.raw_scale_acc = rawAccelScale;
report.raw_scale_brake = rawBrakeScale;
report.scale_min = options.scale_min;
report.scale_max = options.scale_max;
report.model_source = string(ggvModel.source);
report.real_source = string(ggvReal.source);
report.file = "";

if strlength(string(options.calibration_report_file)) > 0
    report.file = saveCalibrationReport( ...
        report.scale_table, string(options.calibration_report_file));
end
ggvCalibrated.calibration_report = report;
end

function limit_g = symmetricLateralLimit(ggv)
limit_g = min(ggv.ay_limit_pos_g(:), abs(ggv.ay_limit_neg_g(:)));
end

function capability0_g = zeroLateralCapability(ggv, branch)
capability0_g = zeros(numel(ggv.v_mps), 1);
for iSpeed = 1:numel(ggv.v_mps)
    if branch == "accel"
        row = ggv.ax_max_g(iSpeed, :);
    else
        row = ggv.ax_min_g(iSpeed, :);
    end
    capability0_g(iSpeed) = interp1(ggv.ay_g, row, 0, "linear");
end
end

function yQuery = interpolateCovered(x, y, xQuery)
yQuery = interp1(x(:), y(:), xQuery(:), "linear", NaN);
end

function [scale, coverage] = processScale(rawScale, options)
coverage = isfinite(rawScale) & rawScale > 0;
scale = ones(size(rawScale));
if any(coverage)
    filledScale = fillmissing(rawScale, "linear", EndValues="nearest");
    window = min(options.calibration_smoothing_window, numel(filledScale));
    smoothedScale = movmean(filledScale, max(1, window));
    scale(coverage) = smoothedScale(coverage);
end
scale = clamp(scale, options.scale_min, options.scale_max);
scale(~coverage) = 1;
end

function filename = saveCalibrationReport(scaleTable, filename)
folder = fileparts(filename);
if strlength(folder) > 0 && ~isfolder(folder)
    mkdir(folder);
end
writetable(scaleTable, filename);
end

function validateOptions(options)
if options.scale_min <= 0 || options.scale_max < options.scale_min ...
        || options.calibration_smoothing_window < 1 ...
        || fix(options.calibration_smoothing_window) ...
        ~= options.calibration_smoothing_window
    error("QSSLTS:CalibrationOptions", ...
        "Calibration bounds/window are invalid.");
end
end
