function [handling, app] = run_four_wheel_handling_demo(tirFile, visible)
%RUN_FOUR_WHEEL_HANDLING_DEMO Run real-TIR YMD and understeer studies.

arguments
    tirFile (1,1) string = ""
    visible (1,1) logical = true
end

projectRoot = string(fileparts(fileparts(mfilename("fullpath"))));
addpath(projectRoot);
project_setup();
if strlength(tirFile) == 0
    tirFile = fullfile(projectRoot, "data", "tire", "local", ...
        "Hoosier_16x75_10_R20.tir");
end
if ~isfile(tirFile)
    error("QSSLTS:HandlingTirMissing", ...
        "Copy the Hoosier TIR to the local tire folder: %s", tirFile);
end

vehicle = vehicle_fs_2026();
aero = struct("enabled", false);
tireModel = load_pac2002_tire(tirFile);

ymdStudy.speed_mps = 12;
ymdStudy.beta_rad = deg2rad((-3:1:3).');
ymdStudy.steer_rad = deg2rad(-6:1.5:6);
handling.ymd = generate_ymd( ...
    vehicle, tireModel, aero, ymdStudy);

understeerStudy.radius_m = 30;
understeerStudy.speed_mps = (5:2:15).';
understeerStudy.linear_fit_range_g = [0, 0.9];
handling.understeer = calc_understeer_gradient( ...
    vehicle, tireModel, aero, understeerStudy);
handling.provenance.tir_file = tirFile;
handling.provenance.tir_sha256 = tireModel.source.sha256;
handling.provenance.vehicle = "vehicle_fs_2026";

app = launch_qsslts_app(visible);
app.showHandlingResult(handling.ymd, handling.understeer);
end
