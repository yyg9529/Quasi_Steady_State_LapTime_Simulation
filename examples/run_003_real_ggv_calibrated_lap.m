%% V1.0 calibrated lap acceptance entry point (synthetic GGV input)
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

vehicle = vehicle_baseline();
models = struct();
models.tire = tire_load_sensitive_baseline();
models.aero = aero_baseline();
models.powertrain = struct("enabled", false);
models.brake = brake_baseline();
models.ggv_real = read_real_ggv(fullfile(projectRoot, "data", ...
    "ggv_real", "synthetic_real_ggv.csv"));
track = read_track_csv(fullfile(projectRoot, "data", "track", ...
    "simple_track.csv"));

options = default_qss_options();
options.v_grid_mps = (0:1:45).';
options.ay_grid_g = -4:0.1:4;
options.calibration_report_file = fullfile(projectRoot, "results", ...
    "example_003_ggv_calibration.csv");
result = run_qss_lap(track, vehicle, models, options);
limiterUsage = summarize_limiter_usage(result);
lapFigure = plot_lap_result(result);

baseConfig = struct();
baseConfig.track = track;
baseConfig.vehicle = vehicle;
baseConfig.models = models;
baseConfig.options = options;
baseConfig.options.calibration_report_file = "";
massSweep = struct();
massSweep.parameter = "vehicle.mass.total_kg";
massSweep.values = [280; 300; 320];
massSweep.make_plot = false;
sensitivity = run_sensitivity_sweep(baseConfig, massSweep);

fprintf("Lap time: %.3f s\n", result.lap_time_s);
fprintf("Max speed: %.3f m/s\n", max(result.v_mps));
fprintf("Max ax / ay: %.3f g / %.3f g\n", ...
    max(result.ax_mps2) / options.gravity_mps2, ...
    max(abs(result.ay_mps2)) / options.gravity_mps2);
fprintf("GGV used: %s\n", result.ggv_used.source);
fprintf("Calibration report: %s\n", result.calibration_report.file);
fprintf("Distance-weighted limiter map:\n");
disp(limiterUsage);
fprintf("Sensitivity summary (vehicle mass):\n");
disp(sensitivity.table);
