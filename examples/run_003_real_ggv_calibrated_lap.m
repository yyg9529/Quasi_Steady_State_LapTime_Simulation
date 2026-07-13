%% V0.5 real-GGV calibrated lap example (synthetic input)
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

vehicle = vehicle_baseline();
models.tire = tire_load_sensitive_baseline();
models.aero = aero_baseline();
models.powertrain = powertrain_baseline();
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

fprintf("Lap time: %.3f s\n", result.lap_time_s);
fprintf("Max speed: %.3f m/s\n", max(result.v_mps));
fprintf("Max ax / ay: %.3f g / %.3f g\n", ...
    max(result.ax_mps2) / options.gravity_mps2, ...
    max(abs(result.ay_mps2)) / options.gravity_mps2);
fprintf("GGV used: %s\n", result.ggv_used.source);
fprintf("Calibration report: %s\n", result.calibration_report.file);
