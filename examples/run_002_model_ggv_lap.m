%% V0.4 model-based GGV lap example
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

vehicle = vehicle_baseline();
models = struct();
models.tire = tire_load_sensitive_baseline();
models.aero = aero_baseline();
models.powertrain = powertrain_emrax228_hvcc_demo();
models.brake = brake_baseline();
models.endurance = struct("num_laps", 1, "safety_factor", 1.0);
track = read_track_csv(fullfile(projectRoot, "data", "track", ...
    "simple_track.csv"));

options = default_qss_options();
options.v_grid_mps = (0:1:45).';
options.ay_grid_g = -4:0.1:4;
result = run_qss_lap(track, vehicle, models, options);

limiterUsage = summarize_limiter_usage(result);

fprintf("Lap time: %.3f s\n", result.lap_time_s);
fprintf("Max speed: %.3f m/s\n", max(result.v_mps));
fprintf("Max ax: %.3f g\n", max(result.ax_mps2) / options.gravity_mps2);
fprintf("Max ay: %.3f g\n", max(abs(result.ay_mps2)) / options.gravity_mps2);
fprintf("Limiter summary:\n");
disp(limiterUsage);
