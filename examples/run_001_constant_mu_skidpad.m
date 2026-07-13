%% V0.1 constant-mu skidpad benchmark
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

vehicle = vehicle_baseline();
models.tire = tire_simple_baseline();
models.aero = aero_baseline();
models.aero.enabled = false;
models.powertrain = powertrain_baseline();
models.powertrain.enabled = false;
models.brake = brake_baseline();

trackFile = fullfile(projectRoot, "data", "track", ...
    "skidpad_constant_radius.csv");
track = read_track_csv(trackFile);

options = default_qss_options();
options.v_max_mps = 30;
options.v_grid_mps = (0:0.25:30).';
result = run_qss_lap(track, vehicle, models, options);

radius_m = 1 / abs(track.kappa_1pm(1));
expectedSpeed_mps = sqrt(models.tire.mu_y * options.gravity_mps2 * radius_m);
expectedLapTime_s = track.length_m / expectedSpeed_mps;

fprintf("Lap time: %.6f s\n", result.lap_time_s);
fprintf("Speed: %.6f m/s (analytic %.6f m/s)\n", ...
    mean(result.v_mps), expectedSpeed_mps);
fprintf("Analytic lap time: %.6f s\n", expectedLapTime_s);
fprintf("Solver converged: %s\n", string(result.solver.converged));
