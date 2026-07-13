%% V0.6 mass and CG-height sensitivity example
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

baseConfig = struct();
baseConfig.track = read_track_csv(fullfile(projectRoot, "data", "track", ...
    "simple_track.csv"));
baseConfig.vehicle = vehicle_baseline();
baseConfig.models.tire = tire_load_sensitive_baseline();
baseConfig.models.aero = aero_baseline();
baseConfig.models.powertrain = powertrain_baseline();
baseConfig.models.brake = brake_baseline();
baseConfig.models.ggv_real = read_real_ggv(fullfile(projectRoot, "data", ...
    "ggv_real", "synthetic_real_ggv.csv"));
baseConfig.options = default_qss_options();
baseConfig.options.v_grid_mps = [0:2:44, 45].';
baseConfig.options.ay_grid_g = -4:0.2:4;

massSweep = struct();
massSweep.parameter = "vehicle.mass.total_kg";
massSweep.values = (260:20:340).';
massSweep.make_plot = true;
massResult = run_sensitivity_sweep(baseConfig, massSweep);

cghSweep = struct();
cghSweep.parameter = "vehicle.mass.cg_height_m";
cghSweep.values = (0.15:0.05:0.35).';
cghSweep.make_plot = true;
cghResult = run_sensitivity_sweep(baseConfig, cghSweep);

fprintf("Calibration mode: %s\n", massResult.calibration_mode);
disp("Mass sensitivity:");
disp(massResult.table);
disp("CG-height sensitivity:");
disp(cghResult.table);
