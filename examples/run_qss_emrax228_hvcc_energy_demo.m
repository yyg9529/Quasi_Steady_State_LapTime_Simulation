%% EMRAX 228 / HVCC fixed-raceline QSS energy demonstration
% The Tianji CSV is a version-controlled project input. This example has no
% fallback track: read_track_csv reports a missing-input error directly.

projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

trackFile = fullfile(projectRoot, "data", "track", ...
    "tianji_kart_QSS_track_closed.csv");
track = read_track_csv(trackFile);

vehicle = vehicle_baseline();
models = struct();
models.tire = tire_load_sensitive_baseline();
models.aero = aero_baseline();
models.powertrain = powertrain_emrax228_hvcc_demo();
models.brake = brake_baseline();
models.endurance = struct("num_laps", 26, "safety_factor", 1.10);

options = default_qss_options();
options.v_grid_mps = (0:1:45).';
options.ay_grid_g = -4:0.1:4;

% No prebuilt GGV is supplied: run_qss_lap generates the composite-powertrain
% GGV online and queries longitudinal capability at the actual Ay.
result = run_qss_lap(track, vehicle, models, options);
limiterUsage = summarize_limiter_usage(result);

fprintf("Lap time: %.3f s\n", result.lap_time_s);
fprintf("Maximum speed: %.3f m/s\n", max(result.v_mps));
fprintf("Maximum motor speed: %.0f rpm\n", ...
    max(result.powertrain.motor_speed_rpm));
fprintf("Maximum TSAC power: %.3f kW\n", ...
    max(result.powertrain.tsac_power_used_W) / 1e3);
fprintf("Maximum TSAC DC current: %.3f A\n", ...
    max(result.powertrain.tsac_dc_current_used_A));
fprintf("Lap TS energy: %.4f kWh\n", result.energy.E_lap_ts_kWh);
fprintf("Lap stored energy: %.4f kWh\n", ...
    result.energy.E_lap_stored_kWh);
fprintf("Nominal endurance capacity required: %.4f kWh\n", ...
    result.energy.E_nominal_required_kWh);
fprintf("\nLimiter distribution by track distance:\n");
disp(limiterUsage)

trackFigure = plot_track_speed_map(result);
powertrainFigure = plot_powertrain_energy_result(result);
ggvFigure = plot_ggv_surface(result);
drawnow

fprintf("Thermal feasibility not evaluated\n");
fprintf("Regenerative braking disabled\n");
fprintf("First-order endurance estimate\n");
