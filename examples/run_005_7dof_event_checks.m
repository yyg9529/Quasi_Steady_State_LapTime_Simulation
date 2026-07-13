%% V0.8 7DOF key-event consistency checks
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

vehicle = vehicle_baseline();
tire = tire_load_sensitive_baseline();
powertrain = powertrain_baseline();
brake = brake_baseline();

ggvOptions = default_qss_options();
ggvOptions.v_grid_mps = (0:2:30).';
ggvOptions.ay_grid_g = -2.5:0.1:2.5;
aeroOff = struct("enabled", false);
ggv = generate_model_ggv( ...
    vehicle, tire, aeroOff, powertrain, brake, ggvOptions);

accelOptions.target_speed_mps = 15;
accelOptions.ggv = ggv;
accel = run_7dof_accel_event( ...
    vehicle, tire, powertrain, brake, accelOptions);

brakeOptions.initial_speed_mps = 15;
brakeOptions.stop_speed_mps = 2;
brakeOptions.ggv = ggv;
braking = run_7dof_brake_event( ...
    vehicle, tire, powertrain, brake, brakeOptions);

radiusOptions.speed_grid_mps = (6:2:20).';
radiusOptions.settle_time_s = 3;
radiusOptions.curvature_tolerance_frac = 0.20;
radiusOptions.ggv = ggv;
radius = run_7dof_constant_radius( ...
    20, vehicle, tire, powertrain, brake, radiusOptions);

fprintf("Acceleration event: %.3f s to %.1f m/s, terminal ax %.3f m/s^2\n", ...
    accel.event_time_s, accel.final_speed_mps, accel.terminal_accel_mps2);
fprintf("  aero-off GGV difference: %.1f %%\n", ...
    100 * accel.comparison.relative_difference);
fprintf("Braking event: %.3f s to %.1f m/s, representative decel %.3f m/s^2\n", ...
    braking.event_time_s, braking.final_speed_mps, ...
    braking.representative_decel_mps2);
fprintf("  aero-off GGV difference: %.1f %%\n", ...
    100 * braking.comparison.relative_difference);
fprintf("20 m radius: stable lower bound ay %.3f m/s^2 at %.3f m/s\n", ...
    radius.max_stable_ay_mps2, radius.max_stable_speed_mps);
fprintf("  refined speed bracket: [%.3f, %.3f] m/s; midpoint estimate " ...
    + "ay %.3f m/s^2\n", ...
    radius.lower_bracket_speed_mps, radius.upper_bracket_speed_mps, ...
    radius.estimated_limit_ay_mps2);
fprintf("  aero-off GGV difference: %.1f %%\n", ...
    100 * radius.comparison.relative_difference);
disp(radius.case_table);
