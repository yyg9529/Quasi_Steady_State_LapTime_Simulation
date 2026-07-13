function baseline = v1_analytic_baseline()
%V1_ANALYTIC_BASELINE Return independently derived V1.0 regression values.
%   The fixture track length reflects the six-decimal CSV, not exact 16*pi.

    baseline.schema_version = "QSSLTS_V1_ANALYTIC_BASELINE_V1";
    baseline.gravity_mps2 = 9.80665;
    baseline.radius_m = 8;
    baseline.mu_y = 1.5;
    baseline.expected_speed_mps = 10.8480320795986;
    baseline.fixture_track_length_m = 50.265482;
    baseline.expected_fixture_lap_time_s = 4.63360373855568;
    baseline.exact_circle_length_m = 50.2654824574367;
    baseline.expected_exact_circle_lap_time_s = 4.6336037807234;
    baseline.front_static_frac = 0.40;
    baseline.cg_height_m = 0.250;
    baseline.wheelbase_m = 1.668;
    baseline.mu_x = 1.50;
    baseline.brake_front_bias = 0.60;
    pitchRatio = baseline.cg_height_m / baseline.wheelbase_m;
    rearStaticFrac = 1 - baseline.front_static_frac;

    % RWD fixed point: z = mu_x * (rear_static_frac + z*h/L).
    baseline.expected_rwd_accel_g = baseline.mu_x * rearStaticFrac ...
        / (1 - baseline.mu_x * pitchRatio);

    % Braking fixed points are checked on both axles; the smaller branch
    % is the vehicle limit for a fixed front braking fraction beta.
    frontBrake_g = baseline.mu_x * baseline.front_static_frac ...
        / (baseline.brake_front_bias - baseline.mu_x * pitchRatio);
    rearBrake_g = baseline.mu_x * rearStaticFrac ...
        / ((1 - baseline.brake_front_bias) ...
        + baseline.mu_x * pitchRatio);
    baseline.expected_brake_magnitude_g = min( ...
        frontBrake_g, rearBrake_g);
end
