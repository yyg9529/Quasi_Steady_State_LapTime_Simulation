classdef v1BaselineRegressionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
        end
    end

    methods (Test, TestTags={'Regression'})
        function testAnalyticConstantMuSkidpadBaseline(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            expected = v1_analytic_baseline();
            track = read_track_csv(fullfile(projectRoot, "data", "track", ...
                "skidpad_constant_radius.csv"));
            vehicle = vehicle_baseline();
            models = struct();
            models.tire = tire_simple_baseline();
            models.powertrain = struct("enabled", false);
            models.brake = brake_baseline();
            options = default_qss_options();
            options.v_max_mps = 30;
            options.v_grid_mps = (0:0.25:30).';

            result = run_qss_lap(track, vehicle, models, options);

            testCase.verifyEqual(options.gravity_mps2, ...
                expected.gravity_mps2, AbsTol=0);
            testCase.verifyEqual(models.tire.mu_y, expected.mu_y, AbsTol=0);
            testCase.verifyEqual(1 / track.kappa_1pm(1), ...
                expected.radius_m, AbsTol=0);
            testCase.verifyEqual(track.length_m, ...
                expected.fixture_track_length_m, AbsTol=1e-9);
            testCase.verifyEqual(mean(result.v_mps), ...
                expected.expected_speed_mps, AbsTol=1e-6);
            testCase.verifyEqual(result.lap_time_s, ...
                expected.expected_fixture_lap_time_s, AbsTol=1e-6);
            testCase.verifyTrue(result.solver.converged);
            testCase.verifyTrue(all(result.limiter == "lateral"));
        end


        function testAnalyticConstantMuRwdGgvAtZeroAy(testCase)
            expected = v1_analytic_baseline();
            vehicle = vehicle_baseline();
            tire = tire_simple_baseline();
            aero = struct("enabled", false);
            powertrain = powertrain_emrax228_hvcc_demo();
            brake = brake_baseline();
            brake.max_decel_g_mechanical = inf;
            brake.max_total_brake_force_N = inf;
            brake.max_total_brake_torque_Nm = inf;
            brake.front_bias = expected.brake_front_bias;
            options = default_qss_options();
            options.v_grid_mps = [10; 20];
            options.ay_grid_g = -2:0.1:2;

            ggv = generate_model_ggv(vehicle, tire, aero, ...
                powertrain, brake, options);
            zeroAyIndex = find(ggv.ay_g == 0, 1);

            testCase.verifyEqual(vehicle.mass.front_static_frac, ...
                expected.front_static_frac, AbsTol=0);
            testCase.verifyEqual(vehicle.mass.cg_height_m, ...
                expected.cg_height_m, AbsTol=0);
            testCase.verifyEqual(vehicle.geometry.wheelbase_m, ...
                expected.wheelbase_m, AbsTol=0);
            testCase.verifyEqual(tire.mu_x, expected.mu_x, AbsTol=0);
            testCase.verifyEqual(ggv.ax_max_g(1, zeroAyIndex), ...
                expected.expected_rwd_accel_g, AbsTol=1e-6);
            testCase.verifyEqual(ggv.ax_min_g(1, zeroAyIndex), ...
                -expected.expected_brake_magnitude_g, AbsTol=1e-6);
            testCase.verifyTrue(ggv.solve_converged_accel(1, zeroAyIndex));
            testCase.verifyTrue(ggv.solve_converged_brake(1, zeroAyIndex));
            testCase.verifyFalse(ggv.wheel_lift(1, zeroAyIndex));
        end
    end
end
