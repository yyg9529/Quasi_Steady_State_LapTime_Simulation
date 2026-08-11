classdef assessGgvHealthTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        Ggv
        Options
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));

            options = default_qss_options();
            options.v_grid_mps = [0; 10; 20];
            options.ay_grid_g = -2:0.1:2;
            testCase.Options = options;
            testCase.Ggv = generate_model_ggv(vehicle_baseline(), ...
                tire_simple_baseline(), struct("enabled", false), ...
                struct("enabled", false), brake_baseline(), options);
        end
    end

    methods (Test)
        function testHealthyModelPassesAllApplicableDiagnostics(testCase)
            health = assess_ggv_health(testCase.Ggv, testCase.Options);

            testCase.verifyTrue(health.valid);
            testCase.verifyEqual(health.status, "healthy");
            testCase.verifyEmpty(health.reason_codes);
            testCase.verifyEqual(health.fixed_point.status, "pass");
            testCase.verifyEqual(health.residual.status, "pass");
            testCase.verifyEqual(health.lateral_grid.status, "pass");
            testCase.verifyEqual(health.wheel_lift.status, "pass");
        end

        function testFixedPointAndResidualFailuresAreReported(testCase)
            unhealthy = testCase.Ggv;
            feasibleIndex = find(unhealthy.feasible, 1);
            unhealthy.solve_converged_accel(feasibleIndex) = false;
            unhealthy.solve_residual_brake_mps2(feasibleIndex) = ...
                10 * testCase.Options.ggv_tolerance_mps2;

            health = assess_ggv_health(unhealthy, testCase.Options);

            testCase.verifyFalse(health.valid);
            testCase.verifyEqual(health.status, "unhealthy");
            testCase.verifyEqual(health.fixed_point.status, "fail");
            testCase.verifyEqual(health.residual.status, "fail");
            testCase.verifyTrue(ismember( ...
                "ggv_fixed_point_not_converged", health.reason_codes));
            testCase.verifyTrue(ismember( ...
                "ggv_residual_exceeded", health.reason_codes));
        end

        function testTruncationAndWheelLiftAreReported(testCase)
            unhealthy = testCase.Ggv;
            feasibleIndex = find(unhealthy.feasible, 1);
            unhealthy.lateral_limit_truncated(1, 1) = true;
            unhealthy.wheel_lift(feasibleIndex) = true;

            health = assess_ggv_health(unhealthy, testCase.Options);

            testCase.verifyFalse(health.valid);
            testCase.verifyEqual(health.lateral_grid.status, "fail");
            testCase.verifyEqual(health.wheel_lift.status, "fail");
            testCase.verifyTrue(ismember( ...
                "ggv_lateral_grid_truncated", health.reason_codes));
            testCase.verifyTrue(ismember( ...
                "ggv_wheel_lift", health.reason_codes));
        end

        function testExternalMapMissingModelDiagnosticsIsNotApplicable( ...
                testCase)
            external = rmfield(testCase.Ggv, [ ...
                "solve_converged_accel", "solve_converged_brake", ...
                "solve_residual_accel_mps2", ...
                "solve_residual_brake_mps2", "wheel_lift", ...
                "lateral_limit_truncated"]);
            external.source = "real_csv";

            health = assess_ggv_health(external, testCase.Options);

            testCase.verifyTrue(health.valid);
            testCase.verifyEqual(health.fixed_point.status, ...
                "not_applicable");
            testCase.verifyEqual(health.residual.status, ...
                "not_applicable");
            testCase.verifyEqual(health.lateral_grid.status, ...
                "not_applicable");
            testCase.verifyEqual(health.wheel_lift.status, ...
                "not_applicable");
        end

        function testNonfiniteFeasibleCapabilityFailsHealth(testCase)
            unhealthy = testCase.Ggv;
            feasibleIndex = find(unhealthy.feasible, 1);
            unhealthy.ax_max_g(feasibleIndex) = NaN;

            health = assess_ggv_health(unhealthy, testCase.Options);

            testCase.verifyFalse(health.valid);
            testCase.verifyEqual(health.finite_data.status, "fail");
            testCase.verifyTrue(ismember( ...
                "ggv_nonfinite_data", health.reason_codes));
        end

        function testMapWithoutFeasibleDomainFailsHealth(testCase)
            unhealthy = testCase.Ggv;
            unhealthy.feasible(:) = false;

            health = assess_ggv_health(unhealthy, testCase.Options);

            testCase.verifyFalse(health.valid);
            testCase.verifyEqual(health.structure.status, "fail");
            testCase.verifyEqual(health.feasible_domain.status, "fail");
            testCase.verifyEqual(health.feasible_domain.feasible_count, 0);
            testCase.verifyTrue(ismember( ...
                "ggv_no_feasible_domain", health.reason_codes));
        end

        function testGravityMetadataMismatchFailsHealth(testCase)
            unhealthy = testCase.Ggv;
            unhealthy.gravity_mps2 = testCase.Options.gravity_mps2 + 0.1;

            health = assess_ggv_health(unhealthy, testCase.Options);

            testCase.verifyFalse(health.valid);
            testCase.verifyEqual(health.structure.status, "fail");
            testCase.verifyEqual(health.gravity_consistency.status, "fail");
            testCase.verifyEqual(health.gravity_consistency.expected_mps2, ...
                testCase.Options.gravity_mps2, AbsTol=0);
            testCase.verifyTrue(ismember( ...
                "ggv_gravity_mismatch", health.reason_codes));
        end

        function testNonfiniteFixedPointFlagFailsHealth(testCase)
            unhealthy = testCase.Ggv;
            feasibleIndex = find(unhealthy.feasible, 1);
            unhealthy.solve_converged_accel = double( ...
                unhealthy.solve_converged_accel);
            unhealthy.solve_converged_accel(feasibleIndex) = NaN;

            health = assess_ggv_health(unhealthy, testCase.Options);

            testCase.verifyFalse(health.valid);
            testCase.verifyEqual(health.structure.status, "fail");
            testCase.verifyEqual(health.fixed_point.status, "fail");
            testCase.verifyTrue(ismember( ...
                "ggv_fixed_point_diagnostic_nonfinite", ...
                health.reason_codes));
        end

        function testLapValiditySeparatesGgvFromPropagationWithoutDrift( ...
                testCase)
            radius_m = 8;
            pointCount = 32;
            ds_m = 2 * pi * radius_m / pointCount;
            track.s_m = (0:pointCount-1).' * ds_m;
            track.ds_m = ds_m * ones(pointCount, 1);
            track.kappa_1pm = ones(pointCount, 1) / radius_m;
            track.is_closed = true;
            healthyModels.ggv = testCase.Ggv;
            unhealthyModels = healthyModels;
            feasibleIndex = find(unhealthyModels.ggv.feasible, 1);
            unhealthyModels.ggv.solve_converged_accel( ...
                feasibleIndex) = false;

            healthy = run_qss_lap(track, vehicle_baseline(), ...
                healthyModels, testCase.Options);
            unhealthy = run_qss_lap(track, vehicle_baseline(), ...
                unhealthyModels, testCase.Options);

            testCase.verifyTrue(healthy.propagation_converged);
            testCase.verifyTrue(healthy.ggv_healthy);
            testCase.verifyTrue(healthy.valid);
            testCase.verifyTrue(unhealthy.propagation_converged);
            testCase.verifyFalse(unhealthy.ggv_healthy);
            testCase.verifyFalse(unhealthy.valid);
            testCase.verifyEqual(unhealthy.status, "ggv_invalid");
            testCase.verifyEqual(unhealthy.v_mps, healthy.v_mps, ...
                AbsTol=0);
            testCase.verifyEqual(unhealthy.lap_time_s, ...
                healthy.lap_time_s, AbsTol=0);
        end
    end
end
