classdef ymdUndersteerTest < matlab.unittest.TestCase
    properties
        Vehicle
        Aero
        TireModel
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));

            testCase.Vehicle = ymdUndersteerTest.makeVehicle();
            testCase.Aero = struct("enabled", false);
            testCase.TireModel.evaluate = @(input) ...
                ymdUndersteerTest.evaluateLinearTire(input, 60000, 80000);
        end
    end

    methods (Test)
        function testYmdGridShapeAndStraightAheadCenter(testCase)
            study.speed_mps = 15;
            study.beta_rad = [-0.02; 0; 0.02];
            study.steer_rad = [-0.04 0 0.04];

            ymd = generate_ymd(testCase.Vehicle, testCase.TireModel, ...
                testCase.Aero, study);

            testCase.verifySize(ymd.ay_mps2, [3 3]);
            testCase.verifySize(ymd.yaw_moment_cg_Nm, [3 3]);
            testCase.verifySize(ymd.yaw_moment_coefficient, [3 3]);
            testCase.verifySize(ymd.converged, [3 3]);
            testCase.verifySize(ymd.wheel.Fz_N, [3 3 4]);
            testCase.verifyEqual(ymd.yaw_rate_radps(2, 2), 0, ...
                AbsTol=1e-10);
            testCase.verifyEqual(ymd.ay_mps2(2, 2), 0, AbsTol=1e-10);
            testCase.verifyEqual(ymd.yaw_moment_cg_Nm(2, 2), 0, ...
                AbsTol=1e-8);
            testCase.verifyEqual(ymd.yaw_moment_coefficient, ...
                ymd.yaw_moment_cg_Nm / (testCase.Vehicle.mass.total_kg ...
                * 9.80665 * testCase.Vehicle.geometry.wheelbase_m), ...
                AbsTol=1e-12);
            expectedNormal_mps2 = ymd.yaw_rate_radps(1, 1) ...
                * study.speed_mps;
            testCase.verifyEqual( ...
                ymd.normal_acceleration_mps2(1, 1), ...
                expectedNormal_mps2, AbsTol=1e-10);
            testCase.verifyEqual(ymd.ay_mps2(1, 1), ...
                expectedNormal_mps2 * cos(study.beta_rad(1)), ...
                AbsTol=1e-10);
            testCase.verifyTrue(all(ymd.converged, "all"));
        end

        function testSteadyCorneringSatisfiesBothEquilibriumResiduals(testCase)
            condition.speed_mps = 10;
            condition.radius_m = 50;

            steady = solve_steady_state_cornering(testCase.Vehicle, ...
                testCase.TireModel, testCase.Aero, condition);

            testCase.verifyTrue(steady.converged);
            testCase.verifyEqual(steady.yaw_rate_radps, 0.2, ...
                AbsTol=1e-10);
            testCase.verifyLessThan(abs(steady.force_residual_N), 1e-6);
            testCase.verifyLessThan(abs(steady.moment_residual_Nm), 1e-6);
        end

        function testUndersteerStudyUsesFixedRadiusAcceleration(testCase)
            study.radius_m = 50;
            study.speed_mps = [5; 10; 15];
            study.linear_fit_range_g = [0 1];
            gravity_mps2 = 9.80665;

            result = calc_understeer_gradient(testCase.Vehicle, ...
                testCase.TireModel, testCase.Aero, study);
            expectedAy_g = study.speed_mps.^2 ...
                / (study.radius_m * gravity_mps2);

            testCase.verifyEqual(result.radius_m, study.radius_m, ...
                AbsTol=1e-12);
            testCase.verifyEqual(result.yaw_rate_radps, ...
                study.speed_mps / study.radius_m, AbsTol=1e-10);
            testCase.verifyEqual(result.ay_g, expectedAy_g, AbsTol=1e-10);
            testCase.verifyTrue(all(result.converged));
        end

        function testLinearTireUndersteerMatchesAnalyticGradient(testCase)
            study.radius_m = 50;
            study.speed_mps = [5; 10; 15];
            study.linear_fit_range_g = [0 1];
            mass_kg = testCase.Vehicle.mass.total_kg;
            wheelbase_m = testCase.Vehicle.geometry.wheelbase_m;
            a_m = 0.5 * wheelbase_m;
            b_m = 0.5 * wheelbase_m;
            expected_rad_per_g = mass_kg * 9.80665 / wheelbase_m ...
                * (b_m / 60000 - a_m / 80000);
            expected_deg_per_g = rad2deg(expected_rad_per_g);

            result = calc_understeer_gradient(testCase.Vehicle, ...
                testCase.TireModel, testCase.Aero, study);

            testCase.verifyEqual(result.linear_fit_gradient_deg_per_g, ...
                expected_deg_per_g, AbsTol=1e-6);
            testCase.verifyEqual(result.linear_fit_R2, 1, AbsTol=1e-10);
            testCase.verifyLessThan(max(abs(result.force_residual_N)), 1e-6);
            testCase.verifyLessThan(max(abs(result.moment_residual_Nm)), 1e-6);
        end

        function testInvalidTireRangeDoesNotContaminateLocalGradient(testCase)
            tireModel.evaluate = @(input) ...
                ymdUndersteerTest.evaluateRangeLimitedTire( ...
                input, 60000, 80000);
            study.radius_m = 50;
            study.speed_mps = [5; 10; 15];
            study.linear_fit_range_g = [0 1];

            result = calc_understeer_gradient(testCase.Vehicle, ...
                tireModel, testCase.Aero, study);

            testCase.verifyEqual(result.within_tire_range, ...
                [true; false; true]);
            testCase.verifyTrue(all(isnan( ...
                result.local_gradient_deg_per_g)));
        end

        function testInsufficientFitPointsRetainUndersteerCurve(testCase)
            study.radius_m = 50;
            study.speed_mps = [5; 10; 15];
            study.linear_fit_range_g = [0.15 0.25];

            result = calc_understeer_gradient(testCase.Vehicle, ...
                testCase.TireModel, testCase.Aero, study);

            testCase.verifyFalse(result.fit_available);
            testCase.verifyEqual(result.fit_point_count, 1);
            testCase.verifyEqual(result.valid_point_count, 3);
            testCase.verifyEqual(result.status, "partial");
            testCase.verifySize(result.ay_g, [3 1]);
            testCase.verifyTrue(all(isfinite(result.roadwheel_steer_rad)));
            testCase.verifyTrue(isnan( ...
                result.linear_fit_gradient_deg_per_g));
            testCase.verifyTrue(isnan( ...
                result.steady_curve_gradient_deg_per_g));
        end
    end

    methods (Static)
        function vehicle = makeVehicle()
            vehicle.mass.total_kg = 1000;
            vehicle.mass.front_static_frac = 0.5;
            vehicle.mass.cg_height_m = 0.3;
            vehicle.geometry.wheelbase_m = 2.4;
            vehicle.geometry.track_front_m = 1.6;
            vehicle.geometry.track_rear_m = 1.6;
            vehicle.inertia.Iz_kgm2 = 1500;
            vehicle.load_transfer.front_lateral_distribution = 0.5;
        end

        function output = evaluateLinearTire(input, frontStiffness_Nprad, ...
                rearStiffness_Nprad)
            stiffness_Nprad = 0.5 * [frontStiffness_Nprad; ...
                frontStiffness_Nprad; rearStiffness_Nprad; ...
                rearStiffness_Nprad];
            output.Fx_N = zeros(4, 1);
            output.Fy_N = stiffness_Nprad .* input.alpha_rad;
            output.Mz_Nm = zeros(4, 1);
        end


        function output = evaluateRangeLimitedTire(input, ...
                frontStiffness_Nprad, rearStiffness_Nprad)
            output = ymdUndersteerTest.evaluateLinearTire( ...
                input, frontStiffness_Nprad, rearStiffness_Nprad);
            valid = abs(mean(input.Vx_mps) - 10) > 1;
            output.within_range = repmat(valid, size(input.Vx_mps));
        end
    end
end
