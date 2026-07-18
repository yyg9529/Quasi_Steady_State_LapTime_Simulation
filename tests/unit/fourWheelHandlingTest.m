classdef fourWheelHandlingTest < matlab.unittest.TestCase
    properties
        Vehicle
        Aero
        LinearTire
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));

            testCase.Vehicle = fourWheelHandlingTest.makeVehicle();
            testCase.Aero = struct("enabled", false);
            testCase.LinearTire.evaluate = @(input) ...
                fourWheelHandlingTest.evaluateLinearTire(input, 60000, 80000);
        end
    end

    methods (Test)
        function testStraightLineWheelSpeeds(testCase)
            state = fourWheelHandlingTest.makeState(20, 0, 0, 0);

            wheel = calc_wheel_kinematics(testCase.Vehicle, state);

            testCase.verifyEqual(wheel.x_m, [1.2; 1.2; -1.2; -1.2], ...
                AbsTol=1e-12);
            testCase.verifyEqual(wheel.y_m, [0.8; -0.8; 0.8; -0.8], ...
                AbsTol=1e-12);
            testCase.verifyEqual(wheel.Vx_tire_mps, 20 * ones(4, 1), ...
                AbsTol=1e-12);
            testCase.verifyEqual(wheel.Vy_tire_mps, zeros(4, 1), ...
                AbsTol=1e-12);
            testCase.verifyEqual(wheel.alpha_rad, zeros(4, 1), ...
                AbsTol=1e-12);
        end

        function testPositiveYawMakesRightWheelsFaster(testCase)
            state = fourWheelHandlingTest.makeState(20, 0, 1, 0);

            wheel = calc_wheel_kinematics(testCase.Vehicle, state);

            testCase.verifyEqual(wheel.Vx_body_mps, ...
                [19.2; 20.8; 19.2; 20.8], AbsTol=1e-12);
            testCase.verifyGreaterThan(wheel.Vx_body_mps([2 4]), ...
                wheel.Vx_body_mps([1 3]));
        end

        function testPositiveFrontSteerProducesPositiveSlipAngle(testCase)
            state = fourWheelHandlingTest.makeState(20, 0, 0, 0.1);

            wheel = calc_wheel_kinematics(testCase.Vehicle, state);

            testCase.verifyEqual(wheel.steer_rad, [0.1; 0.1; 0; 0], ...
                AbsTol=1e-12);
            testCase.verifyEqual(wheel.alpha_rad, [0.1; 0.1; 0; 0], ...
                AbsTol=1e-12);
        end

        function testBodyForcesAndCgMomentMatchAnalyticValue(testCase)
            tireModel.evaluate = @fourWheelHandlingTest.evaluateConstantTire;
            state = fourWheelHandlingTest.makeState(20, 0, 0, 0);

            point = evaluate_four_wheel_state( ...
                testCase.Vehicle, tireModel, testCase.Aero, state);

            testCase.verifyEqual(point.Fx_body_N, [100; 200; 300; 400], ...
                AbsTol=1e-12);
            testCase.verifyEqual(point.Fy_body_N, [10; 20; 30; 40], ...
                AbsTol=1e-12);
            testCase.verifyEqual(point.Fx_total_N, 1000, AbsTol=1e-12);
            testCase.verifyEqual(point.Fy_total_N, 100, AbsTol=1e-12);
            testCase.verifyEqual(point.yaw_moment_cg_Nm, 122, ...
                AbsTol=1e-12);
            testCase.verifyEqual(point.lateral_residual_N, 100, ...
                AbsTol=1e-12);
        end

        function testMirroredStateMirrorsForcesMomentAndWheelLoads(testCase)
            leftState = fourWheelHandlingTest.makeState(20, 0.02, 0.3, 0.05);
            rightState = fourWheelHandlingTest.makeState(20, -0.02, -0.3, -0.05);

            left = evaluate_four_wheel_state(testCase.Vehicle, ...
                testCase.LinearTire, testCase.Aero, leftState);
            right = evaluate_four_wheel_state(testCase.Vehicle, ...
                testCase.LinearTire, testCase.Aero, rightState);

            testCase.verifyEqual(left.Fx_total_N, right.Fx_total_N, ...
                AbsTol=1e-9);
            testCase.verifyEqual(left.Fy_total_N, -right.Fy_total_N, ...
                AbsTol=1e-9);
            testCase.verifyEqual(left.yaw_moment_cg_Nm, ...
                -right.yaw_moment_cg_Nm, AbsTol=1e-9);
            testCase.verifyEqual(left.wheel.Fz_N, ...
                right.wheel.Fz_N([2 1 4 3]), AbsTol=1e-9);
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

        function state = makeState(speed_mps, beta_rad, yawRate_radps, steer_rad)
            state.speed_mps = speed_mps;
            state.beta_rad = beta_rad;
            state.yaw_rate_radps = yawRate_radps;
            state.steer_rad = steer_rad;
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

        function output = evaluateConstantTire(~)
            output.Fx_N = [100; 200; 300; 400];
            output.Fy_N = [10; 20; 30; 40];
            output.Mz_Nm = [1; 2; 3; 4];
        end
    end
end
