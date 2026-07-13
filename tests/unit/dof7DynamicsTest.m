classdef dof7DynamicsTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testStraightCoastIsEquilibrium(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            speed_mps = 10;
            wheelSpeed_radps = speed_mps / tire.rolling_radius_m;
            state = [speed_mps; 0; 0; repmat(wheelSpeed_radps, 4, 1)];
            input = struct("steer_rad", 0, "throttle", 0, "brake", 0);

            [dx, diagnostics] = vehicle_7dof_ode(0, state, input, ...
                vehicle, tire, powertrain, brake);

            testCase.verifyEqual(dx, zeros(7, 1), AbsTol=1e-10);
            testCase.verifyTrue(diagnostics.load_solution_converged);
            testCase.verifyLessThanOrEqual( ...
                diagnostics.load_solution_residual_mps2, 1e-5);
        end

        function testRwdTorqueActsOnlyOnRearWheelsAtRest(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            input = struct("steer_rad", 0, "throttle", 1, "brake", 0);

            dx = vehicle_7dof_ode(0, zeros(7, 1), input, ...
                vehicle, tire, powertrain, brake);
            expectedRearAlpha_radps2 = ...
                0.5 * powertrain.max_total_wheel_torque_Nm ...
                / tire.wheel_inertia_kgm2;

            testCase.verifyEqual(dx(4:5), zeros(2, 1), AbsTol=0);
            testCase.verifyEqual(dx(6:7), ...
                expectedRearAlpha_radps2 * ones(2, 1), AbsTol=1e-12);
        end

        function testBrakeBiasSetsInitialWheelDecelerationRatio(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            speed_mps = 12;
            wheelSpeed_radps = speed_mps / tire.rolling_radius_m;
            state = [speed_mps; 0; 0; repmat(wheelSpeed_radps, 4, 1)];
            input = struct("steer_rad", 0, "throttle", 0, "brake", 1);

            dx = vehicle_7dof_ode(0, state, input, ...
                vehicle, tire, powertrain, brake);
            frontRearRatio = mean(abs(dx(4:5))) / mean(abs(dx(6:7)));

            testCase.verifyEqual(frontRearRatio, ...
                brake.front_bias / (1 - brake.front_bias), RelTol=1e-9);
            testCase.verifyLessThan(dx(4:7), zeros(4, 1));
        end

        function testLeftRightMirrorSymmetry(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            wheelSpeed_radps = 12 / tire.rolling_radius_m;
            leftState = [12; 0.5; 0.2; repmat(wheelSpeed_radps, 4, 1)];
            rightState = [12; -0.5; -0.2; repmat(wheelSpeed_radps, 4, 1)];
            leftInput = struct("steer_rad", 0.05, ...
                "throttle", 0, "brake", 0);
            rightInput = struct("steer_rad", -0.05, ...
                "throttle", 0, "brake", 0);

            leftDx = vehicle_7dof_ode(0, leftState, leftInput, ...
                vehicle, tire, powertrain, brake);
            rightDx = vehicle_7dof_ode(0, rightState, rightInput, ...
                vehicle, tire, powertrain, brake);

            testCase.verifyEqual(rightDx(1), leftDx(1), AbsTol=1e-10);
            testCase.verifyEqual(rightDx(2:3), -leftDx(2:3), AbsTol=1e-10);
            testCase.verifyEqual(rightDx(4:7), ...
                leftDx([5, 4, 7, 6]), AbsTol=1e-10);
        end

        function testLowSpeedDynamicsRemainFinite(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            state = [1e-9; 1e-9; 1e-9; zeros(4, 1)];
            input = struct("steer_rad", 0.01, ...
                "throttle", 0.2, "brake", 0);

            dx = vehicle_7dof_ode(0, state, input, ...
                vehicle, tire, powertrain, brake);

            testCase.verifyTrue(all(isfinite(dx)));
        end

        function testDriveAndBrakeHaveCorrectEnergySigns(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            speed_mps = 10;
            wheelSpeed_radps = speed_mps / tire.rolling_radius_m;
            state = [speed_mps; 0; 0; repmat(wheelSpeed_radps, 4, 1)];
            driveInput = struct("steer_rad", 0, ...
                "throttle", 0.5, "brake", 0);
            brakeInput = struct("steer_rad", 0, ...
                "throttle", 0, "brake", 0.3);

            driveDx = vehicle_7dof_ode(0, state, driveInput, ...
                vehicle, tire, powertrain, brake);
            brakeDx = vehicle_7dof_ode(0, state, brakeInput, ...
                vehicle, tire, powertrain, brake);
            drivePower_W = vehicle.mass.total_kg * speed_mps * driveDx(1) ...
                + tire.wheel_inertia_kgm2 ...
                * sum(state(4:7) .* driveDx(4:7));
            brakePower_W = vehicle.mass.total_kg * speed_mps * brakeDx(1) ...
                + tire.wheel_inertia_kgm2 ...
                * sum(state(4:7) .* brakeDx(4:7));

            testCase.verifyGreaterThan(drivePower_W, 0);
            testCase.verifyLessThan(brakePower_W, 0);
        end

        function testRegenCannotBeSilentlyIgnored(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            brake.regen_enabled = true;
            state = make_7dof_initial_state(10, tire);
            input = struct("steer_rad", 0, "throttle", 0, "brake", 0.2);

            action = @() vehicle_7dof_ode( ...
                0, state, input, vehicle, tire, powertrain, brake);

            testCase.verifyError(action, "QSSLTS:DOF7Regen");
        end

        function testLoadFixedPointCannotFailSilently(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            vehicle.mass.cg_height_m = 0.40;
            tire.load_sensitivity_x = -0.60;
            tire.load_sensitivity_y = -0.60;
            state = make_7dof_initial_state(15, tire);
            state(2) = 2;
            state(3) = 0.8;
            state(4:7) = 1.3 * state(4:7);
            input = struct("steer_rad", 0.15, ...
                "throttle", 0, "brake", 0);

            action = @() vehicle_7dof_ode( ...
                0, state, input, vehicle, tire, powertrain, brake);

            testCase.verifyError(action, "QSSLTS:DOF7LoadConvergence");
        end

        function testBrakeDecelerationCapLimitsWheelTorque(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            brake.max_decel_g_mechanical = 0.20;
            brake.max_total_brake_force_N = inf;
            brake.max_total_brake_torque_Nm = inf;
            state = make_7dof_initial_state(10, tire);
            input = struct("steer_rad", 0, "throttle", 0, "brake", 1);

            [~, diagnostics] = vehicle_7dof_ode( ...
                0, state, input, vehicle, tire, powertrain, brake);
            expectedTorque_Nm = 0.20 * vehicle.mass.total_kg ...
                * 9.80665 * tire.rolling_radius_m;

            testCase.verifyEqual(sum(diagnostics.brake_torque_Nm), ...
                expectedTorque_Nm, RelTol=1e-12);
        end

        function testBrakeForceCapLimitsWheelTorque(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            brake.max_decel_g_mechanical = inf;
            brake.max_total_brake_force_N = 1000;
            brake.max_total_brake_torque_Nm = inf;
            state = make_7dof_initial_state(10, tire);
            input = struct("steer_rad", 0, "throttle", 0, "brake", 1);

            [~, diagnostics] = vehicle_7dof_ode( ...
                0, state, input, vehicle, tire, powertrain, brake);
            expectedTorque_Nm = 1000 * tire.rolling_radius_m;

            testCase.verifyEqual(sum(diagnostics.brake_torque_Nm), ...
                expectedTorque_Nm, RelTol=1e-12);
        end

        function testThrottleRequiresEnabledPowertrain(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            powertrain.enabled = false;
            input = struct("steer_rad", 0, "throttle", 0.2, "brake", 0);

            action = @() vehicle_7dof_ode( ...
                0, zeros(7, 1), input, vehicle, tire, powertrain, brake);

            testCase.verifyError(action, "QSSLTS:DOF7Powertrain");
        end

        function testDrivetrainLayoutsMustMatch(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            powertrain.layout = "FWD";
            input = struct("steer_rad", 0, "throttle", 0, "brake", 0);

            action = @() vehicle_7dof_ode( ...
                0, zeros(7, 1), input, vehicle, tire, powertrain, brake);

            testCase.verifyError(action, "QSSLTS:DOF7DrivetrainMismatch");
        end

        function testInvalidBrakeBiasIsRejected(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            brake.front_bias = 1.2;
            input = struct("steer_rad", 0, "throttle", 0, "brake", 0);

            action = @() vehicle_7dof_ode( ...
                0, zeros(7, 1), input, vehicle, tire, powertrain, brake);

            testCase.verifyError(action, "QSSLTS:DOF7Brake");
        end

        function testNaNVehicleParameterIsRejected(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            vehicle.mass.total_kg = NaN;
            input = struct("steer_rad", 0, "throttle", 0, "brake", 0);

            action = @() vehicle_7dof_ode( ...
                0, zeros(7, 1), input, vehicle, tire, powertrain, brake);

            testCase.verifyError(action, "QSSLTS:DOF7Vehicle");
        end

        function testExternalTireAdapterCanReplaceConstitutiveModel(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            tire.slip_force = make_unitire_placeholder("simple");
            tire.slip_force.metadata.source = "unit-test fixture";
            tire.slip_force.metadata.evaluator_id = ...
                "unit_test_constant_force_v1";
            tire.slip_force.metadata.native_tire_side = "SYMMETRIC";
            tire.slip_force.metadata.source_coordinate_convention = ...
                "QSSLTS_X_FORWARD_Y_LEFT_Z_UP";
            tire.slip_force.metadata.valid_domain = wideTireDomain();
            tire.slip_force.evaluator = @constantTireEvaluator;
            state = make_7dof_initial_state(10, tire);
            input = struct("steer_rad", 0, "throttle", 0, "brake", 0);

            [dx, diagnostics] = vehicle_7dof_ode( ...
                0, state, input, vehicle, tire, powertrain, brake);

            testCase.verifyEqual(dx(1), ...
                40 / vehicle.mass.total_kg, AbsTol=1e-10);
            testCase.verifyEqual(dx(2:3), zeros(2, 1), AbsTol=1e-10);
            testCase.verifyEqual(dx(4:7), ...
                -10 * tire.rolling_radius_m / tire.wheel_inertia_kgm2 ...
                * ones(4, 1), AbsTol=1e-10);
            testCase.verifyFalse(any(diagnostics.tire_extrapolated));
        end

        function testExternalExtrapolationIsExposedInDiagnostics(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            tire.slip_force = configuredExternalTire();
            tire.slip_force.metadata.valid_domain.wheel_vx_mps = [0, 5];
            tire.slip_force.metadata.extrapolation_policy = "allow";
            state = make_7dof_initial_state(10, tire);
            input = struct("steer_rad", 0, "throttle", 0, "brake", 0);

            [~, diagnostics] = vehicle_7dof_ode( ...
                0, state, input, vehicle, tire, powertrain, brake);

            testCase.verifyTrue(all(diagnostics.tire_extrapolated));
        end
    end
end

function [vehicle, tire, powertrain, brake] = baselineModels()
    vehicle = vehicle_baseline();
    tire = tire_load_sensitive_baseline();
    powertrain = powertrain_baseline();
    brake = brake_baseline();
end

function domain = wideTireDomain()
    domain.slip_ratio = [-1, 1];
    domain.slip_angle_rad = [-1, 1];
    domain.Fz_N = [1, 2000];
    domain.camber_rad = [0, 0];
    domain.wheel_vx_mps = [-100, 100];
end

function slipForce = configuredExternalTire()
    slipForce = make_unitire_placeholder("simple");
    slipForce.metadata.source = "unit-test fixture";
    slipForce.metadata.evaluator_id = "unit_test_constant_force_v1";
    slipForce.metadata.native_tire_side = "SYMMETRIC";
    slipForce.metadata.source_coordinate_convention = ...
        "QSSLTS_X_FORWARD_Y_LEFT_Z_UP";
    slipForce.metadata.valid_domain = wideTireDomain();
    slipForce.evaluator = @constantTireEvaluator;
end

function force = constantTireEvaluator(~, ~)
    force.Fx_N = 10;
    force.Fy_N = 0;
end
