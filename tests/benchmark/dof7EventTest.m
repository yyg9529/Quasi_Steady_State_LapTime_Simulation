classdef dof7EventTest < matlab.unittest.TestCase
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
        function testAccelerationEventReachesTarget(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            options.target_speed_mps = 8;
            options.max_time_s = 3;

            result = run_7dof_accel_event( ...
                vehicle, tire, powertrain, brake, options);

            testCase.verifyTrue(result.target_reached);
            testCase.verifyEqual(result.termination_reason, "target_speed");
            testCase.verifyEqual(result.final_speed_mps, 8, AbsTol=1e-5);
            testCase.verifyGreaterThan(result.peak_accel_mps2, 0);
            testCase.verifyTrue(all(result.trace.load_solution_converged));
        end

        function testBrakeEventStopsWithoutWheelReversal(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            options.initial_speed_mps = 12;
            options.stop_speed_mps = 1;
            options.max_time_s = 4;

            result = run_7dof_brake_event( ...
                vehicle, tire, powertrain, brake, options);

            testCase.verifyTrue(result.stop_reached);
            testCase.verifyEqual(result.termination_reason, "stop_speed");
            testCase.verifyEqual(result.final_speed_mps, 1, AbsTol=1e-5);
            testCase.verifyGreaterThan(result.peak_decel_mps2, 0);
            testCase.verifyTrue(all(result.trace.load_solution_converged));
            testCase.verifyGreaterThanOrEqual(min(result.x(:, 4:7), [], "all"), ...
                -1e-6);
        end

        function testConstantRadiusSublimitCaseConverges(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            vehicle.mass.front_static_frac = 0.50;
            vehicle.mass.cg_height_m = 0;
            options.speed_grid_mps = 8;
            options.settle_time_s = 3;
            options.curvature_tolerance_frac = 0.20;

            result = run_7dof_constant_radius( ...
                20, vehicle, tire, powertrain, brake, options);

            testCase.verifyTrue(result.case_table.is_stable(1));
            testCase.verifyGreaterThan(result.max_stable_ay_mps2, 0);
            testCase.verifyEqual(result.case_table.achieved_ay_mps2(1), ...
                8^2 / 20, RelTol=0.15);
            testCase.verifyLessThan(abs(result.case_table.speed_error_mps(1)), ...
                0.5);
            testCase.verifyTrue(all( ...
                result.case_solutions{1}.trace.load_solution_converged));
            testCase.verifyFalse(result.limit_bracketed);
            testCase.verifyTrue(result.max_stable_is_lower_bound);
            testCase.verifyEqual( ...
                result.comparison.reason, "limit_not_bracketed");
        end


        function testAccelerationMatchesIndependentTractionBounds(testCase)
            vehicle = vehicle_baseline();
            vehicle.mass.cg_height_m = 0;
            tire = tire_simple_baseline();
            powertrain = powertrain_baseline();
            brake = brake_baseline();
            options.initial_speed_mps = 5;
            options.target_speed_mps = 12;
            options.max_time_s = 2;

            result = run_7dof_accel_event( ...
                vehicle, tire, powertrain, brake, options);
            gravity_mps2 = 9.80665;

            testCase.verifyTrue(result.target_reached);
            testCase.verifyGreaterThanOrEqual( ...
                result.terminal_accel_mps2, 0.80 * gravity_mps2);
            testCase.verifyLessThanOrEqual( ...
                result.terminal_accel_mps2, 0.91 * gravity_mps2);
            testCase.verifyGreaterThanOrEqual(result.event_time_s, 0.75);
            testCase.verifyLessThanOrEqual(result.event_time_s, 1.05);
        end

        function testAeroOffGgvComparisonIsAvailable(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            ggvOptions = default_qss_options();
            ggvOptions.v_grid_mps = [5; 10; 15];
            ggvOptions.ay_grid_g = -2.5:0.2:2.5;
            ggv = generate_model_ggv(vehicle, tire, ...
                struct("enabled", false), powertrain, brake, ggvOptions);
            accelOptions.initial_speed_mps = 5;
            accelOptions.target_speed_mps = 10;
            accelOptions.ggv = ggv;
            accelOptions.emit_warning = false;
            brakeOptions.initial_speed_mps = 15;
            brakeOptions.stop_speed_mps = 5;
            brakeOptions.ggv = ggv;
            brakeOptions.emit_warning = false;

            accel = run_7dof_accel_event( ...
                vehicle, tire, powertrain, brake, accelOptions);
            braking = run_7dof_brake_event( ...
                vehicle, tire, powertrain, brake, brakeOptions);

            testCase.verifyTrue(accel.comparison.available);
            testCase.verifyLessThan(accel.comparison.relative_difference, 0.20);
            testCase.verifyTrue(braking.comparison.available);
            testCase.verifyLessThan(braking.comparison.relative_difference, 0.20);
        end

        function testComparisonRejectsAeroOnGgv(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            ggvOptions = default_qss_options();
            ggvOptions.v_grid_mps = [5; 10];
            ggvOptions.ay_grid_g = -2.5:0.5:2.5;
            ggv = generate_model_ggv(vehicle, tire, ...
                aero_baseline(), powertrain, brake, ggvOptions);
            options.initial_speed_mps = 5;
            options.target_speed_mps = 6;
            options.ggv = ggv;
            options.emit_warning = false;

            action = @() run_7dof_accel_event( ...
                vehicle, tire, powertrain, brake, options);

            testCase.verifyError(action, "QSSLTS:DOF7GGVAero");
        end

        function testTimedOutEventIsNotCompared(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            options.target_speed_mps = 40;
            options.max_time_s = 0.02;

            result = run_7dof_accel_event( ...
                vehicle, tire, powertrain, brake, options);

            testCase.verifyFalse(result.target_reached);
            testCase.verifyFalse(result.comparison.available);
            testCase.verifyEqual( ...
                result.comparison.reason, "event_not_reached");
        end

        function testComparisonRejectsModelMismatch(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            ggvOptions = default_qss_options();
            ggvOptions.v_grid_mps = [5; 10];
            ggvOptions.ay_grid_g = -2.5:0.5:2.5;
            ggv = generate_model_ggv(vehicle, tire, ...
                struct("enabled", false), powertrain, brake, ggvOptions);
            changedVehicle = vehicle;
            changedVehicle.mass.total_kg = 320;
            options.initial_speed_mps = 5;
            options.target_speed_mps = 6;
            options.ggv = ggv;
            options.emit_warning = false;

            action = @() run_7dof_accel_event( ...
                changedVehicle, tire, powertrain, brake, options);

            testCase.verifyError(action, "QSSLTS:DOF7GGVModelMismatch");
        end

        function testRadiusRmsGateRejectsUnsettledWindow(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            options.speed_grid_mps = 18;
            options.settle_time_s = 1;
            options.analysis_window_s = 0.5;
            options.speed_tolerance_mps = 5;
            options.curvature_tolerance_frac = 1;
            options.yaw_accel_tolerance_radps2 = 1e-4;
            options.vy_dot_tolerance_mps2 = 1e-4;

            result = run_7dof_constant_radius( ...
                20, vehicle, tire, powertrain, brake, options);

            testCase.verifyFalse(result.case_table.is_stable(1));
            testCase.verifyGreaterThan( ...
                result.case_table.rms_yaw_accel_radps2(1), 1e-4);
            testCase.verifyGreaterThan( ...
                result.case_table.rms_vy_derivative_mps2(1), 1e-4);
        end

        function testCoarseRadiusBracketIsRefinedBeforeComparison(testCase)
            [vehicle, tire, powertrain, brake] = baselineModels();
            ggvOptions = default_qss_options();
            ggvOptions.v_grid_mps = (0:5:25).';
            ggvOptions.ay_grid_g = -2.5:0.1:2.5;
            ggv = generate_model_ggv(vehicle, tire, ...
                struct("enabled", false), powertrain, brake, ggvOptions);
            options.speed_grid_mps = [8; 20];
            options.settle_time_s = 3;
            options.curvature_tolerance_frac = 0.20;
            options.ggv = ggv;
            options.emit_warning = false;

            result = run_7dof_constant_radius( ...
                20, vehicle, tire, powertrain, brake, options);

            testCase.verifyTrue(result.limit_bracketed);
            testCase.verifyTrue(result.limit_refined);
            testCase.verifyLessThanOrEqual(result.bracket_width_mps, 0.5);
            testCase.verifyTrue(result.comparison.available);
            testCase.verifyLessThan(result.comparison.relative_difference, 0.25);
        end
    end
end

function [vehicle, tire, powertrain, brake] = baselineModels()
    vehicle = vehicle_baseline();
    tire = tire_load_sensitive_baseline();
    powertrain = powertrain_baseline();
    brake = brake_baseline();
end
