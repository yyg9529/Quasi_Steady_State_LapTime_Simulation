classdef qssltsGuiRuntimeConfigTest < matlab.unittest.TestCase
    properties
        ProjectRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "app"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "src"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "data"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                testCase.ProjectRoot));
        end
    end

    methods (TestMethodSetup)
        function closeFigures(testCase)
            testCase.addTeardown(@() close("all", "force"));
        end
    end

    methods (Test)
        function testBaselineMapsToRuntimeConfig(testCase)
            state = qssltsGuiRuntimeConfigTest.baselineState();

            config = qsslts_gui_state_to_runtime_config( ...
                state, testCase.ProjectRoot);

            testCase.verifyEqual(config.track.length_m, ...
                857.461445744691, AbsTol=1e-9);
            testCase.verifyEqual(config.vehicle.mass.total_kg, ...
                300, AbsTol=1e-12);
            testCase.verifyEqual(config.vehicle.geometry.wheelbase_m, ...
                1.668, AbsTol=1e-12);
            testCase.verifyEqual(config.models.tire.mu_x_ref, ...
                1.60, AbsTol=1e-12);
            testCase.verifyEqual(config.models.aero.CLA_m2, ...
                3.0, AbsTol=1e-12);
            testCase.verifyEqual(config.models.powertrain.gear_ratio, ...
                4.369334602435052, AbsTol=1e-14);
            testCase.verifyEqual(config.models.brake.front_bias, ...
                0.60, AbsTol=1e-12);
            testCase.verifyEqual(config.models.endurance.num_laps, 26);
            testCase.verifyEqual(config.options.v_grid_mps, ...
                (0:0.5:45).', AbsTol=1e-12);
        end

        function testSpeedGridIncludesNondivisibleMaximum(testCase)
            state = qssltsGuiRuntimeConfigTest.baselineState();
            state.options_v_max_mps = 1.0;
            state.options_v_grid_step_mps = 0.3;

            config = qsslts_gui_state_to_runtime_config( ...
                state, testCase.ProjectRoot);

            testCase.verifyEqual(config.options.v_grid_mps, ...
                [0; 0.3; 0.6; 0.9; 1.0], AbsTol=1e-12);
        end

        function testUnsupportedPresetErrors(testCase)
            state = qssltsGuiRuntimeConfigTest.baselineState();
            state.vehicle_preset = "unknown_vehicle";

            action = @() qsslts_gui_state_to_runtime_config( ...
                state, testCase.ProjectRoot);

            testCase.verifyError(action, "QSSLTS:GuiConfig");
        end

        function testPathTraversalErrors(testCase)
            state = qssltsGuiRuntimeConfigTest.baselineState();
            state.track_source_file = "../outside.csv";

            action = @() qsslts_gui_state_to_runtime_config( ...
                state, testCase.ProjectRoot);

            testCase.verifyError(action, "QSSLTS:GuiConfig");
        end

        function testAppBuildsRuntimeConfig(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            config = app.buildRuntimeConfig();

            testCase.verifyEqual(config.models.powertrain.layout, "RWD");
            testCase.verifyEqual(config.options.v_max_mps, ...
                45, AbsTol=1e-12);
        end

        function testAdvancedParametersMapToRuntimeConfig(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            state = app.getParameterState();
            state.vehicle_track_front_m = 1.31;
            state.vehicle_track_rear_m = 1.29;
            state.vehicle_inertia_Iz_kgm2 = 135;
            state.tire_load_sensitivity_x = -0.11;
            state.brake_force_limit_enabled = true;
            state.brake_max_total_brake_force_N = 7200;
            state.motor_peak_torque_Nm = 205;
            state.motor_eta_const = 0.93;
            state.battery_E_nominal_kWh = 9.2;
            state.battery_V_bus_assumed_V = 580;
            state.inverter_I_phase_peak_Arms = 230;

            config = qsslts_gui_state_to_runtime_config( ...
                state, testCase.ProjectRoot);

            testCase.verifyEqual(config.vehicle.geometry.track_front_m, ...
                1.31, AbsTol=1e-12);
            testCase.verifyEqual(config.vehicle.geometry.track_rear_m, ...
                1.29, AbsTol=1e-12);
            testCase.verifyEqual(config.vehicle.inertia.Iz_kgm2, ...
                135, AbsTol=1e-12);
            testCase.verifyEqual( ...
                config.models.tire.load_sensitivity_x, ...
                -0.11, AbsTol=1e-12);
            testCase.verifyEqual( ...
                config.models.brake.max_total_brake_force_N, ...
                7200, AbsTol=1e-12);
            testCase.verifyEqual( ...
                config.models.powertrain.motor.peak_torque_Nm, ...
                205, AbsTol=1e-12);
            testCase.verifyEqual( ...
                config.models.powertrain.motor.eta_const, ...
                0.93, AbsTol=1e-12);
            testCase.verifyEqual( ...
                config.models.powertrain.battery.E_nominal_kWh, ...
                9.2, AbsTol=1e-12);
            testCase.verifyEqual( ...
                config.models.powertrain.battery.V_bus_assumed_V, ...
                580, AbsTol=1e-12);
            testCase.verifyEqual( ...
                config.models.powertrain.inverter.I_phase_peak_Arms, ...
                230, AbsTol=1e-12);
        end
    end

    methods (Static, Access = private)
        function state = baselineState()
            state = qsslts_gui_default_state();
        end
    end
end
