classdef qssltsGuiConfigTest < matlab.unittest.TestCase
    properties
        ProjectRoot
    end

    methods (TestClassSetup)
        function addAppPath(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "app"), ...
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
        function testJsonRoundTripPreservesParameterState(testCase)
            state = qssltsGuiConfigTest.baselineState();
            filePath = testCase.temporaryJsonFile();

            write_qsslts_gui_config(filePath, state);
            actual = read_qsslts_gui_config(filePath);

            testCase.verifyEqual(actual, state);
        end

        function testJsonDocumentHasVersionedGroupedSchema(testCase)
            filePath = testCase.temporaryJsonFile();
            write_qsslts_gui_config(filePath, ...
                qssltsGuiConfigTest.baselineState());

            document = jsondecode(fileread(filePath));

            testCase.verifyEqual(string(document.schema), ...
                "qsslts-gui-parameters");
            testCase.verifyEqual(document.schema_version, 2);
            testCase.verifyEqual(string(document.units), "SI");
            requiredGroups = ["track", "endurance", "vehicle", ...
                "tire", "aero", "brake", "powertrain", "options"];
            testCase.verifyTrue(all(isfield(document, requiredGroups)));
            testCase.verifyEqual(string(document.track.source_file), ...
                "data/track/tianji_kart_QSS_track_closed.csv");
            testCase.verifyEqual(string(document.vehicle.preset), ...
                "vehicle_baseline");
            testCase.verifyEqual(string(document.tire.preset), ...
                "tire_load_sensitive_baseline");
            testCase.verifyEqual(string(document.aero.preset), ...
                "aero_baseline");
            testCase.verifyEqual(string(document.brake.preset), ...
                "brake_baseline");
        end

        function testInvalidNumericParameterIsRejected(testCase)
            state = qssltsGuiConfigTest.baselineState();
            state.vehicle_mass_total_kg = -1;

            action = @() write_qsslts_gui_config( ...
                testCase.temporaryJsonFile(), state);

            testCase.verifyError(action, "QSSLTS:GuiConfig");
        end

        function testSchemaV2StoresAdvancedPowertrainGroups(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            filePath = testCase.temporaryJsonFile();

            write_qsslts_gui_config(filePath, app.getParameterState());
            document = jsondecode(fileread(filePath));

            testCase.verifyEqual(document.schema_version, 2);
            testCase.verifyTrue(isfield(document.powertrain, "motor"));
            testCase.verifyTrue(isfield(document.powertrain, "battery"));
            testCase.verifyTrue(isfield(document.powertrain, "inverter"));
            testCase.verifyTrue(isfield(document.brake, ...
                "force_limit_enabled"));
        end

        function testInvalidAdvancedParameterIsRejected(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            state = app.getParameterState();
            state.brake_front_bias = 1.2;

            action = @() write_qsslts_gui_config( ...
                testCase.temporaryJsonFile(), state);

            testCase.verifyError(action, "QSSLTS:GuiConfig");
        end

        function testVersion1ConfigurationMigratesToAdvancedDefaults(testCase)
            filePath = testCase.temporaryJsonFile();
            qssltsGuiConfigTest.writeVersion1Configuration(filePath);

            state = read_qsslts_gui_config(filePath);

            testCase.verifyEqual(state.vehicle_track_front_m, ...
                1.2, AbsTol=1e-12);
            testCase.verifyEqual(state.motor_peak_torque_Nm, ...
                220, AbsTol=1e-12);
            testCase.verifyEqual(state.battery_E_nominal_kWh, ...
                8, AbsTol=1e-12);
            testCase.verifyFalse(state.brake_force_limit_enabled);
        end

        function testAppImportsAndExportsParameterFile(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            state = app.getParameterState();
            state.vehicle_mass_total_kg = 312.5;
            inputFile = testCase.temporaryJsonFile();
            outputFile = testCase.temporaryJsonFile();
            write_qsslts_gui_config(inputFile, state);

            app.importParameters(inputFile);
            app.exportParameters(outputFile);

            testCase.verifyEqual( ...
                app.getParameterState().vehicle_mass_total_kg, 312.5);
            testCase.verifyEqual(read_qsslts_gui_config(outputFile), state);
        end
    end

    methods (Access = private)
        function filePath = temporaryJsonFile(testCase)
            filePath = string(tempname) + ".json";
            testCase.addTeardown(@() ...
                qssltsGuiConfigTest.deleteIfExists(filePath));
        end
    end

    methods (Static, Access = private)
        function state = baselineState()
            state = qsslts_gui_default_state();
        end

        function deleteIfExists(filePath)
            if isfile(filePath)
                delete(filePath);
            end
        end

        function writeVersion1Configuration(filePath)
            write_qsslts_gui_config(filePath, ...
                qsslts_gui_default_state());
            document = jsondecode(fileread(filePath));
            document.schema_version = 1;
            document.vehicle = rmfield(document.vehicle, ...
                {'track_front_m', 'track_rear_m', 'inertia_Iz_kgm2', ...
                'front_lateral_load_transfer_frac'});
            document.tire = rmfield(document.tire, ...
                {'Fz_ref_N', 'load_sensitivity_x', ...
                'load_sensitivity_y', 'combined_n'});
            document.aero = rmfield(document.aero, ...
                "front_downforce_frac");
            document.brake = struct(preset=document.brake.preset);
            document.powertrain = struct( ...
                preset=document.powertrain.preset, ...
                gear_ratio=document.powertrain.gear_ratio);
            jsonText = jsonencode(document, PrettyPrint=true);
            fileId = fopen(filePath, "w", "n", "UTF-8");
            cleanup = onCleanup(@() fclose(fileId));
            fprintf(fileId, "%s\n", jsonText);
            clear cleanup
        end
    end
end
