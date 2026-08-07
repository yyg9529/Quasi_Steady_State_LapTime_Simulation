classdef qssltsAppTest < matlab.unittest.TestCase
    properties
        ProjectRoot
    end

    methods (TestClassSetup)
        function addAppPath(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            appFolder = fullfile(testCase.ProjectRoot, "app");
            if isfolder(appFolder)
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                    appFolder, IncludingSubfolders=true));
            end
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
        function testAppCreatesRequiredViews(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            testCase.verifyTrue(isgraphics(app.UIFigure, "figure"));
            testCase.verifyEqual(string(app.UIFigure.Name), ...
                "QSSLTS Engineering Studio");
            requiredTags = ["page-dashboard", "page-parameters", ...
                "page-run", "page-results", "kpi-cards", ...
                "run-simulation", "load-parameters", ...
                "save-parameters"];
            for tag = requiredTags
                testCase.verifyNotEmpty(findobj(app.UIFigure, Tag=tag), ...
                    "Missing GUI component: " + tag);
            end
            testCase.verifyEqual(app.ActivePage, "dashboard");
        end

        function testPageNavigationChangesVisiblePanel(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            app.selectPage("parameters");

            testCase.verifyEqual(app.ActivePage, "parameters");
            parametersPage = findobj(app.UIFigure, ...
                Tag="page-parameters");
            dashboardPage = findobj(app.UIFigure, ...
                Tag="page-dashboard");
            testCase.verifyEqual(string(parametersPage.Visible), "on");
            testCase.verifyEqual(string(dashboardPage.Visible), "off");
        end

        function testParameterCardsGridIsScrollable(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            cards = findobj(app.UIFigure, Tag="parameter-cards-grid");

            testCase.verifyNotEmpty(cards);
            testCase.verifyEqual(string(cards.Scrollable), "on");
        end

        function testAppUsesAnalysisEntryPoint(testCase)
            source = string(fileread(which("QssltsApp")));

            testCase.verifyTrue(contains(source, "run_analysis_case("));
            testCase.verifyFalse(contains(source, "run_qss_lap("));
        end

        function testAdvancedVehicleParametersAreExposed(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            state = app.getParameterState();
            required = ["vehicle_track_front_m", ...
                "vehicle_track_rear_m", "vehicle_inertia_Iz_kgm2", ...
                "tire_load_sensitivity_x", "brake_front_bias", ...
                "motor_peak_torque_Nm", "battery_E_nominal_kWh", ...
                "inverter_I_phase_peak_Arms"];

            testCase.verifyTrue(all(isfield(state, required)));
        end

        function testLauncherReturnsApp(testCase)
            app = launch_qsslts_app(false);
            testCase.addTeardown(@() delete(app));

            testCase.verifyClass(app, "QssltsApp");
            testCase.verifyEqual(string(app.UIFigure.Visible), "off");
        end

        function testDirectConstructionRestoresTrackReaderPath(testCase)
            originalPath = path;
            testCase.addTeardown(@() path(originalPath));
            trackFolder = fullfile(testCase.ProjectRoot, ...
                "preprocessing", "track");
            rmpath(trackFolder);
            clear("read_track_csv");
            testCase.verifyEmpty(which("read_track_csv"));

            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            config = app.buildRuntimeConfig();

            testCase.verifyNotEmpty(which("read_track_csv"));
            testCase.verifyEqual(config.track.source_file, ...
                fullfile(testCase.ProjectRoot, "data", "track", ...
                    "tianji_kart_QSS_track_closed.csv"));
        end


        function testHefeiTrackPresetUpdatesSource(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            dropdown = findobj(app.UIFigure, Tag="param-track_preset");

            preset = "fsec_hefei_2025_high_speed_avoidance_closed";
            testCase.verifyTrue(ismember(preset, string(dropdown.ItemsData)));

            dropdown.Value = preset;
            dropdown.ValueChangedFcn(dropdown, []);
            state = app.getParameterState();

            testCase.verifyEqual(state.track_preset, preset);
            testCase.verifyEqual(state.track_source_file, ...
                "data/track/fsec_hefei_2025_high_speed_avoidance_closed.csv");
            testCase.verifyEqual(state.endurance_num_laps, 1);
        end

        function testDynamicEventPresetsUpdateSource(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            dropdown = findobj(app.UIFigure, Tag="param-track_preset");
            presets = ["fsc_2025_acceleration_open", ...
                "fsc_2025_skidpad_event_open"];
            sources = ["data/track/fsc_2025_acceleration_open.csv", ...
                "data/track/fsc_2025_skidpad_event_open.csv"];

            for index = 1:numel(presets)
                testCase.verifyTrue(ismember( ...
                    presets(index), string(dropdown.ItemsData)));
                dropdown.Value = presets(index);
                dropdown.ValueChangedFcn(dropdown, []);
                state = app.getParameterState();
                testCase.verifyEqual(state.track_preset, presets(index));
                testCase.verifyEqual(state.track_source_file, ...
                    sources(index));
                testCase.verifyEqual(state.endurance_num_laps, 1);
            end
        end

        function testDynamicEventKpiUsesEventAndPathLabels(testCase)
            result.event.type = "fsc_skidpad";
            result.energy.E_lap_stored_kWh = 0.12;
            result.solver.converged = true;
            result.solver.iterations = 3;
            summary.lap_time_s = 4.5;
            summary.max_speed_mps = 15;

            data = qsslts_kpi_data(result, summary);

            testCase.verifyEqual(data.timeLabel, "Event time");
            testCase.verifyEqual(data.timeMeta, "赛事计时时间");
            testCase.verifyEqual(data.energyLabel, "Path energy");
            testCase.verifyEqual(data.energyMeta, ...
                "完整开放路径储能侧能量");
        end

        function testTrackImportPreservesExplicitLapCount(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            state = app.getParameterState();
            state.track_preset = ...
                "fsec_hefei_2025_high_speed_avoidance_closed";
            state.track_source_file = ...
                "data/track/fsec_hefei_2025_high_speed_avoidance_closed.csv";
            state.endurance_num_laps = 4;
            filename = string(tempname) + ".json";
            testCase.addTeardown(@() delete(filename));
            write_qsslts_gui_config(filename, state);

            app.importParameters(filename);
            imported = app.getParameterState();

            testCase.verifyEqual(imported.track_preset, ...
                state.track_preset);
            testCase.verifyEqual(imported.track_source_file, ...
                state.track_source_file);
            testCase.verifyEqual(imported.endurance_num_laps, 4);
        end
    end
end
