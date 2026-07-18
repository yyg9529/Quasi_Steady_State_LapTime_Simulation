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
    end
end
