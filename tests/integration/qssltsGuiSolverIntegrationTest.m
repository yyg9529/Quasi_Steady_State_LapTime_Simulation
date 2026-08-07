classdef qssltsGuiSolverIntegrationTest < matlab.unittest.TestCase
    properties
        ProjectRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            folders = ["app", "src", "data", "preprocessing"];
            for folder = folders
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                    fullfile(testCase.ProjectRoot, folder), ...
                    IncludingSubfolders=true));
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
        function testSynchronousRunStoresAndRendersResult(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            app.runSimulation(true);
            result = app.getLastResult();
            summary = app.getLastSummary();
            kpi = findobj(app.UIFigure, Tag="kpi-cards");

            testCase.verifyTrue(result.solver.converged);
            testCase.verifyGreaterThan(result.lap_time_s, 0);
            testCase.verifyEqual(summary.lap_time_s, ...
                result.lap_time_s, AbsTol=1e-12);
            testCase.verifyEqual(app.ActivePage, "results");
            testCase.verifyNotEqual(string(kpi.Data.lapTime), "—");
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-track-speed-line"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-energy-line"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ggv-envelope-surface"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-actual-ggv-points"));
            speedLine = findobj(app.UIFigure, ...
                Tag="result-dynamics-speed-line");
            testCase.verifyEqual(speedLine.YData(:), result.v_mps(:), ...
                AbsTol=1e-12);
            testCase.verifyEqual(speedLine.Color, [0.15 0.72 0.38], ...
                AbsTol=1e-12);
        end

        function testSynchronousRunGeneratesHandlingFromLocalTir(testCase)
            tirFile = fullfile(testCase.ProjectRoot, "data", "tire", ...
                "local", "Hoosier_16x75_10_R20.tir");
            testCase.assumeTrue(isfile(tirFile));
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            app.runSimulation(true);
            handling = app.getLastHandlingResult();

            testCase.verifyTrue(handling.available);
            testCase.verifyTrue(all(handling.ymd.converged, "all"));
            testCase.verifyTrue(all(handling.ymd.within_tire_range, "all"));
            testCase.verifyTrue(all(handling.understeer.converged));
            testCase.verifyTrue(all( ...
                handling.understeer.within_tire_range));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ymd-delta-mesh"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ymd-beta-mesh"));
            testCase.verifyNumElements(findall(app.UIFigure, ...
                Tag="result-ymd-delta-label"), ...
                size(handling.ymd.steer_rad, 2));
            testCase.verifyNumElements(findall(app.UIFigure, ...
                Tag="result-ymd-beta-label"), ...
                size(handling.ymd.beta_rad, 1));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-understeer-line"));
        end

        function testAsyncRunCanBeCancelled(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            app.runSimulation(false);
            runningBeforeCancel = app.isSimulationRunning();
            app.cancelSimulation();

            testCase.verifyTrue(runningBeforeCancel);
            testCase.verifyFalse(app.isSimulationRunning());
        end

        function testRunButtonIsConnected(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            runButton = findobj(app.UIFigure, Tag="run-simulation");

            testCase.verifyNotEmpty(runButton);
            testCase.verifyEqual(string(runButton.Text), "开始仿真");
        end
    end
end
