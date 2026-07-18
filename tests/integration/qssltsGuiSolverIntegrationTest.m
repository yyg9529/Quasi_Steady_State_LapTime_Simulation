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
                Tag="result-ggv-upper-surface"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ggv-lower-surface"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-actual-ggv-points"));
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
