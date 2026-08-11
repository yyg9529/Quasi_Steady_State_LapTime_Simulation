classdef qssltsDoeGuiIntegrationTest < matlab.unittest.TestCase
    properties (SetAccess = private)
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
        function testSynchronousDoeStoresSeparateResult(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            inputTable = findobj(app.UIFigure, Tag="doe-input-table");
            cases = inputTable.Data(2, :);

            app.runDoe(cases, true);
            doeResult = app.getLastDoeResult();
            lapResult = app.getLastResult();
            rankingTable = findobj(app.UIFigure, ...
                Tag="doe-ranking-table");

            testCase.verifyEqual(height(doeResult.ranking), 1);
            testCase.verifyEqual(doeResult.parameter_names, ...
                string(cases.Properties.VariableNames));
            testCase.verifyTrue(all(isfinite( ...
                doeResult.ranking.lap_time_s)));
            testCase.verifyEmpty(fieldnames(lapResult));
            testCase.verifyEqual(rankingTable.Data, doeResult.ranking);
            testCase.verifyEqual(app.ActivePage, "doe");
        end

        function testAsynchronousDoeCanBeCancelled(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            cases = table([290; 300; 310], VariableNames="mass_kg");

            app.runDoe(cases, false);
            runningBeforeCancel = app.isDoeRunning();
            action = @() app.runSimulation(false);

            testCase.verifyError(action, "QSSLTS:AnalysisRunning");
            app.cancelDoe();

            testCase.verifyTrue(runningBeforeCancel);
            testCase.verifyFalse(app.isDoeRunning());
        end
    end
end
