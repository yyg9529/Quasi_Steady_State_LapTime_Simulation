classdef qssltsDoeValidityGuiTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            folders = ["app", "src", "data", "preprocessing"];
            for folder = folders
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                    fullfile(projectRoot, folder), IncludingSubfolders=true));
            end
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                projectRoot));
        end
    end

    methods (TestMethodSetup)
        function closeFigures(testCase)
            testCase.addTeardown(@() close("all", "force"));
        end
    end

    methods (Test)
        function testPartialDoeShowsCountsAndPlotsOnlyValidCases(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            drawnow;
            inputTable = findobj(app.UIFigure, Tag="doe-input-table");
            validCase = inputTable.Data(2, :);
            invalidCase = validCase;
            invalidCase.mass_kg = -1;
            cases = [invalidCase; validCase];

            app.runDoe(cases, true);
            result = app.getLastDoeResult();
            statusLabel = findobj(app.UIFigure, Tag="doe-status");
            response = findobj(app.UIFigure, ...
                Tag="doe-lap-time-response");

            testCase.verifyEqual(result.status, "partial");
            testCase.verifyNotEmpty(statusLabel);
            testCase.verifyTrue(contains(string(statusLabel.Text), ...
                "1 valid / 1 rejected"));
            testCase.verifyNumElements(response.YData, 1);
            testCase.verifyEqual(response.YData, ...
                result.ranking.lap_time_s.', AbsTol=0);
        end

        function testNoValidDoeShowsEmptyStateWithoutResponseLine(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            drawnow;
            inputTable = findobj(app.UIFigure, Tag="doe-input-table");
            cases = inputTable.Data([1; 3], :);
            cases.mass_kg = [-1; -2];

            app.runDoe(cases, true);
            result = app.getLastDoeResult();
            statusLabel = findobj(app.UIFigure, Tag="doe-status");
            response = findobj(app.UIFigure, ...
                Tag="doe-lap-time-response");
            emptyMessage = findobj(app.UIFigure, ...
                Tag="doe-no-valid-message");

            testCase.verifyEqual(result.status, "no_valid_cases");
            testCase.verifyNotEmpty(statusLabel);
            testCase.verifyTrue(contains(string(statusLabel.Text), ...
                "0 valid / 2 rejected"));
            testCase.verifyEmpty(response);
            testCase.verifyNotEmpty(emptyMessage);
        end
    end
end
