classdef qssltsDoePageTest < matlab.unittest.TestCase
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
        function testDoeIsIndependentTopLevelPage(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            app.selectPage("doe");
            page = findobj(app.UIFigure, Tag="page-doe");

            testCase.verifyNotEmpty(page);
            testCase.verifyEqual(app.ActivePage, "doe");
            testCase.verifyEqual(string(page.Visible), "on");
            testCase.verifyNotEmpty(findobj(app.UIFigure, Tag="run-doe"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="doe-ranking-table"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="doe-response-axes"));
        end

        function testDoeResultStateStartsEmptyAndSeparate(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            doeResult = app.getLastDoeResult();
            lapResult = app.getLastResult();

            testCase.verifyEmpty(fieldnames(doeResult));
            testCase.verifyEmpty(fieldnames(lapResult));
        end
    end
end
