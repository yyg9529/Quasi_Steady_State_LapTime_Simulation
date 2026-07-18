classdef qssltsHandlingPageTest < matlab.unittest.TestCase
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
        function testHandlingStateStartsEmptyAndAxesExist(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));

            result = app.getLastHandlingResult();

            testCase.verifyEmpty(fieldnames(result));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ymd-axes"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-understeer-axes"));
        end

        function testHandlingResultRendersWithoutReplacingLapResult(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            handling = qssltsHandlingPageTest.makeHandlingResult();

            app.showHandlingResult(handling.ymd, handling.understeer);
            stored = app.getLastHandlingResult();

            testCase.verifyEqual(stored, handling);
            testCase.verifyEmpty(fieldnames(app.getLastResult()));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ymd-surface"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-understeer-line"));
            testCase.verifyEqual(app.ActivePage, "results");
        end
    end

    methods (Static, Access = private)
        function handling = makeHandlingResult()
            handling.ymd.beta_rad = deg2rad([-2; 0; 2]);
            handling.ymd.steer_rad = deg2rad([-4, 0, 4]);
            handling.ymd.ay_mps2 = [-4 0 4; -3 0 3; -2 0 2];
            handling.ymd.yaw_moment_cg_Nm = ...
                [-80 0 80; -60 0 60; -40 0 40];
            handling.understeer.ay_g = [0.1; 0.4; 0.8];
            handling.understeer.roadwheel_steer_rad = ...
                deg2rad([2.1; 2.5; 3.1]);
            handling.understeer.local_gradient_deg_per_g = ...
                [NaN; 0.8; 1.0];
            handling.understeer.linear_fit_gradient_deg_per_g = 0.9;
        end
    end
end
