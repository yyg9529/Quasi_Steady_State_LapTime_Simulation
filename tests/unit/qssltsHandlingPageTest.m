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
                Tag="result-ymd-delta-mesh"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ymd-beta-mesh"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-understeer-line"));
            testCase.verifyEqual(app.ActivePage, "results");
        end

        function testYmdRendererUsesRcvdForceMomentCoordinates(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            handling = qssltsHandlingPageTest.makeHandlingResult();

            app.showHandlingResult(handling.ymd, handling.understeer);
            ymdAxes = findobj(app.UIFigure, Tag="result-ymd-axes");
            deltaMesh = findobj(app.UIFigure, ...
                Tag="result-ymd-delta-mesh");
            betaMesh = findobj(app.UIFigure, ...
                Tag="result-ymd-beta-mesh");

            testCase.verifyEmpty(findall(app.UIFigure, ...
                Tag="result-ymd-surface"));
            testCase.verifyEqual(deltaMesh.XData, ...
                handling.ymd.ay_g, AbsTol=1e-12);
            testCase.verifyEqual(deltaMesh.YData, ...
                handling.ymd.yaw_moment_coefficient, AbsTol=1e-12);
            testCase.verifyEqual(deltaMesh.ZData, ...
                zeros(size(handling.ymd.ay_g)), AbsTol=1e-12);
            testCase.verifyEqual(deltaMesh.CData, ...
                handling.ymd.ay_g, AbsTol=1e-12);
            testCase.verifyEqual(string(deltaMesh.FaceColor), "none");
            testCase.verifyEqual(string(deltaMesh.EdgeColor), "interp");
            testCase.verifyEqual(string(deltaMesh.MeshStyle), "column");
            testCase.verifyEqual(string(deltaMesh.LineStyle), "-");
            testCase.verifyEqual(deltaMesh.LineWidth, 2.4, AbsTol=1e-12);
            testCase.verifyEqual(string(betaMesh.FaceColor), "none");
            testCase.verifyEqual(string(betaMesh.EdgeColor), "interp");
            testCase.verifyEqual(string(betaMesh.MeshStyle), "row");
            testCase.verifyEqual(string(betaMesh.LineStyle), "--");
            testCase.verifyEqual(betaMesh.LineWidth, 2.2, AbsTol=1e-12);
            testCase.verifyEqual(ymdAxes.View, [0 90], AbsTol=1e-12);
            testCase.verifyNumElements(findall(app.UIFigure, ...
                Tag="result-ymd-delta-label"), ...
                size(handling.ymd.steer_rad, 2));
            testCase.verifyNumElements(findall(app.UIFigure, ...
                Tag="result-ymd-beta-label"), ...
                size(handling.ymd.beta_rad, 1));
            testCase.verifyNotEmpty(findall(app.UIFigure, ...
                Tag="result-ymd-zero-ay"));
            testCase.verifyNotEmpty(findall(app.UIFigure, ...
                Tag="result-ymd-zero-yaw-moment"));
        end

        function testHandlingRendererMasksInvalidOperatingPoints(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            handling = qssltsHandlingPageTest.makeHandlingResult();
            handling.ymd.within_tire_range(1, 1) = false;
            handling.understeer.within_tire_range(2) = false;

            app.showHandlingResult(handling.ymd, handling.understeer);
            deltaMesh = findobj(app.UIFigure, ...
                Tag="result-ymd-delta-mesh");
            betaMesh = findobj(app.UIFigure, ...
                Tag="result-ymd-beta-mesh");
            understeerLine = findobj(app.UIFigure, ...
                Tag="result-understeer-line");

            testCase.verifyTrue(isnan(deltaMesh.XData(1, 1)));
            testCase.verifyTrue(isnan(deltaMesh.YData(1, 1)));
            testCase.verifyTrue(isnan(betaMesh.XData(1, 1)));
            testCase.verifyTrue(isnan(betaMesh.YData(1, 1)));
            testCase.verifyEqual(understeerLine.XData(:), ...
                handling.understeer.ay_g([1 3]), AbsTol=1e-12);
        end


        function testHandlingRendererKeepsPlotsWhenGradientUnavailable(testCase)
            app = QssltsApp(false);
            testCase.addTeardown(@() delete(app));
            handling = qssltsHandlingPageTest.makeHandlingResult();
            handling.understeer.fit_available = false;
            handling.understeer.fit_point_count = 1;
            handling.understeer.valid_point_count = 3;
            handling.understeer.status = "partial";
            handling.understeer.status_message = ...
                "Understeer curve available; gradient unavailable.";
            handling.understeer.linear_fit_gradient_deg_per_g = NaN;

            app.showHandlingResult(handling.ymd, handling.understeer);
            understeerAxes = findobj(app.UIFigure, ...
                Tag="result-understeer-axes");

            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ymd-delta-mesh"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-ymd-beta-mesh"));
            testCase.verifyNotEmpty(findobj(app.UIFigure, ...
                Tag="result-understeer-line"));
            testCase.verifySubstring(string(understeerAxes.Title.String), ...
                "unavailable");
        end
    end

    methods (Static, Access = private)
        function handling = makeHandlingResult()
            handling.ymd.beta_rad = deg2rad([-2; 0; 2]);
            handling.ymd.steer_rad = deg2rad([-4, 0, 4]);
            handling.ymd.ay_mps2 = [-4 0 4; -3 0 3; -2 0 2];
            handling.ymd.ay_g = handling.ymd.ay_mps2 / 9.80665;
            handling.ymd.yaw_moment_cg_Nm = ...
                [-80 0 80; -60 0 60; -40 0 40];
            handling.ymd.yaw_moment_coefficient = ...
                handling.ymd.yaw_moment_cg_Nm / 1000;
            handling.ymd.converged = true(3, 3);
            handling.ymd.within_tire_range = true(3, 3);
            handling.understeer.ay_g = [0.1; 0.4; 0.8];
            handling.understeer.roadwheel_steer_rad = ...
                deg2rad([2.1; 2.5; 3.1]);
            handling.understeer.local_gradient_deg_per_g = ...
                [NaN; 0.8; 1.0];
            handling.understeer.linear_fit_gradient_deg_per_g = 0.9;
            handling.understeer.converged = true(3, 1);
            handling.understeer.within_tire_range = true(3, 1);
        end
    end
end
