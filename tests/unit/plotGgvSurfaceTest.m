classdef plotGgvSurfaceTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (TestMethodSetup)
        function hideFigures(testCase)
            original = get(groot, "DefaultFigureVisible");
            set(groot, "DefaultFigureVisible", "off");
            testCase.addTeardown(@() set(groot, ...
                "DefaultFigureVisible", original));
            testCase.addTeardown(@() close("all", "force"));
        end
    end

    methods (Test)
        function testNonSquareGgvAndActualPointsDoNotMutateResult(testCase)
            result = plotGgvSurfaceTest.makeResult();
            before = result;

            fig = plot_ggv_surface(result, [10 15 25]);

            axesHandles = findall(fig, Type="axes");
            titleText = string(arrayfun(@(ax) ax.Title.String, ...
                axesHandles, UniformOutput=false));
            envelopeSurface = findobj(fig, Tag="ggv-envelope-surface");
            actualPoints3d = findobj(fig, Tag="actual-ggv-points-3d");
            actualPoints = findobj(fig, Tag="actual-ggv-points");
            interpolatedSlice = findobj(fig, ...
                DisplayName="Ax max @ 15.0 m/s");
            testCase.verifyTrue(isgraphics(fig, "figure"));
            testCase.verifyNumElements(axesHandles, 2);
            testCase.verifyEqual(sort(titleText), ...
                sort(["GGV Capability Surface"; "GGV Speed Slices"]));
            testCase.verifyEqual(envelopeSurface.ZData(:, 1), ...
                result.ggv_used.v_mps, AbsTol=1e-12);
            testCase.verifyEqual(envelopeSurface.XData(:, 1), ...
                envelopeSurface.XData(:, end), AbsTol=1e-12);
            testCase.verifyEqual(envelopeSurface.YData(:, 1), ...
                envelopeSurface.YData(:, end), AbsTol=1e-12);
            testCase.verifyEqual(actualPoints3d.XData(:), ...
                result.ax_mps2(:) / result.ggv_used.gravity_mps2, ...
                AbsTol=1e-12);
            testCase.verifyEqual(actualPoints3d.YData(:), ...
                result.ay_mps2(:) / result.ggv_used.gravity_mps2, ...
                AbsTol=1e-12);
            testCase.verifyEqual(actualPoints3d.ZData(:), ...
                result.v_mps(:), AbsTol=1e-12);
            testCase.verifyNumElements(actualPoints.XData, ...
                numel(result.v_mps));
            testCase.verifyEqual(interpolatedSlice.YData, ...
                0.5 * (result.ggv_used.ax_max_g(1, :) ...
                + result.ggv_used.ax_max_g(2, :)), AbsTol=1e-12);
            testCase.verifyTrue(isequaln(result, before));
        end

        function testDefaultSpeedSlicesReturnsFigure(testCase)
            result = plotGgvSurfaceTest.makeResult();

            fig = plot_ggv_surface(result);

            testCase.verifyTrue(isgraphics(fig, "figure"));
        end

        function testNonfiniteSpeedSliceErrors(testCase)
            result = plotGgvSurfaceTest.makeResult();

            action = @() plot_ggv_surface(result, [10 NaN]);

            testCase.verifyError(action, "QSSLTS:PlotGGVSpeedSlices");
        end

        function testOutOfRangeSpeedSliceErrors(testCase)
            result = plotGgvSurfaceTest.makeResult();

            action = @() plot_ggv_surface(result, [10 40]);

            testCase.verifyError(action, "QSSLTS:PlotGGVSpeedSlices");
        end

        function testGgvShapeMismatchErrors(testCase)
            result = plotGgvSurfaceTest.makeResult();
            result.ggv_used.ax_min_g = zeros(5, 3);

            action = @() plot_ggv_surface(result);

            testCase.verifyError(action, "QSSLTS:PlotGGVShape");
        end

        function testInternalFeasibilityHoleIsRejected(testCase)
            result = plotGgvSurfaceTest.makeResult();
            result.ggv_used.feasible(1, 3) = false;

            action = @() plot_ggv_surface(result);

            testCase.verifyError(action, "QSSLTS:GGVBoundaryCoverage");
        end

        function testExactGridEndpointsPreserveLateralAsymmetry(testCase)
            ggv.v_mps = [10; 20];
            ggv.ay_g = [-1 0 1];
            ggv.ax_max_g = [0.1 0.8 0.3; 0.2 0.7 0.4];
            ggv.ax_min_g = [-0.4 -1.0 -0.6; -0.3 -0.9 -0.5];
            ggv.feasible = true(2, 3);
            ggv.ay_limit_neg_g = [-1; -1];
            ggv.ay_limit_pos_g = [1; 1];
            ggv.ax_max_lateral_boundary_g = zeros(2, 1);
            ggv.ax_min_lateral_boundary_g = zeros(2, 1);

            [axBoundary_g, ayBoundary_g] = ...
                prepare_ggv_boundary_surface(ggv);
            [~, positiveIndex] = max(ayBoundary_g(1, :));

            testCase.verifyEqual(axBoundary_g(1, 1), 0.1, ...
                AbsTol=1e-12);
            testCase.verifyEqual(axBoundary_g(1, positiveIndex), 0.3, ...
                AbsTol=2e-2);
        end
    end

    methods (Static, Access=private)
        function result = makeResult()
            ggv.v_mps = [10; 20; 30];
            ggv.ay_g = [-1, -0.5, 0, 0.5, 1];
            ggv.ax_max_g = [0.2 0.5 0.7 0.5 0.2; ...
                0.1 0.4 0.6 0.4 0.1; 0 0.3 0.5 0.3 0];
            ggv.ax_min_g = -[0.3 0.7 1.0 0.7 0.3; ...
                0.3 0.6 0.9 0.6 0.3; 0.2 0.5 0.8 0.5 0.2];
            ggv.feasible = true(3, 5);
            ggv.feasible(3, [1 5]) = false;
            ggv.gravity_mps2 = 9.81;
            ggv.ay_limit_neg_g = [-1; -1; -0.8];
            ggv.ay_limit_pos_g = [1; 1; 0.8];
            ggv.ax_max_lateral_boundary_g = [0.2; 0.1; 0];
            ggv.ax_min_lateral_boundary_g = [-0.3; -0.3; -0.2];
            result.ggv_used = ggv;
            result.v_mps = [10; 15; 22; 30];
            result.ax_mps2 = [1; 2; -4; 0.5];
            result.ay_mps2 = [0; 3; -5; 2];
        end
    end
end
