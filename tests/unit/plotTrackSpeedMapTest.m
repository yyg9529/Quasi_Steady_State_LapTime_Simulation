classdef plotTrackSpeedMapTest < matlab.unittest.TestCase
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
        function testClosedTrackReturnsFigureWithoutMutatingResult(testCase)
            result = plotTrackSpeedMapTest.makeResult();
            before = result;

            fig = plot_track_speed_map(result);

            axesHandle = findall(fig, Type="axes");
            speedLine = findobj(fig, Tag="track-speed-line");
            testCase.verifyTrue(isgraphics(fig, "figure"));
            testCase.verifyEqual(string(axesHandle.Title.String), ...
                "Track Speed Map");
            testCase.verifyEqual(speedLine.CData(:, 1), ...
                [result.v_mps; result.v_mps(1)], AbsTol=0);
            testCase.verifyTrue(isequaln(result, before));
        end

        function testMissingCoordinatesErrors(testCase)
            result = plotTrackSpeedMapTest.makeResult();
            result.track = rmfield(result.track, "x_m");

            action = @() plot_track_speed_map(result);

            testCase.verifyError(action, "QSSLTS:PlotTrackCoordinates");
        end

        function testNonfiniteCoordinatesError(testCase)
            result = plotTrackSpeedMapTest.makeResult();
            result.track.y_m(2) = NaN;

            action = @() plot_track_speed_map(result);

            testCase.verifyError(action, "QSSLTS:PlotTrackCoordinates");
        end

        function testNodeLengthMismatchErrors(testCase)
            result = plotTrackSpeedMapTest.makeResult();
            result.v_mps = result.v_mps(1:3);

            action = @() plot_track_speed_map(result);

            testCase.verifyError(action, "QSSLTS:PlotResultShape");
        end
    end

    methods (Static, Access=private)
        function result = makeResult()
            result.track.x_m = [0; 20; 20; 0];
            result.track.y_m = [0; 0; 10; 10];
            result.track.is_closed = true;
            result.v_mps = [10; 15; 20; 12];
        end
    end
end
