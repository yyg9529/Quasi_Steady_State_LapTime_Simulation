classdef suspensionLookupTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "preprocessing"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testReaderFiltersInvalidRowsAndConvertsAngles(testCase)
            filename = sampleFilename();

            susp = read_simscape_suspension_lookup(filename);

            testCase.verifyEqual(susp.jounce_mm, [-10; 0; 10], AbsTol=0);
            testCase.verifyEqual(susp.toe_rad(1), ...
                deg2rad(0.077361152787276), AbsTol=1e-14);
            testCase.verifyEqual(susp.camber_rad(3), ...
                deg2rad(-0.149075376263716), AbsTol=1e-14);
            testCase.verifyEqual(susp.motion_ratio_definition, ...
                "d(damper_length)/d(wheel_jounce)");
            testCase.verifyFalse(susp.wheel_rate_usage_confirmed);
        end

        function testInterpolationMatchesBreakpointAndMidpoint(testCase)
            susp = read_simscape_suspension_lookup(sampleFilename());

            atBreakpoint = interp_suspension_lookup(susp, 0);
            atMidpoint = interp_suspension_lookup(susp, 5);

            testCase.verifyEqual(atBreakpoint.motion_ratio, ...
                0.225039050526385, AbsTol=1e-14);
            testCase.verifyEqual(atBreakpoint.damper_stroke_mm, 0, AbsTol=0);
            testCase.verifyEqual(atMidpoint.camber_rad, ...
                0.5 * deg2rad(-0.149075376263716), AbsTol=1e-14);
            testCase.verifyFalse(atMidpoint.was_clamped);
        end

        function testOutOfRangePolicyIsExplicit(testCase)
            susp = read_simscape_suspension_lookup(sampleFilename());

            errorAction = @() interp_suspension_lookup(susp, 15);
            clamped = interp_suspension_lookup(susp, 15, OutOfRange="clamp");

            testCase.verifyError(errorAction, "QSSLTS:SuspensionRange");
            testCase.verifyEqual(clamped.jounce_mm, 10, AbsTol=0);
            testCase.verifyTrue(clamped.was_clamped);
        end

        function testConstantFallbackDoesNotInventMotionRatio(testCase)
            susp = make_constant_suspension(deg2rad(-1.5), deg2rad(0.1));

            state = interp_suspension_lookup(susp, 12);

            testCase.verifyEqual(state.camber_rad, deg2rad(-1.5), AbsTol=0);
            testCase.verifyEqual(state.toe_rad, deg2rad(0.1), AbsTol=0);
            testCase.verifyTrue(isnan(state.motion_ratio));
            testCase.verifyFalse(state.motion_ratio_available);
        end
    end
end

function filename = sampleFilename()
    projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
    filename = fullfile(projectRoot, "data", "suspension", ...
        "suspension_lookup_sample.csv");
end
