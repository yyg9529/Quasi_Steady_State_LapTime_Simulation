classdef analysisUtilitiesTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testNestedFieldRoundTrip(testCase)
            input.vehicle.mass.total_kg = 300;

            output = set_nested_field( ...
                input, "vehicle.mass.total_kg", 320);
            value = get_nested_field(output, "vehicle.mass.total_kg");

            testCase.verifyEqual(value, 320, AbsTol=0);
            testCase.verifyEqual(input.vehicle.mass.total_kg, 300, AbsTol=0);
        end

        function testUnknownFieldErrors(testCase)
            input.vehicle.mass.total_kg = 300;

            action = @() set_nested_field(input, "vehicle.mass.unknown", 1);

            testCase.verifyError(action, "QSSLTS:FieldPath");
        end
    end
end
