classdef vehicleFs2026Test < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function test_vehicle_fs_2026_wheelbase(testCase)
            vehicle = vehicle_fs_2026();

            testCase.verifyEqual(vehicle.geometry.wheelbase_m, 1.560, ...
                AbsTol=1e-12);
            testCase.verifyEqual(vehicle.provenance.season, 2026);
            testCase.verifyEqual(vehicle.provenance.wheelbase_source, ...
                "User-confirmed 1560 mm");
            testCase.verifyEqual(vehicle.provenance.inherited_parameters.status, ...
                "provisional");
            testCase.verifyEqual(vehicle.provenance.inherited_parameters.source, ...
                "baseline-derived");
            testCase.verifyEqual( ...
                vehicle.provenance.inherited_parameters.measurement_status, ...
                "not yet measured");
        end

        function test_vehicle_fs_2026_positive_geometry(testCase)
            vehicle = vehicle_fs_2026();
            geometryValues = cell2mat(struct2cell(vehicle.geometry));

            testCase.verifyGreaterThan(geometryValues, 0);
        end

        function test_vehicle_fs_2026_load_transfer_uses_1p560m(testCase)
            vehicle = vehicle_fs_2026();
            state.ax_mps2 = 3.0;
            state.ay_mps2 = 0;
            expected_N = vehicle.mass.total_kg * state.ax_mps2 ...
                * vehicle.mass.cg_height_m / 1.560;

            transfer = calc_load_transfer(state, vehicle);

            testCase.verifyEqual(transfer.longitudinal_N, expected_N, ...
                AbsTol=1e-12);
        end
    end
end
