classdef qssScopeTest < matlab.unittest.TestCase
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
        function test_no_dof7_production_dependency(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            forbiddenFiles = [
                fullfile(projectRoot, "src", "tire", ...
                    "tire_force_from_slip.m")
                fullfile(projectRoot, "src", "tire", ...
                    "tire_slip_force_simple.m")
                fullfile(projectRoot, "src", "tire", ...
                    "tire_force_external_adapter.m")
                fullfile(projectRoot, "src", "tire", ...
                    "convert_tire_force_to_internal.m")
                fullfile(projectRoot, "src", "tire", ...
                    "make_unitire_placeholder.m")
                fullfile(projectRoot, "src", "tire", ...
                    "make_magic_formula_placeholder.m")
            ];
            simpleTire = tire_simple_baseline();
            loadSensitiveTire = tire_load_sensitive_baseline();

            testCase.verifyEmpty(dir(fullfile( ...
                projectRoot, "src", "dof7", "**", "*.m")));
            testCase.verifyFalse(any(isfile(forbiddenFiles)));
            testCase.verifyFalse(isfield(simpleTire, "wheel_inertia_kgm2"));
            testCase.verifyFalse(isfield(simpleTire, "slip_force"));
            testCase.verifyFalse(isfield( ...
                loadSensitiveTire, "wheel_inertia_kgm2"));
            testCase.verifyFalse(isfield(loadSensitiveTire, "slip_force"));
        end

        function test_tire_envelope_provenance_has_no_slip_force_scope(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            sourcePath = fullfile(projectRoot, "src", "tire", ...
                "tire_envelope_provenance.m");

            sourceText = fileread(sourcePath);

            testCase.verifyFalse(contains(sourceText, "slip_force"));
        end

        function test_lap_and_ggv_remain_independent_of_handling_model(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            lapSource = fileread(fullfile(projectRoot, "src", ...
                "core", "run_qss_lap.m"));
            ggvSource = fileread(fullfile(projectRoot, "src", ...
                "ggv", "generate_model_ggv.m"));
            combinedSource = string(lapSource) + newline + string(ggvSource);
            handlingFunctions = ["evaluate_pac2002_tire", ...
                "evaluate_four_wheel_state", "generate_ymd", ...
                "solve_steady_state_cornering", ...
                "calc_understeer_gradient"];

            testCase.verifyFalse(any(contains( ...
                combinedSource, handlingFunctions)));
        end
    end
end
