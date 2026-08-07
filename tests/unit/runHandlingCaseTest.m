classdef runHandlingCaseTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        ProjectRoot
        RealTirPath
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.RealTirPath = fullfile(testCase.ProjectRoot, ...
                "data", "tire", "local", ...
                "Hoosier_16x75_10_R20.tir");
            folders = ["src", "data", "preprocessing"];
            for folder = folders
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                    fullfile(testCase.ProjectRoot, folder), ...
                    IncludingSubfolders=true));
            end
        end
    end

    methods (Test)
        function testMissingTirHasClearError(testCase)
            config = runHandlingCaseTest.makeConfig();
            handlingConfig = struct( ...
                "tir_file", string(tempname) + ".tir");

            action = @() run_handling_case(config, handlingConfig);

            testCase.verifyError(action, "QSSLTS:HandlingTirMissing");
        end

        function testRealTirProducesFiniteValidatedHandling(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            config = runHandlingCaseTest.makeConfig();
            handlingConfig = struct("tir_file", testCase.RealTirPath);

            handling = run_handling_case(config, handlingConfig);

            testCase.verifyTrue(handling.available);
            testCase.verifyTrue(all(isfinite( ...
                handling.ymd.yaw_moment_cg_Nm), "all"));
            testCase.verifyTrue(all(handling.ymd.converged, "all"));
            testCase.verifyTrue(all(handling.ymd.within_tire_range, "all"));
            testCase.verifyTrue(all(isfinite( ...
                handling.understeer.roadwheel_steer_rad)));
            testCase.verifyTrue(all(handling.understeer.converged));
            testCase.verifyTrue(all( ...
                handling.understeer.within_tire_range));
            testCase.verifyEqual(handling.provenance.tir_sha256, ...
                "6CE561640F15CE2DC5CCBA3CC2622978933E4CBE0C6FB1F15EC87520DC42F023");
            testCase.verifyEqual(handling.provenance.tire_evaluator, ...
                "MFeval");
            testCase.verifyEqual(handling.provenance.mfeval_version, ...
                "4.3.1");
        end
    end

    methods (Static, Access = private)
        function config = makeConfig()
            config.vehicle = vehicle_fs_2026();
            config.models.aero = struct("enabled", false);
        end
    end
end
