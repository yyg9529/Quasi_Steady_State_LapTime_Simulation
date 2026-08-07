classdef pac2002FullModelTest < matlab.unittest.TestCase
    %pac2002FullModelTest Contract tests for the full PAC2002 evaluator.

    properties (SetAccess = private)
        ProjectRoot
        RealTirPath
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.ProjectRoot = fileparts(fileparts( ...
                fileparts(mfilename("fullpath"))));
            testCase.RealTirPath = fullfile(testCase.ProjectRoot, ...
                "data", "tire", "local", ...
                "Hoosier_16x75_10_R20.tir");
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "preprocessing"), ...
                IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(testCase.ProjectRoot, "src"), ...
                IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testMissingFileHasClearError(testCase)
            missingPath = string(tempname) + ".tir";

            action = @() load_pac2002_tire(missingPath);

            testCase.verifyError(action, ...
                "load_pac2002_tire:FileNotFound");
        end

        function testRealTirPreservesHashSectionsAndSide(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));

            model = load_pac2002_tire(testCase.RealTirPath);

            testCase.verifySize(model, [1 1]);
            testCase.verifyEqual(model.source.sha256, ...
                "6CE561640F15CE2DC5CCBA3CC2622978933E4CBE0C6FB1F15EC87520DC42F023");
            testCase.verifyEqual(string(model.model_side), "LEFT");
            testCase.verifyNumElements(fieldnames(model.sections), 16);
            testCase.verifyTrue(isfield(model.sections, "MODEL"));
            testCase.verifyTrue(isfield(model.sections, ...
                "LATERAL_COEFFICIENTS"));
            testCase.verifyTrue(isfield(model.sections, ...
                "ALIGNING_COEFFICIENTS"));
        end

        function testZeroLoadBypassesMagicFormula(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            input = testCase.validScalarInput();
            input.Fz_N = 0;

            output = evaluate_pac2002_tire(model, input);

            testCase.verifyEqual(output.Fx_N, 0, AbsTol=1e-12);
            testCase.verifyEqual(output.Fy_N, 0, AbsTol=1e-12);
            testCase.verifyEqual(output.Mx_Nm, 0, AbsTol=1e-12);
            testCase.verifyEqual(output.My_Nm, 0, AbsTol=1e-12);
            testCase.verifyEqual(output.Mz_Nm, 0, AbsTol=1e-12);
            testCase.verifyEqual(output.effective_radius_m, 0.20066, ...
                AbsTol=1e-12);
            testCase.verifyFalse(output.within_range);
        end

        function testVectorInputReturnsFiniteMatchingShape(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            input = testCase.validVectorInput();

            output = evaluate_pac2002_tire(model, input);

            testCase.verifySize(output.Fx_N, size(input.Fz_N));
            testCase.verifySize(output.Fy_N, size(input.Fz_N));
            testCase.verifySize(output.Mx_Nm, size(input.Fz_N));
            testCase.verifySize(output.My_Nm, size(input.Fz_N));
            testCase.verifySize(output.Mz_Nm, size(input.Fz_N));
            testCase.verifySize(output.effective_radius_m, ...
                size(input.Fz_N));
            testCase.verifySize(output.within_range, size(input.Fz_N));
            testCase.verifyTrue(all(isfinite([output.Fx_N, output.Fy_N, ...
                output.Mx_Nm, output.My_Nm, output.Mz_Nm, ...
                output.effective_radius_m])));
            testCase.verifyTrue(all(output.within_range));
        end

        function testRightMountIsPhysicalMirrorOfLeftMount(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            leftInput = testCase.validScalarInput();
            leftInput.kappa = 0.08;
            leftInput.alpha_rad = 0.07;
            leftInput.gamma_rad = 0.025;
            rightInput = leftInput;
            rightInput.alpha_rad = -leftInput.alpha_rad;
            rightInput.gamma_rad = -leftInput.gamma_rad;
            rightInput.mount_side = "RIGHT";

            leftOutput = evaluate_pac2002_tire(model, leftInput);
            rightOutput = evaluate_pac2002_tire(model, rightInput);

            testCase.verifyEqual(rightOutput.Fx_N, leftOutput.Fx_N, ...
                AbsTol=1e-9);
            testCase.verifyEqual(rightOutput.Fy_N, -leftOutput.Fy_N, ...
                AbsTol=1e-9);
            testCase.verifyEqual(rightOutput.Mx_Nm, -leftOutput.Mx_Nm, ...
                AbsTol=1e-9);
            testCase.verifyEqual(rightOutput.My_Nm, leftOutput.My_Nm, ...
                AbsTol=1e-9);
            testCase.verifyEqual(rightOutput.Mz_Nm, -leftOutput.Mz_Nm, ...
                AbsTol=1e-9);
            testCase.verifyEqual(rightOutput.effective_radius_m, ...
                leftOutput.effective_radius_m, AbsTol=1e-12);
            testCase.verifyTrue(leftOutput.within_range);
            testCase.verifyTrue(rightOutput.within_range);
        end

        function testInputsOutsideTirRangesAreRejected(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            input = testCase.validVectorInput();
            input.Fz_N = [10000, 667, 667, 667];
            input.kappa = [0, 1.6, 0, 0];
            input.alpha_rad = [0, 0, 1.6, 0];
            input.gamma_rad = [0, 0, 0, 0.6];
            input.turn_slip_1pm = zeros(1, 4);
            input.Vx_mps = 10 * ones(1, 4);
            input.mount_side = ["LEFT", "LEFT", "RIGHT", "RIGHT"];

            output = evaluate_pac2002_tire(model, input);

            testCase.verifySize(output.within_range, [1 4]);
            testCase.verifyEqual(output.within_range, false(1, 4));
        end

        function testLoadedTirRunsDirectlyInFourWheelModel(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            vehicle = testCase.makeVehicle();
            state.speed_mps = 15;
            state.beta_rad = 0;
            state.yaw_rate_radps = 0;
            state.steer_rad = 0;

            point = evaluate_four_wheel_state( ...
                vehicle, model, struct("enabled", false), state);

            testCase.verifySize(point.wheel.Fz_N, [4 1]);
            testCase.verifyTrue(all(isfinite(point.wheel.Fy_tire_N)));
            testCase.verifyTrue(all(point.within_tire_range));
            testCase.verifyEqual(sum(point.wheel.Fz_N), ...
                vehicle.mass.total_kg * 9.80665, RelTol=1e-10);
        end

        function testVehicleOrientedPositiveSlipProducesPositiveFy(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            positiveInput = testCase.validScalarInput();
            positiveInput.alpha_rad = 1e-4;
            negativeInput = positiveInput;
            negativeInput.alpha_rad = -positiveInput.alpha_rad;

            positive = evaluate_pac2002_tire(model, positiveInput);
            negative = evaluate_pac2002_tire(model, negativeInput);

            testCase.verifyGreaterThan(positive.Fy_N - negative.Fy_N, 0);
        end

        function testMfevalExportCreatesHashBoundParameterFile(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            temporaryFolder = testCase.applyFixture( ...
                matlab.unittest.fixtures.TemporaryFolderFixture);
            copiedTir = fullfile(temporaryFolder.Folder, ...
                "Hoosier_16x75_10_R20.tir");
            copyfile(testCase.RealTirPath, copiedTir);

            manifest = export_mfeval_parameters(temporaryFolder.Folder);
            saved = load(manifest.parameter_file, "mfeval_export");

            testCase.verifyTrue(isfile(manifest.parameter_file));
            testCase.verifyEqual(manifest.mfeval_version, "4.3.1");
            testCase.verifyEqual(saved.mfeval_export.source.sha256, ...
                "6CE561640F15CE2DC5CCBA3CC2622978933E4CBE0C6FB1F15EC87520DC42F023");
            testCase.verifyEqual(saved.mfeval_export.parameters.FITTYP, 6);
            testCase.verifyEqual(saved.mfeval_export.parameters.FNOMIN, 667);
            testCase.verifyEqual(saved.mfeval_export.parameters.FZMAX, 9900);
        end

        function testLoadedTirUsesMfevalOutputs(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            input = testCase.validScalarInput();
            input.kappa = 0.08;
            input.alpha_rad = 0.07;
            input.gamma_rad = 0.025;
            warningState = warning("off", "Solver:CoeffChecks:Exa");
            testCase.addTeardown(@() warning(warningState));

            actual = model.evaluate(input);
            directInput = [input.Fz_N, input.kappa, ...
                -input.alpha_rad, input.gamma_rad, ...
                input.turn_slip_1pm, input.Vx_mps];
            expected = mfeval(model.mfeval.parameters, directInput, ...
                model.mfeval.use_mode);

            testCase.verifyEqual(model.evaluator_name, "MFeval");
            testCase.verifyEqual(model.mfeval.version, "4.3.1");
            testCase.verifyEqual(actual.Fx_N, expected(1), RelTol=1e-12);
            testCase.verifyEqual(actual.Fy_N, expected(2), RelTol=1e-12);
            testCase.verifyEqual(actual.Mx_Nm, expected(4), AbsTol=1e-12);
            testCase.verifyEqual(actual.My_Nm, expected(5), RelTol=1e-12);
            testCase.verifyEqual(actual.Mz_Nm, expected(6), RelTol=1e-12);
            testCase.verifyEqual(actual.effective_radius_m, expected(13), ...
                RelTol=1e-12);
        end

        function testMfevalModelRunsInBackgroundPool(testCase)
            testCase.assumeTrue(isfile(testCase.RealTirPath));
            model = load_pac2002_tire(testCase.RealTirPath);
            input = testCase.validScalarInput();
            input.alpha_rad = 0.03;

            future = parfeval(backgroundPool, model.evaluate, 1, input);
            testCase.addTeardown(@() cancel(future));
            output = fetchOutputs(future);

            testCase.verifyTrue(isfinite(output.Fy_N));
            testCase.verifyTrue(output.within_range);
        end
    end

    methods (Static, Access = private)
        function input = validScalarInput()
            input.Fz_N = 667;
            input.kappa = 0;
            input.alpha_rad = 0;
            input.gamma_rad = 0;
            input.turn_slip_1pm = 0;
            input.Vx_mps = 10;
            input.mount_side = "LEFT";
        end

        function input = validVectorInput()
            input.Fz_N = [300, 667, 1000];
            input.kappa = [-0.05, 0, 0.08];
            input.alpha_rad = [-0.06, 0.02, 0.08];
            input.gamma_rad = [-0.02, 0, 0.03];
            input.turn_slip_1pm = [0, 0.01, -0.01];
            input.Vx_mps = [8, 12, 16];
            input.mount_side = ["LEFT", "LEFT", "RIGHT"];
        end


        function vehicle = makeVehicle()
            vehicle.mass.total_kg = 300;
            vehicle.mass.front_static_frac = 0.5;
            vehicle.mass.cg_height_m = 0.25;
            vehicle.geometry.wheelbase_m = 1.56;
            vehicle.geometry.track_front_m = 1.20;
            vehicle.geometry.track_rear_m = 1.18;
            vehicle.inertia.Iz_kgm2 = 100;
            vehicle.load_transfer.front_lateral_distribution = 0.5;
        end
    end
end
