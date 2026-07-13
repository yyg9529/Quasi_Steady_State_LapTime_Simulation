classdef tireAdapterContractTest < matlab.unittest.TestCase
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
        function testUnitirePlaceholderFailsWithoutEvaluator(testCase)
            tire = tire_load_sensitive_baseline();
            tire.slip_force = make_unitire_placeholder("simple");
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireAdapterUnavailable");
        end

        function testMagicFormulaPlaceholderFailsWithoutEvaluator(testCase)
            tire = tire_load_sensitive_baseline();
            tire.slip_force = make_magic_formula_placeholder();
            operatingPoint = nominalOperatingPoint("LEFT");
            operatingPoint.pressure_Pa = 82000;

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireAdapterUnavailable");
        end

        function testMagicPlaceholderDoesNotAssumeSourceCoordinates(testCase)
            slipForce = make_magic_formula_placeholder();

            testCase.verifyEqual( ...
                slipForce.metadata.source_coordinate_convention, ...
                "UNCONFIGURED");
        end

        function testSchemaMismatchIsRejected(testCase)
            tire = configuredUnitireTire();
            tire.slip_force.metadata.model_schema = "unitire_full_v1";
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireParameterSchema");
        end

        function testSimpleUnitireRejectsCamber(testCase)
            tire = configuredUnitireTire();
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0.01, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireCamberUnsupported");
        end

        function testOutOfDomainFailsByDefault(testCase)
            tire = configuredUnitireTire();
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.30, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireDomain");
        end

        function testExplicitExtrapolationIsReported(testCase)
            tire = configuredUnitireTire();
            tire.slip_force.metadata.extrapolation_policy = "allow";
            operatingPoint = nominalOperatingPoint("LEFT");

            force = tire_force_from_slip( ...
                0.30, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyTrue(force.is_extrapolated);
            testCase.verifyEqual(force.Fx_N, 300, AbsTol=1e-12);
        end

        function testMagicFormulaRequiresPressure(testCase)
            tire = configuredMagicFormulaTire();
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TirePressureRequired");
        end

        function testMixedMagicPressureCannotReusePreviousWheel(testCase)
            tire = configuredMagicFormulaTire();
            operatingPoint.wheel_vx_mps = [10; 10];
            operatingPoint.wheel_omega_radps = [44; 44];
            operatingPoint.wheel_side = ["LEFT"; "RIGHT"];
            operatingPoint.pressure_Pa = [82000; NaN];

            action = @() tire_force_from_slip( ...
                [0.02; 0.02], [0.03; 0.03], [660; 660], [0; 0], ...
                tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TirePressureRequired");
        end

        function testSingleSideModelRejectsOppositeSide(testCase)
            tire = configuredUnitireTire();
            tire.slip_force.metadata.native_tire_side = "LEFT";
            operatingPoint = nominalOperatingPoint("RIGHT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireSideUnsupported");
        end

        function testSaeForceConversionHelperUsesInternalAxes(testCase)
            nativeForce.Fx_N = 20;
            nativeForce.Fy_N = 60;

            force = convert_tire_force_to_internal( ...
                nativeForce, "SAE_J670_X_FORWARD_Y_RIGHT_Z_DOWN");

            testCase.verifyEqual(force.Fx_N, 20, AbsTol=1e-12);
            testCase.verifyEqual(force.Fy_N, -60, AbsTol=1e-12);
        end

        function testCanonicalEvaluatorReceivesCombinedSlip(testCase)
            tire = configuredUnitireTire();
            operatingPoint = nominalOperatingPoint("LEFT");

            force = tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyEqual(force.Fx_N, 20, AbsTol=1e-12);
            testCase.verifyEqual(force.Fy_N, -60, AbsTol=1e-12);
        end

        function testZeroLoadBypassesUnavailableAdapter(testCase)
            tire = tire_load_sensitive_baseline();
            tire.slip_force = make_unitire_placeholder("simple");

            force = tire_force_from_slip(0.2, -0.2, 0, 0, tire);

            testCase.verifyEqual(force.Fx_N, 0, AbsTol=0);
            testCase.verifyEqual(force.Fy_N, 0, AbsTol=0);
        end

        function testUnsupportedModelNeverFallsBack(testCase)
            tire = tire_load_sensitive_baseline();
            tire.slip_force.model_type = "unknown_advanced_model";

            action = @() tire_force_from_slip(0, 0, 660, 0, tire);

            testCase.verifyError(action, "QSSLTS:TireForceModel");
        end

        function testEmptyOperatingPointDoesNotChangeSimpleModel(testCase)
            tire = tire_load_sensitive_baseline();

            legacy = tire_force_from_slip(0.02, 0.03, 660, 0, tire);
            withEmptyInput = tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, struct());

            testCase.verifyEqual(withEmptyInput.Fx_N, legacy.Fx_N);
            testCase.verifyEqual(withEmptyInput.Fy_N, legacy.Fy_N);
        end

        function testMixedZeroLoadDoesNotReachEvaluator(testCase)
            tire = configuredUnitireTire();
            tire.slip_force.metadata.valid_domain = wideDomain();
            tire.slip_force.evaluator = @rejectZeroLoadEvaluator;
            operatingPoint.wheel_vx_mps = [10; 10];
            operatingPoint.wheel_omega_radps = [44; 44];
            operatingPoint.wheel_side = ["LEFT"; "RIGHT"];

            force = tire_force_from_slip( ...
                [0.02; 0.02], [0.03; 0.03], [660; 0], [0; 0], ...
                tire, operatingPoint);

            testCase.verifyEqual(force.Fx_N, [20; 0], AbsTol=1e-12);
            testCase.verifyEqual(force.Fy_N, [-60; 0], AbsTol=1e-12);
        end

        function testGgvProvenanceIgnoresTransientEvaluatorHandle(testCase)
            vehicle = vehicle_baseline();
            envelopeTire = tire_load_sensitive_baseline();
            powertrain = powertrain_baseline();
            brake = brake_baseline();
            options = default_qss_options();
            options.v_grid_mps = [5; 10];
            options.ay_grid_g = -2:0.5:2;
            ggv = generate_model_ggv(vehicle, envelopeTire, ...
                struct("enabled", false), powertrain, brake, options);
            transientTire = configuredUnitireTire();

            action = @() validate_7dof_comparison_ggv( ...
                ggv, vehicle, transientTire, powertrain, brake);

            testCase.verifyWarningFree(action);
            testCase.verifyFalse(isfield(ggv.provenance.tire, "slip_force"));
        end

        function testInvalidEvaluatorOutputIsRejected(testCase)
            tire = configuredUnitireTire();
            tire.slip_force.evaluator = @invalidOutputEvaluator;
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireAdapterOutput");
        end

        function testUnsupportedPhysicalOutputCannotBeSilentlyIgnored(testCase)
            tire = configuredUnitireTire();
            tire.slip_force.evaluator = @unsupportedOutputEvaluator;
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:TireAdapterOutput");
        end

        function testEvaluatorFailureNeverFallsBack(testCase)
            tire = configuredUnitireTire();
            tire.slip_force.evaluator = @failingEvaluator;
            operatingPoint = nominalOperatingPoint("LEFT");

            action = @() tire_force_from_slip( ...
                0.02, 0.03, 660, 0, tire, operatingPoint);

            testCase.verifyError(action, "QSSLTS:FixtureEvaluatorFailure");
        end
    end
end

function tire = configuredUnitireTire()
    tire = tire_load_sensitive_baseline();
    tire.slip_force = make_unitire_placeholder("simple");
    tire.slip_force.metadata.source = "unit-test fixture";
    tire.slip_force.metadata.evaluator_id = "unit_test_canonical_v1";
    tire.slip_force.metadata.native_tire_side = "SYMMETRIC";
    tire.slip_force.evaluator = @fakeCanonicalEvaluator;
end

function tire = configuredMagicFormulaTire()
    tire = tire_load_sensitive_baseline();
    tire.slip_force = make_magic_formula_placeholder();
    tire.slip_force.metadata.source = "unit-test fixture";
    tire.slip_force.metadata.evaluator_id = "unit_test_canonical_v1";
    tire.slip_force.metadata.native_tire_side = "SYMMETRIC";
    tire.slip_force.metadata.source_coordinate_convention = ...
        "QSSLTS_X_FORWARD_Y_LEFT_Z_UP";
    tire.slip_force.metadata.valid_domain = wideDomain();
    tire.slip_force.evaluator = @fakeCanonicalEvaluator;
end

function operatingPoint = nominalOperatingPoint(tireSide)
    operatingPoint.wheel_vx_mps = 40.193 / 3.6;
    operatingPoint.wheel_omega_radps = 48.8;
    operatingPoint.wheel_side = tireSide;
end

function domain = wideDomain()
    domain.slip_ratio = [-1, 1];
    domain.slip_angle_rad = [-1, 1];
    domain.Fz_N = [1, 2000];
    domain.camber_rad = [-0.5, 0.5];
    domain.wheel_vx_mps = [0, 100];
    domain.pressure_Pa = [50000, 150000];
end

function force = fakeCanonicalEvaluator(request, ~)
    force.Fx_N = 1000 * request.kappa;
    force.Fy_N = -2000 * request.alpha_rad;
end

function force = rejectZeroLoadEvaluator(request, parameters)
    assert(request.Fz_N > 0, "Zero load reached the external evaluator.");
    force = fakeCanonicalEvaluator(request, parameters);
end

function force = invalidOutputEvaluator(~, ~)
    force.Fx_N = NaN;
    force.Fy_N = 0;
end

function force = unsupportedOutputEvaluator(request, parameters)
    force = fakeCanonicalEvaluator(request, parameters);
    force.Mz_Nm = 1;
end

function force = failingEvaluator(~, ~)
    force = struct(); %#ok<NASGU>
    error("QSSLTS:FixtureEvaluatorFailure", ...
        "Synthetic evaluator failure for no-fallback testing.");
end
