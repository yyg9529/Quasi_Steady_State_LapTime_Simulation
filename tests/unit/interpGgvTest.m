classdef interpGgvTest < matlab.unittest.TestCase
    properties
        Ggv
        Gravity
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            options = default_qss_options();
            options.v_grid_mps = (0:10:40).';
            options.ay_grid_g = -2:0.5:2;
            tire = tire_simple_baseline();
            testCase.Ggv = generate_model_ggv(vehicle_baseline(), tire, ...
                struct(), struct("enabled", false), brake_baseline(), options);
            testCase.Gravity = options.gravity_mps2;
        end
    end

    methods (Test)
        function testGridNodeReturnsExactCapability(testCase)
            cap = interp_ggv(testCase.Ggv, 20, 0);

            testCase.verifyTrue(cap.is_feasible);
            testCase.verifyEqual(cap.ax_max_mps2, ...
                1.5 * testCase.Gravity, AbsTol=1e-12);
            testCase.verifyEqual(cap.ax_min_mps2, ...
                testCase.Ggv.ax_min_g(3, 5) * testCase.Gravity, ...
                AbsTol=1e-12);
        end

        function testInteriorInterpolationIsFinite(testCase)
            cap = interp_ggv(testCase.Ggv, 15, 0.25);

            testCase.verifyTrue(cap.is_feasible);
            testCase.verifyTrue(isfinite(cap.ax_max_mps2));
            testCase.verifyLessThan(cap.ax_max_mps2, 1.5 * testCase.Gravity);
        end

        function testLateralDemandOutsideEnvelopeIsInfeasible(testCase)
            cap = interp_ggv(testCase.Ggv, 20, 1.6);

            testCase.verifyFalse(cap.is_feasible);
            testCase.verifyTrue(isnan(cap.ax_max_mps2));
            testCase.verifyEqual(cap.limiter, "lateral_infeasible");
        end

        function testExactLateralBoundaryHasNoTireLongitudinalForce(testCase)
            lateralLimit_g = interp1(testCase.Ggv.v_mps, ...
                testCase.Ggv.ay_limit_pos_g, 20, "linear");

            cap = interp_ggv(testCase.Ggv, 20, lateralLimit_g);

            testCase.verifyTrue(cap.is_feasible);
            testCase.verifyEqual(cap.ax_max_mps2, 0, AbsTol=1e-9);
            testCase.verifyEqual(cap.ax_min_mps2, 0, AbsTol=1e-9);
        end
    end
end
