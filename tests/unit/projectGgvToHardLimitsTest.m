classdef projectGgvToHardLimitsTest < matlab.unittest.TestCase
    properties (SetAccess = private)
        HardReference
    end

    methods (TestClassSetup)
        function buildFixture(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "data"), IncludingSubfolders=true));
            options = default_qss_options();
            options.v_grid_mps = [0; 10; 20];
            options.ay_grid_g = -2:0.25:2;
            testCase.HardReference = generate_model_ggv( ...
                vehicle_baseline(), tire_simple_baseline(), ...
                struct("enabled", false), struct("enabled", false), ...
                brake_baseline(), options);
        end
    end

    methods (Test)
        function testIdentityProjectionPreservesCapability(testCase)
            [projected, report] = project_ggv_to_hard_limits( ...
                testCase.HardReference, testCase.HardReference);

            testCase.verifyEqual(projected.ax_max_g, ...
                testCase.HardReference.ax_max_g, AbsTol=0);
            testCase.verifyEqual(projected.ax_min_g, ...
                testCase.HardReference.ax_min_g, AbsTol=0);
            testCase.verifyEqual(projected.feasible, ...
                testCase.HardReference.feasible);
            testCase.verifyEqual(report.status, "identity");
            testCase.verifyFalse(report.requires_component_recalibration);
            testCase.verifyEmpty(report.reason_codes);
        end

        function testConservativeCandidateIsNotExpanded(testCase)
            candidate = testCase.HardReference;
            candidate.ax_max_g = candidate.ax_max_g ...
                - 0.05 * abs(candidate.ax_max_g);
            candidate.ax_min_g = candidate.ax_min_g ...
                + 0.05 * abs(candidate.ax_min_g);
            candidate.source = "conservative_fixture";

            [projected, report] = project_ggv_to_hard_limits( ...
                candidate, testCase.HardReference);

            testCase.verifyEqual(projected.ax_max_g, ...
                candidate.ax_max_g, AbsTol=0);
            testCase.verifyEqual(projected.ax_min_g, ...
                candidate.ax_min_g, AbsTol=0);
            testCase.verifyEqual(report.status, "conservative");
            testCase.verifyFalse(report.requires_component_recalibration);
        end

        function testPositiveResidualIsClippedAndTraceable(testCase)
            candidate = testCase.HardReference;
            candidate.ax_max_g = candidate.ax_max_g + 0.20;
            candidate.ax_min_g = candidate.ax_min_g - 0.30;
            candidate.ay_limit_pos_g = candidate.ay_limit_pos_g + 0.40;
            candidate.ay_limit_neg_g = candidate.ay_limit_neg_g - 0.40;
            candidate.source = "positive_residual_fixture";

            [projected, report] = project_ggv_to_hard_limits( ...
                candidate, testCase.HardReference);

            testCase.verifyLessThanOrEqual(projected.ax_max_g( ...
                projected.feasible), testCase.HardReference.ax_max_g( ...
                projected.feasible));
            testCase.verifyGreaterThanOrEqual(projected.ax_min_g( ...
                projected.feasible), testCase.HardReference.ax_min_g( ...
                projected.feasible));
            testCase.verifyLessThanOrEqual(projected.ay_limit_pos_g, ...
                testCase.HardReference.ay_limit_pos_g);
            testCase.verifyGreaterThanOrEqual(projected.ay_limit_neg_g, ...
                testCase.HardReference.ay_limit_neg_g);
            testCase.verifyEqual(report.status, "clipped");
            testCase.verifyGreaterThan(report.accel_clip_count, 0);
            testCase.verifyGreaterThan(report.brake_clip_count, 0);
            testCase.verifyGreaterThan(report.lateral_clip_count, 0);
            testCase.verifyTrue(report.requires_component_recalibration);
            testCase.verifyTrue(ismember( ...
                "component_recalibration_required", report.reason_codes));
        end

        function testCalibrationScalePathProjectsToInputHardMap(testCase)
            speeds = testCase.HardReference.v_mps;
            scaleTable = table(speeds, 1.2 * ones(3, 1), ...
                1.2 * ones(3, 1), 1.2 * ones(3, 1), ...
                VariableNames=["v_mps", "scale_lat", ...
                "scale_acc", "scale_brake"]);

            [projected, report] = apply_ggv_calibration_scales( ...
                testCase.HardReference, scaleTable);

            testCase.verifyLessThanOrEqual(projected.ax_max_g( ...
                projected.feasible), testCase.HardReference.ax_max_g( ...
                projected.feasible) + 1e-12);
            testCase.verifyGreaterThanOrEqual(projected.ax_min_g( ...
                projected.feasible), testCase.HardReference.ax_min_g( ...
                projected.feasible) - 1e-12);
            testCase.verifyEqual(report.status, "clipped");
            testCase.verifyEqual(projected.hard_limit_projection_report, ...
                report);
        end

        function testLateralBoundaryClipsAreTraceable(testCase)
            candidate = testCase.HardReference;
            candidate.ax_max_lateral_boundary_g = ...
                candidate.ax_max_lateral_boundary_g + 0.10;
            candidate.ax_min_lateral_boundary_g = ...
                candidate.ax_min_lateral_boundary_g - 0.20;

            [projected, report] = project_ggv_to_hard_limits( ...
                candidate, testCase.HardReference);

            testCase.verifyEqual(projected.ax_max_lateral_boundary_g, ...
                testCase.HardReference.ax_max_lateral_boundary_g, ...
                AbsTol=0);
            testCase.verifyEqual(projected.ax_min_lateral_boundary_g, ...
                testCase.HardReference.ax_min_lateral_boundary_g, ...
                AbsTol=0);
            testCase.verifyEqual(report.status, "clipped");
            testCase.verifyEqual(report.accel_boundary_clip_count, ...
                numel(testCase.HardReference.v_mps));
            testCase.verifyEqual(report.brake_boundary_clip_count, ...
                numel(testCase.HardReference.v_mps));
            testCase.verifyTrue(all(report.accel_boundary_clip_mask));
            testCase.verifyTrue(all(report.brake_boundary_clip_mask));
            testCase.verifyTrue(report.requires_component_recalibration);
            testCase.verifyTrue(ismember( ...
                "ggv_accel_boundary_hard_limit_clipped", ...
                report.reason_codes));
            testCase.verifyTrue(ismember( ...
                "ggv_brake_boundary_hard_limit_clipped", ...
                report.reason_codes));
        end

        function testFeasibleDomainExpansionClipIsTraceable(testCase)
            candidate = testCase.HardReference;
            expandedIndex = find(~testCase.HardReference.feasible, 1);
            candidate.feasible(expandedIndex) = true;

            [projected, report] = project_ggv_to_hard_limits( ...
                candidate, testCase.HardReference);

            testCase.verifyFalse(projected.feasible(expandedIndex));
            testCase.verifyTrue( ...
                report.feasible_domain_clip_mask(expandedIndex));
            testCase.verifyEqual(report.feasible_domain_clip_count, 1);
            testCase.verifyEqual(report.status, "clipped");
            testCase.verifyTrue(report.requires_component_recalibration);
            testCase.verifyTrue(ismember( ...
                "ggv_feasible_domain_hard_limit_clipped", ...
                report.reason_codes));
        end

        function testGridMismatchIsRejected(testCase)
            candidate = testCase.HardReference;
            candidate.v_mps(2) = candidate.v_mps(2) + 0.1;

            action = @() project_ggv_to_hard_limits( ...
                candidate, testCase.HardReference);

            testCase.verifyError(action, "QSSLTS:GGVProjectionGrid");
        end
    end
end
