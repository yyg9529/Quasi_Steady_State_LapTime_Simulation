classdef assessAnalysisResultTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "src"), IncludingSubfolders=true));
        end
    end

    methods (Test)
        function testCanonicalValidResultPasses(testCase)
            result = assessAnalysisResultTest.makeValidResult();

            assessment = assess_analysis_result(result);

            testCase.verifyTrue(assessment.valid);
            testCase.verifyEqual(assessment.status, "valid");
            testCase.verifyEmpty(assessment.reason_codes);
            testCase.verifyFalse( ...
                assessment.design_feasibility_evaluated);
            testCase.verifyTrue(isnan(assessment.design_feasible));
        end

        function testNonfiniteOutputHasStableStatus(testCase)
            result = assessAnalysisResultTest.makeValidResult();
            result.lap_time_s = NaN;

            assessment = assess_analysis_result(result);

            testCase.verifyFalse(assessment.valid);
            testCase.verifyEqual(assessment.status, "nonfinite_output");
            testCase.verifyEqual(assessment.reject_reason, ...
                "nonfinite_output");
        end

        function testPropagationFailureIsSeparated(testCase)
            result = assessAnalysisResultTest.makeValidResult();
            result.propagation_converged = false;
            result.solver.propagation_converged = false;
            result.solver.converged = false;
            result.valid = false;

            assessment = assess_analysis_result(result);

            testCase.verifyFalse(assessment.valid);
            testCase.verifyEqual(assessment.status, "solver_invalid");
            testCase.verifyEqual(assessment.reject_reason, ...
                "propagation_not_converged");
        end

        function testGgvFailureIsSeparated(testCase)
            result = assessAnalysisResultTest.makeValidResult();
            result.ggv_healthy = false;
            result.ggv_health.reason_codes = ...
                "ggv_fixed_point_not_converged";
            result.valid = false;

            assessment = assess_analysis_result(result);

            testCase.verifyFalse(assessment.valid);
            testCase.verifyEqual(assessment.status, "ggv_invalid");
            testCase.verifyEqual(assessment.reject_reason, ...
                "ggv_fixed_point_not_converged");
        end

        function testConstraintFailureIsSeparated(testCase)
            result = assessAnalysisResultTest.makeValidResult();
            result.constraint_audit.valid = false;
            result.constraint_audit.reason_codes = ...
                "combined_slip_violation";
            result.valid = false;

            assessment = assess_analysis_result(result);

            testCase.verifyFalse(assessment.valid);
            testCase.verifyEqual(assessment.status, "constraint_invalid");
            testCase.verifyEqual(assessment.reject_reason, ...
                "combined_slip_violation");
        end

        function testEnergyShortfallDoesNotInvalidateNumerics(testCase)
            result = assessAnalysisResultTest.makeValidResult();
            result.energy.can_finish_endurance_estimated = false;

            assessment = assess_analysis_result(result);

            testCase.verifyTrue(assessment.valid);
            testCase.verifyTrue(assessment.design_feasibility_evaluated);
            testCase.verifyFalse(assessment.design_feasible);
        end

        function testMissingCanonicalFieldIsContractError(testCase)
            result = rmfield( ...
                assessAnalysisResultTest.makeValidResult(), "ggv_healthy");

            action = @() assess_analysis_result(result);

            testCase.verifyError(action, ...
                "QSSLTS:AnalysisResultContract");
        end
    end

    methods (Static, Access=private)
        function result = makeValidResult()
            result.lap_time_s = 10;
            result.v_mps = [10; 10];
            result.ax_mps2 = [0; 0];
            result.ay_mps2 = [0; 0];
            result.propagation_converged = true;
            result.solver.converged = true;
            result.solver.propagation_converged = true;
            result.ggv_healthy = true;
            result.ggv_health.reason_codes = strings(0, 1);
            result.constraint_audit.valid = true;
            result.constraint_audit.reason_codes = strings(0, 1);
            result.valid = true;
            result.status = "valid";
            result.reject_reason = "";
        end
    end
end
