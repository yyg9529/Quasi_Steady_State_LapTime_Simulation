function assessment = assess_analysis_result(result)
%ASSESS_ANALYSIS_RESULT Classify one analysis result for decision use.
%   Numerical validity and design feasibility are deliberately separate.

arguments
    result (1,1) struct
end

required = ["lap_time_s", "v_mps", "ax_mps2", "ay_mps2", ...
    "ggv_healthy", "constraint_audit", "valid"];
if ~all(isfield(result, cellstr(required))) ...
        || ~isfield(result.constraint_audit, "valid") ...
        || ~isscalar(result.lap_time_s) ...
        || ~isscalar(result.ggv_healthy) ...
        || ~isscalar(result.constraint_audit.valid) ...
        || ~isscalar(result.valid)
    error("QSSLTS:AnalysisResultContract", ...
        "Analysis result is missing canonical validity fields.");
end

propagationConverged = readPropagationConvergence(result);
finiteOutput = isfinite(result.lap_time_s) ...
    && all(isfinite(result.v_mps), "all") ...
    && all(isfinite(result.ax_mps2), "all") ...
    && all(isfinite(result.ay_mps2), "all");
ggvHealthy = logical(result.ggv_healthy);
constraintValid = logical(result.constraint_audit.valid);
canonicalValid = logical(result.valid);

reasonCodes = strings(0, 1);
if ~finiteOutput
    status = "nonfinite_output";
    reasonCodes(end + 1, 1) = "nonfinite_output";
elseif ~propagationConverged
    status = "solver_invalid";
    reasonCodes(end + 1, 1) = "propagation_not_converged";
elseif ~ggvHealthy
    status = "ggv_invalid";
    reasonCodes = nestedReasons(result, "ggv_health", ...
        "ggv_unhealthy");
elseif ~constraintValid
    status = "constraint_invalid";
    reasonCodes = nestedReasons(result, "constraint_audit", ...
        "constraint_audit_failed");
elseif ~canonicalValid
    status = "result_invalid";
    reasonCodes(end + 1, 1) = "canonical_result_invalid";
else
    status = "valid";
end

assessment.valid = finiteOutput && propagationConverged ...
    && ggvHealthy && constraintValid && canonicalValid;
assessment.status = status;
assessment.reason_codes = reasonCodes(:);
if isempty(reasonCodes)
    assessment.reject_reason = "";
else
    assessment.reject_reason = reasonCodes(1);
end
[assessment.design_feasibility_evaluated, ...
    assessment.design_feasible] = assessDesignFeasibility(result);
end

function converged = readPropagationConvergence(result)
if isfield(result, "propagation_converged") ...
        && isscalar(result.propagation_converged)
    converged = logical(result.propagation_converged);
elseif isfield(result, "solver") ...
        && isfield(result.solver, "propagation_converged") ...
        && isscalar(result.solver.propagation_converged)
    converged = logical(result.solver.propagation_converged);
elseif isfield(result, "solver") ...
        && isfield(result.solver, "converged") ...
        && isscalar(result.solver.converged)
    converged = logical(result.solver.converged);
else
    error("QSSLTS:AnalysisResultContract", ...
        "Analysis result is missing propagation convergence status.");
end
end

function reasons = nestedReasons(result, fieldName, fallback)
reasons = strings(0, 1);
value = result.(fieldName);
if isfield(value, "reason_codes")
    reasons = string(value.reason_codes(:));
    reasons = reasons(strlength(reasons) > 0);
end
if isempty(reasons)
    reasons = string(fallback);
end
end

function [evaluated, feasible] = assessDesignFeasibility(result)
evaluated = false;
feasible = NaN;
if isfield(result, "energy") ...
        && isfield(result.energy, "can_finish_endurance_estimated") ...
        && isscalar(result.energy.can_finish_endurance_estimated)
    evaluated = true;
    feasible = logical(result.energy.can_finish_endurance_estimated);
end
end
