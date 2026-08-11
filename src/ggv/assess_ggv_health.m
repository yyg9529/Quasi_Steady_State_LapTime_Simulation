function health = assess_ggv_health(ggv, options)
%ASSESS_GGV_HEALTH Assess whether a GGV is safe for decision use.
%   Model-only diagnostics that are absent from an external/empirical GGV
%   are reported as not_applicable rather than silently treated as passes.

arguments
    ggv (1,1) struct
    options (1,1) struct = struct()
end

options = default_qss_options(options);
health.source = sourceLabel(ggv);
health.structure = diagnostic("pass");
health.finite_data = diagnostic("pass");
health.feasible_domain = diagnostic("not_applicable");
health.gravity_consistency = diagnostic("not_applicable");
health.fixed_point = diagnostic("not_applicable");
health.residual = diagnostic("not_applicable");
health.lateral_grid = diagnostic("not_applicable");
health.wheel_lift = diagnostic("not_applicable");
reasonCodes = strings(0, 1);

required = ["v_mps", "ay_g", "ax_max_g", "ax_min_g", "feasible", ...
    "ay_limit_pos_g", "ay_limit_neg_g", "accel_limiter", ...
    "brake_limiter", "gravity_mps2"];
missing = required(~isfield(ggv, cellstr(required)));
if ~isempty(missing)
    health.structure = diagnostic("fail", numel(missing));
    health.structure.missing_fields = missing(:);
    reasonCodes(end + 1, 1) = "ggv_structure_invalid";
    health = finalize(health, reasonCodes);
    return
end

[structureValid, expectedSize] = baseStructureValid(ggv);
if ~structureValid
    health.structure = diagnostic("fail", 1);
    reasonCodes(end + 1, 1) = "ggv_structure_invalid";
    health = finalize(health, reasonCodes);
    return
end

feasible = logical(ggv.feasible);
feasibleCount = nnz(feasible);
health.feasible_domain = passFailDiagnostic(double(feasibleCount == 0));
health.feasible_domain.feasible_count = feasibleCount;
if feasibleCount == 0
    health.structure = addDiagnosticFailure(health.structure, 1);
    reasonCodes(end + 1, 1) = "ggv_no_feasible_domain";
end

finiteBase = all(isfinite(ggv.v_mps), "all") ...
    && all(isfinite(ggv.ay_g), "all") ...
    && all(isfinite(ggv.ay_limit_pos_g), "all") ...
    && all(isfinite(ggv.ay_limit_neg_g), "all") ...
    && isfinite(ggv.gravity_mps2) && ggv.gravity_mps2 > 0 ...
    && all(isfinite(ggv.ax_max_g(feasible))) ...
    && all(isfinite(ggv.ax_min_g(feasible)));
if ~finiteBase
    health.finite_data = diagnostic("fail", 1);
    reasonCodes(end + 1, 1) = "ggv_nonfinite_data";
end


gravityTolerance_mps2 = 1e-12;
if isfinite(options.gravity_mps2)
    gravityTolerance_mps2 = gravityTolerance_mps2 ...
        * max(1, abs(options.gravity_mps2));
end
gravityMismatch = ~isfinite(options.gravity_mps2) ...
    || options.gravity_mps2 <= 0 ...
    || ~isfinite(ggv.gravity_mps2) || ggv.gravity_mps2 <= 0 ...
    || abs(ggv.gravity_mps2 - options.gravity_mps2) ...
        > gravityTolerance_mps2;
health.gravity_consistency = ...
    passFailDiagnostic(double(gravityMismatch));
health.gravity_consistency.actual_mps2 = ggv.gravity_mps2;
health.gravity_consistency.expected_mps2 = options.gravity_mps2;
health.gravity_consistency.tolerance_mps2 = gravityTolerance_mps2;
if gravityMismatch
    health.structure = addDiagnosticFailure(health.structure, 1);
    reasonCodes(end + 1, 1) = "ggv_gravity_mismatch";
end

fixedPointFields = ["solve_converged_accel", ...
    "solve_converged_brake"];
residualFields = ["solve_residual_accel_mps2", ...
    "solve_residual_brake_mps2"];
modelDiagnosticFields = [fixedPointFields, residualFields];
diagnosticPresence = isfield(ggv, cellstr(modelDiagnosticFields));
if all(diagnosticPresence)
    diagnosticsHaveShape = all(arrayfun( ...
        @(name) isequal(size(ggv.(name)), expectedSize), ...
        modelDiagnosticFields));
    if diagnosticsHaveShape
        [fixedPointFinite, nonfiniteFixedPointCount] = ...
            finiteFixedPointDiagnostics(ggv, fixedPointFields);
        if fixedPointFinite
            health = assessFixedPoints(health, ggv, feasible);
            if health.fixed_point.status == "fail"
                reasonCodes(end + 1, 1) = ...
                    "ggv_fixed_point_not_converged";
            end
        else
            health.structure = addDiagnosticFailure( ...
                health.structure, nonfiniteFixedPointCount);
            health.fixed_point = diagnostic( ...
                "fail", nonfiniteFixedPointCount);
            reasonCodes(end + 1, 1) = ...
                "ggv_fixed_point_diagnostic_nonfinite";
        end
        health = assessResiduals(health, ggv, feasible, options);
        if health.residual.status == "fail"
            reasonCodes(end + 1, 1) = "ggv_residual_exceeded";
        end
    else
        health.structure = diagnostic("fail", 1);
        health.fixed_point = diagnostic("fail", 1);
        health.residual = diagnostic("fail", 1);
        reasonCodes(end + 1, 1) = ...
            "ggv_diagnostic_structure_invalid";
    end
elseif any(diagnosticPresence)
    health.structure = diagnostic("fail", 1);
    health.fixed_point = diagnostic("fail", 1);
    health.residual = diagnostic("fail", 1);
    reasonCodes(end + 1, 1) = "ggv_diagnostic_structure_invalid";
end

if isfield(ggv, "lateral_limit_truncated")
    if isequal(size(ggv.lateral_limit_truncated), ...
            [expectedSize(1), 2])
        truncatedCount = nnz(logical(ggv.lateral_limit_truncated));
        health.lateral_grid = passFailDiagnostic(truncatedCount);
        if truncatedCount > 0
            reasonCodes(end + 1, 1) = "ggv_lateral_grid_truncated";
        end
    else
        health.structure = diagnostic("fail", 1);
        health.lateral_grid = diagnostic("fail", 1);
        reasonCodes(end + 1, 1) = ...
            "ggv_diagnostic_structure_invalid";
    end
end

if isfield(ggv, "wheel_lift")
    if isequal(size(ggv.wheel_lift), expectedSize)
        liftCount = nnz(logical(ggv.wheel_lift) & feasible);
        health.wheel_lift = passFailDiagnostic(liftCount);
        if liftCount > 0
            reasonCodes(end + 1, 1) = "ggv_wheel_lift";
        end
    else
        health.structure = diagnostic("fail", 1);
        health.wheel_lift = diagnostic("fail", 1);
        reasonCodes(end + 1, 1) = ...
            "ggv_diagnostic_structure_invalid";
    end
end

health = finalize(health, reasonCodes);
end

function [valid, expectedSize] = baseStructureValid(ggv)
expectedSize = [numel(ggv.v_mps), numel(ggv.ay_g)];
valid = isnumeric(ggv.v_mps) && isreal(ggv.v_mps) ...
    && isvector(ggv.v_mps) && ~isempty(ggv.v_mps) ...
    && isnumeric(ggv.ay_g) && isreal(ggv.ay_g) ...
    && isvector(ggv.ay_g) && ~isempty(ggv.ay_g) ...
    && all(diff(ggv.v_mps(:)) > 0) ...
    && all(diff(ggv.ay_g(:)) > 0) ...
    && isnumeric(ggv.ax_max_g) && isreal(ggv.ax_max_g) ...
    && isnumeric(ggv.ax_min_g) && isreal(ggv.ax_min_g) ...
    && isequal(size(ggv.ax_max_g), expectedSize) ...
    && isequal(size(ggv.ax_min_g), expectedSize) ...
    && (islogical(ggv.feasible) || isnumeric(ggv.feasible)) ...
    && isreal(ggv.feasible) && all(isfinite(ggv.feasible), "all") ...
    && isequal(size(ggv.feasible), expectedSize) ...
    && isnumeric(ggv.ay_limit_pos_g) ...
    && isnumeric(ggv.ay_limit_neg_g) ...
    && numel(ggv.ay_limit_pos_g) == expectedSize(1) ...
    && numel(ggv.ay_limit_neg_g) == expectedSize(1) ...
    && isequal(size(ggv.accel_limiter), expectedSize) ...
    && isequal(size(ggv.brake_limiter), expectedSize) ...
    && isnumeric(ggv.gravity_mps2) && isreal(ggv.gravity_mps2) ...
    && isscalar(ggv.gravity_mps2);
end

function health = assessFixedPoints(health, ggv, feasible)
accelConverged = logical(ggv.solve_converged_accel);
brakeConverged = logical(ggv.solve_converged_brake);
failedCount = nnz(feasible & (~accelConverged | ~brakeConverged));
health.fixed_point = passFailDiagnostic(failedCount);
end

function health = assessResiduals(health, ggv, feasible, options)
accelResidual = abs(ggv.solve_residual_accel_mps2(feasible));
brakeResidual = abs(ggv.solve_residual_brake_mps2(feasible));
residuals = [accelResidual(:); brakeResidual(:)];
if isempty(residuals)
    maximumResidual = NaN;
    residualFailureCount = 0;
else
    maximumResidual = max(residuals);
    residualFailureCount = nnz(~isfinite(residuals) ...
        | residuals > options.ggv_tolerance_mps2);
end
health.residual = passFailDiagnostic(residualFailureCount);
health.residual.max_abs_mps2 = maximumResidual;
health.residual.tolerance_mps2 = options.ggv_tolerance_mps2;
end

function [valid, failureCount] = finiteFixedPointDiagnostics(ggv, names)
valid = true;
failureCount = 0;
for iName = 1:numel(names)
    value = ggv.(names(iName));
    if ~(islogical(value) || isnumeric(value)) || ~isreal(value)
        valid = false;
        failureCount = failureCount + numel(value);
    else
        thisFailureCount = nnz(~isfinite(value));
        valid = valid && thisFailureCount == 0;
        failureCount = failureCount + thisFailureCount;
    end
end
failureCount = max(failureCount, double(~valid));
end

function result = passFailDiagnostic(failureCount)
if failureCount == 0
    result = diagnostic("pass", 0);
else
    result = diagnostic("fail", failureCount);
end
end

function result = diagnostic(status, failureCount)
if nargin < 2
    failureCount = 0;
end
result.status = string(status);
result.failure_count = failureCount;
end

function result = addDiagnosticFailure(result, failureCount)
result.status = "fail";
result.failure_count = result.failure_count + failureCount;
end

function health = finalize(health, reasonCodes)
health.reason_codes = unique(reasonCodes, "stable");
health.valid = isempty(health.reason_codes);
health.healthy = health.valid;
if health.valid
    health.status = "healthy";
else
    health.status = "unhealthy";
end
end

function label = sourceLabel(ggv)
if isfield(ggv, "source") && isscalar(string(ggv.source))
    label = string(ggv.source);
else
    label = "unknown";
end
end
