function doeResult = run_doe(baseConfig, doeTable)
%RUN_DOE Run a tabular multi-parameter design of experiments.
%   Invalid numerical cases and expected design/input errors are isolated;
%   unknown program or contract defects are rethrown.

arguments
    baseConfig (1,1) struct
    doeTable table
end

if height(doeTable) < 1 || width(doeTable) < 1
    error("QSSLTS:DOETable", "DOE table must contain cases and parameters.");
end
validateParameterNames(doeTable);

context = prepare_analysis_context(baseConfig);
baselineResult = runBaseline(context);
baselineAssessment = assess_analysis_result(baselineResult);
if ~baselineAssessment.valid
    error("QSSLTS:DOEInvalidBaseline", ...
        "DOE baseline is invalid: %s", baselineAssessment.reject_reason);
end

nCase = height(doeTable);
parameterNames = string(doeTable.Properties.VariableNames);
caseResults = repmat({struct()}, nCase, 1);
caseAssessments = repmat({struct()}, nCase, 1);
lapTime_s = nan(nCase, 1);
valid = false(nCase, 1);
status = strings(nCase, 1);
rejectReason = strings(nCase, 1);
errorId = strings(nCase, 1);
errorMessage = strings(nCase, 1);
designFeasibilityEvaluated = false(nCase, 1);
designFeasible = nan(nCase, 1);

for iCase = 1:nCase
    try
        caseConfig = applyCaseParameters( ...
            context.base_config, doeTable, parameterNames, iCase);
        caseResults{iCase} = run_analysis_case( ...
            caseConfig, context.scale_table);
        assessment = assess_analysis_result(caseResults{iCase});
        caseAssessments{iCase} = assessment;
        lapTime_s(iCase) = caseResults{iCase}.lap_time_s;
        valid(iCase) = assessment.valid;
        status(iCase) = assessment.status;
        rejectReason(iCase) = assessment.reject_reason;
        designFeasibilityEvaluated(iCase) = ...
            assessment.design_feasibility_evaluated;
        designFeasible(iCase) = assessment.design_feasible;
    catch exception
        [expected, stableReason] = expectedCaseFailure(exception);
        if ~expected
            rethrow(exception)
        end
        assessment.valid = false;
        assessment.status = "execution_error";
        assessment.reject_reason = stableReason;
        assessment.reason_codes = stableReason;
        assessment.design_feasibility_evaluated = false;
        assessment.design_feasible = NaN;
        caseAssessments{iCase} = assessment;
        status(iCase) = assessment.status;
        rejectReason(iCase) = stableReason;
        errorId(iCase) = string(exception.identifier);
        errorMessage(iCase) = string(exception.message);
    end
end

case_id = (1:nCase).';
delta_lap_time_s = lapTime_s - baselineResult.lap_time_s;
allCases = table(case_id, valid, status, rejectReason, errorId, ...
    errorMessage, lapTime_s, delta_lap_time_s, ...
    designFeasibilityEvaluated, designFeasible, ...
    VariableNames=["case_id", "valid", "status", "reject_reason", ...
    "error_id", "error_message", "lap_time_s", "delta_lap_time_s", ...
    "design_feasibility_evaluated", "design_feasible"]);
allCases = [allCases, doeTable];

ranking = [allCases(valid, ...
    ["case_id", "lap_time_s", "delta_lap_time_s"]), doeTable(valid, :)];
ranking = sortrows(ranking, "lap_time_s", "ascend");
rejectedCases = allCases(~valid, :);
validCount = nnz(valid);
rejectedCount = nCase - validCount;

doeResult.ranking = ranking;
doeResult.all_cases = allCases;
doeResult.rejected_cases = rejectedCases;
doeResult.baseline_result = baselineResult;
doeResult.baseline_assessment = baselineAssessment;
doeResult.case_results = caseResults;
doeResult.case_assessments = caseAssessments;
doeResult.baseline_lap_time_s = baselineResult.lap_time_s;
doeResult.total_case_count = nCase;
doeResult.valid_case_count = validCount;
doeResult.rejected_case_count = rejectedCount;
doeResult.calibration_mode = context.calibration_mode;
doeResult.parameter_names = parameterNames;

if validCount == nCase
    doeResult.status = "complete";
elseif validCount > 0
    doeResult.status = "partial";
else
    doeResult.status = "no_valid_cases";
end

if validCount > 0
    fastestId = ranking.case_id(1);
    slowestId = ranking.case_id(end);
    doeResult.fastest_case_id = fastestId;
    doeResult.slowest_case_id = slowestId;
    doeResult.fastest_summary = summarize_lap_result( ...
        caseResults{fastestId});
    doeResult.slowest_summary = summarize_lap_result( ...
        caseResults{slowestId});
    doeResult.fastest_limiter = summarize_limiter_usage( ...
        caseResults{fastestId});
    doeResult.slowest_limiter = summarize_limiter_usage( ...
        caseResults{slowestId});
    doeResult.extreme_limiter_comparison = compare_limiter_usage( ...
        caseResults{fastestId}, caseResults{slowestId});
else
    doeResult.fastest_case_id = NaN;
    doeResult.slowest_case_id = NaN;
    doeResult.fastest_summary = struct();
    doeResult.slowest_summary = struct();
    doeResult.fastest_limiter = table();
    doeResult.slowest_limiter = table();
    doeResult.extreme_limiter_comparison = table();
end
end

function baselineResult = runBaseline(context)
try
    baselineResult = run_analysis_case( ...
        context.base_config, context.scale_table);
catch exception
    [expected, ~] = expectedCaseFailure(exception);
    if ~expected
        rethrow(exception)
    end
    wrapped = MException("QSSLTS:DOEInvalidBaseline", ...
        "DOE baseline execution failed: %s", exception.message);
    wrapped = addCause(wrapped, exception);
    throwAsCaller(wrapped)
end
end

function config = applyCaseParameters(config, doeTable, parameterNames, iCase)
for iParameter = 1:numel(parameterNames)
    value = doeTable{iCase, iParameter};
    if iscell(value)
        value = value{1};
    end
    config = apply_analysis_parameter( ...
        config, parameterNames(iParameter), value);
end
end

function [expected, reason] = expectedCaseFailure(exception)
identifiers = [ ...
    "QSSLTS:AnalysisParameter", "QSSLTS:AnalysisRulesImmutable", ...
    "QSSLTS:FieldPath", "QSSLTS:GearRatio", "QSSLTS:TireMuScale", ...
    "QSSLTS:VehicleParameters", "QSSLTS:StaticLoadParameters", ...
    "QSSLTS:LoadTransferParameters", "QSSLTS:AeroParameters", ...
    "QSSLTS:BrakeParameters", "QSSLTS:TireParameters", ...
    "QSSLTS:TireLoadSensitivityParameters", ...
    "QSSLTS:PowertrainConfig", "QSSLTS:PowertrainMotorCount", ...
    "QSSLTS:PowertrainThermalUnsupported", ...
    "QSSLTS:PowertrainRegenUnsupported", ...
    "QSSLTS:DrivetrainMismatch", "QSSLTS:AxleLiftModelInvalid", ...
    "QSSLTS:GGVBoundaryCoverage", "QSSLTS:ZeroSegmentSpeed", ...
    "QSSLTS:EnergyZeroSpeedSegment", "QSSLTS:GGVUnusable"];
reasons = [ ...
    "analysis_parameter", "analysis_rules_immutable", ...
    "field_path", "gear_ratio", "tire_mu_scale", ...
    "vehicle_parameters", "static_load_parameters", ...
    "load_transfer_parameters", "aero_parameters", ...
    "brake_parameters", "tire_parameters", ...
    "tire_load_sensitivity_parameters", ...
    "powertrain_config", "powertrain_motor_count", ...
    "powertrain_thermal_unsupported", "powertrain_regen_unsupported", ...
    "drivetrain_mismatch", "axle_lift", "ggv_boundary_coverage", ...
    "zero_segment_speed", "energy_zero_speed_segment", "ggv_unusable"];
index = find(string(exception.identifier) == identifiers, 1);
expected = ~isempty(index);
if expected
    reason = reasons(index);
else
    reason = "";
end
end

function validateParameterNames(doeTable)
reserved = ["case_id", "valid", "status", "reject_reason", ...
    "error_id", "error_message", "lap_time_s", "delta_lap_time_s", ...
    "design_feasibility_evaluated", "design_feasible"];
names = string(doeTable.Properties.VariableNames);
if any(ismember(names, reserved))
    error("QSSLTS:DOEReservedColumn", ...
        "DOE parameter columns conflict with result columns.");
end
end
