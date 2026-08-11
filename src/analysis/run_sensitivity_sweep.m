function sweepResult = run_sensitivity_sweep(baseConfig, sweepDefinition)
%RUN_SENSITIVITY_SWEEP Run valid-only one-parameter sensitivity cases.
%   Case execution and rejection use the same contract as run_doe so invalid
%   designs cannot enter the sensitivity curve.

arguments
    baseConfig (1,1) struct
    sweepDefinition (1,1) struct
end

if ~isfield(sweepDefinition, "parameter") ...
        || ~isfield(sweepDefinition, "values") ...
        || ~isnumeric(sweepDefinition.values) ...
        || isempty(sweepDefinition.values)
    error("QSSLTS:SweepDefinition", ...
        "sweep_def requires a parameter and nonempty numeric values.");
end

parameter = string(sweepDefinition.parameter);
values = sweepDefinition.values(:);
caseTable = table(values);
caseTable.Properties.VariableNames = cellstr(parameter);
doe = run_doe(baseConfig, caseTable);
validCases = doe.all_cases(doe.all_cases.valid, :);
validValues = validCases.(parameter);

sweepResult.table = table(validValues, validCases.lap_time_s, ...
    validCases.delta_lap_time_s, ...
    VariableNames=["parameter_value", "lap_time_s", "delta_lap_time_s"]);
sweepResult.parameter = parameter;
sweepResult.baseline_lap_time_s = doe.baseline_lap_time_s;
sweepResult.baseline_result = doe.baseline_result;
sweepResult.case_results = doe.case_results;
sweepResult.case_assessments = doe.case_assessments;
sweepResult.calibration_mode = doe.calibration_mode;
sweepResult.all_cases = doe.all_cases;
sweepResult.rejected_cases = doe.rejected_cases;
sweepResult.valid_case_count = doe.valid_case_count;
sweepResult.rejected_case_count = doe.rejected_case_count;
sweepResult.status = doe.status;
sweepResult.figure = gobjects(0);

makePlot = isfield(sweepDefinition, "make_plot") ...
    && logical(sweepDefinition.make_plot);
if makePlot
    sweepResult.figure = figure(Name="QSSLTS sensitivity: " + parameter);
    plot(sweepResult.table.parameter_value, ...
        sweepResult.table.lap_time_s, "o-", LineWidth=1.5);
    grid on
    xlabel(strrep(parameter, "_", "\_"));
    ylabel("Lap time [s]");
    title("Valid-only parameter sensitivity");
end
end
