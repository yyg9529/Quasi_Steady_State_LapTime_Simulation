function sweepResult = run_sensitivity_sweep(baseConfig, sweepDefinition)
%RUN_SENSITIVITY_SWEEP Run one-parameter QSS sensitivity cases.
%   Every case regenerates theoretical GGV. If real GGV is supplied, one
%   baseline residual scale table is frozen and applied to every case.

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
context = prepare_analysis_context(baseConfig);
baselineResult = run_analysis_case( ...
    context.base_config, context.scale_table);
caseResults = cell(numel(values), 1);
lapTime_s = zeros(numel(values), 1);

for iCase = 1:numel(values)
    caseConfig = apply_analysis_parameter( ...
        context.base_config, parameter, values(iCase));
    caseResults{iCase} = run_analysis_case(caseConfig, context.scale_table);
    lapTime_s(iCase) = caseResults{iCase}.lap_time_s;
end

deltaLapTime_s = lapTime_s - baselineResult.lap_time_s;
sweepResult.table = table(values, lapTime_s, deltaLapTime_s, ...
    VariableNames=["parameter_value", "lap_time_s", "delta_lap_time_s"]);
sweepResult.parameter = parameter;
sweepResult.baseline_lap_time_s = baselineResult.lap_time_s;
sweepResult.case_results = caseResults;
sweepResult.calibration_mode = context.calibration_mode;
sweepResult.figure = gobjects(0);

makePlot = isfield(sweepDefinition, "make_plot") ...
    && logical(sweepDefinition.make_plot);
if makePlot
    sweepResult.figure = figure(Name="QSSLTS sensitivity: " + parameter);
    plot(values, lapTime_s, "o-", LineWidth=1.5);
    grid on
    xlabel(strrep(parameter, "_", "\_"));
    ylabel("Lap time [s]");
    title("Parameter sensitivity");
end
end
