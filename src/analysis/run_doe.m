function doeResult = run_doe(baseConfig, doeTable)
%RUN_DOE Run a tabular multi-parameter design of experiments.
%   Supported compact column names are documented in apply_analysis_parameter.
%   Dot-delimited table variable names are also accepted.

arguments
    baseConfig (1,1) struct
    doeTable table
end

if height(doeTable) < 1 || width(doeTable) < 1
    error("QSSLTS:DOETable", "DOE table must contain cases and parameters.");
end

context = prepare_analysis_context(baseConfig);
baselineResult = run_analysis_case( ...
    context.base_config, context.scale_table);
nCase = height(doeTable);
parameterNames = string(doeTable.Properties.VariableNames);
caseResults = cell(nCase, 1);
lapTime_s = zeros(nCase, 1);

for iCase = 1:nCase
    caseConfig = context.base_config;
    for iParameter = 1:numel(parameterNames)
        value = doeTable{iCase, iParameter};
        if iscell(value)
            value = value{1};
        end
        caseConfig = apply_analysis_parameter( ...
            caseConfig, parameterNames(iParameter), value);
    end
    caseResults{iCase} = run_analysis_case(caseConfig, context.scale_table);
    lapTime_s(iCase) = caseResults{iCase}.lap_time_s;
end

case_id = (1:nCase).';
delta_lap_time_s = lapTime_s - baselineResult.lap_time_s;
ranking = [table(case_id, lapTime_s, delta_lap_time_s, ...
    VariableNames=["case_id", "lap_time_s", "delta_lap_time_s"]), doeTable];
ranking = sortrows(ranking, "lap_time_s", "ascend");
fastestId = ranking.case_id(1);
slowestId = ranking.case_id(end);

doeResult.ranking = ranking;
doeResult.case_results = caseResults;
doeResult.baseline_lap_time_s = baselineResult.lap_time_s;
doeResult.fastest_case_id = fastestId;
doeResult.slowest_case_id = slowestId;
doeResult.fastest_summary = summarize_lap_result(caseResults{fastestId});
doeResult.slowest_summary = summarize_lap_result(caseResults{slowestId});
doeResult.calibration_mode = context.calibration_mode;
doeResult.parameter_names = parameterNames;
end
