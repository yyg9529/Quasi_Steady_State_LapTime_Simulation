%% V1.0 deterministic 27-case software-reproducibility study
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

outputDir = fullfile(projectRoot, "results", "v1");
report = run_v1_reproducible_experiment(outputDir);

fprintf("Baseline lap time: %.6f s\n", ...
    report.baseline_summary.lap_time_s);
fprintf("Baseline solver converged: %s\n", ...
    string(report.baseline_result.solver.converged));
fprintf("DOE cases: %d; calibration mode: %s\n", ...
    height(report.doe.ranking), report.doe.calibration_mode);
fprintf("Baseline distance-weighted limiter map:\n");
disp(report.baseline_limiter);
fprintf("Five fastest DOE cases:\n");
disp(report.doe.ranking(1:5, :));
fprintf("Fastest versus slowest limiter comparison:\n");
disp(report.doe.extreme_limiter_comparison);
fprintf("Generated files:\n");
disp(struct2table(report.output_files, AsArray=true));
