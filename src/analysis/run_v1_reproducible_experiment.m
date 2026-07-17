function report = run_v1_reproducible_experiment(outputDir)
%RUN_V1_REPRODUCIBLE_EXPERIMENT Run and write the deterministic V1 study.
%   The DOE output is a software-reproducibility report, not physical truth.

    arguments
        outputDir (1, 1) string
    end

    projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
    baseConfig = buildBaselineConfig(projectRoot);
    doeTable = buildDoeTable();
    doe = run_doe(baseConfig, doeTable);
    baselineResult = doe.baseline_result;
    baselineSummary = summarize_lap_result(baselineResult);
    baselineLimiter = summarize_limiter_usage(baselineResult);
    baselineProfile = makeProfileTable(baselineResult);
    caseMetrics = makeCaseMetrics(doe);
    if ~baselineResult.solver.converged ...
            || ~isfinite(baselineResult.lap_time_s)
        error("QSSLTS:V1ExperimentConvergence", ...
            "The baseline must converge with a finite lap time.");
    end
    if ~all(caseMetrics.solver_converged) ...
            || any(~isfinite(caseMetrics.lap_time_s))
        error("QSSLTS:V1ExperimentConvergence", ...
            "Every DOE case must converge with a finite lap time.");
    end

    if ~isfolder(outputDir)
        mkdir(outputDir);
    end
    outputFiles = reportPaths(outputDir);
    scalarBaselineSummary = rmfield(baselineSummary, "limiter_table");
    writetable(struct2table(scalarBaselineSummary, AsArray=true), ...
        outputFiles.baseline_summary_csv);
    writetable(baselineProfile, outputFiles.baseline_profile_csv);
    writetable(baselineLimiter, outputFiles.baseline_limiter_csv);
    writetable(doe.ranking, outputFiles.doe_ranking_csv);
    writetable(caseMetrics, outputFiles.doe_case_metrics_csv);
    writetable(doe.extreme_limiter_comparison, ...
        outputFiles.doe_extreme_limiter_csv);

    manifest = makeManifest(baseConfig, doe, projectRoot);
    writelines(jsonencode(manifest, PrettyPrint=true), ...
        outputFiles.manifest_json);

    report.manifest = manifest;
    report.baseline_result = baselineResult;
    report.baseline_summary = baselineSummary;
    report.baseline_limiter = baselineLimiter;
    report.baseline_profile = baselineProfile;
    report.doe = doe;
    report.doe_case_metrics = caseMetrics;
    report.output_files = outputFiles;
end

function config = buildBaselineConfig(projectRoot)
    config = struct();
    config.track = read_track_csv(fullfile(projectRoot, ...
        "data", "track", "simple_track.csv"));
    config.vehicle = vehicle_baseline();
    config.models = struct();
    config.models.tire = tire_load_sensitive_baseline();
    config.models.aero = aero_baseline();
    config.models.powertrain = powertrain_emrax228_hvcc_demo();
    config.models.brake = brake_baseline();
    config.models.endurance = struct( ...
        "num_laps", 1, "safety_factor", 1.0);
    config.options = default_qss_options();
    config.options.v_grid_mps = [0:2:44, 45].';
    config.options.ay_grid_g = -4:0.2:4;
    config.options.calibration_report_file = "";
end

function cases = buildDoeTable()
    [mass_kg, inverter_power_W, tire_mu_scale] = ndgrid( ...
        [280, 300, 320], [60000, 80000, 100000], [0.9, 1.0, 1.1]);
    cases = table(mass_kg(:), inverter_power_W(:), tire_mu_scale(:), ...
        VariableNames=["mass_kg", "inverter_power_W", "tire_mu_scale"]);
end

function profile = makeProfileTable(result)
    profile = table(result.s_m, result.v_mps, result.ax_mps2, ...
        result.ay_mps2, result.track.kappa_1pm(:), ...
        string(result.limiter(:)), ...
        VariableNames=["s_m", "v_mps", "ax_mps2", "ay_mps2", ...
        "kappa_1pm", "limiter"]);
end

function metrics = makeCaseMetrics(doe)
    caseCount = numel(doe.case_results);
    case_id = (1:caseCount).';
    lap_time_s = zeros(caseCount, 1);
    delta_lap_time_s = zeros(caseCount, 1);
    max_speed_mps = zeros(caseCount, 1);
    max_ax_g = zeros(caseCount, 1);
    max_brake_g = zeros(caseCount, 1);
    max_ay_g = zeros(caseCount, 1);
    dominant_limiter = strings(caseCount, 1);
    dominant_limiter_percent = zeros(caseCount, 1);
    solver_converged = false(caseCount, 1);
    for index = 1:caseCount
        summary = summarize_lap_result(doe.case_results{index});
        usage = summary.limiter_table;
        lap_time_s(index) = summary.lap_time_s;
        delta_lap_time_s(index) = ...
            summary.lap_time_s - doe.baseline_lap_time_s;
        max_speed_mps(index) = summary.max_speed_mps;
        max_ax_g(index) = summary.max_ax_g;
        max_brake_g(index) = summary.max_brake_g;
        max_ay_g(index) = summary.max_ay_g;
        dominant_limiter(index) = usage.limiter(1);
        dominant_limiter_percent(index) = usage.percent_distance(1);
        solver_converged(index) = doe.case_results{index}.solver.converged;
    end
    metrics = table(case_id, lap_time_s, delta_lap_time_s, ...
        max_speed_mps, max_ax_g, max_brake_g, max_ay_g, ...
        dominant_limiter, dominant_limiter_percent, solver_converged);
end

function files = reportPaths(outputDir)
    files.manifest_json = fullfile(outputDir, "experiment_manifest.json");
    files.baseline_summary_csv = fullfile(outputDir, "baseline_summary.csv");
    files.baseline_profile_csv = fullfile(outputDir, "baseline_profile.csv");
    files.baseline_limiter_csv = fullfile(outputDir, "baseline_limiter.csv");
    files.doe_ranking_csv = fullfile(outputDir, "doe_ranking.csv");
    files.doe_case_metrics_csv = fullfile(outputDir, "doe_case_metrics.csv");
    files.doe_extreme_limiter_csv = fullfile( ...
        outputDir, "doe_extreme_limiter_comparison.csv");
end

function manifest = makeManifest(config, doe, projectRoot)
    manifest.schema_version = "QSSLTS_V1_REPORT_V1";
    manifest.project_version = "V1.0";
    manifest.experiment_id = "theory_only_simple_track_27_case_doe";
    manifest.track_input = "data/track/simple_track.csv";
    manifest.calibration_mode = doe.calibration_mode;
    manifest.doe_case_count = height(doe.ranking);
    manifest.gravity_mps2 = config.options.gravity_mps2;
    manifest.v_grid_mps = config.options.v_grid_mps.';
    manifest.ay_grid_g = config.options.ay_grid_g;
    manifest.baseline.track = makeTrackManifest(config.track);
    manifest.baseline.vehicle = makeJsonSafe(config.vehicle);
    manifest.baseline.tire = makeJsonSafe( ...
        tire_envelope_provenance(config.models.tire));
    manifest.baseline.aero = makeJsonSafe(config.models.aero);
    manifest.baseline.powertrain = makeJsonSafe(config.models.powertrain);
    manifest.baseline.brake = makeJsonSafe(config.models.brake);
    manifest.baseline.endurance = makeJsonSafe(config.models.endurance);
    manifest.baseline.options = makeJsonSafe(config.options);
    manifest.baseline.model_functions = struct( ...
        "vehicle", "vehicle_baseline", ...
        "tire", "tire_load_sensitive_baseline", ...
        "aero", "aero_baseline", ...
        "powertrain", "powertrain_emrax228_hvcc_demo", ...
        "brake", "brake_baseline");
    manifest.doe.design = "full_factorial_3x3x3";
    manifest.doe.parameter_names = doe.parameter_names(:);
    manifest.doe.mass_kg = [280; 300; 320];
    manifest.doe.inverter_power_W = [60000; 80000; 100000];
    manifest.doe.tire_mu_scale = [0.9; 1.0; 1.1];
    manifest.doe.case_count = height(doe.ranking);
    manifest.doe.all_cases_converged = all(cellfun( ...
        @(result) result.solver.converged, doe.case_results));
    manifest.doe.all_lap_times_finite = all(isfinite( ...
        doe.ranking.lap_time_s));
    manifest.doe.calibration_mode = doe.calibration_mode;
    manifest.environment.matlab_release = string(version("-release"));
    manifest.environment.matlab_version = string(version);
    manifest.environment.computer = string(computer);
    manifest.environment.is_pc = ispc;
    manifest.code_identity.method = "SHA-256 per production input file";
    manifest.code_identity.source_files = makeSourceIdentity(projectRoot);
    manifest.note = ...
        "Deterministic implementation report; not external physical validation.";
end

function entries = makeSourceIdentity(projectRoot)
    files = [dir(fullfile(projectRoot, "src", "**", "*.m")); ...
        dir(fullfile(projectRoot, "preprocessing", "**", "*.m")); ...
        dir(fullfile(projectRoot, "data", "**", "*.m")); ...
        dir(fullfile(projectRoot, "data", "track", "simple_track.csv"))];
    fullPaths = string(fullfile({files.folder}, {files.name})).';
    relativePaths = extractAfter(fullPaths, strlength(projectRoot) + 1);
    relativePaths = replace(relativePaths, "\", "/");
    [relativePaths, order] = sort(relativePaths);
    fullPaths = fullPaths(order);
    entries = repmat(struct("path", "", "sha256", ""), ...
        numel(fullPaths), 1);
    for index = 1:numel(fullPaths)
        entries(index).path = relativePaths(index);
        entries(index).sha256 = sha256File(fullPaths(index));
    end
end

function digest = sha256File(filePath)
    fileId = fopen(filePath, "rb");
    if fileId < 0
        error("QSSLTS:ReportManifest", ...
            "Cannot read source identity file: %s", filePath);
    end
    cleanup = onCleanup(@() fclose(fileId));
    bytes = fread(fileId, inf, "*uint8");
    messageDigest = java.security.MessageDigest.getInstance("SHA-256");
    messageDigest.update(bytes);
    digestBytes = typecast(messageDigest.digest(), "uint8");
    digest = string(lower(reshape(dec2hex(digestBytes, 2).', 1, [])));
    clear cleanup
end

function track = makeTrackManifest(configTrack)
    track.source_file = "data/track/simple_track.csv";
    track.s_m = configTrack.s_m(:);
    track.ds_m = configTrack.ds_m(:);
    track.kappa_1pm = configTrack.kappa_1pm(:);
    track.is_closed = logical(configTrack.is_closed);
    track.length_m = configTrack.length_m;
end

function value = makeJsonSafe(value)
    if isstruct(value)
        names = fieldnames(value);
        for index = 1:numel(names)
            name = names{index};
            value.(name) = makeJsonSafe(value.(name));
        end
    elseif isnumeric(value) && any(~isfinite(value), "all")
        if ~isscalar(value)
            error("QSSLTS:ReportManifest", ...
                "Nonfinite numeric arrays require an explicit schema.");
        elseif isnan(value)
            value = "NaN";
        elseif value > 0
            value = "Inf";
        else
            value = "-Inf";
        end
    end
end
