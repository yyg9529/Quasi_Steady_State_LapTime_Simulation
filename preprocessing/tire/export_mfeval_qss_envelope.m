function manifest = export_mfeval_qss_envelope(tireDirectory, outputDirectory)
%EXPORT_MFEVAL_QSS_ENVELOPE Export conservative lightweight QSS envelopes.
%   MFeval is used only here, during offline preprocessing. The saved MAT
%   artifact contains numeric reduced parameters and traceable samples; it
%   does not contain an evaluator or a full PAC2002 parameter structure.

arguments
    tireDirectory (1, 1) string
    outputDirectory (1, 1) string = tireDirectory
end

if ~isfolder(tireDirectory)
    error("QSSLTS:QssEnvelopeExportDirectory", ...
        "Tire directory does not exist: %s", tireDirectory);
end
if ~isfolder(outputDirectory)
    error("QSSLTS:QssEnvelopeOutputDirectory", ...
        "QSS envelope output directory does not exist: %s", ...
        outputDirectory);
end

tirFiles = dir(fullfile(tireDirectory, "*.tir"));
if isempty(tirFiles)
    error("QSSLTS:QssEnvelopeExportEmpty", ...
        "No .tir files were found in %s.", tireDirectory);
end

[~, order] = sort(lower(string({tirFiles.name})));
tirFiles = tirFiles(order);
manifest = repmat(struct( ...
    "tir_file", "", "envelope_file", "", ...
    "source_sha256", "", "mfeval_version", ""), ...
    numel(tirFiles), 1);

for fileIndex = 1:numel(tirFiles)
    tirFile = string(fullfile( ...
        tirFiles(fileIndex).folder, tirFiles(fileIndex).name));
    model = load_pac2002_tire(tirFile);
    extraction = defaultExtraction(model);
    pureBoundary = samplePureBoundary(model, extraction);
    [longitudinalFit, lateralFit] = fitPureBoundary( ...
        pureBoundary, model.nominal_load_N);
    combinedBoundary = sampleCombinedBoundary( ...
        model, extraction, pureBoundary);
    combinedFit = fitCombinedBoundary(combinedBoundary);

    tire = makeReducedTire(model, longitudinalFit, lateralFit, ...
        combinedFit, extraction);
    qss_envelope = makeArtifact(model, tire, pureBoundary, ...
        combinedBoundary, longitudinalFit, lateralFit, ...
        combinedFit, extraction);

    [~, stem] = fileparts(tirFile);
    envelopeFile = string(fullfile(outputDirectory, ...
        stem + ".qss-envelope.mat"));
    save(char(envelopeFile), "qss_envelope", "-v7");

    manifest(fileIndex).tir_file = tirFile;
    manifest(fileIndex).envelope_file = envelopeFile;
    manifest(fileIndex).source_sha256 = model.source.sha256;
    manifest(fileIndex).mfeval_version = model.mfeval.version;
end
end

function extraction = defaultExtraction(model)
preferredLoads_N = [100; 200; 300; 400; 500; 667; 800; 1000; ...
    1250; 1500; 1750; 2000; 2500; 3000];
loads_N = preferredLoads_N( ...
    preferredLoads_N >= model.ranges.Fz_N(1) & ...
    preferredLoads_N <= model.ranges.Fz_N(2));
loads_N = unique([loads_N; model.nominal_load_N]);
if numel(loads_N) < 4
    loads_N = linspace(model.ranges.Fz_N(1), ...
        model.ranges.Fz_N(2), 8).';
    loads_N = unique([loads_N; model.nominal_load_N]);
end

preferredCombinedLoads_N = [300; 667; 1000; 1500; 2000];
combinedLoads_N = preferredCombinedLoads_N( ...
    ismember(preferredCombinedLoads_N, loads_N));
if numel(combinedLoads_N) < 3
    combinedLoads_N = loads_N(round(linspace( ...
        1, numel(loads_N), min(5, numel(loads_N)))));
end

extraction.Fz_N = loads_N;
extraction.combined_Fz_N = unique(combinedLoads_N);
extraction.speed_mps = max(model.reference_speed_mps, ...
    1.1 * model.low_speed_threshold_mps);
extraction.camber_rad = 0;
extraction.turn_slip_1pm = 0;
extraction.mount_side = "LEFT";
extraction.kappa_grid = linspace( ...
    model.ranges.kappa(1), model.ranges.kappa(2), 1201).';
extraction.alpha_grid_rad = linspace( ...
    model.ranges.alpha_rad(1), model.ranges.alpha_rad(2), 1601).';
extraction.combined_kappa_grid = linspace( ...
    max(model.ranges.kappa(1), -0.5), ...
    min(model.ranges.kappa(2), 0.5), 161).';
extraction.combined_alpha_grid_rad = linspace( ...
    max(model.ranges.alpha_rad(1), -0.8), ...
    min(model.ranges.alpha_rad(2), 0.8), 201).';
extraction.combined_y_targets = (0.1:0.1:0.9).';
extraction.combined_y_bin_half_width = 0.0125;
extraction.combined_n_candidates = (1:0.001:4).';
end

function boundary = samplePureBoundary(model, extraction)
nLoads = numel(extraction.Fz_N);
FxPeak_N = zeros(nLoads, 1);
FyPeak_N = zeros(nLoads, 1);
kappaAtPeak = zeros(nLoads, 1);
alphaAtPeak_rad = zeros(nLoads, 1);
for loadIndex = 1:nLoads
    Fz_N = extraction.Fz_N(loadIndex);
    longitudinal = evaluateGrid(model, Fz_N, ...
        extraction.kappa_grid, zeros(size(extraction.kappa_grid)), ...
        extraction);
    [FxPeak_N(loadIndex), peakIndex] = max(abs(longitudinal.Fx_N));
    kappaAtPeak(loadIndex) = extraction.kappa_grid(peakIndex);

    lateral = evaluateGrid(model, Fz_N, ...
        zeros(size(extraction.alpha_grid_rad)), ...
        extraction.alpha_grid_rad, extraction);
    [FyPeak_N(loadIndex), peakIndex] = max(abs(lateral.Fy_N));
    alphaAtPeak_rad(loadIndex) = ...
        extraction.alpha_grid_rad(peakIndex);
end

boundary.Fz_N = extraction.Fz_N;
boundary.Fx_peak_N = FxPeak_N;
boundary.Fy_peak_N = FyPeak_N;
boundary.mu_x_peak = FxPeak_N ./ extraction.Fz_N;
boundary.mu_y_peak = FyPeak_N ./ extraction.Fz_N;
boundary.kappa_at_Fx_peak = kappaAtPeak;
boundary.alpha_at_Fy_peak_rad = alphaAtPeak_rad;
end

function [longitudinal, lateral] = fitPureBoundary(boundary, FzRef_N)
longitudinal = conservativeLinearFit( ...
    boundary.Fz_N, boundary.mu_x_peak, FzRef_N);
lateral = conservativeLinearFit( ...
    boundary.Fz_N, boundary.mu_y_peak, FzRef_N);
end

function fit = conservativeLinearFit(Fz_N, sampledMu, FzRef_N)
normalizedLoad = (Fz_N - FzRef_N) / FzRef_N;
design = [ones(size(normalizedLoad)), normalizedLoad];
rawCoefficients = design \ sampledMu;
rawPrediction = design * rawCoefficients;
if rawCoefficients(1) <= 0 || any(rawPrediction <= 0)
    error("QSSLTS:QssEnvelopePureFit", ...
        "The sampled pure-slip boundary cannot form a positive QSS fit.");
end

conservativeScale = max([1; rawPrediction ./ sampledMu]);
conservativeScale = conservativeScale * (1 + 32 * eps);
coefficients = rawCoefficients / conservativeScale;
prediction = design * coefficients;

fit.Fz_ref_N = FzRef_N;
fit.mu_ref = coefficients(1);
fit.load_sensitivity = coefficients(2) / coefficients(1);
fit.raw_mu_ref = rawCoefficients(1);
fit.raw_load_sensitivity = rawCoefficients(2) / rawCoefficients(1);
fit.conservative_scale = conservativeScale;
fit.predicted_mu = prediction;
fit.max_force_excess_N = max((prediction - sampledMu) .* Fz_N);
fit.rms_mu_error = sqrt(mean((prediction - sampledMu) .^ 2));
end

function boundary = sampleCombinedBoundary( ...
        model, extraction, pureBoundary)
[kappa, alphaRad] = ndgrid(extraction.combined_kappa_grid, ...
    extraction.combined_alpha_grid_rad);
kappa = kappa(:);
alphaRad = alphaRad(:);
nLoads = numel(extraction.combined_Fz_N);
nTargets = numel(extraction.combined_y_targets);
FzSamples_N = repelem(extraction.combined_Fz_N, nTargets);
yTargets = repmat(extraction.combined_y_targets, nLoads, 1);
xBoundary = nan(size(yTargets));

for loadIndex = 1:nLoads
    Fz_N = extraction.combined_Fz_N(loadIndex);
    output = evaluateGrid(model, Fz_N, kappa, alphaRad, extraction);
    FxPeak_N = interp1(pureBoundary.Fz_N, ...
        pureBoundary.Fx_peak_N, Fz_N, "linear");
    FyPeak_N = interp1(pureBoundary.Fz_N, ...
        pureBoundary.Fy_peak_N, Fz_N, "linear");
    xFraction = abs(output.Fx_N) / FxPeak_N;
    yFraction = abs(output.Fy_N) / FyPeak_N;
    first = (loadIndex - 1) * nTargets + 1;
    last = loadIndex * nTargets;
    xBoundary(first:last) = boundaryAtTargets( ...
        xFraction, yFraction, extraction.combined_y_targets, ...
        extraction.combined_y_bin_half_width);
end

if any(~isfinite(xBoundary))
    error("QSSLTS:QssEnvelopeCombinedSamples", ...
        "The combined-slip grid did not cover every normalized target.");
end
boundary.Fz_N = FzSamples_N;
boundary.y_fraction = yTargets;
boundary.x_boundary_fraction = xBoundary;
boundary.y_bin_half_width = ...
    extraction.combined_y_bin_half_width;
end

function boundary = boundaryAtTargets( ...
        xFraction, yFraction, targets, halfWidth)
boundary = nan(size(targets));
for targetIndex = 1:numel(targets)
    selected = abs(yFraction - targets(targetIndex)) <= halfWidth;
    if any(selected)
        boundary(targetIndex) = max(xFraction(selected));
    end
end
end

function fit = fitCombinedBoundary(boundary)
candidates = (1:0.001:4).';
valid = false(size(candidates));
maxExcess = nan(size(candidates));
for candidateIndex = 1:numel(candidates)
    exponent = candidates(candidateIndex);
    predicted = max(0, 1 - boundary.y_fraction .^ exponent) ...
        .^ (1 / exponent);
    maxExcess(candidateIndex) = max( ...
        predicted - boundary.x_boundary_fraction);
    valid(candidateIndex) = maxExcess(candidateIndex) <= 0;
end
if ~any(valid)
    error("QSSLTS:QssEnvelopeCombinedFit", ...
        "No conservative p-norm exponent fits the sampled boundary.");
end
selectedIndex = find(valid, 1, "last");
fit.combined_n = candidates(selectedIndex);
fit.max_boundary_excess = maxExcess(selectedIndex);
fit.candidate_step = 0.001;
fit.candidate_range = [1, 4];
fit.basis = "largest sampled p-norm exponent that does not " + ...
    "exceed any normalized MFeval combined-slip boundary point";
end

function output = evaluateGrid(model, Fz_N, kappa, alphaRad, extraction)
count = numel(kappa);
input.Fz_N = repmat(Fz_N, count, 1);
input.kappa = kappa;
input.alpha_rad = alphaRad;
input.gamma_rad = repmat(extraction.camber_rad, count, 1);
input.turn_slip_1pm = repmat( ...
    extraction.turn_slip_1pm, count, 1);
input.Vx_mps = repmat(extraction.speed_mps, count, 1);
input.mount_side = repmat(extraction.mount_side, count, 1);
output = model.evaluate(input);
end

function tire = makeReducedTire(model, longitudinal, lateral, ...
        combined, extraction)
tire.model_type = "load_sensitive";
tire.Fz_ref_N = model.nominal_load_N;
tire.mu_x_ref = longitudinal.mu_ref;
tire.mu_y_ref = lateral.mu_ref;
tire.load_sensitivity_x = longitudinal.load_sensitivity;
tire.load_sensitivity_y = lateral.load_sensitivity;
tire.combined_n = combined.combined_n;
tire.provenance.source_tir_file_name = model.source.file_name;
tire.provenance.source_tir_sha256 = upper(string(model.source.sha256));
tire.provenance.reduction = ...
    "offline MFeval pure/combined-slip conservative QSS boundary fit";
tire.provenance.mfeval_version = model.mfeval.version;
tire.provenance.combined_n_basis = combined.basis;
tire.provenance.speed_mps = extraction.speed_mps;
tire.provenance.camber_rad = extraction.camber_rad;
tire.provenance.load_range_N = ...
    [min(extraction.Fz_N), max(extraction.Fz_N)];
end

function artifact = makeArtifact(model, tire, pureBoundary, ...
        combinedBoundary, longitudinalFit, lateralFit, ...
        combinedFit, extraction)
artifact.schema_version = 1;
artifact.source.file_name = model.source.file_name;
artifact.source.sha256 = upper(string(model.source.sha256));
artifact.mfeval.version = model.mfeval.version;
artifact.mfeval.use_mode = model.mfeval.use_mode;
artifact.tire = tire;
artifact.pure_boundary = pureBoundary;
artifact.combined_boundary = combinedBoundary;
artifact.fit.longitudinal = longitudinalFit;
artifact.fit.lateral = lateralFit;
artifact.fit.combined = combinedFit;
artifact.extraction.Fz_N = extraction.Fz_N;
artifact.extraction.combined_Fz_N = extraction.combined_Fz_N;
artifact.extraction.speed_mps = extraction.speed_mps;
artifact.extraction.camber_rad = extraction.camber_rad;
artifact.extraction.kappa_grid_size = numel(extraction.kappa_grid);
artifact.extraction.alpha_grid_size = ...
    numel(extraction.alpha_grid_rad);
artifact.extraction.combined_kappa_grid_size = ...
    numel(extraction.combined_kappa_grid);
artifact.extraction.combined_alpha_grid_size = ...
    numel(extraction.combined_alpha_grid_rad);
artifact.exported_at_utc = datetime("now", TimeZone="UTC");
end
