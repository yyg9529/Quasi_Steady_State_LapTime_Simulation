function manifest = export_mfeval_parameters(tireDirectory)
%EXPORT_MFEVAL_PARAMETERS Export every local TIR through MFeval readTIR.

arguments
    tireDirectory (1, 1) string
end

if ~isfolder(tireDirectory)
    error("QSSLTS:MFevalExportDirectory", ...
        "Tire directory does not exist: %s", tireDirectory);
end

installation = resolve_mfeval_installation();
tirFiles = dir(fullfile(tireDirectory, "*.tir"));
if isempty(tirFiles)
    error("QSSLTS:MFevalExportEmpty", ...
        "No .tir files were found in %s.", tireDirectory);
end

[~, order] = sort(lower(string({tirFiles.name})));
tirFiles = tirFiles(order);
manifest = repmat(struct( ...
    "tir_file", "", "parameter_file", "", ...
    "source_sha256", "", "mfeval_version", ""), ...
    numel(tirFiles), 1);
for index = 1:numel(tirFiles)
    tirFile = string(fullfile(tirFiles(index).folder, tirFiles(index).name));
    parsed = read_pac2002_tir(tirFile);
    parameters = mfeval.readTIR(char(tirFile));
    [~, stem] = fileparts(tirFile);
    parameterFile = string(fullfile(tireDirectory, ...
        stem + ".mfeval.mat"));

    mfeval_export.schema_version = 1;
    mfeval_export.source.file_name = parsed.source.file_name;
    mfeval_export.source.sha256 = parsed.source.sha256;
    mfeval_export.parameters = parameters;
    mfeval_export.mfeval.version = installation.version;
    mfeval_export.mfeval.function_file = installation.function_file;
    mfeval_export.mfeval.use_mode = 111;
    mfeval_export.exported_at_utc = datetime("now", TimeZone="UTC");
    save(char(parameterFile), "mfeval_export", "-v7");

    manifest(index).tir_file = tirFile;
    manifest(index).parameter_file = parameterFile;
    manifest(index).source_sha256 = parsed.source.sha256;
    manifest(index).mfeval_version = installation.version;
end
end
