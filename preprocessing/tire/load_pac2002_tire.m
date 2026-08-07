function model = load_pac2002_tire(filePath)
%LOAD_PAC2002_TIRE Load a complete scalar PAC2002 TIR model.

arguments
    filePath (1, 1) string
end

if ~isfile(filePath)
    error("load_pac2002_tire:FileNotFound", ...
        "PAC2002 TIR file does not exist: %s", filePath);
end

parsed = read_pac2002_tir(filePath);
sections = parsed.sections;
requireSections(sections, [ ...
    "MODEL", "DIMENSION", "LONG_SLIP_RANGE", ...
    "SLIP_ANGLE_RANGE", "INCLINATION_ANGLE_RANGE", ...
    "VERTICAL_FORCE_RANGE", "VERTICAL", ...
    "SCALING_COEFFICIENTS", "LONGITUDINAL_COEFFICIENTS", ...
    "LATERAL_COEFFICIENTS", "OVERTURNING_COEFFICIENTS", ...
    "ROLLING_COEFFICIENTS", "ALIGNING_COEFFICIENTS"]);

[mfevalParameters, mfevalMetadata] = loadMfevalParameters(filePath, parsed);
modelSide = normalizeModelSide(mfevalParameters.TYRESIDE);
if ~ismember(modelSide, ["LEFT", "RIGHT"])
    error("load_pac2002_tire:InvalidModelSide", ...
        "MODEL.TYRESIDE must be LEFT or RIGHT.");
end

model = parsed;
model.source.file_path = string(filePath);
model.source.mfeval_parameter_file = mfevalMetadata.parameter_file;
model.model_side = modelSide;
model.nominal_load_N = finiteScalar( ...
    mfevalParameters.FNOMIN * mfevalParameters.LFZO, ...
    "MFeval FNOMIN * LFZO");
model.unloaded_radius_m = finiteScalar( ...
    mfevalParameters.UNLOADED_RADIUS, "MFeval UNLOADED_RADIUS");
model.reference_speed_mps = finiteScalar( ...
    mfevalParameters.LONGVL, "MFeval LONGVL");
model.low_speed_threshold_mps = finiteScalar( ...
    mfevalParameters.VXLOW, "MFeval VXLOW");
model.ranges.Fz_N = numericPair(mfevalParameters.FZMIN, ...
    mfevalParameters.FZMAX, "vertical load");
model.ranges.kappa = numericPair(mfevalParameters.KPUMIN, ...
    mfevalParameters.KPUMAX, "longitudinal slip");
model.ranges.alpha_rad = numericPair(mfevalParameters.ALPMIN, ...
    mfevalParameters.ALPMAX, "slip angle");
model.ranges.gamma_rad = numericPair(mfevalParameters.CAMMIN, ...
    mfevalParameters.CAMMAX, "inclination angle");
model.evaluator_name = "MFeval";
model.mfeval.parameters = mfevalParameters;
model.mfeval.version = mfevalMetadata.version;
model.mfeval.root = mfevalMetadata.root;
model.mfeval.use_mode = mfevalMetadata.use_mode;
model.mfeval.evaluate = @mfeval;
evaluatorModel = model;
model.evaluate = @(input) evaluate_pac2002_tire(evaluatorModel, input);
end

function [parameters, metadata] = loadMfevalParameters(filePath, parsed)
installation = resolve_mfeval_installation();
[folder, stem] = fileparts(filePath);
parameterFile = string(fullfile(folder, stem + ".mfeval.mat"));
if isfile(parameterFile)
    saved = load(parameterFile, "mfeval_export");
    if ~isfield(saved, "mfeval_export") ...
            || ~isfield(saved.mfeval_export, "parameters") ...
            || ~isfield(saved.mfeval_export, "source") ...
            || ~isfield(saved.mfeval_export.source, "sha256")
        error("load_pac2002_tire:InvalidMfevalExport", ...
            "MFeval parameter export is invalid: %s", parameterFile);
    end
    if saved.mfeval_export.source.sha256 ~= parsed.source.sha256
        error("load_pac2002_tire:StaleMfevalExport", ...
            "MFeval parameter export does not match the TIR SHA-256: %s", ...
            parameterFile);
    end
    parameters = saved.mfeval_export.parameters;
else
    parameters = mfeval.readTIR(char(filePath));
    parameterFile = "<runtime MFeval import>";
end
if ~isfield(parameters, "FITTYP") || parameters.FITTYP ~= 6
    error("load_pac2002_tire:UnsupportedMfevalModel", ...
        "The local handling model requires the PAC2002/FITTYP 6 TIR.");
end
metadata.version = installation.version;
metadata.root = installation.root;
metadata.use_mode = 111;
metadata.parameter_file = parameterFile;
end

function side = normalizeModelSide(value)
side = upper(strtrim(string(value)));
side = erase(side, "'");
side = erase(side, '"');
end

function requireSections(sections, names)
for index = 1:numel(names)
    if ~isfield(sections, names(index))
        error("load_pac2002_tire:MissingSection", ...
            "Required TIR section [%s] is missing.", names(index));
    end
end
end

function value = finiteScalar(value, qualifiedName)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("load_pac2002_tire:InvalidField", ...
        "TIR field %s must be a finite numeric scalar.", qualifiedName);
end
end

function range = numericPair(minimum, maximum, label)
minimum = finiteScalar(minimum, "MFeval " + label + " minimum");
maximum = finiteScalar(maximum, "MFeval " + label + " maximum");
if minimum >= maximum
    error("load_pac2002_tire:InvalidRange", ...
        "The MFeval %s range must have minimum < maximum.", label);
end
range = [minimum, maximum];
end
