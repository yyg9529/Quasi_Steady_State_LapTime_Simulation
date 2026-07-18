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

modelSide = upper(string(requireValue(sections.MODEL, "TYRESIDE")));
if ~ismember(modelSide, ["LEFT", "RIGHT"])
    error("load_pac2002_tire:InvalidModelSide", ...
        "MODEL.TYRESIDE must be LEFT or RIGHT.");
end

model = parsed;
model.source.file_path = string(filePath);
model.model_side = modelSide;
model.nominal_load_N = finiteScalar( ...
    sections.VERTICAL.FNOMIN * sections.SCALING_COEFFICIENTS.LFZO, ...
    "VERTICAL.FNOMIN * SCALING_COEFFICIENTS.LFZO");
model.unloaded_radius_m = finiteScalar( ...
    requireValue(sections.DIMENSION, "UNLOADED_RADIUS"), ...
    "DIMENSION.UNLOADED_RADIUS");
model.reference_speed_mps = finiteScalar( ...
    requireValue(sections.MODEL, "LONGVL"), "MODEL.LONGVL");
model.low_speed_threshold_mps = finiteScalar( ...
    requireValue(sections.MODEL, "VXLOW"), "MODEL.VXLOW");
model.ranges.Fz_N = numericRange(sections.VERTICAL_FORCE_RANGE, ...
    "FZMIN", "FZMAX", "vertical load");
model.ranges.kappa = numericRange(sections.LONG_SLIP_RANGE, ...
    "KPUMIN", "KPUMAX", "longitudinal slip");
model.ranges.alpha_rad = numericRange(sections.SLIP_ANGLE_RANGE, ...
    "ALPMIN", "ALPMAX", "slip angle");
model.ranges.gamma_rad = numericRange( ...
    sections.INCLINATION_ANGLE_RANGE, "CAMMIN", "CAMMAX", ...
    "inclination angle");
evaluatorModel = model;
model.evaluate = @(input) evaluate_pac2002_tire(evaluatorModel, input);
end

function requireSections(sections, names)
for index = 1:numel(names)
    if ~isfield(sections, names(index))
        error("load_pac2002_tire:MissingSection", ...
            "Required TIR section [%s] is missing.", names(index));
    end
end
end

function value = requireValue(section, name)
if ~isfield(section, name)
    error("load_pac2002_tire:MissingField", ...
        "Required TIR field %s is missing.", name);
end
value = section.(name);
end

function value = finiteScalar(value, qualifiedName)
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("load_pac2002_tire:InvalidField", ...
        "TIR field %s must be a finite numeric scalar.", qualifiedName);
end
end

function range = numericRange(section, minimumName, maximumName, label)
minimum = finiteScalar(requireValue(section, minimumName), minimumName);
maximum = finiteScalar(requireValue(section, maximumName), maximumName);
if minimum >= maximum
    error("load_pac2002_tire:InvalidRange", ...
        "The PAC2002 %s range must have minimum < maximum.", label);
end
range = [minimum, maximum];
end
