function parsed = read_pac2002_tir(filePath)
%READ_PAC2002_TIR Parse scalar PAC2002 TIR entries for offline reduction.

arguments
    filePath (1, 1) string
end

if ~isfile(filePath)
    error("read_pac2002_tir:FileNotFound", ...
        "PAC2002 TIR file does not exist: %s", filePath);
end

sourceText = string(fileread(filePath));
lines = splitlines(sourceText);
sections = struct();
currentSection = "";

for lineIndex = 1:numel(lines)
    line = stripInlineComment(strtrim(lines(lineIndex)));
    if strlength(line) == 0
        continue
    end

    sectionToken = regexp(line, "^\[([^\]]+)\]$", "tokens", "once");
    if ~isempty(sectionToken)
        currentSection = string(matlab.lang.makeValidName( ...
            upper(strtrim(sectionToken{1}))));
        if ~isfield(sections, currentSection)
            sections.(currentSection) = struct();
        end
        continue
    end

    entryToken = regexp(line, ...
        "^([A-Za-z][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$", ...
        "tokens", "once");
    if isempty(entryToken) || strlength(currentSection) == 0
        continue
    end

    key = string(matlab.lang.makeValidName(upper(entryToken{1})));
    sections.(currentSection).(key) = parseScalarValue(entryToken{2});
end

validateUnits(sections);
model = requireSection(sections, "MODEL");
format = requireField(model, "PROPERTY_FILE_FORMAT", ...
    "MODEL.PROPERTY_FILE_FORMAT");
if ~isTextValue(format) || ~strcmpi(strtrim(string(format)), "PAC2002")
    error("read_pac2002_tir:InvalidPropertyFileFormat", ...
        "MODEL.PROPERTY_FILE_FORMAT must be PAC2002.");
end

requireNumericField(sections, "VERTICAL", "FNOMIN");
requireNumericField(sections, "SCALING_COEFFICIENTS", "LFZO");
requireNumericField(sections, "SCALING_COEFFICIENTS", "LMUX");
requireNumericField(sections, "SCALING_COEFFICIENTS", "LMUY");
requireNumericField(sections, "LONGITUDINAL_COEFFICIENTS", "PDX1");
requireNumericField(sections, "LONGITUDINAL_COEFFICIENTS", "PDX2");
requireNumericField(sections, "LATERAL_COEFFICIENTS", "PDY1");
requireNumericField(sections, "LATERAL_COEFFICIENTS", "PDY2");

[~, sourceName, sourceExtension] = fileparts(filePath);
parsed.sections = sections;
parsed.source.file_name = string(sourceName) + string(sourceExtension);
parsed.source.sha256 = fileSha256(filePath);
end

function line = stripInlineComment(line)
commentIndex = strfind(line, "$", ForceCellOutput=false);
if ~isempty(commentIndex)
    line = extractBefore(line, commentIndex(1));
end
line = strtrim(line);
end

function value = parseScalarValue(valueText)
valueText = strtrim(string(valueText));
if strlength(valueText) >= 2 && ...
        ((startsWith(valueText, "'") && endsWith(valueText, "'")) || ...
        (startsWith(valueText, '"') && endsWith(valueText, '"')))
    value = extractBetween(valueText, 2, strlength(valueText) - 1);
    return
end

numericValue = str2double(valueText);
if ~isnan(numericValue)
    value = numericValue;
else
    value = valueText;
end
end

function validateUnits(sections)
units = requireSection(sections, "UNITS");
expected = {
    "LENGTH", "meter"
    "FORCE", "newton"
    "ANGLE", "radians"
    "MASS", "kg"
    "TIME", "second"
};

for unitIndex = 1:size(expected, 1)
    name = expected{unitIndex, 1};
    value = requireField(units, name, "UNITS." + name);
    if ~isTextValue(value) || ...
            ~strcmpi(strtrim(string(value)), expected{unitIndex, 2})
        error("read_pac2002_tir:InvalidUnits", ...
            "UNITS.%s must be '%s'.", name, expected{unitIndex, 2});
    end
end
end

function section = requireSection(sections, name)
if ~isfield(sections, name)
    error("read_pac2002_tir:MissingSection", ...
        "Required TIR section [%s] is missing.", name);
end
section = sections.(name);
end

function value = requireField(section, name, qualifiedName)
if ~isfield(section, name)
    error("read_pac2002_tir:MissingField", ...
        "Required TIR field %s is missing.", qualifiedName);
end
value = section.(name);
end

function value = requireNumericField(sections, sectionName, fieldName)
section = requireSection(sections, sectionName);
qualifiedName = sectionName + "." + fieldName;
value = requireField(section, fieldName, qualifiedName);
if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value)
    error("read_pac2002_tir:InvalidField", ...
        "Required TIR field %s must be a finite numeric scalar.", ...
        qualifiedName);
end
end

function tf = isTextValue(value)
tf = (isstring(value) && isscalar(value)) || ...
    (ischar(value) && isrow(value));
end

function hash = fileSha256(filePath)
fileId = fopen(filePath, "rb");
if fileId < 0
    error("read_pac2002_tir:FileReadFailed", ...
        "Could not open TIR file for hashing: %s", filePath);
end
cleanup = onCleanup(@() fclose(fileId));
bytes = fread(fileId, Inf, "*uint8");

digest = java.security.MessageDigest.getInstance("SHA-256");
digest.update(bytes);
digestBytes = typecast(digest.digest(), "uint8");
hash = upper(string(reshape(dec2hex(digestBytes, 2).', 1, [])));
end
