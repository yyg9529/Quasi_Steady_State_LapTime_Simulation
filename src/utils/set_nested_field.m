function output = set_nested_field(input, fieldPath, value)
%SET_NESTED_FIELD Set an existing dot-delimited struct field.

arguments
    input (1,1) struct
    fieldPath (1,1) string
    value
end

parts = split(fieldPath, ".");
if any(strlength(parts) == 0)
    error("QSSLTS:FieldPath", "Invalid field path: %s", fieldPath);
end
output = setRecursive(input, parts, value, fieldPath);
end

function value = setRecursive(value, parts, newValue, fullPath)
name = char(parts(1));
if ~isfield(value, name)
    error("QSSLTS:FieldPath", ...
        "Field path does not exist: %s", fullPath);
end
if isscalar(parts)
    value.(name) = newValue;
else
    if ~isstruct(value.(name)) || ~isscalar(value.(name))
        error("QSSLTS:FieldPath", ...
            "Intermediate field is not a scalar struct: %s", name);
    end
    value.(name) = setRecursive(value.(name), parts(2:end), ...
        newValue, fullPath);
end
end
