function value = get_nested_field(input, fieldPath)
%GET_NESTED_FIELD Read an existing dot-delimited struct field.

arguments
    input (1,1) struct
    fieldPath (1,1) string
end

parts = split(fieldPath, ".");
value = input;
for iPart = 1:numel(parts)
    name = char(parts(iPart));
    if ~isstruct(value) || ~isscalar(value) || ~isfield(value, name)
        error("QSSLTS:FieldPath", ...
            "Field path does not exist: %s", fieldPath);
    end
    value = value.(name);
end
end
