function force = tire_force_external_adapter( ...
        slipRatio, slipAngle_rad, Fz_N, camber_rad, tire, operatingPoint)
%TIRE_FORCE_EXTERNAL_ADAPTER Evaluate a canonical steady algebraic adapter.
%   The evaluator is called independently for each loaded wheel. It must be
%   deterministic and stateless because the 7DOF load fixed point may call
%   it repeatedly at the same simulation time.

    targetSize = size(slipRatio);
    slipAngle_rad = expandNumeric(slipAngle_rad, targetSize, "slip angle");
    Fz_N = expandNumeric(Fz_N, targetSize, "normal load");
    camber_rad = expandNumeric(camber_rad, targetSize, "camber");
    force = zeroForce(targetSize);
    loaded = Fz_N > 0;
    if ~any(loaded, "all")
        return
    end

    [evaluator, parameters, metadata] = validateConfiguration(tire);
    wheelVx_mps = operatingNumeric( ...
        operatingPoint, "wheel_vx_mps", targetSize);
    wheelOmega_radps = operatingNumeric( ...
        operatingPoint, "wheel_omega_radps", targetSize);
    wheelSide = operatingSide(operatingPoint, targetSize);
    pressure_Pa = operatingNumeric( ...
        operatingPoint, "pressure_Pa", targetSize, NaN);

    flatLoaded = find(loaded(:)).';
    for index = flatLoaded
        request = struct();
        request.kappa = slipRatio(index);
        request.alpha_rad = slipAngle_rad(index);
        request.Fz_N = Fz_N(index);
        request.camber_rad = camber_rad(index);
        request.wheel_vx_mps = wheelVx_mps(index);
        request.wheel_omega_radps = wheelOmega_radps(index);
        request.wheel_side = wheelSide(index);
        if isfinite(pressure_Pa(index))
            request.pressure_Pa = pressure_Pa(index);
        end

        validateCapabilities(request, metadata);
        outsideDomain = isOutsideDomain(request, metadata);
        if outsideDomain && metadata.extrapolation_policy == "error"
            error("QSSLTS:TireDomain", ...
                "External tire request is outside the declared valid domain.");
        end
        evaluated = evaluator(request, parameters);
        validateCanonicalOutput(evaluated);
        force.Fx_N(index) = evaluated.Fx_N;
        force.Fy_N(index) = evaluated.Fy_N;
        force.is_extrapolated(index) = outsideDomain;
    end
    force.evaluator_id = metadata.evaluator_id;
    force.model_schema = metadata.model_schema;
end

function force = zeroForce(targetSize)
    force.Fx_N = zeros(targetSize);
    force.Fy_N = zeros(targetSize);
    force.is_extrapolated = false(targetSize);
    force.evaluator_id = "";
    force.model_schema = "";
end

function [evaluator, parameters, metadata] = validateConfiguration(tire)
    if ~isfield(tire, "slip_force") ...
            || ~isfield(tire.slip_force, "metadata") ...
            || ~isstruct(tire.slip_force.metadata)
        error("QSSLTS:TireParameterSchema", ...
            "External tire adapter metadata is required.");
    end
    metadata = tire.slip_force.metadata;
    required = ["adapter_contract", "adapter_type", "model_schema", "source", ...
        "evaluator_id", "source_coordinate_convention", ...
        "native_tire_side", "valid_domain", "extrapolation_policy", ...
        "supports_camber", "requires_pressure"];
    if ~all(isfield(metadata, cellstr(required)))
        error("QSSLTS:TireParameterSchema", ...
            "External tire adapter metadata is incomplete.");
    end
    adapterContract = string(metadata.adapter_contract);
    adapterType = string(metadata.adapter_type);
    modelSchema = string(metadata.model_schema);
    validIdentifiers = isscalar(adapterType) && isscalar(modelSchema);
    validTypeAndSchema = false;
    if validIdentifiers
        validTypeAndSchema = (adapterType == "unitire_simple" ...
            && modelSchema == "unitire_simple_v1") ...
            || (adapterType == "unitire_full" ...
            && modelSchema == "unitire_full_v1") ...
            || (adapterType == "magic_formula_pac2002" ...
            && modelSchema == "pac2002_v1");
    end
    validSchema = isscalar(adapterContract) ...
        && adapterContract == "QSSLTS_TIRE_FORCE_V1" ...
        && validIdentifiers && validTypeAndSchema;
    validFlags = isLogicalScalar(metadata.supports_camber) ...
        && isLogicalScalar(metadata.requires_pressure);
    if ~validSchema || ~validFlags ...
            || (modelSchema == "unitire_simple_v1" ...
            && logical(metadata.supports_camber)) ...
            || (modelSchema == "pac2002_v1" ...
            && ~logical(metadata.requires_pressure))
        error("QSSLTS:TireParameterSchema", ...
            "Adapter contract or model parameter schema is incompatible.");
    end
    if ~isfield(tire.slip_force, "evaluator") ...
            || ~isa(tire.slip_force.evaluator, "function_handle")
        error("QSSLTS:TireAdapterUnavailable", ...
            "The external tire evaluator is not configured.");
    end
    evaluator = tire.slip_force.evaluator;
    if isfield(tire.slip_force, "parameters")
        parameters = tire.slip_force.parameters;
    else
        parameters = struct();
    end
    if ~isstruct(parameters) || ~isfield(parameters, "model_schema") ...
            || ~isscalar(string(parameters.model_schema)) ...
            || string(parameters.model_schema) ~= modelSchema
        error("QSSLTS:TireParameterSchema", ...
            "External tire parameter bundle schema does not match metadata.");
    end
    metadata.source = requiredConfiguredString(metadata.source, "source");
    metadata.evaluator_id = requiredConfiguredString( ...
        metadata.evaluator_id, "evaluator_id");
    metadata.source_coordinate_convention = validateCoordinateConvention( ...
        metadata.source_coordinate_convention);
    metadata.native_tire_side = validateNativeSide(metadata.native_tire_side);
    metadata.extrapolation_policy = lower(string( ...
        metadata.extrapolation_policy));
    if ~isscalar(metadata.extrapolation_policy) ...
            || ~ismember(metadata.extrapolation_policy, ["error", "allow"]) ...
            || ~isstruct(metadata.valid_domain)
        error("QSSLTS:TireParameterSchema", ...
            "External tire adapter flags/domain metadata are invalid.");
    end
    metadata.supports_camber = logical(metadata.supports_camber);
    metadata.requires_pressure = logical(metadata.requires_pressure);
end

function value = requiredConfiguredString(value, fieldName)
    value = string(value);
    if ~isscalar(value) || strlength(value) == 0 ...
            || upper(value) == "UNCONFIGURED"
        error("QSSLTS:TireParameterSchema", ...
            "External tire adapter %s is not configured.", fieldName);
    end
end

function convention = validateCoordinateConvention(convention)
    convention = upper(string(convention));
    supported = ["QSSLTS_X_FORWARD_Y_LEFT_Z_UP", ...
        "SAE_J670_X_FORWARD_Y_RIGHT_Z_DOWN"];
    if ~isscalar(convention) || ~ismember(convention, supported)
        error("QSSLTS:TireParameterSchema", ...
            "External tire source coordinate convention is unsupported.");
    end
end

function nativeSide = validateNativeSide(nativeSide)
    nativeSide = upper(string(nativeSide));
    if ~isscalar(nativeSide) ...
            || ~ismember(nativeSide, ["LEFT", "RIGHT", "SYMMETRIC"])
        error("QSSLTS:TireParameterSchema", ...
            "External tire native side must be LEFT, RIGHT, or SYMMETRIC.");
    end
end

function valid = isLogicalScalar(value)
    valid = (islogical(value) || isnumeric(value)) && isscalar(value) ...
        && isreal(value) && isfinite(value) && (value == 0 || value == 1);
end

function validateCapabilities(request, metadata)
    if ~isfinite(request.wheel_vx_mps) ...
            || ~isfinite(request.wheel_omega_radps)
        error("QSSLTS:TireDomain", ...
            "Wheel longitudinal speed and angular speed are required.");
    end
    side = upper(string(request.wheel_side));
    if ~isscalar(side) || ~ismember(side, ["LEFT", "RIGHT"])
        error("QSSLTS:TireSideUnsupported", ...
            "Each external tire request requires LEFT or RIGHT wheel_side.");
    end
    if metadata.native_tire_side ~= "SYMMETRIC" ...
            && metadata.native_tire_side ~= side
        error("QSSLTS:TireSideUnsupported", ...
            "Native tire side does not support the requested wheel side.");
    end
    if ~metadata.supports_camber && abs(request.camber_rad) > 1e-12
        error("QSSLTS:TireCamberUnsupported", ...
            "The configured external tire model does not support camber.");
    end
    if metadata.requires_pressure ...
            && (~isfield(request, "pressure_Pa") ...
            || ~isfinite(request.pressure_Pa) || request.pressure_Pa <= 0)
        error("QSSLTS:TirePressureRequired", ...
            "The configured external tire model requires pressure_Pa.");
    end
end

function outside = isOutsideDomain(request, metadata)
    fields = ["kappa", "alpha_rad", "Fz_N", "camber_rad", ...
        "wheel_vx_mps"];
    domainFields = ["slip_ratio", "slip_angle_rad", "Fz_N", ...
        "camber_rad", "wheel_vx_mps"];
    if metadata.requires_pressure
        fields(end + 1) = "pressure_Pa";
        domainFields(end + 1) = "pressure_Pa";
    end
    outside = false;
    for index = 1:numel(fields)
        domainField = domainFields(index);
        if ~isfield(metadata.valid_domain, domainField)
            error("QSSLTS:TireParameterSchema", ...
                "External tire valid_domain is incomplete.");
        end
        limits = metadata.valid_domain.(domainField);
        if ~isnumeric(limits) || ~isequal(size(limits), [1, 2]) ...
                || ~all(isfinite(limits)) || ~isreal(limits) ...
                || limits(1) > limits(2)
            error("QSSLTS:TireParameterSchema", ...
                "External tire valid_domain limits are invalid.");
        end
        value = request.(fields(index));
        outside = outside || value < limits(1) || value > limits(2);
    end
end

function validateCanonicalOutput(force)
    outputFields = strings(0, 1);
    if isstruct(force)
        outputFields = string(fieldnames(force));
    end
    validFields = numel(outputFields) == 2 ...
        && all(ismember(outputFields, ["Fx_N", "Fy_N"]));
    valid = isstruct(force) && validFields ...
        && isnumeric(force.Fx_N) && isnumeric(force.Fy_N) ...
        && isscalar(force.Fx_N) && isscalar(force.Fy_N) ...
        && isreal(force.Fx_N) && isreal(force.Fy_N) ...
        && isfinite(force.Fx_N) && isfinite(force.Fy_N);
    if ~valid
        error("QSSLTS:TireAdapterOutput", ...
            "Canonical evaluator must return only finite scalar Fx_N/Fy_N.");
    end
end

function value = operatingNumeric(operatingPoint, fieldName, targetSize, default)
    if nargin < 4
        default = NaN;
    end
    if isfield(operatingPoint, fieldName)
        raw = operatingPoint.(fieldName);
    else
        raw = default;
    end
    value = expandNumeric(raw, targetSize, fieldName);
end

function side = operatingSide(operatingPoint, targetSize)
    if isfield(operatingPoint, "wheel_side")
        raw = string(operatingPoint.wheel_side);
    else
        raw = "UNSPECIFIED";
    end
    if isscalar(raw)
        side = repmat(raw, targetSize);
    elseif isequal(size(raw), targetSize)
        side = raw;
    else
        error("QSSLTS:TireForceSize", ...
            "wheel_side must be scalar or match the slip input size.");
    end
end

function value = expandNumeric(value, targetSize, fieldName)
    if ~isnumeric(value) || ~isreal(value)
        error("QSSLTS:TireForceSize", ...
            "%s must be real numeric data.", fieldName);
    end
    if isscalar(value)
        value = repmat(value, targetSize);
    elseif ~isequal(size(value), targetSize)
        error("QSSLTS:TireForceSize", ...
            "%s must be scalar or match the slip input size.", fieldName);
    end
end
