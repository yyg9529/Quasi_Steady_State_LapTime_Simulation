function slipForce = make_magic_formula_placeholder()
%MAKE_MAGIC_FORMULA_PLACEHOLDER Return an unconfigured PAC2002 contract.
%   No reference .tir file or third-party evaluator is activated here.

    slipForce.model_type = "external_adapter";
    slipForce.velocity_regularization_mps = 1.0;
    slipForce.evaluator = [];
    slipForce.parameters = struct();
    slipForce.metadata.adapter_contract = "QSSLTS_TIRE_FORCE_V1";
    slipForce.metadata.adapter_type = "magic_formula_pac2002";
    slipForce.metadata.model_schema = "pac2002_v1";
    slipForce.parameters.model_schema = "pac2002_v1";
    slipForce.metadata.source = "UNCONFIGURED";
    slipForce.metadata.evaluator_id = "UNCONFIGURED";
    slipForce.metadata.source_coordinate_convention = "UNCONFIGURED";
    slipForce.metadata.native_tire_side = "UNCONFIGURED";
    slipForce.metadata.valid_domain = struct();
    slipForce.metadata.extrapolation_policy = "error";
    slipForce.metadata.supports_camber = true;
    slipForce.metadata.requires_pressure = true;
end
