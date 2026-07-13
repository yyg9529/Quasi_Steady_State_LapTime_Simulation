function slipForce = make_unitire_placeholder(variant)
%MAKE_UNITIRE_PLACEHOLDER Return an unconfigured UniTire adapter contract.
%   The placeholder never falls back to the simple tire model. Attach a
%   validated canonical evaluator before using it in the 7DOF model.

    arguments
        variant (1, 1) string {mustBeMember(variant, ["simple", "full"])} ...
            = "simple"
    end

    slipForce.model_type = "external_adapter";
    slipForce.velocity_regularization_mps = 1.0;
    slipForce.evaluator = [];
    slipForce.parameters = struct();
    slipForce.metadata.adapter_contract = "QSSLTS_TIRE_FORCE_V1";
    slipForce.metadata.source = "UNCONFIGURED";
    slipForce.metadata.evaluator_id = "UNCONFIGURED";
    slipForce.metadata.source_coordinate_convention = ...
        "SAE_J670_X_FORWARD_Y_RIGHT_Z_DOWN";
    slipForce.metadata.native_tire_side = "UNCONFIGURED";
    slipForce.metadata.extrapolation_policy = "error";
    slipForce.metadata.requires_pressure = false;

    if variant == "simple"
        slipForce.metadata.adapter_type = "unitire_simple";
        slipForce.metadata.model_schema = "unitire_simple_v1";
        slipForce.parameters.model_schema = "unitire_simple_v1";
        slipForce.metadata.supports_camber = false;
        slipForce.metadata.valid_domain = simpleReferenceBounds();
    else
        slipForce.metadata.adapter_type = "unitire_full";
        slipForce.metadata.model_schema = "unitire_full_v1";
        slipForce.parameters.model_schema = "unitire_full_v1";
        slipForce.metadata.supports_camber = true;
        slipForce.metadata.valid_domain = struct();
    end
end

function domain = simpleReferenceBounds()
    domain.slip_ratio = [-0.2176, 0.1817];
    domain.slip_angle_rad = [-0.2135, 0.2133];
    domain.Fz_N = [222, 1112];
    domain.camber_rad = [0, 0];
    referenceSpeed_mps = 40.193 / 3.6;
    domain.wheel_vx_mps = [referenceSpeed_mps, referenceSpeed_mps];
end
