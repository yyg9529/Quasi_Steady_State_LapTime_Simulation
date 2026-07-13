function validate_7dof_comparison_ggv( ...
        ggv, vehicle, tire, powertrain, brake)
%VALIDATE_7DOF_COMPARISON_GGV Require an aero-off matching model map.

    arguments
        ggv (1, 1) struct
        vehicle (1, 1) struct
        tire (1, 1) struct
        powertrain (1, 1) struct
        brake (1, 1) struct
    end

    if ~isfield(ggv, "aero_enabled") || ggv.aero_enabled
        error("QSSLTS:DOF7GGVAero", ...
            "V0.8 7DOF comparisons require a model GGV with aero disabled.");
    end
    if ~isfield(ggv, "gravity_mps2") ...
            || ~isfinite(ggv.gravity_mps2) ...
            || abs(ggv.gravity_mps2 - 9.80665) > 1e-12
        error("QSSLTS:DOF7GGVGravity", ...
            "7DOF comparison GGV must use gravity 9.80665 m/s^2.");
    end
    if ~isfield(ggv, "source") ...
            || ~startsWith(string(ggv.source), "model_")
        error("QSSLTS:DOF7GGVSource", ...
            "7DOF consistency checks require an uncalibrated model GGV.");
    end
    if ~isfield(ggv, "provenance")
        error("QSSLTS:DOF7GGVProvenance", ...
            "Comparison GGV does not contain model provenance.");
    end
    provenance = ggv.provenance;
    required = ["vehicle", "tire", "powertrain", "brake"];
    if ~all(isfield(provenance, cellstr(required)))
        error("QSSLTS:DOF7GGVProvenance", ...
            "Comparison GGV model provenance is incomplete.");
    end
    matching = isequaln(provenance.vehicle, vehicle) ...
        && isequaln(provenance.tire, tire_envelope_provenance(tire)) ...
        && isequaln(provenance.powertrain, powertrain) ...
        && isequaln(provenance.brake, brake);
    if ~matching
        error("QSSLTS:DOF7GGVModelMismatch", ...
            "7DOF and GGV vehicle/tire/powertrain/brake inputs must match.");
    end
end
