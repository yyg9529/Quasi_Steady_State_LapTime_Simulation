%% V0.9 advanced-tire adapter contract demonstration
projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(projectRoot);
project_setup();

tire = tire_load_sensitive_baseline();
tire.slip_force = make_unitire_placeholder("simple");
operatingPoint.wheel_vx_mps = 40.193 / 3.6;
operatingPoint.wheel_omega_radps = 48.8;
operatingPoint.wheel_side = "LEFT";

try
    tire_force_from_slip(0.02, 0.03, 660, 0, tire, operatingPoint);
    error("QSSLTS:Example", "Unconfigured placeholder did not fail.");
catch exception
    assert(exception.identifier == "QSSLTS:TireAdapterUnavailable");
    fprintf("Unconfigured UniTire placeholder: %s\n", exception.identifier);
end

% Synthetic evaluator used only to demonstrate the contract. It is not a
% fitted UniTire model and must not be used as physical tire data.
tire.slip_force.metadata.source = "synthetic contract example";
tire.slip_force.metadata.evaluator_id = "example_combined_input_v1";
tire.slip_force.metadata.native_tire_side = "SYMMETRIC";
tire.slip_force.evaluator = @syntheticCanonicalEvaluator;

force = tire_force_from_slip( ...
    0.02, 0.03, 660, 0, tire, operatingPoint);
fprintf("Canonical combined input: Fx %.1f N, Fy %.1f N, extrapolated %d\n", ...
    force.Fx_N, force.Fy_N, force.is_extrapolated);

magicFormula = make_magic_formula_placeholder();
fprintf("Magic Formula placeholder: %s / %s (pressure required: %d)\n", ...
    magicFormula.metadata.adapter_type, ...
    magicFormula.metadata.model_schema, ...
    magicFormula.metadata.requires_pressure);

function force = syntheticCanonicalEvaluator(request, ~)
    force.Fx_N = 1000 * request.kappa;
    force.Fy_N = -2000 * request.alpha_rad;
end
