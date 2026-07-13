function aeroForce = calc_aero_forces(v_mps, aero)
%CALC_AERO_FORCES Calculate drag/downforce magnitudes [N] at speed [m/s].
%   CDA_m2 and CLA_m2 are coefficient-area products in m^2.

arguments
    v_mps double {mustBeFinite, mustBeNonnegative}
    aero (1,1) struct
end

if ~isfield(aero, "enabled") || ~aero.enabled
    zero = zeros(size(v_mps));
    aeroForce.drag_N = zero;
    aeroForce.downforce_total_N = zero;
    aeroForce.downforce_front_N = zero;
    aeroForce.downforce_rear_N = zero;
    return
end

required = ["rho_kgpm3", "CDA_m2", "CLA_m2", "front_downforce_frac"];
if ~all(isfield(aero, cellstr(required)))
    error("QSSLTS:AeroParameters", "Aero model is missing required fields.");
end
if aero.rho_kgpm3 <= 0 || aero.CDA_m2 < 0 || aero.CLA_m2 < 0 ...
        || aero.front_downforce_frac < 0 || aero.front_downforce_frac > 1
    error("QSSLTS:AeroParameters", "Aero parameters are outside valid bounds.");
end

dynamicPressure_Pa = 0.5 * aero.rho_kgpm3 .* v_mps.^2;
aeroForce.drag_N = dynamicPressure_Pa .* aero.CDA_m2;
aeroForce.downforce_total_N = dynamicPressure_Pa .* aero.CLA_m2;
aeroForce.downforce_front_N = aero.front_downforce_frac ...
    .* aeroForce.downforce_total_N;
aeroForce.downforce_rear_N = (1 - aero.front_downforce_frac) ...
    .* aeroForce.downforce_total_N;
end
