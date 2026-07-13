function aero = aero_baseline()
%AERO_BASELINE Return the baseline aerodynamic model in SI units.

aero.enabled = true;
aero.rho_kgpm3 = 1.225;
aero.CLA_m2 = 3.0;
aero.CDA_m2 = 1.2;
aero.front_downforce_frac = 0.50;
end
