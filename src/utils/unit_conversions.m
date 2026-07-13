function units = unit_conversions()
%UNIT_CONVERSIONS Return commonly used deterministic conversion factors.

units.g_mps2 = 9.80665;
units.deg_to_rad = pi / 180;
units.rad_to_deg = 180 / pi;
units.mm_to_m = 1e-3;
units.kph_to_mps = 1 / 3.6;
end
