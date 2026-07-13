function staticLoads = calc_static_loads(vehicle, gravity_mps2)
%CALC_STATIC_LOADS Calculate static axle and wheel normal loads [N].
%   Positive Fz denotes compressive normal-load magnitude.

arguments
    vehicle (1,1) struct
    gravity_mps2 (1,1) double {mustBePositive} = 9.80665
end

mass_kg = vehicle.mass.total_kg;
frontFraction = vehicle.mass.front_static_frac;
if mass_kg <= 0 || frontFraction < 0 || frontFraction > 1
    error("QSSLTS:StaticLoadParameters", ...
        "Mass must be positive and front_static_frac must be in [0,1].");
end

totalLoad_N = mass_kg * gravity_mps2;
staticLoads.Fz_front_total_N = frontFraction * totalLoad_N;
staticLoads.Fz_rear_total_N = (1 - frontFraction) * totalLoad_N;
staticLoads.Fz_FL_N = 0.5 * staticLoads.Fz_front_total_N;
staticLoads.Fz_FR_N = 0.5 * staticLoads.Fz_front_total_N;
staticLoads.Fz_RL_N = 0.5 * staticLoads.Fz_rear_total_N;
staticLoads.Fz_RR_N = 0.5 * staticLoads.Fz_rear_total_N;
staticLoads.Fz_total_N = totalLoad_N;
end
