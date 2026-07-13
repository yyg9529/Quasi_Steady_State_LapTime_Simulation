function transfer = calc_load_transfer(state, vehicle)
%CALC_LOAD_TRANSFER Calculate quasi-static load-transfer increments [N].
%   Positive ax transfers load from front to rear. Positive ay is a left
%   turn and transfers load from left wheels to right wheels. Lateral
%   values are per-side transfer amounts, not outer-minus-inner difference.

arguments
    state (1,1) struct
    vehicle (1,1) struct
end

mass_kg = vehicle.mass.total_kg;
height_m = vehicle.mass.cg_height_m;
wheelbase_m = vehicle.geometry.wheelbase_m;
trackFront_m = vehicle.geometry.track_front_m;
trackRear_m = vehicle.geometry.track_rear_m;
frontDistribution = vehicle.load_transfer.front_lateral_distribution;

if any([mass_kg, wheelbase_m, trackFront_m, trackRear_m] <= 0) ...
        || height_m < 0 || frontDistribution < 0 || frontDistribution > 1
    error("QSSLTS:LoadTransferParameters", ...
        "Vehicle geometry/load-transfer parameters are invalid.");
end

transfer.longitudinal_N = mass_kg * state.ax_mps2 * height_m / wheelbase_m;
transfer.front_lateral_per_side_N = frontDistribution ...
    * mass_kg * state.ay_mps2 * height_m / trackFront_m;
transfer.rear_lateral_per_side_N = (1 - frontDistribution) ...
    * mass_kg * state.ay_mps2 * height_m / trackRear_m;
end
