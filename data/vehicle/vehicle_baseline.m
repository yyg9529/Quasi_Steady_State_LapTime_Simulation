function vehicle = vehicle_baseline()
%VEHICLE_BASELINE Return the baseline FSAE vehicle parameters in SI units.

vehicle.mass.total_kg = 300;
vehicle.mass.front_static_frac = 0.40;
vehicle.mass.cg_height_m = 0.250;

vehicle.geometry.wheelbase_m = 1.668;
vehicle.geometry.track_front_m = 1.200;
vehicle.geometry.track_rear_m = 1.200;

vehicle.inertia.Iz_kgm2 = 120;
vehicle.load_transfer.front_lateral_distribution = 0.50;

vehicle.drivetrain.layout = "RWD";
vehicle.notes = "Baseline FSAE vehicle";
end
