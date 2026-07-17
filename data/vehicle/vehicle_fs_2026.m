function vehicle = vehicle_fs_2026()
%VEHICLE_FS_2026 Return the Formula Student 2026 vehicle preset.
%   The wheelbase is user-confirmed. All other vehicle parameters remain
%   provisional baseline-derived values that have not yet been measured.

vehicle = vehicle_baseline();
vehicle.geometry.wheelbase_m = 1.560;

vehicle.provenance.season = 2026;
vehicle.provenance.wheelbase_source = "User-confirmed 1560 mm";
vehicle.provenance.inherited_parameters.fields = [ ...
    "mass.total_kg"
    "mass.front_static_frac"
    "mass.cg_height_m"
    "geometry.track_front_m"
    "geometry.track_rear_m"
    "inertia.Iz_kgm2"
    "load_transfer.front_lateral_distribution"
    "drivetrain.layout"];
vehicle.provenance.inherited_parameters.status = "provisional";
vehicle.provenance.inherited_parameters.source = "baseline-derived";
vehicle.provenance.inherited_parameters.measurement_status = ...
    "not yet measured";

vehicle.notes = "FS 2026 preset: wheelbase confirmed; " + ...
    "all other parameters are provisional baseline-derived values " + ...
    "not yet measured";
end
