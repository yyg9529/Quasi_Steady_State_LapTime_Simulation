function state = make_vehicle_state(v_mps, ax_mps2, ay_mps2)
%MAKE_VEHICLE_STATE Construct a validated quasi-static vehicle state in SI.

arguments
    v_mps (1,1) double {mustBeFinite, mustBeNonnegative}
    ax_mps2 (1,1) double {mustBeFinite}
    ay_mps2 (1,1) double {mustBeFinite}
end

state.v_mps = v_mps;
state.ax_mps2 = ax_mps2;
state.ay_mps2 = ay_mps2;
end
