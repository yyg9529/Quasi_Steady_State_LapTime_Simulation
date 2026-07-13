function force_N = calc_wheel_force_from_power( ...
        wheelPower_W, v_mps, minSpeed_mps)
%CALC_WHEEL_FORCE_FROM_POWER Convert wheel power [W] to force [N].
%   minSpeed_mps regularizes the v=0 division; the torque limit is expected
%   to govern in that region.

arguments
    wheelPower_W double {mustBeNonnegative}
    v_mps double {mustBeFinite, mustBeNonnegative}
    minSpeed_mps double {mustBeFinite, mustBePositive} = 0.5
end

force_N = wheelPower_W ./ max(v_mps, minSpeed_mps);
end
