function powertrain = powertrain_baseline()
%POWERTRAIN_BASELINE Return the baseline wheel-side powertrain limits.

powertrain.enabled = true;
powertrain.layout = "RWD";
powertrain.max_power_W = 80000;
powertrain.max_wheel_torque_Nm = 1000;
powertrain.max_speed_mps = 45;
powertrain.drive_efficiency = 0.90;
end
