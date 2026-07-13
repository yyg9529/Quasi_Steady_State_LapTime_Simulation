function brake = brake_baseline()
%BRAKE_BASELINE Return the baseline mechanical brake limits.

brake.enabled = true;
brake.max_decel_g_mechanical = 2.0;
brake.max_total_brake_force_N = inf;
brake.max_total_brake_torque_Nm = 1400;
brake.front_bias = 0.60;
brake.regen_enabled = false;
end
