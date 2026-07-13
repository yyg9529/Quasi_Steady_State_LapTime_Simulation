function brake = brake_baseline()
%BRAKE_BASELINE Return the baseline mechanical brake limits.

brake.enabled = true;
brake.max_decel_g_mechanical = 2.0;
brake.front_bias = 0.60;
brake.regen_enabled = false;
end
