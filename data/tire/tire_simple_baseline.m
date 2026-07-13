function tire = tire_simple_baseline()
%TIRE_SIMPLE_BASELINE Return a constant-mu tire model in SI units.

tire.model_type = "constant_mu";
tire.mu_x = 1.50;
tire.mu_y = 1.50;
tire.combined_n = 2;
tire.rolling_radius_m = 0.2286;
tire.wheel_inertia_kgm2 = 0.45;
tire.slip_force.model_type = "simple_saturated";
tire.slip_force.Fz_ref_N = 750;
tire.slip_force.longitudinal_stiffness_ref_N = 30000;
tire.slip_force.cornering_stiffness_ref_Nprad = 10000;
tire.slip_force.velocity_regularization_mps = 1.0;
tire.notes = "Concept-level constant friction coefficient model";
end
