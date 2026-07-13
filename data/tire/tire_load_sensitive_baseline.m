function tire = tire_load_sensitive_baseline()
%TIRE_LOAD_SENSITIVE_BASELINE Return a concept load-sensitive tire model.

tire.model_type = "load_sensitive";
tire.Fz_ref_N = 750;
tire.mu_x_ref = 1.60;
tire.mu_y_ref = 1.60;
tire.load_sensitivity_x = -0.08;
tire.load_sensitivity_y = -0.08;
tire.combined_n = 2;
tire.rolling_radius_m = 0.2286;
tire.wheel_inertia_kgm2 = 0.45;
tire.slip_force.model_type = "simple_saturated";
tire.slip_force.Fz_ref_N = tire.Fz_ref_N;
tire.slip_force.longitudinal_stiffness_ref_N = 30000;
tire.slip_force.cornering_stiffness_ref_Nprad = 10000;
tire.slip_force.velocity_regularization_mps = 1.0;
tire.notes = "Concept load sensitivity, not fitted tire-test data";
end
