function tire = tire_simple_baseline()
%TIRE_SIMPLE_BASELINE Return a constant-mu tire model in SI units.

tire.model_type = "constant_mu";
tire.mu_x = 1.50;
tire.mu_y = 1.50;
tire.combined_n = 2;
tire.rolling_radius_m = 0.2286;
tire.notes = "Concept-level constant friction coefficient model";
end
