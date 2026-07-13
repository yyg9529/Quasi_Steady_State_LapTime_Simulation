function options = default_qss_options(options)
%DEFAULT_QSS_OPTIONS Fill missing QSS options with deterministic defaults.
%   All speed, distance, acceleration and tolerance values use SI units.

if nargin == 0 || isempty(options)
    options = struct();
end

options = setDefault(options, "gravity_mps2", 9.80665);
options = setDefault(options, "v_max_mps", 45);
options = setDefault(options, "start_speed_mps", 0);
options = setDefault(options, "finish_speed_mps", NaN);
options = setDefault(options, "min_query_speed_mps", 0.5);
options = setDefault(options, "v_grid_mps", (0:0.5:45).');
options = setDefault(options, "ay_grid_g", -4.0:0.05:4.0);
options = setDefault(options, "ggv_max_iterations", 80);
options = setDefault(options, "ggv_tolerance_mps2", 1e-7);
options = setDefault(options, "ggv_relaxation", 1.0);
options = setDefault(options, "solver_max_iterations", 500);
options = setDefault(options, "solver_tolerance_mps", 1e-8);
options = setDefault(options, "limiter_tolerance_mps", 1e-5);
options = setDefault(options, "accel_tolerance_mps2", 1e-5);
options = setDefault(options, "speed_query_policy", "clamp");
end

function value = setDefault(value, name, defaultValue)
if ~isfield(value, name) || isempty(value.(name))
    value.(name) = defaultValue;
end
end
