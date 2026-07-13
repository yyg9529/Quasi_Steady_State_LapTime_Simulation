function x0 = make_7dof_initial_state(speed_mps, tire)
%MAKE_7DOF_INITIAL_STATE Create a straight pure-rolling initial state.

    arguments
        speed_mps (1, 1) double {mustBeFinite, mustBeNonnegative}
        tire (1, 1) struct
    end

    if ~isfield(tire, "rolling_radius_m") || tire.rolling_radius_m <= 0
        error("QSSLTS:DOF7Tire", "Tire rolling radius must be positive.");
    end
    wheelSpeed_radps = speed_mps / tire.rolling_radius_m;
    x0 = [speed_mps; 0; 0; repmat(wheelSpeed_radps, 4, 1)];
end
