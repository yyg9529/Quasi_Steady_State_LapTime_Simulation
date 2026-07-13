function fig = plot_ggv_slice(ggv, vQuery_mps)
%PLOT_GGV_SLICE Plot longitudinal-vs-lateral acceleration at one speed.
%   Both plotted axes use g; vQuery_mps uses m/s.

arguments
    ggv (1,1) struct
    vQuery_mps (1,1) double {mustBeFinite, mustBeNonnegative}
end

vQuery_mps = clamp(vQuery_mps, ggv.v_mps(1), ggv.v_mps(end));
axMax_g = interp1(ggv.v_mps, ggv.ax_max_g, vQuery_mps, "linear");
axMin_g = interp1(ggv.v_mps, ggv.ax_min_g, vQuery_mps, "linear");
positiveLimit_g = interp1(ggv.v_mps, ggv.ay_limit_pos_g, ...
    vQuery_mps, "linear");
negativeLimit_g = interp1(ggv.v_mps, ggv.ay_limit_neg_g, ...
    vQuery_mps, "linear");
feasible = ggv.ay_g <= positiveLimit_g & ggv.ay_g >= negativeLimit_g;

fig = figure(Name=sprintf("GGV slice at %.1f m/s", vQuery_mps));
plot(ggv.ay_g(feasible), axMax_g(feasible), LineWidth=1.5);
hold on
plot(ggv.ay_g(feasible), axMin_g(feasible), LineWidth=1.5);
grid on; axis equal
xlabel("a_y [g]"); ylabel("a_x [g]");
legend("Acceleration", "Braking", Location="best");
end
